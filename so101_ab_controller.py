# so101_ab_controller.py
#
# SO-101 follower controller driven by ArtificialBrains (AB) Python SDK.
#
# What this does (minimal + safe):
# - Loads:
#     targets/safety_cage.json        (per-joint RAW tick min/max + buffer)
#     targets/target_touch_mat.json   (a single target pose in RAW ticks)
# - Connects to your SO-101 follower arm via LeRobot (Feetech bus).
# - Starts an AB run (ABClient), listens for:
#     - io:need      -> sends robot:state + requested *inputs* (Image, etc.)
#     - cycle:update -> decodes output spikes into tiny per-joint deltas
# - Applies motion safely:
#     - per-step rate limit (tiny)
#     - clamp to cage (min+buffer .. max-buffer)
# - Builds:
#     - objective scalar per timestep: joint_error_to_target (lower is better)
#     - deviation feedback: based on error improvement (error[t-1] - error[t])
#     - global reward: based on progress over the cycle (same scalar)
#
# IMPORTANT:
# - This controller uses RAW ticks intentionally.
#   “Calibration” in LeRobot is for *normalized units* (degrees/[-100..100]) and requires
#   a calibration registry being loaded by the bus. Your cage + target are already in RAW,
#   so RAW is the safest/most deterministic path right now.
#
# Safety tips:
# - Keep movements SMALL first (DEFAULT_PER_STEP_MAX_TICKS = 3).
# - Don’t manually move the arm while torque is enabled.
# - Unplug power when done (Feetech/Dynamixel-like servos can overheat).
#
# Run:
#   source .venv/bin/activate
#   export API_KEY=...
#   export PROJECT_ID=...
#   export AB_BASE_URL=http://app.artificialbrains.ai/api   # or your server
#   export SO101_PORT=/dev/ttyACM0
#   export SO101_ID=my_awesome_follower_arm
#   python so101_ab_controller.py
#
# Files expected:
#   targets/safety_cage.json
#   targets/target_touch_mat.json
#
# References (LeRobot read/write names):
# - Present_Position, Goal_Position, Torque_Enable are standard “bus data_name” used by LeRobot examples. :contentReference[oaicite:0]{index=0}

from __future__ import annotations

import os
import sys
import json
import time
import copy
import subprocess
import inspect
import math
import signal
import statistics
import threading
import re
import cv2
import numpy as np
import atexit
import logging
import traceback
from pathlib import Path
from types import SimpleNamespace
from collections import deque
from array import array
from typing import Any, Dict, List, Optional, Tuple
from policies._contract import (
    STDP3_LAYERS,
    FEEDBACK_IDS as CONTRACT_FEEDBACK_IDS,
    FEEDBACK_T as CONTRACT_FEEDBACK_T,
    FEEDBACK_N as CONTRACT_FEEDBACK_N,
    FEEDBACK_INFO as CONTRACT_FEEDBACK_INFO,
)
from policies.reward_policy import CycleSummary as RewardSummary, compute_reward as compute_reward_policy
from policies.error_deviation_policy import ( ErrorContext, compute_error )

# ----------------------------
# Execution / buffering alignment
# ----------------------------
# We buffer up to 2 cycles total (executing + next), to support the "two-cycle" pipeline:
# - while the brain computes cycle C+1, the robot executes cycle C
# - keep at most MAX_AHEAD cycles buffered beyond the currently executing cycle
MAX_AHEAD = 2

# IMPORTANT: LeRobot Feetech/Dynamixel-style buses are NOT thread-safe.
# AB SDK callbacks (io:need) can run concurrently with your main loop.
# Serialize ALL bus access to avoid "Incorrect status packet" / "No status packet".
#
bus_lock = threading.Lock()
last_q_raw: Optional[List[float]] = None
prev_q_raw: Optional[List[float]] = None
last_err: Optional[float] = None
last_q_ts: float = 0.0
prev_q_ts: float = 0.0


HERE = Path(__file__).resolve().parent
ENV_PATH = HERE / ".env"

def load_env_file(path: Path) -> None:
    if not path.exists():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        k = k.strip()
        v = v.strip().strip("'").strip('"')
        # only set if not already exported in the shell
        os.environ.setdefault(k, v)

load_env_file(ENV_PATH)

# Optional: also support python-dotenv if installed (won't crash if missing)
try:
    from dotenv import load_dotenv  # type: ignore
    load_dotenv(ENV_PATH)
except Exception:
    pass

# ----------------------------
# AB SDK
# ----------------------------
from ab_sdk import ABClient
from ab_sdk.plugins.decoder import decode_stream_rows


# ----------------------------
# Camera (USB → jpeg for AB Image input)
# ----------------------------
SO101_CAM_DEV = os.getenv("SO101_CAM_DEV", "/dev/video2")
SO101_CAM_W = int(os.getenv("SO101_CAM_W", "640"))
SO101_CAM_H = int(os.getenv("SO101_CAM_H", "480"))
SO101_CAM_JPEG_QUALITY = int(os.getenv("SO101_CAM_JPEG_QUALITY", "85"))


# ----------------------------
# Logging
# ----------------------------
logging.basicConfig(level=logging.INFO)
log = logging.getLogger("so101_ab_controller")
logging.getLogger("ab_sdk").setLevel(logging.INFO)


def _row_to_tuple(row):
    """
    Normalize AB output rows:
      dict: {"t":..., "id":..., "bits":[...]}
      list/tuple: [t, id, bits]
    Returns (t, id, bits) or (None, None, None)
    """
    if isinstance(row, dict):
        return row.get("t"), row.get("id"), row.get("bits")
    if isinstance(row, (list, tuple)) and len(row) >= 3:
        return row[0], row[1], row[2]
    return None, None, None

def _bitcount(bits):
    try:
        # bits might be list[int] 0/1
        return int(sum(1 for b in bits if int(b) == 1))
    except Exception:
        return None
    

def read_raw(bus: Any, data_name: str) -> List[int]:
    with bus_lock:
        m = bus.sync_read(data_name, motor_names, normalize=False)
    return [int(m[n]) for n in motor_names]



# ----------------------------
# Goal-reached stopper (wait until motor converges)
# ----------------------------
# If we command faster than the servo can move, we "queue" goals and it looks broken.
# This stopper blocks the next step until the joint is close enough to its goal (or timeout).
SO101_WAIT_EPS_TICKS    = float(os.getenv("SO101_WAIT_EPS_TICKS", "6.0"))   # |goal-present| <= eps
SO101_WAIT_STABLE_READS = int(os.getenv("SO101_WAIT_STABLE_READS", "2"))    # must be in-range N polls
SO101_WAIT_TIMEOUT_S    = float(os.getenv("SO101_WAIT_TIMEOUT_S", "0.40"))  # max wait per step
SO101_WAIT_POLL_HZ      = float(os.getenv("SO101_WAIT_POLL_HZ", "60"))      # polling rate
SO101_WAIT_MIN_DV_TICKS = float(os.getenv("SO101_WAIT_MIN_DV_TICKS", "1.0"))# don't wait for tiny/no-op moves

def wait_until_joint_reached(
    bus: Any,
    joint_idx: int,
    *,
    eps_ticks: float = SO101_WAIT_EPS_TICKS,
    stable_reads: int = SO101_WAIT_STABLE_READS,
    timeout_s: float = SO101_WAIT_TIMEOUT_S,
    poll_hz: float = SO101_WAIT_POLL_HZ,
) -> bool:
    """
    Returns True if joint's Present_Position reaches Goal_Position within eps_ticks
    for stable_reads consecutive polls, else False on timeout.
    """
    eps = float(max(0.0, eps_ticks))
    need = int(max(1, stable_reads))
    dt = 1.0 / float(max(5.0, poll_hz))
    t0 = time.time()
    ok = 0
    while True:
        try:
            g = read_raw(bus, DATA_GOAL)
            p = read_raw(bus, DATA_PRESENT)
            if abs(float(g[joint_idx]) - float(p[joint_idx])) <= eps:
                ok += 1
                if ok >= need:
                    return True
            else:
                ok = 0
        except Exception:
            ok = 0
        if (time.time() - t0) >= float(timeout_s):
            return False
        time.sleep(dt)


 # ----------------------------
# Mode setting (from smoke test)
# ----------------------------
def set_mode_position(bus: Any, motor_names: List[str]) -> None:
    """
    Force position-control mode before torque enable.
    Some Feetech setups won't actuate unless Mode is set.
    """
    ok = 0
    for n in motor_names:
        try:
            bus.write(DATA_MODE, 0, n, normalize=False)
            ok += 1
        except Exception:
            pass
    if ok:
        log.info(f"[mode] set Mode=0 for {ok}/{len(motor_names)} motors")
    else:
        log.warning(f"[mode] could not set Mode (may prevent actuation)")



# ----------------------------
# Camera helpers
# ----------------------------

def get_rgb_jpeg_bytes_raspberry() -> Tuple[bytes, Dict[str, Any]]:
    """
    Capture one frame directly to memory (no temp files) - Raspberry Pi version.
    """
    dev = str(SO101_CAM_DEV)
    w = int(SO101_CAM_W)
    h = int(SO101_CAM_H)
    quality = int(SO101_CAM_JPEG_QUALITY)
    
    # Force V4L2 backend and use device index
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Use index 0 for /dev/video0
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {dev}")
    
    try:
        # Set MJPEG format (camera supports it, more efficient)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))  # Fixed this line
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Grab a few frames to let camera auto-adjust
        for _ in range(3):
            cap.grab()
        
        # Capture the actual frame
        ret, frame = cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to capture frame")
        
        # Encode to JPEG in memory
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        ret, jpeg_buf = cv2.imencode('.jpg', frame, encode_param)
        if not ret:
            raise RuntimeError("Failed to encode JPEG")
        
        jpeg_bytes = jpeg_buf.tobytes()
        meta = {
            "width": w,
            "height": h,
            "quality": quality,
            "device": dev,
            "via": "opencv-v4l2"
        }
        
        return jpeg_bytes, meta
        
    finally:
        cap.release()


def get_rgb_jpeg_bytes_default() -> Tuple[bytes, Dict[str, Any]]:
    """
    Capture one frame directly to memory (no temp files).
    """
    dev = str(SO101_CAM_DEV)
    w = int(SO101_CAM_W)
    h = int(SO101_CAM_H)
    quality = int(SO101_CAM_JPEG_QUALITY)
    
    # Open camera
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {dev}")
    
    try:
        # Set resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        
        # Grab a few frames to let camera auto-adjust
        for _ in range(3):
            cap.read()
        
        # Capture the actual frame
        ret, frame = cap.read()
        if not ret or frame is None:
            raise RuntimeError("Failed to capture frame")
        
        # Encode to JPEG in memory
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        ret, jpeg_buf = cv2.imencode('.jpg', frame, encode_param)
        if not ret:
            raise RuntimeError("Failed to encode JPEG")
        
        jpeg_bytes = jpeg_buf.tobytes()
        meta = {
            "width": w,
            "height": h,
            "quality": quality,
            "device": dev,
            "via": "opencv"
        }
        
        return jpeg_bytes, meta
        
    finally:
        cap.release()


IS_RASPBERRY = os.getenv("IS_RASPBERRY", "false").lower() in ("true", "1", "yes")

if IS_RASPBERRY:
    get_rgb_jpeg_bytes = get_rgb_jpeg_bytes_raspberry
else:
    get_rgb_jpeg_bytes = get_rgb_jpeg_bytes_default

# ----------------------------
# Env
# ----------------------------
API_KEY = os.getenv("API_KEY", "") or os.getenv("AB_API_KEY", "")
PROJECT_ID = os.getenv("PROJECT_ID", "")
BASE_URL = os.getenv("AB_BASE_URL", "http://app.artificialbrains.ai/api")
NAME_SPACE = os.getenv("SOCKET_NAMESPACE", "/ab")

SO101_PORT = os.getenv("SO101_PORT", "/dev/ttyACM0")
SO101_ID = os.getenv("SO101_ID", "my_awesome_follower_arm")

TARGETS_DIR = Path(os.getenv("SO101_TARGETS_DIR", "targets"))
SAFETY_PATH = Path(os.getenv("SO101_SAFETY_JSON", str(TARGETS_DIR / "safety_cage.json")))
TARGET_PATH = Path(os.getenv("SO101_TARGET_JSON", str(TARGETS_DIR / "target_touch_mat.json")))


# ----------------------------
# Run mode: reset vs normal (startup-only)
# ----------------------------
# SO101_DO_RESET=1 enables:
#   - home to init pose at startup (if file exists)
#   - clear local queues/buffers/state before loop
#   - reset integrator baseline every timestep (goal_q := q_now)
SO101_DO_RESET = bool(int(os.getenv("SO101_DO_RESET", "0")))
INIT_PATH = Path(os.getenv("SO101_INIT_JSON", str(TARGETS_DIR / "init_pose.json")))

# Control loop
CONTROL_HZ = float(os.getenv("SO101_CONTROL_HZ", "30"))  # small and steady
if CONTROL_HZ <= 0:
    raise SystemExit(f"[FATAL] SO101_CONTROL_HZ must be > 0 (got {CONTROL_HZ})")
DT = 1.0 / CONTROL_HZ

# Motion scaling (start SMALL)
DEFAULT_PER_STEP_MAX_TICKS = float(os.getenv("SO101_PER_STEP_MAX_TICKS", "10"))
DEFAULT_GAIN = float(os.getenv("SO101_GAIN", "1"))
DEFAULT_DEADZONE = float(os.getenv("SO101_DEADZONE", "0.0"))

# Reward/feedback shaping
SUCCESS_ERR = float(os.getenv("SO101_SUCCESS_ERR", "0.03"))      # normalized error threshold

# ---- Actuation diagnostics ----
# If Goal_Position changes but Present_Position does not, this helps pinpoint *which* joints are stuck.
STUCK_POS_EPS_TICKS = float(os.getenv("SO101_STUCK_POS_EPS_TICKS", "1.0"))     # "no motion" threshold
STUCK_GOAL_EPS_TICKS = float(os.getenv("SO101_STUCK_GOAL_EPS_TICKS", "1.0"))   # goal changed threshold
STUCK_STEPS_TO_WARN  = int(os.getenv("SO101_STUCK_STEPS_TO_WARN", "20"))       # ~20 steps @30Hz ~= 0.66s

# Optional: set a speed value at startup (some buses default to very slow/0 depending on model/firmware)
SO101_SET_SPEED_ON_START = bool(int(os.getenv("SO101_SET_SPEED_ON_START", "1")))
SO101_SPEED_VALUE = int(os.getenv("SO101_SPEED_VALUE", "300"))

MAX_USEFUL_PROGRESS = float(os.getenv("SO101_MAX_PROGRESS", "0.20"))  # normalized error improvement for “full reward”
DEV_CAP = float(os.getenv("SO101_DEV_CAP", "0.06"))
DEV_SMOOTH = float(os.getenv("SO101_DEV_SMOOTH", "0.25"))
DEV_DEADZONE = float(os.getenv("SO101_DEV_DEADZONE", "0.0"))

# Optional: scale raw dd so it's not microscopic (does NOT clamp)
DEV_SCALE = float(os.getenv("SO101_DEV_SCALE", "50.0"))

active_intent: Optional[Dict[str, float]] = None
intent_steps_left: int = 0

if not PROJECT_ID:
    raise SystemExit("[FATAL] PROJECT_ID not set")
if not API_KEY:
    log.warning("API_KEY not set. Server may reject auth.")

# ----------------------------
# Helpers
# ----------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)

def now() -> float:
    return time.time()

def load_json(p: Path) -> Dict[str, Any]:
    return json.loads(p.read_text(encoding="utf-8"))

def load_pose_q_raw(
    path: Path,
    *,
    motor_names_ref: List[str],
) -> Optional[List[float]]:
    """
    Load a q_raw pose from a JSON file and align to motor_names_ref.

    Accepts any of these shapes:
      - {"motor_names":[...], "q_raw":[...]}
      - {"robot":{"motor_names":[...]} , "init":{"q_raw":[...]}}
      - {"robot":{"motor_names":[...]} , "target":{"q_raw":[...]}}  (re-use teach_target format)
    """
    if not path.exists():
        return None
    j = load_json(path)
    m = list((j.get("robot") or {}).get("motor_names") or j.get("motor_names") or motor_names_ref)
    q = (
        j.get("q_raw")
        or (j.get("init") or {}).get("q_raw")
        or (j.get("target") or {}).get("q_raw")
        or (j.get("TARGET_Q_RAW"))
        or (j.get("q"))
        or []
    )
    if not q:
        return None
    return ensure_same_order(motor_names_ref, m, [float(x) for x in q])


def ensure_same_order(
    names_ref: List[str],
    names_other: List[str],
    values_other: List[float],
) -> List[float]:
    """
    Reorder values_other (aligned to names_other) to match names_ref ordering.
    """
    idx = {n: i for i, n in enumerate(names_other)}
    out: List[float] = []
    for n in names_ref:
        if n not in idx:
            raise RuntimeError(f"Name '{n}' missing in target file. Have={names_other}")
        out.append(float(values_other[idx[n]]))
    return out

def ema(xs: List[float], a: float) -> List[float]:
    a = clamp(float(a), 0.0, 1.0)
    out: List[float] = []
    e = 0.0
    for v in xs:
        e = (1.0 - a) * e + a * float(v)
        out.append(float(e))
    return out

def resample_to_T(xs: List[float], T: int) -> List[float]:
    T = int(T)
    if T <= 0:
        return []
    if not xs:
        return [0.0] * T
    M = len(xs)
    if M == T:
        return xs[:]
    out = [0.0] * T
    for t in range(T):
        i = int((t / float(T)) * M)
        if i >= M:
            i = M - 1
        out[t] = float(xs[i])
    return out



# ----------------------------
# Cycle smoothing (integrate -> smooth trajectory)
# ----------------------------
# Goal: avoid back-and-forth jitter when decoder emits oscillatory deltas
# across the output window. We keep your server/learn gating intact:
# - buffering per-cycle stays the same (cycle_buffers)
# - cycle_done threshold logic stays the same (_cycle_done_threshold_steps)
# - reward/feedback send after cycle_done is unchanged
#
# Strategy:
#   1) Sum (integrate) decoded deltas over the whole cycle window -> total per joint
#   2) Re-distribute that total over N steps using smoothstep ramp
#
def _smoothstep(u: float) -> float:
    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
    return u * u * (3.0 - 2.0 * u)  # C1 continuous endpoints

def _sum_cycle_deltas(cmds: List[Dict[str, Any]]) -> Dict[str, float]:
    acc: Dict[str, float] = {}
    for c in cmds:
        d = c.get("deltas") or {}
        if not isinstance(d, dict):
            continue
        for k, v in d.items():
            if isinstance(v, (int, float)):
                kk = str(k)
                acc[kk] = acc.get(kk, 0.0) + float(v)
    return acc

def _make_smooth_plan(total: Dict[str, float], steps: int) -> List[Dict[str, Any]]:
    steps = int(max(1, steps))
    plan: List[Dict[str, Any]] = []
    prev_s = 0.0
    for i in range(steps):
        u = (i + 1) / float(steps)
        s = _smoothstep(u)
        ds = s - prev_s
        prev_s = s
        deltas = {k: float(v) * ds for k, v in total.items()}
        plan.append({"t": i, "deltas": deltas})
    return plan

# ----------------------------
# Load safety + target
# ----------------------------
if not SAFETY_PATH.exists():
    raise SystemExit(f"[FATAL] Missing safety cage: {SAFETY_PATH}")
if not TARGET_PATH.exists():
    raise SystemExit(f"[FATAL] Missing target pose: {TARGET_PATH}")

safety = load_json(SAFETY_PATH)
target = load_json(TARGET_PATH)

motor_names: List[str] = list(safety.get("robot", {}).get("motor_names") or safety.get("motor_names") or [])
if not motor_names:
    # fallback to common
    motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

buffer_ticks = float(safety.get("buffer_ticks", 25.0))
limits_raw: Dict[str, Dict[str, float]] = safety.get("limits_raw") or {}

# target file could be written by our earlier script(s)
t_motor_names: List[str] = list(target.get("motor_names") or motor_names)
t_q_raw: List[float] = list(
    target.get("q_raw")
    or (target.get("target") or {}).get("q_raw")
    or target.get("TARGET_Q_RAW")
    or target.get("q")
    or []
)
if not t_q_raw:
    raise SystemExit(f"[FATAL] Target pose file missing q_raw list: {TARGET_PATH}")

target_q_raw: List[float] = ensure_same_order(motor_names, t_motor_names, [float(x) for x in t_q_raw])

# Precompute per-motor clamping bounds (min+buffer .. max-buffer)
cage_lo: List[float] = []
cage_hi: List[float] = []
cage_span: List[float] = []
for n in motor_names:
    lim = limits_raw.get(n)
    if not lim:
        raise SystemExit(f"[FATAL] safety cage missing limits for motor '{n}'. Found keys={list(limits_raw.keys())}")
    lo = float(lim["min"]) + float(buffer_ticks)
    hi = float(lim["max"]) - float(buffer_ticks)
    if hi <= lo:
        raise SystemExit(f"[FATAL] bad cage for '{n}': min/max too tight after buffer (lo={lo}, hi={hi})")
    cage_lo.append(lo)
    cage_hi.append(hi)
    cage_span.append(max(1.0, hi - lo))

log.info(f"[targets] motor_names={motor_names}")
log.info(f"[targets] target_q_raw={','.join(str(int(x)) for x in target_q_raw)}")
log.info(f"[cage] buffer_ticks={buffer_ticks} (raw ticks)")

# Optional init pose (used only in reset mode)
init_q_raw: Optional[List[float]] = None
if SO101_DO_RESET:
    init_q_raw = load_pose_q_raw(INIT_PATH, motor_names_ref=motor_names)
    if init_q_raw is None:
        log.warning(f"[reset] SO101_DO_RESET=1 but init pose missing/invalid at {INIT_PATH} (will use current pose)")
    else:
        log.info(f"[reset] init pose loaded from {INIT_PATH}")

# ----------------------------
# LeRobot: SOFollower (Feetech bus)
# ----------------------------
def connect_robot() -> Tuple[Any, Any]:
    """
    Returns (robot, bus). We'll use bus.sync_read/sync_write in RAW mode.
    """
    try:
        from lerobot.robots.so_follower.so_follower import SOFollower
    except Exception:
        log.error("Failed to import SOFollower from lerobot.robots.so_follower.so_follower")
        raise

    cfg = SimpleNamespace(
        calibration_dir=None,             # we stay RAW (your cage+target are RAW)
        cameras={},
        disable_torque_on_disconnect=True,
        id=SO101_ID,
        max_relative_target=None,
        port=SO101_PORT,
        use_degrees=False,
    )

    r = SOFollower(cfg)
    r.connect(calibrate=False)

    bus = getattr(r, "bus", None)
    if bus is None:
        raise RuntimeError("Robot has no .bus attribute")

    return r, bus

def _list_control_table_keys(bus: Any) -> List[str]:
    """
    Best-effort: discover which control-table fields exist for this bus instance.
    Different LeRobot backends expose this differently.
    """
    for attr in ("control_table", "_control_table", "ct", "_ct"):
        ct = getattr(bus, attr, None)
        if isinstance(ct, dict):
            return sorted(ct.keys())
    # some implementations keep it on the underlying driver
    drv = getattr(bus, "driver", None) or getattr(bus, "_driver", None)
    for attr in ("control_table", "_control_table", "ct", "_ct"):
        ct = getattr(drv, attr, None)
        if isinstance(ct, dict):
            return sorted(ct.keys())
    return []

def _pick_first_existing(keys: List[str], candidates: List[str]) -> Optional[str]:
    low = {k.lower(): k for k in keys}
    for c in candidates:
        k = low.get(c.lower())
        if k:
            return k
    return None

DATA_PRESENT = "Present_Position"
DATA_GOAL = "Goal_Position"
DATA_TORQUE = "Torque_Enable"
DATA_SPEED = "Moving_Speed" 

def _sync_write_any(bus: Any, data_name: str, names: List[str], values: List[int], *, normalize: bool) -> None:
    """
    LeRobot has multiple sync_write signatures depending on version/backend.

    Try, in order:
      A) sync_write(data_name, names, values, normalize=...)
      B) sync_write(data_name, mapping{name: value}, normalize=...)
      C) sync_write(mapping{name: {data_name: value}}, normalize=...)  (rare)
    """
    # A) (data_name, names, values)
    try:
        return bus.sync_write(data_name, names, values, normalize=normalize)
    except TypeError:
        pass
    except Exception:
        # don't swallow non-signature failures here; let other forms try
        pass

    # B) (data_name, mapping)
    mapping = {n: int(values[i]) for i, n in enumerate(names)}
    try:
        return bus.sync_write(data_name, mapping, normalize=normalize)
    except TypeError:
        pass

    try:
        return bus.sync_write(data_name, mapping, normalize)
    except TypeError:
        pass
    # C) mapping-of-mapping fallback
    mapping2 = {n: {data_name: int(values[i])} for i, n in enumerate(names)}
    try:
        return bus.sync_write(mapping2, normalize=normalize)
    except TypeError:
        return bus.sync_write(mapping2, normalize)

def read_q_raw(bus: Any) -> List[float]:
    with bus_lock:
        pos_map = bus.sync_read(DATA_PRESENT, motor_names, normalize=False)
    out: List[float] = []
    for n in motor_names:
        v = pos_map.get(n, None)
        if v is None:
            raise RuntimeError(f"sync_read missing '{n}'")
        out.append(float(v))
    return out

def write_speed_raw(bus: Any, speed: int) -> None:
    """
    Set a reasonable speed/profile if the control table supports it.
    Your previous attempt hard-coded names that often do NOT exist (e.g. sts3215 table).
    This version discovers the *actual* key name first.
    """
    sp = int(speed)
    keys = _list_control_table_keys(bus)
    # ADD "Step_Speed" to this list as it is common for SO-101
    key = _pick_first_existing(
        keys,
        candidates=["Step_Speed", "Moving_Speed", "Goal_Speed", "Speed"]
    )
    if not key:
        log.warning(f"[SO101] speed not set: no supported speed key found in control table (keys~{[k for k in keys if 'speed' in k.lower() or 'vel' in k.lower()][:10]})")
        return
    vals = [sp] * len(motor_names)
    try:
        with bus_lock:
            _sync_write_any(bus, key, motor_names, vals, normalize=False)
        log.info(f"[SO101] set {key}={sp} for all motors")
    except Exception as e:
        log.warning(f"[SO101] set {key} failed: {e}")

def write_goal_raw(bus: Any, q_goal: List[float]) -> None:
    vals = [int(round(x)) for x in q_goal]
    with bus_lock:
        _sync_write_any(bus, DATA_GOAL, motor_names, vals, normalize=False)

def set_torque(bus: Any, enabled: bool) -> None:
    """
    Enable/disable torque safely.
    Your previous version tried to write a hard-coded "Mode" register (often doesn't exist)
    AND ran the fallback _inside_ the per-motor loop (can spam/wedge the bus).
    """
    val = 1 if enabled else 0
    # First try sync_write (fast + consistent)
    try:
        with bus_lock:
            _sync_write_any(bus, DATA_TORQUE, motor_names, [val] * len(motor_names), normalize=False)
        return
    except Exception:
        pass

    # Fallback: per-motor write with common argument orders
    for n in motor_names:
        try:
            with bus_lock:
                bus.write(DATA_TORQUE, n, val, normalize=False)
            continue
        except TypeError:
            pass
        except Exception:
            pass
        try:
            with bus_lock:
                bus.write(DATA_TORQUE, val, n, normalize=False)
        except Exception:
            pass

def debug_bus_registers(bus: Any) -> None:
    keys = _list_control_table_keys(bus)
    if not keys:
        log.info("[SO101] debug: no visible control_table keys on bus")
        return
    speedish = [k for k in keys if ("speed" in k.lower() or "vel" in k.lower() or "profile" in k.lower())]
    modeish  = [k for k in keys if ("mode" in k.lower() or "operation" in k.lower() or "work" in k.lower())]
    log.info(f"[SO101] debug: speed-ish keys={speedish[:20]}")
    log.info(f"[SO101] debug: mode-ish  keys={modeish[:20]}")


def read_torque(bus: Any) -> List[int]:
    try:
        with bus_lock:
            m = bus.sync_read(DATA_TORQUE, motor_names, normalize=False)
        return [int(m[n]) for n in motor_names]
    except Exception:
        return [-1] * len(motor_names)


def startup_jog_test(bus: Any) -> bool:
    """
    Minimal diagnostic:
      1) read Present_Position
      2) write Goal_Position with a tiny delta on one joint (clamped)
      3) read Present_Position -> confirm movement
      4) return to original goal -> confirm return

    Returns True if any meaningful movement is observed.
    """
    try:
        q0 = read_q_raw(bus)
        if not q0 or len(q0) != len(motor_names):
            log.warning("[jog] cannot read initial pose; skipping jog test")
            return False

        # Try up to two joints: first non-gripper, then the next one.
        candidates = []
        for i, n in enumerate(motor_names):
            if "gripper" not in n.lower():
                candidates.append(i)
        if not candidates:
            candidates = [0]
        candidates = candidates[:2]

        jog = int(JOG_TICKS)
        if jog == 0:
            log.info("[jog] JOG_TICKS=0; skipping jog test")
            return False

        for test_idx in candidates:
            name = motor_names[test_idx]

            # Build goal with +jog (clamp to cage)
            q1 = q0[:]
            q1[test_idx] = clamp(float(q1[test_idx] + jog), float(cage_lo[test_idx]), float(cage_hi[test_idx]))
            q2 = q0[:]  # return target

            log.info(f"[jog] testing joint='{name}' idx={test_idx} jog={jog} ticks")
            log.info(f"[jog] q0[{name}]={int(round(q0[test_idx]))} -> q1[{name}]={int(round(q1[test_idx]))} (clamped)")

            # Write jog goal
            write_goal_raw(bus, q1)
            time.sleep(float(JOG_WAIT_S))
            p1 = read_q_raw(bus)
            g1 = read_raw(bus, DATA_GOAL)

            # Write return goal
            write_goal_raw(bus, q2)
            time.sleep(float(JOG_WAIT_S))
            p2 = read_q_raw(bus)
            g2 = read_raw(bus, DATA_GOAL)

            # Separate "write didn't apply" from "motor didn't move"
            dg1 = abs(float(g1[test_idx]) - float(q0[test_idx]))
            dp1 = abs(float(p1[test_idx]) - float(q0[test_idx]))
            dp2 = abs(float(p2[test_idx]) - float(q0[test_idx]))

            log.info(f"[jog] goal  after jog:  {int(g1[test_idx])} (Δgoal={dg1:.1f})")
            log.info(f"[jog] present after jog: {int(round(p1[test_idx]))} (Δpos={dp1:.1f})")
            log.info(f"[jog] goal  after return:{int(g2[test_idx])}")
            log.info(f"[jog] present after return:{int(round(p2[test_idx]))} (Δpos={dp2:.1f})")

            wrote_ok = dg1 >= 2.0
            moved_ok = (dp1 >= 2.0) or (dp2 >= 2.0)
            if wrote_ok and moved_ok:
                log.info("[jog] ✅ write+motion observed (Goal_Position writes + motor response OK)")
                return True
            if wrote_ok and not moved_ok:
                log.warning("[jog] ⚠️ Goal_Position changed but Present_Position did NOT (motor/ID/power/gear/cage issue)")
            if not wrote_ok:
                log.warning("[jog] ❌ Goal_Position did NOT change (bus sync_write signature / motor name->ID mapping issue)")

        return False
    except Exception as e:
        log.warning(f"[jog] test failed: {e}")
        return False


# ----------------------------
# AB mapping: output population -> joint name
# User request: mapping should be name of joint (your id) to the id.
# So: node_id == channel == joint name.
# ----------------------------
mapping: List[Dict[str, Any]] = []
for jname in motor_names:
    mapping.append(
        {
            "node_id": jname,                 # output pop id in AB graph
            "channel": jname,                 # controller channel = motor name
            "scheme": "bipolarSplit",
            "per_step_max": float(DEFAULT_PER_STEP_MAX_TICKS),
            "gain": float(DEFAULT_GAIN),
            "deadzone": float(DEFAULT_DEADZONE),
        }
    )

output_channel_by_pop = {m["node_id"]: m["channel"] for m in mapping}

# ----------------------------
# Tracking for feedback/reward
# ----------------------------
# We’ll record per-output-timestep objective scalar:
#   err_by_t[t] = normalized joint error to target (lower is better)
err_by_t: Dict[int, float] = {}
applied_deltas_by_t: Dict[int, Dict[str, float]] = {}


def _per_joint_errors_from_q(q_raw: List[float]) -> Dict[str, float]:
    """
    Normalized abs error per joint for an arbitrary pose.
    """
    out: Dict[str, float] = {}
    for i, jname in enumerate(motor_names):
        out[jname] = float(abs(float(q_raw[i]) - float(target_q_raw[i])) / float(cage_span[i]))
    return out


def objective_error(q_now: List[float]) -> float:
    # normalized mean absolute error across joints (0..~1)
    acc = 0.0
    for i in range(len(motor_names)):
        acc += abs(float(q_now[i]) - float(target_q_raw[i])) / float(cage_span[i])
    return float(acc / max(1, len(motor_names)))


def _per_joint_errors(q_now: List[float]) -> Dict[str, float]:
    """
    Normalized abs error per joint.
    Deterministic + in [0..~1].
    """
    out: Dict[str, float] = {}
    for i, jname in enumerate(motor_names):
        out[jname] = float(abs(float(q_now[i]) - float(target_q_raw[i])) / float(cage_span[i]))
    return out


def _drop_cycles_older_than(base_cyc: int) -> None:
    """
    Hard invariant:
      - never keep buffers for cycles <= base_cyc (already executing or done),
      - keep at most MAX_AHEAD future cycles beyond base_cyc.
    """
    try:
        base = int(base_cyc)
    except Exception:
        return

    # Drop anything older/equal to base (we never go backwards)
    old = [c for c in list(cycle_buffers.keys()) if int(c) <= base]
    for c in old:
        try:
            del cycle_buffers[c]
        except Exception:
            pass

    # Keep only (base+1 .. base+MAX_AHEAD) among remaining
    keep = set([c for c in cycle_buffers.keys() if (int(c) > base and int(c) <= base + int(MAX_AHEAD))])
    drop = [c for c in list(cycle_buffers.keys()) if c not in keep]
    for c in drop:
        try:
            del cycle_buffers[c]
        except Exception:
            pass

def _arm_next_cycle_if_idle() -> None:
    global active_intent, intent_steps_left, executing_cycle
    global executed_steps_in_cycle, goal_q

    if active_intent is not None or not cycle_buffers:
        return

    pick = min(cycle_buffers.keys())
    intent = cycle_buffers.pop(pick)
    
    # RESET EVERYTHING TO FRESH START
    executed_steps_in_cycle = 0
    active_intent = dict(intent["total_deltas"])
    intent_steps_left = len(motor_names)
    executing_cycle = int(pick)

    # CRITICAL: Reset the goal to the ACTUAL current position
    # This prevents the 'repetitive' math accumulation
    goal_q = None
    
    log.info(f"[intent] NEW CYCLE {pick} - Baseline reset to present pose.")


def _finalize_cycle_local(cyc: int) -> None:
    """
    After we send learning + mark cycle done, drop any local state for that cycle
    so nothing can "accumulate" across cycles.
    """
    c = int(cyc)
    # Drop per-cycle pose snapshots
    try: cycle_start_q_raw.pop(c, None)
    except Exception: pass
    try: cycle_end_q_raw.pop(c, None)
    except Exception: pass
    # Drop expected_T bookkeeping
    try: expected_T_by_cycle.pop(c, None)
    except Exception: pass
    # NOTE: err_by_t/applied_deltas_by_t are keyed only by t (0..T-1) today.
    # Clear them on finalize so they don't accumulate indefinitely.
    try: err_by_t.clear()
    except Exception: pass
    try: applied_deltas_by_t.clear()
    except Exception: pass



def _clear_local_execution_state() -> None:
    """
    Clears ONLY local controller execution state.
    Does NOT stop the AB run or change socket callbacks.
    Does NOT change the cycle_done / send_learning_for_cycle flow.
    """
    global goal_q, cmd_queue, queued_cycle, cycle_buffers, latest_cycle_seen
    global executing_cycle, executed_any_for_cycle, executed_steps_in_cycle, cycle_done_sent
    global expected_T_by_cycle, err_by_t, applied_deltas_by_t
    global cycle_start_q_raw, cycle_end_q_raw

    # integrator
    goal_q = None

    # queues/buffers
    cycle_buffers.clear()
    latest_cycle_seen = -1

    # cycle tracking
    executing_cycle = None
    executed_any_for_cycle = False
    executed_steps_in_cycle = 0
    cycle_done_sent.clear()
    expected_T_by_cycle.clear()

    # learning alignment
    err_by_t.clear()
    applied_deltas_by_t.clear()
    cycle_start_q_raw.clear()
    cycle_end_q_raw.clear()

# ----------------------------
# Safe integrator (tiny deltas, clamp to cage, rate limit)
# ----------------------------
goal_q: Optional[List[float]] = None

# Buffer future cycles as INTENTS (one per cycle).
# Key: cycle -> {"cycle": int, "total_deltas": {joint: float, ...}}
cycle_buffers: Dict[int, Dict[str, Any]] = {}
latest_cycle_seen: int = -1

# ----------------------------
# Cycle execution tracking
# ----------------------------
# We only tell the server "robot:cycle_done" AFTER we actually execute
# the whole output window for a cycle (i.e., after we drain cmd_queue for that cycle).
executing_cycle: Optional[int] = None
executed_any_for_cycle: bool = False
cycle_done_sent: set[int] = set()

# track expected T per cycle based on t_max from cycle:update
expected_T_by_cycle: Dict[int, int] = {}
executed_steps_in_cycle: int = 0
last_cycle_update_ts: float = 0.0

DEBUG_MOTION_READBACK = bool(int(os.getenv("SO101_DEBUG_MOTION", "0")))

# Ensure reset happens exactly once per cycle, and only AFTER finishing execution.
reset_done_for_cycle: set[int] = set()

# per-cycle pose snapshots for reward policy
# - start_q_raw: first pose observed when executing a given cycle
# - end_q_raw: last pose observed while executing that cycle
cycle_start_q_raw: Dict[int, List[float]] = {}
cycle_end_q_raw: Dict[int, List[float]] = {}

def apply_safe_deltas(
    q_now: List[float],
    deltas: Dict[str, float],
) -> List[float]:
    """
    Returns q_goal_next (raw ticks) after:
      - init goal from current pose
      - apply deltas (already in ticks scale from decoder mapping)
      - cage clamp
    """
    global goal_q
    # Baseline integrator: keep a goal memory across steps so deltas can accumulate.
    # (Reset is applied AFTER a cycle finishes, not every timestep.)
    if goal_q is None:
        goal_q = q_now[:]

    q_next = goal_q[:]

    # apply per joint (if channel matches motor name)
    for i, name in enumerate(motor_names):
        dv = deltas.get(name, 0.0)
        if not isinstance(dv, (int, float)):
            dv = 0.0

        q_next[i] = float(q_next[i]) + float(dv)

        # clamp to cage
        q_next[i] = clamp(float(q_next[i]), float(cage_lo[i]), float(cage_hi[i]))

    goal_q = q_next[:]
    return q_next

client = None
run = None
server_constants: Dict[str, Any] = {}

def _feedback_n_for_id(fid: str) -> int:
    """
    Prefer the per-feedback N from the synced contract (CONTRACT_FEEDBACK_INFO),
    else fall back to CONTRACT_FEEDBACK_N.
    """
    try:
        for info in (CONTRACT_FEEDBACK_INFO or []):
            if str(info.get("id")) == str(fid):
                n = int(info.get("n") or 0)
                if n > 0:
                    return n
    except Exception:
        pass
    return int(CONTRACT_FEEDBACK_N)

def send_learning_for_cycle(cyc: int) -> None:
    # Use the machine-owned synced contract (no reprocessing / no rediscovery)
    feedback_t = int(CONTRACT_FEEDBACK_T)

    # ---- FEEDBACK ----
    if feedback_t > 0 and CONTRACT_FEEDBACK_IDS:
        # Build per-cycle init/end/target snapshots for the ERROR policy.
        # (This does NOT affect reward; reward still uses q0/q1 below.)
        q0 = cycle_start_q_raw.get(int(cyc)) or (last_q_raw[:] if last_q_raw else target_q_raw[:])
        q1 = cycle_end_q_raw.get(int(cyc))   or (last_q_raw[:] if last_q_raw else target_q_raw[:])

        q_init_by_joint   = {motor_names[i]: float(q0[i]) for i in range(len(motor_names))}
        q_end_by_joint    = {motor_names[i]: float(q1[i]) for i in range(len(motor_names))}
        q_target_by_joint = {motor_names[i]: float(target_q_raw[i]) for i in range(len(motor_names))}

        ctx = ErrorContext(
            q_init_by_joint=q_init_by_joint,
            q_end_by_joint=q_end_by_joint,
            q_target_by_joint=q_target_by_joint,
        )

        for fb_id in CONTRACT_FEEDBACK_IDS:
            fbN = _feedback_n_for_id(str(fb_id))
            dev = compute_error(str(fb_id), T=feedback_t, ctx=ctx)

            # DEBUG per channel
            try:
                dv_min = min(dev) if dev else 0.0
                dv_max = max(dev) if dev else 0.0
                dv_mean = (sum(abs(x) for x in dev) / len(dev)) if dev else 0.0
                log.info(
                    f"[learn] err[{fb_id}] stats: min={dv_min:.6f} max={dv_max:.6f} mean|.|={dv_mean:.6f} T={len(dev)}"
                )
            except Exception:
                pass

            arr = array("f", [float(x) for x in dev])
            if sys.byteorder != "little":
                arr.byteswap()
            payload = arr.tobytes()
            # IMPORTANT:
            # - seq MUST be the executed cycle (server falls back to seq if cycle missing)
            # - include cycle in meta for debugging even if SDK/server ignores it
            run.send_input_chunk(
                input_id=str(fb_id),
                kind="Feedback",
                seq=int(cyc),
                t=now(),
                fmt="deviation_f32",
                meta={"T": int(feedback_t), "N": int(fbN), "cycle": int(cyc)},
                data=payload,
            )
    else:
        log.warning(
            f"[learn] feedback disabled by contract: FEEDBACK_T={feedback_t} "
            f"FEEDBACK_IDS={list(CONTRACT_FEEDBACK_IDS) if CONTRACT_FEEDBACK_IDS else []}"
        )
 

    # ---- REWARD ----
    q0 = cycle_start_q_raw.get(int(cyc)) or (last_q_raw[:] if last_q_raw else target_q_raw[:])
    q1 = cycle_end_q_raw.get(int(cyc))   or (last_q_raw[:] if last_q_raw else target_q_raw[:])

    summary = RewardSummary(
        motor_names=motor_names,
        target_q_raw=target_q_raw,
        start_q_raw=q0,
        end_q_raw=q1,
        span_raw=cage_span,
        max_useful_progress=float(MAX_USEFUL_PROGRESS),
        success_err=float(SUCCESS_ERR),
    )
    global_r, by_layer = compute_reward_policy(summary, stdp_layers=STDP3_LAYERS)

    # IMPORTANT: include cycle=<executed cycle>
    run.send_reward(global_reward=float(global_r), by_layer=by_layer, cycle=int(cyc))

def _apply_reset_pose(bus: Any) -> None:
    """
    Reset the robot goal to init pose (if provided) AFTER completing a cycle.
    This does NOT clear buffers; it only commands a safe home pose.
    """
    global goal_q
    try:
        q_now = read_q_raw(bus)
    except Exception:
        q_now = last_q_raw[:] if last_q_raw else target_q_raw[:]

    # Prefer explicit init pose; otherwise "reset" means hold current pose.
    q_home = init_q_raw[:] if init_q_raw is not None else q_now[:]

    # Clamp into cage
    for i in range(len(motor_names)):
        q_home[i] = clamp(float(q_home[i]), float(cage_lo[i]), float(cage_hi[i]))

    goal_q = q_home[:]
    try:
        write_goal_raw(bus, goal_q)
        log.info("[reset] applied reset pose AFTER cycle completion")
    except Exception as e:
        log.warning(f"[reset] failed to apply reset pose: {e}")

def emit_need_received(run_id: str, cycle: int, nid: str, kind: str) -> None:
    try:
        run.socket.emit(
            "io:need_received",
            {
                "runId": str(run_id),
                "cycle": int(cycle),
                "id": str(nid),
                "kind": str(kind),
                "ts": float(now()),
            },
            namespace=run.namespace,
        )
    except Exception as e:
        log.warning(f"[io:need_received] emit failed: {e}")

def on_io_need(msg: Dict[str, Any]) -> None:
    """
    msg: { runId, cycle, needs:[{id,kind}], constants:{...} }
    """
    run_id = str(msg.get("runId") or run.run_id)
    cycle = int(msg.get("cycle", 0))
    needs = msg.get("needs") or []

    global server_constants
    server_constants = msg.get("constants") or server_constants

    # Always publish robot state using cached pose to avoid concurrent serial reads.
    try:
        if last_q_raw is not None:
            run.socket.emit(
                "robot:state",
                {
                    "runId": run_id,
                    "cycle": cycle,
                    "state": {
                        "q_raw": list(last_q_raw),
                        "err": float(last_err if last_err is not None else 0.0),
                        "dt": float(DT),
                        "ts": float(last_q_ts),
                    },
                },
                namespace=run.namespace,
            )

    except Exception as e:
        log.warning(f"[robot:state] failed: {e}")

    for need in needs:
        try:
            nid = str(need.get("id"))
            kind = str(need.get("kind"))
            # receipt ACK
            emit_need_received(run_id, cycle, nid, kind)

            if kind == "Image":
                # Camera is ONLY for input signals.
                # We send a jpeg only when requested.
                jpg, meta = get_rgb_jpeg_bytes()
                run.send_input_chunk(
                    input_id=nid,
                    kind="Image",
                    seq=cycle,
                    t=now(),
                    fmt="jpeg",
                    meta=meta,
                    data=jpg,
                )
            
            elif kind in ("Proprioception"):
                _send_proprioception_for_need(need, cycle)

            else:
                # This controller is “actuation + reward/feedback only” for now.
                # If your graph asks for Image/Depth/etc, add those later.
                log.warning(f"[io:need] ignoring need kind={kind} id={nid}")

        except Exception as e:
            log.warning(f"[io:need] failed need={need}: {e}")

def _resolve_joint_index_from_need(need: Dict[str, Any]) -> Optional[int]:
    # primary: server-provided mapping key
    out_from = need.get("fromOutput")
    if out_from:
        s = str(out_from).strip()
        if s in motor_names:
            return motor_names.index(s)

    # fallback: sometimes id itself is the motor name
    nid = str(need.get("id") or "").strip()
    if nid in motor_names:
        return motor_names.index(nid)

    # fallback: allow prefix formats "Prop:shoulder_pan"
    if ":" in nid:
        tail = nid.split(":")[-1].strip()
        if tail in motor_names:
            return motor_names.index(tail)

    return None

def _send_proprioception_for_need(need: Dict[str, Any], cycle: int) -> None:
    nid = str(need.get("id"))

    idx = _resolve_joint_index_from_need(need)
    if idx is None:
        log.warning(f"[io:need] proprio cannot map motor (id={nid}, fromOutput={need.get('fromOutput')})")
        return
    if last_q_raw is None:
        log.warning(f"[io:need] proprio requested but last_q_raw is None (id={nid})")
        return

    raw_tick = float(last_q_raw[idx])

    # prev tick
    prev_tick = raw_tick
    if prev_q_raw is not None and idx < len(prev_q_raw):
        prev_tick = float(prev_q_raw[idx])

    # velocity (ticks/sec) from cached timestamps
    vel = 0.0
    try:
        dt = float(last_q_ts) - float(prev_q_ts)
        if dt > 1e-6:
            vel = (raw_tick - prev_tick) / dt
    except Exception:
        vel = 0.0

    meta = {
        "cycle": int(cycle),
        "joint": str(motor_names[idx]),
        "fromOutput": str(need.get("fromOutput") or ""),
        "rawTick": raw_tick,
        "prevRawTick": prev_tick,
        "velTickPerS": vel,
        "lo": float(cage_lo[idx]),
        "hi": float(cage_hi[idx]),
        "span": float(cage_span[idx]),
        "ts_now": float(last_q_ts),
        "ts_prev": float(prev_q_ts),
    }

    # Use SDK helper (no reinvention)
    run.send_proprioception_ticks(
        input_id=nid,                 # input id from contract
        raw_tick=raw_tick,
        prev_raw_tick=prev_tick,
        vel_tick_s=vel,
        cycle=int(cycle),
        seq=int(cycle),
        t=now(),
        lo=float(cage_lo[idx]),
        hi=float(cage_hi[idx]),
        meta=meta,
    )

def on_cycle_update(msg: Dict[str, Any]) -> None:
    """
    msg: { runId, cycle, outputs: [...] }
    outputs: list of rows: {"t","id","bits"} OR [t,id,bits]
    """
    outs = msg.get("outputs") or []
    cycle = int(msg.get("cycle", 0) or 0)
    run_id = str(msg.get("runId") or getattr(run, "run_id", ""))

    if not isinstance(outs, list) or not outs:
        log.warning(f"[cycle:update] runId={run_id} cycle={cycle} outs=EMPTY")
        return
    
    global last_cycle_update_ts
    last_cycle_update_ts = time.time()

    # ---- SUMMARY ----
    ids = []
    t_min = None
    t_max = None
    lens = []
    ones = []

    sample_rows = []
    for row in outs[-10:]:
        t, oid, bits = _row_to_tuple(row)
        sample_rows.append((t, oid, (len(bits) if isinstance(bits, list) else None), _bitcount(bits) if isinstance(bits, list) else None))

    for row in outs:
        t, oid, bits = _row_to_tuple(row)
        if oid is not None:
            ids.append(str(oid))
        if isinstance(t, (int, float)):
            t = int(t)
            t_min = t if t_min is None else min(t_min, t)
            t_max = t if t_max is None else max(t_max, t)
        if isinstance(bits, list):
            lens.append(len(bits))
            bc = _bitcount(bits)
            if bc is not None:
                ones.append(bc)

    uniq_ids = sorted(set(ids))

    # --- compute summary scalars for logging (avoid f-string conditional format bugs) ---
    if lens:
        avg_bits_len = f"{(sum(lens) / len(lens)):.1f}"
    else:
        avg_bits_len = "NA"

    if ones:
        avg_ones = f"{(sum(ones) / len(ones)):.1f}"
    else:
        avg_ones = "NA"
    log.info(
        f"[cycle:update] runId={run_id} cycle={cycle} outs={len(outs)} "
        f"t=[{t_min},{t_max}] uniq_ids={uniq_ids[:20]}{'…' if len(uniq_ids) > 20 else ''} "
        f"bits_len≈{avg_bits_len} "
        f"ones≈{avg_ones}"
    )
    log.info(f"[cycle:update] last10={sample_rows}")

    # remember how many timesteps the server gave us for this cycle.
    # We’ll use this to emit cycle_done even if the queue logic gets weird.
    if isinstance(t_max, int) and t_max >= 0:
        expected_T_by_cycle[cycle] = int(t_max) + 1

    # ---- KEEP YOUR EXISTING PIPELINE ----
    try:
        cmds = decode_stream_rows(outs, mapping)
        if not cmds:
            log.warning(f"[cycle:update] cycle={cycle} decode_stream_rows -> 0 cmds (mapping mismatch likely)")
            return

        total = _sum_cycle_deltas(cmds)

        # store ONE intent per cycle
        cycle_buffers[cycle] = {
            "cycle": cycle,
            "total_deltas": total,
        }


        global latest_cycle_seen
        latest_cycle_seen = max(latest_cycle_seen, cycle)

        # Enforce buffer invariant
        if executing_cycle is not None:
            _drop_cycles_older_than(int(executing_cycle))
        else:
            # If nothing executing yet, keep only newest MAX_AHEAD cycles overall
            if len(cycle_buffers) > int(MAX_AHEAD):
                keep = set(sorted(cycle_buffers.keys())[-int(MAX_AHEAD):])
                for c in list(cycle_buffers.keys()):
                    if c not in keep:
                        del cycle_buffers[c]

        # If we're idle, arm the next cycle intent (cycle_buffers -> active_intent)
        _arm_next_cycle_if_idle()
        log.info(f"[intent] buffered cycle={cycle} total_deltas={{{', '.join(f'{k}:{v:.3f}' for k,v in list(total.items())[:6])}}}…")


    except Exception as e:
        log.warning(f"[cycle:update] decode failed: {e}")

def _cycle_done_threshold_steps(cyc: int) -> int:
    """
    The server should consider a cycle "done" when we've executed enough steps
    to cover the feedback raster length (FeedbackT).

    Why:
      - Feedback is sent as deviation_f32 with meta {"T": FeedbackT, ...}
      - So we only want to finalize a cycle once we have at least that many steps
        to build a meaningful deviation signal.

    Rule:
      - If CONTRACT_FEEDBACK_T > 0, use it.
      - Else fall back to expected_T_by_cycle (derived from cycle:update t_max).
      - If neither exists, return 0 (can't decide safely).
    """
    try:
        fbT = int(CONTRACT_FEEDBACK_T or 0)
    except Exception:
        fbT = 0
    if fbT > 0:
        return fbT
    try:
        return int(expected_T_by_cycle.get(int(cyc), 0) or 0)
    except Exception:
        return 0

def start_ab_after_robot_connected() -> None:
    POLICIES_DIR = os.path.join(HERE, "policies")
    global client, run
    client = ABClient(base_url=BASE_URL, api_key=API_KEY)
    # 1) Ensure policies exist + refresh machine-owned contract files
    client.sync_policies(PROJECT_ID, policies_dir=POLICIES_DIR)
    run = client.start(PROJECT_ID)

    # after run is created
    sio = run.socket

    @sio.event(namespace=run.namespace)
    def connect():
        log.info(f"[socket] connected namespace={run.namespace}")

    @sio.event(namespace=run.namespace)
    def disconnect():
        log.warning(f"[socket] DISCONNECTED namespace={run.namespace}")

    @sio.on("connect_error", namespace=run.namespace)
    def _connect_error(err):
        log.warning(f"[socket] connect_error: {err}")

    @sio.on("error", namespace=run.namespace)
    def _error(err):
        log.warning(f"[socket] error: {err}")

    # Engine.IO layer (very useful)
    try:
        eio = sio.eio
        @eio.on("disconnect")
        def _eio_disconnect():
            log.warning("[eio] disconnect (engine.io transport down)")
    except Exception as e:
        log.info(f"[debug] cannot attach eio handlers: {e}")


    log.info(f"[AB] started run: runId={run.run_id} base={BASE_URL} namespace={run.namespace}")
    run.on_io_need(on_io_need)
    run.on_cycle_update(on_cycle_update)

# ----------------------------
# Shutdown / safety
# ----------------------------
_stop_sent = False

def stop_server() -> None:
    global _stop_sent
    if _stop_sent:
        return
    _stop_sent = True
    try:
        log.info("[AB] stopping run...")
        if client is not None and run is not None:
            client.stop(PROJECT_ID, run_id=run.run_id)
    except Exception as e:
        log.warning(f"[AB] stop failed: {e}")

def handle_sig(signum, frame) -> None:
    raise KeyboardInterrupt()

atexit.register(stop_server)
signal.signal(signal.SIGINT, handle_sig)
signal.signal(signal.SIGTERM, handle_sig)

# ----------------------------
# Main
# ----------------------------
robot = None
bus = None

try:
    robot, bus = connect_robot()
    log.info("[SO101] connected")

    # One-time debug: see what registers the bus actually supports
    debug_bus_registers(bus)

    # start AB only after bus is valid
    start_ab_after_robot_connected()

    # Enable torque so the robot holds position while we command it
    set_mode_position(bus, motor_names)
    
    set_torque(bus, True)
    
    log.info("[SO101] torque enabled (do NOT move by hand)")

    if SO101_SET_SPEED_ON_START:
        write_speed_raw(bus, SO101_SPEED_VALUE)

    # NEW: verify torque actually enabled
    tq = read_torque(bus)
    if any(v == 0 for v in tq):
        bad = [(motor_names[i], tq[i]) for i in range(len(motor_names)) if tq[i] == 0]
        log.warning(f"[SO101] torque readback indicates NOT enabled on: {bad}")
    else:
        log.info(f"[SO101] torque readback OK: {tq}")

    # Seed goal from current pose
    q0 = read_q_raw(bus)
    last_q_raw = q0[:]
    last_err = objective_error(q0); last_q_ts = time.time()

    # Reset-mode (startup only): clear local state + optionally home to init pose
    if SO101_DO_RESET:
        log.warning("[reset] SO101_DO_RESET=1 -> clearing local execution state + homing to init pose (startup only)")
        _clear_local_execution_state()

        q_now = read_q_raw(bus)
        q_home = init_q_raw[:] if init_q_raw is not None else q_now[:]
        # clamp home pose into cage
        for i in range(len(motor_names)):
            q_home[i] = clamp(float(q_home[i]), float(cage_lo[i]), float(cage_hi[i]))
        goal_q = q_home[:]
        write_goal_raw(bus, goal_q)

        # refresh cached pose after issuing home command
        last_q_raw = q_now[:]
        last_err = objective_error(q_now); last_q_ts = time.time()
    else:
        goal_q = q0[:]
        write_goal_raw(bus, goal_q)
 

    next_tick = time.perf_counter()

    # ---- live stuck-motor detector ----
    # If Goal changes but Present doesn't for N steps, log exactly which joints are not responding.
    stuck_counts = {n: 0 for n in motor_names}
    prev_goal = read_raw(bus, DATA_GOAL)
    prev_pos  = read_raw(bus, DATA_PRESENT)

    while True:
        # Watchdog: if we haven’t seen a new cycle:update for a while, say it loudly.
        if last_cycle_update_ts and (time.time() - last_cycle_update_ts) > 3.0:
            log.warning(f"[watchdog] no cycle:update for {time.time() - last_cycle_update_ts:.1f}s (server likely waiting on cycle_done)")
            # don't spam every tick
            last_cycle_update_ts = time.time()
        

        # pace loop (deadline-based)
        now_perf = time.perf_counter()
        sleep_s = next_tick - now_perf
        if sleep_s > 0:
           time.sleep(sleep_s)
        else:
            # if we fell behind, don't "compress" time; just reset the schedule
            next_tick = now_perf
        next_tick += DT


        # Always read once per tick (serialized) and update cache for io:need
        q_now = read_q_raw(bus)
        if last_q_raw is not None:
            prev_q_raw = last_q_raw[:]
            prev_q_ts = float(last_q_ts)
        last_q_raw = q_now[:]
        last_err = objective_error(q_now)
        last_q_ts = time.time()

        # If idle, arm next cycle intent (cycle_buffers -> active_intent)
        _arm_next_cycle_if_idle()

        if active_intent is not None:
            # Track per-cycle start/end poses for reward policy
            if executing_cycle is not None:
                cyc = int(executing_cycle)
                cycle_end_q_raw[cyc] = q_now[:]
                if cyc not in cycle_start_q_raw:
                    cycle_start_q_raw[cyc] = q_now[:]

            # Use a stable timestep index for err/credit assignment
            ti = int(executed_steps_in_cycle)

            # Record per-joint error history (for per-feedback deviation computation)
            try:
                per_joint = _per_joint_errors(q_now)
                for jname, e in per_joint.items():
                    if jname not in err_by_joint_by_t:
                        err_by_joint_by_t[jname] = {}
                    err_by_joint_by_t[jname][ti] = float(e)
            except Exception:
                pass

            # One joint per timestep: timestep i moves motor_names[i]
            i = int(executed_steps_in_cycle)
            jname = motor_names[i % len(motor_names)]

            dv = float(active_intent.get(jname, 0.0))

            # Apply ONLY this joint this tick
            step_deltas = {jname: dv}

            # Consume this joint's remaining delta (so it won't be applied again this cycle)
            active_intent[jname] = 0.0

            log.info(f"[step] cyc={executing_cycle} i={i} joint={jname} dv={dv:.3f}")

            # Apply & command
            q_goal = apply_safe_deltas(q_now, step_deltas)
            write_goal_raw(bus, q_goal)

            # ---- STOPPER: wait until this joint is close to goal (or timeout) ----
            # Only wait if we actually asked for a meaningful move.
            if abs(float(dv)) >= float(SO101_WAIT_MIN_DV_TICKS):
                j = motor_names.index(jname)
                reached = wait_until_joint_reached(bus, j)
                if not reached:
                    # Don't crash; just warn so you can tune eps/timeout.
                    try:
                        g = read_raw(bus, DATA_GOAL)
                        p = read_raw(bus, DATA_PRESENT)
                        log.warning(
                            f"[wait] timeout cyc={executing_cycle} i={executed_steps_in_cycle} joint={jname} "
                            f"goal={g[j]} present={p[j]} |goal-present|={abs(g[j]-p[j])} "
                            f"(eps={SO101_WAIT_EPS_TICKS}, timeout_s={SO101_WAIT_TIMEOUT_S})"
                        )
                    except Exception:
                        log.warning(
                            f"[wait] timeout cyc={executing_cycle} i={executed_steps_in_cycle} joint={jname} "
                            f"(eps={SO101_WAIT_EPS_TICKS}, timeout_s={SO101_WAIT_TIMEOUT_S})"
                        )

            # --- DEBUG: confirm bus write + motor response for the ONE joint we touched ---
            j = motor_names.index(jname)
            try:
                g = read_raw(bus, DATA_GOAL)      # goal ticks (all joints)
                p = read_raw(bus, DATA_PRESENT)   # present ticks (all joints)
                log.info(
                    f"[check] cyc={executing_cycle} i={executed_steps_in_cycle} joint={jname} "
                    f"goal={g[j]} present={p[j]} |goal-present|={abs(g[j]-p[j])}"
                )
            except Exception as e:
                log.warning(f"[check] readback failed: {e}")

            # Record objective + credit assignment aligned to ti
            applied_deltas_by_t[ti] = {str(k): float(v) for k, v in step_deltas.items()}
            err_by_t[ti] = float(last_err if last_err is not None else objective_error(q_now))

            executed_any_for_cycle = True
            executed_steps_in_cycle += 1
            intent_steps_left -= 1

            # If intent completed, finalize cycle exactly once
            if intent_steps_left <= 0:
                active_intent = None

                if executing_cycle is not None:
                    cyc = int(executing_cycle)
                    if cyc not in cycle_done_sent:
                        try:
                            run.socket.emit(
                                "robot:cycle_done",
                                {"runId": run.run_id, "cycle": cyc},
                                namespace=run.namespace,
                            )
                            cycle_done_sent.add(cyc)
                            log.info(f"[robot] cycle_done emitted for cycle={cyc} (intent completed)")

                            send_learning_for_cycle(cyc)
                            log.info(f"[learn] pushed reward+feedback for cycle={cyc}")

                            _finalize_cycle_local(cyc)
                            _drop_cycles_older_than(cyc)

                        except Exception as e:
                            log.warning(f"[robot] cycle_done / learn failed for cycle={cyc}: {e}")

            if DEBUG_MOTION_READBACK:
                time.sleep(0.1)  # 20ms lets you see Present_Position change
                goal_after = read_raw(bus, "Goal_Position")
                pos_after  = read_raw(bus, "Present_Position")
                log.info(f"[check] goal_after={goal_after} pos_after={pos_after}")

                # --- stuck analysis (per joint) ---
                # Condition: goal changed meaningfully, but position didn't.
                for i, name in enumerate(motor_names):
                    dg = abs(float(goal_after[i]) - float(prev_goal[i]))
                    dp = abs(float(pos_after[i])  - float(prev_pos[i]))
                    if dg >= STUCK_GOAL_EPS_TICKS and dp <= STUCK_POS_EPS_TICKS:
                        stuck_counts[name] = stuck_counts.get(name, 0) + 1
                    else:
                        stuck_counts[name] = 0

                prev_goal = goal_after[:]
                prev_pos  = pos_after[:]

                offenders = [n for n, c in stuck_counts.items() if c >= STUCK_STEPS_TO_WARN]
                if offenders:
                    # Show a compact per-joint snapshot: (name, goal, pos, |goal-pos|)
                    snap = []
                    for i, n in enumerate(motor_names):
                        if n in offenders:
                            snap.append((n, int(goal_after[i]), int(pos_after[i]), int(abs(goal_after[i] - pos_after[i]))))
                    log.warning(
                        "[actuation] ⚠️ Goal is changing but position is not following for "
                        f"{STUCK_STEPS_TO_WARN} steps on: {offenders}. "
                        f"snap={snap}. "
                        "This usually means: wrong motor name->ID mapping, insufficient power, "
                        "a joint hitting a hard stop, speed/torque/current limits, or the bus is writing to a different motor."
                    )
                    # don't spam: reset counts after warning
                    for n in offenders:
                        stuck_counts[n] = 0

            if (active_intent is None) and SO101_DO_RESET and (executing_cycle is not None):
                cyc = int(executing_cycle)
                # Only reset after cycle completion is acknowledged (cycle_done emitted),
                # and only once per cycle.
                if (cyc in cycle_done_sent) and (cyc not in reset_done_for_cycle):
                    _apply_reset_pose(bus)
                    reset_done_for_cycle.add(cyc)

        else:
            # no commands: still record objective occasionally so reward isn’t empty
            q_now = read_q_raw(bus)
            # cache prev for proprio (io:need handler must not read bus)
            if last_q_raw is not None:
                prev_q_raw = last_q_raw[:]
                prev_q_ts = float(last_q_ts)
            last_q_raw = q_now[:]
            last_err = objective_error(q_now)
            last_q_ts = time.time()

            # If we're idle but still have an executing_cycle, keep end pose fresh
            # (helps if server asks Reward before next window executes).
            if executing_cycle is not None:
                cyc = int(executing_cycle)
                cycle_end_q_raw[cyc] = q_now[:]
                if cyc not in cycle_start_q_raw:
                    cycle_start_q_raw[cyc] = q_now[:]


except KeyboardInterrupt:
    pass
except Exception:
    log.error("Controller crashed:\n" + traceback.format_exc())
finally:
    try:
        if bus is not None:
            # Disable torque so you can safely reposition / not fight the motors
            set_torque(bus, False)
            log.info("[SO101] torque disabled (hold the arm so it doesn’t drop)")
    except Exception:
        pass

    try:
        if robot is not None:
            robot.disconnect()
            log.info("[SO101] disconnected")
    except Exception:
        pass

    stop_server()
    try:
        if run is not None:
            run.close()
    except TypeError as e:
        # older SDK versions may not accept namespace kwarg internally
        log.warning(f"[AB] run.close() skipped due to SDK signature mismatch: {e}")
    except Exception:
        pass
