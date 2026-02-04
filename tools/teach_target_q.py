# tools/teach_target_q.py
#
# Teach / record a target joint pose for LeRobot SO-101 follower arm.
#
# What it does:
# - Connects to the SO-101 follower via LeRobot
# - Finds baudrate + motor registry
# - Reads RAW joint positions (Present_Position, normalize=False)
# - Lets you manually position the robot, then press Enter to record a target
# - Saves to JSON (targets/target_touch_mat.json by default)
#
# Run:
#   source .venv/bin/activate
#   python tools/teach_target_q.py
#
# Optional env:
#   SO101_PORT=/dev/ttyACM0
#   SO101_ID=my_awesome_follower_arm
#   TARGET_OUT=targets/target_touch_mat.json
#
from __future__ import annotations

import os
import sys
import json
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Optional, Tuple


# ----------------------------
# Config
# ----------------------------
PORT = os.environ.get("SO101_PORT", "/dev/ttyACM0")
RID = os.environ.get("SO101_ID", "my_awesome_follower_arm")
OUT = os.environ.get("TARGET_OUT", "targets/target_touch_mat.json")

# DataName for Feetech present position in LeRobot.
DATA_NAME_PRESENT = "Present_Position"

# If scan_port fails, fallback baud candidates:
BAUD_CANDIDATES = [1_000_000, 500_000, 250_000, 115_200]


@dataclass
class RobotCtx:
    robot: object
    bus: object
    best_baud: int
    motor_names: List[str]


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _pick_best_baud_from_scan(scan: Dict[int, List[int]]) -> Optional[int]:
    # scan_port returns dict like {baud: [ids...], ...}
    if not scan:
        return None
    # pick baud with the most ids; tie -> higher baud
    items = sorted(scan.items(), key=lambda kv: (len(kv[1] or []), kv[0]), reverse=True)
    return int(items[0][0])


def _connect_robot() -> RobotCtx:
    print("python =", sys.executable)
    print("PORT   =", PORT)
    print("RID    =", RID)

    # NOTE: class name in your install prints as SOFollower
    try:
        from lerobot.robots.so_follower.so_follower import SOFollower
    except Exception:
        print("\n[FATAL] Could not import lerobot SOFollower from lerobot.robots.so_follower.so_follower")
        traceback.print_exc()
        raise

    # Use SimpleNamespace because LeRobot expects attribute access (config.id, config.port, etc.)
    cfg = SimpleNamespace(
        calibration_dir=None,               # default cache location
        cameras={},                         # none for this tool
        disable_torque_on_disconnect=True,
        id=RID,
        max_relative_target=None,
        port=PORT,
        use_degrees=False,
    )

    r = SOFollower(cfg)

    # Connect without re-calibrating
    r.connect(calibrate=False)

    bus = getattr(r, "bus", None)
    if bus is None:
        raise RuntimeError("Robot has no .bus attribute; cannot read motors.")

    # Print helpful signatures
    try:
        import inspect

        print("Robot class:", type(r))
        print("Bus class:", type(bus))
        if hasattr(bus, "scan_port"):
            print("bus.scan_port signature:", str(inspect.signature(bus.scan_port)))
        if hasattr(bus, "sync_read"):
            print("bus.sync_read signature:", str(inspect.signature(bus.sync_read)))
        if hasattr(bus, "get_baudrate"):
            print("bus.get_baudrate signature:", str(inspect.signature(bus.get_baudrate)))
        if hasattr(bus, "set_baudrate"):
            print("bus.set_baudrate signature:", str(inspect.signature(bus.set_baudrate)))
    except Exception:
        pass

    # 1) Determine best baud
    best_baud: Optional[int] = None
    scan = {}
    if hasattr(bus, "scan_port"):
        try:
            scan = bus.scan_port(PORT)
            # Some versions return {baud: [ids]} and also print progress bars.
            best_baud = _pick_best_baud_from_scan(scan)
        except Exception as e:
            print("[warn] bus.scan_port failed:", repr(e))
            scan = {}

    if best_baud is None:
        # Fallback: try each candidate by setting baud and pinging a known id
        best_baud = 1_000_000
        if hasattr(bus, "set_baudrate") and hasattr(bus, "ping"):
            for b in BAUD_CANDIDATES:
                try:
                    bus.set_baudrate(int(b))
                    # try ping motor 1 quickly; raise_on_error False
                    res = bus.ping(1, raise_on_error=False)
                    if res is not None:
                        best_baud = int(b)
                        break
                except Exception:
                    continue

    # Set it explicitly (safe even if already set)
    if hasattr(bus, "set_baudrate"):
        try:
            bus.set_baudrate(int(best_baud))
        except Exception:
            pass

    # 2) Motor names from registry, if available
    motor_names: List[str] = []
    # You already saw: bus.motors is a dict with keys = names
    motors_dict = getattr(bus, "motors", None)
    if isinstance(motors_dict, dict) and motors_dict:
        motor_names = list(motors_dict.keys())
    else:
        # conservative fallback for SO-101 based on what you saw earlier
        motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    print("\nbest_baud =", best_baud)
    print("motor_names =", motor_names)

    return RobotCtx(robot=r, bus=bus, best_baud=int(best_baud), motor_names=motor_names)


def _read_q_raw(bus: object, motor_names: List[str]) -> List[float]:
    if not hasattr(bus, "sync_read"):
        raise RuntimeError("Bus has no sync_read; cannot read positions.")

    # normalize=False ensures it works even if calibration isn't registered to this bus instance.
    pos_map = bus.sync_read(DATA_NAME_PRESENT, motor_names, normalize=False)

    # pos_map expected: dict[name -> value]
    q = []
    for name in motor_names:
        v = pos_map.get(name, None)
        if v is None:
            raise RuntimeError(f"Missing '{name}' in sync_read result. keys={list(pos_map.keys())[:20]}")
        q.append(float(v))
    return q


def main() -> int:
    r = None
    try:
        ctx = _connect_robot()
        r = ctx.robot
        bus = ctx.bus
        motor_names = ctx.motor_names

        out_path = Path(OUT)
        _ensure_parent(out_path)

        print("\n--- Teach mode ---")
        print("Move the robot by hand to the desired pose (e.g., touching the mat).")
        print("Then press Enter to CAPTURE & SAVE.")
        print("Type 'p' + Enter to PRINT current q without saving.")
        print("Type 'q' + Enter to quit.\n")

        while True:
            cmd = input("> ").strip().lower()

            if cmd == "q":
                print("Exiting.")
                break

            if cmd == "p":
                q_now = _read_q_raw(bus, motor_names)
                print("q_raw =", q_now)
                continue

            # Default: capture + save
            q_now = _read_q_raw(bus, motor_names)
            payload = {
                "robot": {
                    "type": "so101_follower",
                    "id": RID,
                    "port": PORT,
                    "baud": ctx.best_baud,
                    "data_name_present": DATA_NAME_PRESENT,
                    "motor_names": motor_names,
                },
                "target": {
                    "q_raw": q_now,
                    "captured_at_unix": time.time(),
                },
            }

            out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            print(f"\n[SAVED] {out_path}\n")

            print("COPY THIS:")
            print("SO101_MOTORS=" + ",".join(motor_names))
            print("TARGET_Q_RAW=" + ",".join(f"{x:.6f}" for x in q_now))
            print("")

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130

    except Exception:
        print("\nERROR")
        traceback.print_exc()
        return 1

    finally:
        if r is not None:
            try:
                r.disconnect()
                print("Disconnected.")
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
