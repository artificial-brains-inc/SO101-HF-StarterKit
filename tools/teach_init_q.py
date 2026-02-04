# tools/teach_init_q.py
#
# Teach / record an INIT (home) joint pose for LeRobot SO-101 follower arm.
#
# What it does:
# - Connects to the SO-101 follower via LeRobot
# - Reads RAW joint positions (Present_Position, normalize=False)
# - Lets you manually position the robot, then press Enter to record init
# - Saves to JSON (targets/init_pose.json by default)
#
# Run:
#   source .venv/bin/activate
#   python tools/teach_init_q.py
#
# Optional env:
#   SO101_PORT=/dev/ttyACM0
#   SO101_ID=my_awesome_follower_arm
#   INIT_OUT=targets/init_pose.json
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
from typing import Dict, List, Optional

PORT = os.environ.get("SO101_PORT", "/dev/ttyACM0")
RID  = os.environ.get("SO101_ID", "my_awesome_follower_arm")
OUT  = os.environ.get("INIT_OUT", "targets/init_pose.json")

DATA_NAME_PRESENT = "Present_Position"
DATA_NAME_TORQUE  = "Torque_Enable"

@dataclass
class RobotCtx:
    robot: object
    bus: object
    motor_names: List[str]

def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

def _connect_robot() -> RobotCtx:
    print("python =", sys.executable)
    print("PORT   =", PORT)
    print("RID    =", RID)
    try:
        from lerobot.robots.so_follower.so_follower import SOFollower
    except Exception:
        print("\n[FATAL] Could not import lerobot SOFollower")
        traceback.print_exc()
        raise

    cfg = SimpleNamespace(
        calibration_dir=None,
        cameras={},
        disable_torque_on_disconnect=True,
        id=RID,
        max_relative_target=None,
        port=PORT,
        use_degrees=False,
    )
    r = SOFollower(cfg)
    r.connect(calibrate=False)

    bus = getattr(r, "bus", None)
    if bus is None:
        raise RuntimeError("Robot has no .bus attribute; cannot read motors.")

    motor_names: List[str] = []
    motors_dict = getattr(bus, "motors", None)
    if isinstance(motors_dict, dict) and motors_dict:
        motor_names = list(motors_dict.keys())
    else:
        motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    
    # ---- IMPORTANT ----
    # Disable torque so the robot can be moved by hand.
    # This is teach mode: no holding, no control.
    try:
        # Preferred: sync_write
        bus.sync_write(
            DATA_NAME_TORQUE,
            {name: 0 for name in motor_names},
            normalize=False,
        )
    except Exception:
        # Fallback: per-motor write
        for name in motor_names:
            try:
                bus.write(DATA_NAME_TORQUE, name, 0, normalize=False)
            except Exception:
                pass

    print("motor_names =", motor_names)
    return RobotCtx(robot=r, bus=bus, motor_names=motor_names)

def _read_q_raw(bus: object, motor_names: List[str]) -> List[float]:
    if not hasattr(bus, "sync_read"):
        raise RuntimeError("Bus has no sync_read; cannot read positions.")
    pos_map = bus.sync_read(DATA_NAME_PRESENT, motor_names, normalize=False)
    q: List[float] = []
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

        print("\n--- Teach INIT pose ---")
        print("Move the robot by hand to the INIT/HOME pose.")
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

            q_now = _read_q_raw(bus, motor_names)
            payload: Dict[str, object] = {
                "robot": {
                    "type": "so101_follower",
                    "id": RID,
                    "port": PORT,
                    "data_name_present": DATA_NAME_PRESENT,
                    "motor_names": motor_names,
                },
                "init": {
                    "q_raw": q_now,
                    "captured_at_unix": time.time(),
                },
            }
            out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            print(f"\n[SAVED] {out_path}\n")
            print("COPY THIS:")
            print("SO101_INIT_JSON=" + str(out_path))
            print("INIT_Q_RAW=" + ",".join(f"{x:.6f}" for x in q_now))
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