# tools/build_safety_cage.py
#
# One-time safety limits recorder for SO-101 follower.
# Records per-joint SAFE MIN / SAFE MAX in RAW ticks (Present_Position).
#
# Run:
#   source .venv/bin/activate
#   python tools/build_safety_cage.py
#
# Env (optional):
#   SO101_PORT=/dev/ttyACM0
#   SO101_ID=my_awesome_follower_arm
#   CAGE_OUT=targets/safety_cage.json
#
from __future__ import annotations

import os
import sys
import json
import time
import traceback
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List


PORT = os.environ.get("SO101_PORT", "/dev/ttyACM0")
RID = os.environ.get("SO101_ID", "my_awesome_follower_arm")
OUT = os.environ.get("CAGE_OUT", "targets/safety_cage.json")

DATA_NAME_PRESENT = "Present_Position"


def read_q_raw(bus: Any, motor_names: List[str]) -> List[float]:
    # LeRobot Feetech bus supports: sync_read(data_name, motors, normalize=...)
    pos_map = bus.sync_read(DATA_NAME_PRESENT, motor_names, normalize=False)
    out: List[float] = []
    for name in motor_names:
        v = pos_map.get(name, None)
        if v is None:
            raise RuntimeError(f"Missing '{name}' in sync_read result. Got keys={list(pos_map.keys())[:20]}")
        out.append(float(v))
    return out


def main() -> int:
    print("python =", sys.executable)
    print("PORT   =", PORT)
    print("RID    =", RID)

    r = None
    torque_disabled = False

    try:
        # LeRobot class (you already confirmed this works on your machine)
        from lerobot.robots.so_follower.so_follower import SOFollower

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
        print("Connected.")

        bus = getattr(r, "bus", None)
        if bus is None:
            raise RuntimeError("Robot has no .bus")

        motors_dict = getattr(bus, "motors", None)
        if not isinstance(motors_dict, dict) or not motors_dict:
            raise RuntimeError("bus.motors dict not found; cannot list motor names safely.")

        motor_names = list(motors_dict.keys())
        print("motor_names =", motor_names)

        # IMPORTANT: disable torque so you can move by hand
        print("\nDisabling torque so you can move joints by hand...")
        bus.disable_torque()
        torque_disabled = True
        print("Torque disabled.\n")

        limits: Dict[str, Dict[str, float]] = {}

        print("We will record SAFE MIN then SAFE MAX for each joint.")
        print("Do NOT force the joint into hard stops.\n")

        for j, name in enumerate(motor_names):
            input(f"[{j+1}/{len(motor_names)}] Move '{name}' to SAFE MIN, then press Enter...")
            q_min = read_q_raw(bus, motor_names)
            vmin = q_min[j]
            print(f"  min[{name}] = {vmin:.1f}")

            input(f"[{j+1}/{len(motor_names)}] Move '{name}' to SAFE MAX, then press Enter...")
            q_max = read_q_raw(bus, motor_names)
            vmax = q_max[j]
            print(f"  max[{name}] = {vmax:.1f}")

            lo = float(min(vmin, vmax))
            hi = float(max(vmin, vmax))
            limits[name] = {"min": lo, "max": hi}

        out_path = Path(OUT)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        payload: Dict[str, Any] = {
            "robot": {
                "type": "so101_follower",
                "id": RID,
                "port": PORT,
                "data_name_present": DATA_NAME_PRESENT,
                "motor_names": motor_names,
            },
            "created_at_unix": time.time(),
            "limits_raw": limits,
            # Controller should clamp using: [min+buffer, max-buffer]
            "buffer_ticks": 25.0,
        }

        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\n[SAVED] {out_path}")

        print("\nNext (controller logic):")
        print("- Load limits_raw + buffer_ticks")
        print("- Clamp every commanded target per joint:")
        print("    target = min(max(target, min+buffer), max-buffer)")
        print("- Also add a per-step rate limit (max ticks per loop)")

        # Re-enable torque at end so robot doesn't stay floppy (optional)
        print("\nRe-enabling torque...")
        bus.enable_torque()
        torque_disabled = False
        print("Torque enabled.")

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
                # If user interrupted while torque disabled, it's safer to leave torque OFF
                # (prevents sudden moves). But if you prefer always-on, flip this logic.
                if torque_disabled:
                    print("\nLeaving torque disabled due to interruption/error (safer).")
                r.disconnect()
                print("Disconnected.")
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
