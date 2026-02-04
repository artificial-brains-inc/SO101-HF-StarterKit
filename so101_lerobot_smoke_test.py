#!/usr/bin/env python3
"""
SO-101 follower smoke test (LeRobot-only).

What it does:
- Connects to SOFollower (Feetech bus)
- Best-effort: forces Position mode (Mode=0) before torque enable
- Best-effort: sets speed (Moving_Speed or Speed) if supported
- Enables torque
- Reads Present_Position + Goal_Position
- Writes a small Goal_Position jog on one joint
- Waits, reads again, reports whether the joint actually moved
- Returns to original goal, reads again

No AB / server involved.
"""

from __future__ import annotations

import os
import time
import argparse
from types import SimpleNamespace
from typing import Any, List


DATA_MODE = "Mode"
DATA_PRESENT = "Present_Position"
DATA_GOAL = "Goal_Position"
DATA_TORQUE = "Torque_Enable"


def _sync_write_any(bus: Any, data_name: str, names: List[str], values: List[int], *, normalize: bool) -> None:
    """
    LeRobot has multiple sync_write signatures depending on version/backend.

    Try, in order:
      A) sync_write(values, names, data_name, normalize=...)
      B) sync_write(data_name, mapping{name: value}, normalize=...)
      C) sync_write(mapping{name: {data_name: value}}, normalize=normalize)  (rare)
    """
    # A) (values, names, data_name)
    try:
        return bus.sync_write(values, names, data_name, normalize=normalize)
    except TypeError:
        pass

    # B) (data_name, mapping)
    mapping = {n: int(values[i]) for i, n in enumerate(names)}
    try:
        return bus.sync_write(data_name, mapping, normalize=normalize)
    except TypeError:
        pass

    # C) mapping-of-mapping fallback
    mapping2 = {n: {data_name: int(values[i])} for i, n in enumerate(names)}
    return bus.sync_write(mapping2, normalize=normalize)


def read_vec(bus: Any, data_name: str, motor_names: List[str]) -> List[int]:
    m = bus.sync_read(data_name, motor_names, normalize=False)
    return [int(m[n]) for n in motor_names]


def set_torque(bus: Any, motor_names: List[str], enabled: bool) -> None:
    val = 1 if enabled else 0
    _sync_write_any(bus, DATA_TORQUE, motor_names, [val] * len(motor_names), normalize=False)


def write_goal(bus: Any, motor_names: List[str], q: List[int]) -> None:
    _sync_write_any(bus, DATA_GOAL, motor_names, [int(x) for x in q], normalize=False)


def set_mode_position(bus: Any, motor_names: List[str]) -> None:
    """
    Best-effort: set position-control mode before torque enable.
    Some Feetech setups won’t actuate unless Mode is set.
    """
    ok = 0
    for n in motor_names:
        try:
            # Many LeRobot buses expose per-motor write(name, value, motor_name, normalize=...)
            bus.write(DATA_MODE, 0, n, normalize=False)
            ok += 1
        except Exception:
            pass

    if ok:
        print(f"[mode] set {DATA_MODE}=0 for {ok}/{len(motor_names)} motors (best-effort)")
    else:
        print(f"[mode] could not set {DATA_MODE} (may be fine depending on bus/model)")


def try_set_speed(bus: Any, motor_names: List[str], speed: int = 300) -> None:
    """
    Optional: some setups default to 0/very low speed. Try common keys.
    """
    for key in ("Moving_Speed", "Speed"):
        try:
            _sync_write_any(bus, key, motor_names, [int(speed)] * len(motor_names), normalize=False)
            print(f"[speed] set {key}={speed} for all motors (best-effort)")
            return
        except Exception:
            continue
    print("[speed] could not set speed (may be fine depending on bus/model)")


def print_mapping(motor_names: List[str]) -> None:
    """
    Prints the mapping that this script uses everywhere:
      index -> motor_name
    And explains how vectors align to that order.
    """
    print("[mapping] index -> motor_name")
    for i, n in enumerate(motor_names):
        print(f"[mapping]   {i}: {n}")
    print("[mapping] Any vector we print (present/goal) is in this exact order.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.getenv("SO101_PORT", "/dev/ttyACM0"))
    ap.add_argument("--id", dest="robot_id", default=os.getenv("SO101_ID", "my_awesome_follower_arm"))
    ap.add_argument("--joint", default=os.getenv("SO101_TEST_JOINT", "shoulder_pan"))
    ap.add_argument("--jog", type=int, default=int(os.getenv("SO101_TEST_JOG_TICKS", "12")))
    ap.add_argument("--wait", type=float, default=float(os.getenv("SO101_TEST_WAIT_S", "0.5")))
    ap.add_argument("--speed", type=int, default=int(os.getenv("SO101_TEST_SPEED", "300")))
    args = ap.parse_args()

    try:
        from lerobot.robots.so_follower.so_follower import SOFollower
    except Exception:
        print("[FATAL] Could not import SOFollower. Is lerobot installed in this venv?")
        raise

    cfg = SimpleNamespace(
        calibration_dir=None,
        cameras={},
        disable_torque_on_disconnect=True,
        id=args.robot_id,
        max_relative_target=None,
        port=args.port,
        use_degrees=False,
    )

    robot = None
    bus = None
    motor_names: List[str] = []

    try:
        robot = SOFollower(cfg)
        robot.connect(calibrate=False)

        bus = getattr(robot, "bus", None)
        if bus is None:
            print("[FATAL] robot.bus is missing")
            return 2

        motor_names = getattr(robot, "motor_names", None) or []
        if not motor_names:
            motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

        if args.joint not in motor_names:
            print(f"[FATAL] joint '{args.joint}' not in motor_names={motor_names}")
            return 2

        j = motor_names.index(args.joint)

        print(f"[info] port={args.port} id={args.robot_id}")
        print(f"[info] motor_names={motor_names}")
        print(f"[info] test_joint={args.joint} idx={j} jog={args.jog} ticks wait={args.wait}s speed={args.speed}")

        # ✅ Print the mapping in-script (what you asked for)
        print_mapping(motor_names)
        print(f"[mapping] test_joint '{args.joint}' -> index {j}")

        # Best-effort mode+speed setup BEFORE torque
        set_mode_position(bus, motor_names)
        try_set_speed(bus, motor_names, speed=args.speed)

        # Enable torque
        set_torque(bus, motor_names, True)
        tq = read_vec(bus, DATA_TORQUE, motor_names)
        print(f"[torque] readback={tq}")

        # Baseline reads
        p0 = read_vec(bus, DATA_PRESENT, motor_names)
        g0 = read_vec(bus, DATA_GOAL, motor_names)
        print(f"[start] present={p0}")
        print(f"[start] goal   ={g0}")
        print(f"[start] present[{args.joint}]={p0[j]}  goal[{args.joint}]={g0[j]}")

        # Jog goal
        g1 = g0[:]
        g1[j] = int(g1[j] + args.jog)
        write_goal(bus, motor_names, g1)
        time.sleep(args.wait)

        p1 = read_vec(bus, DATA_PRESENT, motor_names)
        g1r = read_vec(bus, DATA_GOAL, motor_names)
        print(f"[jog]  goal   ={g1r}  (Δgoal={abs(g1r[j]-g0[j])})")
        print(f"[jog]  present={p1}   (Δpos ={abs(p1[j]-p0[j])})")
        print(f"[jog]  present[{args.joint}]={p1[j]}  goal[{args.joint}]={g1r[j]}")

        # Return goal
        write_goal(bus, motor_names, g0)
        time.sleep(args.wait)

        p2 = read_vec(bus, DATA_PRESENT, motor_names)
        g2 = read_vec(bus, DATA_GOAL, motor_names)
        print(f"[back] goal   ={g2}  (Δgoal_back={abs(g2[j]-g0[j])})")
        print(f"[back] present={p2}   (Δpos_back ={abs(p2[j]-p0[j])})")
        print(f"[back] present[{args.joint}]={p2[j]}  goal[{args.joint}]={g2[j]}")

        moved = (abs(p1[j] - p0[j]) >= 2) or (abs(p2[j] - p0[j]) >= 2)
        if moved:
            print("[OK] ✅ Motor motion observed.")
            return 0

        print("[WARN] ⚠️ Goal changed but Present did not.")
        print("       Most common causes:")
        print("       - Servo power rail is OFF / insufficient current (comms still work)")
        print("       - Wrong control mode (Mode not set / read-only / different key)")
        print("       - Speed/profile is 0 (movement disabled)")
        print("       - Wrong motor mapping/IDs (writing to a different device)")
        print("       - Mechanical hard stop / binding")
        return 1

    finally:
        try:
            if bus is not None and motor_names:
                set_torque(bus, motor_names, False)
                print("[cleanup] torque disabled")
        except Exception:
            pass
        try:
            if robot is not None:
                robot.disconnect()
                print("[cleanup] disconnected")
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
