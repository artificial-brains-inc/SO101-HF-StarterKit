# tools/read_q.py
from __future__ import annotations

import os
import sys
import inspect
import traceback
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Tuple

from lerobot.robots.so_follower.so_follower import SO100Follower


def _pick_best_scan(scan: Dict[int, List[int]]) -> Tuple[Optional[int], List[int]]:
    if not scan:
        return None, []
    items = [(b, list(ids)) for b, ids in scan.items() if ids]
    if not items:
        return None, []
    items.sort(key=lambda x: (len(x[1]), x[0]), reverse=True)
    return items[0][0], items[0][1]


def main() -> int:
    print("python =", sys.executable)

    PORT = os.environ.get("SO101_PORT", "/dev/ttyACM0")
    RID  = os.environ.get("SO101_ID", "my_awesome_follower_arm")

    print("PORT =", PORT)
    print("RID  =", RID)

    cfg = SimpleNamespace(
        calibration_dir=None,   # we’ll address calibration loading after we can read raw
        cameras={},
        disable_torque_on_disconnect=True,
        id=RID,
        max_relative_target=None,
        port=PORT,
        use_degrees=False,
    )

    r = None
    try:
        r = SO100Follower(cfg)
        r.connect(calibrate=False)
        bus = r.bus

        print("Robot class:", type(r))
        print("bus  =", type(bus))

        for fn in ("scan_port", "ping", "broadcast_ping", "sync_read", "read", "get_baudrate"):
            if hasattr(bus, fn):
                try:
                    print(f"bus.{fn} signature:", inspect.signature(getattr(bus, fn)))
                except Exception:
                    pass

        baud = None
        try:
            baud = bus.get_baudrate()
        except Exception:
            pass
        print("baud =", baud)

        scan = bus.scan_port(PORT)
        print("scan_port(PORT) ->", scan)

        best_baud, ids = _pick_best_scan(scan if isinstance(scan, dict) else {})
        print("best_baud =", best_baud)
        print("motor_ids =", ids)

        if not ids:
            raise RuntimeError("No motor IDs found")

        for mid in ids:
            print(f"ping({mid}) ->", bus.ping(mid))

        # Motor registry (we found this already)
        motor_names: Optional[List[str]] = None
        reg = getattr(bus, "motors", None)
        if isinstance(reg, dict):
            motor_names = list(reg.keys())
        print("\nmotor_names =", motor_names)

        if not motor_names:
            raise RuntimeError("Could not discover motor_names from bus.motors")

        # ---- READ RAW (normalize=False) ----
        pos_raw = bus.sync_read("Present_Position", motor_names, normalize=False)
        q_raw = [float(pos_raw[n]) for n in motor_names]
        print("\nRead OK (RAW, normalize=False).")
        print("q_raw =", q_raw)

        # ---- TRY NORMALIZED (optional) ----
        try:
            pos_norm = bus.sync_read("Present_Position", motor_names, normalize=True)
            q_norm = [float(pos_norm[n]) for n in motor_names]
            print("\nRead OK (NORMALIZED, normalize=True).")
            print("q_norm =", q_norm)
        except Exception as e:
            print("\nNormalized read failed (expected until calibration is registered):", repr(e))
            q_norm = None

        # Copy/paste helper
        print("\nCOPY THIS:")
        print("SO101_MOTORS=" + ",".join(motor_names))
        print("TARGET_Q_RAW=" + ",".join(f"{x:.6f}" for x in q_raw))
        if q_norm is not None:
            print("TARGET_Q_NORM=" + ",".join(f"{x:.6f}" for x in q_norm))

        return 0

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
