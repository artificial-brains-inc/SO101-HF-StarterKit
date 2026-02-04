"""
User-owned policy file (created once; never overwritten).

Goal:
  For EACH feedback input id (fb.id), produce deviation_f32:
    dev: list[float] length T, values typically in [-1,1] but not required.
  The server will convert dev into a feedback raster using baseline + your corrections.

Contract inputs:
  from policies._contract import FEEDBACK_IDS, FEEDBACK_INFO

We interpret FEEDBACK_INFO to map:
  feedback_id -> joint_name   (via info["fromOutput"] or info["outputFrom"])

IMPORTANT CHANGE (distance-based deviation):
  We compute ONE deviation per joint per cycle from END normalized error:
    e = err_end                 (0 near target, larger when far)
    dev[t] = e * scale          (constant across all t in [0..T-1])

So:
  - dev ≈ 0 means "very close to target" (should protect / less random)
  - dev large means "far from target" (should explore / more random)

Deterministic + stateless (besides what the controller passes in ctx).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from policies._contract import FEEDBACK_INFO


@dataclass
class DeviationContext:
    """
    Controller supplies per-cycle error snapshots.

    err_start_by_joint:
      joint_name -> normalized abs error at cycle start (0..~1)
      (kept for compatibility / debugging; not used for distance-based dev)

    err_end_by_joint:
      joint_name -> normalized abs error at cycle end (0..~1)
      (this is the distance-to-target signal we use)
    """
    err_start_by_joint: Optional[Dict[str, float]] = None
    err_end_by_joint: Optional[Dict[str, float]] = None

    # shaping knobs (controller can pass env-derived values)
    dev_scale: float = 50.0
    dev_deadzone: float = 0.0


def _feedback_joint_for_id(feedback_id: str) -> Optional[str]:
    """
    Map a feedback input id -> joint name via FEEDBACK_INFO.

    We accept either:
      info["fromOutput"]  (preferred)
      info["outputFrom"]  (alternate key)
      info["joint"]       (fallback if you ever add it)
    """
    fid = str(feedback_id)
    try:
        for info in (FEEDBACK_INFO or []):
            if str(info.get("id")) != fid:
                continue
            j = info.get("fromOutput") or info.get("outputFrom") or info.get("joint")
            if j:
                return str(j)
    except Exception:
        pass
    return None


def _build_constant_deviation(
    *,
    err_end: float,
    T: int,
    dev_scale: float,
    dev_deadzone: float,
) -> List[float]:
    """
    Distance-based deviation per cycle:
      e = err_end                 (0 near target, larger when far)
      dev[t] = e * scale          constant for all t
    """
    T = int(T)
    if T <= 0:
        return []

    e = float(err_end)  # distance to target (assumed >= 0 in your normalized abs error)
    dz = float(dev_deadzone)
    if dz > 0.0 and e < dz:
        e = 0.0

    v = e * float(dev_scale)
    return [v] * T


def compute_deviation(
    feedback_id: str,
    *,
    T: int,
    ctx: Optional[DeviationContext] = None,
) -> List[float]:
    """
    Return dev[t] length T for THIS feedback channel.

    Distance-based meaning:
      dev ~ 0   => close to target
      dev large => far from target

    Default is all zeros if:
      - controller didn't provide end error for that joint
    """
    T = int(T)
    if T <= 0:
        return []

    if ctx is None or not ctx.err_end_by_joint:
        return [0.0] * T

    joint = _feedback_joint_for_id(feedback_id)
    if not joint:
        return [0.0] * T

    j = str(joint)
    if j not in ctx.err_end_by_joint:
        return [0.0] * T

    return _build_constant_deviation(
        err_end=float(ctx.err_end_by_joint.get(j, 0.0)),
        T=T,
        dev_scale=float(ctx.dev_scale),
        dev_deadzone=float(ctx.dev_deadzone),
    )
