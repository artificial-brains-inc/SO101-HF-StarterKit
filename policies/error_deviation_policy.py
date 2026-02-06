"""
User-owned policy file (created once; never overwritten).

Goal:
  For EACH feedback input id (fb.id), produce deviation_f32-like data:
    dev: list[float] length T, values in [-1, 1].

We keep the output format identical to the "deviation_f32" path so the server
doesn't need to change.

Meaning (now this is ERROR, not deviation):
  dev[t] = signed_normalized_error_joint in [-1,1], constant across all t in [0..T-1]

Normalization rule (per joint):
  Let:
    e_init = (q_init - q_target)   # signed
    e_end  = (q_end  - q_target)   # signed

  mag = clamp01(abs(e_end) / max(abs(e_init), eps))
  crossed = (e_init * e_end) < 0     # sign flip => overshoot
  err_signed = -mag if crossed else +mag

Edge case:
  If d_init is ~0 (started at target):
    - if d_end is ~0 => err = 0
    - else           => err = 1

Deterministic, stateless, simple.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from policies._contract import FEEDBACK_INFO


@dataclass
class ErrorContext:
    """
    Controller snapshot for ONE cycle.
    Values are joint positions in ANY units (ticks, degrees, etc) as long as
    init/end/target are consistent.
    """
    q_init_by_joint: Dict[str, float]
    q_end_by_joint: Dict[str, float]
    q_target_by_joint: Dict[str, float]


def _joint_for_feedback_id(feedback_id: str) -> Optional[str]:
    fid = str(feedback_id)
    for info in (FEEDBACK_INFO or []):
        if str(info.get("id")) != fid:
            continue
        j = info.get("fromOutput") or info.get("outputFrom") or info.get("joint")
        if j:
            return str(j)
    return None


def _clamp01(x: float) -> float:
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return float(x)

def _clamp11(x: float) -> float:
    if x < -1.0:
        return -1.0
    if x > 1.0:
        return 1.0
    return float(x)


def _constant_T(v: float, T: int) -> List[float]:
    T = int(T)
    if T <= 0:
        return []
    return [float(v)] * T


def compute_error(
    feedback_id: str,
    *,
    T: int,
    ctx: ErrorContext,
) -> List[float]:
    """
    Returns dev-like list[float] length T in [-1,1] for THIS feedback channel,
    representing per-joint SIGNED normalized error (negative => overshoot).
    """
    T = int(T)
    if T <= 0:
        return []

    joint = _joint_for_feedback_id(feedback_id)
    if not joint:
        return _constant_T(0.0, T)

    q0 = float(ctx.q_init_by_joint.get(joint, 0.0))
    q1 = float(ctx.q_end_by_joint.get(joint, 0.0))
    qt = float(ctx.q_target_by_joint.get(joint, 0.0))

    e0 = (q0 - qt)   # signed error at init
    e1 = (q1 - qt)   # signed error at end
    d0 = abs(e0)
    d1 = abs(e1)

    eps = 1e-9
    if d0 <= eps:
        # started essentially at target: no side to define overshoot
        mag = 0.0 if d1 <= eps else 1.0
        err_signed = 0.0 if d1 <= eps else 1.0
    else:
        mag = _clamp01(d1 / d0)
        crossed = (e0 * e1) < 0.0
        err_signed = (-mag) if crossed else (+mag)

    return _constant_T(_clamp11(err_signed), T)
