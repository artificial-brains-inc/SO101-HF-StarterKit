"""
User-owned policy file (created once; never overwritten).

Implement:
  compute_reward(summary, stdp_layers) -> (global_reward, by_layer)

This version computes:
- per-joint reward (based on progress toward target)
- global reward (mean of joint rewards)
- by_layer reward keyed by STDP3 layer ids (each maps to the joint it targets)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _layer_to_joint(layer_id: str) -> Optional[str]:
    """
    Your layer ids look like:
      W_STDP3_Outshoulder_pan_to_shoulder_pan
    so we can safely parse the joint name from the suffix after '_to_'.
    """
    s = str(layer_id or "")
    if "_to_" not in s:
        return None
    return s.split("_to_", 1)[1] or None


@dataclass
class CycleSummary:
    # required
    motor_names: List[str]
    target_q_raw: List[float]

    # measured at start/end of cycle execution
    start_q_raw: List[float]
    end_q_raw: List[float]

    # normalization
    span_raw: Optional[List[float]] = None  # per joint; if missing, fallback span=1

    # shaping knobs (can be overridden by controller if you want)
    max_useful_progress: float = 0.20   # normalized error improvement for "full credit"
    success_err: float = 0.03           # mean normalized err threshold for success bonus


def compute_reward(
    summary: Optional[CycleSummary],
    *,
    stdp_layers: List[str],
) -> Tuple[float, Dict[str, float]]:
    """
    Returns (global_reward, by_layer)

    Reward is deterministic and based on progress:
      err = |q - target| / span
      progress = err_start - err_end  (positive is good)

    Map progress to [0,1] with:
      p = clamp(progress / max_useful_progress, -1, 1)
      r = clamp(0.5 + 0.45*p, 0, 1)
    """
    if summary is None:
        r = 0.5
        return float(r), {layer_id: float(r) for layer_id in (stdp_layers or [])}

    names = list(summary.motor_names or [])
    if not names:
        r = 0.5
        return float(r), {layer_id: float(r) for layer_id in (stdp_layers or [])}

    tgt = list(summary.target_q_raw or [])
    q0 = list(summary.start_q_raw or [])
    q1 = list(summary.end_q_raw or [])
    span = list(summary.span_raw or [])

    J = len(names)
    if len(tgt) != J or len(q0) != J or len(q1) != J:
        # malformed summary => neutral
        r = 0.5
        return float(r), {layer_id: float(r) for layer_id in (stdp_layers or [])}

    def joint_span(i: int) -> float:
        if i < len(span):
            s = float(span[i])
            return s if s > 0 else 1.0
        return 1.0

    # per-joint normalized errors (start/end)
    err0: Dict[str, float] = {}
    err1: Dict[str, float] = {}
    for i, name in enumerate(names):
        s = joint_span(i)
        err0[name] = abs(float(q0[i]) - float(tgt[i])) / s
        err1[name] = abs(float(q1[i]) - float(tgt[i])) / s

    # per-joint reward
    maxp = float(summary.max_useful_progress or 0.20)
    if maxp <= 0:
        maxp = 0.20

    r_joint: Dict[str, float] = {}
    for name in names:
        progress = float(err0[name]) - float(err1[name])  # positive good
        p = _clamp(progress / maxp, -1.0, 1.0)
        r = _clamp(0.5 + 0.45 * p, 0.0, 1.0)
        r_joint[name] = float(r)

    # global reward = mean joint reward (+ small success bonus)
    global_r = sum(r_joint.values()) / max(1, len(r_joint))

    mean_err_end = sum(err1.values()) / max(1, len(err1))
    success = (mean_err_end <= float(summary.success_err or 0.03))
    if success:
        global_r = _clamp(global_r + 0.10, 0.0, 1.0)

    # by_layer: STDP3 layer id -> reward for its target joint
    by_layer: Dict[str, float] = {}
    for layer_id in (stdp_layers or []):
        j = _layer_to_joint(layer_id)
        if j is None:
            by_layer[layer_id] = float(global_r)
        else:
            by_layer[layer_id] = float(r_joint.get(j, global_r))

    return float(global_r), by_layer
