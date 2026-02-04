"""
AUTO-GENERATED FILE. DO NOT EDIT.

This file is machine-owned and overwritten whenever you "sync contract".
It is meant to give developers the IDs they need for:
- per-layer STDP3 reward (by stdp layer id)
- per-feedback deviation (by feedback input id)

If this changes, your graph/IO changed. Compare sha256 or diff _contract.json.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional, TypedDict


CONTRACT_SHA256 = "45e355cf37efc3764b251fcf93a0d4cb3fa4994f21f88590f1d231f1d19a3e86"

# Constants (as reported by server)
GAMMA: int = 64
OUTPUT_WINDOW_N: int = 32
FEEDBACK_N: int = 32
FEEDBACK_T: int = 128

# IDs you typically need in policies
INPUT_IDS: List[str] = ['Cam', 'fbGripper', 'FB_shoulder_pan', 'FB_shoulder_lift', 'FB_elbow_flex', 'FB_wrist_flex', 'FB_wrist_roll', 'shoulder_pan_input', 'shoulder_lift_input', 'elbow_felx_input', 'wrist_roll_input', 'wrist_flex_input', 'gripper_input']
OUTPUT_IDS: List[str] = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
FEEDBACK_IDS: List[str] = ['fbGripper', 'FB_shoulder_pan', 'FB_shoulder_lift', 'FB_elbow_flex', 'FB_wrist_flex', 'FB_wrist_roll']

# Per-layer reward keys (STDP3)
STDP3_LAYERS: List[str] = ['W_STDP3_Outshoulder_pan_to_shoulder_pan', 'W_STDP3_Outshoulder_lift_to_shoulder_lift', 'W_STDP3_Outelbow_flex_to_elbow_flex', 'W_STDP3_Outwrist_flex_to_wrist_flex', 'W_STDP3_Outwrist_roll_to_wrist_roll', 'W_STDP3_Outgripper_to_gripper']


class FeedbackInfo(TypedDict, total=False):
    id: str
    n: int
    fromOutput: Optional[str]
    outputKind: Optional[str]


FEEDBACK_INFO: List[FeedbackInfo] = [{'id': 'fbGripper', 'n': 50, 'fromOutput': 'gripper', 'outputKind': None}, {'id': 'FB_shoulder_pan', 'n': 50, 'fromOutput': 'shoulder_pan', 'outputKind': None}, {'id': 'FB_shoulder_lift', 'n': 50, 'fromOutput': 'shoulder_lift', 'outputKind': None}, {'id': 'FB_elbow_flex', 'n': 50, 'fromOutput': 'elbow_flex', 'outputKind': None}, {'id': 'FB_wrist_flex', 'n': 50, 'fromOutput': 'wrist_flex', 'outputKind': None}, {'id': 'FB_wrist_roll', 'n': 50, 'fromOutput': 'wrist_roll', 'outputKind': None}]
