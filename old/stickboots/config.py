from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FeetVars:
    X: float = 0.24
    Y: float = 0.24
    Z: float = 0.24
    box_x: float = 0.101
    box_y: float = 0.0527
    scad_fn: int = 100


@dataclass
class StickbotParams:
    # Structure and masses
    gap_ft: float = 0.032
    w_arm: float = 0.0625
    l_arm: float = 0.104
    l_leg: float = 0.153
    hip_offset: float = -0.01
    leg_mass: float = 0.1
    feet_mass: float = 0.13
    hand_mass: float = 0.15
    # Feet shape
    feet_vars: FeetVars = FeetVars()
