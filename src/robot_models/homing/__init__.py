"""Homing stop core: where the stops are, what a stall shows, and the pure fit.

Imports numpy and robot_msgs only. Exact mesh contact lives in ``robot_models.homing.contact``
and the cad2robot bundle reader in ``robot_models.homing.cad_bundle`` (both need the
``contact`` extra: coal + trimesh).
"""

from robot_models.homing.fit import fit
from robot_models.homing.types import (
    BACKLASH_KINDS,
    LOWER,
    SIDES,
    UPPER,
    BacklashModel,
    BacklashTable,
    HomingResult,
    JointBacklash,
    JointZero,
    StallEvidence,
    StopMap,
    StopSide,
)

__all__ = [
    "BACKLASH_KINDS",
    "LOWER",
    "SIDES",
    "UPPER",
    "BacklashModel",
    "BacklashTable",
    "HomingResult",
    "JointBacklash",
    "JointZero",
    "StallEvidence",
    "StopMap",
    "StopSide",
    "fit",
]
