"""Exact first contact of two mesh sets turned about one joint axis (the ``contact`` extra: coal).

The kinematic homing rung: rigid, no softness. It is the cad2robot exporter's algorithm 2
(``cad2robot/hardstops.py``, python-fcl) ported to coal, the hpp-fcl successor:

* sweep  conservative advancement capped at ``step_rad``: a moving mesh's surface moves at
         most ``r_max * dq``, so a pair ``d`` apart cannot touch within ``(d - contact) / r_max``.
* refine bisection pins first contact to ``tolerance_rad``.
* result the last angle verified clear (the exporter's ``q_rad`` semantics); ``None`` when a
         full turn never touches.

``contact_m`` is the touching criterion: ``CONTACT_M`` (0.05 mm, the exporter's proximity
``contact_mm``) or ``INTERFERENCE_M`` (the meshes overlap), which the exporter's loop-joint
sweep records as ``distance_mm: null``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

CONTACT_M = 5e-5
# WHY 1 nm and not 0: coal reports 0 (or a negative depth) once two BVH meshes overlap.
INTERFERENCE_M = 1e-9
STEP_RAD = math.radians(1.0)
TOLERANCE_RAD = math.radians(0.05) / 8


@dataclass(frozen=True, eq=False)
class Mesh:
    """A triangle mesh placed in the robot's world frame at the export pose (metres)."""

    vertices: np.ndarray
    faces: np.ndarray
    name: str = ""

    def rotated(self, axis_point: Sequence[float], axis: Sequence[float], angle_rad: float) -> "Mesh":
        """The same mesh turned by ``angle_rad`` about the axis (used to move a stop on purpose)."""
        raise NotImplementedError("contract only (docs/peps/0001)")


@dataclass(frozen=True)
class FirstContact:
    """``q_rad``: signed joint angle last verified clear; first contact lies within tolerance past it."""

    q_rad: float
    distance_m: float
    point_m: tuple[float, float, float]
    moving_index: int
    fixed_index: int
    queries: int

    @property
    def q_deg(self) -> float:
        return math.degrees(self.q_rad)


def first_contact(
    moving: Sequence[Mesh],
    fixed: Sequence[Mesh],
    axis_point: Sequence[float],
    axis: Sequence[float],
    side: int,
    *,
    contact_m: float = CONTACT_M,
    step_rad: float = STEP_RAD,
    tolerance_rad: float = TOLERANCE_RAD,
    max_rad: float = 2.0 * math.pi,
) -> FirstContact | None:
    """Turn ``moving`` by ``side * q`` about the axis from q = 0 until it first touches ``fixed``."""
    raise NotImplementedError("contract only (docs/peps/0001): the implementation lands after the ruler")
