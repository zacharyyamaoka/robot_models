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
        point = np.asarray(axis_point, float)
        rotation = rotation_matrix(axis, angle_rad)
        return Mesh((self.vertices - point) @ rotation.T + point, self.faces, self.name)


def rotation_matrix(axis: Sequence[float], angle_rad: float) -> np.ndarray:
    """Rodrigues: rotation by ``angle_rad`` about the unit ``axis``."""
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    t = 1.0 - c
    return np.array(
        [
            [c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t],
        ]
    )


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
    # DELIBERATELY WRONG (the ruler's red demonstration, PEP 0001): a plain 1 deg grid with no
    # conservative advancement and no bisection, so the stop is only known to within a step.
    # The next commit replaces it.
    import coal  # noqa: PLC0415

    if not moving or not fixed:
        return None
    point = np.asarray(axis_point, float)

    def bvh(mesh: Mesh):
        model = coal.BVHModelOBBRSS()
        model.beginModel(len(mesh.faces), len(mesh.vertices))
        model.addVertices(np.ascontiguousarray(mesh.vertices, float))
        model.addTriangles(np.ascontiguousarray(mesh.faces, np.int64))
        model.endModel()
        return model

    movers, stays = [bvh(m) for m in moving], [bvh(m) for m in fixed]
    identity = coal.Transform3s()
    queries = 0
    q = 0.0
    while q < max_rad:
        rotation = rotation_matrix(axis, side * (q + step_rad))
        pose = coal.Transform3s(rotation, point - rotation @ point)
        for i, a in enumerate(movers):
            for j, b in enumerate(stays):
                queries += 1
                distance = coal.distance(a, pose, b, identity, coal.DistanceRequest(), coal.DistanceResult())
                if distance < contact_m:
                    return FirstContact(side * q, distance, (0.0, 0.0, 0.0), i, j, queries)
        q += step_rad
    return None
