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
    """Turn ``moving`` by ``side * q`` about the axis from q = 0 until it first touches ``fixed``.

    Touching means ``distance < contact_m``. A pair touching at q = 0 is walked away in
    ``tolerance_rad`` steps for up to ``RESTING_PROBE_RAD``: if it frees, the sweep goes on from
    there; if not, the joint rests on this stop and the result is 0.
    """
    if side not in (-1, 1):
        raise ValueError(f"side must be -1 or +1, not {side!r}")
    if not moving or not fixed:
        return None
    sweep = _Sweep(moving, fixed, axis_point, axis, side, contact_m)
    floor = tolerance_rad
    clear = np.zeros((len(moving), len(fixed)))  # angle below which each pair cannot touch
    q, last = 0.0, 0.0
    if sweep.touching(0.0):
        while q < RESTING_PROBE_RAD and sweep.touching(q):
            q += floor
        if sweep.touching(q):
            return sweep.result(0.0)
        last = q - floor
        clear[:] = q
    while True:
        due = np.argwhere(clear <= q + 1e-15)
        for i, j in due:
            distance = sweep.distance(int(i), int(j), q)
            if distance < contact_m:
                return sweep.result(_refine(sweep, last, q, tolerance_rad))
            # WHY r_max: the moving surface travels at most r_max * dq, so this pair stays clear
            # until q + (distance - contact) / r_max (the exporter's conservative advancement).
            clear[i, j] = q + (distance - contact_m) / sweep.r_max[i]
        if q >= max_rad:
            return None
        last = q
        q = min(max(min(float(clear.min()), q + step_rad), q + floor), max_rad)


RESTING_PROBE_RAD = math.radians(5.0)


class _Sweep:
    def __init__(self, moving, fixed, axis_point, axis, side, contact_m):
        import coal  # noqa: PLC0415 -- the contact extra

        self.coal = coal
        self.point = np.asarray(axis_point, float)
        self.axis = np.asarray(axis, float) / np.linalg.norm(axis)
        self.side, self.contact_m = side, contact_m
        self.movers = [self._bvh(mesh) for mesh in moving]
        self.stays = [self._bvh(mesh) for mesh in fixed]
        self.r_max = np.array(
            [max(float(np.linalg.norm(np.cross(m.vertices - self.point, self.axis), axis=1).max()), 1e-9) for m in moving]
        )
        self.identity = coal.Transform3s()
        self.queries = 0
        self._pose_q: float | None = None
        self._pose = None

    def _bvh(self, mesh: Mesh):
        model = self.coal.BVHModelOBBRSS()
        model.beginModel(len(mesh.faces), len(mesh.vertices))
        model.addVertices(np.ascontiguousarray(mesh.vertices, float))
        model.addTriangles(np.ascontiguousarray(mesh.faces, np.int64))
        model.endModel()
        return model

    def pose(self, q: float):
        if self._pose_q != q:
            rotation = rotation_matrix(self.axis, self.side * q)
            self._pose = self.coal.Transform3s(rotation, self.point - rotation @ self.point)
            self._pose_q = q
        return self._pose

    def query(self, i: int, j: int, q: float):
        """coal always fills the nearest points (enable_nearest_points is deprecated in coal 3)."""
        self.queries += 1
        request = self.coal.DistanceRequest()
        result = self.coal.DistanceResult()
        distance = float(self.coal.distance(self.movers[i], self.pose(q), self.stays[j], self.identity, request, result))
        return distance, result

    def distance(self, i: int, j: int, q: float) -> float:
        return self.query(i, j, q)[0]

    def closest(self, q: float) -> tuple[float, int, int]:
        best = (math.inf, -1, -1)
        for i in range(len(self.movers)):
            for j in range(len(self.stays)):
                distance = self.distance(i, j, q)
                if distance < best[0]:
                    best = (distance, i, j)
        return best

    def touching(self, q: float) -> bool:
        return self.closest(q)[0] < self.contact_m

    def result(self, q_clear: float) -> FirstContact:
        _, i, j = self.closest(q_clear)
        distance, result = self.query(i, j, q_clear)
        point = 0.5 * (np.asarray(result.getNearestPoint1()) + np.asarray(result.getNearestPoint2()))
        return FirstContact(
            self.side * q_clear + 0.0, max(distance, 0.0), tuple(float(x) for x in point), i, j, self.queries
        )


def _refine(sweep: _Sweep, low: float, high: float, tolerance_rad: float) -> float:
    """Bisect between the last clear angle and the first touching one; return the last clear."""
    while high - low > tolerance_rad:
        middle = 0.5 * (low + high)
        if sweep.touching(middle):
            high = middle
        else:
            low = middle
    return low
