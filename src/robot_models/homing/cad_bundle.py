"""Read a cad2robot bundle's hard stops and place the meshes they name (the ``contact`` extra).

``hardstops.json`` (schema ``cad2robot_hardstops/1``) names, per joint side, the stop pair
``files`` (moving, fixed) and their ``links``. The meshes are placed exactly as the exporter's
URDF places them at q = 0 (``hardstops.json["urdf"]``, never build's ``.cad2robot.urdf``); the
joint axis is the URDF ``<axis>`` turned into the world frame at the zero pose.

stdlib XML for the URDF; trimesh only to read a mesh file.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from robot_models.homing.contact import CONTACT_M, INTERFERENCE_M, Mesh
from robot_models.homing.types import LOWER, UPPER, StopMap, StopSide

SCHEMA = "cad2robot_hardstops/1"
_URI = re.compile(r"^(?:file://)?(?:package://[^/]+/)?")
_SIDE_KEYS = ((LOWER, "lower"), (UPPER, "upper"))


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(float(x) for x in text.split()) if text else default


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _origin(element: ET.Element | None) -> np.ndarray:
    pose = np.eye(4)
    tag = element.find("origin") if element is not None else None
    if tag is not None:
        pose[:3, :3] = _rpy(*_floats(tag.get("rpy"), (0.0, 0.0, 0.0)))
        pose[:3, 3] = _floats(tag.get("xyz"), (0.0, 0.0, 0.0))
    return pose


@dataclass(frozen=True)
class CadStop:
    """One stop side as the exporter found it, with what is needed to find it again."""

    stop: StopSide
    moving_file: str
    fixed_file: str
    moving_link: str
    fixed_link: str
    axis_point: tuple[float, float, float]
    axis: tuple[float, float, float]
    contact_m: float
    kind: str


class CadBundle:
    """One exported robot folder (the one holding ``hardstops.json``)."""

    def __init__(self, root: str | Path):
        self.root = Path(root)
        path = self.root / "hardstops.json"
        raw = path.read_bytes()
        self.hardstops_sha256 = hashlib.sha256(raw).hexdigest()
        self.hardstops = json.loads(raw)
        if self.hardstops.get("schema") != SCHEMA:
            raise ValueError(f"{path}: schema {self.hardstops.get('schema')!r}, expected {SCHEMA!r}")
        self.urdf_path = self.root / self.hardstops["urdf"]
        self._parse_urdf()
        self._mesh_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    # -- the URDF at the zero pose --------------------------------------------------------

    def _parse_urdf(self) -> None:
        root = ET.parse(self.urdf_path).getroot()
        self.visuals: dict[str, list[tuple[str, np.ndarray, np.ndarray]]] = {}
        for link in root.iter("link"):
            items = []
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None or not mesh.get("filename"):
                    continue
                rel = _URI.sub("", mesh.get("filename").strip())
                items.append((rel, np.array(_floats(mesh.get("scale"), (1.0, 1.0, 1.0))), _origin(visual)))
            self.visuals[link.get("name")] = items
        self.joints = {}
        for joint in root.iter("joint"):
            axis_tag = joint.find("axis")
            axis = np.array(_floats(axis_tag.get("xyz") if axis_tag is not None else None, (1.0, 0.0, 0.0)))
            self.joints[joint.get("name")] = (
                joint.find("parent").get("link"),
                joint.find("child").get("link"),
                _origin(joint),
                axis / np.linalg.norm(axis),
            )
        children = {child for _, child, _, _ in self.joints.values()}
        self.world = {name: np.eye(4) for name in self.visuals if name not in children}
        pending = dict(self.joints)
        while pending:
            placed = [name for name, (parent, *_) in pending.items() if parent in self.world]
            if not placed:
                raise ValueError(f"{self.urdf_path}: joints with no path to the root: {sorted(pending)}")
            for name in placed:
                parent, child, origin, _ = pending.pop(name)
                self.world[child] = self.world[parent] @ origin

    def joint_axis(self, joint: str) -> tuple[np.ndarray, np.ndarray]:
        """(point on the axis, unit axis), world frame at the zero pose."""
        parent, _, origin, axis = self.joints[joint]
        frame = self.world[parent] @ origin
        return frame[:3, 3].copy(), frame[:3, :3] @ axis

    # -- stops --------------------------------------------------------------------------

    def stop_map(self) -> StopMap:
        """Every side of every swept joint; a null side is ``q_rad=None``, never dropped."""
        sides = []
        source = f"{self.root.name} hardstops.json sha256 {self.hardstops_sha256[:12]}"
        for joint, entry in self.hardstops["joints"].items():
            for side, key in _SIDE_KEYS:
                found = entry.get(key)
                sides.append(
                    StopSide(
                        joint,
                        side,
                        None if not found else float(found["q_rad"]),
                        basis="cad",
                        source=source,
                        pair=tuple(found["pair"]) if found else (),
                    )
                )
        return StopMap(tuple(sides), source=source)

    def cad_stops(self) -> tuple[CadStop, ...]:
        """The stop sides that exist, with their mesh pair, axis and touching criterion."""
        stops = self.stop_map()
        out = []
        for joint, entry in self.hardstops["joints"].items():
            point, axis = self.joint_axis(joint)
            for side, key in _SIDE_KEYS:
                found = entry.get(key)
                if not found:
                    continue
                # WHY: the exporter's loop-joint sweep (kind own_frame / interference) records no
                # distance and stops at overlap; the plain sweep stops at contact_mm (PEP 0001).
                proximity = found.get("distance_mm") is not None
                contact = float(self.hardstops.get("contact_mm", 0.05)) / 1000.0 if proximity else INTERFERENCE_M
                out.append(
                    CadStop(
                        stop=stops.get(joint, side),
                        moving_file=found["files"][0],
                        fixed_file=found["files"][1],
                        moving_link=found["links"][0],
                        fixed_link=found["links"][1],
                        axis_point=tuple(float(x) for x in point),
                        axis=tuple(float(x) for x in axis),
                        contact_m=contact,
                        kind=found.get("kind", "contact"),
                    )
                )
        return tuple(out)

    # -- meshes -------------------------------------------------------------------------

    def _read(self, rel: str) -> tuple[np.ndarray, np.ndarray]:
        if rel not in self._mesh_cache:
            import trimesh  # noqa: PLC0415 -- the contact extra

            mesh = trimesh.load(str(self.urdf_path.parent / rel), force="mesh", process=False)
            faces = np.asarray(mesh.faces, np.int64)
            if len(faces) == 0:
                raise ValueError(f"{rel}: no triangles")
            self._mesh_cache[rel] = (np.asarray(mesh.vertices, float), faces)
        return self._mesh_cache[rel]

    def placed(self, link: str, rel: str) -> list[Mesh]:
        """Every placement of mesh file ``rel`` on ``link``, in the world frame at q = 0."""
        out = []
        for file, scale, link_to_mesh in self.visuals.get(link, []):
            if file != rel:
                continue
            vertices, faces = self._read(rel)
            pose = self.world[link] @ link_to_mesh
            out.append(Mesh((vertices * scale) @ pose[:3, :3].T + pose[:3, 3], faces, f"{link}:{rel}"))
        return out

    def stop_meshes(self, stop: CadStop) -> tuple[list[Mesh], list[Mesh]]:
        return self.placed(stop.moving_link, stop.moving_file), self.placed(stop.fixed_link, stop.fixed_file)


__all__ = ["CONTACT_M", "INTERFERENCE_M", "CadBundle", "CadStop"]
