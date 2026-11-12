"""RULER (PEP 0001): coal first contact reproduces the exporter's CAD hard stops.

Frozen before the code it judges. The expected angles are written here, not read back from
the bundle, and the bundle's hardstops.json is pinned by sha256, so a re-export cannot quietly
move the ruler. Mutations at the bottom must go red: a ruler that passes everything cannot
pass this file.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("coal", reason="the ruler needs the 'contact' extra (coal + trimesh)")

from robot_models.homing.cad_bundle import CadBundle  # noqa: E402
from robot_models.homing.contact import first_contact  # noqa: E402

pytestmark = pytest.mark.ruler

BUNDLES = Path(os.environ.get("ROBOT_MODELS_CAD_BUNDLES", "~/.cache/cad2robot-build/bundles")).expanduser()
FIVEBAR = BUNDLES / "8957130" / "fixtures" / "cad2robot" / "bam_fb_test_rig"
WRIST = BUNDLES / "583ee39" / "fixtures" / "cad2robot" / "ur_wrist_test_rig"
HARDSTOPS_SHA256 = {
    "fivebar": "9d42c878c519023c4513279fe80129668f6811fe4014b079c60239edeb161717",
    "wrist": "2541a389db6ed502fd9b075cf9685105960d644d27b4b22267526b05850ccd20",
}
ROOTS = {"fivebar": FIVEBAR, "wrist": WRIST}

# The CAD stops (degrees; None = the geometry never stops that side). v11 five-bar bundle
# 8957130 and UR wrist bundle 583ee39, as hardstops.json records them.
CAD_DEG = {
    "fivebar": {
        ("revolute_3", -1): -89.975, ("revolute_3", +1): 89.975,
        ("revolute_6", -1): -90.03, ("revolute_6", +1): 125.00,
        ("revolute_7", -1): -112.69, ("revolute_7", +1): 197.77,
        ("revolute_8", -1): None, ("revolute_8", +1): None,
        ("revolute_34", -1): None, ("revolute_34", +1): None,
    },
    "wrist": {
        ("revolute_11", -1): -169.85, ("revolute_11", +1): 169.85,
        ("revolute_14", -1): -169.849, ("revolute_14", +1): 169.849,
        ("revolute_17", -1): -169.849, ("revolute_17", +1): 169.849,
    },
}
TOLERANCE_DEG = 0.05  # today's exporter tolerance_deg
STRICT_TOLERANCE_DEG = 0.01  # once the exporter's stop meshes land
SHIFT_DEG = 0.1

STOPS = [(rig, joint, side) for rig, table in CAD_DEG.items() for (joint, side), deg in table.items() if deg is not None]


def _bundle(rig: str) -> CadBundle:
    root = ROOTS[rig]
    if not (root / "hardstops.json").is_file():
        pytest.fail(f"ruler bundle missing: {root} (set ROBOT_MODELS_CAD_BUNDLES)")
    return _CACHE.setdefault(rig, CadBundle(root))


_CACHE: dict[str, CadBundle] = {}


def _cad_stop(rig: str, joint: str, side: int):
    return next(s for s in _bundle(rig).cad_stops() if s.stop.joint == joint and s.stop.side == side)


def _find(rig, joint, side, *, moving=None, fixed=None):
    bundle = _bundle(rig)
    stop = _cad_stop(rig, joint, side)
    default_moving, default_fixed = bundle.stop_meshes(stop)
    found = first_contact(
        default_moving if moving is None else moving,
        default_fixed if fixed is None else fixed,
        stop.axis_point,
        stop.axis,
        side,
        contact_m=stop.contact_m,
    )
    return None if found is None else found.q_deg


def stop_error(found_deg: float | None, cad_deg: float | None, tolerance_deg: float) -> str | None:
    """None when the found stop agrees with CAD; else why not. The comparator the mutations attack."""
    if cad_deg is None or found_deg is None:
        return None if cad_deg is found_deg else f"found {found_deg} vs CAD {cad_deg}"
    delta = found_deg - cad_deg
    return None if abs(delta) <= tolerance_deg else f"found {found_deg:.4f} vs CAD {cad_deg:.4f} deg ({delta:+.4f})"


# -- the frozen inputs ------------------------------------------------------------------------


@pytest.mark.parametrize("rig", sorted(ROOTS))
def test_bundle_is_the_frozen_one(rig):
    assert _bundle(rig).hardstops_sha256 == HARDSTOPS_SHA256[rig]


@pytest.mark.parametrize("rig", sorted(ROOTS))
def test_cad_stop_map_matches_the_frozen_table(rig):
    stops = _bundle(rig).stop_map()
    assert {(s.joint, s.side) for s in stops.sides} == set(CAD_DEG[rig])
    for (joint, side), deg in CAD_DEG[rig].items():
        q_rad = stops.q_rad(joint, side)
        if deg is None:
            assert q_rad is None, (joint, side)
        else:
            # WHY 0.005: hardstops.json prints the loop joints' q_deg to 0.01 deg (197.77 for q_rad 3.451798).
            assert math.degrees(q_rad) == pytest.approx(deg, abs=0.005), (joint, side)


@pytest.mark.parametrize("rig", sorted(ROOTS))
def test_axis_from_the_urdf_matches_hardstops_json(rig):
    bundle = _bundle(rig)
    for joint, entry in bundle.hardstops["joints"].items():
        point, axis = bundle.joint_axis(joint)
        assert np.allclose(point, entry["origin_m"], atol=1e-4), joint
        assert np.allclose(axis, entry["axis"], atol=1e-6), joint


# -- the ruler --------------------------------------------------------------------------------


@pytest.mark.parametrize(("rig", "joint", "side"), STOPS)
def test_first_contact_reproduces_the_cad_stop(rig, joint, side):
    found = _find(rig, joint, side)
    print(f"{rig} {joint} {'upper' if side > 0 else 'lower'}: found {found} CAD {CAD_DEG[rig][(joint, side)]} deg")
    assert stop_error(found, CAD_DEG[rig][(joint, side)], TOLERANCE_DEG) is None


@pytest.mark.xfail(
    strict=True,
    reason="the exporter does not emit stops.json / meshes/stops/*.stl yet (exporter change request, homing "
    "report section 7); the gate tightens to 0.01 deg on the stop_mesh channel once it does",
)
@pytest.mark.parametrize("rig", sorted(ROOTS))
def test_stop_mesh_channel_within_0_01_deg(rig):
    bundle = _bundle(rig)
    manifest = bundle.root / "stops.json"
    assert manifest.is_file(), f"no {manifest}"
    import json

    entries = json.loads(manifest.read_text(encoding="utf-8"))["stops"]
    for entry in entries:
        stop = _cad_stop(rig, entry["joint"], entry["side"])
        moving = bundle.placed(stop.moving_link, entry["stop_mesh"]["moving"])
        fixed = bundle.placed(stop.fixed_link, entry["stop_mesh"]["fixed"])
        found = _find(rig, entry["joint"], entry["side"], moving=moving, fixed=fixed)
        assert stop_error(found, CAD_DEG[rig][(entry["joint"], entry["side"])], STRICT_TOLERANCE_DEG) is None


# -- mutations: each must turn the ruler red ---------------------------------------------------


@pytest.mark.parametrize(("rig", "joint", "side"), STOPS)
def test_mutation_deleted_stop_mesh_goes_red(rig, joint, side):
    cad = CAD_DEG[rig][(joint, side)]
    assert stop_error(_find(rig, joint, side, fixed=[]), cad, TOLERANCE_DEG) is not None
    assert stop_error(_find(rig, joint, side, moving=[]), cad, TOLERANCE_DEG) is not None


@pytest.mark.parametrize(("rig", "joint", "side"), STOPS)
@pytest.mark.parametrize("direction", [+1, -1], ids=["later", "earlier"])
def test_mutation_stop_shifted_0_1_deg_goes_red(rig, joint, side, direction):
    bundle = _bundle(rig)
    stop = _cad_stop(rig, joint, side)
    _, fixed = bundle.stop_meshes(stop)
    shift = direction * side * math.radians(SHIFT_DEG)
    moved = [mesh.rotated(stop.axis_point, stop.axis, shift) for mesh in fixed]
    found = _find(rig, joint, side, fixed=moved)
    cad = CAD_DEG[rig][(joint, side)]
    assert stop_error(found, cad, TOLERANCE_DEG) is not None
    # ... and red for the right reason: the found stop moved with the geometry.
    assert found - cad == pytest.approx(direction * side * SHIFT_DEG, abs=0.02)
