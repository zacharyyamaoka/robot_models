"""RULER (PEP 0001): the pure fit recovers a hidden zero and hidden backlash on the kinematic rung.

The plant here is independent of the package: the link is pinned on the CAD stop (rigid,
kinematic rung) and the motor angle is SOLVED numerically from a dead-zone spring law
(Modelica ElastoBacklash with no damping, as gear_backlash.BacklashSpec.transmitted_torque_nm)
-- not the closed form fit() inverts. Hidden truths are the 11-11 homing page's: J1 3.5',
J2 2.5', C1 4.0'. Mutations at the bottom must go red.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import pytest

from robot_models.homing import (
    LOWER,
    UPPER,
    BacklashModel,
    BacklashTable,
    JointBacklash,
    StallEvidence,
    fit,
)
from robot_models.homing.types import StopMap

pytestmark = pytest.mark.ruler

BUNDLES = Path(os.environ.get("ROBOT_MODELS_CAD_BUNDLES", "~/.cache/cad2robot-build/bundles")).expanduser()
FIVEBAR = BUNDLES / "8957130" / "fixtures" / "cad2robot" / "bam_fb_test_rig"

ARCMIN = math.radians(1.0 / 60.0)
HIDDEN_GAP_RAD = {"revolute_3": 3.5 * ARCMIN, "revolute_6": 2.5 * ARCMIN, "revolute_7": 4.0 * ARCMIN}  # J1, J2, C1
HIDDEN_OFFSET_RAD = {"revolute_3": 0.7, "revolute_6": -1.3, "revolute_7": 2.1}
MESH_STIFFNESS_NM_PER_RAD = 2000.0  # fixture: T = 2 N m moves the motor 0.057 deg past the link
RIGID_NM_PER_RAD = 1e12
TORQUES_NM = (1.0, 2.0)

ZERO_GATE_DEG = 0.01  # kinematic rung gate (homing report, metrics)


def backlash_gate_deg(truth_deg: float) -> float:
    """The report's sim gate: max(1 arcmin, 25 % of truth)."""
    return max(1.0 / 60.0, 0.25 * truth_deg)


def _stops() -> StopMap:
    if not (FIVEBAR / "hardstops.json").is_file():
        pytest.fail(f"ruler bundle missing: {FIVEBAR} (set ROBOT_MODELS_CAD_BUNDLES)")
    pytest.importorskip("trimesh")
    from robot_models.homing.cad_bundle import CadBundle

    return CadBundle(FIVEBAR).stop_map()


def link_torque_nm(twist_rad: float, gap_rad: float, stiffness: float) -> float:
    """Torque the mesh puts on the LINK for twist = q_link - q_motor (dead-zone spring)."""
    half = 0.5 * gap_rad
    if twist_rad > half:
        penetration = twist_rad - half
    elif twist_rad < -half:
        penetration = twist_rad + half
    else:
        return 0.0
    return -stiffness * penetration


def motor_at_stall(q_stop: float, side: int, torque_nm: float, gap_rad: float, stiffness: float) -> float:
    """Motor angle that pushes the pinned link into its stop with ``torque_nm`` (bisection)."""
    low, high = q_stop - 1.0, q_stop + 1.0  # link_torque(q_stop - m) rises with m
    for _ in range(200):
        mid = 0.5 * (low + high)
        if link_torque_nm(q_stop - mid, gap_rad, stiffness) < side * torque_nm:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


def plant_stalls(stops, gaps, stiffness, *, sides=(LOWER, UPPER), torques=TORQUES_NM):
    out = []
    for joint, gap in gaps.items():
        for side in sides:
            for torque in torques:
                motor = motor_at_stall(stops.q_rad(joint, side), side, torque, gap, stiffness)
                out.append(StallEvidence(joint, side, torque, motor - HIDDEN_OFFSET_RAD[joint]))
    return out


def fit_errors(result, *, gaps, stiffness=None, check_backlash=True) -> list[str]:
    """Every way the result misses the hidden truth. The comparator the mutations attack."""
    errors = []
    for joint, offset in HIDDEN_OFFSET_RAD.items():
        zero = result.zero(joint)
        if zero is None:
            errors.append(f"{joint}: no zero")
            continue
        error = abs(math.degrees(zero.offset_rad - offset))
        if error > ZERO_GATE_DEG:
            errors.append(f"{joint}: zero off by {error:.5f} deg")
        if not check_backlash:
            continue
        entry = result.backlash.get(joint)
        truth = math.degrees(gaps[joint])
        if entry is None or not entry.gap_observable:
            errors.append(f"{joint}: gap not identified")
        else:
            gap_error = abs(math.degrees(entry.model.gap_rad) - truth)
            if gap_error > backlash_gate_deg(truth):
                errors.append(f"{joint}: gap off by {gap_error:.5f} deg")
        if stiffness is not None:
            if entry is None or not entry.compliance_observable:
                errors.append(f"{joint}: compliance not identified")
            elif entry.model.compliance_rad_per_nm != pytest.approx(1.0 / stiffness, rel=0.01):
                errors.append(f"{joint}: compliance {entry.model.compliance_rad_per_nm} vs {1.0 / stiffness}")
    return errors


def _truth_table(kind="B2", stiffness=MESH_STIFFNESS_NM_PER_RAD, gaps=HIDDEN_GAP_RAD) -> BacklashTable:
    compliance = 0.0 if kind == "B1" else 1.0 / stiffness
    return BacklashTable(
        tuple(JointBacklash(j, BacklashModel(kind, g, compliance), True, kind == "B2", "truth") for j, g in gaps.items())
    )


# -- the ruler --------------------------------------------------------------------------------


def test_identify_b2_recovers_zero_gap_and_compliance():
    stops = _stops()
    result = fit(plant_stalls(stops, HIDDEN_GAP_RAD, MESH_STIFFNESS_NM_PER_RAD), stops, identify="B2")
    for joint in HIDDEN_GAP_RAD:
        entry, zero = result.backlash.get(joint), result.zero(joint)
        print(
            f"{joint}: zero error {math.degrees(zero.offset_rad - HIDDEN_OFFSET_RAD[joint]):+.2e} deg, gap "
            f"{math.degrees(entry.model.gap_rad) * 60:.4f}' vs {math.degrees(HIDDEN_GAP_RAD[joint]) * 60:.4f}'"
        )
    assert fit_errors(result, gaps=HIDDEN_GAP_RAD, stiffness=MESH_STIFFNESS_NM_PER_RAD) == []


def test_identify_b1_on_a_rigid_plant():
    stops = _stops()
    result = fit(plant_stalls(stops, HIDDEN_GAP_RAD, RIGID_NM_PER_RAD, torques=(2.0,)), stops, identify="B1")
    assert fit_errors(result, gaps=HIDDEN_GAP_RAD) == []
    assert all(e.model.kind == "B1" and e.model.compliance_rad_per_nm == 0.0 for e in result.backlash.entries)


def test_boot_with_the_stored_table_needs_one_side():
    stops = _stops()
    stalls = plant_stalls(stops, HIDDEN_GAP_RAD, MESH_STIFFNESS_NM_PER_RAD, sides=(UPPER,))
    result = fit(stalls, stops, table=_truth_table())
    assert fit_errors(result, gaps=HIDDEN_GAP_RAD, check_backlash=False) == []


def test_corrected_rederives_the_zero_without_motion():
    stops = _stops()
    stalls = plant_stalls(stops, HIDDEN_GAP_RAD, MESH_STIFFNESS_NM_PER_RAD, sides=(UPPER,))
    stale = BacklashTable(tuple(JointBacklash(j, BacklashModel("B0")) for j in HIDDEN_GAP_RAD))
    first = fit(stalls, stops, table=stale)
    assert fit_errors(first, gaps=HIDDEN_GAP_RAD, check_backlash=False) != []  # B0 ignores the gap
    assert fit_errors(first.corrected(_truth_table()), gaps=HIDDEN_GAP_RAD, check_backlash=False) == []


def test_one_side_never_reports_a_gap():
    stops = _stops()
    result = fit(plant_stalls(stops, HIDDEN_GAP_RAD, MESH_STIFFNESS_NM_PER_RAD, sides=(LOWER,)), stops, identify="B2")
    for entry in result.backlash.entries:
        assert entry.gap_observable is False
        assert entry.compliance_observable is True


# -- mutations: each must turn the ruler red ---------------------------------------------------


def test_mutation_doubled_hidden_gap_goes_red_on_identification():
    stops = _stops()
    doubled = {j: 2.0 * g for j, g in HIDDEN_GAP_RAD.items()}
    result = fit(plant_stalls(stops, doubled, MESH_STIFFNESS_NM_PER_RAD), stops, identify="B2")
    errors = fit_errors(result, gaps=HIDDEN_GAP_RAD, stiffness=MESH_STIFFNESS_NM_PER_RAD)
    assert sorted(e.split(":")[0] for e in errors if "gap off" in e) == sorted(HIDDEN_GAP_RAD)


def test_mutation_doubled_hidden_gap_goes_red_on_the_stored_table():
    stops = _stops()
    doubled = {j: 2.0 * g for j, g in HIDDEN_GAP_RAD.items()}
    stalls = plant_stalls(stops, doubled, MESH_STIFFNESS_NM_PER_RAD, sides=(UPPER,))
    errors = fit_errors(fit(stalls, stops, table=_truth_table()), gaps=HIDDEN_GAP_RAD, check_backlash=False)
    assert sorted(e.split(":")[0] for e in errors if "zero off" in e) == sorted(HIDDEN_GAP_RAD)
