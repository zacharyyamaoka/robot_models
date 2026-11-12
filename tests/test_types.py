"""The frozen contract: validation, lookups and a lossless robot_msgs round trip."""

from __future__ import annotations

import math

import pytest

from robot_models.homing import (
    LOWER,
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


def _stops() -> StopMap:
    return StopMap(
        (
            StopSide("revolute_6", LOWER, math.radians(-90.03), source="8957130"),
            StopSide("revolute_6", UPPER, math.radians(125.0), source="8957130"),
            StopSide("revolute_8", LOWER, None),
        ),
        source="hardstops.json",
    )


def test_stop_map_lookups():
    stops = _stops()
    assert stops.joints == ("revolute_6", "revolute_8")
    assert stops.get("revolute_6", UPPER).q_deg == pytest.approx(125.0)
    assert stops.q_rad("revolute_8", LOWER) is None
    assert stops.get("revolute_8", UPPER) is None
    assert math.degrees(stops.span_rad("revolute_6")) == pytest.approx(215.03)
    assert stops.span_rad("revolute_8") is None


def test_invalid_values_refused():
    with pytest.raises(ValueError):
        StopSide("j", 0, 1.0)
    with pytest.raises(ValueError):
        StopSide("j", UPPER, float("nan"))
    with pytest.raises(ValueError):
        StopMap((StopSide("j", UPPER, 1.0), StopSide("j", UPPER, 1.1)))
    with pytest.raises(ValueError):
        BacklashModel("B0", gap_rad=0.001)
    with pytest.raises(ValueError):
        BacklashModel("B1", gap_rad=0.001, compliance_rad_per_nm=1e-4)
    with pytest.raises(ValueError):
        BacklashModel("B3")
    with pytest.raises(ValueError):
        StallEvidence("j", UPPER, -1.0, 0.0)


def test_motor_past_link_follows_the_sign_convention():
    model = BacklashModel("B2", gap_rad=0.002, compliance_rad_per_nm=1e-4)
    assert model.motor_past_link_rad(UPPER, 2.0) == pytest.approx(0.001 + 2e-4)
    assert model.motor_past_link_rad(LOWER, 2.0) == pytest.approx(-(0.001 + 2e-4))


def test_round_trip_is_lossless():
    table = BacklashTable((JointBacklash("revolute_6", BacklashModel("B1", gap_rad=0.0007), True, False, "x"),))
    result = HomingResult(
        zeros=(JointZero("revolute_6", 0.25, 1e-6, 2, (LOWER, UPPER)),),
        backlash=table,
        stops=_stops(),
        evidence=(StallEvidence("revolute_6", UPPER, 2.0, 1.9),),
    )
    again = HomingResult.from_dict(result.to_dict())
    assert again == result
    assert again.zero("revolute_6").offset_rad == 0.25
    assert again.backlash.get("revolute_6").gap_observable is True
    assert again.stops.q_rad("revolute_8", LOWER) is None
