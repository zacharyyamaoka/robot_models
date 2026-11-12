"""Pure fit: settled stalls in, a zero per joint and a backlash table out (numpy + stdlib only).

The forward model it inverts (docs/peps/0001, sign convention in ``types``)::

    motor_raw = q_stop(side) + side * (gap / 2 + compliance * torque) - offset

``identify`` sets how far up the B0 / B1 / B2 ladder the stalls are asked to go:

* compliance (B2) needs two torque levels on one side: the slope of motor_raw against torque.
* gap (B1, B2) needs both sides of a joint whose stops both exist: what the motor travels
  between the two stalls minus the CAD span. WHY flagged sim-only in the PEP: on a real robot
  a CAD span error aliases straight into this gap.
* A part that the stalls cannot show is reported unobservable (a prior or 0), never as a
  measured 0.

``table`` (the boot path) skips identification and applies the stored backlash instead.
"""

from __future__ import annotations

from collections.abc import Sequence

from robot_models.homing.types import BacklashTable, HomingResult, StallEvidence, StopMap


def fit(
    evidence: Sequence[StallEvidence],
    stops: StopMap,
    *,
    identify: str = "B2",
    table: BacklashTable | None = None,
    rung: str | None = None,
) -> HomingResult:
    """Zero every joint the stalls touch; identify its backlash unless ``table`` gives it."""
    # DELIBERATELY WRONG (the ruler's red demonstration, PEP 0001): B0 always -- the zero ignores
    # the gap and the compliance, and nothing is identified. The next commit replaces it.
    from robot_models.homing.types import BacklashModel, JointBacklash, JointZero

    joints = list(dict.fromkeys(e.joint for e in evidence))
    zeros = []
    for joint in joints:
        offsets = [stops.q_rad(joint, e.side) - e.motor_raw_rad for e in evidence if e.joint == joint]
        zeros.append(JointZero(joint, sum(offsets) / len(offsets), max(offsets) - min(offsets), len(offsets)))
    table_out = BacklashTable(tuple(JointBacklash(j, BacklashModel("B0")) for j in joints))
    return HomingResult(tuple(zeros), table_out, stops, tuple(evidence), rung or "kinematic")
