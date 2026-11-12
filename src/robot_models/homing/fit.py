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

from robot_models.homing.types import (
    BACKLASH_KINDS,
    LOWER,
    UPPER,
    BacklashModel,
    BacklashTable,
    HomingResult,
    JointBacklash,
    JointZero,
    StallEvidence,
    StopMap,
)


def fit(
    evidence: Sequence[StallEvidence],
    stops: StopMap,
    *,
    identify: str = "B2",
    table: BacklashTable | None = None,
    rung: str | None = None,
) -> HomingResult:
    """Zero every joint the stalls touch; identify its backlash unless ``table`` gives it."""
    if identify not in BACKLASH_KINDS:
        raise ValueError(f"identify must be one of {BACKLASH_KINDS}, not {identify!r}")
    evidence = tuple(evidence)
    if not evidence:
        raise ValueError("fit needs at least one stall")
    by_joint: dict[str, list[StallEvidence]] = {}
    for stall in evidence:
        if stops.q_rad(stall.joint, stall.side) is None:
            raise ValueError(
                f"{stall.joint} stalled on its {'upper' if stall.side > 0 else 'lower'} side, "
                "but the stop map has no stop there"
            )
        by_joint.setdefault(stall.joint, []).append(stall)

    zeros, entries = [], []
    for joint, stalls in by_joint.items():
        stored = table.get(joint) if table is not None else None
        entry = stored if stored is not None else _identify(joint, stalls, stops, identify)
        offsets = [
            stops.q_rad(joint, s.side) + entry.model.motor_past_link_rad(s.side, s.torque_nm) - s.motor_raw_rad
            for s in stalls
        ]
        zeros.append(
            JointZero(
                joint,
                sum(offsets) / len(offsets),
                max(offsets) - min(offsets),
                len(offsets),
                tuple(sorted({s.side for s in stalls})),
            )
        )
        entries.append(entry)
    return HomingResult(tuple(zeros), BacklashTable(tuple(entries)), stops, evidence, rung or evidence[0].rung)


def _identify(joint: str, stalls: list[StallEvidence], stops: StopMap, kind: str) -> JointBacklash:
    sides = {side: [s for s in stalls if s.side == side] for side in (LOWER, UPPER)}
    notes = []

    # compliance: side * raw = side * level + compliance * torque on each side; pooled slope
    compliance, compliance_observable = 0.0, False
    if kind == "B2":
        numerator = denominator = 0.0
        for side, items in sides.items():
            torques = [s.torque_nm for s in items]
            if len(set(torques)) < 2:
                continue
            mean_torque = sum(torques) / len(torques)
            mean_y = sum(side * s.motor_raw_rad for s in items) / len(items)
            numerator += sum((s.torque_nm - mean_torque) * (side * s.motor_raw_rad - mean_y) for s in items)
            denominator += sum((s.torque_nm - mean_torque) ** 2 for s in items)
        if denominator > 0:
            compliance_observable = True
            compliance = numerator / denominator
            notes.append("compliance: two-level torque slope")
            if compliance < 0:
                notes.append(f"compliance clipped from {compliance:.3e}")
                compliance = 0.0

    # gap: motor travel between the two stalls minus the CAD span (sim only: a CAD span error aliases in)
    gap, gap_observable = 0.0, False
    span = stops.span_rad(joint)
    if kind in ("B1", "B2") and sides[LOWER] and sides[UPPER] and span is not None:
        levels = {
            side: sum(s.motor_raw_rad - side * compliance * s.torque_nm for s in items) / len(items)
            for side, items in sides.items()
        }
        gap = levels[UPPER] - levels[LOWER] - span
        gap_observable = True
        notes.append("gap: stop span vs CAD")
        if gap < 0:
            notes.append(f"gap clipped from {gap:.3e} rad")
            gap = 0.0

    if compliance_observable:
        model = BacklashModel("B2", gap, compliance)
    elif gap_observable:
        model = BacklashModel("B1", gap)
    else:
        model = BacklashModel("B0")
    return JointBacklash(joint, model, gap_observable, compliance_observable, "identify: " + "; ".join(notes))
