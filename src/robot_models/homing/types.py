"""The homing stop core's frozen types (docs/peps/0001-homing-stop-core.md).

Every angle is radians in the URDF joint coordinate (zero = the CAD export pose, sign per the
URDF ``<axis>``). Degrees are for display only; convert at the edge.

Sign convention (the same as ``gear_backlash``): ``twist = q_link - q_motor``, both output
referred. Pressing a joint into its ``side`` stop (-1 lower, +1 upper) with torque ``T`` pins the
link on the stop and leaves the motor *past* it by ``side * (gap / 2 + compliance * T)``.

The zero sits at the centre of the gap (``gear_backlash`` ``zero="center"``): an unloaded link
rests at ``q_motor`` on average, so ``raw + offset`` is the joint angle a controller should use.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from robot_msgs import Msg

LOWER = -1
UPPER = +1
SIDES = (LOWER, UPPER)

BacklashKind = Literal["B0", "B1", "B2"]
BACKLASH_KINDS: tuple[str, ...] = ("B0", "B1", "B2")
STOP_BASES = ("cad", "typed", "measured")


def _check_side(side: int) -> None:
    if side not in SIDES:
        raise ValueError(f"side must be -1 (lower) or +1 (upper), not {side!r}")


def _finite(name: str, value: float) -> None:
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, not {value!r}")


# --------------------------------------------------------------------------------------------
# where the stops are
# --------------------------------------------------------------------------------------------


@dataclass(frozen=True, eq=False, repr=False)
class StopSide(Msg):
    """One side of one joint's hard stop.

    ``q_rad`` is the joint angle at first contact of the stop pair; ``None`` means the geometry
    never stops the joint that way (a passive loop joint, a free-turning axis) -- never a
    silent 0. ``basis`` says where the number came from: ``cad`` (an exporter sweep),
    ``typed`` (Zach typed it) or ``measured`` (a homing rung found it).
    """

    joint: str
    side: int
    q_rad: float | None
    basis: str = "cad"
    source: str = ""
    pair: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        _check_side(self.side)
        if self.q_rad is not None:
            _finite("q_rad", self.q_rad)
        if self.basis not in STOP_BASES:
            raise ValueError(f"basis must be one of {STOP_BASES}, not {self.basis!r}")

    @property
    def q_deg(self) -> float | None:
        return None if self.q_rad is None else math.degrees(self.q_rad)


@dataclass(frozen=True, eq=False, repr=False)
class StopMap(Msg):
    """Every stop side of one robot, from one source."""

    sides: tuple[StopSide, ...]
    source: str = ""

    def __post_init__(self) -> None:
        seen = set()
        for stop in self.sides:
            key = (stop.joint, stop.side)
            if key in seen:
                raise ValueError(f"duplicate stop side {key}")
            seen.add(key)

    @property
    def joints(self) -> tuple[str, ...]:
        return tuple(dict.fromkeys(stop.joint for stop in self.sides))

    def get(self, joint: str, side: int) -> StopSide | None:
        _check_side(side)
        for stop in self.sides:
            if stop.joint == joint and stop.side == side:
                return stop
        return None

    def q_rad(self, joint: str, side: int) -> float | None:
        stop = self.get(joint, side)
        return None if stop is None else stop.q_rad

    def span_rad(self, joint: str) -> float | None:
        """Upper minus lower stop, or None unless both sides stop."""
        lower, upper = self.q_rad(joint, LOWER), self.q_rad(joint, UPPER)
        return None if lower is None or upper is None else upper - lower


# --------------------------------------------------------------------------------------------
# backlash: the B0 / B1 / B2 ladder
# --------------------------------------------------------------------------------------------


@dataclass(frozen=True, eq=False, repr=False)
class BacklashModel(Msg):
    """One drive's lost motion between motor and link, output referred.

    ``B0``  none: link = motor (``gap_rad = compliance = 0``).
    ``B1``  dead zone: ``gear_backlash.BacklashModel(method="deadzone", zero="center")`` in its
            rigid limit -- the link floats ``+-gap/2`` about the motor and carries no torque
            inside the gap; outside it nothing gives.
    ``B2``  elastic: the same dead zone plus a series compliance ``1/k``. Without an output
            sensor the gear mesh, the stop and the structure are not separable, so
            ``compliance_rad_per_nm`` is their sum -- all the zero needs.
    """

    kind: str = "B0"
    gap_rad: float = 0.0
    compliance_rad_per_nm: float = 0.0

    def __post_init__(self) -> None:
        if self.kind not in BACKLASH_KINDS:
            raise ValueError(f"kind must be one of {BACKLASH_KINDS}, not {self.kind!r}")
        _finite("gap_rad", self.gap_rad)
        _finite("compliance_rad_per_nm", self.compliance_rad_per_nm)
        if self.gap_rad < 0 or self.compliance_rad_per_nm < 0:
            raise ValueError("gap_rad and compliance_rad_per_nm must be >= 0")
        if self.kind == "B0" and (self.gap_rad or self.compliance_rad_per_nm):
            raise ValueError("B0 has no gap and no compliance")
        if self.kind == "B1" and self.compliance_rad_per_nm:
            raise ValueError("B1 is rigid outside the gap: use B2 for a compliance")

    def motor_past_link_rad(self, side: int, torque_nm: float) -> float:
        """``q_motor - q_link`` while the link is pressed into its ``side`` stop with ``torque_nm``."""
        _check_side(side)
        if torque_nm < 0:
            raise ValueError("torque_nm is the magnitude pressing into the stop: >= 0")
        return side * (0.5 * self.gap_rad + self.compliance_rad_per_nm * torque_nm)


@dataclass(frozen=True, eq=False, repr=False)
class JointBacklash(Msg):
    """One joint's backlash model and what was actually observed.

    A flag that is False means that part of the model is a prior (or 0 for B0), not a
    measurement: a page shows it as '--', never as an identified 0.
    """

    joint: str
    model: BacklashModel
    gap_observable: bool = False
    compliance_observable: bool = False
    source: str = ""


@dataclass(frozen=True, eq=False, repr=False)
class BacklashTable(Msg):
    """The stored per-joint backlash ``home()`` uses on every boot; ``identify_backlash()`` writes it."""

    entries: tuple[JointBacklash, ...] = ()

    def __post_init__(self) -> None:
        names = [entry.joint for entry in self.entries]
        if len(names) != len(set(names)):
            raise ValueError(f"duplicate joints in BacklashTable: {names}")

    def get(self, joint: str) -> JointBacklash | None:
        for entry in self.entries:
            if entry.joint == joint:
                return entry
        return None


# --------------------------------------------------------------------------------------------
# what a stall gives, and what homing returns
# --------------------------------------------------------------------------------------------


@dataclass(frozen=True, eq=False, repr=False)
class StallEvidence(Msg):
    """One settled stall of one joint against one stop.

    ``motor_raw_rad`` is the motor-side encoder, output referred, in the encoder's own frame
    (before any zero). ``torque_nm`` is the magnitude pressing into the stop.
    """

    joint: str
    side: int
    torque_nm: float
    motor_raw_rad: float
    rung: str = "kinematic"
    settled: bool = True

    def __post_init__(self) -> None:
        _check_side(self.side)
        _finite("torque_nm", self.torque_nm)
        _finite("motor_raw_rad", self.motor_raw_rad)
        if self.torque_nm < 0:
            raise ValueError("torque_nm is the magnitude pressing into the stop: >= 0")


@dataclass(frozen=True, eq=False, repr=False)
class JointZero(Msg):
    """One joint's zero: ``q_joint = motor_raw_rad + offset_rad``.

    ``spread_rad`` is max minus min of the per-stall offsets (0 for a single stall): the repeat
    check, never the error against truth.
    """

    joint: str
    offset_rad: float
    spread_rad: float = 0.0
    n_evidence: int = 0
    sides: tuple[int, ...] = ()

    @property
    def offset_deg(self) -> float:
        return math.degrees(self.offset_rad)


@dataclass(frozen=True, eq=False, repr=False)
class HomingResult(Msg):
    """What a homing run concludes, plus everything needed to re-derive it without moving.

    ``corrected(table)`` re-fits the same stalls with a new BacklashTable (a fresh calibration
    re-derives the zero with no motion).
    """

    zeros: tuple[JointZero, ...]
    backlash: BacklashTable
    stops: StopMap
    evidence: tuple[StallEvidence, ...] = ()
    rung: str = "kinematic"

    def zero(self, joint: str) -> JointZero | None:
        for item in self.zeros:
            if item.joint == joint:
                return item
        return None

    def corrected(self, table: BacklashTable) -> HomingResult:
        from robot_models.homing.fit import fit  # noqa: PLC0415 -- fit imports this module

        return fit(self.evidence, self.stops, table=table, rung=self.rung)
