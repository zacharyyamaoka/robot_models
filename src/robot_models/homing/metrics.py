"""DELIBERATELY WRONG (the ruler commit, PEP 0002): four planted mistakes.

The five black-box homing metrics (docs/peps/0002-homing-bench.md), from outputs and truth only (numpy-free stdlib).

A homing procedure is scored per episode (one start), never by reading its code:

* ``time_to_home_s``         tick clock from the first command to done (median and p95 over finished episodes).
* ``reliability_pct``        done with no abort, timeout or refusal, and every stall it relied on truly on the CAD stop
                             it was meant for (within ``STALL_TOLERANCE_DEG``, bam_fb_test_rig ``stall_proof``'s 1.0 deg).
                             Every episode is in the denominator.
* ``any_start_pct``          share of episodes that are reliable AND have every joint's zero inside the rung's gate.
* ``backlash_error_deg``     |identified gap - hidden gap| per drive, the worst drive per episode, p95 over reliable
                             episodes; a drive not identified is '-' (None), never 0, and ``observable_pct`` says how many
                             were. Gate per drive: max(1 arcmin, 25 % of truth).
* ``joint_angle_error_deg``  |identified zero - true zero| per joint, the worst joint per episode, p95 over reliable
                             episodes (reliability holds the failures, so a wrong-stop zero never poses as an accuracy).

WHY no weighted sum (report section 2, C's harness): the leaderboard sorts by reliability, then zero error, then time;
time never buys accuracy. WHY p95 by nearest rank: it is a value an episode actually produced.
"""

from __future__ import annotations

import math
import statistics
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from robot_msgs import Msg

ARCMIN_DEG = 1.0 / 60.0
STALL_TOLERANCE_DEG = 1.0
BACKLASH_GATE_FRACTION = 0.25


@dataclass(frozen=True, eq=False, repr=False)
class StallCheck(Msg):
    """One stall a procedure used as a reference: where the link truly was, against the CAD stop it was meant for."""

    joint: str
    side: int
    link_deg: float
    cad_deg: float

    def ok(self, tolerance_deg: float = STALL_TOLERANCE_DEG) -> bool:
        return abs(self.link_deg - self.cad_deg) <= tolerance_deg


@dataclass(frozen=True, eq=False, repr=False)
class JointOutcome(Msg):
    """One joint after one episode. ``zero_error_deg`` None: the procedure left it unzeroed. ``gap_identified_deg``
    None: it did not identify the gap (shown '-')."""

    joint: str
    zero_error_deg: float | None
    gap_identified_deg: float | None
    gap_truth_deg: float


@dataclass(frozen=True, eq=False, repr=False)
class EpisodeRecord(Msg):
    """One procedure from one start in one environment (rung x backlash model)."""

    procedure: str
    environment: str
    start: str
    finished: bool
    outcome: str
    time_s: float | None
    stalls: tuple[StallCheck, ...]
    joints: tuple[JointOutcome, ...]

    def worst_zero_error_deg(self) -> float | None:
        errors = [abs(j.zero_error_deg) for j in self.joints if j.zero_error_deg is not None]
        return max(errors) if errors else None


@dataclass(frozen=True)
class TimeStat:
    median: float
    p95: float
    n: int


@dataclass(frozen=True)
class ErrorStat:
    p95: float | None
    max: float | None
    median: float | None
    n: int
    missing: int
    worst_joint: str | None


@dataclass(frozen=True)
class BacklashStat:
    value: float | None
    max: float | None
    observable_pct: float
    n: int


def nearest_rank(values: Iterable[float], percentile: float) -> float:
    """The nearest-rank percentile: the smallest value with at least ``percentile`` % of the values at or below it."""
    ordered = sorted(values)
    if not ordered:
        raise ValueError("nearest_rank of no values")
    rank = max(1, math.ceil(percentile / 100.0 * len(ordered)))
    return ordered[rank - 1]


def reliable(episode: EpisodeRecord, tolerance_deg: float = STALL_TOLERANCE_DEG) -> bool:
    return bool(episode.finished)  # WRONG ON PURPOSE: stalls never checked


def reliability_pct(episodes: Sequence[EpisodeRecord]) -> float | None:
    episodes = list(episodes)
    if not episodes:
        return None
    finished = [e for e in episodes if e.finished] or episodes  # WRONG ON PURPOSE: unfinished dropped
    return 100.0 * sum(reliable(e) for e in finished) / len(finished)


def time_to_home_s(episodes: Sequence[EpisodeRecord]) -> TimeStat | None:
    times = [e.time_s for e in episodes if e.finished and e.time_s is not None]
    if not times:
        return None
    return TimeStat(statistics.median(times), statistics.mean(times), len(times))  # WRONG ON PURPOSE: mean


def joint_angle_error_deg(episodes: Sequence[EpisodeRecord]) -> ErrorStat:
    worst, missing, by_joint = [], 0, {}
    for episode in episodes:
        if not reliable(episode):
            continue
        errors = []
        for joint in episode.joints:
            if joint.zero_error_deg is None:
                missing += 1
                continue
            errors.append(abs(joint.zero_error_deg))
            by_joint[joint.joint] = max(by_joint.get(joint.joint, 0.0), abs(joint.zero_error_deg))
        if errors:
            worst.append(max(errors))
    if not worst:
        return ErrorStat(None, None, None, 0, missing, None)
    return ErrorStat(nearest_rank(worst, 95), max(worst), statistics.median(worst), len(worst), missing,
                     max(by_joint, key=by_joint.get))


def any_start_pct(episodes: Sequence[EpisodeRecord], gate_deg: float) -> float | None:
    episodes = list(episodes)
    if not episodes:
        return None

    def inside(episode: EpisodeRecord) -> bool:
        return reliable(episode) and all(
            j.zero_error_deg is not None and abs(j.zero_error_deg) <= gate_deg for j in episode.joints)

    return 100.0 * sum(inside(e) for e in episodes) / len(episodes)


def backlash_gate_deg(truth_deg: float) -> float:
    return max(ARCMIN_DEG, BACKLASH_GATE_FRACTION * truth_deg)


def backlash_error_deg(episodes: Sequence[EpisodeRecord]) -> BacklashStat:
    worst, identified, total = [], 0, 0
    for episode in episodes:
        if not reliable(episode):
            continue
        errors = []
        for joint in episode.joints:
            total += 1
            identified += 1  # WRONG ON PURPOSE: None read as 0
            errors.append(abs((joint.gap_identified_deg or 0.0) - joint.gap_truth_deg))
        if errors:
            worst.append(max(errors))
    observable = 100.0 * identified / total if total else 0.0
    if not worst:
        return BacklashStat(None, None, observable, 0)
    return BacklashStat(nearest_rank(worst, 95), max(worst), observable, len(worst))


def backlash_gate_pass(episodes: Sequence[EpisodeRecord]) -> bool | None:
    verdicts = [abs(j.gap_identified_deg - j.gap_truth_deg) <= backlash_gate_deg(j.gap_truth_deg)
                for e in episodes if reliable(e) for j in e.joints if j.gap_identified_deg is not None]
    return all(verdicts) if verdicts else None


def scorecard(episodes: Sequence[EpisodeRecord], *, gate_deg: float, reliability_target_pct: float = 100.0) -> dict:
    """The five metrics plus the rung's verdicts, flat (a W&B summary / leaderboard row)."""
    episodes = list(episodes)
    time = time_to_home_s(episodes)
    zero = joint_angle_error_deg(episodes)
    gap = backlash_error_deg(episodes)
    reliability = reliability_pct(episodes)
    return {
        "episodes": len(episodes),
        "time_to_home_s": None if time is None else time.median,
        "time_to_home_p95_s": None if time is None else time.p95,
        "reliability_pct": reliability,
        "any_start_pct": any_start_pct(episodes, gate_deg),
        "backlash_error_deg": gap.value,
        "backlash_error_max_deg": gap.max,
        "backlash_observable_pct": gap.observable_pct,
        "joint_angle_error_deg": zero.p95,
        "joint_angle_error_max_deg": zero.max,
        "joint_angle_error_median_deg": zero.median,
        "joint_angle_error_worst_joint": zero.worst_joint,
        "joints_unzeroed": zero.missing,
        "zero_gate_deg": gate_deg,
        "zero_gate_pass": None if zero.max is None else bool(zero.max <= gate_deg and zero.missing == 0),
        "reliability_gate_pass": None if reliability is None else bool(reliability >= reliability_target_pct),
        "backlash_gate_pass": backlash_gate_pass(episodes),
    }


__all__ = [
    "ARCMIN_DEG",
    "STALL_TOLERANCE_DEG",
    "BacklashStat",
    "EpisodeRecord",
    "ErrorStat",
    "JointOutcome",
    "StallCheck",
    "TimeStat",
    "any_start_pct",
    "backlash_error_deg",
    "backlash_gate_deg",
    "backlash_gate_pass",
    "joint_angle_error_deg",
    "nearest_rank",
    "reliability_pct",
    "reliable",
    "scorecard",
    "time_to_home_s",
]
