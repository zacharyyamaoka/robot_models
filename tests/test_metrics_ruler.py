"""RULER (PEP 0002): the five black-box homing metrics, computed from outputs and truth only.

Frozen before the code it judges. Every expected number below is worked out by hand from the episodes written here,
and each metric has a planted mistake it must catch (an unfinished episode left out of the denominator, an unidentified
gap read as 0, a mean where the report asks for the 95th percentile, a stall off its stop passing as reliable).
"""

from __future__ import annotations

import math

import pytest

from robot_models.homing.metrics import (
    ARCMIN_DEG,
    EpisodeRecord,
    JointOutcome,
    StallCheck,
    any_start_pct,
    backlash_error_deg,
    backlash_gate_deg,
    joint_angle_error_deg,
    nearest_rank,
    reliability_pct,
    reliable,
    scorecard,
    time_to_home_s,
)

pytestmark = pytest.mark.ruler

JOINTS = ("revolute_3", "revolute_6", "revolute_7")
TRUTH_GAP_DEG = {"revolute_3": 3.5 / 60.0, "revolute_6": 2.5 / 60.0, "revolute_7": 4.0 / 60.0}


def _episode(start, *, finished=True, time_s=20.0, errors=(0.001, 0.002, 0.003), gaps=(None, None, None),
             stall_link_deg=(89.97, 124.99, 197.76), cad_deg=(89.975, 125.0, 197.7735), outcome="done"):
    return EpisodeRecord(
        procedure="p", environment="kinematic · B1", start=start, finished=finished, outcome=outcome,
        time_s=time_s if finished else None,
        stalls=tuple(StallCheck(j, +1, link, cad) for j, link, cad in zip(JOINTS, stall_link_deg, cad_deg)),
        joints=tuple(JointOutcome(j, e, g, TRUTH_GAP_DEG[j]) for j, e, g in zip(JOINTS, errors, gaps)),
    )


def test_nearest_rank_is_the_report_percentile():
    values = list(range(1, 21))  # 20 values: the 95th percentile by nearest rank is the 19th
    assert nearest_rank(values, 95) == 19
    assert nearest_rank([7.0], 95) == 7.0
    assert nearest_rank([3.0, 1.0, 2.0], 50) == 2.0
    with pytest.raises(ValueError):
        nearest_rank([], 95)


def test_reliable_needs_finished_and_every_stall_on_its_stop():
    assert reliable(_episode("a"))
    assert not reliable(_episode("b", finished=False, outcome="aborted: timeout"))
    # one stall 1.5 deg short of the CAD stop it was meant to be on: a contact elsewhere, never a home
    assert not reliable(_episode("c", stall_link_deg=(89.97, 123.5, 197.76)))
    assert reliable(_episode("d", stall_link_deg=(89.97, 124.01, 197.76)))  # 0.99 deg: inside the stall_proof tolerance
    # no stall at all is not a home either
    empty = EpisodeRecord("p", "e", "s", True, "done", 1.0, (), _episode("x").joints)
    assert not reliable(empty)


def test_reliability_counts_every_episode_in_the_denominator():
    episodes = [_episode("a"), _episode("b"), _episode("c", finished=False, outcome="aborted"),
                _episode("d", stall_link_deg=(89.97, 120.0, 197.76))]
    assert reliability_pct(episodes) == pytest.approx(50.0)  # 2 of 4; a metric dropping the unfinished one reads 66.7
    assert reliability_pct([]) is None


def test_time_to_home_is_median_and_p95_over_finished_episodes():
    episodes = [_episode(str(i), time_s=float(t)) for i, t in enumerate(range(1, 21))]
    episodes.append(_episode("dnf", finished=False))
    stat = time_to_home_s(episodes)
    assert stat.n == 20
    assert stat.median == pytest.approx(10.5)
    assert stat.p95 == pytest.approx(19.0)  # nearest rank, never the mean (10.5) or the max (20)
    assert time_to_home_s([_episode("dnf", finished=False)]) is None


def test_joint_angle_error_is_the_worst_joint_p95_over_reliable_episodes():
    episodes = [_episode(str(i), errors=(0.001 * i, -0.002, 0.0)) for i in range(1, 21)]  # worst joint 0.002..0.020
    episodes.append(_episode("off-stop", errors=(5.0, 5.0, 5.0), stall_link_deg=(80.0, 124.99, 197.76)))
    stat = joint_angle_error_deg(episodes)
    assert stat.n == 20  # the unreliable episode's zero is not scored as an accuracy (reliability holds it)
    assert stat.p95 == pytest.approx(0.019)
    assert stat.max == pytest.approx(0.020)
    assert stat.worst_joint == "revolute_3"
    # the sign never helps: a -0.03 error is a 0.03 error
    assert joint_angle_error_deg([_episode("neg", errors=(0.0, -0.03, 0.0))]).p95 == pytest.approx(0.03)


def test_a_joint_left_unzeroed_is_missing_not_perfect():
    episode = _episode("half", errors=(None, 0.001, 0.001))
    stat = joint_angle_error_deg([episode])
    assert stat.missing == 1
    assert any_start_pct([episode], gate_deg=0.01) == 0.0  # an unzeroed joint can never pass the gate


def test_any_start_needs_reliable_and_inside_the_gate():
    episodes = [_episode("in", errors=(0.009, 0.0, 0.0)), _episode("out", errors=(0.011, 0.0, 0.0)),
                _episode("dnf", finished=False), _episode("bad-stall", stall_link_deg=(0.0, 124.99, 197.76))]
    assert any_start_pct(episodes, gate_deg=0.01) == pytest.approx(25.0)


def test_backlash_error_reads_unidentified_as_dash_never_zero():
    nothing = [_episode("a"), _episode("b")]
    stat = backlash_error_deg(nothing)
    assert stat.value is None and stat.observable_pct == 0.0  # '-': a metric reading None as 0 would claim 0 error
    truth = TRUTH_GAP_DEG
    some = [_episode("c", gaps=(truth["revolute_3"] + 0.004, None, truth["revolute_7"] - 0.001)),
            _episode("d", gaps=(truth["revolute_3"], truth["revolute_6"] + 0.002, None))]
    stat = backlash_error_deg(some)
    assert stat.value == pytest.approx(0.004)  # worst drive per episode (0.004, 0.002), p95 by nearest rank of 2 = 0.004
    assert stat.observable_pct == pytest.approx(100.0 * 4 / 6)


def test_backlash_gate_is_one_arcmin_or_a_quarter_of_truth():
    assert backlash_gate_deg(2.5 / 60.0) == pytest.approx(ARCMIN_DEG)  # 25 % of 2.5' is 0.625' < 1'
    assert backlash_gate_deg(8.0 / 60.0) == pytest.approx(2.0 / 60.0)  # 25 % of 8' is 2'
    assert ARCMIN_DEG == pytest.approx(1.0 / 60.0)


def test_scorecard_carries_all_five_and_the_gate_verdicts():
    episodes = [_episode(str(i), errors=(0.001, 0.0, 0.0)) for i in range(4)]
    card = scorecard(episodes, gate_deg=0.01)
    assert set(card) >= {"time_to_home_s", "time_to_home_p95_s", "reliability_pct", "any_start_pct", "backlash_error_deg",
                         "backlash_observable_pct", "joint_angle_error_deg", "joint_angle_error_max_deg", "episodes",
                         "zero_gate_pass", "reliability_gate_pass", "backlash_gate_pass"}
    assert card["reliability_pct"] == 100.0 and card["zero_gate_pass"] is True
    assert card["backlash_error_deg"] is None and card["backlash_gate_pass"] is None  # nothing identified: no verdict, no pass
    worse = card | {}
    assert math.isclose(worse["joint_angle_error_deg"], 0.001)
