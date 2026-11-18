"""The 1-DOF pendulum arm (an integrated servo turning a lever): its models, and beside them its metrics.

    from pendulum_arm import tip_stop_error, tracking_error      # what is measured, each with its unit
    import pendulum_arm
    pendulum_arm.metrics                                         # the arm's list of summarised metrics

Each tip number is the joint's angle error times the run's lever length (``Run.tip_length_m``, a fact of the station),
so one measure serves all six stations.
"""

from __future__ import annotations

from robot_models.ladder import ladders

from integrated_servo import STOPS, faulted, joint_error, lag, limited_row, overshoot, tracking_error
from measures import StopValue, card as metric_card

__all__ = ["faulted", "joint_error", "lag", "limited_row", "metrics", "models", "overshoot", "stop_settled",
           "tip_settle_time", "tip_stop_error", "tip_stop_offset", "tracking_error"]

models = ladders()["rigid_body_chain"]     # a servo plus one rigid link

tip_stop_error = StopValue(
    "tip stop error", "mm", short="tip stop", of="error", at="tip",
    description="how far the tip ended from its target at each stop, averaged over the stop's last 50 ms",
    references=(STOPS, "ISO 9283:1998 7.2 (pose accuracy at a stop)"))

tip_stop_offset = StopValue(
    "tip stop offset", "mm", short="tip stop", of="offset", at="tip",
    description="each stop's error at the tip with its sign (measured minus target), for repeatability",
    references=(STOPS, "ISO 9283:1998 7.2.2 (pose repeatability)"))

tip_settle_time = StopValue(
    "tip settle time", "s", short="tip settle", of="settle", at="tip", band=1.0,
    description="seconds from each move's end until the tip stays within 1 mm of the target",
    references=(STOPS,))

stop_settled = StopValue(
    "stop settled", "", of="settled", at="tip", band=1.0,
    description="1 when the tip was within 1 mm of the target by the end of the stop's dwell",
    references=(STOPS,))

metrics = [
    tracking_error.rms().per_run().median(),
    joint_error.max(),
    tip_stop_error.median(),
    tip_stop_error.max(),
    tip_settle_time.median(),
    stop_settled.all(),
    tip_stop_error.count(),
    overshoot.max(if_none=0.0),
    lag.median(),
    limited_row.share(),
    faulted.any(),
]


def card(name: str) -> dict:
    """Zach's Metric fields for one of the arm's metrics, by its column name."""

    return metric_card(next(metric for metric in metrics if metric.name == name))
