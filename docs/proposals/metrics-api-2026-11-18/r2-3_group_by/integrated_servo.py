"""The integrated servo: its models, and beside them its metrics, a list of expressions over its recordings.

    import integrated_servo
    integrated_servo.models          # robot_models' family: M0.ideal ... M7.learned
    integrated_servo.metrics         # what is measured in a recording, and how each is summarised

A metric here is a measure (what is read from a recording, with its unit) and a summary (how its values are pooled).
Adding one is one constructor call and one line in the list; no registry.
"""

from __future__ import annotations

from robot_models import IntegratedServo

from measures import ChannelCode, RowError, RunFlag, StopValue, TimeOffset

COMPARE = "robot_msgs.compare (the error, reference - measured, on the measured rows)"
STOPS = "bam_rack_twin pendulum_score.stops / RobotDx T4 stop_accuracy (round 1's stock.StopTracker)"

models = IntegratedServo

tracking_error = RowError(
    "tracking error", "deg", short="tracking", rows="moving",
    description="the angle between measured and target on each row where the target moves",
    references=(COMPARE, "the round-8 scorecard's tracking RMS (align: the plan at each reply's instant)"))

joint_error = RowError(
    "joint error", "deg", rows="all",
    description="the angle between measured and target on every row, moving or still",
    references=(COMPARE, "the round-8 scorecard's tracking peak"))

stop_error = StopValue(
    "stop error", "deg", of="error", at="joint",
    description="each stop's angle error, averaged over the dwell's last 50 ms",
    references=(STOPS, "ISO 9283:1998 7.2 (pose accuracy at a stop)"))

overshoot = StopValue(
    "overshoot", "deg", of="overshoot", at="joint",
    description="how far each move ran past its stop's target",
    references=("robot_msgs.compare.goal (overshoot)", STOPS))

lag = TimeOffset(
    "lag", "ms",
    description="how far the response runs behind the command, one value per run",
    references=("robot_msgs.compare.estimate_time_offset", "evo associate_trajectories"))

limited_row = ChannelCode(
    "limited row", "", description="1 when the drive's limiter was active on the row (codes 97, 98, 99, 102)",
    references=("moteus fault register: limiter codes 97 / 98 / 99 / 102",))

faulted = RunFlag(
    "faulted", "", short="faulted run", attribute="faulted",
    description="1 when the runner stopped the run before it completed",
    references=("the runner's session.json outcome",))

torque_residual = RowError(
    "torque residual", "N·m", field="effort", rows="all", scale=1.0,
    description="the torque the feedback still supplies on each row: measured minus the feed-forward",
    references=(COMPARE, "Zach, 2026-10-02: a smaller feedback term"))

metrics = [
    tracking_error.rms().per_run().median(),
    joint_error.max(),
    stop_error.percentile(90),
    overshoot.max(if_none=0.0),
    lag.median(),
    limited_row.share(),
]
