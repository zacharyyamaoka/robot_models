"""Each part type's metrics: a plain list of functions, next to the models of the same part type.

    INTEGRATED_SERVO_MODELS, INTEGRATED_SERVO_METRICS     the servo on its own: tracking, overshoot, lag, limits, faults
    PENDULUM_ARM_MODELS, PENDULUM_ARM_METRICS             the 1-DOF pendulum: tracking, and its tip at the stops
    RIGID_BODY_CHAIN_MODELS, RIGID_BODY_CHAIN_METRICS     the five-bar: each joint, and the tool at the L2 tip

WHY plain lists (no registry, no class): adding a metric to a part is adding one name to its list, and a reader sees
every metric the part has in one place. The lists hold no thresholds; gates.py does. In robot_models these lines would
sit in each part type's module beside its ModelFamily (``RobotPart.metrics`` beside ``.models``).
"""

from __future__ import annotations

from robot_models import IntegratedServo
from robot_models.ladder import ladders

from run_metrics import (all_stops_settled, c1_tracking_rms_deg, faulted_runs, j2_tracking_rms_deg, lag_ms,
                         limited_rows_pct, median_tip_stop_mm, overshoot_deg, stop_count, tip_settle_s,
                         tool_tracking_rms_mm, tracking_peak_deg, tracking_rms_deg, worst_tip_stop_mm,
                         worst_tool_error_mm)

INTEGRATED_SERVO_MODELS = IntegratedServo
INTEGRATED_SERVO_METRICS = [
    tracking_rms_deg,
    tracking_peak_deg,
    overshoot_deg,
    limited_rows_pct,
    faulted_runs,
    lag_ms,
]

PENDULUM_ARM_MODELS = ladders()["rigid_body_chain"]
PENDULUM_ARM_METRICS = [
    tracking_rms_deg,
    tracking_peak_deg,
    median_tip_stop_mm,
    worst_tip_stop_mm,
    tip_settle_s,
    all_stops_settled,
    stop_count,
]

RIGID_BODY_CHAIN_MODELS = ladders()["rigid_body_chain"]
RIGID_BODY_CHAIN_METRICS = [
    j2_tracking_rms_deg,
    c1_tracking_rms_deg,
    tool_tracking_rms_mm,
    worst_tool_error_mm,
]
