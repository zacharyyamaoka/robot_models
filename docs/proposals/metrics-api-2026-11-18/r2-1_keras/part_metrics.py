"""Each part type's metrics, one function per part type, written the way a part type lists its models.

    integrated_servo_metrics()                 -> MetricDict   what a servo is judged on at its output angle
    pendulum_arm_metrics(tip_length_m=0.339)   -> MetricDict   a 1-DOF pendulum station, judged at its tip
    rigid_body_chain_metrics()                 -> {"joints": MetricDict, "tool": MetricDict}   the five-bar
    PART_TYPES                                 -> {part type: {"models": ..., "metrics": the function}}

Where it would live: beside the models in robot_models (``IntegratedServo.metrics`` next to ``IntegratedServo``'s 14
models), each entry a configured metric from ``metrics.py`` (which would be robot_metrics). Here it is a proposal
module, so nothing in src/ changes until Zach picks.

WHY the tip is a ``scale`` and not a new stream: the tip moves tip_length x the joint angle exactly, so "tip stop
worst" in mm and a stop error in degrees are one computation shown two ways, and every gate still reads the joint
encoders (Zach, 2026-11-11). The chain's tool is not a linear map of one joint (it goes through FK), so it gets its
own streams instead.
"""

from __future__ import annotations

import math

from robot_models import IntegratedServo
from robot_models.ladder import ladders

from metrics import (AllStopsSettled, CodeShare, Lag, MetricDict, Overshoot, SettleTime, StopCount, StopError,
                     TrackingError)

DEGREES = math.degrees(1.0)          # radians -> degrees, for display only
LIMITER_CODES = (97, 98, 99, 102)    # moteus: max power, back-EMF, current, torque cap


def integrated_servo_metrics() -> MetricDict:
    """The servo at its output angle: tracking, stops, overshoot, lag behind the plan, time spent on a limiter."""

    return MetricDict([
        TrackingError("tracking RMS", statistic="rms", while_moving=True, unit="deg", scale=DEGREES),
        TrackingError("tracking peak", statistic="peak", unit="deg", scale=DEGREES),
        StopError("stop error p90", statistic="p90", unit="deg", scale=DEGREES),
        Overshoot("overshoot", unit="deg", scale=DEGREES),
        Lag("lag", unit="ms", scale=1000.0),
        CodeShare("limiter share", codes=LIMITER_CODES, unit="%", scale=100.0),
    ])


def pendulum_arm_metrics(tip_length_m: float) -> MetricDict:
    """A pendulum station: the servo's tracking, and its stops seen at the tip, in mm."""

    mm_at_tip = 1000.0 * tip_length_m      # mm at the tip per radian at the joint
    return MetricDict([
        TrackingError("tracking RMS", statistic="rms", while_moving=True, unit="deg", scale=DEGREES),
        TrackingError("tracking peak", statistic="peak", unit="deg", scale=DEGREES),
        StopError("tip stop median", statistic="median", unit="mm", scale=mm_at_tip),
        StopError("tip stop worst", statistic="worst", unit="mm", scale=mm_at_tip),
        SettleTime("tip settle median", within=1.0, within_unit="mm", scale=mm_at_tip),
        AllStopsSettled("all stops settled", within=1.0, within_unit="mm", scale=mm_at_tip),
        StopCount("stops"),
    ])


def rigid_body_chain_metrics() -> dict[str, MetricDict]:
    """The five-bar: each joint's tracking (target vs measured), the tool's (tool target vs tool measured)."""

    return {
        "joints": MetricDict([
            TrackingError("j2 tracking RMS", statistic="rms", joint=0, unit="deg", scale=DEGREES),
            TrackingError("c1 tracking RMS", statistic="rms", joint=1, unit="deg", scale=DEGREES),
        ]),
        "tool": MetricDict([
            TrackingError("tool tracking RMS", statistic="rms", unit="mm", scale=1000.0),
            TrackingError("tool worst", statistic="peak", unit="mm", scale=1000.0),
        ]),
    }


PART_TYPES = {
    "integrated_servo": {"models": IntegratedServo, "metrics": integrated_servo_metrics},
    "pendulum_arm": {"models": ladders()["rigid_body_chain"], "metrics": pendulum_arm_metrics},
    "rigid_body_chain": {"models": ladders()["rigid_body_chain"], "metrics": rigid_body_chain_metrics},
}
