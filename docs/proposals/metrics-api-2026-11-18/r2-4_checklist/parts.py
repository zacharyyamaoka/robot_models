"""Each part type's checklist, beside its models: the proposed home is ``RobotPart.metrics`` next to ``.models``.

    INTEGRATED_SERVO.models     robot_models' model family for the servo (IntegratedServo)
    INTEGRATED_SERVO.metrics    its checklist (a Card): the rows round 8 scores at the servo's output
    PENDULUM_ARM.metrics        the servo's rows plus the tip rows (the joint read through the lever)
    RIGID_CHAIN.metrics         the bam_fb five-bar: each joint's RMS and the tool's position error

The targets are round 8's: tracking is the agent's (pendulum_goal), the tip's 1 mm is Zach's (2026-11-11: "we
certainly need final tool error sub 1mm"); 1 mm on the worst stop instead of the median is proposed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from checklist import (Card, Check, faulted, lag, limiter_share, median_stop_error, overshoot, peak_tracking_error,
                       rms_tracking_error, settle_time, stop_count, worst_stop_error)
from robot_models import IntegratedServo
from robot_models.ladder import ladders

AGENT = "AI agent"
ZACH = "Zach 2026-11-11"
PROPOSED = "proposed, Zach's 1 mm"


@dataclass(frozen=True)
class Part:
    """A part type: its models (robot_models) and its metrics (a checklist), side by side."""

    name: str
    models: Any
    metrics: Card


SERVO_ROWS = [
    Check("tracking RMS", measure=rms_tracking_error(), unit="deg", target="<= 0.5", owner=AGENT),
    Check("tracking peak", measure=peak_tracking_error(), unit="deg", target="<= 1.5", owner=AGENT),
    Check("limiter share", measure=limiter_share(), unit="%", target="<= 2.0", owner=AGENT),
    Check("faulted runs", measure=faulted(), unit="runs", target="== 0", owner=AGENT),
    Check("overshoot", measure=overshoot(), unit="deg"),
    Check("lag", measure=lag(), unit="ms"),
    Check("stops", measure=stop_count(), unit="stops"),
]

TIP_ROWS = [
    Check("median tip stop", measure=median_stop_error(at="tip"), unit="mm", target="<= 1.0", owner=ZACH),
    Check("tip settle", measure=settle_time(at="tip", within_mm=1.0), unit="s", target="<= 0.3", owner=AGENT),
    Check("worst tip stop", measure=worst_stop_error(at="tip"), unit="mm", target="<= 1.0", owner=PROPOSED),
]

CHAIN_ROWS = [
    Check("j2 RMS", measure=rms_tracking_error(joint=0, while_moving=False), unit="deg"),   # joint 0 is j2
    Check("c1 RMS", measure=rms_tracking_error(joint=1, while_moving=False), unit="deg"),
    Check("tool RMS", measure=rms_tracking_error(of="tool", while_moving=False), unit="mm"),
    Check("tool worst", measure=peak_tracking_error(of="tool"), unit="mm"),
]

INTEGRATED_SERVO = Part("integrated servo", IntegratedServo, Card("integrated servo", SERVO_ROWS))
PENDULUM_ARM = Part("pendulum arm", ladders()["rigid_body_chain"], Card("pendulum arm", SERVO_ROWS + TIP_ROWS))
RIGID_CHAIN = Part("five-bar arm", ladders()["rigid_body_chain"], Card("five-bar arm", CHAIN_ROWS))
