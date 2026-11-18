"""The v5 proposal's tree: the rack -> six pendulum arms -> each arm's servo; the five-bar -> its tool and two joints.

Each node is a RobotPart: ``.models`` (robot_models), ``.metrics`` (aliases on ITS signal), ``.children``. The arm's
tip aliases read the servo child's events through the lever in the arm's context: the servo computes, the arm shows.
"""

from __future__ import annotations

import math

from robot_models import IntegratedServo
from robot_models.ladder import ladders

from part_metrics import ERROR, Alias, Check, CodeShare, Lag, Part, StopEvents, TrackingError

DEGREES = 180.0 / math.pi
SERVO_SIGNALS = {"output": ("target", "state"), "command": ("plan", "state")}


def servo_aliases() -> tuple[Alias, ...]:
    return (
        Alias("tracking RMS", TrackingError(statistic="rms"), "output", "deg", DEGREES, across="median"),
        Alias("tracking peak", TrackingError(statistic="max_abs"), "output", "deg", DEGREES, across="max"),
        Alias("stop error p90 (T4)", StopEvents(of="error", statistic="p90"), "output", "deg", DEGREES),
        Alias("overshoot", StopEvents(of="overshoot", statistic="max"), "output", "deg", DEGREES),
        Alias("lag", Lag(), "command", "ms", 1e3, across="median"),
        Alias("limiter share", CodeShare(), "output", "%", 100.0),
    )


def servo(station: str) -> Part:
    return Part(f"{station} servo", IntegratedServo, servo_aliases(), SERVO_SIGNALS,
                checks=(Check("tracking RMS", 0.5, "agent"), Check("tracking peak", 1.5, "agent"),
                        Check("limiter share", 2.0, "agent")))


def pendulum_arm(station: str, tip_length_m: float) -> Part:
    child = f"{station} servo"
    band = 1e-3 / tip_length_m
    tip = (
        Alias("tip tracking RMS", TrackingError(statistic="rms"), "output", "mm", 1.0, "lever_mm", "median", child),
        Alias("tip stop error", StopEvents(of="error", statistic="median"), "output", "mm", 1.0, "lever_mm", "pool", child),
        Alias("tip stop worst", StopEvents(of="error", statistic="max"), "output", "mm", 1.0, "lever_mm", "pool", child),
        Alias("tip settle", StopEvents(of="settle", statistic="median", band=band), "output", "s", 1.0, "", "pool", child),
        Alias("all stops settled", StopEvents(of="settled", statistic="all", band=band), "output", "", 1.0, "", "pool",
              child),
        Alias("stops", StopEvents(of="count", statistic="count"), "output", "", 1.0, "", "pool", child),
    )
    return Part(f"{station} arm", ladders()["rigid_body_chain"], tip, {}, children=(servo(station),),
                context={"lever_mm": 1e3 * tip_length_m, "tip_length_m": tip_length_m},
                checks=(Check("tip stop error", 1.0, "Zach 2026-11-11"), Check("tip settle", 0.3, "agent"),
                        Check("tip stop worst", 1.0, "proposed", level=ERROR)))


def rack(tips: dict[str, float]) -> Part:
    return Part("one-dof-test-rack", None, (), {}, children=tuple(pendulum_arm(s, tip) for s, tip in sorted(tips.items())))


FIVE_BAR = Part("bam_fb five-bar", ladders()["rigid_body_chain"], (
    Alias("tool tracking RMS", TrackingError(statistic="rms"), "tool", "mm", 1e3),
    Alias("tool worst", TrackingError(statistic="max_abs"), "tool", "mm", 1e3, across="max"),
), {"tool": ("tool/target", "tool/state")}, children=tuple(
    Part(f"{joint} servo", IntegratedServo, (
        Alias(f"{joint} tracking RMS", TrackingError(statistic="rms", joint=index), "joints", "deg", DEGREES),),
        {"joints": ("target", "state")}) for index, joint in enumerate(("j2", "c1"))))
RUNGS = (0.2, 0.4, 0.6, 0.8, 0.9, 1.0)
