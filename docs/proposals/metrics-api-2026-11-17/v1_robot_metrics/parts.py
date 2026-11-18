"""How robotparts would declare a part type's metrics beside its models (the v1 proposal's ``RobotPart.metrics``).

    INTEGRATED_SERVO.models      -> robot_models.IntegratedServo      (a ModelFamily)
    INTEGRATED_SERVO.metrics     -> (Alias, ...)                      configured stock metrics, named
    pendulum_arm(tip).metrics    -> the servo's aliases + the tip's    (the lever is the alias's display factor)
    RIGID_BODY_CHAIN.metrics     -> joint and tool aliases over the SAME ErrorStat base

WHY the lever is a display factor and not a new stream: the tip moves tip_length x the joint angle exactly, so "tip
stop error" in mm and "stop error" in degrees are one computation shown two ways (G7: units belong to the alias), and
every gate still reads the joint encoders (Zach, 2026-11-11). A part whose output is not a linear map of a joint (the
chain's tool, through FK) is given its own stream instead.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from robot_models import IntegratedServo
from robot_models.ladder import ladders

from robot_metrics import (DEGREES, Alias, AllSettled, CodeShare, Diagnostic, DwellError, ErrorStat, Gate, Limits,
                           MetricCollection, Overshoot, SettleTime, TimeOffset)

SERVO_SIGNALS = {"joint": ("target", "state"), "command": ("plan", "state")}
CHAIN_SIGNALS = {"joints": ("target", "state"), "tool": ("tool/target", "tool/state")}


@dataclass(frozen=True)
class PartType:
    name: str
    models: Any
    signals: Mapping[str, tuple[str, str]]
    metrics: tuple[Alias, ...]
    contains: tuple[str, ...] = ()

    def collection(self) -> MetricCollection:
        return MetricCollection(self.metrics, self.signals)

    def table(self) -> str:
        width = max(len(alias.name) for alias in self.metrics)
        lines = [f"{self.name}: {len(self.metrics)} metrics · models: {_models(self.models)}"]
        for alias in self.metrics:
            lines.append(f"  {alias.name:<{width}}  {alias.metric!r:<44} on {alias.signal:<7} [{alias.unit or '-'}]"
                         + (f"  across runs: {alias.across_runs}" if alias.across_runs != "pool" else ""))
        return "\n".join(lines)


def _models(models: Any) -> str:
    """A ModelFamily by its name and size; a v1 Ladder (a type with no family yet) by its type and rung count."""

    keys = getattr(models, "keys", None)
    if callable(keys):
        return f"{getattr(models, '__name__', type(models).__name__)} ({len(keys())} models)"
    return f"Ladder {models.type!r} ({len(models.rungs)} rungs, no ModelFamily yet)"


def servo_metrics() -> tuple[Alias, ...]:
    """What an integrated servo is judged on at its output angle (the note's servo table and generic rows)."""

    return (
        Alias("tracking RMS", ErrorStat(statistic="rms", rows="moving"), "joint", "deg", DEGREES, "median",
              owner="Zach 2026-10-30: 'a simple one for actuators ... is tracking error'"),
        Alias("tracking peak", ErrorStat(statistic="max_abs"), "joint", "deg", DEGREES),
        Alias("stop error p90 (T4)", DwellError(statistic="p90"), "joint", "deg", DEGREES, owner="bam_rack_twin T4"),
        Alias("overshoot", Overshoot(), "joint", "deg", DEGREES),
        Alias("lag", TimeOffset(), "command", "ms", 1e3),
        Alias("limiter share", CodeShare(codes=(97, 98, 99, 102)), "joint", "%", 100.0),
    )


INTEGRATED_SERVO = PartType("integrated_servo", IntegratedServo, SERVO_SIGNALS, servo_metrics())


def pendulum_arm(tip_length_m: float) -> PartType:
    """A 1-DOF pendulum station: the servo as mounted, judged at its tip (the note's Q2 default: a 1-joint chain)."""

    mm = 1e3 * tip_length_m           # display: radians at the joint -> millimetres at the tip
    band = 1e-3 / tip_length_m        # 1 mm at the tip, in the joint's radians (the config is in the signal's unit)
    tip = (
        Alias("tip tracking RMS", ErrorStat(statistic="rms", rows="moving"), "joint", "mm", mm, "median"),
        Alias("tip stop error", DwellError(statistic="median"), "joint", "mm", mm, owner="pendulum_score (median)"),
        Alias("tip stop worst", DwellError(statistic="max"), "joint", "mm", mm),
        Alias("tip settle", SettleTime(band=band), "joint", "s", 1.0),
        Alias("all stops settled", AllSettled(band=band), "joint", "", 1.0),
        Alias("stops", DwellError(statistic="count"), "joint", "", 1.0),
    )
    return PartType(f"1-DOF pendulum arm (tip {tip_length_m * 1e3:g} mm)", ladders()["rigid_body_chain"], SERVO_SIGNALS,
                    servo_metrics() + tip, contains=("integrated_servo",))


RIGID_BODY_CHAIN = PartType("rigid_body_chain (bam_fb five-bar)", ladders()["rigid_body_chain"], CHAIN_SIGNALS, (
    Alias("j2 tracking RMS", ErrorStat(statistic="rms", joint=0), "joints", "deg", DEGREES),
    Alias("c1 tracking RMS", ErrorStat(statistic="rms", joint=1), "joints", "deg", DEGREES),
    Alias("tool tracking RMS", ErrorStat(statistic="rms"), "tool", "mm", 1e3),
    Alias("tool worst", ErrorStat(statistic="max_abs"), "tool", "mm", 1e3),
), contains=("integrated_servo", "integrated_servo"))


# The rack's gates (pendulum_goal's cell gates; the 1 mm is Zach's, 2026-11-11). Targets never enter compute().
GATES = (
    Gate("tracks", (Diagnostic("tracking RMS", Limits.at_most(0.5), "agent"),
                    Diagnostic("tracking peak", Limits.at_most(1.5), "agent"),
                    Diagnostic("limiter share", Limits.at_most(2.0), "agent"),
                    Diagnostic("faults", Limits.at_most(0), "agent"))),
    Gate("tip stop (median)", (Diagnostic("tip stop error", Limits.at_most(1.0), "Zach 2026-11-11"),
                               Diagnostic("tip settle", Limits.at_most(0.3), "agent"))),
    Gate("tip stop (every stop)", (Diagnostic("tip stop worst", Limits.at_most(1.0), "proposed"),)),
)
RUNGS = (0.2, 0.4, 0.6, 0.8, 0.9, 1.0)
assert math.isclose(DEGREES, 57.29577951308232)
