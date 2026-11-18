"""The v4 proposal's part declarations: ``RobotPart.metrics`` as aliases over value-state metrics, beside ``.models``.

The rack's gates are OpenSLO occurrence objectives grouped into the scorecard's three verdicts. The same alias
("tip stop error") carries two gates: the scorecard's median (on its value) and Zach's 1 mm on every stop (on its items).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from robot_models import IntegratedServo
from robot_models.ladder import ladders

from robot_metrics import Alias, CodeShare, Collection, Dwell, ErrorStat, Objective, TimeOffset

DEGREES = 180.0 / math.pi
JOINT, COMMAND = ("target", "state"), ("plan", "state")


@dataclass(frozen=True)
class PartType:
    name: str
    models: Any
    metrics: tuple[Alias, ...]

    def collection(self) -> Collection:
        return Collection(self.metrics)

    def table(self) -> str:
        width = max(len(a.name) for a in self.metrics)
        lines = [f"{self.name}.metrics ({len(self.metrics)} aliases over {self.collection().computations} states)"]
        lines += [f"  {a.name:<{width}}  {a.metric!r:<74} [{a.unit or '-'}] {'live' if a.metric.live else 'end of run'}"
                  for a in self.metrics]
        return "\n".join(lines)


def servo_aliases() -> tuple[Alias, ...]:
    return (
        Alias("tracking RMS", ErrorStat(statistic="rms", rows="moving"), JOINT, "deg", DEGREES, "median"),
        Alias("tracking peak", ErrorStat(statistic="max_abs"), JOINT, "deg", DEGREES),
        Alias("stop error p90 (T4)", Dwell(of="error", statistic="p90"), JOINT, "deg", DEGREES),
        Alias("overshoot", Dwell(of="overshoot", statistic="max"), JOINT, "deg", DEGREES),
        Alias("lag", TimeOffset(), COMMAND, "ms", 1e3),
        Alias("limiter share", CodeShare(), JOINT, "%", 100.0),
    )


INTEGRATED_SERVO = PartType("integrated_servo", IntegratedServo, servo_aliases())


def pendulum_arm(tip_length_m: float) -> PartType:
    mm, band = 1e3 * tip_length_m, 1e-3 / tip_length_m
    return PartType(f"1-DOF pendulum arm (tip {tip_length_m * 1e3:g} mm)", ladders()["rigid_body_chain"], servo_aliases() + (
        Alias("tip tracking RMS", ErrorStat(statistic="rms", rows="moving"), JOINT, "mm", mm, "median"),
        Alias("tip stop error", Dwell(of="error", statistic="median"), JOINT, "mm", mm),
        Alias("tip stop worst", Dwell(of="error", statistic="max"), JOINT, "mm", mm),
        Alias("tip settle", Dwell(of="settle", statistic="median", band=band), JOINT, "s", 1.0),
        Alias("all stops settled", Dwell(of="settled", statistic="all", band=band), JOINT, "", 1.0),
        Alias("stops", Dwell(of="count", statistic="count"), JOINT, "", 1.0),
    ))


RIGID_BODY_CHAIN = PartType("rigid_body_chain (bam_fb five-bar)", ladders()["rigid_body_chain"], (
    Alias("j2 tracking RMS", ErrorStat(statistic="rms", joint=0), JOINT, "deg", DEGREES),
    Alias("c1 tracking RMS", ErrorStat(statistic="rms", joint=1), JOINT, "deg", DEGREES),
    Alias("tool tracking RMS", ErrorStat(statistic="rms"), ("tool/target", "tool/state"), "mm", 1e3),
    Alias("tool worst", ErrorStat(statistic="max_abs"), ("tool/target", "tool/state"), "mm", 1e3),
))

AGENT, ZACH = "agent (pendulum_goal)", "Zach 2026-11-11"
VERDICTS = {
    "tracks": (Objective("tracking RMS", "le", 0.5, on="value", owner=AGENT),
               Objective("tracking peak", "le", 1.5, on="value", owner=AGENT),
               Objective("limiter share", "le", 2.0, on="value", owner=AGENT),
               Objective("faults", "le", 0, on="value", owner=AGENT)),
    "tip stop (median)": (Objective("tip stop error", "le", 1.0, on="value", owner=ZACH + " (the scorecard's median)"),
                          Objective("tip settle", "le", 0.3, on="value", owner=AGENT)),
    "tip stop (every stop)": (Objective("tip stop error", "le", 1.0, share=1.0, on="items",
                                        owner="proposed: " + ZACH + " on every stop"),),
}
RUNGS = (0.2, 0.4, 0.6, 0.8, 0.9, 1.0)
