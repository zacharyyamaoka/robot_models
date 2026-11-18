"""The v3 proposal's catalog: what each part type is judged on, as aliases over robot_msgs.metrics' signal math.

This file is where RobotDx's ``catalog.py`` (89 named diagnostics over ~30 stock functions, each with its Limits and
owner) would move: robotparts, beside each part type's ``.models``. ``PartType.metrics`` IS the alias tuple.

    INTEGRATED_SERVO.metrics   the servo at its output angle        INTEGRATED_SERVO.models -> IntegratedServo
    pendulum_arm(tip).metrics  the servo's aliases + the tip's (the lever is the alias's display factor)
    RIGID_BODY_CHAIN.metrics   RootMeanSquare on joints AND on the tool: one base, meaning from the stream
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from robot_models import IntegratedServo
from robot_models.ladder import ladders

from robot_msgs_metrics import (Alias, Check, CodeShare, CrossCorrelationLag, Limits, MaxAbsolute, RootMeanSquare,
                                StopStatistic)

DEGREES = 180.0 / math.pi
SERVO_SIGNALS = {"joint": ("/joint/command", "/joint/state", "hermite")}
CHAIN_SIGNALS = {"joints": ("/joints/command", "/joints/state", "same_clock"),
                 "tool": ("/tool/command", "/tool/state", "same_clock")}


@dataclass(frozen=True)
class PartType:
    name: str
    models: Any
    signals: dict
    metrics: tuple[Alias, ...]
    checks: tuple[Check, ...] = ()

    def table(self) -> str:
        width = max(len(alias.name) for alias in self.metrics)
        lines = [f"{self.name}.metrics ({len(self.metrics)} aliases)"]
        for alias in self.metrics:
            lines.append(f"  {alias.name:<{width}}  {alias.metric!r:<62} {alias.signal:<6} [{alias.unit or '-'}] "
                         f"across runs: {alias.across}")
        return "\n".join(lines)


def servo_aliases() -> tuple[Alias, ...]:
    return (
        Alias("tracking RMS", RootMeanSquare(rows="moving"), "joint", "deg", DEGREES, "median"),
        Alias("tracking peak", MaxAbsolute(), "joint", "deg", DEGREES, "max"),
        Alias("stop error p90 (T4)", StopStatistic(of="error", statistic="p90"), "joint", "deg", DEGREES, "pooled:p90"),
        Alias("overshoot", StopStatistic(of="overshoot", statistic="max"), "joint", "deg", DEGREES, "pooled:max"),
        Alias("lag", CrossCorrelationLag(), "joint", "ms", 1e3, "median"),
        Alias("limiter share", CodeShare(codes=(97, 98, 99, 102)), "joint", "%", 100.0, "pooled:mean"),
    )


INTEGRATED_SERVO = PartType("integrated_servo", IntegratedServo, SERVO_SIGNALS, servo_aliases())


def pendulum_arm(tip_length_m: float) -> PartType:
    mm, band = 1e3 * tip_length_m, 1e-3 / tip_length_m
    tip = (
        Alias("tip tracking RMS", RootMeanSquare(rows="moving"), "joint", "mm", mm, "median"),
        Alias("tip stop error", StopStatistic(of="error", statistic="median"), "joint", "mm", mm, "pooled:median"),
        Alias("tip stop worst", StopStatistic(of="error", statistic="max"), "joint", "mm", mm, "pooled:max"),
        Alias("tip settle", StopStatistic(of="settle", statistic="median", band=band), "joint", "s", 1.0, "pooled:median"),
        Alias("all stops settled", StopStatistic(of="settled", statistic="all", band=band), "joint", "", 1.0, "pooled:all"),
        Alias("stops", StopStatistic(of="count", statistic="count"), "joint", "", 1.0, "pooled:count"),
    )
    zach, agent = "Zach 2026-11-11", "agent (pendulum_goal)"
    checks = (
        Check("tracking RMS", Limits(high=0.5), agent), Check("tracking peak", Limits(high=1.5), agent),
        Check("limiter share", Limits(high=2.0), agent), Check("faults", Limits(high=0), agent),
        Check("tip stop error", Limits(high=1.0), zach), Check("tip settle", Limits(high=0.3), agent),
        Check("tip stop worst", Limits(high=1.0), "proposed"),
    )
    return PartType(f"1-DOF pendulum arm (tip {tip_length_m * 1e3:g} mm)", ladders()["rigid_body_chain"], SERVO_SIGNALS,
                    servo_aliases() + tip, checks)


RIGID_BODY_CHAIN = PartType("rigid_body_chain (bam_fb five-bar)", ladders()["rigid_body_chain"], CHAIN_SIGNALS, (
    Alias("j2 tracking RMS", RootMeanSquare(joint=0), "joints", "deg", DEGREES),
    Alias("c1 tracking RMS", RootMeanSquare(joint=1), "joints", "deg", DEGREES),
    Alias("tool tracking RMS", RootMeanSquare(), "tool", "mm", 1e3),
    Alias("tool worst", MaxAbsolute(), "tool", "mm", 1e3, "max"),
))

# the rung verdicts, as the scorecard's cell gates: each a set of catalog checks
VERDICTS = {"tracks": ("tracking RMS", "tracking peak", "limiter share", "faults"),
            "tip stop (median)": ("tip stop error", "tip settle"),
            "tip stop (every stop)": ("tip stop worst",)}
RUNGS = (0.2, 0.4, 0.6, 0.8, 0.9, 1.0)
