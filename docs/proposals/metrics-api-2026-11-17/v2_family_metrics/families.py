"""The v2 proposal's declarations: each part type's metrics on its ModelFamily, beside its models.

In production these lines are the ``metrics = Metrics(...)`` attribute in ``integrated_servo_family.py``'s class body
(next to ``class Ideal``, ``class Stribeck`` ...). Here, adds-only, they are attached to the real ``IntegratedServo`` at
import. A type with no ModelFamily yet (``rigid_body_chain``) is keyed by its type id until it gets one.

The 1-DOF pendulum arm is a 1-joint rigid_body_chain judged at its tip (the vault note's Q2 default): its metrics are
the servo's plus the tip's, the lever a display factor of the alias (tip = tip_length x joint angle exactly).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from robot_models import IntegratedServo

from robot_models_metrics import (DEGREES, Alias, CodeShare, Deployment, Dwell, ErrorStat, Gate, Metric, Metrics,
                                  TimeOffset, at_most)

SERVO_SIGNALS = {"joint": ("target", "state"), "command": ("plan", "state")}
CHAIN_SIGNALS = {"joints": ("target", "state"), "tool": ("tool/target", "tool/state")}

IntegratedServo.metrics = Metrics(
    Alias("tracking RMS", ErrorStat(statistic="rms", rows="moving", across_runs="median"), "joint", "deg", DEGREES,
          owner="Zach 2026-10-30"),
    Alias("tracking peak", ErrorStat(statistic="max_abs", across_runs="max"), "joint", "deg", DEGREES),
    Alias("stop error p90 (T4)", Dwell(quantity="error", statistic="p90"), "joint", "deg", DEGREES),
    Alias("overshoot", Dwell(quantity="overshoot", statistic="max"), "joint", "deg", DEGREES),
    Alias("lag", TimeOffset(across_runs="median"), "command", "ms", 1e3),
    Alias("limiter share", CodeShare(codes=(97, 98, 99, 102)), "joint", "%", 100.0),
    signals=SERVO_SIGNALS,
)

RIGID_BODY_CHAIN_METRICS = Metrics(
    Alias("j2 tracking RMS", ErrorStat(statistic="rms", joint=0), "joints", "deg", DEGREES),
    Alias("c1 tracking RMS", ErrorStat(statistic="rms", joint=1), "joints", "deg", DEGREES),
    Alias("tool tracking RMS", ErrorStat(statistic="rms"), "tool", "mm", 1e3),
    Alias("tool worst", ErrorStat(statistic="max_abs", across_runs="max"), "tool", "mm", 1e3),
    signals=CHAIN_SIGNALS,
)
METRICS_BY_TYPE = {"integrated_servo": IntegratedServo.metrics, "rigid_body_chain": RIGID_BODY_CHAIN_METRICS}


def pendulum_arm(tip_length_m: float) -> Metrics:
    mm, band = 1e3 * tip_length_m, 1e-3 / tip_length_m
    return IntegratedServo.metrics + (
        Alias("tip tracking RMS", ErrorStat(statistic="rms", rows="moving", across_runs="median"), "joint", "mm", mm),
        Alias("tip stop error", Dwell(quantity="error", statistic="median"), "joint", "mm", mm),
        Alias("tip stop worst", Dwell(quantity="error", statistic="max"), "joint", "mm", mm),
        Alias("tip settle", Dwell(quantity="settle", statistic="median", band=band), "joint", "s", 1.0),
        Alias("all stops settled", Dwell(quantity="settled", statistic="all", band=band), "joint", "", 1.0),
        Alias("stops", Dwell(quantity="count", statistic="sum"), "joint", "", 1.0),
    )


ZACH = "Zach 2026-11-11 ('final tool error sub 1mm')"
AGENT = "agent (pendulum_goal.Thresholds)"
DEPLOYMENT = Deployment("one-dof-test-rack", verdicts={
    "tracks": (Gate("tracking RMS", at_most(0.5, "deg", AGENT)), Gate("tracking peak", at_most(1.5, "deg", AGENT)),
               Gate("limiter share", at_most(2.0, "%", AGENT)), Gate("faults", at_most(0, "", AGENT))),
    "tip stop (median)": (Gate("tip stop error", at_most(1.0, "mm", ZACH)), Gate("tip settle", at_most(0.3, "s", AGENT))),
    "tip stop (every stop)": (Gate("tip stop worst", at_most(1.0, "mm", "proposed: Zach's 1 mm on every stop")),),
})
RUNGS = (0.2, 0.4, 0.6, 0.8, 0.9, 1.0)


@dataclass(frozen=True)
class FeedbackShareConfig:
    rows: str = "moving"


class FeedbackShare(Metric):
    """What config cannot say, so a subclass: RMS(effort - feed-forward) / RMS(effort), two channels of one stream."""

    name = "feedback share"
    description = "How much of the torque the feedback still supplies: RMS(tau - tau_ff) / RMS(tau)."
    references = ("bam_rack_twin vt_kpi.feedback_share", "Kawato: feedback-error learning")
    Config = FeedbackShareConfig

    @classmethod
    def functional(cls, config, runs):
        shares = []
        for target, measured in runs:
            tau, ff = measured.effort[:, 0], measured.channels["ff_nm"]
            keep = np.isfinite(tau) & np.isfinite(ff) & (np.abs(target.velocities[:, 0]) >= 1e-9)
            shares.append(float(np.sqrt(np.mean((tau[keep] - ff[keep]) ** 2)) / np.sqrt(np.mean(tau[keep] ** 2))))
        return float(np.median(shares))
