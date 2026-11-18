"""robot_msgs.metrics (the v3 proposal): signal math only, beside robot_msgs.compare; meaning is bound by aliases.

    from robot_msgs.metrics import RootMeanSquare, StopStatistic, Aligner, Plan, aggregate
    Aligner()                         # pairs a command topic and a state topic LIVE: the plan at each reply's instant
    Plan.compile(part.aliases)        # aliases -> unique computations (signal, base, config); stops found once
    plan.stream(recording.messages()) # live, message by message; plan.run(recording) offline: the same code path
    aggregate(results, alias)         # across runs: rliable-style point estimate with a stratified bootstrap CI

Layering: robot_msgs (+ .metrics) <- robotparts' catalog (aliases, Limits, owners: RobotDx's catalog.py moved) <-
sim2real / RobotDx. Nothing in robot_msgs.metrics knows a part type, a unit for display, or a threshold.
"""

from .aggregate import Estimate, aggregate, iqm
from .align import Aligner, Recording
from .metric import (CodeShare, CrossCorrelationLag, HeldOutPair, MaxAbsolute, Metric, PerformanceGain, Repeat,
                     Repeatability, RootMeanSquare, StopStatistic, Stops)
from .plan import Alias, Check, Limits, Plan

__all__ = ["HeldOutPair", "PerformanceGain", "Repeat", "Repeatability", "Aligner", "Alias", "Check", "CodeShare", "CrossCorrelationLag", "Estimate", "Limits", "MaxAbsolute", "Metric",
           "Plan", "Recording", "RootMeanSquare", "StopStatistic", "Stops", "aggregate", "iqm"]
