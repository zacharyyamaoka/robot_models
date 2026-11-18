"""robot_metrics (the v4 proposal): the prior-art note's recommended core, grown to the scenario.

A metric is a frozen config whose state is a value (empty / update / merge / compute / close); ``Stream`` is the
torchmetrics face. Lives in io-diagnostics beside RobotDx (the metric half of robotdx.diagnostics lifted out), on
robot_msgs only; RobotDx keeps Limits / judge / Result / records and judges these values.
"""

from .collection import Alias, Collection, Objective, Run, rdx, summarize, wilson
from .core import Metric, Stream
from .metrics import (CodeShare, Dwell, DwellState, ErrorStat, HeldOutPair, PerformanceGain, Repeat, Repeatability,
                      TimeOffset)

__all__ = ["Alias", "CodeShare", "Collection", "Dwell", "DwellState", "ErrorStat", "HeldOutPair", "Metric", "Objective",
           "PerformanceGain", "Repeat", "Repeatability", "Run", "Stream", "TimeOffset", "rdx", "summarize", "wilson"]
