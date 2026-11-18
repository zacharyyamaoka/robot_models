"""robot_metrics: torchmetrics' shape on numpy, over robot_msgs streams (v1 proposal; production home: its own repo).

    from robot_metrics import ErrorStat, DwellError, Alias, MetricCollection, Diagnostic, Limits, Gate

Layering: robot_msgs <- robot_metrics <- robotparts (a part type's ``.metrics`` lists configured aliases, beside its
``.models``). robot_models never imports it: models and metrics are both read through the part.
"""

from .base import (AllSettled, CodeShare, DwellError, DwellMetric, Dwells, ErrorStat, HeldOutPair, Overshoot,
                   PerformanceGain, PerRun, Repeat, Repeatability, SettleTime, TimeOffset)
from .collection import DEGREES, Alias, MetricCollection, RunRecording
from .diagnostic import Diagnostic, Gate, Limits, Verdict, judge
from .metric import Metric

__all__ = ["HeldOutPair", "PerformanceGain", "Repeat", "Repeatability", "AllSettled", "Alias", "CodeShare", "DEGREES", "Diagnostic", "DwellError", "DwellMetric", "Dwells",
           "ErrorStat", "Gate", "Limits", "Metric", "MetricCollection", "Overshoot", "PerRun", "RunRecording",
           "SettleTime", "TimeOffset", "Verdict", "judge"]
