"""robot_models.metrics (the v2 proposal): torchmetrics' shape, numpy inside, declared on each part type's ModelFamily.

    from robot_models import IntegratedServo
    IntegratedServo.models  -> the family's models (it IS the ModelFamily)      IntegratedServo.metrics -> Metrics(...)

Layering: robot_msgs <- robot_models (models AND metrics, per type) <- robotparts (a RobotPart reads both from its
family) <- sim2real / RobotDx (consumers: gates, pages, records).
"""

from .base import (STOP_PASSES, CodeShare, Dwell, ErrorStat, HeldOutPair, PerformanceGain, Repeat, Repeatability,
                   TimeOffset)
from .metric import MEMO, Memo, Metric
from .metrics import DEGREES, Alias, Deployment, Episode, Gate, Metrics, at_most, view

__all__ = ["HeldOutPair", "PerformanceGain", "Repeat", "Repeatability", "Alias", "CodeShare", "DEGREES", "Deployment", "Dwell", "Episode", "ErrorStat", "Gate", "MEMO", "Memo",
           "Metric", "Metrics", "STOP_PASSES", "TimeOffset", "at_most", "view"]
