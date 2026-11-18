"""robotparts.metrics (the v5 proposal): metrics evaluated by the part tree, over segment events.

Layering: robot_msgs <- robotparts (RobotPart: .models from robot_models, .metrics here, .children for assemblies) <-
sim2real / RobotDx. The deployment is a tree of parts; a node judges its own output and reads its children's.
"""

from .metric import (CodeShare, Event, Lag, Metric, PerformanceGain, Repeatability, Segmenter, StopEvents,
                     TrackingError)
from .tree import ERROR, LEVEL, OK, STALE, WARN, Alias, Check, Part

__all__ = ["Alias", "Check", "CodeShare", "ERROR", "Event", "LEVEL", "Lag", "Metric", "OK", "Part", "PerformanceGain",
           "Repeatability", "STALE", "Segmenter", "StopEvents", "TrackingError", "WARN"]
