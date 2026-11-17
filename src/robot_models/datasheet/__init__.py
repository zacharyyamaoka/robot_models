"""robot_models.datasheet: typed part fields, and a part's values resolved from its raw sources (PEP 0003).

    from robot_models.datasheet import Source, Stated          # what a sources/<name>.py writes
    from robot_models.datasheet import Sources                 # what a part file loads
    from robot_models.actuators.servo import IntegratedServo   # a type: its typed fields

Stdlib only, like the rest of robot_models' core.
"""

from . import literal, units
from .datasheet import (
    ELECTRICAL,
    EXTRA,
    GROUPS,
    INTERFACES,
    KEY_SPECS,
    MECHANICAL,
    PERFORMANCE,
    Datasheet,
    Disagreement,
    Field,
    FieldSpec,
    check_source,
    check_value,
    reflected,
)
from .source import KINDS, ColumnError, Source, Sources, load_source
from .stated import Stated

__all__ = [
    "ELECTRICAL",
    "EXTRA",
    "GROUPS",
    "INTERFACES",
    "KEY_SPECS",
    "KINDS",
    "MECHANICAL",
    "PERFORMANCE",
    "ColumnError",
    "Datasheet",
    "Disagreement",
    "Field",
    "FieldSpec",
    "Source",
    "Sources",
    "Stated",
    "check_source",
    "check_value",
    "literal",
    "load_source",
    "reflected",
    "units",
]
