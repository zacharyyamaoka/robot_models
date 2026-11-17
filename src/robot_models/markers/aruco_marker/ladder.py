"""A printed sheet carrying one or more ArUco markers rigidly: its ladder of models (PEP 0004).

The ladder places the sheet's marker corners in the world. M0 reads the sheet as CAD designed it, M1 adds how it was
actually stuck down, M2 how far the print is from its designed size, and M3 that the sheet is not flat.

Spec: PEP 0004 §5.13 and §6.12.
"""

from __future__ import annotations

from robot_models import references
from robot_models.ladder import Alias, Ladder, Parameter, Rung

# --- the vocabulary, grouped by the rung that first reads it -----------------------------------------------------------

# M0.nominal: the sheet as designed
DICTIONARY = Parameter(
    name="dictionary",
    unit="",
    definition="The ArUco dictionary the sheet is printed in.",
    shape="text",
)
MARKER_IDS = Parameter(
    name="marker_ids",
    unit="",
    definition="The marker ids the sheet carries.",
    shape="list",
)
MARKER_LENGTH = Parameter(
    name="marker_length",
    unit="m",
    definition="Side of each marker's black square.",
)
LAYOUT = Parameter(
    name="layout",
    unit="m",
    definition="Each marker's corners in the sheet's own frame.",
    shape="map",
)
POSE_ON_LINK = Parameter(
    name="pose_on_link",
    unit="m, rad",
    definition="The sheet's frame on the link it is stuck to.",
    shape="pose",
)

# M1.as_built: how it was stuck down
AS_BUILT_OFFSET = Parameter(
    name="as_built_offset",
    unit="m, rad",
    definition="How the sheet was actually stuck down: its offset from pose_on_link.",
    shape="pose",
)

# M2.print_scale: the print against the design
PRINT_SCALE = Parameter(
    name="print_scale",
    unit="",
    definition="Printed size over designed size.",
)

# M3.curl: a sheet that is not flat
FLATNESS = Parameter(
    name="flatness",
    unit="m",
    definition="The sheet's out-of-plane bow at its far end.",
)

VOCABULARY = (
    DICTIONARY,
    MARKER_IDS,
    MARKER_LENGTH,
    LAYOUT,
    POSE_ON_LINK,
    AS_BUILT_OFFSET,
    PRINT_SCALE,
    FLATNESS,
)

# --- the rungs, M0 upward ----------------------------------------------------------------------------------------------

# WHY each rung adds one thing over the last (PEP 0004 §5.13): the fallback chain is the order a sheet is trusted less
# and fitted more, so a sheet with no measured offset still reaches M0 and places its corners where CAD put them.
RUNGS = (
    Rung(
        level=0,
        name="nominal",
        title="The sheet as designed, where CAD put it",
        base=None,
        adds=("dictionary", "marker_ids", "marker_length", "layout", "pose_on_link"),
        equation="X_corner = FK(q) · pose_on_link · layout_corner",
        references=(references.ZHANG_2000, references.GARRIDO_2014),
    ),
    Rung(
        level=1,
        name="as_built",
        title="How the sheet was actually stuck down",
        base="M0.nominal",
        adds=("as_built_offset",),
        equation="X_corner = FK(q) · pose_on_link · as_built_offset · layout_corner",
        references=(references.TRIGGS_2000,),
    ),
    Rung(
        level=2,
        name="print_scale",
        title="Print scale",
        base="M1.as_built",
        adds=("print_scale",),
        equation="X_corner = FK(q) · pose_on_link · as_built_offset · (print_scale · layout_corner)",
        references=(references.STROBL_2011,),
    ),
    Rung(
        level=3,
        name="curl",
        title="A sheet that is not flat",
        base="M2.print_scale",
        adds=("flatness",),
        equation="layout_corner z = flatness·(2x/L)² (a bow across the sheet's length L)",
        references=(references.STROBL_2011,),
    ),
)

# --- the other names ---------------------------------------------------------------------------------------------------

# WHY plain aliases with no note or unit (PEP 0004 §6.12): each is the canonical quantity one for one, only spelled with
# the SI unit its sim2real card wrote in the name.
ALIASES = (
    Alias(name="marker_length_m", canonical="marker_length"),
    Alias(name="layout_m", canonical="layout"),
    Alias(name="flatness_m", canonical="flatness"),
)

# WHY no packed or deployment-only names (PEP 0004 §5.13, §7): the sheet's quantities are all its own and none packs
# several into one array.

LADDER = Ladder(
    type="aruco_marker",
    title="A printed sheet carrying one or more ArUco markers rigidly.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
)
