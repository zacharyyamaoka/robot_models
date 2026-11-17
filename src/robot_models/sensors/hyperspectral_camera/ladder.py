"""A line-scan spectral camera over a belt: its ladder of models (PEP 0004).

Each frame is one line of the belt, split into spectral bands; the belt's motion sweeps the lines into a cube. The
ladder starts at the bands and the references that turn counts into reflectance, then adds the optics' distortion, the
sweep geometry, and the white reference's drift over deployment hours.

Spec: PEP 0004 §5.9 and §6.8.
"""

from __future__ import annotations

from robot_models import references
from robot_models.ladder import Alias, Ladder, Packed, Parameter, Rung, Wear

# --- vocabulary, grouped by the rung that first reads it -------------------------------------------------------------

VOCABULARY = (
    # M0.bands
    Parameter(
        name="band_centres",
        unit="m",
        definition="Centre wavelength of each spectral band.",
        shape="list",
    ),
    # WHY two frames, not one pair: they drift apart (lamp ageing lowers the white frame, dark current raises the dark
    # one), so M3.wear can grow the white frame alone; sim2real's pair is the packed name dark_white_refs.
    Parameter(
        name="dark_reference",
        unit="DN",
        definition="The dark reference frame (shutter closed): each pixel's and band's counts at zero light.",
        shape="map",
    ),
    Parameter(
        name="white_reference",
        unit="DN",
        definition="The white reference frame: each pixel's and band's counts over a white tile of known reflectance.",
        shape="map",
    ),
    # M1.band_widths
    Parameter(
        name="band_fwhm",
        unit="m",
        definition="Full width at half maximum of each band's spectral response.",
        shape="list",
    ),
    # M2.smile_keystone
    Parameter(
        name="smile_keystone_map",
        unit="px",
        definition="The spectral (smile) and spatial (keystone) distortion map the cube is resampled with.",
        shape="map",
    ),
    # M3.pushbroom
    Parameter(
        name="line_geometry",
        unit="m, rad",
        definition="The scan line's pose, and the belt distance per line.",
        shape="pair",
    ),
    # M3.wear
    Parameter(
        name="reference_drift",
        unit="DN/h",
        definition=("How fast the white reference's DN drifts per deployment hour (lamp ageing). The dark reference "
                    "drifts the other way (dark current) and is not modelled by this rate."),
    ),
)

# --- rungs, M0 upward -------------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0,
        name="bands",
        title="Ideal spectral bands",
        base=None,
        adds=("band_centres", "dark_reference", "white_reference"),
        equation="R(λ) = (DN − dark_reference)/(white_reference − dark_reference), at each band's band_centres",
        references=(references.BURGER_2011,),
    ),
    Rung(
        level=1,
        name="band_widths",
        title="Band widths",
        base="M0.bands",
        adds=("band_fwhm",),
        equation="each band a Gaussian of its band_fwhm about its band_centres",
        references=(references.BURGER_2011, references.KRUSE_1993),
    ),
    Rung(
        level=2,
        name="smile_keystone",
        title="Smile and keystone",
        base="M1.band_widths",
        adds=("smile_keystone_map",),
        # WHY the resampling comes first: smile shifts a band's wavelength and keystone shifts where it sees the line,
        # so the cube is put back on the nominal bands before reflectance is read (Mouroulis; PEP 0004 §5.9).
        equation="the cube resampled through smile_keystone_map, then R(λ) per band",
        references=(references.MOUROULIS_2000,),
    ),
    Rung(
        level=3,
        name="pushbroom",
        title="Pushbroom line geometry",
        base="M2.smile_keystone",
        adds=("line_geometry",),
        equation="a line camera swept by the belt: line k sees the belt at k·(belt distance per line)",
        references=(references.GUPTA_1997,),
    ),
    Rung(
        level=3,
        name="wear",
        title="A white reference that drifts with hours",
        base="M3.pushbroom",
        adds=("reference_drift",),
        # WHY linear in deployment hours: lamp ageing moves the white frame steadily, and linear is the simplest
        # degradation path (Lu and Meeker); it is PEP 0004 §5.9's choice, sim2real's too. WHY the white frame alone:
        # dark current drifts the dark frame the other way, so one rate cannot carry both (PEP 0004 amendment 1).
        wears=(
            Wear(
                parameter="white_reference",
                usage="deployment hours",
                usage_unit="h",
                law="linear",
                rate="reference_drift",
            ),
        ),
        equation="white_reference(h) = white_reference + reference_drift·h; dark_reference stays",
        references=(references.LU_MEEKER_1993,),
    ),
)

# --- aliases, packed names and deployment-only names (PEP 0004 §6.8: this type has no deployment names) -----------

ALIASES = (
    Alias(name="reference_drift_per_h", canonical="reference_drift"),
)

PACKED = (
    Packed(name="dark_white_refs", holds=("dark_reference", "white_reference"), ordered=True,
           note="sim2real's camera_hsi card: (dark, white) as its name reads, both in DN."),
)

LADDER = Ladder(
    type="hyperspectral_camera",
    title="A line-scan spectral camera over a belt.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    packed=PACKED,
)
