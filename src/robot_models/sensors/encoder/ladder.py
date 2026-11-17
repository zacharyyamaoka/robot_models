"""An angle sensor on a shaft, such as a drive's rotor encoder: its ladder of models.

The reading is the shaft's angle, quantised to counts_per_rev (M0), with noise and a delay (M1), plus an angle error
that repeats with the shaft angle (M2.eccentricity), which grows over vibration hours as the mount loosens (M2.wear).

Spec: PEP 0004 §5.6 and §6.5.
"""

from __future__ import annotations

from robot_models import references
from robot_models.ladder import Alias, Ladder, Parameter, Rung, Wear

# --- the vocabulary, grouped by the rung that first reads each name ------------------------------------------------

VOCABULARY = (
    # M0.quantiser
    Parameter(
        name="counts_per_rev",
        unit="1/rev",
        shape="integer",
        definition="Distinct readings per revolution of the shaft it reads: its quantisation.",
    ),
    # M1.noise_latency
    Parameter(
        name="noise_sigma",
        unit="rad",
        shape="scalar",
        definition="Standard deviation of the reading's noise at standstill.",
    ),
    Parameter(
        name="latency",
        unit="s",
        shape="scalar",
        definition="Delay from the shaft's angle to the reading that reports it.",
    ),
    # M2.eccentricity
    Parameter(
        name="eccentricity_harmonics",
        unit="rad",
        shape="list",
        definition=("Angle error periodic in the shaft angle (an eccentric magnet or disc), a list of (harmonic, "
                    "amplitude, phase)."),
    ),
    # M2.wear
    Parameter(
        name="eccentricity_growth",
        unit="rad/√h",
        shape="scalar",
        definition=("How fast the eccentricity grows with the square root of vibration hours (a loosening "
                    "mount)."),
    ),
)

# --- the rungs, M0 upward ------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0,
        name="quantiser",
        title="An ideal quantiser",
        base=None,
        adds=("counts_per_rev",),
        equation="q_reported = round(q/lsb)·lsb, lsb = 2π/counts_per_rev",
        references=(references.WIDROW_2008,),
    ),
    Rung(
        level=1,
        name="noise_latency",
        title="Noise and latency",
        base="M0.quantiser",
        adds=("noise_sigma", "latency"),
        equation="q_reported(t) = round((q(t − latency) + n)/lsb)·lsb, n ~ N(0, noise_sigma²)",
        references=(references.MERRY_2010,),
    ),
    Rung(
        level=2,
        name="eccentricity",
        title="Eccentricity harmonics",
        base="M1.noise_latency",
        adds=("eccentricity_harmonics",),
        equation="q_reported = round((q + e(q) + n)/lsb)·lsb, delayed; e(q) the eccentricity harmonics",
        references=(references.ZHAO_2024,),
    ),
    Rung(
        level=2,
        name="wear",
        title="A mount that loosens over vibration hours",
        base="M2.eccentricity",
        adds=("eccentricity_growth",),
        # WHY square root (PEP 0004 §5.6): a loosening mount drifts diffusively, so the harmonics' expected size grows
        # with the square root of vibration hours (Whitmore 1995) rather than linearly.
        wears=(
            Wear(
                parameter="eccentricity_harmonics",
                usage="vibration hours",
                usage_unit="h",
                law="sqrt",
                rate="eccentricity_growth",
            ),
        ),
        equation="e(h) = e + eccentricity_growth·√h",
        references=(references.JUNKER_1969, references.WHITMORE_1995),
    ),
)

# --- other names for the vocabulary (PEP 0004 §6.5) ----------------------------------------------------------------

ALIASES = (
    Alias(name="latency_s", canonical="latency"),
    Alias(name="eccentricity_growth_mrad_per_sqrt_h", canonical="eccentricity_growth", unit="mrad/√h"),
    Alias(name="encoder_counts_per_rev", canonical="counts_per_rev"),
    Alias(name="encoder_counts", canonical="counts_per_rev"),
    Alias(
        name="encoder_lines",
        canonical="counts_per_rev",
        note="Lines: a quadrature encoder counts 4 × lines per revolution.",
    ),
    Alias(name="encoder_bits", canonical="counts_per_rev", note="Bits: 2^bits counts per revolution."),
)

# No packed names and no deployment-only names: nothing here packs several quantities into one array, and every
# quantity the models read belongs to the sensor itself.

LADDER = Ladder(
    type="encoder",
    title="An angle sensor on a shaft, such as a drive's rotor encoder.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
)
