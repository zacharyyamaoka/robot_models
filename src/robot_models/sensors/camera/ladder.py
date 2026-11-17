"""A frame camera's colour image: projection, distortion, exposure (PEP 0004).

Spec: PEP 0004 §5.7 and §6.6.

The ladder climbs from the ideal pinhole (M0) through the lens (M1), the sensor's light response and the frame's delay
(M2), a rolling shutter (M3) and a mount that loosens with vibration (M3.wear), to a grey-box correction of every
pixel's ray (M6). There is no black-box rung: every rung stays a geometric pixel-to-ray model that a calibration target
can measure, and a learned image-to-image black box gives no rays to check.

Read it top down: the vocabulary, grouped by the rung that first reads each name; then the rungs, M0 upward; then the
aliases. This type has no packed names and nothing deployment-only.
"""

from __future__ import annotations

from robot_models import references as refs
from robot_models.ladder import Alias, Ladder, Parameter, Rung, Wear

# --- the vocabulary, by the rung that first reads each name -----------------------------------------------------------

M0_PINHOLE = (
    Parameter(name="fx_fy", unit="px", shape="pair", definition="Focal lengths in pixels, (fx, fy)."),
    Parameter(name="cx_cy", unit="px", shape="pair", definition="Principal point in pixels, (cx, cy)."),
    Parameter(name="extrinsics", unit="m, rad", shape="pose",
              definition="The camera's pose in the frame it is calibrated against."),
)
M1_DISTORTION = (
    Parameter(name="distortion", unit="", shape="list",
              definition="Brown-Conrady lens distortion coefficients, radial and tangential."),
)
M2_EXPOSURE_LATENCY = (
    Parameter(name="exposure_response", unit="DN/(lx·s)", shape="scalar",
              definition="Pixel value per unit of light exposure: the sensor's response."),
    Parameter(name="latency", unit="s", shape="scalar", definition="Delay from exposure to the frame's arrival."),
)
M3_ROLLING_SHUTTER = (
    Parameter(name="line_delay", unit="s", shape="scalar",
              definition="Rolling shutter: the time between the exposures of successive rows."),
)
M3_WEAR = (
    # WHY no rotation rate (PEP 0004 §5.7): no source states one, so only the translation drifts.
    Parameter(name="extrinsic_drift", unit="m/√h", shape="scalar",
              definition=("How fast the extrinsics' translation drifts with the square root of vibration hours (a "
                          "loosening mount); no source states a rotation rate.")),
)

# --- the rungs, M0 upward ---------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0, name="pinhole", title="Pinhole projection", base=None,
        adds=("fx_fy", "cx_cy", "extrinsics"),
        equation="p = K [R | t] X, K from fx_fy and cx_cy, [R | t] the extrinsics",
        references=(refs.ZHANG_2000,),
    ),
    Rung(
        level=1, name="distortion", title="Brown-Conrady distortion", base="M0.pinhole",
        adds=("distortion",),
        equation="radial (k1, k2, k3) and tangential (p1, p2) distortion of the projected point",
        references=(refs.ZHANG_2000, refs.BROWN_1966, refs.HEIKKILA_1997),
    ),
    Rung(
        level=2, name="exposure_latency", title="Exposure response and latency", base="M1.distortion",
        adds=("exposure_response", "latency"),
        equation="DN = exposure_response·illuminance·exposure time; the frame arrives latency after exposure",
        references=(refs.ZHANG_2000,),
    ),
    Rung(
        level=3, name="rolling_shutter", title="Rolling shutter: a line delay", base="M2.exposure_latency",
        adds=("line_delay",),
        equation="row v exposed at t0 + v·line_delay",
        references=(refs.OTH_2013, refs.MEINGAST_2005),
    ),
    Rung(
        level=3, name="wear", title="A mount that loosens over vibration hours", base="M3.rolling_shutter",
        adds=("extrinsic_drift",),
        wears=(Wear(parameter="extrinsics", usage="vibration hours", usage_unit="h", law="sqrt",
                    rate="extrinsic_drift"),),
        equation="translation offset(h) = e + extrinsic_drift·√h",
        references=(refs.JUNKER_1969, refs.WHITMORE_1995),
    ),
    Rung(
        # WHY residual and no learned rung: the grid corrects each pixel's ray, so the model stays a pixel-to-ray map a
        # calibration target can measure; a black box gives no rays to check. Source: sim2real's camera_rgb card, the
        # note on its M6 rung (PEP 0004 §5.7 lists the rung and its parameters, not the reason).
        level=6, name="residual", title="Grey-box residual on M3.wear", base="M3.wear", kind="residual",
        equation="M3.wear + a B-spline grid of ray offsets over the pixel (u, v)",
        references=(refs.SCHOPS_2020,),
    ),
)

# --- other names for the same quantities (PEP 0004 §6.6) --------------------------------------------------------------

ALIASES = (
    Alias(name="latency_s", canonical="latency"),
    Alias(name="camera_latency_s", canonical="latency"),
    Alias(name="camera_latency_ms", canonical="latency", unit="ms"),
    Alias(name="line_delay_s", canonical="line_delay"),
    Alias(name="extrinsic_drift_mm_per_sqrt_h", canonical="extrinsic_drift", unit="mm/√h"),
)

LADDER = Ladder(
    type="camera",
    title="A frame camera's colour image: projection, distortion, exposure.",
    vocabulary=(*M0_PINHOLE, *M1_DISTORTION, *M2_EXPOSURE_LATENCY, *M3_ROLLING_SHUTTER, *M3_WEAR),
    rungs=RUNGS,
    aliases=ALIASES,
)
