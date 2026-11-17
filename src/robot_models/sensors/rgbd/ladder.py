"""The depth_camera ladder: a stereo depth camera, depth on top of a colour stream.

Spec: PEP 0004 §5.8 and §6.7.

    z = stereo_focal_length·baseline/d                     # M0: depth from the disparity d, in pixels
    σ_z = z²·subpixel_sigma/(stereo_focal_length·baseline)  # M1: the matcher's subpixel noise, growing with z²
    z_true = z + b(u, v, z)                                 # M2: a bias that grows with depth, varying over the image

The colour stream itself is the camera type's (`sensors/camera`), so this ladder lists only what depth adds. Depth is
triangulated with `stereo_focal_length`, the stereo imagers' rectified focal length in pixels (on a typical unit, the IR
pair): it is the f of triangulation, not the colour camera's, which only receives depth after the `depth_to_color`
transform. sim2real's `camera_rgbd` card maps one to one onto all five rungs (PEP 0004's rung map).
"""

from robot_models import references as refs
from robot_models.ladder import Alias, Ladder, Parameter, Rung, Wear

# --- vocabulary, grouped by the rung that first reads it --------------------------------------------------------------

# M0.stereo
# WHY its own name and not the colour camera's fx_fy: depth is triangulated on the stereo imagers' rectified focal
# length, which on a typical unit is the IR pair's and differs from the colour camera's (PEP 0004 amendment 1).
STEREO_FOCAL_LENGTH = Parameter(
    name="stereo_focal_length", unit="px", shape="scalar",
    definition=("The stereo imagers' rectified focal length (on a typical unit the IR pair): the f of triangulation, "
                "not the colour camera's."),
)
BASELINE = Parameter(
    name="baseline", unit="m", shape="scalar",
    definition="Distance between the stereo pair's optical centres.",
)
DEPTH_SCALE = Parameter(
    name="depth_scale", unit="m", shape="scalar",
    definition="Metres per unit of the depth image.",
)
DEPTH_TO_COLOR = Parameter(
    name="depth_to_color", unit="m, rad", shape="pose",
    definition="The depth frame's pose in the colour camera's frame.",
)

# M1.disparity_noise
SUBPIXEL_SIGMA = Parameter(
    name="subpixel_sigma", unit="px", shape="scalar",
    definition="Standard deviation of the disparity: the stereo matcher's subpixel noise.",
)

# M2.depth_bias
DEPTH_BIAS_COEFFS = Parameter(
    name="depth_bias_coeffs", unit="m", shape="list",
    definition="Coefficients of a depth bias that grows with depth and varies across the image.",
)

# M2.wear
DEPTH_TO_COLOR_DRIFT = Parameter(
    name="depth_to_color_drift", unit="m/√h", shape="scalar",
    definition=("How fast depth_to_color's translation drifts with the square root of vibration hours; no source "
                "states a rotation rate."),
)

# --- the rungs, M0 upward ---------------------------------------------------------------------------------------------

M0_STEREO = Rung(
    level=0, name="stereo", title="Ideal stereo", base=None,
    adds=("stereo_focal_length", "baseline", "depth_scale", "depth_to_color"),
    equation="z = stereo_focal_length·baseline/d, d the disparity, scaled by depth_scale",
    references=(refs.KHOSHELHAM_2012,),
)
M1_DISPARITY_NOISE = Rung(
    level=1, name="disparity_noise", title="Disparity noise", base=M0_STEREO.key,
    adds=("subpixel_sigma",),
    equation="σ_z = z²·subpixel_sigma/(stereo_focal_length·baseline)",
    references=(refs.KHOSHELHAM_2012,),
)
M2_DEPTH_BIAS = Rung(
    level=2, name="depth_bias", title="Depth bias", base=M1_DISPARITY_NOISE.key,
    adds=("depth_bias_coeffs",),
    equation="z_true = z + b(u, v, z), b from depth_bias_coeffs",
    references=(refs.HERRERA_2012,),
)
M2_WEAR = Rung(
    level=2, name="wear", title="A depth-to-colour mount that loosens over vibration hours", base=M2_DEPTH_BIAS.key,
    adds=("depth_to_color_drift",),
    # WHY sqrt in vibration hours (PEP 0004 §5.8): a loosening mount creeps with the vibration it has seen, as a joint
    # zero does; the sources state a translation rate only, so the rotation is left alone.
    wears=(Wear(parameter="depth_to_color", usage="vibration hours", usage_unit="h", law="sqrt",
                rate="depth_to_color_drift"),),
    equation="translation offset(h) = e + depth_to_color_drift·√h, e the depth_to_color translation",
    references=(refs.JUNKER_1969, refs.WHITMORE_1995),
)
M6_RESIDUAL = Rung(
    level=6, name="residual", title="Grey-box residual on M2.wear", base=M2_WEAR.key, kind="residual",
    equation="M2.wear + a B-spline grid of depth corrections over (u, v, measured depth)",
    references=(refs.SCHOPS_2020, refs.HERRERA_2012),
)

# --- other names ------------------------------------------------------------------------------------------------------

# Names that carry their unit (sim2real's cards, the exporter's columns); the unit on the alias converts them.
ALIASES = (
    Alias(name="baseline_m", canonical="baseline"),
    Alias(name="depth_to_color_drift_mm_per_sqrt_h", canonical="depth_to_color_drift", unit="mm/√h"),
)

LADDER = Ladder(
    type="depth_camera",
    title="A stereo depth camera: depth on top of a colour stream.",
    vocabulary=(
        STEREO_FOCAL_LENGTH, BASELINE, DEPTH_SCALE, DEPTH_TO_COLOR, SUBPIXEL_SIGMA, DEPTH_BIAS_COEFFS,
        DEPTH_TO_COLOR_DRIFT,
    ),
    rungs=(M0_STEREO, M1_DISPARITY_NOISE, M2_DEPTH_BIAS, M2_WEAR, M6_RESIDUAL),
    aliases=ALIASES,
)
