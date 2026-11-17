"""A chain of rigid links and joints, open or closed: a whole arm, a sub-assembly, a wrist, a five-bar. Its ladder of
models (PEP 0004).

Spec: PEP 0004 §5.10 and §6.9 (and §0 flag 2: arm is folded into this type, per Zach Nov 6 "the 6 DOF robot arm is
not a type").

The ladder forks after M1.zero_offsets into two branches that both fall back on it:

- the kinematic branch (a whole arm): M2.geometric (DH errors), M3.compliance, M3.wear, then the residual M6.residual;
- the dynamic branch (a sub-assembly, a closed chain): M2.inertial, M2.wear, then the residual M6.inertial_residual.

Read top to bottom: the vocabulary, grouped by the rung that first reads it, then the rungs from M0 up, then the other
names a source may use for them.
"""

from __future__ import annotations

from robot_models import references as refs
from robot_models.ladder import Alias, DeploymentOnly, Ladder, Parameter, Rung, Wear

# --- the vocabulary, by the rung that first reads each name ----------------------------------------------------------

VOCABULARY = (
    # M0.nominal
    Parameter(
        name="link_geometry",
        unit="m, rad",
        definition=("The chain's nominal joint axes and link frames, as a URDF, a DH table or a CAD export states "
                    "them."),
        shape="per joint",
    ),
    # M1.zero_offsets
    Parameter(
        name="joint_zero_offsets",
        unit="rad",
        definition="Each joint's offset between its encoder zero and its kinematic zero.",
        shape="per joint",
    ),
    # M2.geometric
    Parameter(
        name="dh_errors",
        unit="m, rad",
        definition="Errors of the chain's DH parameters from their nominal values.",
        shape="per joint",
    ),
    # M2.inertial
    Parameter(
        name="link_inertials",
        unit="kg, m, kg·m²",
        definition="Each link's mass, centre of mass and inertia tensor.",
        shape="per link",
    ),
    Parameter(
        name="closure_tolerance",
        unit="m",
        definition="How far a closed chain's loop may stay open at its closure joint.",
    ),
    # M2.wear
    Parameter(
        name="joint_zero_drift",
        unit="rad/√h",
        definition="How fast the joint zero offsets drift with the square root of vibration hours.",
    ),
    # M3.compliance
    Parameter(
        name="joint_compliance",
        unit="rad/(N·m)",
        definition="Each joint's deflection per N·m it carries.",
        shape="per joint",
    ),
    # M3.wear
    Parameter(
        name="compliance_wear",
        unit="rad/(N·m)/(N·m·rev)",
        definition="How fast joint_compliance grows per torque-weighted joint revolution (Archard).",
    ),
)

# --- the rungs, M0 upward --------------------------------------------------------------------------------------------

# WHY the residual reads the same inputs on both branches (sim2real's arm and arm sub-assembly cards): three samples
# of each joint's error, speed and torque are what both cards feed their residual MLPs.
RESIDUAL_INPUTS = "3 samples of each joint's error, speed and torque"

RUNGS = (
    Rung(
        level=0,
        name="nominal",
        title="Nominal kinematics",
        base=None,
        adds=("link_geometry",),
        equation="tool = FK(q; link_geometry)",
        references=(refs.SICILIANO_2009, refs.DH_1955),
    ),
    Rung(
        level=1,
        name="zero_offsets",
        title="Joint zero offsets",
        base="M0.nominal",
        adds=("joint_zero_offsets",),
        equation="tool = FK(q + joint_zero_offsets; link_geometry)",
        references=(refs.HOLLERBACH_WAMPLER_1996,),
    ),
    # --- the kinematic branch: a whole arm
    Rung(
        level=2,
        name="geometric",
        title="Geometric (DH) errors",
        base="M1.zero_offsets",
        adds=("dh_errors",),
        equation="tool = FK(q + joint_zero_offsets; link_geometry + dh_errors)",
        references=(refs.HAYATI_1983, refs.OKAMURA_1996, refs.HOLLERBACH_2016),
        note="sim2real's arm M2: the kinematic branch of an open chain.",
    ),
    # --- the dynamic branch: a sub-assembly, open or closed
    # WHY its base is M1.zero_offsets and not M2.geometric: a sub-assembly's dynamics need no DH errors (sim2real's arm
    # sub-assembly card goes from M1 straight to its link inertials), so it never falls back on them.
    Rung(
        level=2,
        name="inertial",
        title="Link inertials and loop closure",
        base="M1.zero_offsets",
        adds=("link_inertials", "closure_tolerance"),
        equation=("tool = FK(q + joint_zero_offsets; link_geometry); "
                  "dynamics from link_inertials with each joint's own type; "
                  "a closed loop may stay open by closure_tolerance"),
        references=(refs.ATKESON_1986, refs.GAUTIER_1990, refs.SWEVERS_1997),
        note="sim2real's arm sub-assembly M2: the dynamic branch, which needs no DH errors.",
    ),
    Rung(
        level=2,
        name="wear",
        title="Joint zeros that drift over vibration hours",
        base="M2.inertial",
        adds=("joint_zero_drift",),
        wears=(
            Wear(parameter="joint_zero_offsets", usage="vibration hours", usage_unit="h", law="sqrt",
                 rate="joint_zero_drift"),
        ),
        equation="joint_zero_offsets(h) = joint_zero_offsets + joint_zero_drift·√h",
        references=(refs.JUNKER_1969, refs.WHITMORE_1995),
    ),
    # --- back on the kinematic branch
    Rung(
        level=3,
        name="compliance",
        title="Joint compliance and link inertials",
        base="M2.geometric",
        adds=("joint_compliance", "link_inertials"),
        equation=("tool = FK(q + joint_zero_offsets + joint_compliance·τ; link_geometry + dh_errors); "
                  "τ from link_inertials, each joint its own type"),
        references=(refs.SPONG_1987, refs.ATKESON_1986, refs.GAUTIER_1990, refs.SWEVERS_1997),
    ),
    Rung(
        level=3,
        name="wear",
        title="Joint compliance that grows with use (Archard)",
        base="M3.compliance",
        adds=("compliance_wear",),
        wears=(
            Wear(parameter="joint_compliance", usage="torque-weighted joint revolutions", usage_unit="N·m·rev",
                 law="archard", rate="compliance_wear"),
        ),
        equation="joint_compliance(u) = joint_compliance + compliance_wear·Σ|τ|·Δrev",
        references=(refs.ARCHARD_1953, refs.FLODIN_1997),
    ),
    # --- the learned rungs: one residual per branch, one net for the type
    Rung(
        level=6,
        name="residual",
        title="Grey-box residual on M3.wear",
        base="M3.wear",
        kind="residual",
        equation=f"M3.wear + an MLP residual of joint torques from {RESIDUAL_INPUTS}",
        references=(refs.NGUYEN_TUONG_2010, refs.ZENG_2020),
    ),
    Rung(
        level=6,
        name="inertial_residual",
        title="Grey-box residual on M2.wear",
        base="M2.wear",
        kind="residual",
        equation=f"M2.wear + an MLP residual of joint torques from {RESIDUAL_INPUTS}",
        references=(refs.NGUYEN_TUONG_2010, refs.ZENG_2020),
        note="the residual of the dynamic branch (sim2real's arm sub-assembly M6).",
    ),
    Rung(
        level=7,
        name="learned",
        title="Black-box network",
        base=None,
        kind="learned",
        equation="an MLP over 3 samples of each joint's error and speed → joint torques",
        references=(refs.HWANGBO_2019,),
    ),
)

# --- the other names a source uses ------------------------------------------------------------------------------------

ALIASES = (
    Alias(name="link_lengths", canonical="link_geometry",
          note="Lengths only: the geometry also carries the joint axes."),
    Alias(name="joint_offsets", canonical="joint_zero_offsets"),
    Alias(name="joint_offset_drift_rad_per_sqrt_h", canonical="joint_zero_drift"),
    Alias(name="joint_zero_drift_rad_per_sqrt_h", canonical="joint_zero_drift"),
    Alias(name="compliance_wear_per_nm_rev", canonical="compliance_wear"),
)

# WHY not on a rung (PEP 0004 §7): where the chain stands and what it carries on its flange belong to the cell it is
# mounted in, so the description gives them; they are no property of the type.
DEPLOYMENT = (
    DeploymentOnly(name="base_frame", role="assembly", why="Where the chain stands in its cell: the description's."),
    DeploymentOnly(name="tool_frame", role="assembly", why="The tool mounted on its flange: the description's."),
)

LADDER = Ladder(
    type="rigid_body_chain",
    title="A chain of rigid links and joints, open or closed: a whole arm, a sub-assembly, a wrist, a five-bar.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    deployment=DEPLOYMENT,
)
