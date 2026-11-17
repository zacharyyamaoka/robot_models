"""A gear stage between a motor and its load: its ladder of models (PEP 0004).

The ladder climbs from the ideal ratio (M0) through the datasheet's stated values (M1), no-load friction (M2),
friction that grows with the transmitted load (M3) and backlash with torsional stiffness (M4), to backlash that opens
with use (M4.wear), a grey-box residual (M6) and a black-box network (M7).

Its vocabulary is the `Gearbox` datasheet's typed fields (which M1 states) plus the names a fit adds on the rungs
above. A gearbox has no packed names and no deployment-only names: its parameters are all its own.

Spec: PEP 0004 §5.4 and §6.3.
"""

from __future__ import annotations

from robot_models import references
from robot_models.actuators.servo.gearbox.gearbox import Gearbox
from robot_models.ladder import Alias, Ladder, Parameter, Rung, Wear, datasheet_vocabulary

# --- the vocabulary, grouped by the rung that first reads it ----------------------------------------------------------

# M0 and M1: the datasheet's typed fields, with their units and definitions taken from the class.
# WHY not restated here (PEP 0003 §6, PEP 0004 decision 3): the datasheet and the ladder can then never hold two
# spellings of one field, and M1.datasheet holds exactly these.
DATASHEET_FIELDS = datasheet_vocabulary(Gearbox)

# M2: friction with the rotor turning at no load, at the output.
M2_PARAMETERS = (
    Parameter(
        name="coulomb_friction",
        unit="N·m",
        definition="Coulomb friction torque at the output: the constant torque that opposes motion.",
    ),
    Parameter(
        name="viscous_friction",
        unit="N·m·s/rad",
        definition="Viscous friction coefficient at the output.",
    ),
)

# M3: friction that depends on the load the mesh transmits.
M3_PARAMETERS = (
    Parameter(
        name="friction_load_coefficient",
        unit="",
        definition="Extra output friction per N·m of load torque the part transmits: Fc + μ·abs(τ_load).",
    ),
    Parameter(
        name="output_friction_fraction",
        unit="",
        definition="The share of the reducer's friction that acts on its output side.",
    ),
)

# M4: the drivetrain's lost motion and compliance.
M4_PARAMETERS = (
    Parameter(
        name="torsional_stiffness",
        unit="N·m/rad",
        definition="Drivetrain stiffness at the output with the rotor held: output torque per radian of twist.",
    ),
)

# M4.wear: how fast the lost motion grows.
M4_WEAR_PARAMETERS = (
    Parameter(
        name="backlash_wear",
        unit="rad/(N·m·rev)",
        definition="How fast backlash opens per torque-weighted output revolution (Archard).",
    ),
)

VOCABULARY = (
    *DATASHEET_FIELDS,
    *M2_PARAMETERS,
    *M3_PARAMETERS,
    *M4_PARAMETERS,
    *M4_WEAR_PARAMETERS,
)

# --- the rungs, M0 upward ---------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0,
        name="ideal",
        title="An ideal ratio and its reflected inertia",
        base=None,
        adds=("gear_ratio", "gearbox_input_inertia"),
        equation="q_out = q_in/gear_ratio; reflected inertia = gear_ratio²·gearbox_input_inertia",
        references=(references.SICILIANO_2009, references.MUJOCO_ARMATURE),
        note="the actuator's M0.ideal for a reducer.",
    ),
    Rung(
        level=1,
        name="datasheet",
        title="The datasheet: the stated values, nothing fitted",
        base="M0.ideal",
        # WHY optional and not adds (PEP 0004 decision 3): M1 states every typed field but needs none beyond M0.
        optional=tuple(
            parameter.name for parameter in DATASHEET_FIELDS
            if parameter.name not in ("gear_ratio", "gearbox_input_inertia")
        ),
        equation="the ideal ratio, with τ_out = efficiency·gear_ratio·τ_in when efficiency is stated",
        references=(references.SICILIANO_2009,),
    ),
    Rung(
        level=2,
        name="coulomb_viscous",
        title="Coulomb and viscous friction at the output",
        base="M1.datasheet",
        adds=("coulomb_friction", "viscous_friction"),
        equation="τ_r = coulomb_friction·sign(v) + viscous_friction·v at the output",
        references=(references.ARMSTRONG_1994, references.OLSSON_1998),
        note=("the actuator's M2 on a reducer, its no-load running torque: robotparts' load rung requires both, and "
              "its coverage ruler asks a gearbox's M2 for coulomb_friction."),
    ),
    Rung(
        level=3,
        name="load",
        title="Mesh friction that grows with the transmitted load",
        base="M2.coulomb_viscous",
        adds=("efficiency", "output_friction_fraction"),
        optional=("friction_load_coefficient",),
        equation=("τ_out = efficiency·gear_ratio·τ_in; output_friction_fraction of the friction acts on the output "
                  "side"),
        references=(references.KIRCANSKI_1997, references.OLSSON_1998),
        # WHY efficiency needed and friction_load_coefficient only read when stated: they state one effect
        # (load-dependent mesh loss) two ways. The rung runs on η, as sim2real's gearbox does; μ is read only when a
        # source states μ, and then the two must agree, driving forward μ = (1 − η)/η. WHY no μ term in the equation:
        # coulomb_friction + μ·abs(τ_load) on top of η would charge the same mesh loss twice.
        note=("sim2real's gearbox M3. efficiency (η) is needed; friction_load_coefficient (μ) is read only when a "
              "source states μ, and then the two agree: driving forward μ = (1 − η)/η."),
    ),
    Rung(
        level=4,
        name="drivetrain",
        title="Backlash and torsional stiffness",
        base="M3.load",
        adds=("backlash", "torsional_stiffness"),
        equation="q_out = q_in/gear_ratio + lash(τ, backlash) + τ/torsional_stiffness",
        references=(references.KIRCANSKI_1997, references.NORDIN_2002, references.TUTTLE_1996),
    ),
    Rung(
        level=4,
        name="wear",
        title="Backlash that opens with use (Archard)",
        base="M4.drivetrain",
        adds=("backlash_wear",),
        wears=(
            Wear(
                parameter="backlash",
                usage="torque-weighted output revolutions",
                usage_unit="N·m·rev",
                law="archard",
                rate="backlash_wear",
            ),
        ),
        equation="backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev, over the output revolutions Δrev",
        references=(references.ARCHARD_1953, references.FLODIN_1997),
    ),
    Rung(
        level=6,
        name="residual",
        title="Grey-box residual on M4.wear",
        base="M4.wear",
        kind="residual",
        equation=("M4.wear + an MLP residual transmission error from 3 samples of input angle, output torque and "
                  "input speed"),
        references=(references.NGUYEN_TUONG_2010, references.ZENG_2020),
    ),
    Rung(
        level=7,
        name="learned",
        title="Black-box network",
        base=None,
        kind="learned",
        # WHY 5 samples: hysteresis and backlash need a longer memory than a residual's 3 (Zhu 2025).
        equation="an MLP over 5 samples of input angle, input speed and output torque → output angle error",
        references=(references.HWANGBO_2019, references.ZHU_2025),
    ),
)

# --- other names for the same quantities (PEP 0004 §6.3) --------------------------------------------------------------

ALIASES = (
    Alias(
        name="mesh_efficiency_eta",
        canonical="efficiency",
        note=("The twin's constant η in τ_out = η·N·τ_in; a datasheet's efficiency is at its stated operating "
              "point; μ = (1 − η)/η driving forward."),
    ),
    Alias(name="backlash_rad", canonical="backlash"),
    Alias(name="backlash_arcmin", canonical="backlash", unit="arcmin"),
    Alias(
        name="gear_stiffness_nm_rad",
        canonical="torsional_stiffness",
        note="Fitted on a mounted unit: it includes the mount's compliance.",
    ),
    Alias(name="backlash_wear_rad_per_nm_rev", canonical="backlash_wear"),
    Alias(name="rated_torque_nm", canonical="rated_torque"),
    Alias(name="peak_torque_nm", canonical="peak_torque"),
    Alias(name="mass_kg", canonical="mass"),
    Alias(name="od_mm", canonical="outer_diameter", unit="mm"),
    Alias(name="length_mm", canonical="axial_length", unit="mm"),
)

LADDER = Ladder(
    type="gearbox",
    title="A gear stage between a motor and its load.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    datasheet=Gearbox,
)
