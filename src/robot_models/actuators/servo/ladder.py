"""A joint in one housing, a motor, its drive, a reducer or none, and an encoder: its ladder of models (PEP 0004).

Spec: PEP 0004 §5.2 and §6.1 (docs/peps/0004-model-ladders-by-type.md).

The ladder climbs the way a joint's torque model grows:

- M0 and M1 state the part (its inertia and ratio, then every datasheet field) and fit nothing.
- M2 to M3 are friction, from Coulomb and viscous through Stribeck, load, LuGre, and weakening with a capture level.
- M4 adds the drivetrain, rotor ripple, backlash and compliance, and composes the drive's own delay loop.
- M5 is the electrics and heat (MuJoCo's DC motor, and the drive's ceiling) and the wear that grows with use.
- M6 is a learned residual on the wear rung and M7 a black-box network.

Torques, speeds and inertias are at the OUTPUT, after the reducer, as `IntegratedServo`'s datasheet states them; kt,
kv and rotor_inertia stay on the rotor. Every name below is a canonical one: sim2real's twin names, the registry's
keys and the exporter's fields are aliases (a unit never appears in a canonical name), and MuJoCo's arrays are packed
names. What a deployment sets (gains, current limit) is `deployment`, never a rung's parameter.
"""

from __future__ import annotations

from robot_models import references
from robot_models.actuators.servo.integrated_servo import IntegratedServo
from robot_models.ladder import (
    Alias,
    DeploymentOnly,
    Ladder,
    Packed,
    Parameter,
    Rung,
    Wear,
    datasheet_vocabulary,
)

# WHY a folder constant: robotparts' eight actuator rungs live under one folder, each named by its word (PEP 0004 §8).
RUNG_CODE = "actuators/rungs"

# WHY these usages: the servo card counts a deployment's hours, and backlash opens with the torque it carried over the
# revolutions it turned (Archard), so its usage is their product (PEP 0004 §5.2, M5.wear).
HOURS = ("deployment hours", "h")
TORQUE_REVOLUTIONS = ("torque-weighted output revolutions", "N·m·rev")

# --- the vocabulary: the datasheet's fields, then the names each rung first reads ---------------------------

# WHY from the class (PEP 0003 §6): M1.datasheet's names, units and definitions are the typed fields', never a second
# spelling of them.
DATASHEET_VOCABULARY = datasheet_vocabulary(IntegratedServo)

# M3: state-dependent friction.
FRICTION_VOCABULARY = (
    Parameter(
        name="breakaway_torque",
        unit="N·m",
        definition=("Static friction at the output: the torque that starts it moving from rest, the top of the "
                    "Stribeck curve."),
    ),
    Parameter(
        name="stribeck_velocity",
        unit="rad/s",
        definition="Output speed over which friction falls from breakaway_torque to coulomb_friction.",
    ),
    Parameter(
        name="stribeck_exponent",
        unit="",
        definition="Shape exponent of the Stribeck curve, how sharply friction falls with speed; 2 when absent.",
    ),
    Parameter(
        name="friction_load_coefficient",
        unit="",
        definition="Extra output friction per N·m of load torque the part transmits: Fc + μ·abs(τ_load).",
    ),
    Parameter(
        name="sigma0",
        unit="N·m/rad",
        definition="LuGre bristle stiffness: output torque per radian of bristle deflection.",
    ),
    Parameter(
        name="sigma1",
        unit="N·m·s/rad",
        definition="LuGre bristle damping: output torque per rad/s of bristle deflection rate.",
    ),
    Parameter(
        name="friction_weakening",
        unit="N·m·s/rad",
        definition=("How fast the static friction level falls with output speed, up to "
                    "friction_weakening_limit."),
    ),
    Parameter(
        name="friction_weakening_limit",
        unit="rad/s",
        definition="The output speed up to which the friction level keeps weakening.",
    ),
    Parameter(
        name="friction_capture",
        unit="N·m",
        definition=("The friction level that holds a slowed output until it has stayed slow for "
                    "friction_capture_dwell."),
    ),
    Parameter(
        name="friction_capture_speed",
        unit="rad/s",
        definition="Below this output speed a joint counts as slow for capture.",
    ),
    Parameter(
        name="friction_capture_dwell",
        unit="s",
        definition="How long a joint must stay slow before the static level returns.",
    ),
)

# M4: the drivetrain.
DRIVETRAIN_VOCABULARY = (
    Parameter(
        name="harmonics",
        unit="N·m",
        definition=("Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, amplitude, phase): "
                    "cogging and commutation ripple."),
        shape="list",
    ),
    Parameter(
        name="torsional_stiffness",
        unit="N·m/rad",
        definition="Drivetrain stiffness at the output with the rotor held: output torque per radian of twist.",
    ),
)

# M5: heat. kt and resistance_line_to_line are datasheet fields, so they sit in the datasheet vocabulary.
HEAT_VOCABULARY = (
    Parameter(
        name="thermal_resistance",
        unit="K/W",
        definition=("Winding-to-ambient thermal resistance: winding temperature rise per watt of copper loss."),
    ),
    Parameter(
        name="thermal_time_constant",
        unit="s",
        definition="Time constant of the winding temperature's first-order rise.",
    ),
)

# M5.wear: the rates a deployment's hours and turned load wear the joint at.
WEAR_VOCABULARY = (
    Parameter(
        name="friction_growth",
        unit="N·m/h",
        definition=("How fast the friction level grows per deployment hour (seals, grease, bearings); on a Stribeck "
                    "curve breakaway and Coulomb grow together."),
    ),
    Parameter(
        name="backlash_wear",
        unit="rad/(N·m·rev)",
        definition="How fast backlash opens per torque-weighted output revolution (Archard).",
    ),
)

VOCABULARY = (
    *DATASHEET_VOCABULARY,
    *FRICTION_VOCABULARY,
    *DRIVETRAIN_VOCABULARY,
    *HEAT_VOCABULARY,
    *WEAR_VOCABULARY,
)

# --- the rungs, M0 upward -----------------------------------------------------------------------------------

M0_ADDS = ("output_inertia", "gear_ratio")

# WHY every typed field and none needed (PEP 0004 decision 3; robotparts' M1 marks all its terms required: false): the
# datasheet rung states what the source states, so a part with no friction figure still reaches M1.
DATASHEET_OPTIONAL = tuple(name for name in IntegratedServo.typed_fields() if name not in M0_ADDS)

RUNGS = (
    Rung(
        level=0,
        name="ideal",
        title="The ideal actuator: the stated inertia and ratio, no losses",
        base=None,
        adds=M0_ADDS,
        code=f"{RUNG_CODE}/ideal",
        equation="tau_r = 0; armature = output_inertia",
        references=(references.SICILIANO_2009, references.MUJOCO_ARMATURE),
    ),
    Rung(
        level=1,
        name="datasheet",
        title="The datasheet: the stated values, nothing fitted",
        base="M0.ideal",
        optional=DATASHEET_OPTIONAL,
        code=f"{RUNG_CODE}/datasheet",
        equation=("tau_r = coulomb_friction·sign(v) + viscous_friction·v when stated; armature = output_inertia, or "
                  "gear_ratio²·(rotor_inertia + gearbox_input_inertia)"),
        references=(references.ARMSTRONG_1994, references.MUJOCO_ARMATURE),
        note="States every IntegratedServo field and needs none beyond M0 (robotparts' M1: required false).",
    ),
    Rung(
        level=2,
        name="coulomb_viscous",
        title="Coulomb and viscous friction, fitted",
        base="M1.datasheet",
        adds=("coulomb_friction", "viscous_friction"),
        code=f"{RUNG_CODE}/coulomb_viscous",
        equation="tau_r = coulomb_friction·sign(v) + viscous_friction·v",
        references=(references.ARMSTRONG_1994, references.OLSSON_1998),
    ),
    Rung(
        level=3,
        name="stribeck",
        title="Stribeck friction: breakaway falling to Coulomb with speed",
        base="M2.coulomb_viscous",
        adds=("breakaway_torque", "stribeck_velocity"),
        optional=("stribeck_exponent",),
        code=f"{RUNG_CODE}/stribeck",
        equation=("tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|v|/stribeck_velocity)"
                  "^stribeck_exponent))·sign(v) + viscous_friction·v"),
        references=(references.ARMSTRONG_1994, references.OLSSON_1998),
    ),
    Rung(
        level=3,
        name="load",
        title="Friction that grows with the transmitted load",
        base="M2.coulomb_viscous",
        adds=("friction_load_coefficient",),
        code=f"{RUNG_CODE}/load",
        equation=("tau_r = (coulomb_friction + friction_load_coefficient·|tau_load|)·sign(v) "
                  "+ viscous_friction·v"),
        references=(references.KIRCANSKI_1997, references.OLSSON_1998),
    ),
    Rung(
        level=3,
        name="lugre",
        title="LuGre dynamic friction",
        base="M3.stribeck",
        adds=("sigma0", "sigma1"),
        code=f"{RUNG_CODE}/lugre",
        # WHY viscous_friction stands for LuGre's sigma2: one quantity, one canonical name; "sigma2" is its alias.
        equation=("dz/dt = v − sigma0·|v|·z/g(v), g(v) the Stribeck level of M3.stribeck; "
                  "tau_r = sigma0·z + sigma1·dz/dt + viscous_friction·v (LuGre's sigma2)"),
        references=(references.CANUDAS_1995,),
    ),
    Rung(
        level=3,
        name="weakening_capture",
        title="Stribeck with weakening and a capture level at rest",
        base="M3.stribeck",
        adds=(
            "friction_weakening",
            "friction_weakening_limit",
            "friction_capture",
            "friction_capture_speed",
            "friction_capture_dwell",
        ),
        equation=(
            "level(v) = breakaway_torque − friction_weakening·min(|v|, friction_weakening_limit) − "
            "(breakaway_torque − coulomb_friction)·(1 − exp(−(|v|/stribeck_velocity)²)); "
            "friction_capture holds below friction_capture_speed until friction_capture_dwell has passed"
        ),
        references=(references.ARMSTRONG_1994, references.CANUDAS_1995),
        # WHY no code (PEP 0004 §11, call 7): the type holds the structure and its terms; the twin that fitted it
        # is never published.
        note="sim2real's servo M3, the rack twin's friction: its structure only, the twin is never published.",
    ),
    Rung(
        level=4,
        name="ripple",
        title="Rotor ripple on Coulomb and viscous friction",
        base="M2.coulomb_viscous",
        adds=("harmonics",),
        code=f"{RUNG_CODE}/ripple",
        equation=("tau_r = Σ_k amplitude_k·sin(rotor_harmonic_k·gear_ratio·θ + phase_k) "
                  "+ coulomb_friction·sign(v) + viscous_friction·v"),
        references=(references.JAHNS_1996,),
    ),
    Rung(
        level=4,
        name="drivetrain",
        title="The drivetrain: ripple, backlash and compliance on the full friction model",
        base="M3.weakening_capture",
        adds=("harmonics", "backlash", "torsional_stiffness"),
        composes=(("foc_controller", "M1.delay_loop"),),
        equation=("J q̈ = tau_drive − tau_r; tau_r = level(v)·sign(v) + viscous_friction·v + the harmonics; the "
                  "output across a backlash gap and torsional_stiffness; tau_drive from foc_controller "
                  "M1.delay_loop"),
        references=(references.JAHNS_1996, references.NORDIN_2002, references.TUTTLE_1996, references.KRISHNAN_2010),
        note="sim2real's servo M4, the rack twin (today's model), less its heat: that is M5.envelope's.",
    ),
    Rung(
        level=5,
        name="dcmotor",
        title="MuJoCo's DC motor: electrics, heat, cogging and LuGre friction",
        base="M3.lugre",
        adds=("kt", "resistance_line_to_line"),
        optional=("harmonics", "thermal_resistance", "thermal_time_constant"),
        code=f"{RUNG_CODE}/dcmotor",
        equation=("MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR "
                  "(back-EMF, cogging, LuGre friction)"),
        references=(references.KRAUSE_2013, references.MUJOCO_DCMOTOR),
        note=("robotparts' rung: its terms are MuJoCo's arrays (the packed names); motorconst and resistance "
              "required, the rest optional, as its model.json marks them."),
    ),
    Rung(
        level=5,
        name="envelope",
        title="The drive's ceiling, the motor's electrics and heat on the drivetrain",
        base="M4.drivetrain",
        adds=("kt", "resistance_line_to_line", "thermal_resistance", "thermal_time_constant"),
        composes=(("foc_controller", "M2.ceiling_heat"),),
        equation=("kt_out = kt·gear_ratio, R = resistance_line_to_line/2 per phase: motoring torque ≤ "
                  "kt_out·(voltage_available − |v|·kt_out/1.5)/R; any torque ≤ kt_out·I_max (the deployment's "
                  "current cap); winding rise T: thermal_time_constant·dT/dt = thermal_resistance·i²R − T"),
        references=(references.KRAUSE_2013, references.KRISHNAN_2010, references.MUJOCO_DCMOTOR),
        note=("sim2real's servo M5 (the envelope ladder): the rack twin plus the drive's ceiling and the motor's "
              "electrics; the heat the twin carries at its M4 sits here, robotparts' layer 5."),
    ),
    Rung(
        level=5,
        name="wear",
        title="Wear: friction grows with hours, backlash opens with load",
        base="M5.envelope",
        adds=("friction_growth", "backlash_wear"),
        wears=(
            Wear(parameter="breakaway_torque", usage=HOURS[0], usage_unit=HOURS[1], law="linear",
                 rate="friction_growth"),
            Wear(parameter="coulomb_friction", usage=HOURS[0], usage_unit=HOURS[1], law="linear",
                 rate="friction_growth"),
            Wear(parameter="backlash", usage=TORQUE_REVOLUTIONS[0], usage_unit=TORQUE_REVOLUTIONS[1], law="archard",
                 rate="backlash_wear"),
        ),
        equation=("backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev; breakaway_torque and coulomb_friction both "
                  "grow by friction_growth·h (the Stribeck drop stays)"),
        references=(references.ARCHARD_1953, references.FLODIN_1997, references.LU_MEEKER_1993),
    ),
    Rung(
        level=6,
        name="residual",
        title="Grey-box residual on M5.wear",
        base="M5.wear",
        kind="residual",
        equation=("M5.wear + an MLP residual torque from 3 samples of position error, velocity and torque "
                  "command"),
        references=(references.NGUYEN_TUONG_2010, references.ZENG_2020, references.KOVALEV_2026),
    ),
    Rung(
        level=7,
        name="learned",
        title="Black-box network",
        base=None,
        kind="learned",
        equation="an MLP over 3 samples of position error and velocity → output torque",
        references=(references.HWANGBO_2019, references.HWANGBO_2019_ARXIV, references.ZHU_2025),
    ),
)

# --- other names for the same quantities --------------------------------------------------------------------

# WHY a note whenever an alias is not its canonical one for one (PEP 0004 §6): a side, a phase, a level against a
# difference, a part against a mount. Where `unit` is set the source writes that unit; "" is the canonical SI unit.
ALIASES = (
    # The registry's old user-part keys and its catalogue columns.
    Alias(
        name="torque_constant",
        canonical="kt",
        note="Old user-part key: a geared part's source may state it at the output (kt × gear_ratio).",
    ),
    Alias(
        name="torque_coefficient",
        canonical="kt",
        note="Catalogue part key: check RMS against peak amperes (kt is per peak ampere).",
    ),
    Alias(
        name="phase_resistance",
        canonical="resistance_line_to_line",
        note=("Old user-part key: a 'phase to phase' value is line to line; a per-phase value is half of it on a "
              "wye winding."),
    ),
    Alias(
        name="phase_inductance",
        canonical="inductance_line_to_line",
        note="Old user-part key: per phase is half of line to line on a wye winding.",
    ),
    Alias(
        name="noload_speed",
        canonical="max_output_speed",
        note="Old user-part key: its unit is the part file's; check it before converting.",
    ),
    Alias(
        name="max_speed",
        canonical="max_output_speed",
        note="Old user-part key: may be a mechanical limit rather than the no-load speed.",
    ),
    Alias(
        name="peak_phase_current",
        canonical="max_phase_current",
        note="The registry's driver key: what the integrated drive can deliver; the motor alone may take more.",
    ),
    Alias(
        name="encoder_counts",
        canonical="encoder_lines",
        note="Counts per turn: 4 × lines on a quadrature encoder.",
    ),
    # Catalogue atlas columns, which carry their unit in the name.
    Alias(name="rated_torque_nm", canonical="rated_torque"),
    Alias(name="peak_torque_nm", canonical="peak_torque"),
    Alias(name="noload_speed_rpm", canonical="max_output_speed", unit="rpm"),
    Alias(name="rated_speed_rpm", canonical="rated_speed", unit="rpm"),
    Alias(name="rated_power_w", canonical="rated_power"),
    Alias(name="voltage_v", canonical="nominal_voltage"),
    Alias(name="mass_kg", canonical="mass"),
    Alias(name="od_mm", canonical="outer_diameter", unit="mm"),
    Alias(name="length_mm", canonical="axial_length", unit="mm"),
    Alias(name="rotor_inertia_kg_m2", canonical="rotor_inertia"),
    Alias(name="peak_current_a", canonical="max_phase_current"),
    Alias(name="rated_current_a", canonical="rated_phase_current"),
    Alias(name="backlash_arcmin", canonical="backlash", unit="arcmin"),
    # Cad2Robot's exporter fields.
    Alias(
        name="phase_resistance_ohm",
        canonical="resistance_line_to_line",
        unit="Ω",
        note="Per phase (sim2real's servo, Cad2Robot's motor): line to line is twice it on a wye winding.",
    ),
    Alias(
        name="phase_inductance_h",
        canonical="inductance_line_to_line",
        unit="H",
        note="Per phase (Cad2Robot's motor): half of line to line on a wye winding.",
    ),
    Alias(
        name="max_effort_nm",
        canonical="peak_torque",
        note="Cad2Robot's servo field: the effort limit it exports.",
    ),
    Alias(
        name="max_velocity_rad_s",
        canonical="max_output_speed",
        note="Cad2Robot's servo field: the velocity limit it exports.",
    ),
    # sim2real's servo card and the rack twin's names.
    Alias(
        name="armature_kg_m2",
        canonical="output_inertia",
        note=("As mounted: the whole axis' inertia at the output, the part's own plus what it drives; "
              "output_inertia is the part alone."),
    ),
    Alias(
        name="frictionloss_nm",
        canonical="coulomb_friction",
        note=("The rack twin's frictionloss is the static level: with no Stribeck term it is coulomb_friction; with "
              "one it is breakaway_torque, and coulomb_friction = frictionloss − friction_stribeck_drop."),
    ),
    Alias(name="damping_nm_s_rad", canonical="viscous_friction"),
    Alias(name="friction_weakening_nm_s_rad", canonical="friction_weakening"),
    Alias(name="friction_weakening_limit_rad_s", canonical="friction_weakening_limit"),
    Alias(
        name="friction_stribeck_drop_nm",
        canonical="breakaway_torque",
        note="A difference, not a level: breakaway_torque − coulomb_friction.",
    ),
    Alias(
        name="friction_stribeck_speed_rad_s",
        canonical="stribeck_velocity",
        note="The twin's Stribeck exponent is fixed at 2.",
    ),
    Alias(name="friction_capture_nm", canonical="friction_capture"),
    Alias(name="friction_capture_speed_rad_s", canonical="friction_capture_speed"),
    Alias(name="friction_capture_dwell_s", canonical="friction_capture_dwell"),
    Alias(name="ripple_harmonics", canonical="harmonics"),
    Alias(
        name="ripple_rotor_per_output",
        canonical="gear_ratio",
        note="The ripple's rotor turns per output turn: the gear ratio as the twin's ripple reads it.",
    ),
    Alias(name="backlash_rad", canonical="backlash"),
    Alias(
        name="gear_stiffness_nm_rad",
        canonical="torsional_stiffness",
        note="Fitted on a mounted unit: it includes the mount's compliance.",
    ),
    Alias(
        name="kt_out_nm_a",
        canonical="kt",
        note="Output side: kt_out = kt × gear_ratio.",
    ),
    Alias(name="backlash_wear_rad_per_nm_rev", canonical="backlash_wear"),
    Alias(
        name="friction_growth_nm_per_h",
        canonical="friction_growth",
        note="It grows the rack twin's frictionloss, the static level: breakaway_torque and coulomb_friction together.",
    ),
    # robotparts' rung terms that name one quantity the vocabulary already has.
    Alias(
        name="sigma2",
        canonical="viscous_friction",
        note="LuGre's own symbol (M3.lugre) for the same viscous coefficient.",
    ),
    Alias(
        name="resistance",
        canonical="resistance_line_to_line",
        unit="Ω",
        note=("M5.dcmotor's winding resistance as MuJoCo takes it, per phase as sim2real reads it "
              "(phase_resistance_ohm): line to line is twice it on a wye winding."),
    ),
)

# --- names that pack several quantities into one array ------------------------------------------------------

# WHY packed and not an alias of the first entry (PEP 0004 §4, the fidelity review): an array is never one parameter.
# MuJoCo's own layouts are not verified here, so each says what it carries and what is MuJoCo's.
MUJOCO_ARRAY = "M5.dcmotor's MuJoCo array (unit mujoco)"

PACKED = (
    Packed(
        name="thermal_r_c",
        holds=("thermal_resistance", "thermal_time_constant"),
        ordered=True,
        note=("The rack twin's (K/W, s) pair. Its thermal model fits a Foster network of one or two poles; with "
              "two, thermal_resistance = ΣR_k and no single time constant holds."),
    ),
    Packed(
        name="motorconst",
        holds=("kt",),
        note=(f"{MUJOCO_ARRAY}, 2 numbers: sim2real reads the torque constant from it at the OUTPUT "
              "(kt × gear_ratio, unless the actuator's gear carries the ratio); the other entry is MuJoCo's, "
              "unverified here."),
    ),
    Packed(
        name="nominal",
        holds=("nominal_voltage",),
        note=f"{MUJOCO_ARRAY}, 3 numbers of nominal ratings; the other entries are MuJoCo's, unverified here.",
    ),
    Packed(
        name="inductance",
        holds=("inductance_line_to_line",),
        note=(f"{MUJOCO_ARRAY}, 2 numbers; per phase or line to line is MuJoCo's convention, unverified here."),
    ),
    Packed(
        name="cogging",
        holds=("harmonics",),
        note=(f"{MUJOCO_ARRAY}, 3 numbers: one cogging harmonic (harmonic, amplitude, phase) in MuJoCo's order, "
              "unverified here."),
    ),
    Packed(
        name="thermal",
        holds=("thermal_resistance", "thermal_time_constant"),
        note=f"{MUJOCO_ARRAY}, 6 numbers; the rest and the order are MuJoCo's, unverified here.",
    ),
    Packed(
        name="lugre",
        holds=("sigma0", "sigma1"),
        note=f"{MUJOCO_ARRAY}, 5 numbers: the M3.lugre terms in MuJoCo's order, unverified here.",
    ),
)

# --- names a deployment sets, which no rung reads as a parameter of the type --------------------------------

# WHY not on a rung (PEP 0004 §7): these come from the station or the drive's config. The card's driver-role names
# (torque_scale, current_loop_hz, command_delay_s, voltage_available_v) are no entry here: they resolve through
# foc_controller, which M4.drivetrain and M5.envelope compose.
DEPLOYMENT = (
    DeploymentOnly(
        name="gravity_nm",
        role="assembly",
        why="The station's gravity torque on the joint: the description's.",
    ),
    DeploymentOnly(name="kp_nm_rad", role="config", why="The drive's position gain: the deployment's config."),
    DeploymentOnly(name="kd_nm_s_rad", role="config", why="The drive's velocity gain: the deployment's config."),
    DeploymentOnly(name="max_current_a", role="config", why="The drive's current limit: the deployment's config."),
    DeploymentOnly(
        name="controller",
        role="config",
        why="M5.dcmotor's MuJoCo controller array: the drive's control settings.",
    ),
    DeploymentOnly(
        name="saturation",
        role="config",
        why="M5.dcmotor's MuJoCo saturation array: the drive's ceilings.",
    ),
)

LADDER = Ladder(
    type="integrated_servo",
    title="A joint in one housing: a motor, its drive, a reducer or none, and an encoder.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    packed=PACKED,
    deployment=DEPLOYMENT,
    datasheet=IntegratedServo,
)
