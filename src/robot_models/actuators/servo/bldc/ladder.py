"""A brushless motor alone, at its shaft: its ladder of models (PEP 0004).

Rung M1 states every `BldcMotor` field; the fitted rungs above it are sim2real's `motor` card, moved here, with a
Stribeck rung (M3: breakaway falling to Coulomb with shaft speed) between its bearing friction (M2) and its cogging
(M4). A bare motor has no reducer, so the output the ladder speaks of is the rotor itself, its shaft, and robotparts'
actuator rungs (M0 to M5) read as 1:1 actuators on it.

Spec: PEP 0004 §5.3 and §6.2.
"""

from __future__ import annotations

from robot_models import references
from robot_models.ladder import Alias, Ladder, Packed, Parameter, Rung, Wear, datasheet_vocabulary

from .bldc_motor import BldcMotor

# --- the vocabulary, grouped by the rung that first reads it ----------------------------------------------------------

DATASHEET = {parameter.name: parameter for parameter in datasheet_vocabulary(BldcMotor)}

# WHY the datasheet's own Parameters and nothing re-typed (PEP 0003 §6, PEP 0004 decision 3): names, SI units and
# definitions come from the BldcMotor class, so the datasheet and the ladder can never hold two spellings of a field.
M0_NEEDS = ("kt", "rotor_inertia")
M2_NEEDS = ("coulomb_friction", "viscous_friction")
M5_NEEDS = ("resistance_line_to_line", "inductance_line_to_line")

M0_VOCABULARY = tuple(DATASHEET[name] for name in M0_NEEDS)
M1_VOCABULARY = tuple(parameter for name, parameter in DATASHEET.items()
                      if name not in (*M0_NEEDS, *M2_NEEDS, *M5_NEEDS))
M2_VOCABULARY = tuple(DATASHEET[name] for name in M2_NEEDS)

# WHY the integrated servo's own definitions, "at the output", word for word: a bare motor's output is its shaft (the
# docstring says so once), and the datasheet's coulomb_friction and viscous_friction say "at the output" too, so the
# whole friction curve reads at one point and the README matches PEP 0004 §5.3 (amendment 1).
M3_VOCABULARY = (
    Parameter(
        name="breakaway_torque", unit="N·m",
        definition=("Static friction at the output: the torque that starts it moving from rest, the top of the "
                    "Stribeck curve."),
    ),
    Parameter(
        name="stribeck_velocity", unit="rad/s",
        definition="Output speed over which friction falls from breakaway_torque to coulomb_friction.",
    ),
    Parameter(
        name="stribeck_exponent", unit="",
        definition="Shape exponent of the Stribeck curve, how sharply friction falls with speed; 2 when absent.",
    ),
)

M4_VOCABULARY = (
    Parameter(
        name="harmonics", unit="N·m", shape="list",
        definition=("Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, amplitude, phase): "
                    "cogging and commutation ripple."),
    ),
)

M5_VOCABULARY = (
    *(DATASHEET[name] for name in M5_NEEDS),
    Parameter(
        name="thermal_resistance", unit="K/W",
        definition=("Winding-to-ambient thermal resistance: winding temperature rise per watt of copper loss."),
    ),
    Parameter(
        name="thermal_time_constant", unit="s",
        definition="Time constant of the winding temperature's first-order rise.",
    ),
)

WEAR_VOCABULARY = (
    Parameter(
        name="friction_growth", unit="N·m/h",
        definition=("How fast the friction level grows per deployment hour (seals, grease, bearings); on a Stribeck "
                    "curve breakaway and Coulomb grow together."),
    ),
)

VOCABULARY = (
    *M0_VOCABULARY, *M1_VOCABULARY, *M2_VOCABULARY, *M3_VOCABULARY, *M4_VOCABULARY, *M5_VOCABULARY, *WEAR_VOCABULARY,
)

# --- the rungs, M0 upward ---------------------------------------------------------------------------------------------

# WHY computed from the class: M1.datasheet reads every typed field not already needed at M0 (PEP 0004 decision 3), and
# robotparts' M1 marks all its terms required: false, so none is needed here beyond M0.
M1_OPTIONAL = tuple(name for name in BldcMotor.typed_fields() if name not in M0_NEEDS)

# WHY these usages: the motor card counts a deployment's hours; both friction levels grow with them (M5.wear).
DEPLOYMENT_HOURS = "deployment hours"
HOURS_UNIT = "h"

RUNGS = (
    Rung(
        level=0, name="ideal",
        title="An ideal torque source: kt·i on the rotor inertia",
        base=None,
        adds=M0_NEEDS,
        equation="τ = kt·i; rotor_inertia·ω̇ = τ − τ_load",
        references=(references.SICILIANO_2009, references.KRAUSE_2013),
        note="robotparts' M0.ideal read as a 1:1 actuator, whose output is the rotor.",
    ),
    Rung(
        level=1, name="datasheet",
        title="The datasheet: the stated values, nothing fitted",
        base="M0.ideal",
        optional=M1_OPTIONAL,
        equation="τ = kt·i on rotor_inertia, less coulomb_friction·sign(ω) + viscous_friction·ω when stated",
        references=(references.KRAUSE_2013, references.ARMSTRONG_1994),
    ),
    Rung(
        level=2, name="coulomb_viscous",
        title="Coulomb and viscous bearing friction",
        base="M1.datasheet",
        adds=M2_NEEDS,
        equation="τ_r = coulomb_friction·sign(ω) + viscous_friction·ω",
        references=(references.ARMSTRONG_1994, references.OLSSON_1998),
        note=("the actuator's M2 on a bare motor: robotparts' actuator rungs cover its category, and its coverage "
              "ruler asks a BLDC motor's M2 for both."),
    ),
    # WHY an M3 the motor card does not have (PEP 0004 amendment 1): robotparts' coverage ruler asks a BLDC motor's M3
    # for breakaway_torque and stribeck_velocity, and sim2real's motor card has no M3 to say otherwise. No code
    # folder: the eight rung folders are the integrated servo's (PEP 0004 §11, call 6); this is that model at the shaft.
    Rung(
        level=3, name="stribeck",
        title="Stribeck friction: breakaway falling to Coulomb with shaft speed",
        base="M2.coulomb_viscous",
        adds=("breakaway_torque", "stribeck_velocity"),
        optional=("stribeck_exponent",),
        equation=("τ_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|ω|/stribeck_velocity)"
                  "^stribeck_exponent))·sign(ω) + viscous_friction·ω"),
        references=(references.ARMSTRONG_1994, references.OLSSON_1998),
        note=("the actuator's M3.stribeck on a bare motor, whose output is the shaft: robotparts' coverage ruler asks "
              "a BLDC motor's M3 for both."),
    ),
    Rung(
        level=4, name="ripple",
        title="Cogging: rotor-periodic ripple",
        base="M3.stribeck",
        adds=("harmonics",),
        equation="τ = kt·i − Σ_k amplitude_k·sin(k·θ + phase_k)",
        references=(references.JAHNS_1996,),
        note="sim2real's motor M4; a datasheet's torque_ripple states one amplitude of it.",
    ),
    Rung(
        level=5, name="dcmotor",
        title="DC motor electrics and heat",
        base="M4.ripple",
        adds=(*M5_NEEDS, "thermal_resistance", "thermal_time_constant"),
        # WHY the EMF from kt: kt is per phase-current amplitude, so τ = (3/2)·λ·i and the per-phase EMF amplitude is
        # λ·ω = (kt/1.5)·ω, from a name this rung needs; it equals back_emf_constant/√3 only for a line-to-line PEAK
        # constant, and back_emf_constant is read at M1 alone, when stated, with its rms-or-peak convention unsaid.
        # The servo's M5.envelope and foc_controller's M2.ceiling_heat write the same kt/1.5. WHY the 3/4:
        # resistance_line_to_line is line to line, so the three-phase copper loss is
        # (3/2)·i²·R_phase = (3/4)·i²·resistance_line_to_line.
        equation=("τ = kt·i, i the phase-current amplitude; per phase on a wye winding: "
                  "V = (resistance_line_to_line/2)·i + (inductance_line_to_line/2)·di/dt "
                  "+ (kt/1.5)·ω, the per-phase EMF amplitude; "
                  "thermal_time_constant·dΔT/dt = thermal_resistance·(3/4)·resistance_line_to_line·i² − ΔT, "
                  "ΔT the winding's rise over ambient (the three phases' copper loss)"),
        references=(references.KRAUSE_2013, references.KRISHNAN_2010, references.MUJOCO_DCMOTOR),
        note="sim2real's motor M5 (today's model): the actuator's M5 on a bare motor, without LuGre friction.",
    ),
    Rung(
        level=5, name="wear",
        title="Bearing friction that grows with hours",
        base="M5.dcmotor",
        adds=("friction_growth",),
        # WHY both levels at one rate: growing Coulomb alone would pass breakaway and turn the Stribeck term
        # negative (friction rising with speed, no static peak); the drop between them stays as fitted.
        wears=(
            Wear(parameter="breakaway_torque", usage=DEPLOYMENT_HOURS, usage_unit=HOURS_UNIT, law="linear",
                 rate="friction_growth"),
            Wear(parameter="coulomb_friction", usage=DEPLOYMENT_HOURS, usage_unit=HOURS_UNIT, law="linear",
                 rate="friction_growth"),
        ),
        equation=("breakaway_torque and coulomb_friction both grow by friction_growth·h "
                  "(the Stribeck drop stays)"),
        references=(references.LU_MEEKER_1993, references.ARMSTRONG_1994),
    ),
    Rung(
        level=6, name="residual", kind="residual",
        title="Grey-box residual on M5.wear",
        base="M5.wear",
        equation=("M5.wear + an MLP residual shaft torque from 3 samples of q-axis current, rotor speed and winding "
                  "temperature"),
        references=(references.NGUYEN_TUONG_2010, references.ZENG_2020),
    ),
    Rung(
        level=7, name="learned", kind="learned",
        title="Black-box network",
        base=None,
        equation="an MLP over 3 samples of q-axis current command and rotor speed → shaft torque",
        references=(references.HWANGBO_2019, references.HWANGBO_2019_ARXIV),
    ),
)

# --- other names for the same quantities -----------------------------------------------------------------------------

ALIASES = (
    # old user-part keys, catalogue atlas columns, Cad2Robot exporter fields: the datasheet's other spellings
    Alias(name="torque_constant", canonical="kt",
          note="Old user-part key: a geared part's source may state it at the output (kt × gear_ratio)."),
    Alias(name="torque_coefficient", canonical="kt",
          note="Catalogue part key: check RMS against peak amperes (kt is per peak ampere)."),
    Alias(name="phase_resistance", canonical="resistance_line_to_line",
          note=("Old user-part key: a 'phase to phase' value is line to line; a per-phase value is half of it on a "
                "wye winding.")),
    Alias(name="phase_resistance_ohm", canonical="resistance_line_to_line", unit="Ω",
          note="Per phase (sim2real's servo, Cad2Robot's motor): line to line is twice it on a wye winding."),
    Alias(name="phase_inductance", canonical="inductance_line_to_line",
          note="Old user-part key: per phase is half of line to line on a wye winding."),
    Alias(name="phase_inductance_h", canonical="inductance_line_to_line", unit="H",
          note="Per phase (Cad2Robot's motor): half of line to line on a wye winding."),
    Alias(name="noload_speed", canonical="max_output_speed",
          note="Old user-part key: its unit is the part file's; check it before converting."),
    Alias(name="max_speed", canonical="max_output_speed",
          note="Old user-part key: may be a mechanical limit rather than the no-load speed."),
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
    Alias(name="encoder_counts", canonical="encoder_lines",
          note="Counts per turn: 4 × lines on a quadrature encoder."),
    # sim2real's motor card
    Alias(name="motorconst", canonical="kt",
          note=("sim2real's motor card: the scalar kt, rotor side (robotparts' dcmotor packs an array under this "
                "name).")),
    Alias(name="resistance", canonical="resistance_line_to_line", unit="Ω",
          note=("sim2real's motor card, after robotparts' dcmotor: per phase or line to line is not stated; check it "
                "before converting.")),
    Alias(name="inductance", canonical="inductance_line_to_line", unit="H",
          note="sim2real's motor card: the convention as for resistance."),
    Alias(name="cogging", canonical="torque_ripple",
          note="sim2real's motor card: one cogging amplitude; its harmonic is not stated."),
    Alias(name="bearing_friction_nm", canonical="coulomb_friction",
          note="A bare motor's Coulomb friction is its bearings'."),
    Alias(name="bearing_friction_growth_nm_per_h", canonical="friction_growth"),
)

# --- names that pack several quantities -------------------------------------------------------------------------------

PACKED = (
    Packed(name="thermal", holds=("thermal_resistance", "thermal_time_constant"), ordered=True,
           note="sim2real's motor card: (K/W, s)."),
)

# WHY no deployment-only names (PEP 0004 §5.3): a bare motor has no drive, gains or mount; those belong to
# foc_controller and to the assembly that holds the motor.

LADDER = Ladder(
    type="bldc_motor",
    title="A brushless motor alone, at its shaft.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
    packed=PACKED,
    datasheet=BldcMotor,
)
