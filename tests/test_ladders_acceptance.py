"""The ruler for robot_models v1's ladder lanes (PEP 0004), committed before they start and off-limits to them.

Red until every type folder holds its `ladder.py` and robotparts' eight actuator rung folders sit in
`actuators/rungs/<word>/` byte for byte. For each of the 12 types it judges: the folder; the rung keys; each rung's
base, the canonical parameters it needs (adds) and reads when stated (optional), its code folder, its wear, its
composition, its equation and its references; the vocabulary with SI units and shapes; every alias (target, note,
unit); every packed name; the deployment-only names; and that no module in the type folder holds an instance value, a
maker or a path.
Across the repo: the rung folders' bytes, each folder's terms against the servo rung that names it, and that every
robot_models module imports in a fresh interpreter.

WHY each type's tests import its own `ladder.py` and never `discover()` (the breakage review): one lane's half-written
ladder, or a type another lane has not landed yet, must not turn every other lane red. Discovery and the compositions
across types are judged in two tests of their own.

It never reads robotparts or sim2real: the expected bytes and terms are hard-coded below (read from robotparts @
d16f6a8, models/actuator/, on 2026-11-16; unchanged at e742ba0), because robot_models' public CI has no sibling
checkout. The spec is checked to be a valid ladder by the core itself (test_the_spec_is_itself_a_valid_ladder), so the
ruler cannot ask for the impossible.
"""

from __future__ import annotations

import ast
import hashlib
import importlib
import importlib.util
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pytest

from robot_models import references as REFS
from robot_models.actuators.servo import BldcMotor, Gearbox, IntegratedServo
from robot_models.ladder import (
    Alias,
    DeploymentOnly,
    Ladder,
    Packed,
    Parameter,
    Rung,
    Wear,
    datasheet_vocabulary,
    discover,
    si_unit,
)

pytestmark = pytest.mark.ruler

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src" / "robot_models"
RUNGS_FOLDER = "actuators/rungs"

# M1.datasheet's optional names on a type with a Datasheet class (PEP 0003 §6, PEP 0004 decision 3): it states every
# typed field and needs none beyond M0, as robotparts' own M1 marks all its terms required: false.
DATASHEET = "every typed field of the type's Datasheet class not already on M0"


@dataclass(frozen=True)
class R:
    """A rung as the spec states it."""

    key: str
    title: str
    base: str | None
    adds: tuple[str, ...] = ()                    # required: what the rung's model cannot run without
    optional: tuple[str, ...] | str = ()          # read when stated (or DATASHEET)
    code: str = ""
    wears: tuple[tuple[str, str, str, str, str], ...] = ()  # (parameter, usage, usage unit, law, rate)
    composes: tuple[tuple[str, str], ...] = ()
    equation: str = ""                            # the spec's form; the ruler asks the lane's to be non-empty
    refs: tuple[str, ...] = ()                    # robot_models.references names the rung must cite (at least)
    note: str = ""


@dataclass(frozen=True)
class T:
    """A type as the spec states it."""

    folder: str
    title: str
    cards: tuple[str, ...]                        # sim2real's cards (v2 ids) this type covers
    category: str                                 # robotparts' catalogue category; "" when none
    datasheet: type | None
    rungs: tuple[R, ...]
    parameters: dict[str, tuple[str, str, str]]   # every parameter not a datasheet field: (SI unit, shape, definition)
    aliases: dict[str, tuple[str, str, str]]      # alias: (canonical, note, unit); a note whenever not the same 1:1
    packed: dict[str, tuple[tuple[str, ...], bool, str]] = field(default_factory=dict)  # name: (holds, ordered, note)
    deployment: dict[str, tuple[str, str]] = field(default_factory=dict)              # name: (role, where it belongs)


def a(canonical: str, note: str = "", unit: str = "") -> tuple[str, str, str]:
    """An alias's spec: its canonical name, its note, the unit its source writes ("" = the canonical SI unit)."""
    return (canonical, note, unit)


def residual(base: str, equation: str, refs: tuple[str, ...]) -> R:
    return R("M6.residual", f"Grey-box residual on {base}", base, equation=equation, refs=refs)


def learned(equation: str, refs: tuple[str, ...]) -> R:
    return R("M7.learned", "Black-box network", None, equation=equation, refs=refs)


HOURS = ("deployment hours", "h")
VIBRATION = ("vibration hours", "h")
TORQUE_REVOLUTIONS = ("torque-weighted output revolutions", "N·m·rev")

# --- shared names -----------------------------------------------------------------------------------------------

FRICTION = {
    "breakaway_torque": ("N·m", "scalar", ("Static friction at the output: the torque that starts it moving from rest, "
                                           "the top of the Stribeck curve.")),
    "stribeck_velocity": ("rad/s", "scalar", ("Output speed over which friction falls from breakaway_torque to "
                                              "coulomb_friction.")),
    "stribeck_exponent": ("", "scalar", ("Shape exponent of the Stribeck curve, how sharply friction falls with speed; "
                                         "2 when absent.")),
    "friction_load_coefficient": ("", "scalar", ("Extra output friction per N·m of load torque the part transmits: "
                                                 "Fc + μ·abs(τ_load).")),
    "sigma0": ("N·m/rad", "scalar", "LuGre bristle stiffness: output torque per radian of bristle deflection."),
    "sigma1": ("N·m·s/rad", "scalar", "LuGre bristle damping: output torque per rad/s of bristle deflection rate."),
    "friction_weakening": ("N·m·s/rad", "scalar", ("How fast the static friction level falls with output speed, up to "
                                                   "friction_weakening_limit.")),
    "friction_weakening_limit": ("rad/s", "scalar", "The output speed up to which the friction level keeps weakening."),
    "friction_capture": ("N·m", "scalar", ("The friction level that holds a slowed output until it has stayed slow for "
                                           "friction_capture_dwell.")),
    "friction_capture_speed": ("rad/s", "scalar", "Below this output speed a joint counts as slow for capture."),
    "friction_capture_dwell": ("s", "scalar", "How long a joint must stay slow before the static level returns."),
}
DRIVETRAIN = {
    "harmonics": ("N·m", "list", ("Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, "
                                  "amplitude, phase): cogging and commutation ripple.")),
    "torsional_stiffness": ("N·m/rad", "scalar", ("Drivetrain stiffness at the output with the rotor held: output "
                                                  "torque per radian of twist.")),
}
HEAT = {
    "thermal_resistance": ("K/W", "scalar", ("Winding-to-ambient thermal resistance: winding temperature rise per watt "
                                             "of copper loss.")),
    "thermal_time_constant": ("s", "scalar", "Time constant of the winding temperature's first-order rise."),
}
FRICTION_GROWTH = ("N·m/h", "scalar", ("How fast the friction level grows per deployment hour (seals, grease, "
                                       "bearings); on a Stribeck curve breakaway and Coulomb grow together."))
BACKLASH_WEAR = ("rad/(N·m·rev)", "scalar", "How fast backlash opens per torque-weighted output revolution (Archard).")

# Other names for a motor's datasheet fields: old user-part keys, catalogue atlas columns, Cad2Robot exporter fields.
MOTOR_DATASHEET_ALIASES = {
    "torque_constant": a("kt", ("Old user-part key: a geared part's source may state it at the output "
                                "(kt × gear_ratio).")),
    "torque_coefficient": a("kt", "Catalogue part key: check RMS against peak amperes (kt is per peak ampere)."),
    "phase_resistance": a("resistance_line_to_line", ("Old user-part key: a 'phase to phase' value is line to line; a "
                                                      "per-phase value is half of it on a wye winding.")),
    "phase_resistance_ohm": a("resistance_line_to_line", ("Per phase (sim2real's servo, Cad2Robot's motor): line to "
                                                          "line is twice it on a wye winding."), "Ω"),
    "phase_inductance": a("inductance_line_to_line", "Old user-part key: per phase is half of line to line on a wye "
                                                     "winding."),
    "phase_inductance_h": a("inductance_line_to_line", "Per phase (Cad2Robot's motor): half of line to line on a wye "
                                                       "winding.", "H"),
    "noload_speed": a("max_output_speed", ("Old user-part key: its unit is the part file's; check it before "
                                           "converting.")),
    "max_speed": a("max_output_speed", "Old user-part key: may be a mechanical limit rather than the no-load speed."),
    "rated_torque_nm": a("rated_torque"),
    "peak_torque_nm": a("peak_torque"),
    "noload_speed_rpm": a("max_output_speed", unit="rpm"),
    "rated_speed_rpm": a("rated_speed", unit="rpm"),
    "rated_power_w": a("rated_power"),
    "voltage_v": a("nominal_voltage"),
    "mass_kg": a("mass"),
    "od_mm": a("outer_diameter", unit="mm"),
    "length_mm": a("axial_length", unit="mm"),
    "rotor_inertia_kg_m2": a("rotor_inertia"),
    "peak_current_a": a("max_phase_current"),
    "rated_current_a": a("rated_phase_current"),
    "encoder_counts": a("encoder_lines", "Counts per turn: 4 × lines on a quadrature encoder."),
}
MUJOCO_ARRAY = "M5.dcmotor's MuJoCo array (unit mujoco)"

# --- the 12 types -------------------------------------------------------------------------------------------------

TYPES: dict[str, T] = {
    "integrated_servo": T(
        folder="actuators/servo",
        title="A joint in one housing: a motor, its drive, a reducer or none, and an encoder.",
        cards=("servo",), category="actuator", datasheet=IntegratedServo,
        rungs=(
            R("M0.ideal", "The ideal actuator: the stated inertia and ratio, no losses", None,
              ("output_inertia", "gear_ratio"), code=f"{RUNGS_FOLDER}/ideal",
              equation="tau_r = 0; armature = output_inertia", refs=("SICILIANO_2009", "MUJOCO_ARMATURE")),
            R("M1.datasheet", "The datasheet: the stated values, nothing fitted", "M0.ideal", optional=DATASHEET,
              code=f"{RUNGS_FOLDER}/datasheet",
              equation=("tau_r = coulomb_friction·sign(v) + viscous_friction·v when stated; armature = output_inertia, "
                        "or gear_ratio²·(rotor_inertia + gearbox_input_inertia)"),
              refs=("ARMSTRONG_1994", "MUJOCO_ARMATURE"),
              note="states every IntegratedServo field and needs none beyond M0 (robotparts' M1: required false)."),
            R("M2.coulomb_viscous", "Coulomb and viscous friction, fitted", "M1.datasheet",
              ("coulomb_friction", "viscous_friction"), code=f"{RUNGS_FOLDER}/coulomb_viscous",
              equation="tau_r = coulomb_friction·sign(v) + viscous_friction·v", refs=("ARMSTRONG_1994", "OLSSON_1998")),
            R("M3.stribeck", "Stribeck friction: breakaway falling to Coulomb with speed", "M2.coulomb_viscous",
              ("breakaway_torque", "stribeck_velocity"), ("stribeck_exponent",), code=f"{RUNGS_FOLDER}/stribeck",
              equation=("tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|v|/stribeck_velocity)"
                        "^stribeck_exponent))·sign(v) + viscous_friction·v"),
              refs=("ARMSTRONG_1994", "OLSSON_1998")),
            R("M3.load", "Friction that grows with the transmitted load", "M2.coulomb_viscous",
              ("friction_load_coefficient",), code=f"{RUNGS_FOLDER}/load",
              equation="tau_r = (coulomb_friction + friction_load_coefficient·|tau_load|)·sign(v) + viscous_friction·v",
              refs=("KIRCANSKI_1997", "OLSSON_1998")),
            R("M3.lugre", "LuGre dynamic friction", "M3.stribeck", ("sigma0", "sigma1"), code=f"{RUNGS_FOLDER}/lugre",
              equation=("dz/dt = v − sigma0·|v|·z/g(v), g(v) the Stribeck level of M3.stribeck; "
                        "tau_r = sigma0·z + sigma1·dz/dt + viscous_friction·v (LuGre's sigma2)"),
              refs=("CANUDAS_1995",)),
            R("M3.weakening_capture", "Stribeck with weakening and a capture level at rest", "M3.stribeck",
              ("friction_weakening", "friction_weakening_limit", "friction_capture", "friction_capture_speed",
               "friction_capture_dwell"),
              equation=("level(v) = breakaway_torque − friction_weakening·min(|v|, friction_weakening_limit) − "
                        "(breakaway_torque − coulomb_friction)·(1 − exp(−(|v|/stribeck_velocity)²)); "
                        "friction_capture holds below friction_capture_speed until friction_capture_dwell has passed"),
              refs=("ARMSTRONG_1994", "CANUDAS_1995"),
              note="sim2real's servo M3, the rack twin's friction: its structure only, the twin is never published."),
            R("M4.ripple", "Rotor ripple on Coulomb and viscous friction", "M2.coulomb_viscous", ("harmonics",),
              code=f"{RUNGS_FOLDER}/ripple",
              equation=("tau_r = Σ_k amplitude_k·sin(rotor_harmonic_k·gear_ratio·θ + phase_k) "
                        "+ coulomb_friction·sign(v) + viscous_friction·v"),
              refs=("JAHNS_1996",)),
            R("M4.drivetrain", "The drivetrain: ripple, backlash and compliance on the full friction model",
              "M3.weakening_capture", ("harmonics", "backlash", "torsional_stiffness"),
              composes=(("foc_controller", "M1.delay_loop"),),
              equation=("output_inertia·q̈ = tau_drive − tau_r; tau_r = level(v)·sign(v) + viscous_friction·v + "
                        "the harmonics; the output across a backlash gap and torsional_stiffness; tau_drive from "
                        "foc_controller M1.delay_loop"),
              refs=("JAHNS_1996", "NORDIN_2002", "TUTTLE_1996", "KRISHNAN_2010"),
              note="sim2real's servo M4, the rack twin (today's model), less its heat: that is M5.envelope's."),
            R("M5.dcmotor", "MuJoCo's DC motor: electrics, heat, cogging and LuGre friction", "M3.lugre",
              ("kt", "resistance_line_to_line"), ("harmonics", "thermal_resistance", "thermal_time_constant"),
              code=f"{RUNGS_FOLDER}/dcmotor",
              equation=("MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR "
                        "(back-EMF, cogging, LuGre friction)"),
              refs=("KRAUSE_2013", "MUJOCO_DCMOTOR"),
              note=("robotparts' rung: its terms are MuJoCo's arrays (the packed names); motorconst and resistance "
                    "required, the rest optional, as its model.json marks them.")),
            R("M5.envelope", "The drive's ceiling, the motor's electrics and heat on the drivetrain", "M4.drivetrain",
              ("kt", "resistance_line_to_line", "thermal_resistance", "thermal_time_constant"),
              composes=(("foc_controller", "M2.ceiling_heat"),),
              equation=("kt_out = kt·gear_ratio, R = resistance_line_to_line/2 per phase: motoring torque ≤ "
                        "kt_out·(voltage_available − |v|·kt_out/1.5)/R; any torque ≤ kt_out·I_max (the deployment's "
                        "current cap); winding rise ΔT over ambient: "
                        "thermal_time_constant·dΔT/dt = thermal_resistance·1.5·i²R − ΔT "
                        "(the three phases' copper loss, i the phase-current amplitude)"),
              refs=("KRAUSE_2013", "KRISHNAN_2010", "MUJOCO_DCMOTOR"),
              note=("sim2real's servo M5 (the envelope ladder): the rack twin plus the drive's ceiling and the motor's "
                    "electrics; the heat the twin carries at its M4 sits here, robotparts' layer 5.")),
            R("M5.wear", "Wear: friction grows with hours, backlash opens with load", "M5.envelope",
              ("friction_growth", "backlash_wear"),
              wears=(("breakaway_torque", *HOURS, "linear", "friction_growth"),
                     ("coulomb_friction", *HOURS, "linear", "friction_growth"),
                     ("backlash", *TORQUE_REVOLUTIONS, "archard", "backlash_wear")),
              equation=("backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev; breakaway_torque and coulomb_friction both "
                        "grow by friction_growth·h (the Stribeck drop stays)"),
              refs=("ARCHARD_1953", "FLODIN_1997", "LU_MEEKER_1993")),
            residual("M5.wear", "M5.wear + an MLP residual torque from 3 samples of position error, velocity and "
                                "torque command", ("NGUYEN_TUONG_2010", "ZENG_2020", "KOVALEV_2026")),
            learned("an MLP over 3 samples of position error and velocity → output torque",
                    ("HWANGBO_2019", "HWANGBO_2019_ARXIV", "ZHU_2025")),
        ),
        parameters={**FRICTION, **DRIVETRAIN, **HEAT, "friction_growth": FRICTION_GROWTH,
                    "backlash_wear": BACKLASH_WEAR},
        aliases={
            **MOTOR_DATASHEET_ALIASES,
            "peak_phase_current": a("max_phase_current", ("The registry's driver key: what the integrated drive can "
                                                          "deliver; the motor alone may take more.")),
            "backlash_arcmin": a("backlash", unit="arcmin"),
            "max_effort_nm": a("peak_torque", "Cad2Robot's servo field: the effort limit it exports."),
            "max_velocity_rad_s": a("max_output_speed", "Cad2Robot's servo field: the velocity limit it exports."),
            # sim2real's servo card and the rack twin's names
            "armature_kg_m2": a("output_inertia", ("As mounted: the whole axis' inertia at the output, the part's own "
                                                   "plus what it drives; output_inertia is the part alone.")),
            "frictionloss_nm": a("coulomb_friction", ("The rack twin's frictionloss is the static level: with no "
                                                      "Stribeck term it is coulomb_friction; with one it is "
                                                      "breakaway_torque, and coulomb_friction = frictionloss − "
                                                      "friction_stribeck_drop.")),
            "damping_nm_s_rad": a("viscous_friction"),
            "friction_weakening_nm_s_rad": a("friction_weakening"),
            "friction_weakening_limit_rad_s": a("friction_weakening_limit"),
            "friction_stribeck_drop_nm": a("breakaway_torque", ("A difference, not a level: breakaway_torque − "
                                                                "coulomb_friction.")),
            "friction_stribeck_speed_rad_s": a("stribeck_velocity", "The twin's Stribeck exponent is fixed at 2."),
            "friction_capture_nm": a("friction_capture"),
            "friction_capture_speed_rad_s": a("friction_capture_speed"),
            "friction_capture_dwell_s": a("friction_capture_dwell"),
            "ripple_harmonics": a("harmonics"),
            "ripple_rotor_per_output": a("gear_ratio", "The ripple's rotor turns per output turn: the gear ratio as "
                                                       "the twin's ripple reads it."),
            "backlash_rad": a("backlash"),
            "gear_stiffness_nm_rad": a("torsional_stiffness", "Fitted on a mounted unit: it includes the mount's "
                                                              "compliance."),
            "kt_out_nm_a": a("kt", "Output side: kt_out = kt × gear_ratio."),
            "backlash_wear_rad_per_nm_rev": a("backlash_wear"),
            "friction_growth_nm_per_h": a("friction_growth", ("It grows the rack twin's frictionloss, the static "
                                                              "level: breakaway_torque and coulomb_friction "
                                                              "together.")),
            # robotparts' rung terms that name one quantity the vocabulary already has
            "sigma2": a("viscous_friction", "LuGre's own symbol (M3.lugre) for the same viscous coefficient."),
            "resistance": a("resistance_line_to_line", ("M5.dcmotor's winding resistance as MuJoCo takes it, per "
                                                        "phase as sim2real reads it (phase_resistance_ohm): line to "
                                                        "line is twice it on a wye winding."), "Ω"),
        },
        packed={
            "thermal_r_c": (("thermal_resistance", "thermal_time_constant"), True,
                            ("The rack twin's (K/W, s) pair. Its thermal model fits a Foster network of one or two "
                             "poles; with two, thermal_resistance = ΣR_k and no single time constant holds.")),
            "motorconst": (("kt",), False, (f"{MUJOCO_ARRAY}, 2 numbers: sim2real reads the torque constant from it "
                                            "at the OUTPUT (kt × gear_ratio, unless the actuator's gear carries the "
                                            "ratio); the other entry is MuJoCo's, unverified here.")),
            "nominal": (("nominal_voltage",), False, (f"{MUJOCO_ARRAY}, 3 numbers of nominal ratings; the other "
                                                      "entries are MuJoCo's, unverified here.")),
            "inductance": (("inductance_line_to_line",), False, (f"{MUJOCO_ARRAY}, 2 numbers; per phase or line to "
                                                                 "line is MuJoCo's convention, unverified here.")),
            "cogging": (("harmonics",), False, (f"{MUJOCO_ARRAY}, 3 numbers: one cogging harmonic (harmonic, "
                                                "amplitude, phase) in MuJoCo's order, unverified here.")),
            "thermal": (("thermal_resistance", "thermal_time_constant"), False,
                        f"{MUJOCO_ARRAY}, 6 numbers; the rest and the order are MuJoCo's, unverified here."),
            "lugre": (("sigma0", "sigma1"), False, (f"{MUJOCO_ARRAY}, 5 numbers: the M3.lugre terms in MuJoCo's "
                                                    "order, unverified here.")),
        },
        deployment={
            "gravity_nm": ("assembly", "The station's gravity torque on the joint: the description's."),
            "kp_nm_rad": ("config", "The drive's position gain: the deployment's config."),
            "kd_nm_s_rad": ("config", "The drive's velocity gain: the deployment's config."),
            "max_current_a": ("config", "The drive's current limit: the deployment's config."),
            "controller": ("config", "M5.dcmotor's MuJoCo controller array: the drive's control settings."),
            "saturation": ("config", "M5.dcmotor's MuJoCo saturation array: the drive's ceilings."),
        },
    ),
    "bldc_motor": T(
        folder="actuators/servo/bldc",
        title="A brushless motor alone, at its shaft.",
        cards=("motor",), category="actuator", datasheet=BldcMotor,
        rungs=(
            R("M0.ideal", "An ideal torque source: kt·i on the rotor inertia", None, ("kt", "rotor_inertia"),
              equation="τ = kt·i; rotor_inertia·ω̇ = τ − τ_load", refs=("SICILIANO_2009", "KRAUSE_2013"),
              note="robotparts' M0.ideal read as a 1:1 actuator, whose output is the rotor."),
            R("M1.datasheet", "The datasheet: the stated values, nothing fitted", "M0.ideal", optional=DATASHEET,
              equation=("τ = kt·i on rotor_inertia, less coulomb_friction·sign(ω) + viscous_friction·ω when stated"),
              refs=("KRAUSE_2013", "ARMSTRONG_1994")),
            R("M2.coulomb_viscous", "Coulomb and viscous bearing friction", "M1.datasheet",
              ("coulomb_friction", "viscous_friction"),
              equation="τ_r = coulomb_friction·sign(ω) + viscous_friction·ω", refs=("ARMSTRONG_1994", "OLSSON_1998"),
              note=("the actuator's M2 on a bare motor: robotparts' actuator rungs cover its category, and its "
                    "coverage ruler asks a BLDC motor's M2 for both.")),
            # WHY (PEP 0004 amendment 1): robotparts' coverage ruler asks a BLDC motor's M3 for breakaway_torque
            # and stribeck_velocity; sim2real's motor card has no M3 to say otherwise.
            R("M3.stribeck", "Stribeck friction: breakaway falling to Coulomb with shaft speed", "M2.coulomb_viscous",
              ("breakaway_torque", "stribeck_velocity"), ("stribeck_exponent",),
              equation=("τ_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|ω|/stribeck_velocity)"
                        "^stribeck_exponent))·sign(ω) + viscous_friction·ω"),
              refs=("ARMSTRONG_1994", "OLSSON_1998"),
              note=("the actuator's M3.stribeck on a bare motor, whose output is the shaft: robotparts' coverage "
                    "ruler asks a BLDC motor's M3 for both.")),
            R("M4.ripple", "Cogging: rotor-periodic ripple", "M3.stribeck", ("harmonics",),
              equation="τ = kt·i − Σ_k amplitude_k·sin(k·θ + phase_k)", refs=("JAHNS_1996",),
              note="sim2real's motor M4; a datasheet's torque_ripple states one amplitude of it."),
            R("M5.dcmotor", "DC motor electrics and heat", "M4.ripple",
              ("resistance_line_to_line", "inductance_line_to_line", "thermal_resistance", "thermal_time_constant"),
              equation=("τ = kt·i, i the phase-current amplitude; per phase on a wye winding: "
                        "V = (resistance_line_to_line/2)·i + (inductance_line_to_line/2)·di/dt "
                        "+ (back_emf_constant/√3)·ω; "
                        "thermal_time_constant·dΔT/dt = thermal_resistance·(3/4)·resistance_line_to_line·i² − ΔT, "
                        "ΔT the winding's rise over ambient (the three phases' copper loss)"),
              refs=("KRAUSE_2013", "KRISHNAN_2010", "MUJOCO_DCMOTOR"),
              note="sim2real's motor M5 (today's model): the actuator's M5 on a bare motor, without LuGre friction."),
            R("M5.wear", "Bearing friction that grows with hours", "M5.dcmotor", ("friction_growth",),
              wears=(("breakaway_torque", *HOURS, "linear", "friction_growth"),
                     ("coulomb_friction", *HOURS, "linear", "friction_growth")),
              equation=("breakaway_torque and coulomb_friction both grow by friction_growth·h "
                        "(the Stribeck drop stays)"),
              refs=("LU_MEEKER_1993", "ARMSTRONG_1994")),
            residual("M5.wear", "M5.wear + an MLP residual shaft torque from 3 samples of q-axis current, rotor speed "
                                "and winding temperature", ("NGUYEN_TUONG_2010", "ZENG_2020")),
            learned("an MLP over 3 samples of q-axis current command and rotor speed → shaft torque",
                    ("HWANGBO_2019", "HWANGBO_2019_ARXIV")),
        ),
        parameters={
            **{name: FRICTION[name] for name in ("breakaway_torque", "stribeck_velocity", "stribeck_exponent")},
            "harmonics": DRIVETRAIN["harmonics"], **HEAT, "friction_growth": FRICTION_GROWTH,
        },
        aliases={
            **MOTOR_DATASHEET_ALIASES,
            # sim2real's motor card
            "motorconst": a("kt", "sim2real's motor card: the scalar kt, rotor side (robotparts' dcmotor packs an "
                                  "array under this name)."),
            "resistance": a("resistance_line_to_line", ("sim2real's motor card, after robotparts' dcmotor: per phase "
                                                        "or line to line is not stated; check it before converting."),
                            "Ω"),
            "inductance": a("inductance_line_to_line", "sim2real's motor card: the convention as for resistance.",
                            "H"),
            "cogging": a("torque_ripple", "sim2real's motor card: one cogging amplitude; its harmonic is not stated."),
            "bearing_friction_nm": a("coulomb_friction", "A bare motor's Coulomb friction is its bearings'."),
            "bearing_friction_growth_nm_per_h": a("friction_growth"),
        },
        packed={"thermal": (("thermal_resistance", "thermal_time_constant"), True, "sim2real's motor card: (K/W, s).")},
    ),
    "gearbox": T(
        folder="actuators/servo/gearbox",
        title="A gear stage between a motor and its load.",
        cards=("gearbox",), category="gearbox", datasheet=Gearbox,
        rungs=(
            R("M0.ideal", "An ideal ratio and its reflected inertia", None, ("gear_ratio", "gearbox_input_inertia"),
              equation="q_out = q_in/gear_ratio; reflected inertia = gear_ratio²·gearbox_input_inertia",
              refs=("SICILIANO_2009", "MUJOCO_ARMATURE"), note="the actuator's M0.ideal for a reducer."),
            R("M1.datasheet", "The datasheet: the stated values, nothing fitted", "M0.ideal", optional=DATASHEET,
              equation="the ideal ratio, with τ_out = efficiency·gear_ratio·τ_in when efficiency is stated",
              refs=("SICILIANO_2009",)),
            R("M2.coulomb_viscous", "Coulomb and viscous friction at the output", "M1.datasheet",
              ("coulomb_friction", "viscous_friction"),
              equation="τ_r = coulomb_friction·sign(v) + viscous_friction·v at the output",
              refs=("ARMSTRONG_1994", "OLSSON_1998"),
              note=("the actuator's M2 on a reducer, its no-load running torque: robotparts' load rung requires "
                    "both, and its coverage ruler asks a gearbox's M2 for coulomb_friction.")),
            R("M3.load", "Mesh friction that grows with the transmitted load", "M2.coulomb_viscous",
              ("efficiency", "output_friction_fraction"), ("friction_load_coefficient",),
              equation=("τ_out = efficiency·gear_ratio·τ_in; output_friction_fraction of the friction acts on the "
                        "output side"),
              refs=("KIRCANSKI_1997", "OLSSON_1998"),
              note=("sim2real's gearbox M3. efficiency (η) is needed; friction_load_coefficient (μ) is read only "
                    "when a source states μ, and then the two agree: driving forward μ = (1 − η)/η.")),
            R("M4.drivetrain", "Backlash and torsional stiffness", "M3.load", ("backlash", "torsional_stiffness"),
              equation="q_out = q_in/gear_ratio + lash(τ, backlash) + τ/torsional_stiffness",
              refs=("KIRCANSKI_1997", "NORDIN_2002", "TUTTLE_1996")),
            R("M4.wear", "Backlash that opens with use (Archard)", "M4.drivetrain", ("backlash_wear",),
              wears=(("backlash", *TORQUE_REVOLUTIONS, "archard", "backlash_wear"),),
              equation="backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev", refs=("ARCHARD_1953", "FLODIN_1997")),
            residual("M4.wear", "M4.wear + an MLP residual transmission error from 3 samples of input angle, output "
                                "torque and input speed", ("NGUYEN_TUONG_2010", "ZENG_2020")),
            learned("an MLP over 5 samples of input angle, input speed and output torque → output angle error",
                    ("HWANGBO_2019", "ZHU_2025")),
        ),
        parameters={
            "coulomb_friction": ("N·m", "scalar", ("Coulomb friction torque at the output: the constant torque that "
                                                   "opposes motion.")),
            "viscous_friction": ("N·m·s/rad", "scalar", "Viscous friction coefficient at the output."),
            "friction_load_coefficient": FRICTION["friction_load_coefficient"],
            "output_friction_fraction": ("", "scalar", ("The share of the reducer's friction that acts on its output "
                                                        "side.")),
            "torsional_stiffness": DRIVETRAIN["torsional_stiffness"],
            "backlash_wear": BACKLASH_WEAR,
        },
        aliases={
            "mesh_efficiency_eta": a("efficiency", ("The twin's constant η in τ_out = η·N·τ_in; a datasheet's "
                                                    "efficiency is at its stated operating point; μ = (1 − η)/η "
                                                    "driving forward.")),
            "backlash_rad": a("backlash"),
            "backlash_arcmin": a("backlash", unit="arcmin"),
            "gear_stiffness_nm_rad": a("torsional_stiffness", "Fitted on a mounted unit: it includes the mount's "
                                                              "compliance."),
            "backlash_wear_rad_per_nm_rev": a("backlash_wear"),
            "rated_torque_nm": a("rated_torque"),
            "peak_torque_nm": a("peak_torque"),
            "mass_kg": a("mass"),
            "od_mm": a("outer_diameter", unit="mm"),
            "length_mm": a("axial_length", unit="mm"),
        },
    ),
    "foc_controller": T(
        folder="actuators/servo/foc_controller",
        title="A field-oriented-control motor drive board.",
        cards=("driver",), category="driver", datasheet=None,
        rungs=(
            R("M0.ideal", "An ideal drive: the commanded current", None,
              equation="i_q = i_cmd (a deployment's torque cap clamps it)", refs=("KRISHNAN_2010",)),
            R("M1.delay_loop", "Transport delay, a first-order current loop and its torque scale", "M0.ideal",
              ("command_latency", "current_loop_bandwidth", "torque_scale"),
              equation="i_q = torque_scale·LPF(current_loop_bandwidth)[i_cmd(t − command_latency)]",
              refs=("KRISHNAN_2010",)),
            R("M2.ceiling_heat", "Voltage ceiling and board heating", "M1.delay_loop",
              ("voltage_available", "board_thermal_resistance", "board_thermal_time_constant"),
              equation=("M1.delay_loop clamped to the voltage ceiling (voltage_available − ω·kt/1.5)/R of the motor "
                        "it drives; board_thermal_time_constant·dT/dt = board_thermal_resistance·P_loss − T"),
              refs=("KRAUSE_2013", "KRISHNAN_2010")),
            R("M2.wear", "Board cooling that ages with hours", "M2.ceiling_heat", ("board_thermal_growth",),
              wears=(("board_thermal_resistance", *HOURS, "linear", "board_thermal_growth"),),
              equation="board_thermal_resistance(h) = board_thermal_resistance + board_thermal_growth·h",
              refs=("LU_MEEKER_1993", "MEEKER_1998")),
            residual("M2.wear", "M2.wear + an MLP residual q-current from 3 samples of current command, rotor speed "
                                "and bus voltage", ("NGUYEN_TUONG_2010",)),
            learned("an MLP over 3 samples of current command, rotor speed, bus voltage and board temperature → "
                    "q-current", ("HWANGBO_2019",)),
        ),
        parameters={
            "command_latency": ("s", "scalar", ("Time from a command reaching the drive to its first response: the "
                                                "transport delay before the current loop.")),
            "current_loop_bandwidth": ("Hz", "scalar", ("The drive's closed-loop current (torque) bandwidth: the "
                                                        "corner of its first-order current loop.")),
            "torque_scale": ("", "scalar", ("Torque the drive delivers over the torque it was commanded: its "
                                            "torque-constant calibration error.")),
            "voltage_available": ("V", "scalar", ("The largest phase voltage the drive can apply: its bus voltage "
                                                  "times its modulation limit.")),
            "board_thermal_resistance": ("K/W", "scalar", "The board's temperature rise per watt it dissipates."),
            "board_thermal_time_constant": ("s", "scalar", "Time constant of the board temperature's first-order "
                                                           "rise."),
            "board_thermal_growth": ("K/W/h", "scalar", ("How fast board_thermal_resistance grows per deployment hour "
                                                         "(its cooling ageing).")),
        },
        aliases={
            "command_delay_s": a("command_latency", ("The twin's transport delay; the registry's command_latency is "
                                                     "measured host to first response over a named bus.")),
            "current_loop_hz": a("current_loop_bandwidth", "The twin's first-order loop corner, which is its −3 dB "
                                                           "bandwidth."),
            "voltage_available_v": a("voltage_available"),
            "board_thermal_growth_per_h": a("board_thermal_growth"),
        },
        packed={"board_thermal": (("board_thermal_resistance", "board_thermal_time_constant"), True,
                                  "sim2real's driver card: (K/W, s).")},
        deployment={
            "torque_cap_nm": ("config", "The runner's torque cap: the deployment's config."),
            "telemetry_rate_hz": ("config", "The host loop's rate: the deployment's config."),
            "max_voltage_factor": ("config", "The firmware's modulation-limit setting: the deployment's config."),
        },
    ),
    "encoder": T(
        folder="sensors/encoder",
        title="An angle sensor on a shaft, such as a drive's rotor encoder.",
        cards=("encoder",), category="sensor", datasheet=None,
        rungs=(
            R("M0.quantiser", "An ideal quantiser", None, ("counts_per_rev",),
              equation="q_reported = round(q/lsb)·lsb, lsb = 2π/counts_per_rev", refs=("WIDROW_2008",)),
            R("M1.noise_latency", "Noise and latency", "M0.quantiser", ("noise_sigma", "latency"),
              equation="q_reported(t) = round((q(t − latency) + n)/lsb)·lsb, n ~ N(0, noise_sigma²)",
              refs=("MERRY_2010",)),
            R("M2.eccentricity", "Eccentricity harmonics", "M1.noise_latency", ("eccentricity_harmonics",),
              equation="q_reported = round((q + e(q) + n)/lsb)·lsb, delayed; e(q) the eccentricity harmonics",
              refs=("ZHAO_2024",)),
            R("M2.wear", "A mount that loosens over vibration hours", "M2.eccentricity", ("eccentricity_growth",),
              wears=(("eccentricity_harmonics", *VIBRATION, "sqrt", "eccentricity_growth"),),
              equation=("amplitude_k(h) = amplitude_k + eccentricity_growth·√h for each eccentricity harmonic "
                        "(its harmonic and phase stay)"), refs=("JUNKER_1969", "WHITMORE_1995")),
        ),
        parameters={
            "counts_per_rev": ("1/rev", "integer", ("Distinct readings per revolution of the shaft it reads: its "
                                                    "quantisation.")),
            "noise_sigma": ("rad", "scalar", "Standard deviation of the reading's noise at standstill."),
            "latency": ("s", "scalar", "Delay from the shaft's angle to the reading that reports it."),
            "eccentricity_harmonics": ("rad", "list", ("Angle error periodic in the shaft angle (an eccentric magnet "
                                                       "or disc), a list of (harmonic, amplitude, phase).")),
            "eccentricity_growth": ("rad/√h", "scalar", ("How fast the eccentricity grows with the square root of "
                                                         "vibration hours (a loosening mount).")),
        },
        aliases={
            "latency_s": a("latency"),
            "eccentricity_growth_mrad_per_sqrt_h": a("eccentricity_growth", unit="mrad/√h"),
            "encoder_counts_per_rev": a("counts_per_rev"),
            "encoder_counts": a("counts_per_rev"),
            "encoder_lines": a("counts_per_rev", "Lines: a quadrature encoder counts 4 × lines per revolution."),
            "encoder_bits": a("counts_per_rev", "Bits: 2^bits counts per revolution."),
        },
    ),
    "camera": T(
        folder="sensors/camera",
        title="A frame camera's colour image: projection, distortion, exposure.",
        cards=("camera_rgb",), category="camera", datasheet=None,
        rungs=(
            R("M0.pinhole", "Pinhole projection", None, ("fx_fy", "cx_cy", "extrinsics"),
              equation="p = K [R | t] X, K from fx_fy and cx_cy, [R | t] the extrinsics", refs=("ZHANG_2000",)),
            R("M1.distortion", "Brown-Conrady distortion", "M0.pinhole", ("distortion",),
              equation="radial (k1, k2, k3) and tangential (p1, p2) distortion of the projected point",
              refs=("ZHANG_2000", "BROWN_1966", "HEIKKILA_1997")),
            R("M2.exposure_latency", "Exposure response and latency", "M1.distortion",
              ("exposure_response", "latency"),
              equation="DN = exposure_response·illuminance·exposure time; the frame arrives latency after exposure",
              refs=("ZHANG_2000",)),
            R("M3.rolling_shutter", "Rolling shutter: a line delay", "M2.exposure_latency", ("line_delay",),
              equation="row v exposed at t0 + v·line_delay", refs=("OTH_2013", "MEINGAST_2005")),
            R("M3.wear", "A mount that loosens over vibration hours", "M3.rolling_shutter", ("extrinsic_drift",),
              wears=(("extrinsics", *VIBRATION, "sqrt", "extrinsic_drift"),),
              equation="translation offset(h) = e + extrinsic_drift·√h", refs=("JUNKER_1969", "WHITMORE_1995")),
            residual("M3.wear", "M3.wear + a B-spline grid of ray offsets over the pixel (u, v)", ("SCHOPS_2020",)),
        ),
        parameters={
            "fx_fy": ("px", "pair", "Focal lengths in pixels, (fx, fy)."),
            "cx_cy": ("px", "pair", "Principal point in pixels, (cx, cy)."),
            "extrinsics": ("m, rad", "pose", "The camera's pose in the frame it is calibrated against."),
            "distortion": ("", "list", "Brown-Conrady lens distortion coefficients, radial and tangential."),
            "exposure_response": ("DN/(lx·s)", "scalar", ("Pixel value per unit of light exposure: the sensor's "
                                                          "response.")),
            "latency": ("s", "scalar", "Delay from exposure to the frame's arrival."),
            "line_delay": ("s", "scalar", "Rolling shutter: the time between the exposures of successive rows."),
            "extrinsic_drift": ("m/√h", "scalar", ("How fast the extrinsics' translation drifts with the square root "
                                                   "of vibration hours (a loosening mount); no source states a "
                                                   "rotation rate.")),
        },
        aliases={
            "latency_s": a("latency"),
            "camera_latency_s": a("latency"),
            "camera_latency_ms": a("latency", unit="ms"),
            "line_delay_s": a("line_delay"),
            "extrinsic_drift_mm_per_sqrt_h": a("extrinsic_drift", unit="mm/√h"),
        },
    ),
    "depth_camera": T(
        folder="sensors/rgbd",
        title="A stereo depth camera: depth on top of a colour stream.",
        cards=("camera_rgbd",), category="camera", datasheet=None,
        rungs=(
            R("M0.stereo", "Ideal stereo", None,
              ("stereo_focal_length", "baseline", "depth_scale", "depth_to_color"),
              equation="z = stereo_focal_length·baseline/d, d the disparity, scaled by depth_scale",
              refs=("KHOSHELHAM_2012",)),
            R("M1.disparity_noise", "Disparity noise", "M0.stereo", ("subpixel_sigma",),
              equation="σ_z = z²·subpixel_sigma/(stereo_focal_length·baseline)", refs=("KHOSHELHAM_2012",)),
            R("M2.depth_bias", "Depth bias", "M1.disparity_noise", ("depth_bias_coeffs",),
              equation="z_true = z + b(u, v, z), b from depth_bias_coeffs", refs=("HERRERA_2012",)),
            R("M2.wear", "A depth-to-colour mount that loosens over vibration hours", "M2.depth_bias",
              ("depth_to_color_drift",),
              wears=(("depth_to_color", *VIBRATION, "sqrt", "depth_to_color_drift"),),
              equation="translation offset(h) = e + depth_to_color_drift·√h", refs=("JUNKER_1969", "WHITMORE_1995")),
            residual("M2.wear", "M2.wear + a B-spline grid of depth corrections over (u, v, measured depth)",
                     ("SCHOPS_2020", "HERRERA_2012")),
        ),
        parameters={
            "stereo_focal_length": ("px", "scalar", ("The stereo imagers' rectified focal length (on a typical unit "
                                                     "the IR pair): the f of triangulation, not the colour camera's.")),
            "baseline": ("m", "scalar", "Distance between the stereo pair's optical centres."),
            "depth_scale": ("m", "scalar", "Metres per unit of the depth image."),
            "depth_to_color": ("m, rad", "pose", "The depth frame's pose in the colour camera's frame."),
            "subpixel_sigma": ("px", "scalar", ("Standard deviation of the disparity: the stereo matcher's subpixel "
                                                "noise.")),
            "depth_bias_coeffs": ("m", "list", ("Coefficients of a depth bias that grows with depth and varies "
                                                "across the image.")),
            "depth_to_color_drift": ("m/√h", "scalar", ("How fast depth_to_color's translation drifts with the square "
                                                        "root of vibration hours; no source states a rotation rate.")),
        },
        aliases={
            "baseline_m": a("baseline"),
            "depth_to_color_drift_mm_per_sqrt_h": a("depth_to_color_drift", unit="mm/√h"),
        },
    ),
    "hyperspectral_camera": T(
        folder="sensors/hyperspectral_camera",
        title="A line-scan spectral camera over a belt.",
        cards=("camera_hsi",), category="camera", datasheet=None,
        rungs=(
            R("M0.bands", "Ideal spectral bands", None, ("band_centres", "dark_white_refs"),
              equation="R(λ) = (DN − dark)/(white − dark)", refs=("BURGER_2011",)),
            R("M1.band_widths", "Band widths", "M0.bands", ("band_fwhm",),
              equation="each band a Gaussian of its band_fwhm about its band_centres",
              refs=("BURGER_2011", "KRUSE_1993")),
            R("M2.smile_keystone", "Smile and keystone", "M1.band_widths", ("smile_keystone_map",),
              equation="the cube resampled through smile_keystone_map, then R(λ) per band", refs=("MOUROULIS_2000",)),
            R("M3.pushbroom", "Pushbroom line geometry", "M2.smile_keystone", ("line_geometry",),
              equation="a line camera swept by the belt: line k sees the belt at k·(belt distance per line)",
              refs=("GUPTA_1997",)),
            R("M3.wear", "References that drift with hours", "M3.pushbroom", ("reference_drift",),
              wears=(("dark_white_refs", *HOURS, "linear", "reference_drift"),),
              equation="white(h) = white + reference_drift·h", refs=("LU_MEEKER_1993",)),
        ),
        parameters={
            "band_centres": ("m", "list", "Centre wavelength of each spectral band."),
            "dark_white_refs": ("DN", "pair", ("The dark and white reference frames that turn raw counts into "
                                               "reflectance.")),
            "band_fwhm": ("m", "list", "Full width at half maximum of each band's spectral response."),
            "smile_keystone_map": ("px", "map", ("The spectral (smile) and spatial (keystone) distortion map the cube "
                                                 "is resampled with.")),
            "line_geometry": ("m, rad", "pair", "The scan line's pose, and the belt distance per line."),
            "reference_drift": ("DN/h", "scalar", ("How fast the white reference's DN drifts per deployment hour "
                                                   "(lamp ageing). The dark reference drifts the other way "
                                                   "(dark current) and is not modelled by this rate.")),
        },
        aliases={"reference_drift_per_h": a("reference_drift")},
    ),
    "rigid_body_chain": T(
        folder="arms/rigid_body_chain",
        title="A chain of rigid links and joints, open or closed: a whole arm, a sub-assembly, a wrist, a five-bar.",
        cards=("arm_6dof", "arm_subassembly"), category="arm", datasheet=None,
        rungs=(
            R("M0.nominal", "Nominal kinematics", None, ("link_geometry",),
              equation="tool = FK(q; link_geometry)", refs=("SICILIANO_2009", "DH_1955")),
            R("M1.zero_offsets", "Joint zero offsets", "M0.nominal", ("joint_zero_offsets",),
              equation="tool = FK(q + joint_zero_offsets; link_geometry)", refs=("HOLLERBACH_WAMPLER_1996",)),
            R("M2.geometric", "Geometric (DH) errors", "M1.zero_offsets", ("dh_errors",),
              equation="tool = FK(q + joint_zero_offsets; link_geometry + dh_errors)",
              refs=("HAYATI_1983", "OKAMURA_1996", "HOLLERBACH_2016"),
              note="sim2real's arm M2: the kinematic branch of an open chain."),
            R("M2.inertial", "Link inertials and loop closure", "M1.zero_offsets",
              ("link_inertials", "closure_tolerance"),
              equation=("tool = FK(q + joint_zero_offsets; link_geometry); dynamics from link_inertials with each "
                        "joint's own type; "
                        "a closed loop may stay open by closure_tolerance"),
              refs=("ATKESON_1986", "GAUTIER_1990", "SWEVERS_1997"),
              note="sim2real's arm sub-assembly M2: the dynamic branch, which needs no DH errors."),
            R("M2.wear", "Joint zeros that drift over vibration hours", "M2.inertial", ("joint_zero_drift",),
              wears=(("joint_zero_offsets", *VIBRATION, "sqrt", "joint_zero_drift"),),
              equation="joint_zero_offsets(h) = joint_zero_offsets + joint_zero_drift·√h",
              refs=("JUNKER_1969", "WHITMORE_1995")),
            R("M3.compliance", "Joint compliance and link inertials", "M2.geometric",
              ("joint_compliance", "link_inertials"),
              equation=("tool = FK(q + joint_zero_offsets + joint_compliance·τ; link_geometry + dh_errors); τ from "
                        "link_inertials, each joint its own type"),
              refs=("SPONG_1987", "ATKESON_1986", "GAUTIER_1990", "SWEVERS_1997")),
            R("M3.wear", "Joint compliance that grows with use (Archard)", "M3.compliance", ("compliance_wear",),
              wears=(("joint_compliance", "torque-weighted joint revolutions", "N·m·rev", "archard",
                      "compliance_wear"),),
              equation="joint_compliance(u) = joint_compliance + compliance_wear·Σ|τ|·Δrev",
              refs=("ARCHARD_1953", "FLODIN_1997")),
            residual("M3.wear", "M3.wear + an MLP residual of joint torques from 3 samples of each joint's error, "
                                "speed and torque, added to M3.compliance's τ, so it moves the tool through the "
                                "compliance", ("NGUYEN_TUONG_2010", "ZENG_2020")),
            R("M6.inertial_residual", "Grey-box residual on M2.wear", "M2.wear",
              equation="M2.wear + an MLP residual of joint torques from 3 samples of each joint's error, speed and "
                       "torque", refs=("NGUYEN_TUONG_2010", "ZENG_2020"),
              note="the residual of the dynamic branch (sim2real's arm sub-assembly M6)."),
            learned("an MLP over 3 samples of each joint's error and speed → joint torques", ("HWANGBO_2019",)),
        ),
        parameters={
            "link_geometry": ("m, rad", "per joint", ("The chain's nominal joint axes and link frames, as a URDF, a "
                                                      "DH table or a CAD export states them.")),
            "joint_zero_offsets": ("rad", "per joint", ("Each joint's offset between its encoder zero and its "
                                                        "kinematic zero.")),
            "dh_errors": ("m, rad", "per joint", "Errors of the chain's DH parameters from their nominal values."),
            "link_inertials": ("kg, m, kg·m²", "per link", "Each link's mass, centre of mass and inertia tensor."),
            "closure_tolerance": ("m", "scalar", "How far a closed chain's loop may stay open at its closure joint."),
            "joint_zero_drift": ("rad/√h", "scalar", ("How fast the joint zero offsets drift with the square root of "
                                                      "vibration hours.")),
            "joint_compliance": ("rad/(N·m)", "per joint", "Each joint's deflection per N·m it carries."),
            "compliance_wear": ("rad/(N·m)/(N·m·rev)", "scalar", ("How fast joint_compliance grows per "
                                                                  "torque-weighted joint revolution (Archard).")),
        },
        aliases={
            "link_lengths": a("link_geometry", "Lengths only: the geometry also carries the joint axes."),
            "joint_offsets": a("joint_zero_offsets"),
            "joint_offset_drift_rad_per_sqrt_h": a("joint_zero_drift"),
            "joint_zero_drift_rad_per_sqrt_h": a("joint_zero_drift"),
            "compliance_wear_per_nm_rev": a("compliance_wear"),
        },
        deployment={
            "base_frame": ("assembly", "Where the chain stands in its cell: the description's."),
            "tool_frame": ("assembly", "The tool mounted on its flange: the description's."),
        },
    ),
    "gripper": T(
        folder="end_effectors/gripper",
        title="An end effector that grasps, on a chain's flange.",
        cards=("arm_6dof_gripper",), category="gripper", datasheet=None,
        rungs=(
            R("M0.tcp", "A rigid gripper at the tool centre point", None, ("tcp_offset",),
              equation="tool = flange · tcp_offset", refs=("SICILIANO_2009",)),
            R("M1.width_map", "Finger width map", "M0.tcp", ("finger_width_map",),
              equation="width = finger_width_map(command)", refs=("PRATTICHIZZO_2016",)),
            R("M2.contact", "Grip force and contact friction", "M1.width_map", ("grip_force_map", "contact_friction"),
              equation=("force = grip_force_map(command, width); the object holds while, at each contact, "
                        "|tangential| ≤ contact_friction·force (two pads hold about twice one pad's)"),
              refs=("PRATTICHIZZO_2016", "MUJOCO_CONTACT")),
            R("M2.wear", "Finger pads that glaze with grasps", "M2.contact", ("contact_friction_change",),
              wears=(("contact_friction", "grasp cycles", "cycle", "linear", "contact_friction_change"),),
              equation="contact_friction(n) = contact_friction + contact_friction_change·n",
              refs=("ARCHARD_1953", "LU_MEEKER_1993")),
            residual("M2.wear", "M2.wear + an MLP residual width from 3 samples of width command, measured width and "
                                "finger current", ("NGUYEN_TUONG_2010",)),
            learned("an MLP over 5 samples of width command, width and finger current → finger force",
                    ("HWANGBO_2019",)),
        ),
        parameters={
            "tcp_offset": ("m", "pose", "The tool centre point's offset from the flange the gripper mounts on."),
            "finger_width_map": ("m", "map", "Finger opening as a function of the width command."),
            "grip_force_map": ("N", "map", "Grip force as a function of the command and the width."),
            "contact_friction": ("", "scalar", "Coefficient of friction between the finger pads and the object."),
            "contact_friction_change": ("1/cycle", "scalar", ("How much contact_friction changes per grasp cycle (pad "
                                                              "wear and glazing).")),
        },
        aliases={"contact_friction_change_per_cycle": a("contact_friction_change")},
    ),
    "conveyor": T(
        folder="conveyors/conveyor",
        title="A belt driven through a drum.",
        cards=("conveyor",), category="", datasheet=None,
        rungs=(
            R("M0.ideal", "An ideal belt: kinematics and a rigid mass", None,
              ("drum_radius", "gear_ratio", "moving_mass"),
              equation="x = θ_motor·drum_radius/gear_ratio; (moving_mass + load)·ẍ = F", refs=("ZHANG_XIA_2011",),
              note="numbered by the actuator's layers (sim2real's decision); no M1: no Datasheet class yet."),
            R("M2.coulomb_viscous", "Coulomb and viscous friction on the belt", "M0.ideal",
              ("coulomb_friction", "viscous_friction"),
              equation="F_f = coulomb_friction·sign(ẋ) + viscous_friction·ẋ",
              refs=("ARMSTRONG_1994", "ZHANG_XIA_2011")),
            R("M3.stribeck", "Static friction (Stribeck)", "M2.coulomb_viscous",
              ("breakaway_force", "stribeck_velocity"),
              equation=("F_f = (coulomb_friction + (breakaway_force − coulomb_friction)·exp(−(|ẋ|/stribeck_velocity)²))"
                        "·sign(ẋ) + viscous_friction·ẋ"),
              refs=("ARMSTRONG_1994",), note="the actuator's M3.stribeck as forces on the belt."),
            R("M4.stretch", "Belt stretch: drum and load as two masses", "M3.stribeck",
              ("stretch_stiffness", "stretch_damping", "drum_mass"),
              equation=("drum: drum_mass·ẍ_d = gear_ratio·τ/drum_radius − stretch_stiffness·(x_d − x_l) "
                        "− stretch_damping·(ẋ_d − ẋ_l); load: (moving_mass − drum_mass + load)·ẍ_l = the same "
                        "stretch force − F_f"),
              refs=("SPONG_1987", "HE_2017"),
              note="M4 splits moving_mass: drum_mass on the drive side of the stretch, the rest moves with the load."),
            R("M4.wear", "Friction that grows with belt distance", "M4.stretch", ("friction_growth",),
              wears=(("breakaway_force", "belt distance", "km", "linear", "friction_growth"),
                     ("coulomb_friction", "belt distance", "km", "linear", "friction_growth")),
              equation=("breakaway_force and coulomb_friction both grow by friction_growth·s "
                        "(the Stribeck drop stays)"),
              refs=("LU_MEEKER_1993", "ZHANG_XIA_2011")),
            residual("M4.wear", "M4.wear + an MLP residual drive force from 3 samples of belt speed error, belt speed "
                                "and motor current", ("NGUYEN_TUONG_2010", "ZENG_2020")),
            learned("an MLP over 3 samples of position error and belt speed → drive force",
                    ("HWANGBO_2019", "HWANGBO_2019_ARXIV")),
        ),
        parameters={
            "drum_radius": ("m", "scalar", "Radius of the drive drum the belt wraps."),
            "gear_ratio": ("", "scalar", "Motor turns per drum turn."),
            "moving_mass": ("kg", "scalar", "The moving mass referred to the belt: belt, drum and reflected rotor."),
            "coulomb_friction": ("N", "scalar", ("Coulomb friction force on the belt: the constant force that opposes "
                                                 "its motion.")),
            "viscous_friction": ("N·s/m", "scalar", "Viscous friction coefficient on the belt."),
            "breakaway_force": ("N", "scalar", ("Static friction on the belt: the force that starts it moving from "
                                                "rest.")),
            "stribeck_velocity": ("m/s", "scalar", ("Belt speed over which friction falls from breakaway_force to "
                                                    "coulomb_friction.")),
            "stretch_stiffness": ("N/m", "scalar", "The belt's stiffness along its length, between drum and load."),
            "stretch_damping": ("N·s/m", "scalar", "Damping of the belt's stretch between drum and load."),
            "drum_mass": ("kg", "scalar", ("The drive side's mass referred to the belt, read from M4: the drum and "
                                           "the reflected rotor; moving_mass less drum_mass moves with the load.")),
            "friction_growth": ("N/km", "scalar", ("How fast the friction level grows per kilometre of belt travel "
                                                   "(idlers and slider bed wearing); breakaway_force and "
                                                   "coulomb_friction grow together.")),
        },
        aliases={
            "drum_radius_m": a("drum_radius"),
            "drum_radius_mm": a("drum_radius", unit="mm"),
            "moving_mass_kg": a("moving_mass"),
            "coulomb_n": a("coulomb_friction"),
            "viscous_n_s_m": a("viscous_friction"),
            "static_n": a("breakaway_force"),
            "stretch_stiffness_n_m": a("stretch_stiffness"),
            "stretch_damping_n_s_m": a("stretch_damping"),
            "coulomb_growth_n_per_km": a("friction_growth"),
        },
        deployment={
            "load_mass_kg": ("assembly", "The parcels on the belt: the deployment's."),
            "encoder_counts_per_m": ("config", "Derived from the encoder, the ratio and the drum radius."),
        },
    ),
    "aruco_marker": T(
        folder="markers/aruco_marker",
        title="A printed sheet carrying one or more ArUco markers rigidly.",
        cards=("aruco_marker",), category="", datasheet=None,
        rungs=(
            R("M0.nominal", "The sheet as designed, where CAD put it", None,
              ("dictionary", "marker_ids", "marker_length", "layout", "pose_on_link"),
              equation="X_corner = FK(q) · pose_on_link · layout_corner", refs=("ZHANG_2000", "GARRIDO_2014")),
            R("M1.as_built", "How the sheet was actually stuck down", "M0.nominal", ("as_built_offset",),
              equation="X_corner = FK(q) · pose_on_link · as_built_offset · layout_corner", refs=("TRIGGS_2000",)),
            R("M2.print_scale", "Print scale", "M1.as_built", ("print_scale",),
              equation="X_corner = FK(q) · pose_on_link · as_built_offset · (print_scale · layout_corner)",
              refs=("STROBL_2011",)),
            R("M3.curl", "A sheet that is not flat", "M2.print_scale", ("flatness",),
              equation="layout_corner z = flatness·(2x/L)² (a bow across the sheet's length L)", refs=("STROBL_2011",)),
        ),
        parameters={
            "dictionary": ("", "text", "The ArUco dictionary the sheet is printed in."),
            "marker_ids": ("", "list", "The marker ids the sheet carries."),
            "marker_length": ("m", "scalar", "Side of each marker's black square."),
            "layout": ("m", "map", "Each marker's corners in the sheet's own frame."),
            "pose_on_link": ("m, rad", "pose", "The sheet's frame on the link it is stuck to."),
            "as_built_offset": ("m, rad", "pose", ("How the sheet was actually stuck down: its offset from "
                                                   "pose_on_link.")),
            "print_scale": ("", "scalar", "Printed size over designed size."),
            "flatness": ("m", "scalar", "The sheet's out-of-plane bow at its far end."),
        },
        aliases={
            "marker_length_m": a("marker_length"),
            "layout_m": a("layout"),
            "flatness_m": a("flatness"),
        },
    ),
}

# robotparts' eight actuator rungs, as published (robotparts @ d16f6a8, models/actuator/<word>/). dcmotor has no
# reference code: only MuJoCo implements it, so its folder is model.json and README.md alone.
RUNG_FILES = {
    "ideal": {
        "README.md": "23e453d4989df9f01d788714026655079ed4284c02a4683e204581815ecb7103",
        "model.js": "533668d33e34e67d5e6d1dcc04fdf3f3f9d1b5d7d409f7f67c264d808a28e805",
        "model.json": "4bcee8de7d6f1390d9492115e05026d8a5d67f3734120ccc3de0b95d25fc82e2",
        "model.py": "ba6b33a4cbd28aea7cb1874e6e646e9f5dd34c67a33290cb489296aeda0d4b76",
    },
    "datasheet": {
        "README.md": "41119901a62f3d35bd1affa6052505141bcb4d37418896ca7f3249a16297c2b3",
        "model.js": "bb054894e901a1d6464545efbbe0e81975c5be4cd8463eac105c01985e64c41e",
        "model.json": "03b234a7f22220cd1eae425edca3af6d9ea19ab41a8af812fc91a9fdaeb74e6c",
        "model.py": "3ed5e7d8d90d98efec0af84957f271ca20f365c62f660739be25b8ccad1ec2dc",
    },
    "coulomb_viscous": {
        "README.md": "529c2bf1f968b67adf5806a6a49d9a192dfdeb36b1d805f99a04e02d18af6530",
        "model.js": "5b8131ce0acd3f0d844985fb888d0ef91d553342ea2980896901b010db0c1e00",
        "model.json": "1638a61405f7e54835d809633a899c652749fdd773780301e5ae1abff634e0a8",
        "model.py": "e20d6f415822572bb89f305344dce3efc659a9db2822a34300141afa5665b095",
    },
    "stribeck": {
        "README.md": "5a1c264a105f1425b918710e6e478bc9980bffeb2e78c893273b1d72017b96cc",
        "model.js": "15bdb5ab166da818de23201e58f524d1cfb5e0604aa64ecad9f2ce51d68f1c34",
        "model.json": "336b22e8b61428b5cc58a24504d711778a59b455c29875152e0da7fefb77b8d8",
        "model.py": "87f9bc8becd9dd5db7e1811b349fb96eb31e422435ab4c18ed43db7ab3952a5b",
    },
    "load": {
        "README.md": "261bf3e5975117bb9b0dbff795183092848149697f63b9e580f4d2df2df7db0a",
        "model.js": "d50eb2222039019497aa11972aca5479a7517a429ef2b99c28cd8655c9b3a0d0",
        "model.json": "fc915dd8f01476838ebb081de75b37b12b3ed2e002abbc0754789a8396f6d157",
        "model.py": "03b5cb124ad4a66bffa3298b4fccf831f17c33c147d15c43d99ef44920587033",
    },
    "lugre": {
        "README.md": "657e977e53358fbb5324deaf059284b85693baea1c0355904e695aadb77ce552",
        "model.js": "e8e42ee33d8acfef3fa7be6542c68ee06c165ff1dc2f3dd67787424a0009cb0a",
        "model.json": "560a2029e8846474a357466fbcb584451a81c3f99dd5507015e9a06914fd88aa",
        "model.py": "63cb93d20325eb8bfe76e7fa2899928c725eb5de354b45619830c7ef4d59cde0",
    },
    "ripple": {
        "README.md": "de559c03e6c7dbe2ca3eeb2ec07e5ed0ce1b68b5bd246cadc2ee46ff374bb3af",
        "model.js": "31f08577ba8ccdf3c12dd8bbe672a67ac870e94b1ccb6fc348fc64cb0640ddba",
        "model.json": "426b7ff1b64c6c9c3f21c8fafd4ba209e1b23c8423f6d35782e20189d31e1b6e",
        "model.py": "e30eb5e3ed7c4ae09b06fc448e31e52c81e5a4cad9bd3e9d0e61b504c6d0395c",
    },
    "dcmotor": {
        "README.md": "aabc9ebce725dcfb2b3ee41c16e01e32e85d43ea7c41c09cb212ed72949709f7",
        "model.json": "cfae50f856db6518519ecc18817b5745361a3b0368b8586641096f4668405931",
    },
}

# Each folder's model.json `name` and `terms` with their `required` flags, as published (same commit): what the spec is
# checked against now, before the folders move (test_the_spec_holds_every_robotparts_term).
RUNG_TERMS = {
    "ideal": ("M0.ideal", {"output_inertia": True, "gear_ratio": False}),
    "datasheet": ("M1.datasheet", {"output_inertia": False, "rotor_inertia": False, "gearbox_input_inertia": False,
                                   "gear_ratio": False, "kt": False, "coulomb_friction": False,
                                   "viscous_friction": False}),
    "coulomb_viscous": ("M2.coulomb_viscous", {"coulomb_friction": True, "viscous_friction": True,
                                               "output_inertia": False}),
    "stribeck": ("M3.stribeck", {"coulomb_friction": True, "viscous_friction": True, "breakaway_torque": True,
                                 "stribeck_velocity": True, "stribeck_exponent": False, "output_inertia": False}),
    "load": ("M3.load", {"coulomb_friction": True, "viscous_friction": True, "friction_load_coefficient": True,
                         "output_inertia": False}),
    "lugre": ("M3.lugre", {"coulomb_friction": True, "breakaway_torque": True, "stribeck_velocity": True,
                           "sigma0": True, "sigma1": True, "sigma2": True, "stribeck_exponent": False,
                           "output_inertia": False}),
    "ripple": ("M4.ripple", {"harmonics": True, "coulomb_friction": False, "viscous_friction": False,
                             "output_inertia": False}),
    "dcmotor": ("M5.dcmotor", {"motorconst": True, "resistance": True, "nominal": False, "saturation": False,
                               "inductance": False, "cogging": False, "controller": False, "thermal": False,
                               "lugre": False}),
}

# What a type folder's modules must never hold: a maker or SKU, a unit's identity, a path into a machine or a repo.
FORBIDDEN = (
    r"steadywin", r"\bgim\s?\d", r"mjbots", r"moteus", r"\bjmc\b", r"60asm", r"stepperonline", r"\bad60\b", r"aifitlab",
    r"realsense", r"\bintel\b", r"\bd4\d\d\b", r"specim", r"\bfx1\d\b", r"robotiq", r"universal robots", r"\bur\d+e?\b",
    r"ufactory", r"\bxarm", r"odrive", r"cubemars", r"t-motor", r"myactuator", r"dynamixel", r"robotis", r"\bmaxon\b",
    r"as5047", r"\bma600\b", r"acaproto", r"fdcanusb", r"\bbam\b", r"bam_", r"\bcan\s?\d+\b", r"can_id",
    r"/home/", r"/users/", r"[a-z]:\\\\", r"~/",
)
# A number written inside text (a fitted value in a note); a DOI or a URL is a citation, never a value (the fidelity
# review: "10.1109/9.376053" is no number).
NUMBER_IN_TEXT = r"\d+\.\d{3,}"
CITATION = re.compile(r"\b10\.\d{4,9}/\S+|https?://\S+")


# --- helpers -----------------------------------------------------------------------------------------------------


def datasheet_optional(spec: T) -> tuple[str, ...]:
    m0 = next(rung for rung in spec.rungs if rung.key.startswith("M0."))
    return tuple(name for name in spec.datasheet.typed_fields() if name not in m0.adds)


def optional_of(spec: T, rung: R) -> tuple[str, ...]:
    return datasheet_optional(spec) if rung.optional == DATASHEET else tuple(rung.optional)


def build(type_name: str) -> Ladder:
    """The spec as a Ladder, through the core's own checks: proof the ruler asks for a ladder that can exist."""
    spec = TYPES[type_name]
    vocabulary = datasheet_vocabulary(spec.datasheet) if spec.datasheet else ()
    vocabulary += tuple(Parameter(name, unit, definition, shape)
                        for name, (unit, shape, definition) in spec.parameters.items())
    rungs = []
    for rung in spec.rungs:
        level, name = rung.key[1:].split(".", 1)  # the spec's own keys: the core never parses one
        kind = {"6": "residual", "7": "learned"}.get(level, "physics")
        rungs.append(Rung(int(level), name, rung.title, rung.base, rung.adds, kind, optional_of(spec, rung),
                          rung.code, tuple(Wear(*wear) for wear in rung.wears), rung.composes, rung.equation,
                          tuple(getattr(REFS, ref) for ref in rung.refs), rung.note))
    return Ladder(type_name, spec.title, vocabulary, tuple(rungs),
                  tuple(Alias(alias, canonical, note, unit) for alias, (canonical, note, unit) in spec.aliases.items()),
                  tuple(Packed(name, holds, note, ordered) for name, (holds, ordered, note) in spec.packed.items()),
                  tuple(DeploymentOnly(name, role, why) for name, (role, why) in spec.deployment.items()),
                  spec.datasheet)


def scan(text: str, numbers: bool = True) -> list[str]:
    """Each forbidden token in a module's text; with `numbers`, also each number it holds that is not a level."""
    found = [pattern for pattern in FORBIDDEN if re.search(pattern, text, re.IGNORECASE)]
    if not numbers:
        return found
    if re.search(NUMBER_IN_TEXT, CITATION.sub("", text)):
        found.append(f"a number written in text ({NUMBER_IN_TEXT}): type data names parameters, never values")
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float) and not (
                type(node.value) is int and 0 <= node.value <= 7):
            found.append(f"the number {node.value!r} (line {node.lineno}): type data names parameters, never values")
    return found


def ladder_of(type_name: str) -> Ladder:
    """The type's own ladder.py, imported directly: another lane's state never decides this type's verdict."""
    spec = TYPES[type_name]
    path = SRC / spec.folder / "ladder.py"
    assert path.is_file(), f"no ladder for {type_name}: write {spec.folder}/ladder.py with LADDER = Ladder(...)"
    module = importlib.import_module("robot_models." + spec.folder.replace("/", ".") + ".ladder")
    ladder = getattr(module, "LADDER", None)
    assert isinstance(ladder, Ladder), f"{spec.folder}/ladder.py defines no LADDER = Ladder(...)"
    assert ladder.type == type_name, f"{spec.folder}/ladder.py is type {ladder.type!r}, not {type_name!r}"
    return ladder


# --- the spec itself -----------------------------------------------------------------------------------------------


def test_the_spec_is_itself_a_valid_ladder():
    built = {name: build(name) for name in TYPES}
    assert len(built) == 12
    assert sum(len(ladder.rungs) for ladder in built.values()) == 84  # 83 + bldc_motor's M3.stribeck (amendment 1)
    for ladder in built.values():
        for rung in ladder.rungs:
            for other, key in rung.composes:
                built[other].rung(key)


def test_the_spec_holds_every_robotparts_term():
    """Each moved folder's terms: a required one is a parameter its servo rung needs, an optional one one it holds."""
    servo = build("integrated_servo")
    deployment = {other.name for other in servo.deployment}
    for word, (key, terms) in RUNG_TERMS.items():
        rung = servo.rung(key)
        assert rung.code == f"{RUNGS_FOLDER}/{word}", word
        for term, required in terms.items():
            if term in deployment:
                continue
            target = set(servo.parameters(key) if required else servo.holds(key))
            assert set(servo.resolve(term)) <= target, f"{word}'s {term} (required {required}) on {key}"


# --- the type folders --------------------------------------------------------------------------------------------


def test_exactly_the_twelve_types_are_discovered_at_their_folders():
    found = discover()
    assert sorted(found) == sorted(TYPES)
    assert {name: item.folder for name, item in found.items()} == {name: spec.folder for name, spec in TYPES.items()}


def test_every_composition_names_a_landed_type_and_rung():
    found = discover()  # WHY its own test: discover() checks compositions across types, so it waits for every lane
    for type_name, spec in TYPES.items():
        for rung in spec.rungs:
            for other, key in rung.composes:
                assert type_name in found and other in found, f"{type_name} {rung.key} composes {other} {key}"
                assert (other, key) in found[type_name].ladder.rung(rung.key).composes
                found[other].ladder.rung(key)


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_the_ladder_lives_in_its_type_folder(type_name):
    assert ladder_of(type_name).title.strip()


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_rung_keys_bases_and_what_each_adds(type_name):
    ladder, spec = ladder_of(type_name), TYPES[type_name]
    assert sorted(ladder.keys) == sorted(rung.key for rung in spec.rungs)
    for rung in spec.rungs:
        built = ladder.rung(rung.key)
        assert built.base == rung.base, rung.key
        assert sorted(built.adds) == sorted(rung.adds), rung.key
        assert sorted(built.optional) == sorted(optional_of(spec, rung)), rung.key
        assert built.code == rung.code, rung.key
        assert dict(built.composes) == dict(rung.composes), rung.key
        assert {(w.parameter, w.usage_unit, w.law, w.rate) for w in built.wears} == {
            (w[0], w[2], w[3], w[4]) for w in rung.wears}, rung.key


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_every_rung_shows_its_equation_and_references(type_name):
    """Zach, Nov 5 (settled point 3): each rung shows its parameters, its equation and its references."""
    ladder, spec = ladder_of(type_name), TYPES[type_name]
    for rung in spec.rungs:
        built = ladder.rung(rung.key)
        assert built.equation.strip(), f"{rung.key}: write its equation (the spec's: {rung.equation})"
        cited = set(built.references)
        missing = [ref for ref in rung.refs if getattr(REFS, ref) not in cited]
        assert not missing, f"{rung.key} cites robot_models.references {missing}"


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_the_vocabulary_its_units_and_shapes(type_name):
    ladder, spec = ladder_of(type_name), TYPES[type_name]
    datasheet = spec.datasheet.typed_fields() if spec.datasheet else {}
    assert sorted(ladder.names) == sorted([*datasheet, *spec.parameters])
    assert ladder.datasheet is spec.datasheet
    for name, field_spec in datasheet.items():
        assert ladder.parameter(name).unit == si_unit(field_spec.units), name
    for name, (unit, shape, _) in spec.parameters.items():
        assert (ladder.parameter(name).unit, ladder.parameter(name).shape) == (unit, shape), name


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_every_alias_resolves_with_its_note_and_unit(type_name):
    ladder, spec = ladder_of(type_name), TYPES[type_name]
    assert sorted(alias.name for alias in ladder.aliases) == sorted(spec.aliases)
    for alias, (canonical, note, unit) in spec.aliases.items():
        assert ladder.canonical(alias) == canonical, alias
        assert ladder.alias(alias).unit == unit, f"{alias}'s values are in {unit or 'the SI unit'}"
        if note:
            assert ladder.alias(alias).note.strip(), f"{alias} is not {canonical} one for one: its note says how"


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_every_packed_name_unpacks_and_is_never_one_parameter(type_name):
    ladder, spec = ladder_of(type_name), TYPES[type_name]
    assert sorted(packed.name for packed in ladder.packed) == sorted(spec.packed)
    for name, (holds, ordered, _) in spec.packed.items():
        assert ladder.unpack(name) == holds if ordered else set(ladder.unpack(name)) == set(holds), name
        assert ladder.packing(name).ordered is ordered, name
        assert ladder.packing(name).note.strip(), f"{name}: say what it packs and where its layout is defined"
        with pytest.raises(KeyError):
            ladder.canonical(name)


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_deployment_only_names(type_name):
    ladder, spec = ladder_of(type_name), TYPES[type_name]
    assert {other.name: other.role for other in ladder.deployment} == {
        name: role for name, (role, _) in spec.deployment.items()}


# --- robotparts' eight rungs, moved in byte for byte ------------------------------------------------------------


@pytest.mark.parametrize("word", sorted(RUNG_FILES))
def test_the_rung_folder_is_byte_identical_to_robotparts(word):
    folder = SRC / RUNGS_FOLDER / word
    assert folder.is_dir(), f"move robotparts' models/actuator/{word}/ to {RUNGS_FOLDER}/{word}/ unchanged"
    # WHY files only, dot-files skipped: robotparts' build hashes every non-dot file in the folder and publishes it,
    # so nothing else may sit there; a __pycache__ directory is not a file and is skipped by its build too (the sync
    # check of the next phase copies files, never the folder).
    files = {path.name for path in folder.iterdir() if path.is_file() and not path.name.startswith(".")}
    assert files == set(RUNG_FILES[word]), f"{word}: exactly {sorted(RUNG_FILES[word])}, nothing else"
    for name, sha256 in RUNG_FILES[word].items():
        assert hashlib.sha256((folder / name).read_bytes()).hexdigest() == sha256, f"{word}/{name} changed"


@pytest.mark.parametrize("word", sorted(RUNG_FILES))
def test_each_rung_folder_is_the_servo_rung_that_names_it(word):
    ladder = ladder_of("integrated_servo")
    definition = json.loads((SRC / RUNGS_FOLDER / word / "model.json").read_text(encoding="utf-8"))
    rung = ladder.rung(definition["name"])
    assert (rung.level, rung.code) == (definition["rung"], f"{RUNGS_FOLDER}/{word}")
    deployment = {other.name for other in ladder.deployment}
    for term, spec in definition["terms"].items():
        if term in deployment:
            continue
        target = set(ladder.parameters(rung.key) if spec.get("required") else ladder.holds(rung.key))
        assert set(ladder.resolve(term)) <= target, f"{word}'s term {term} is no parameter {rung.key} reads"


def test_the_rung_bytes_are_pinned_against_line_ending_conversion():
    """A checkout with core.autocrlf=true (the Windows box) would rewrite the bytes the hashes and robotparts pin."""
    attributes = (ROOT / ".gitattributes").read_text(encoding="utf-8").splitlines()
    assert "src/robot_models/actuators/rungs/** -text" in attributes


# --- type data only -----------------------------------------------------------------------------------------------


@pytest.mark.parametrize("type_name", sorted(TYPES))
def test_no_type_module_holds_a_value_a_maker_or_a_path(type_name):
    """Every .py in the type folder (decision 4: no maker 'also in docstrings'); numbers are judged in ladder.py."""
    folder = SRC / TYPES[type_name].folder
    assert (folder / "ladder.py").is_file(), f"no {TYPES[type_name].folder}/ladder.py yet"
    for path in sorted(folder.glob("*.py")):
        assert scan(path.read_text(encoding="utf-8"), numbers=path.name == "ladder.py") == [], path.name


def test_the_references_cite_no_maker_and_no_path():
    assert scan((SRC / "references.py").read_text(encoding="utf-8"), numbers=False) == []


def test_the_scan_goes_red_on_planted_text():
    assert scan('NOTE = "fitted on the GIM4305 at CAN 11"\n')
    assert scan('X = 0.12193\n')
    assert scan('NOTE = "Fc was 0.12193 on the rack"\n')
    assert scan('PATH = "/home/someone/robotparts/models"\n')
    assert scan('NOTE = "a moteus board"\n')
    assert scan('"""A docstring naming SteadyWin."""\n', numbers=False)
    assert scan('R = Rung(3, "stribeck", "Stribeck", "M2.coulomb_viscous")\nT = "exp(-(v/vs)^2)"\n') == []
    assert scan('REF = Reference("Canudas de Wit (1995)", doi="10.1109/9.376053", '
                'url="https://doi.org/10.1109/9.376053")\n') == []


def test_every_module_imports_in_a_fresh_interpreter():
    """Strengthens test_import_direction: a relative import that resolves nowhere passes an AST scan, not an import."""
    extras = {"homing/contact.py", "homing/cad_bundle.py"}
    have_extras = all(importlib.util.find_spec(name) for name in ("coal", "trimesh"))
    names = []
    for path in sorted(SRC.rglob("*.py")):
        rel = path.relative_to(SRC.parent)
        if "__pycache__" in rel.parts or (rel.relative_to("robot_models").as_posix() in extras and not have_extras):
            continue
        parts = rel.with_suffix("").parts
        names.append(".".join(parts[:-1] if parts[-1] == "__init__" else parts))
    code = ("import importlib, sys\nbad = []\n"
            f"for name in {names!r}:\n"
            "    try:\n        importlib.import_module(name)\n"
            "    except Exception as error:\n        bad.append(f'{name}: {type(error).__name__}: {error}')\n"
            "print('\\n'.join(bad))\nsys.exit(1 if bad else 0)\n")
    # WHY no bytecode: importing a rung's model.py would otherwise leave __pycache__/ inside a folder robotparts vendors
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False,
                            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    assert result.returncode == 0, result.stdout + result.stderr
