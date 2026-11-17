"""integrated_servo as a ModelFamily: the model classes ARE the ladder; today's `Ladder` is derived from them.

A class body declares what its rung ADDS (a Parameter with no `optional` is needed; `.when_stated()` is read when
stated); `requires=` makes an inherited optional one needed (M1 states coulomb_friction, M2 needs it). Same-level
alternatives are sibling classes; the fallback is the MRO. A rung with its own `torque` runs (numpy, held to today's
model.py at class creation); the rest are declared: their parameters, fallback and prose, no physics.

Shared quantities are declared once at module level and reused, so a name cannot mean two things in one type.
"""

from __future__ import annotations

import numpy as np

from robot_models import references as ref
from robot_models.actuators.servo import ladder as today
from robot_models.actuators.servo.integrated_servo import IntegratedServo as ServoDatasheet
from robot_models.ladder import Wear as WearsWith

from .core import Model, ModelFamily, Parameter, Weights

RUNGS = "actuators/rungs"

BREAKAWAY_TORQUE = Parameter("N·m", "Static friction at the output: the torque that starts it moving from rest, the "
                             "top of the Stribeck curve.")
STRIBECK_VELOCITY = Parameter("rad/s", "Output speed over which friction falls from breakaway_torque to "
                              "coulomb_friction.", positive=True)
STRIBECK_EXPONENT = Parameter("", "Shape exponent of the Stribeck curve, how sharply friction falls with speed; 2 when "
                              "absent.", optional=True, default=2.0)
HARMONICS = Parameter("N·m", "Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, amplitude, "
                      "phase): cogging and commutation ripple.", shape="list")
THERMAL_RESISTANCE = Parameter("K/W", "Winding-to-ambient thermal resistance: winding temperature rise per watt of "
                               "copper loss.")
THERMAL_TIME_CONSTANT = Parameter("s", "Time constant of the winding temperature's first-order rise.")


def _sign(v: np.ndarray) -> np.ndarray:
    return np.sign(v)  # WHY sign(0) = 0: today's model.py resists nothing at rest; MuJoCo's frictionloss sticks


class IntegratedServo(ModelFamily, type="integrated_servo", title=today.LADDER.title, datasheet=ServoDatasheet,
                      aliases=today.ALIASES, packed=today.PACKED, deployment=today.DEPLOYMENT):
    """A joint in one housing: a motor, its drive, a reducer or none, and an encoder. Torques at the OUTPUT."""

    class Ideal(Model, level=0, name="ideal"):
        """The ideal actuator: the stated inertia and ratio, no losses"""
        output_inertia = Parameter.from_datasheet(ServoDatasheet, "output_inertia")
        gear_ratio = Parameter.from_datasheet(ServoDatasheet, "gear_ratio")
        code, equation = f"{RUNGS}/ideal", "tau_r = 0; armature = output_inertia"
        references = (ref.SICILIANO_2009, ref.MUJOCO_ARMATURE)

        @staticmethod
        def torque(p, v):
            return np.zeros_like(v)

    class Datasheet(Ideal, level=1, name="datasheet", states=ServoDatasheet):
        """The datasheet: the stated values, nothing fitted"""
        code = f"{RUNGS}/datasheet"
        equation = ("tau_r = coulomb_friction·sign(v) + viscous_friction·v when stated; armature = output_inertia, or "
                    "gear_ratio²·(rotor_inertia + gearbox_input_inertia)")
        references = (ref.ARMSTRONG_1994, ref.MUJOCO_ARMATURE)

        @staticmethod
        def torque(p, v):
            return (p.coulomb_friction or 0.0) * _sign(v) + (p.viscous_friction or 0.0) * v

    class CoulombViscous(Datasheet, level=2, name="coulomb_viscous", requires=("coulomb_friction", "viscous_friction")):
        """Coulomb and viscous friction, fitted"""
        code, equation = f"{RUNGS}/coulomb_viscous", "tau_r = coulomb_friction·sign(v) + viscous_friction·v"
        references = (ref.ARMSTRONG_1994, ref.OLSSON_1998)

        @staticmethod
        def torque(p, v):
            return p.coulomb_friction * _sign(v) + p.viscous_friction * v

    class Stribeck(CoulombViscous, level=3, name="stribeck"):
        """Stribeck friction: breakaway falling to Coulomb with speed"""
        breakaway_torque = BREAKAWAY_TORQUE
        stribeck_velocity = STRIBECK_VELOCITY
        stribeck_exponent = STRIBECK_EXPONENT
        code = f"{RUNGS}/stribeck"
        equation = ("tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|v|/stribeck_velocity)"
                    "^stribeck_exponent))·sign(v) + viscous_friction·v")
        references = (ref.ARMSTRONG_1994, ref.OLSSON_1998)

        @staticmethod
        def torque(p, v):
            exponent = 2.0 if p.stribeck_exponent is None else p.stribeck_exponent
            level = p.coulomb_friction + (p.breakaway_torque - p.coulomb_friction) * np.exp(
                -((np.abs(v) / p.stribeck_velocity) ** exponent))
            return level * _sign(v) + p.viscous_friction * v

    # --- declared, not runnable here (the round's stop line): parameters, fallback and prose only -----------------

    class Load(CoulombViscous, level=3, name="load"):
        """Friction that grows with the transmitted load"""
        friction_load_coefficient = Parameter("", "Extra output friction per N·m of load torque the part transmits: "
                                              "Fc + μ·abs(τ_load).")
        code = f"{RUNGS}/load"
        equation = "tau_r = (coulomb_friction + friction_load_coefficient·|tau_load|)·sign(v) + viscous_friction·v"
        references = (ref.KIRCANSKI_1997, ref.OLSSON_1998)

    class LuGre(Stribeck, level=3, name="lugre"):
        """LuGre dynamic friction"""
        sigma0 = Parameter("N·m/rad", "LuGre bristle stiffness: output torque per radian of bristle deflection.")
        sigma1 = Parameter("N·m·s/rad", "LuGre bristle damping: output torque per rad/s of bristle deflection rate.")
        code = f"{RUNGS}/lugre"
        equation = ("dz/dt = v − sigma0·|v|·z/g(v), g(v) the Stribeck level of M3.stribeck; "
                    "tau_r = sigma0·z + sigma1·dz/dt + viscous_friction·v (LuGre's sigma2)")
        references = (ref.CANUDAS_1995,)

    class WeakeningCapture(Stribeck, level=3, name="weakening_capture"):
        """Stribeck with weakening and a capture level at rest"""
        friction_weakening = Parameter("N·m·s/rad", "How fast the static friction level falls with output speed, up "
                                       "to friction_weakening_limit.")
        friction_weakening_limit = Parameter("rad/s", "The output speed up to which the friction level keeps "
                                             "weakening.")
        friction_capture = Parameter("N·m", "The friction level that holds a slowed output until it has stayed slow "
                                     "for friction_capture_dwell.")
        friction_capture_speed = Parameter("rad/s", "Below this output speed a joint counts as slow for capture.")
        friction_capture_dwell = Parameter("s", "How long a joint must stay slow before the static level returns.")
        equation = ("level(v) = breakaway_torque − friction_weakening·min(|v|, friction_weakening_limit) − "
                    "(breakaway_torque − coulomb_friction)·(1 − exp(−(|v|/stribeck_velocity)²)); friction_capture "
                    "holds below friction_capture_speed until friction_capture_dwell has passed")
        references = (ref.ARMSTRONG_1994, ref.CANUDAS_1995)

    class Ripple(CoulombViscous, level=4, name="ripple"):
        """Rotor ripple on Coulomb and viscous friction"""
        harmonics = HARMONICS
        code = f"{RUNGS}/ripple"
        equation = ("tau_r = Σ_k amplitude_k·sin(rotor_harmonic_k·gear_ratio·θ + phase_k) + coulomb_friction·sign(v) "
                    "+ viscous_friction·v")
        references = (ref.JAHNS_1996,)

    class Drivetrain(WeakeningCapture, level=4, name="drivetrain", requires=("backlash",)):
        """The drivetrain: ripple, backlash and compliance on the full friction model"""
        harmonics = HARMONICS
        torsional_stiffness = Parameter("N·m/rad", "Drivetrain stiffness at the output with the rotor held: output "
                                        "torque per radian of twist.")
        composes = (("foc_controller", "M1.delay_loop"),)
        equation = ("output_inertia·q̈ = tau_drive − tau_r; tau_r = level(v)·sign(v) + viscous_friction·v + the "
                    "harmonics; the output across a backlash gap and torsional_stiffness; tau_drive from "
                    "foc_controller M1.delay_loop")
        references = (ref.JAHNS_1996, ref.NORDIN_2002, ref.TUTTLE_1996, ref.KRISHNAN_2010)

    class DcMotor(LuGre, level=5, name="dcmotor", requires=("kt", "resistance_line_to_line")):
        """MuJoCo's DC motor: electrics, heat, cogging and LuGre friction"""
        harmonics = HARMONICS.when_stated()
        thermal_resistance = THERMAL_RESISTANCE.when_stated()
        thermal_time_constant = THERMAL_TIME_CONSTANT.when_stated()
        code = f"{RUNGS}/dcmotor"
        equation = ("MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR "
                    "(back-EMF, cogging, LuGre friction)")
        references = (ref.KRAUSE_2013, ref.MUJOCO_DCMOTOR)

    class Envelope(Drivetrain, level=5, name="envelope", requires=("kt", "resistance_line_to_line")):
        """The drive's ceiling, the motor's electrics and heat on the drivetrain"""
        thermal_resistance = THERMAL_RESISTANCE
        thermal_time_constant = THERMAL_TIME_CONSTANT
        composes = (("foc_controller", "M2.ceiling_heat"),)
        equation = ("kt_out = kt·gear_ratio, R = resistance_line_to_line/2 per phase: motoring torque ≤ "
                    "kt_out·(voltage_available − |v|·kt_out/1.5)/R; any torque ≤ kt_out·I_max (the deployment's "
                    "current cap); winding rise ΔT over ambient: thermal_time_constant·dΔT/dt = "
                    "thermal_resistance·1.5·i²R − ΔT (the three phases' copper loss, i the phase-current amplitude)")
        references = (ref.KRAUSE_2013, ref.KRISHNAN_2010, ref.MUJOCO_DCMOTOR)

    class Wear(Envelope, level=5, name="wear"):
        """Wear: friction grows with hours, backlash opens with load"""
        friction_growth = Parameter("N·m/h", "How fast the friction level grows per deployment hour (seals, grease, "
                                    "bearings); on a Stribeck curve breakaway and Coulomb grow together.")
        backlash_wear = Parameter("rad/(N·m·rev)", "How fast backlash opens per torque-weighted output revolution "
                                  "(Archard).")
        wears = (WearsWith("breakaway_torque", today.DEPLOYMENT_HOURS, today.HOURS_UNIT, "linear", "friction_growth"),
                 WearsWith("coulomb_friction", today.DEPLOYMENT_HOURS, today.HOURS_UNIT, "linear", "friction_growth"),
                 WearsWith("backlash", today.TORQUE_REVOLUTIONS, today.TORQUE_REVOLUTIONS_UNIT, "archard",
                           "backlash_wear"))
        equation = ("backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev; breakaway_torque and coulomb_friction both "
                    "grow by friction_growth·h (the Stribeck drop stays)")
        references = (ref.ARCHARD_1953, ref.FLODIN_1997, ref.LU_MEEKER_1993)

    class Residual(Wear, level=6, name="residual"):
        """Grey-box residual on M5.wear"""
        kind = "residual"
        equation = "M5.wear + an MLP residual torque from 3 samples of position error, velocity and torque command"
        references = (ref.NGUYEN_TUONG_2010, ref.ZENG_2020, ref.KOVALEV_2026)

    # --- the black box: same outside interface (fit, predict, param_count), weights instead of named quantities -----

    class Learned(Model, level=7, name="learned"):
        """Black-box network"""
        forward = Weights(4, "Coefficients of [1, |v|, ln|v|, ln²|v|] for v > 0 [N·m per feature]")
        backward = Weights(4, "Coefficients of [1, |v|, ln|v|, ln²|v|] for v < 0 [N·m per feature]")
        kind = "learned"
        equation = ("per direction, tau_r = w·[1, |v|, ln|v|, ln²|v|] (a numpy stand-in for today's MLP over 3 "
                    "samples of position error and velocity)")
        references = (ref.HWANGBO_2019, ref.HWANGBO_2019_ARXIV, ref.ZHU_2025)

        @staticmethod
        def torque(p, v):
            speed = np.abs(v) + (v == 0)  # WHY: ln 0 is -inf; at rest both branches are masked to 0 below
            features = np.stack([np.ones_like(v), speed, np.log(speed), np.log(speed) ** 2], axis=-1)
            return np.where(v > 0, features @ np.asarray(p.forward), np.where(v < 0, features @ np.asarray(p.backward),
                                                                                0.0))
