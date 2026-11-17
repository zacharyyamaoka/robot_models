"""integrated_servo as a ModelFamily: the family class is the type, each rung a subclass of the rung it extends.

A class body lists the Parameters its rung ADDS: `Parameter()` is needed, `Parameter(optional=True)` read when stated.
A parameter optional below may be needed above (M1 states coulomb_friction, M2 needs it): the subclass redeclares it
required. A rung that defines `torque` runs (numpy, vectorized, held to the package's model.py by `parity`); one that
does not is declared: its parameters, fallback and prose, never its parent's physics run under its name.

The vocabulary, aliases, packed and deployment-only names are today's servo type data, imported, never retyped.
"""

from __future__ import annotations

import numpy as np

from robot_models import references as ref
from robot_models.actuators.servo import ladder as today
from robot_models.actuators.servo.integrated_servo import IntegratedServo as IntegratedServoDatasheet
from robot_models.ladder import Wear as WearLaw

from family_per_unit import Log, ModelFamily, Parameter

RUNGS = "actuators/rungs"


class IntegratedServo(ModelFamily, type_name="integrated_servo", vocabulary=today.VOCABULARY, aliases=today.ALIASES,
                      packed=today.PACKED, deployment=today.DEPLOYMENT, datasheet=IntegratedServoDatasheet):
    """A joint in one housing: a motor, its drive, a reducer or none, and an encoder."""


class Ideal(IntegratedServo.Model, level=0, name="ideal"):
    """The ideal actuator: the stated inertia and ratio, no losses"""

    output_inertia = Parameter[float]()
    gear_ratio = Parameter[float]()
    code, equation, references = f"{RUNGS}/ideal", "tau_r = 0; armature = output_inertia", (
        ref.SICILIANO_2009, ref.MUJOCO_ARMATURE)

    def torque(self, v: np.ndarray) -> np.ndarray:
        return np.zeros_like(v)


class Datasheet(Ideal, level=1, name="datasheet", fields_from=IntegratedServoDatasheet):
    """The datasheet: the stated values, nothing fitted"""

    code, references = f"{RUNGS}/datasheet", (ref.ARMSTRONG_1994, ref.MUJOCO_ARMATURE)
    equation = ("tau_r = coulomb_friction·sign(v) + viscous_friction·v when stated; armature = output_inertia, or "
                "gear_ratio²·(rotor_inertia + gearbox_input_inertia)")
    note = "States every IntegratedServo field and needs none beyond M0 (robotparts' M1: required false)."

    def torque(self, v: np.ndarray) -> np.ndarray:
        return (self.coulomb_friction or 0.0) * np.sign(v) + (self.viscous_friction or 0.0) * v


class CoulombViscous(Datasheet, level=2, name="coulomb_viscous"):
    """Coulomb and viscous friction, fitted"""

    coulomb_friction = Parameter[float]()  # stated (optional) on M1, needed from here
    viscous_friction = Parameter[float]()
    code, equation = f"{RUNGS}/coulomb_viscous", "tau_r = coulomb_friction·sign(v) + viscous_friction·v"
    references = (ref.ARMSTRONG_1994, ref.OLSSON_1998)

    def torque(self, v: np.ndarray) -> np.ndarray:
        return self.coulomb_friction * np.sign(v) + self.viscous_friction * v


class Stribeck(CoulombViscous, level=3, name="stribeck"):
    """Stribeck friction: breakaway falling to Coulomb with speed"""

    breakaway_torque = Parameter[float]()
    stribeck_velocity = Parameter[float]()
    stribeck_exponent = Parameter[float](optional=True)  # 2 when absent (the reference's default)
    code, references = f"{RUNGS}/stribeck", (ref.ARMSTRONG_1994, ref.OLSSON_1998)
    equation = ("tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(|v|/stribeck_velocity)"
                "^stribeck_exponent))·sign(v) + viscous_friction·v")

    def torque(self, v: np.ndarray) -> np.ndarray:
        drop = (self.breakaway_torque - self.coulomb_friction) * np.exp(
            -(np.abs(v) / self.stribeck_velocity) ** (self.stribeck_exponent or 2.0))
        return (self.coulomb_friction + drop) * np.sign(v) + self.viscous_friction * v


class Load(CoulombViscous, level=3, name="load"):
    """Friction that grows with the transmitted load"""

    friction_load_coefficient = Parameter[float]()
    inputs = ("v", "tau_load")
    code, references = f"{RUNGS}/load", (ref.KIRCANSKI_1997, ref.OLSSON_1998)
    equation = "tau_r = (coulomb_friction + friction_load_coefficient·|tau_load|)·sign(v) + viscous_friction·v"

    def torque(self, v: np.ndarray, tau_load: np.ndarray = np.zeros(1)) -> np.ndarray:
        level = self.coulomb_friction + self.friction_load_coefficient * np.abs(tau_load)
        return level * np.sign(v) + self.viscous_friction * v


class LuGre(Stribeck, level=3, name="lugre"):
    """LuGre dynamic friction"""

    sigma0 = Parameter[float]()
    sigma1 = Parameter[float]()
    code, references = f"{RUNGS}/lugre", (ref.CANUDAS_1995,)
    # WHY viscous_friction stands for LuGre's sigma2: one quantity, one canonical name; "sigma2" is its alias.
    equation = ("dz/dt = v − sigma0·|v|·z/g(v), g(v) the Stribeck level of M3.stribeck; "
                "tau_r = sigma0·z + sigma1·dz/dt + viscous_friction·v (LuGre's sigma2)")


class WeakeningCapture(Stribeck, level=3, name="weakening_capture"):
    """Stribeck with weakening and a capture level at rest"""

    friction_weakening = Parameter[float]()
    friction_weakening_limit = Parameter[float]()
    friction_capture = Parameter[float]()
    friction_capture_speed = Parameter[float]()
    friction_capture_dwell = Parameter[float]()
    references = (ref.ARMSTRONG_1994, ref.CANUDAS_1995)
    equation = ("level(v) = breakaway_torque − friction_weakening·min(|v|, friction_weakening_limit) − "
                "(breakaway_torque − coulomb_friction)·(1 − exp(−(|v|/stribeck_velocity)²)); "
                "friction_capture holds below friction_capture_speed until friction_capture_dwell has passed")
    note = "sim2real's servo M3, the rack twin's friction: its structure only, the twin is never published."


class Ripple(CoulombViscous, level=4, name="ripple"):
    """Rotor ripple on Coulomb and viscous friction"""

    harmonics = Parameter[tuple]()
    code, references = f"{RUNGS}/ripple", (ref.JAHNS_1996,)
    equation = ("tau_r = Σ_k amplitude_k·sin(rotor_harmonic_k·gear_ratio·θ + phase_k) "
                "+ coulomb_friction·sign(v) + viscous_friction·v")


class Drivetrain(WeakeningCapture, level=4, name="drivetrain"):
    """The drivetrain: ripple, backlash and compliance on the full friction model"""

    harmonics = Parameter[tuple]()
    backlash = Parameter[float]()  # stated (optional) on M1, needed from here
    torsional_stiffness = Parameter[float]()
    composes = (("foc_controller", "M1.delay_loop"),)
    references = (ref.JAHNS_1996, ref.NORDIN_2002, ref.TUTTLE_1996, ref.KRISHNAN_2010)
    equation = ("output_inertia·q̈ = tau_drive − tau_r; tau_r = level(v)·sign(v) + viscous_friction·v + "
                "the harmonics; the output across a backlash gap and torsional_stiffness; tau_drive from "
                "foc_controller M1.delay_loop")


class DcMotor(LuGre, level=5, name="dcmotor"):
    """MuJoCo's DC motor: electrics, heat, cogging and LuGre friction"""

    kt = Parameter[float]()
    resistance_line_to_line = Parameter[float]()
    harmonics = Parameter[tuple](optional=True)
    thermal_resistance = Parameter[float](optional=True)
    thermal_time_constant = Parameter[float](optional=True)
    code, references = f"{RUNGS}/dcmotor", (ref.KRAUSE_2013, ref.MUJOCO_DCMOTOR)
    equation = ("MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR "
                "(back-EMF, cogging, LuGre friction)")


class Envelope(Drivetrain, level=5, name="envelope"):
    """The drive's ceiling, the motor's electrics and heat on the drivetrain"""

    kt = Parameter[float]()
    resistance_line_to_line = Parameter[float]()
    thermal_resistance = Parameter[float]()
    thermal_time_constant = Parameter[float]()
    composes = (("foc_controller", "M2.ceiling_heat"),)
    references = (ref.KRAUSE_2013, ref.KRISHNAN_2010, ref.MUJOCO_DCMOTOR)
    equation = ("motoring torque ≤ kt_out·(voltage_available − |v|·kt_out/1.5)/R; any torque ≤ kt_out·I_max; "
                "thermal_time_constant·dΔT/dt = thermal_resistance·1.5·i²R − ΔT")


class Wear(Envelope, level=5, name="wear"):
    """Wear: friction grows with hours, backlash opens with load"""

    friction_growth = Parameter[float]()
    backlash_wear = Parameter[float]()
    wears = (WearLaw("breakaway_torque", today.DEPLOYMENT_HOURS, today.HOURS_UNIT, "linear", "friction_growth"),
             WearLaw("coulomb_friction", today.DEPLOYMENT_HOURS, today.HOURS_UNIT, "linear", "friction_growth"),
             WearLaw("backlash", today.TORQUE_REVOLUTIONS, today.TORQUE_REVOLUTIONS_UNIT, "archard", "backlash_wear"))
    references = (ref.ARCHARD_1953, ref.FLODIN_1997, ref.LU_MEEKER_1993)
    equation = ("backlash(u) = backlash + backlash_wear·Σ|τ|·Δrev; breakaway_torque and coulomb_friction both "
                "grow by friction_growth·h (the Stribeck drop stays)")


class Residual(Wear, level=6, name="residual", kind="residual"):
    """Grey-box residual on M5.wear"""

    references = (ref.NGUYEN_TUONG_2010, ref.ZENG_2020, ref.KOVALEV_2026)
    equation = "M5.wear + an MLP residual torque from 3 samples of position error, velocity and torque command"


class Learned(IntegratedServo.Model, level=7, name="learned", kind="learned"):
    """Black-box network"""

    # WHY a numpy RBF stand-in for today's MLP: it shows the outside interface (fit, predict, param_count, and a seat
    # in select_best) is the same for a black box; per direction because friction is not odd in v on a real servo.
    CENTERS = 5
    param_count = 2 * (2 + CENTERS)        # per direction: a bias, a slope in |v|, one weight per centre
    references = (ref.HWANGBO_2019, ref.HWANGBO_2019_ARXIV, ref.ZHU_2025)
    equation = "a per-direction ridge regression on [1, |v|, Gaussian bumps in log|v|] (stands in for the MLP)"

    def _features(self, v: np.ndarray) -> np.ndarray:
        speed = np.log(np.maximum(np.abs(v), 1e-6))[..., None]
        width = np.diff(self.centers[:2])[0]
        one = np.concatenate([np.ones_like(speed), np.abs(v)[..., None],
                              np.exp(-((speed - self.centers) / width) ** 2)], axis=-1)
        return np.concatenate([one * (v > 0)[..., None], -one * (v < 0)[..., None]], axis=-1)

    def torque(self, v: np.ndarray) -> np.ndarray:
        return self._features(v) @ self.weights

    def _solve(self, log: Log) -> None:
        speeds = np.abs(log["v"][log["v"] != 0])
        self.centers = np.linspace(np.log(speeds.min()), np.log(speeds.max()), self.CENTERS)
        phi = self._features(log["v"])
        self.weights = np.linalg.solve(phi.T @ phi + 1e-6 * np.eye(phi.shape[1]), phi.T @ log["tau"])
        self.fitted_count = self.weights.size

    def missing(self) -> list[str]:
        return [] if hasattr(self, "weights") else ["weights"]

    @property
    def params(self) -> dict:
        return {"weights": self.weights.round(5).tolist()} if hasattr(self, "weights") else {}
