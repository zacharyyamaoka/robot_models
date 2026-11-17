"""integrated_servo as a class tree: ServoModel is the type, and each rung is a subclass of the rung it extends.

A class body lists the fields its rung ADDS: no default means the rung needs it, `= None` that it reads it when stated.
A field optional below may be needed above (M1 states coulomb_friction, M2 needs it): the subclass redeclares it with
no default. Same-level alternatives are sibling (or cousin) classes; the fallback is the MRO. The root fills each
field's unit and definition from v1's vocabulary and checks every class against v1's rung record.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import numpy as np

from robot_models.actuators.servo.integrated_servo import IntegratedServo

from .model import Log, Model

Harmonics = tuple[tuple[float, float, float], ...]  # (rotor_harmonic, amplitude [N·m], phase [rad]) per entry


class ServoModel(Model, type="integrated_servo"):
    """A joint in one housing: a motor, its drive, a reducer or none, and an encoder. Torques at the OUTPUT."""


class Ideal(ServoModel, level=0, name="ideal"):
    output_inertia: float
    gear_ratio: float


class Datasheet(Ideal, level=1, name="datasheet", fields_from=IntegratedServo):
    """Every IntegratedServo field, each optional: a part with no friction figure still reaches M1."""


class CoulombViscous(Datasheet, level=2, name="coulomb_viscous"):
    coulomb_friction: float  # stated (optional) on M1, needed from here
    viscous_friction: float


class Stribeck(CoulombViscous, level=3, name="stribeck"):
    breakaway_torque: float
    stribeck_velocity: float
    stribeck_exponent: float | None = None  # 2 when absent (the reference's default)

    @classmethod
    def seeds(cls, log: Log, start: dict[str, Any]) -> Iterator[dict[str, Any]]:
        # WHY one start per stretch of the logged speeds: the curve is linear in its friction levels but not in the
        # speed it falls over, so a single start can settle in the wrong valley.
        speeds = np.abs(log["v"][log["v"] != 0])
        breakaway = start.get("breakaway_torque") or 1.5 * start.get("coulomb_friction", 0.0)
        for stribeck_velocity in np.geomspace(speeds.min(), speeds.max(), 6):
            yield {**start, "breakaway_torque": breakaway, "stribeck_velocity": float(stribeck_velocity)}


class Load(CoulombViscous, level=3, name="load"):
    friction_load_coefficient: float


class LuGre(Stribeck, level=3, name="lugre"):
    sigma0: float
    sigma1: float


class WeakeningCapture(Stribeck, level=3, name="weakening_capture"):
    friction_weakening: float
    friction_weakening_limit: float
    friction_capture: float
    friction_capture_speed: float
    friction_capture_dwell: float


# --- declared, not runnable here (the contract's stop line): fields, fallback and prose only -----------------------


class Ripple(CoulombViscous, level=4, name="ripple"):
    harmonics: Harmonics


class Drivetrain(WeakeningCapture, level=4, name="drivetrain"):
    harmonics: Harmonics
    backlash: float  # stated (optional) on M1, needed from here
    torsional_stiffness: float


class DcMotor(LuGre, level=5, name="dcmotor"):
    kt: float
    resistance_line_to_line: float
    harmonics: Harmonics | None = None
    thermal_resistance: float | None = None
    thermal_time_constant: float | None = None


class Envelope(Drivetrain, level=5, name="envelope"):
    kt: float
    resistance_line_to_line: float
    thermal_resistance: float
    thermal_time_constant: float


class Wear(Envelope, level=5, name="wear"):
    friction_growth: float
    backlash_wear: float


class Residual(Wear, level=6, name="residual"):
    """A grey-box residual on M5.wear: its weights are not named parameters."""


class Learned(ServoModel, level=7, name="learned"):
    """A black-box network: it extends no model and needs none of its parameters."""
