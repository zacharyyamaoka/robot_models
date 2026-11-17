"""The one scenario every API proposal runs: "I have an accurate datasheet and the rig; I selected the models. What now?"

    datasheet() -> robot_models IntegratedServo   the catalogue's word for one servo, in the catalogue's units
    rig_log()   -> dict of numpy arrays            a velocity-plateau friction sweep from the 1-DOF rig: v [rad/s], tau [N·m]

The servo and its numbers are illustrative (no maker, no SKU); they live here, in a proposals folder, never in src/.
The rig log is simulated from a hidden Stribeck truth (_truth.py) plus noise, so a fit can be graded against it.
"""

from __future__ import annotations

import math

import numpy as np

from robot_models.actuators.servo.integrated_servo import IntegratedServo
from robot_models.datasheet import Stated

PLATEAUS = (0.03, 0.06, 0.1, 0.18, 0.3, 0.5, 0.8, 1.3, 2.0, 3.0)   # rad/s, each run both directions
SAMPLES_PER_PLATEAU = 25


def datasheet() -> IntegratedServo:
    """What the catalogue states: units as printed (rpm, kg·cm², mN·m·s/rad), friction lower than the rig will show."""
    return IntegratedServo(
        rated_torque=Stated(1.05, "N·m"),
        peak_torque=Stated(3.5, "N·m"),
        max_output_speed=Stated(230, "rpm"),
        gear_ratio=Stated(10, ":1"),
        output_inertia=Stated(0.42, "kg·cm²"),
        coulomb_friction=Stated(0.30, "N·m"),
        viscous_friction=Stated(20, "mN·m·s/rad"),
    )


def rig_log() -> dict[str, np.ndarray]:
    """Velocity plateaus, both directions: the output speed held and the torque the motor had to add (tau_r)."""
    from _truth import NOISE_STD, SEED, TRUTH

    rng = np.random.default_rng(SEED)
    speeds = np.repeat(np.array(PLATEAUS), SAMPLES_PER_PLATEAU)
    v = np.concatenate([speeds, -speeds])
    t = TRUTH
    level = t["coulomb_friction"] + (t["breakaway_torque"] - t["coulomb_friction"]) * np.exp(
        -((np.abs(v) / t["stribeck_velocity"]) ** 2))
    tau = level * np.sign(v) + t["viscous_friction"] * v + rng.normal(0.0, NOISE_STD, v.shape)
    return {"v": v, "tau": tau}


def expected_m1_at(v: float) -> float:
    """What a datasheet-only model must predict at speed v, from the stated values in SI (for the check)."""
    ds = datasheet()
    return ds.coulomb_friction.si * math.copysign(1.0, v) + ds.viscous_friction.si * v
