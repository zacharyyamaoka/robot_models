"""The one scenario every API proposal runs: "I have an accurate datasheet and the rig; I selected the models. What now?"

    datasheet()                 -> robot_models IntegratedServo   the catalogue's word for one servo, in its units
    rig_log(case, split)        -> dict of numpy arrays            a velocity-plateau friction sweep on the 1-DOF rig:
                                   v [rad/s], tau [N·m], plateau (an id per held speed and direction)

`case` is "A" or "B": two servos on the same rig, opaque on purpose (a scenario passes it through, never branches on
it). `split` is "fit" (plateaus 1, 3, 5, 7, 9: 0.03 to 2 rad/s), "test" (2, 4, 6, 8, 10: held out) or "all". Choose rungs on "test": a rung
that nests another always fits its own training data at least as well (amendment 1, Zach 2026-11-17: "if doing a more
complex model doesn't actually improve test time performance you are just overfitting").

The servo and its numbers are illustrative (no maker, no SKU); they live here, in a proposals folder, never in src/.
The rig logs are simulated from hidden truths (_truth.py) plus noise, so fits and choices can be graded.
"""

from __future__ import annotations

import math

import numpy as np

from robot_models.actuators.servo.integrated_servo import IntegratedServo
from robot_models.datasheet import Stated

PLATEAUS = (0.03, 0.06, 0.1, 0.18, 0.3, 0.5, 0.8, 1.3, 2.0, 3.0)   # rad/s, each run both directions
SAMPLES_PER_PLATEAU = 25
SPLITS = ("fit", "test", "all")


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


def rig_log(case: str = "A", split: str = "all") -> dict[str, np.ndarray]:
    """Velocity plateaus, both directions: the output speed held and the torque the motor had to add (tau_r)."""
    from _truth import CASES, NOISE_STD

    if split not in SPLITS:
        raise ValueError(f"split {split!r} is one of {SPLITS}")
    truth, seed = CASES[case]
    rng = np.random.default_rng(seed)
    index = np.repeat(np.arange(len(PLATEAUS)), SAMPLES_PER_PLATEAU)
    speeds = np.array(PLATEAUS)[index]
    v = np.concatenate([speeds, -speeds])
    plateau = np.concatenate([index + 1, -(index + 1)])          # +k forward, -k backward, k = 1..10
    level = truth["coulomb_friction"] + (truth["breakaway_torque"] - truth["coulomb_friction"]) * np.exp(
        -((np.abs(v) / truth["stribeck_velocity"]) ** 2))
    tau = level * np.sign(v) + truth["viscous_friction"] * v + rng.normal(0.0, NOISE_STD, v.shape)
    keep = np.ones(v.shape, bool) if split == "all" else ((np.abs(plateau) % 2 == 1) == (split == "fit"))
    return {"v": v[keep], "tau": tau[keep], "plateau": plateau[keep]}


def expected_m1_at(v: float) -> float:
    """What a datasheet-only model must predict at speed v, from the stated values in SI (for the check)."""
    ds = datasheet()
    return ds.coulomb_friction.si * math.copysign(1.0, v) + ds.viscous_friction.si * v
