"""Amendment 4's data: a calibration outcome and the held-out runs played with nominal and with fitted values.

    outcome()                 -> CalibrationOutcome   M1.datasheet (nominal) vs M3.stribeck fitted on the sealed ladder
    held_out_runs(values)     -> tuple[Run, ...]      the held-out plateaus, the feed-forward computed from `values`

The sealed sim ladder is the earlier robot_models API round's friction sweep (docs/proposals/api-2026-11-17/
fixture.py: a 1-DOF rig's velocity plateaus simulated from a hidden Stribeck truth plus noise; fit plateaus 1, 3, 5, 7,
9, held out 2, 4, 6, 8, 10). The fit is robot_models' own (``IntegratedServo.m3.stribeck.fit``), unchanged.

A held-out run here is one plateau: the joint held at a speed while a feed-forward supplies the friction torque the
model predicts. ``target.effort`` is that feed-forward, ``measured.effort`` the torque the rig needed: their RMS is the
torque the feedback still has to supply (Zach's Oct 2 goal "a smaller feedback term"). Playing the runs with given
values is the runner's job, done here; a metric only ever sees the two recordings (G5: no metric reads a model).

``CalibrationOutcome`` is the proposed record. It must not live in sim2real (that would reverse the import arrow):
each variant README names its home, robot_metrics or robot_msgs.
"""

from __future__ import annotations

import functools
import importlib.util
import math
import pathlib
import sys
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from robot_msgs.std.header import Header
from robot_msgs.trajectory import JointTrajectory

from fixture import Run

SEALED = pathlib.Path(__file__).resolve().parent.parent / "api-2026-11-17"
PLATEAU_DT_S = 0.01   # one sample per 10 ms tick on a plateau (the sweep's samples carry no clock of their own)


@dataclass(frozen=True)
class Interval:
    value: float
    low: float | None = None
    high: float | None = None


@dataclass(frozen=True)
class CalibrationOutcome:
    """What one calibration produced: per parameter the nominal value, the recovered one (with an interval when the
    fit gives one), the truth when it is known (a sealed sim only), and the tolerance a recovery is held to."""

    part_type: str
    model: str
    nominal: Mapping[str, float]
    recovered: Mapping[str, Interval]
    truth: Mapping[str, float] = field(default_factory=dict)
    tolerance: Mapping[str, float] = field(default_factory=dict)   # relative
    fitted_on: str = ""
    held_out: str = ""

    def values(self, which: str) -> dict[str, float]:
        if which == "nominal":
            return dict(self.nominal)
        return {**self.nominal, **{name: interval.value for name, interval in self.recovered.items()}}


@functools.cache
def _sealed() -> Any:
    spec = importlib.util.spec_from_file_location("sealed_fixture", SEALED / "fixture.py")
    module = importlib.util.module_from_spec(spec)
    if str(SEALED) not in sys.path:
        # WHY appended, not inserted: its rig_log() imports _truth at call time, and its own fixture.py must never
        # shadow this round's (the first fixture on sys.path wins)
        sys.path.append(str(SEALED))
    spec.loader.exec_module(module)
    return module


@functools.cache
def _models() -> tuple[Any, Any]:
    from robot_models import IntegratedServo

    sealed = _sealed()
    nominal = IntegratedServo.m1.datasheet.from_datasheet(sealed.datasheet())
    fitted = IntegratedServo.m3.stribeck.fit(sealed.rig_log("A", "fit"), start=nominal)
    return nominal, fitted


def outcome() -> CalibrationOutcome:
    nominal, fitted = _models()
    _sealed()
    from _truth import CASES  # the sealed truth: a sim knows it, a rig never does
    names = ("coulomb_friction", "viscous_friction", "breakaway_torque", "stribeck_velocity")
    fitted_values = fitted.values()
    return CalibrationOutcome(
        part_type="integrated_servo", model=type(fitted).key,
        nominal={name: value for name, value in nominal.values().items()},
        recovered={name: Interval(float(fitted_values[name])) for name in names},
        truth={name: float(CASES["A"][0][name]) for name in names},
        tolerance={"coulomb_friction": 0.05, "breakaway_torque": 0.05, "stribeck_velocity": 0.10,
                   "viscous_friction": 0.10},
        fitted_on="sealed ladder case A, plateaus 1, 3, 5, 7, 9",
        held_out="plateaus 2, 4, 6, 8, 10, both directions")


def held_out_runs(which: str) -> tuple[Run, ...]:
    """The held-out plateaus played with the ``which`` ("nominal" | "fitted") values' feed-forward, one Run each."""

    nominal, fitted = _models()
    model = nominal if which == "nominal" else fitted
    log = _sealed().rig_log("A", "test")
    runs = []
    for plateau in np.unique(log["plateau"]):
        rows = np.flatnonzero(log["plateau"] == plateau)
        speed, torque = log["v"][rows], log["tau"][rows]
        t = PLATEAU_DT_S * np.arange(rows.size)
        header = Header(stamp=0.0, frame_id="output", meta={"source": f"sealed ladder plateau {int(plateau)}"})
        names = ("servo",)
        feedforward = np.asarray(model.predict(speed), float)
        zeros = np.zeros((rows.size, 1))
        target = JointTrajectory(t=t, joint_names=names, positions=zeros, velocities=speed[:, None],
                                 effort=feedforward[:, None], header=header)
        measured = JointTrajectory(t=t, joint_names=names, positions=zeros, velocities=speed[:, None],
                                   effort=torque[:, None], header=header)
        runs.append(Run(id=f"sealed:{which}:plateau{int(plateau):+d}", deployment="sealed friction ladder",
                        station="servo", part_type="integrated_servo", source="sim",
                        labels={"plateau": int(plateau), "values": which}, target=target, measured=measured))
    return tuple(runs)


def oracle_residual_rms(which: str) -> float:
    """For the check: RMS of (measured - feed-forward torque) over every held-out sample, straight from numpy."""

    runs = held_out_runs(which)
    residual = np.concatenate([run.measured.effort[:, 0] - run.target.effort[:, 0] for run in runs])
    return float(math.sqrt(np.mean(residual ** 2)))
