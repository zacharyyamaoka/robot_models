"""Acceptance for every API proposal: frozen before the variants were built; builders do not edit it.

    /home/bam/robot_models/.venv/bin/python check_scenario.py [v1_name ...]

Each variant folder vN_<name>/ has scenario.py with run(case="A") -> dict holding:
    ladder                 the integrated_servo family's model keys, in today's order
    fallback               the fallback chain of M3.stribeck, itself first
    alias                  what "frictionloss_nm" resolves to
    m1_at_1                the datasheet model's resisting torque at v = 1 rad/s [N·m]
    fits                   {"M2.coulomb_viscous": {"params": {...}}, "M3.stribeck": {"params": {...}}}, fitted on
                           rig_log(case, "fit") only
    chosen                 the key the comparison picks, judged on rig_log(case, "test")

Amendment 1 (2026-11-17, before any variant was adapted; raised by the "Robot models sim-to-real summary" session):
v0 compared TRAINING rms, which M3.stribeck always wins because it nests M2.coulomb_viscous, so the choice could not
fail. Now the check (a) recomputes each fit's HELD-OUT rms itself from its params with the package's own rung physics,
(b) runs a negative control (case B, plain Coulomb + viscous) where the simpler rung must be chosen, and (c) reports
each rung's held-out rms over the noise floor, sim2real's selection measure (cards/schema.py select(): the lowest rung
whose held-out error ÷ floor ≤ 1.5).
"""

from __future__ import annotations

import importlib.util
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from _truth import CASES  # noqa: E402
from fixture import expected_m1_at, rig_log  # noqa: E402

import robot_models  # noqa: E402

TODAY_KEYS = ("M0.ideal", "M1.datasheet", "M2.coulomb_viscous", "M3.stribeck", "M3.load", "M3.lugre",
              "M3.weakening_capture", "M4.ripple", "M4.drivetrain", "M5.dcmotor", "M5.envelope", "M5.wear",
              "M6.residual", "M7.learned")
EXPECTED = {"A": "M3.stribeck", "B": "M2.coulomb_viscous"}
RECOVERED = {"A": ("M3.stribeck", {"coulomb_friction": 0.05, "breakaway_torque": 0.05, "stribeck_velocity": 0.10,
                                   "viscous_friction": 0.10}),
             "B": ("M2.coulomb_viscous", {"coulomb_friction": 0.05, "viscous_friction": 0.10})}
RUNG_CODE = pathlib.Path(robot_models.__file__).parent / "actuators" / "rungs"


def _physics(word: str):
    spec = importlib.util.spec_from_file_location(f"oracle_{word}", RUNG_CODE / word / "model.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.torque


ORACLE = {"M2.coulomb_viscous": _physics("coulomb_viscous"), "M3.stribeck": _physics("stribeck")}


def held_out_rms(key: str, params: dict, log: dict) -> float:
    """The oracle: the package's own rung physics on the held-out plateaus, never the variant's own number."""
    torque = ORACLE[key]
    terms = {name: float(value) for name, value in params.items()}
    predicted = np.array([torque(terms, {"v": float(v)})[0] for v in log["v"]])
    return float(np.sqrt(np.mean((predicted - log["tau"]) ** 2)))


def noise_floor(log: dict) -> float:
    """Pooled within-plateau standard deviation: what the rig repeats to, observable from the data itself."""
    residuals = np.concatenate([log["tau"][log["plateau"] == p] - log["tau"][log["plateau"] == p].mean()
                                for p in np.unique(log["plateau"])])
    return float(np.sqrt(np.sum(residuals ** 2) / (len(residuals) - len(np.unique(log["plateau"])))))


def check(folder: pathlib.Path) -> tuple[list[str], list[str]]:
    problems, notes = [], []
    for path in folder.rglob("*.py"):
        text = path.read_text()
        if re.search(r"\b_truth\b|\bTRUTH\b|\bCASES\b|NOISE_STD", text):
            problems.append(f"G3: {path.relative_to(HERE)} reads the hidden truth")
        if re.search(r"""case\s*(==|!=|in)\s*[\("'\[]""", text):
            problems.append(f"G3: {path.relative_to(HERE)} branches on the case name")
    sys.path.insert(0, str(folder))
    try:
        spec = importlib.util.spec_from_file_location(f"{folder.name}_scenario", folder / "scenario.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        outs = {case: module.run(case) for case in EXPECTED}
    finally:
        sys.path.remove(str(folder))
    out = outs["A"]
    if tuple(out["ladder"]) != TODAY_KEYS:
        problems.append(f"ladder {tuple(out['ladder'])} != today's keys")
    if tuple(out["fallback"]) != ("M3.stribeck", "M2.coulomb_viscous", "M1.datasheet", "M0.ideal"):
        problems.append(f"fallback {tuple(out['fallback'])}")
    if out["alias"] != "coulomb_friction":
        problems.append(f"alias frictionloss_nm -> {out['alias']!r}")
    if abs(out["m1_at_1"] - expected_m1_at(1.0)) > 1e-9:
        problems.append(f"m1_at_1 {out['m1_at_1']} != {expected_m1_at(1.0)} (the stated values in SI)")
    for case, result in outs.items():
        test = rig_log(case, "test")
        floor = noise_floor(test)
        ratios = {key: held_out_rms(key, result["fits"][key]["params"], test) / floor for key in ORACLE}
        notes.append(f"case {case}: chose {result['chosen']}; held-out rms ÷ floor ({floor:.4f} N·m): "
                     + ", ".join(f"{key} {ratio:.2f}" for key, ratio in ratios.items()))
        if result["chosen"] != EXPECTED[case]:
            problems.append(f"case {case}: chose {result['chosen']!r}, the held-out evidence supports "
                            f"{EXPECTED[case]!r}")
        key, tolerances = RECOVERED[case]
        truth = CASES[case][0]
        for name, tol in tolerances.items():
            got = float(result["fits"][key]["params"][name])
            if abs(got - truth[name]) > tol * abs(truth[name]):
                problems.append(f"G3: case {case} {key} {name} = {got:.4g}, hidden truth {truth[name]} "
                                f"(tolerance {tol:.0%})")
    return problems, notes


def main(names: list[str]) -> int:
    folders = [HERE / n for n in names] if names else sorted(p for p in HERE.glob("v[0-9]_*") if p.is_dir())
    failed = 0
    for folder in folders:
        try:
            problems, notes = check(folder)
        except Exception as error:  # a crash is a FAIL with its reason, not a traceback wall
            problems, notes = [f"crashed: {type(error).__name__}: {error}"], []
        print(f"{'PASS' if not problems else 'FAIL'}  {folder.name}")
        for n in notes:
            print(f"      · {n}")
        for p in problems:
            print(f"      - {p}")
        failed += bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
