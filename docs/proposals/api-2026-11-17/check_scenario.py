"""Acceptance for every API proposal: frozen before the variants were built; builders do not edit it.

    /home/bam/robot_models/.venv/bin/python check_scenario.py [v1_name ...]

Each variant folder vN_<name>/ has scenario.py with run() -> dict holding:
    ladder                 the integrated_servo family's model keys, in today's order
    fallback               the fallback chain of M3.stribeck, itself first
    alias                  what "frictionloss_nm" resolves to
    m1_at_1                the datasheet model's resisting torque at v = 1 rad/s [N·m]
    fits                   {"M2.coulomb_viscous": {"params": {...}, "rms": float}, "M3.stribeck": {...}}
    chosen                 the key the comparison picks
"""

from __future__ import annotations

import importlib.util
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from _truth import TRUTH  # noqa: E402
from fixture import expected_m1_at  # noqa: E402

TODAY_KEYS = ("M0.ideal", "M1.datasheet", "M2.coulomb_viscous", "M3.stribeck", "M3.load", "M3.lugre",
              "M3.weakening_capture", "M4.ripple", "M4.drivetrain", "M5.dcmotor", "M5.envelope", "M5.wear",
              "M6.residual", "M7.learned")
TOLERANCE = {"coulomb_friction": 0.05, "breakaway_torque": 0.05, "stribeck_velocity": 0.10, "viscous_friction": 0.10}


def check(folder: pathlib.Path) -> list[str]:
    problems = []
    for path in folder.rglob("*.py"):
        if re.search(r"\b_truth\b|\bTRUTH\b|NOISE_STD", path.read_text()):
            problems.append(f"G3: {path.relative_to(HERE)} reads the hidden truth")
    sys.path.insert(0, str(folder))
    try:
        spec = importlib.util.spec_from_file_location(f"{folder.name}_scenario", folder / "scenario.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        out = module.run()
    finally:
        sys.path.remove(str(folder))
    if tuple(out["ladder"]) != TODAY_KEYS:
        problems.append(f"ladder {tuple(out['ladder'])} != today's keys")
    if tuple(out["fallback"]) != ("M3.stribeck", "M2.coulomb_viscous", "M1.datasheet", "M0.ideal"):
        problems.append(f"fallback {tuple(out['fallback'])}")
    if out["alias"] != "coulomb_friction":
        problems.append(f"alias frictionloss_nm -> {out['alias']!r}")
    if abs(out["m1_at_1"] - expected_m1_at(1.0)) > 1e-9:
        problems.append(f"m1_at_1 {out['m1_at_1']} != {expected_m1_at(1.0)} (the stated values in SI)")
    m2, m3 = out["fits"]["M2.coulomb_viscous"], out["fits"]["M3.stribeck"]
    if not m3["rms"] < m2["rms"]:
        problems.append(f"M3 rms {m3['rms']} is not below M2 rms {m2['rms']}")
    for name, tol in TOLERANCE.items():
        got = m3["params"][name]
        if abs(got - TRUTH[name]) > tol * abs(TRUTH[name]):
            problems.append(f"G3: M3 {name} = {got:.4g}, hidden truth {TRUTH[name]} (tolerance {tol:.0%})")
    if out["chosen"] != "M3.stribeck":
        problems.append(f"chosen {out['chosen']!r}, expected M3.stribeck")
    return problems


def main(names: list[str]) -> int:
    folders = [HERE / n for n in names] if names else sorted(p for p in HERE.glob("v[0-9]_*") if p.is_dir())
    failed = 0
    for folder in folders:
        try:
            problems = check(folder)
        except Exception as error:  # a crash is a FAIL with its reason, not a traceback wall
            problems = [f"crashed: {type(error).__name__}: {error}"]
        print(f"{'PASS' if not problems else 'FAIL'}  {folder.name}")
        for p in problems:
            print(f"      - {p}")
        failed += bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
