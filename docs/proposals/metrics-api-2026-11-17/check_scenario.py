"""Acceptance for every metrics-API proposal: frozen before the variants were built; builders do not edit it.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/check_scenario.py [vN_name ...]

The parity oracle is the round-8 scorecard (/archive/datasets/bam_rig/onedof_2026-11-15/scorecards/latest_real.json,
scored 2026-11-17T12:00:29Z by bam_rack_twin.pendulum_score/1, align "measured") and, for the rigid chain, the
five-bar speed ladder's own summary.json (every number robot_msgs.compare's). Only this file reads them; a variant
that opens either fails.

Each variant folder vN_<name>/ has scenario.py with run() -> dict holding:

    real_plays    {play id: {"tracking_rms_deg", "tracking_peak_deg", "stops", "overshoot_deg", "lag_ms"}}
    real_rungs    {"<station>@<rung>": {"tracking_rms_deg", "tracking_peak_deg", "stop_tool_mm", "stop_tool_max_mm",
                                        "settle_s", "limiter_share"}}
    sim_plays     {play id: {"tracking_rms_deg"}}                     the twin's answer to the same play (target vs sim)
    sim_rungs     {"<station>@<rung>": {"tracking_rms_deg", "tracking_peak_deg", "stop_tool_mm", "stop_tool_max_mm"}}
    highest_rung  {"<station>": the highest rung with every rung up to it passing tracking and the tip stop}
    gates         {"tool_stop_median": [passed, total], "tool_stop_worst": [passed, total], "tracking": [passed, total]}
    chain         {"<real|sim>:<rung name>": {"positions/j2", "positions/c1" (joint RMS, rad), "tool_position_rms_m",
                                              "tool_position_max_m"}}
    live          {"recording": play id, "values": {"tracking_rms_deg": [live, offline], "tracking_peak_deg": [...],
                                                     "stops": [...], ...}}   streamed sample by sample or in chunks
    metric_cards  [{"name", "description", "references": [...], "function", "unit", "target", "value_type"}, ...]
    declared      {"integrated_servo": [metric names], "rigid_body_chain": [metric names]}
    added         {"name": metric added in the session, "values": {play id: number}}
    aliases       [{"name", "base", "config", "unit", "signal"}, ...]   amendment 3: a part's aliases, base = the class
    inputs        {base class name: its declared input type}   amendment 4
    repeatability {"group": "<station>@<rung>", "k", "value" (rad, worst stop index), "repeats"}   over real repeats
    performance_gain {"alias", "nominal", "fitted", "gain"}   the alias on the sealed ladder's held-out runs (N·m)
    dedup         {"aliases": aliases evaluated on one recording, "computed": base computations actually run}

Definitions the numbers follow (the scorecard's, plus two rows it lacks):
- play tracking: RMS of measured - target over the moving rows; peak: the worst |error| of the episode; degrees.
- rung tracking: the median of its plays' RMS; peak: the max of its plays' peaks. Stops are pooled over the rung's
  plays: ``stop_tool_mm`` their median, ``stop_tool_max_mm`` the worst, ``settle_s`` the median settle (None or inf when
  that median never settled), ``limiter_share`` the rung's rows flagged 97/98/99/102 over all its rows.
- ``overshoot_deg``: the largest robot_msgs.compare.goal overshoot over the play's stops (0 with none).
- ``lag_ms``: robot_msgs.compare.estimate_time_offset(run.plan, run.measured).offset_s x 1e3 (its default search range).
- gates: ``tracking`` = RMS <= 0.5 deg and peak <= 1.5 deg and limiter share <= 2 % and no fault (agent);
  ``tool_stop_median`` = median stop <= 1 mm at the tip and median settle <= 0.3 s (Zach's 1 mm; the scorecard's rule);
  ``tool_stop_worst`` = every stop <= 1 mm (proposed: the worst stop against Zach's gate).

Tolerances: frozen with the contract on 2026-11-17 at 20:05 MST, amendment 1 (Zach's Metric design, Z1-Z2) folded in
before any variant was built.
"""

from __future__ import annotations

import importlib.util
import json
import math
import pathlib
import re
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import fixture  # noqa: E402
from robot_msgs.compare import estimate_time_offset  # noqa: E402

SCORECARD = pathlib.Path("/archive/datasets/bam_rig/onedof_2026-11-15/scorecards/round8.json")  # WHY: latest_real.json moves every round (round 9 on 11-17 21:43); round8.json is the byte-identical round-8 file (md5 3b98de359d22)
CHAIN_SUMMARY = fixture.CHAIN_LADDER / "summary.json"

# WHY these tolerances: each variant runs the same arithmetic on the same files as the scorer, so anything above float
# reassociation is a different ruler. The scorecard prints 0.01 deg and gates at 0.5 deg / 1 mm; the nearest wrong
# ruler (scoring each reply against its plan TICK, the "tick" align, instead of the plan at its own instant) moves
# play RMS by ~1e-3 deg, three orders above TRACKING_DEG.
TRACKING_DEG = 1e-6
STOP_MM = 1e-6
SETTLE_S = 1e-9
SHARE = 1e-12
OVERSHOOT_DEG = 1e-6
LAG_MS = 1e-6
CHAIN_REL = 1e-9
LIVE = 1e-9
LIMITERS = (97, 98, 99, 102)
FORBIDDEN_READS = re.compile(r"scorecards|latest_real|summary\.json|check_scenario|oracle_residual_rms|_truth")
FORBIDDEN_IMPORTS = re.compile(r"^\s*(import|from)\s+(pandas|scipy|torch|sim2real|bam_rack_twin|traj_compare)\b", re.M)


def _key(station: str, fraction: float) -> str:
    return f"{station}@{float(fraction):g}"


def _scorecard_oracle() -> dict:
    card = json.loads(SCORECARD.read_text(encoding="utf-8"))
    limits = card["thresholds"]
    plays, sim_plays, rungs, sim_rungs, gates, highest = {}, {}, {}, {}, {"tracking": 0, "tool_stop_median": 0,
                                                                            "tool_stop_worst": 0}, {}
    total = 0
    for station, block in card["stations"].items():
        for rung in block["rungs"]:
            key = _key(station, rung["fraction"])
            real, sim = rung["real"], rung["sim"]
            rungs[key] = {name: real.get(name) for name in ("tracking_rms_deg", "tracking_peak_deg", "stop_tool_mm",
                                                             "stop_tool_max_mm", "settle_s", "limiter_share")}
            sim_rungs[key] = {name: sim.get(name) for name in ("tracking_rms_deg", "tracking_peak_deg", "stop_tool_mm",
                                                                "stop_tool_max_mm")}
            for play in rung["plays"]:
                play_id = play["run"] + (f"#{play['board']}" if play.get("board") else "")
                plays[play_id] = {"tracking_rms_deg": play["tracking_rms_deg"],
                                  "tracking_peak_deg": play["tracking_peak_deg"], "stops": play["stops"]}
                sim_plays[play_id] = {"tracking_rms_deg": play.get("sim_tracking_rms_deg")}
            total += 1
            gates["tracking"] += bool(real["tracking_rms_deg"] <= limits["tracking_rms_deg"]
                                      and real["tracking_peak_deg"] <= limits["tracking_peak_deg"]
                                      and (real.get("limiter_share") or 0.0) <= limits["limiter_share_max"]
                                      and not real.get("faults"))
            gates["tool_stop_median"] += bool(real["stop_tool_mm"] <= limits["stop_tool_mm"]
                                              and real.get("settle_s") is not None
                                              and real["settle_s"] <= limits["settle_s"])
            gates["tool_stop_worst"] += bool(real["stop_tool_max_mm"] <= limits["stop_tool_mm"])
        highest[station] = card["goal"][f"max_trackable_fraction_{station}"]
    return {"plays": plays, "sim_plays": sim_plays, "rungs": rungs, "sim_rungs": sim_rungs, "highest": highest,
            "gates": {name: [count, total] for name, count in gates.items()}}


def _chain_oracle() -> dict:
    summary = json.loads(CHAIN_SUMMARY.read_text(encoding="utf-8"))
    out = {}
    for step in summary["steps"]:
        if step.get("kind") != "rung":
            continue
        for source, block in (("real", step.get("metrics")), ("sim", step.get("sim_metrics"))):
            if block:
                stats = block["stats"]
                out[f"{source}:{step['name']}"] = {"positions/j2": stats["joints/positions/j2"]["rms"],
                                                   "positions/c1": stats["joints/positions/c1"]["rms"],
                                                   "tool_position_rms_m": stats["tool/position"]["rms"],
                                                   "tool_position_max_m": stats["tool/position"]["max_abs"]}
    return out


def _overshoot_oracle(run: fixture.Run) -> float:
    """Independent of stock.py: dwells from runs of zero commanded speed on one target, the overshoot past each stop's
    target along its move, read on the dwell's rows (goal()'s settle phase)."""

    t = run.measured.t
    q = run.measured.positions[:, 0]
    target = run.target.positions[:, 0]
    still = np.abs(run.target.velocities[:, 0]) < 1e-9
    worst, last_still, row = 0.0, 0, 0
    while row < len(t):
        if not still[row]:
            row += 1
            continue
        end = row
        while end + 1 < len(t) and still[end + 1] and abs(target[end + 1] - target[row]) < 1e-9:
            end += 1
        if row > 0 and t[end] - t[row] >= 0.25:
            direction = np.sign(target[row] - target[last_still])
            worst = max(worst, float(np.max((q[row:end + 1] - target[row]) * direction)))
        last_still, row = end, end + 1
    return math.degrees(max(worst, 0.0))


def _repeatability_oracle(group: list, k: float) -> float | None:
    """ISO 9283 RP at each stop index over same-input repeats: mean(l) + k * std(l, ddof=1), l = |e - mean(e)| of the
    signed stop errors (mean of the dwell's last 50 ms); the worst stop index, radians. Independent of stock.py."""

    per_repeat = []
    for run in group:
        t, q = run.measured.t, run.measured.positions[:, 0]
        target, still = run.target.positions[:, 0], np.abs(run.target.velocities[:, 0]) < 1e-9
        errors, row = [], 0
        while row < len(t):
            if not still[row]:
                row += 1
                continue
            end = row
            while end + 1 < len(t) and still[end + 1] and abs(target[end + 1] - target[row]) < 1e-9:
                end += 1
            if row > 0 and t[end] - t[row] >= 0.25:
                tail = np.arange(row, end + 1)[t[row:end + 1] >= t[end] - 0.05]
                errors.append(float(np.mean(q[tail] - target[tail])))
            row = end + 1
        per_repeat.append(errors)
    if len({len(errors) for errors in per_repeat}) != 1 or len(per_repeat) < 2:
        return None
    matrix = np.asarray(per_repeat)
    distance = np.abs(matrix - matrix.mean(axis=0))
    return float(np.max(distance.mean(axis=0) + k * distance.std(axis=0, ddof=1)))


def _close(got, want, tol: float) -> bool:
    if want is None or (isinstance(want, float) and not math.isfinite(want)):
        return got is None or (isinstance(got, float) and not math.isfinite(got))
    return got is not None and math.isfinite(float(got)) and abs(float(got) - float(want)) <= tol


def _compare_rows(label: str, got: dict, want: dict, tolerances: dict, problems: list, worst: dict) -> None:
    missing = sorted(set(want) - set(got))
    if missing:
        problems.append(f"{label}: {len(missing)} of {len(want)} missing, e.g. {missing[0]}")
    for key, expected in want.items():
        if key not in got:
            continue
        for name, tol in tolerances.items():
            if name not in expected:
                continue
            value, oracle = got[key].get(name), expected[name]
            if oracle is not None and value is not None and math.isfinite(float(oracle)) and math.isfinite(float(value)):
                worst[f"{label}.{name}"] = max(worst.get(f"{label}.{name}", 0.0), abs(float(value) - float(oracle)))
            if not _close(value, oracle, tol):
                problems.append(f"G1 {label} {key} {name}: {value!r} vs scorecard {oracle!r} (tolerance {tol:g})")
                return


def check(folder: pathlib.Path, oracle: dict, chain: dict) -> tuple[list[str], list[str]]:
    problems, notes = [], []
    for path in folder.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        if FORBIDDEN_READS.search(text):
            problems.append(f"G1: {path.relative_to(HERE)} reads an oracle file")
        if FORBIDDEN_IMPORTS.search(text):
            problems.append(f"G5: {path.relative_to(HERE)} imports a heavy or upper-layer package at runtime")
    sys.path.insert(0, str(folder))
    try:
        spec = importlib.util.spec_from_file_location(f"{folder.name}_scenario", folder / "scenario.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        out = module.run()
    finally:
        sys.path.remove(str(folder))
        # harness fix 2026-11-17 22:05 MST (no ruler change): two variants may both call their package robot_metrics or
        # their module parts; forget every module loaded from this variant's folder so the next one imports its own
        for name, loaded in list(sys.modules.items()):
            if str(getattr(loaded, "__file__", "") or "").startswith(str(folder) + "/"):
                del sys.modules[name]
    worst: dict[str, float] = {}
    _compare_rows("real_plays", out["real_plays"], oracle["plays"],
                  {"tracking_rms_deg": TRACKING_DEG, "tracking_peak_deg": TRACKING_DEG, "stops": 0}, problems, worst)
    _compare_rows("real_rungs", out["real_rungs"], oracle["rungs"],
                  {"tracking_rms_deg": TRACKING_DEG, "tracking_peak_deg": TRACKING_DEG, "stop_tool_mm": STOP_MM,
                   "stop_tool_max_mm": STOP_MM, "settle_s": SETTLE_S, "limiter_share": SHARE}, problems, worst)
    _compare_rows("sim_plays", out["sim_plays"], oracle["sim_plays"], {"tracking_rms_deg": TRACKING_DEG}, problems, worst)
    _compare_rows("sim_rungs", out["sim_rungs"], oracle["sim_rungs"],
                  {"tracking_rms_deg": TRACKING_DEG, "tracking_peak_deg": TRACKING_DEG, "stop_tool_mm": STOP_MM,
                   "stop_tool_max_mm": STOP_MM}, problems, worst)
    if {k: float(v) for k, v in out["highest_rung"].items()} != {k: float(v) for k, v in oracle["highest"].items()}:
        problems.append(f"G1 highest_rung {out['highest_rung']} vs scorecard {oracle['highest']}")
    for name, expected in oracle["gates"].items():
        if list(out["gates"].get(name, [])) != expected:
            problems.append(f"G1 gates.{name} {out['gates'].get(name)} vs scorecard {expected}")
    for key, expected in chain.items():
        got = out["chain"].get(key)
        if got is None:
            problems.append(f"G1 chain {key} missing")
            continue
        for name, value in expected.items():
            deviation = abs(float(got[name]) - value) / max(abs(value), 1e-12)
            worst["chain.rel"] = max(worst.get("chain.rel", 0.0), deviation)
            if deviation > CHAIN_REL:
                problems.append(f"G1 chain {key} {name}: {got[name]!r} vs summary.json {value!r}")
    # the two rows the scorecard lacks, against independent oracles
    runs = {run.id: run for run in fixture.pendulum_runs("real")}
    for index, (play_id, run) in enumerate(runs.items()):
        row = out["real_plays"].get(play_id, {})
        oracle_overshoot = _overshoot_oracle(run)
        worst["overshoot_deg"] = max(worst.get("overshoot_deg", 0.0), abs(row.get("overshoot_deg", math.nan) - oracle_overshoot)
                                     if row.get("overshoot_deg") is not None else math.inf)
        if not _close(row.get("overshoot_deg"), oracle_overshoot, OVERSHOOT_DEG):
            problems.append(f"new row overshoot {play_id}: {row.get('overshoot_deg')!r} vs oracle {oracle_overshoot!r}")
            break
        if index % 25 == 0:
            lag = estimate_time_offset(run.plan, run.measured).offset_s * 1e3
            if not _close(row.get("lag_ms"), lag, LAG_MS):
                problems.append(f"new row lag {play_id}: {row.get('lag_ms')!r} vs oracle {lag!r}")
                break
    # Zach's design (amendment 1, 2026-11-17 20:03 MST)
    live = out.get("live") or {}
    values = live.get("values") or {}
    for name in ("tracking_rms_deg", "tracking_peak_deg", "stops"):
        pair = values.get(name)
        if not pair or not _close(pair[0], pair[1], LIVE):
            problems.append(f"Z2 live {name}: {pair!r} (live must equal offline)")
        elif name in out["real_plays"].get(live.get("recording"), {}) and not _close(
                pair[1], out["real_plays"][live["recording"]][name], 1e-12):
            problems.append(f"Z2 live {name}: its offline value is not the session's own for {live.get('recording')}")
    cards = out.get("metric_cards") or []
    fields = {"name", "description", "references", "function", "unit", "target", "value_type"}
    if not cards or any(not fields <= set(card) for card in cards):
        problems.append(f"Z1 metric_cards need the fields {sorted(fields)}")
    elif not {"number", "bool"} <= {card["value_type"] for card in cards}:
        problems.append("Z1 metric_cards: show one number-valued and one bool-valued Metric")
    elif any(not card["references"] or not card["description"] for card in cards):
        problems.append("Z1 a Metric needs a description and references")
    declared = out.get("declared") or {}
    if not declared.get("integrated_servo") or not declared.get("rigid_body_chain"):
        problems.append("declared: both integrated_servo and rigid_body_chain must declare their metrics")
    added = out.get("added") or {}
    if not added.get("name") or len(added.get("values") or {}) != len(runs) or not all(
            math.isfinite(float(v)) for v in added["values"].values()):
        problems.append("added: the new metric must have a finite value on every real play")
    # amendment 3 (2026-11-17 20:15 MST, before any variant was built): aliases over math-only bases, computed once
    aliases = out.get("aliases") or []
    if not aliases or any(not {"name", "base", "config", "unit", "signal"} <= set(alias) for alias in aliases):
        problems.append("G7 aliases need name, base, config, unit and signal")
    else:
        bases = [alias["base"] for alias in aliases]
        if len(set(bases)) == len(bases):
            problems.append("G6 no base metric serves two aliases: the meaning must come from the data, not the class")
        # ruler fix 2026-11-17 21:05 MST (after v1-v3 were built; it only narrows a false positive): compare whole words
        # of the CamelCase name, never substrings (the old test read the "s" of StopStatistic as the unit seconds)
        unit_words = {"deg", "degrees", "mm", "rad", "mrad", "ms", "millimetres", "radians"}
        for alias in aliases:
            words = {word.lower() for word in re.findall(r"[A-Z][a-z]*|[a-z]+", alias["base"])}
            if words & (unit_words | ({alias["unit"].lower()} if len(alias["unit"]) > 1 else set())):
                problems.append(f"G7 base {alias['base']} names a unit: units belong to the alias")
                break
    dedup = out.get("dedup") or {}
    if not dedup or not (0 < int(dedup.get("computed", 0)) < int(dedup.get("aliases", 0))):
        problems.append(f"G8 dedup {dedup!r}: aliases sharing (base, config) and data must compute once")
    # amendment 4 (2026-11-17 21:05 MST, the Sim2Real Routines session): input types per class, two worked examples
    inputs = out.get("inputs") or {}
    if len(set(inputs.values())) < 3 or not {"Repeatability", "PerformanceGain"} <= set(inputs):
        problems.append(f"A4 inputs {inputs!r}: each class declares its input; Repeatability and PerformanceGain "
                        "take other inputs than a run recording")
    repeat = out.get("repeatability") or {}
    station, rung = str(repeat.get("group", "@")).split("@")
    group = [run for run in runs.values() if run.station == station and f"{run.rung:g}" == rung and run.labels["kind"] == "repeat"]
    oracle_rp = _repeatability_oracle(group, float(repeat.get("k", 3.0))) if group else None
    if oracle_rp is None or not _close(repeat.get("value"), oracle_rp, 1e-12) or repeat.get("repeats") != len(group):
        problems.append(f"A4 repeatability {repeat!r} vs oracle {oracle_rp!r} over {len(group)} repeats")
    gain = out.get("performance_gain") or {}
    import calibration
    nominal, fitted = calibration.oracle_residual_rms("nominal"), calibration.oracle_residual_rms("fitted")
    if not (_close(gain.get("nominal"), nominal, 1e-9) and _close(gain.get("fitted"), fitted, 1e-9)
            and _close(gain.get("gain"), nominal - fitted, 1e-9)):
        problems.append(f"A4 performance_gain {gain!r} vs oracle nominal {nominal!r} fitted {fitted!r}")
    notes.append("max |deviation|: " + ", ".join(f"{k} {v:.2e}" for k, v in sorted(worst.items())))
    return problems, notes


def main(names: list[str]) -> int:
    oracle, chain = _scorecard_oracle(), _chain_oracle()
    folders = [HERE / n for n in names] if names else sorted(p for p in HERE.glob("v[0-9]_*") if p.is_dir())
    failed = 0
    for folder in folders:
        try:
            problems, notes = check(folder, oracle, chain)
        except Exception as error:  # a crash is a FAIL with its reason, not a traceback wall
            problems, notes = [f"crashed: {type(error).__name__}: {error}"], []
        print(f"{'PASS' if not problems else 'FAIL'}  {folder.name}")
        for note in notes:
            print(f"      · {note}")
        for problem in problems[:12]:
            print(f"      - {problem}")
        failed += bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
