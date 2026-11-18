"""v4 · Folded values: the shared scenario as one REPL session on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v4_folded_values/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs the viz extra). ``run()`` is what the
frozen check calls: the same API, no transcript.
"""

from __future__ import annotations

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

import calibration  # noqa: E402
import fixture  # noqa: E402
from _session import Session  # noqa: E402
from parts import DEGREES, INTEGRATED_SERVO, RIGID_BODY_CHAIN, RUNGS, VERDICTS, pendulum_arm  # noqa: E402
from robot_metrics import (Alias, CodeShare, Collection, Dwell, ErrorStat, HeldOutPair, Objective,  # noqa: E402
                           PerformanceGain, Repeat, Repeatability, Run, Stream, TimeOffset, rdx, summarize, wilson)

MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
TIPS = {station: facts["tip_length_m"] for station, facts in fixture.stations().items()}
ARMS = {station: pendulum_arm(tip) for station, tip in TIPS.items()}
COLLECTIONS = {station: arm.collection() for station, arm in ARMS.items()}
FF_RESIDUAL = Alias("feed-forward residual RMS", ErrorStat(statistic="rms", field="effort"), ("target", "state"), "N·m")
VELOCITY_ERROR = Alias("velocity error RMS", ErrorStat(statistic="rms", rows="moving", field="velocities"),
                       ("target", "state"), "deg/s", DEGREES, "median")


def runs(source: str) -> list[Run]:
    return [Run.from_fixture(run) for run in fixture.pendulum_runs(source)]


def score(source: str) -> tuple[dict, dict]:
    """Per play its folded state's values; per stratum (station x rung) the merged states and its runs' values."""

    all_runs = runs(source)
    plays = {run.id: {**COLLECTIONS[run.labels["station"]].values(COLLECTIONS[run.labels["station"]].fold(run)),
                      "faults": int(run.outcome != "complete")} for run in all_runs}
    strata = {}
    for station, collection in COLLECTIONS.items():
        strata.update(summarize(collection, [r for r in all_runs if r.labels["station"] == station], ("station", "rung")))
    return plays, strata


def judged(strata: dict) -> dict:
    cells = {}
    for key, stratum in strata.items():
        collection = COLLECTIONS[key[0]]
        cells[key] = {name: rdx.worst(objective.judge(collection, stratum).verdict for objective in objectives)
                      for name, objectives in VERDICTS.items()}
    highest = {}
    for station in sorted(COLLECTIONS):
        reached = 0.0
        for rung in RUNGS:
            cell = cells.get((station, rung), {})
            if not (cell.get("tracks") == rdx.PASS and cell.get("tip stop (median)") == rdx.PASS):
                break
            reached = rung
        highest[station] = reached
    return {"cells": cells, "highest": highest}


def chain_values() -> dict:
    collection = RIGID_BODY_CHAIN.collection()
    return {run.id: collection.values(collection.fold(Run.from_fixture(run)))
            for source in ("real", "sim") for run in fixture.chain_runs(source)}


def live_vs_offline(run: Run, rows: int = 7) -> tuple[dict, dict]:
    collection = COLLECTIONS[run.labels["station"]]
    offline = collection.values(collection.fold(run))
    state = collection.empty()
    for chunk in run.chunks(rows):
        state = collection.update(state, chunk)
    return offline, collection.values(collection.close(state))


def repeatability(station: str = "can11", rung: float = 0.9) -> tuple[Repeatability, tuple, int]:
    metric = Repeatability(k=3.0)
    state = metric.empty()
    repeats = [r for r in fixture.pendulum_runs("real") if r.station == station and r.rung == rung
               and r.labels["kind"] == "repeat"]
    for run in repeats:
        state = metric.update(state, Repeat((station, rung), run.target, run.measured))
    return metric, state, len(repeats)


def performance_gain() -> tuple[PerformanceGain, tuple]:
    gain = PerformanceGain(metric=FF_RESIDUAL.metric, signal=FF_RESIDUAL.signal)
    state = gain.update(gain.empty(), HeldOutPair(tuple(Run.from_fixture(r) for r in calibration.held_out_runs("nominal")),
                                                  tuple(Run.from_fixture(r) for r in calibration.held_out_runs("fitted"))))
    return gain, state


def check_dict(real: tuple, sim: tuple, chain: dict, live: dict) -> dict:
    def play_rows(plays):
        return {k: {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                    "stops": int(v["stops"]), "overshoot_deg": v["overshoot"], "lag_ms": v["lag"]}
                for k, v in plays.items()}

    def rung_rows(strata):
        return {f"{s}@{r:g}": {"tracking_rms_deg": v["values"]["tracking RMS"],
                               "tracking_peak_deg": v["values"]["tracking peak"],
                               "stop_tool_mm": v["values"]["tip stop error"],
                               "stop_tool_max_mm": v["values"]["tip stop worst"], "settle_s": v["values"]["tip settle"],
                               "limiter_share": v["values"]["limiter share"] / 100.0}
                for (s, r), v in strata.items()}

    verdicts = judged(real[1])
    gates = {key: [sum(c[name] == rdx.PASS for c in verdicts["cells"].values()), len(verdicts["cells"])]
             for key, name in (("tracking", "tracks"), ("tool_stop_median", "tip stop (median)"),
                               ("tool_stop_worst", "tip stop (every stop)"))}
    added = Collection((VELOCITY_ERROR,))
    rp, rp_state, repeats = repeatability()
    gain, gain_state = performance_gain()
    arm = ARMS["can11"]
    return {
        "real_plays": play_rows(real[0]), "real_rungs": rung_rows(real[1]),
        "sim_plays": {k: {"tracking_rms_deg": v["tracking_rms_deg"]} for k, v in play_rows(sim[0]).items()},
        "sim_rungs": rung_rows(sim[1]), "highest_rung": verdicts["highest"], "gates": gates,
        "chain": {k: {"positions/j2": v["j2 tracking RMS"] / DEGREES, "positions/c1": v["c1 tracking RMS"] / DEGREES,
                      "tool_position_rms_m": v["tool tracking RMS"] / 1e3, "tool_position_max_m": v["tool worst"] / 1e3}
                  for k, v in chain.items()},
        "live": live,
        "metric_cards": [a.card() for a in arm.metrics if a.name in ("tip stop error", "all stops settled")],
        "declared": {"integrated_servo": [a.name for a in INTEGRATED_SERVO.metrics],
                     "rigid_body_chain": [a.name for a in RIGID_BODY_CHAIN.metrics]},
        "added": {"name": VELOCITY_ERROR.name,
                  "values": {r.id: added.values(added.fold(r))[VELOCITY_ERROR.name] for r in runs("real")}},
        "aliases": [{**a.card(), "signal": "/".join(a.signal)} for a in arm.metrics],
        "dedup": {"aliases": len(arm.metrics), "computed": COLLECTIONS["can11"].computations},
        "inputs": {cls.__name__: cls.Input for cls in (ErrorStat, Dwell, TimeOffset, CodeShare, Repeatability,
                                                       PerformanceGain)},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": rp.compute(rp_state), "repeats": repeats},
        "performance_gain": {"alias": FF_RESIDUAL.name, "nominal": gain_state[0], "fitted": gain_state[1],
                             "gain": gain.compute(gain_state)},
    }


def run() -> dict:
    first = runs("real")[0]
    offline, live = live_vs_offline(first)
    pairs = {"tracking_rms_deg": "tracking RMS", "tracking_peak_deg": "tracking peak", "stops": "stops",
             "stop_tool_max_mm": "tip stop worst", "overshoot_deg": "overshoot"}
    live_block = {"recording": first.id, "values": {key: [live[name], offline[name]] for key, name in pairs.items()}}
    return check_dict(score("real"), score("sim"), chain_values(), live_block)


def plays_of(station: str, rung: float) -> list[Run]:
    """One rung cell's real plays, as the roll-up groups them."""

    return [run for run in runs("real") if run.labels["station"] == station and run.labels["rung"] == rung]


def rung_table(strata: dict, verdicts: dict, stations: tuple[str, ...]) -> str:
    """The rung cells of ``stations`` as a table, with each cell's three verdicts (display only)."""

    mark = {"pass": "✓", "fail": "✗", "not_run": "?"}
    lines = ["station rung   RMS °  peak °  stop mm  worst mm   tracks · median · every stop"]
    for (station, rung), stratum in sorted(strata.items()):
        if station not in stations:
            continue
        values, cell = stratum["values"], verdicts["cells"][(station, rung)]
        lines.append(f"{station:<7}{rung:5.1f}{values['tracking RMS']:8.2f}{values['tracking peak']:8.2f}"
                     f"{values['tip stop error']:9.2f}{values['tip stop worst']:10.2f}   "
                     + "        ".join(mark[cell[name]] for name in VERDICTS))
    return "\n".join(lines)


# WHY the same seven headings in every variant: the five glances are compared side by side, row by row.
GLANCE = ("API at a glance", "The design's shape before any detail: a frozen metric whose state is a value, an alias, "
          "a part type's .metrics, live as a fold, offline as the same fold once, an OpenSLO objective judged into "
          "RobotDx's Result, and a rung as its plays' states merged.", '''
# a metric: a frozen config (its identity); its state is a plain value
stop_worst = Dwell(of="error", statistic="max")
stop_worst.empty()

# an alias: a name, the signal pair it reads, a display unit
tip_stop_worst = Alias("tip stop worst", stop_worst, ("target", "state"), "mm", 339.0)
tip_stop_worst

# a part type's metrics, beside its models
INTEGRATED_SERVO.models
arm = pendulum_arm(tip_length_m=0.339)
[alias.name for alias in arm.metrics]

# live: a fold, each chunk returns a new state value
run = Run.from_fixture(fixture.pendulum_runs("real")[32])
collection = arm.collection()
state = collection.empty()
for chunk in run.chunks(7):
    state = collection.update(state, chunk)
live = collection.values(collection.close(state))
live["tip stop worst"]

# offline: the same fold, once
offline = collection.values(collection.fold(run))
offline["tip stop worst"]

# a gate: an OpenSLO occurrence objective on every stop, judged into RobotDx's Result
owner = "Zach 2026-11-11"
gate = Objective("tip stop worst", "le", 1.0, share=1.0, on="items", owner=owner)
this_play = summarize(collection, [run], by=())[()]
result = gate.judge(collection, this_play)
result.verdict, result.message

# across runs: a rung is its plays' states merged, every stop kept
rungs = summarize(collection, plays_of("can11", rung=0.9), by=("station", "rung"))
rung = rungs[("can11", 0.9)]
rung["values"]["tip stop worst"]
result = gate.judge(collection, rung)
result.verdict, result.message
''')

CELLS = [
    ("a part's metrics", "Each metric is a frozen config (hashable: its identity); the part lists aliases; equal "
     "identities share one state, and every dwell metric on a signal shares one stop pass.", '''
print(INTEGRATED_SERVO.table())
INTEGRATED_SERVO.models

arm = pendulum_arm(tip_length_m=TIPS["can11"])    # CAN 11: the 339 mm heavy pendulum
print(arm.table())

print(RIGID_BODY_CHAIN.table())

aliases = {alias.name: alias for alias in arm.metrics}
rms_while_moving = ErrorStat(statistic="rms", rows="moving")
rms_while_moving == aliases["tip tracking RMS"].metric
hash(rms_while_moving) == hash(aliases["tracking RMS"].metric)

aliases["tip stop error"].card()
'''),
    ("one recording, offline and live", "Live is a fold over 7-row chunks of new state VALUES; offline the same fold "
     "once. Stream is the torchmetrics face over it.", '''
run = Run.from_fixture(fixture.pendulum_runs("real")[32])    # CAN 11, rung 0.9, a repeat
offline, live = live_vs_offline(run)

names = ["tracking RMS", "tracking peak", "tip stop error", "tip stop worst",
         "tip settle", "overshoot", "lag", "stops"]
print(f"{'':<15} {'offline':>10} {'live':>10}")
for name in names:
    difference = abs(offline[name] - live[name])
    print(f"{name:<15} {offline[name]:10.6f} {live[name]:10.6f}  diff {difference:.1e}")

stream = Stream(ErrorStat(statistic="rms", rows="moving"))    # update / compute / reset
for chunk in run.chunks(7):
    stream.update(chunk.streams["target"], chunk.streams["state"])
stream.compute() * DEGREES
stream.state          # a plain value: (sum of squares, rows, max |error|, sum |error|)

card = aliases["tip stop error"].card()
stored = {"alias": "tip stop error", "value": offline["tip stop error"], "unit": "mm",
          "base": card["base"], "config": card["config"], "version": card["version"],
          "provenance": run.provenance, "rrd": run.links["rrd"]}
stored
'''),
    ("runs to a deployment, gates applied", "Strata are station x rung; each verdict is OpenSLO objectives judged by "
     "RobotDx's own Result. The same alias carries the median gate and the every-stop gate.", '''
real_plays, real_strata = score("real")
len(real_plays), len(real_strata)

verdicts = judged(real_strata)
print(rung_table(real_strata, verdicts, stations=("can11", "can12")))

cells = verdicts["cells"].values()
{name: sum(cell[name] == "pass" for cell in cells) for name in VERDICTS}
verdicts["highest"]

VERDICTS["tip stop (every stop)"]

can11_collection = COLLECTIONS["can11"]
can11_runs = [run for run in runs("real") if run.labels["station"] == "can11"]
station = summarize(can11_collection, can11_runs, by=("station",))[("can11",)]
gate_99 = Objective("tip stop error", "le", 1.0, share=0.99, on="items", owner="proposed")
result = gate_99.judge(can11_collection, station)
result.verdict, result.title
result.message

settled = 0
for state in station["states"]:
    settled += can11_collection.value(state, "all stops settled")
settled, len(station["states"])
wilson(settled, len(station["states"]))       # 95 % interval on the share of settled runs
'''),
    ("the twin's output (sim vs target)", "The same collections on the twin's answers.", '''
sim_plays, sim_strata = score("sim")

print("CAN 11 rung   RMS real °   RMS sim °   stop real mm   stop sim mm")
for rung in RUNGS:
    real_values = real_strata[("can11", rung)]["values"]
    sim_values = sim_strata[("can11", rung)]["values"]
    rms = (real_values["tracking RMS"], sim_values["tracking RMS"])
    stop = (real_values["tip stop error"], sim_values["tip stop error"])
    print(f"{rung:11.1f} {rms[0]:12.3f} {rms[1]:11.3f} {stop[0]:14.3f} {stop[1]:13.3f}")

sim_cells = judged(sim_strata)["cells"].values()
sum(cell["tip stop (median)"] == "pass" for cell in sim_cells)
'''),
    ("the chain, a view, a new metric", "The five-bar through the same ErrorStat; a view of every CAN 11 stop to "
     "matplotlib, W&B and Rerun; a new alias is one more frozen config.", '''
chain = chain_values()
for run_id, values in chain.items():
    rounded = {name: round(value, 3) for name, value in values.items()}
    print(run_id, rounded)

stops, rung_labels = [], []
for rung in RUNGS:
    merged = real_strata[("can11", rung)]["merged"]
    rung_stops = can11_collection.items(merged, "tip stop error")
    stops += rung_stops
    rung_labels += [f"{rung:g}"] * len(rung_stops)
len(stops)

import _viz
title = "CAN 11 · every stop at the tip, by rung"
view = _viz.View(title=title, unit="mm", hint="distribution",
                 x=list(range(len(stops))), y=stops, group=rung_labels,
                 target=1.0, target_label="Zach's 1 mm", x_label="stop", y_label="mm")
_viz.plot(view, path=MEDIA / "v4-tip-stops.png")

payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
payload["summary"]

calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
_viz.write_rrd(calls, MEDIA / "v4-tip-stops.rrd")

VELOCITY_ERROR
with_velocity = Collection(arm.metrics + (VELOCITY_ERROR,))
velocity_values = with_velocity.values(with_velocity.fold(run))
velocity_values["velocity error RMS"]
with_velocity.computations
'''),
    ("other inputs: repeats and a calibration", "Each class declares its Input; a Run is the common case. "
     "Repeatability folds one real repeat at a time; PerformanceGain folds the alias's metric over both sides.", '''
inputs = (ErrorStat, Dwell, TimeOffset, Repeatability, PerformanceGain)
{cls.__name__: cls.Input for cls in inputs}

repeatability_metric, repeatability_state, repeats = repeatability("can11", 0.9)
repeats
tip_mm = 1e3 * TIPS["can11"]
repeatability_metric.compute(repeatability_state) * tip_mm    # mm at the tip

outcome = calibration.outcome()           # a robot_msgs CalibrationOutcome message
outcome.model

gain, gain_state = performance_gain()
gain
nominal, fitted = gain_state
nominal, fitted                           # N·m on the held-out runs
gain.compute(gain_state)
'''),
]


def main() -> None:
    session = Session(dict(globals()))
    session.cell(*GLANCE, kind="glance")
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    MEDIA.mkdir(parents=True, exist_ok=True)
    main()
