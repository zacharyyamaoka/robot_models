"""r2-2 · Plain functions, table out: round 2's story as one REPL session on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/r2-2_functions/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs matplotlib and rerun-sdk). The glance
runs in one fresh namespace and the session in a second one, so neither can lean on a name this file defines.
``run()`` is what the frozen parity check calls: the same functions, keyed in the check's vocabulary.
"""

from __future__ import annotations

import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROUND1 = HERE.parent.parent / "metrics-api-2026-11-17"
# WHY exactly these three: a reader's sys.path holds the variant folder, round 2's folder and round 1's, nothing else
for folder in (ROUND1, HERE.parent, HERE):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import calibration  # noqa: E402
import fixture  # noqa: E402
import gates  # noqa: E402
import parts  # noqa: E402
from _session import Session  # noqa: E402
from run_metrics import (DEGREES, LiveScore, c1_tracking_rms_deg, chunks, faulted_runs, feedforward_residual_rms_nm,  # noqa: E402
                         j2_tracking_rms_deg, lag_ms, limited_rows_pct, median_tip_stop_mm, metric_card,
                         overshoot_deg, passes_over_rows, performance_gain, repeatability_rad, scorecard, stop_count,
                         tip_settle_s, tool_tracking_rms_mm, tracking_peak_deg, tracking_rms_deg, worst_tip_stop_mm,
                         worst_tool_error_mm)

# WHY one source for the new metric: the session defines it on the page, and run() executes the very same text
NEW_METRIC = '''
# a new metric is one plain function (stock.moving: the rows with a commanded speed)
import math
import statistics

import numpy as np
import stock
from run_metrics import as_list


def velocity_error_rms_deg_s(runs):
    """RMS of measured - target speed while the arm moves, in deg/s"""
    per_run = []
    for one_run in as_list(runs):
        moving = stock.moving(one_run.target)
        measured = one_run.measured.velocities[moving, 0]
        target = one_run.target.velocities[moving, 0]
        per_run.append(math.degrees(math.sqrt(np.mean((measured - target) ** 2))))
    return statistics.median(per_run)
'''

GLANCE = ("API at a glance", "Plain functions in the sklearn.metrics shape: each metric is a function you call on a "
          "run and it returns a number in the unit its name ends with. Many runs go into a table with scorecard(), "
          "a pandas-like group-by. Live is a LiveScore fed chunks of rows; the gate is a number compared with a "
          "column of the table.", '''
# load the real runs: round 8 of the 1-DOF pendulum rack, 252 recordings from /archive
from fixture import pendulum_runs              # reads the recordings from disk
from parts import PENDULUM_ARM_METRICS         # each part's list of metric functions
from run_metrics import LiveScore, chunks, scorecard
from run_metrics import median_tip_stop_mm, worst_tip_stop_mm
runs = pendulum_runs("real")
len(runs)

# the pendulum arm's metrics: a plain list of functions; a name ends in its unit
for metric in PENDULUM_ARM_METRICS:
    print(f"{metric.__name__:<19} {metric.__doc__}")

# one recording, offline: call a metric on the run and get a number back
run = runs[32]                     # station CAN 11, rung 0.9, a repeat
median_tip_stop_mm(run)
worst_tip_stop_mm(run)

# the same recording, live: its rows fed in 7 at a time, as they would arrive
live = LiveScore(PENDULUM_ARM_METRICS)
for chunk in chunks(run, rows=7):
    live.add(chunk)
live.result()["worst_tip_stop_mm"]

# equal: live and offline give the same value for every metric
offline = {metric.__name__: metric(run) for metric in PENDULUM_ARM_METRICS}
live.result() == offline

# many runs into a table: CAN 11's 42 runs, one row per station and rung
can11_runs = [recording for recording in runs if recording.station == "can11"]
stop_metrics = [median_tip_stop_mm, worst_tip_stop_mm]
table = scorecard(can11_runs, by=["station", "rung"], metrics=stop_metrics)
table

# Zach's 1 mm gate on the tip stop: a plain number, kept outside the metric functions
gate_mm = 1.0                      # owner: Zach, 2026-11-11

# its verdict per rung: the median stop passes every rung, the worst stop fails three
table["median passes"] = [stop <= gate_mm for stop in table["median_tip_stop_mm"]]
table["worst passes"] = [stop <= gate_mm for stop in table["worst_tip_stop_mm"]]
table
''')

CELLS = [
    ("setup", "Where everything comes from: round 1's fixture reads the recordings (round 8's real plays, the twin's "
     "answers to them, the five-bar's speed ladder); this variant's three modules hold the functions, each part "
     "type's list of them, and the gates.", '''
# the data: round 8 of the pendulum rack (real and twin), the five-bar's speed ladder
import fixture

# this variant: the metric functions, each part type's list of them, and the rack's gates
import gates
import parts
from run_metrics import LiveScore, Table, chunks, scorecard
from run_metrics import metric_card, metric_table, passes_over_rows
from run_metrics import lag_ms, limited_rows_pct, overshoot_deg, faulted_runs
from run_metrics import median_tip_stop_mm, worst_tip_stop_mm
from run_metrics import tip_settle_s, tracking_rms_deg

real_runs = fixture.pendulum_runs("real")
sim_runs = fixture.pendulum_runs("sim")
chain_runs = fixture.chain_runs("real") + fixture.chain_runs("sim")
len(real_runs), len(sim_runs), len(chain_runs)
'''),
    ("a part type's metrics", "Each part type is two plain names in parts.py: its models and its list of metric "
     "functions. A function's card holds Zach's Metric fields (name, description, references) plus its unit, how "
     "several runs combine, and whether it can run live.", '''
parts.INTEGRATED_SERVO_MODELS
metric_table(parts.INTEGRATED_SERVO_METRICS)

print(parts.PENDULUM_ARM_MODELS.title)
metric_table(parts.PENDULUM_ARM_METRICS)

metric_table(parts.RIGID_BODY_CHAIN_METRICS)

metric_card(median_tip_stop_mm)
'''),
    ("one recording, offline and live", "Offline is a function call on the run. Live is LiveScore fed chunks: 7 rows "
     "at a time, or one row at a time; both equal offline exactly. Lag needs the whole run, and LiveScore says so "
     "instead of quietly buffering it.", '''
run = real_runs[32]                     # CAN 11, rung 0.9, a repeat
run

live_metrics = parts.PENDULUM_ARM_METRICS + [overshoot_deg, limited_rows_pct]
in_sevens = LiveScore(live_metrics)
for chunk in chunks(run, rows=7):
    in_sevens.add(chunk)
one_by_one = LiveScore(live_metrics)
for single_row in chunks(run, rows=1):
    one_by_one.add(single_row)

seven_rows, one_row = in_sevens.result(), one_by_one.result()
rows = []
for metric in live_metrics:
    name = metric.__name__
    rows.append({"metric": name, "offline": metric(run), "live, 7 rows": seven_rows[name],
                 "live, 1 row": one_row[name]})
Table(rows)

offline = {metric.__name__: metric(run) for metric in live_metrics}
seven_rows == offline, one_row == offline
passes_over_rows(run)                   # nine metrics, the run's rows read once

in_sevens.kept_in_memory()

lag_ms(run)
try:
    LiveScore([lag_ms])
except ValueError as reason:
    print(reason)
'''),
    ("runs to a deployment, gates applied", "scorecard() groups the runs and calls each function on each group; a "
     "metric decides how several runs combine (a rung's median stop pools all its runs' stops). The gates are rows "
     "of (column, at most, owner) in gates.py, compared with the table's columns.", '''
rung_metrics = parts.PENDULUM_ARM_METRICS + [limited_rows_pct, faulted_runs]
table = scorecard(real_runs, by=["station", "rung"], metrics=rung_metrics)
len(table)

Table(gates.gate_rows())

for gate_name, gate in gates.GATES.items():
    table[gate_name] = gates.passes(table, gate)
for gate_name in gates.GATES:
    print(f"{gate_name:<21} passes {sum(table[gate_name])} of {len(table)} rung cells")

table.pivot(index="station", columns="rung", values="worst_tip_stop_mm")
table.pivot(index="station", columns="rung", values="tip stop, every stop")
gates.highest_rung(table, ["tracks", "tip stop, median"])

station_metrics = [tracking_rms_deg, median_tip_stop_mm, worst_tip_stop_mm]
scorecard(real_runs, by=["station"], metrics=station_metrics)
scorecard(real_runs, by=["deployment"], metrics=station_metrics)
'''),
    ("the twin's output (sim vs target)", "The same functions on the twin's answers: a run's source is a label, so "
     "real and sim sit in one table.", '''
sim_metrics = [tracking_rms_deg, median_tip_stop_mm, worst_tip_stop_mm, tip_settle_s]
sim_table = scorecard(sim_runs, by=["station", "rung"], metrics=sim_metrics)
sum(gates.passes(sim_table, gates.GATES["tip stop, median"]))

can11_both = [one for one in real_runs + sim_runs if one.station == "can11"]
scorecard(can11_both, by=["rung", "source"], metrics=sim_metrics[:3])
'''),
    ("the chain, a view, a new metric", "The five-bar through the same scorecard; every CAN 11 tip stop as one view to "
     "matplotlib, a W&B payload and a Rerun recording, with the data-ui hint the metric carries; and a new metric "
     "added as one plain function.", '''
scorecard(chain_runs, by=["source", "rung"], metrics=parts.RIGID_BODY_CHAIN_METRICS)

import pathlib
import _viz
import run_metrics

can11_runs = [one for one in real_runs if one.station == "can11"]
can11_runs.sort(key=lambda one: one.rung)
stops_mm, rung_labels = [], []
for one in can11_runs:
    run_stops = run_metrics.tip_stop_errors_mm(one)
    stops_mm += run_stops
    rung_labels += [f"rung {one.rung}"] * len(run_stops)
len(stops_mm), max(stops_mm)

view = _viz.View(title="CAN 11 · every stop at the tip, by rung", unit="mm",
                 hint=worst_tip_stop_mm.hint, x=list(range(len(stops_mm))), y=stops_mm,
                 group=rung_labels, target=1.0, target_label="Zach's 1 mm",
                 x_label="stop", y_label="mm")
media = pathlib.Path(run_metrics.__file__).parent / "media"
media.mkdir(exist_ok=True)
figure, axes = _viz.plot(view, path=media / "tip-stops.png")
axes.get_title()

payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
payload["summary"]

calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
rrd_file = _viz.write_rrd(calls, media / "tip-stops.rrd")
len(calls)
''' + NEW_METRIC + '''
velocity_error_rms_deg_s(run)
new_metrics = [tracking_rms_deg, velocity_error_rms_deg_s]
scorecard(can11_runs, by=["rung"], metrics=new_metrics)
'''),
    ("other inputs: repeats and a calibration", "Two metrics whose input is not one recording: repeatability takes "
     "runs of the same command; a performance gain takes a metric and two sets of held-out runs, played with "
     "nominal and with fitted friction values.", '''
import calibration
from run_metrics import tip_repeatability_mm
from run_metrics import feedforward_residual_rms_nm, performance_gain

can11_rung_09 = [one for one in real_runs if one.station == "can11" and one.rung == 0.9]
repeats = [one for one in can11_rung_09 if one.labels["kind"] == "repeat"]
len(repeats)
tip_repeatability_mm(repeats, k=3)      # ISO 9283, mm at the tip, the worst stop

nominal_runs = calibration.held_out_runs("nominal")
fitted_runs = calibration.held_out_runs("fitted")
feedforward_residual_rms_nm(nominal_runs)
feedforward_residual_rms_nm(fitted_runs)
performance_gain(feedforward_residual_rms_nm, nominal_runs, fitted_runs)
'''),
]


def _new_metric():
    namespace: dict = {"__name__": "new_metric"}
    exec(NEW_METRIC, namespace)
    return namespace["velocity_error_rms_deg_s"]


def run() -> dict:
    """The parity check's contract: the same functions, keyed in the check's vocabulary (none of it on the page)."""

    real, sim = fixture.pendulum_runs("real"), fixture.pendulum_runs("sim")
    real_plays = {}
    for one in real:
        real_plays[one.id] = {"tracking_rms_deg": tracking_rms_deg(one), "tracking_peak_deg": tracking_peak_deg(one),
                              "stops": stop_count(one), "overshoot_deg": overshoot_deg(one), "lag_ms": lag_ms(one)}
    rung_metrics = [tracking_rms_deg, tracking_peak_deg, median_tip_stop_mm, worst_tip_stop_mm, tip_settle_s,
                    limited_rows_pct, faulted_runs]
    table = scorecard(list(real), by=["station", "rung"], metrics=rung_metrics)
    for name, gate in gates.GATES.items():
        table[name] = gates.passes(table, gate)
    real_rungs = {f"{row['station']}@{row['rung']:g}": {
        "tracking_rms_deg": row["tracking_rms_deg"], "tracking_peak_deg": row["tracking_peak_deg"],
        "stop_tool_mm": row["median_tip_stop_mm"], "stop_tool_max_mm": row["worst_tip_stop_mm"],
        "settle_s": row["tip_settle_s"], "limiter_share": row["limited_rows_pct"] / 100.0} for row in table}

    sim_ids = {one.id: one for one in sim}
    sim_plays = {play_id: {"tracking_rms_deg": tracking_rms_deg(sim_ids[play_id]) if play_id in sim_ids else None}
                 for play_id in real_plays}
    sim_table = scorecard(list(sim), by=["station", "rung"], metrics=rung_metrics[:4])
    sim_rungs = {f"{row['station']}@{row['rung']:g}": {
        "tracking_rms_deg": row["tracking_rms_deg"], "tracking_peak_deg": row["tracking_peak_deg"],
        "stop_tool_mm": row["median_tip_stop_mm"], "stop_tool_max_mm": row["worst_tip_stop_mm"]} for row in sim_table}

    chain = {}
    for one in fixture.chain_runs("real") + fixture.chain_runs("sim"):
        chain[one.id] = {"positions/j2": j2_tracking_rms_deg(one) / DEGREES,
                         "positions/c1": c1_tracking_rms_deg(one) / DEGREES,
                         "tool_position_rms_m": tool_tracking_rms_mm(one) / 1e3,
                         "tool_position_max_m": worst_tool_error_mm(one) / 1e3}

    recording = real[32]
    live_metrics = parts.PENDULUM_ARM_METRICS + [overshoot_deg, limited_rows_pct]
    live = LiveScore(live_metrics)
    for chunk in chunks(recording, rows=7):
        live.add(chunk)
    live_values = live.result()
    pairs = {"tracking_rms_deg": "tracking_rms_deg", "tracking_peak_deg": "tracking_peak_deg", "stops": "stop_count",
             "stop_tool_max_mm": "worst_tip_stop_mm", "overshoot_deg": "overshoot_deg"}
    offline = {metric.__name__: metric(recording) for metric in live_metrics}

    every_metric = []
    for metric in parts.PENDULUM_ARM_METRICS + parts.INTEGRATED_SERVO_METRICS + parts.RIGID_BODY_CHAIN_METRICS:
        if metric not in every_metric:
            every_metric.append(metric)
    cards = []
    for metric in (median_tip_stop_mm, parts.PENDULUM_ARM_METRICS[5]):
        card = metric_card(metric)
        cards.append({"name": card["name"], "description": card["description"], "references": card["references"],
                      "function": card["function"], "unit": card["unit"], "target": gates.target_of(card["name"]),
                      "value_type": card["value"]})

    new_metric = _new_metric()
    repeats = [one for one in real if one.station == "can11" and one.rung == 0.9 and one.labels["kind"] == "repeat"]
    nominal_runs, fitted_runs = calibration.held_out_runs("nominal"), calibration.held_out_runs("fitted")
    return {
        "real_plays": real_plays, "real_rungs": real_rungs, "sim_plays": sim_plays, "sim_rungs": sim_rungs,
        "highest_rung": gates.highest_rung(table, ["tracks", "tip stop, median"]),
        "gates": {"tracking": [sum(table["tracks"]), len(table)],
                  "tool_stop_median": [sum(table["tip stop, median"]), len(table)],
                  "tool_stop_worst": [sum(table["tip stop, every stop"]), len(table)]},
        "chain": chain,
        "live": {"recording": recording.id,
                 "values": {key: [live_values[name], offline[name]] for key, name in pairs.items()}},
        "metric_cards": cards,
        "declared": {"integrated_servo": [metric.__name__ for metric in parts.INTEGRATED_SERVO_METRICS],
                     "rigid_body_chain": [metric.__name__ for metric in parts.RIGID_BODY_CHAIN_METRICS]},
        "added": {"name": new_metric.__name__, "values": {one.id: new_metric(one) for one in real}},
        "aliases": [{"name": metric.__name__, "base": metric.base, "config": metric.config, "unit": metric.unit,
                     "signal": metric.signal} for metric in every_metric],
        "dedup": {"aliases": len(live_metrics), "computed": passes_over_rows(recording)},
        "inputs": {"moving_error_rms": "a run, or its chunks (target and measured on the same instants)",
                   "time_offset": "a whole run: the plan on its ticks and the measured run",
                   "error_stats": "a whole run of a chain: joint and tool target and measured",
                   "Repeatability": "runs of the same command (a list of repeats)",
                   "PerformanceGain": "a metric and two lists of held-out runs (nominal values, fitted values)"},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": repeatability_rad(repeats, k=3.0),
                          "repeats": len(repeats)},
        "performance_gain": {"alias": feedforward_residual_rms_nm.__name__,
                             "nominal": feedforward_residual_rms_nm(nominal_runs),
                             "fitted": feedforward_residual_rms_nm(fitted_runs),
                             "gain": performance_gain(feedforward_residual_rms_nm, nominal_runs, fitted_runs)},
    }


def main() -> None:
    # WHY two fresh dicts, never globals(): round 1's harness ran its cells in this file's globals, so a displayed
    # block could use a hidden name and still "run"; here the glance and the session each start empty
    session = Session({"__name__": "__main__"})
    session.cell(*GLANCE, kind="glance")
    session.namespace = {"__name__": "__main__"}
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    main()
