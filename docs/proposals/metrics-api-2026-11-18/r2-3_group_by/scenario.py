"""r2-3 · Group-by expressions: the shared story as one glance and one session on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/r2-3_group_by/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs matplotlib and rerun-sdk). ``run()`` is
what the parity check calls: the same API, no transcript. The glance runs in one fresh namespace and the session in a
second one, so every name the page uses is imported or defined on the page.
"""

from __future__ import annotations

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROUND2 = HERE.parent
ROUND1 = ROUND2.parent / "metrics-api-2026-11-17"
for folder in (ROUND1, ROUND2, HERE):        # a reader's sys.path: this folder, round 2's, round 1's
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import calibration  # noqa: E402
import integrated_servo  # noqa: E402
import pendulum_arm  # noqa: E402
import rack_gates  # noqa: E402
import rigid_chain  # noqa: E402
from _session import Session  # noqa: E402
from fixture import chain_runs, pendulum_runs  # noqa: E402
from measures import COMPUTED, DEGREES, MEASURE_KINDS, PerformanceGain, RowError, card, recording_key  # noqa: E402
from rung_ladder import highest_rung  # noqa: E402
from runs_table import Live, Runs, chunks  # noqa: E402

MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-18")
VELOCITY_ERROR = RowError("velocity error", "deg/s", field="velocities", rows="moving",
                          description="the speed between measured and target on each moving row",
                          references=(integrated_servo.COMPARE,))


def _key(station: str, rung: float) -> str:
    return f"{station}@{float(rung):g}"


def run() -> dict:
    """Every number the parity check reads, from the API as the page uses it."""

    real, sim = Runs(pendulum_runs("real")), Runs(pendulum_runs("sim"))
    arm = pendulum_arm.metrics
    tracking, joint_peak = arm[0], arm[1]
    tip_stop_error, tip_settle_time = pendulum_arm.tip_stop_error, pendulum_arm.tip_settle_time

    plays = real.group_by("id").agg(*arm)
    real_plays = {row["id"]: {"tracking_rms_deg": row[tracking.name], "tracking_peak_deg": row[joint_peak.name],
                              "stops": row["tip stop count"], "overshoot_deg": row["worst overshoot (deg)"],
                              "lag_ms": row["median lag (ms)"]} for row in plays.rows()}

    rungs = rack_gates.judge(real.group_by("station", "rung").agg(*arm))
    real_rungs = {_key(row["station"], row["rung"]): {
        "tracking_rms_deg": row[tracking.name], "tracking_peak_deg": row[joint_peak.name],
        "stop_tool_mm": row["median tip stop (mm)"], "stop_tool_max_mm": row["worst tip stop (mm)"],
        "settle_s": row["median tip settle (s)"], "limiter_share": row["limited row share (%)"] / 100.0}
        for row in rungs.rows()}

    sim_plays = {row["id"]: {"tracking_rms_deg": row["value"]}
                 for row in sim.group_by("id").agg(value=pendulum_arm.tracking_error.rms()).rows()}
    sim_table = sim.group_by("station", "rung").agg(tracking, joint_peak, tip_stop_error.median(), tip_stop_error.max())
    sim_rungs = {_key(row["station"], row["rung"]): {
        "tracking_rms_deg": row[tracking.name], "tracking_peak_deg": row[joint_peak.name],
        "stop_tool_mm": row["median tip stop (mm)"], "stop_tool_max_mm": row["worst tip stop (mm)"]}
        for row in sim_table.rows()}

    chain = Runs(chain_runs("real") + chain_runs("sim")).group_by("id").agg(*rigid_chain.metrics)
    chain_rows = {row["id"]: {"positions/j2": row["j2 RMS (deg)"] / DEGREES,
                              "positions/c1": row["c1 RMS (deg)"] / DEGREES,
                              "tool_position_rms_m": row["tool RMS (mm)"] / 1e3,
                              "tool_position_max_m": row["worst tool (mm)"] / 1e3} for row in chain.rows()}

    recording = pendulum_runs("real")[32]
    live_metrics = {"tracking_rms_deg": pendulum_arm.tracking_error.rms(), "tracking_peak_deg": joint_peak,
                    "stops": tip_stop_error.count(), "stop_tool_max_mm": tip_stop_error.max(),
                    "overshoot_deg": pendulum_arm.overshoot.max(if_none=0.0),
                    "settle_s": tip_settle_time.median()}
    live = Live(**live_metrics)
    for chunk in chunks(recording, rows=7):
        live.update(chunk)
    live_row = live.result().rows()[0]
    offline_row = Runs([recording]).agg(**live_metrics).rows()[0]

    repeats = real.filter(station="can11", rung=0.9, kind="repeat")
    repeat_mm = repeats.agg(value=pendulum_arm.tip_stop_offset.repeatability(k=3.0)).item()
    tip_mm = 1e3 * repeats.runs[0].tip_length_m

    residual = integrated_servo.torque_residual.rms()
    held_out = Runs(calibration.held_out_runs("nominal") + calibration.held_out_runs("fitted"))
    gain = PerformanceGain(residual)
    nominal, fitted = gain.sides(held_out.runs)

    added = real.group_by("id").agg(value=VELOCITY_ERROR.rms())
    Runs([recording]).agg(*arm)        # every metric of the arm on one recording: how many passes ran over it?
    return {
        "real_plays": real_plays, "real_rungs": real_rungs, "sim_plays": sim_plays, "sim_rungs": sim_rungs,
        "highest_rung": highest_rung(rungs, "tracks", "median_stop_ok"),
        "gates": {"tracking": [rungs["tracks"].sum(), len(rungs)],
                  "tool_stop_median": [rungs["median_stop_ok"].sum(), len(rungs)],
                  "tool_stop_worst": [rungs["every_stop_ok"].sum(), len(rungs)]},
        "chain": chain_rows,
        "live": {"recording": recording.id,
                 "values": {name: [live_row[name], offline_row[name]] for name in live_metrics}},
        "metric_cards": [pendulum_arm.card("worst tip stop (mm)"), pendulum_arm.card("every stop settled")],
        "declared": {"integrated_servo": [metric.name for metric in integrated_servo.metrics],
                     "rigid_body_chain": [metric.name for metric in rigid_chain.metrics]},
        "added": {"name": VELOCITY_ERROR.rms().name, "values": {row["id"]: row["value"] for row in added.rows()}},
        "aliases": [{"name": metric.name, "base": type(metric.measure).__name__, "config": metric.config(),
                     "unit": metric.unit, "signal": "/".join(metric.measure.streams)} for metric in arm],
        "dedup": {"aliases": len(arm), "computed": COMPUTED[recording_key(recording)]},
        "inputs": {kind.__name__: kind.input for kind in MEASURE_KINDS},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": repeat_mm / tip_mm, "repeats": len(repeats)},
        "performance_gain": {"alias": card(residual)["name"], "nominal": nominal, "fitted": fitted,
                             "gain": held_out.agg(value=gain).item()},
    }


# WHY eight blank-line groups under one-line comments: the contract's story, in its order, so the five glances compare
# step by step (2026-11-18 contract, "The glance: one story, the same in every variant").
GLANCE = ("API at a glance", "A metric is an expression: what is measured (tip_stop_error, in mm), then how it is "
          "summarised (.median(), .max(), .rms()). The same expressions summarise one recording, a live feed of it, "
          "and runs grouped by station and rung; a gate is a comparison on the finished table.", '''
# load the real runs: 252 recordings from round 8 on the 1-DOF test rack
from fixture import pendulum_runs
# the API: a table of runs to group and summarise, as in pandas or Polars
from runs_table import Live, Runs, Threshold, chunks
from pendulum_arm import tip_stop_error, tracking_error  # the pendulum arm's metrics

# the pendulum arm's metrics: what is measured, each with its unit
tip_stop_error
tracking_error
# and how each is summarised: the median stop, the worst stop, the RMS tracking error
summaries = [tip_stop_error.median(), tip_stop_error.max(), tracking_error.rms()]

# one recording, offline: CAN 11, rung 0.9, a repeat
run = pendulum_runs("real")[32]
offline = Runs([run]).agg(*summaries)
offline

# the same recording, live: fed 7 rows at a time, as they would arrive
live = Live(*summaries)
for chunk in chunks(run, rows=7):
    live.update(chunk)
live.result()

# equal: live and offline give the same numbers
live.result().equals(offline, within=1e-9)

# many runs into a table by station and rung: each cell pools the stops of its 7 runs
runs = Runs(pendulum_runs("real"))
table = runs.group_by("station", "rung").agg(*summaries)
# the worst stop of each cell, one column per station
table.pivot(on="station", index="rung", values="worst tip stop (mm)")

# Zach's 1 mm tip-stop gate: a threshold with its owner, kept outside the metrics
limit = Threshold(1.0, "mm", owner="Zach, 2026-11-11")
limit

# its verdict per rung cell: the median stop and the worst stop, each against the gate
median_ok = table["median tip stop (mm)"] <= limit
worst_ok = table["worst tip stop (mm)"] <= limit
table = table.with_columns(median_ok=median_ok, worst_ok=worst_ok)
table.filter(station="can11")
print("median stop within 1 mm:", median_ok.sum(), "of", len(table), "cells")
print("worst stop within 1 mm:", worst_ok.sum(), "of", len(table), "cells")
''')

CELLS = [
    ("setup", "Imports, then the real data: round 8's 252 real plays, the twin's answer to each, the five-bar's "
     "speed ladder and the sealed friction ladder's held-out runs. A Runs table has one row per recording; its "
     "columns are the recording's labels.", '''
# the data: round 8's real plays, the twin's answers, the five-bar ladder, a calibration
from fixture import chain_runs, pendulum_runs
import calibration
# the group-by face: tables of runs, expressions, and the parts that list their metrics
from runs_table import Live, Runs, Threshold, chunks, metrics_table
from measures import MEASURE_KINDS, PerformanceGain, RowError
import integrated_servo
import pendulum_arm
import rigid_chain
from pendulum_arm import tip_stop_error, tracking_error
# the gates and one helper, both outside every metric
import rack_gates
from rung_ladder import highest_rung

real = Runs(pendulum_runs("real"))
real

sim = Runs(pendulum_runs("sim"))
len(sim)
'''),
    ("a part's metrics", "Each part type is a module: its models, and beside them its metrics, a list of "
     "expressions. A measure says what is read from a recording and its unit; the summary says how its values are "
     "pooled. Adding a metric is one line in the list.", '''
# the integrated servo: its models, and its metrics beside them
integrated_servo.models
metrics_table(integrated_servo.metrics)

# the pendulum arm: the servo's rows, plus the tip in mm (each run carries its lever)
metrics_table(pendulum_arm.metrics)

# a metric is a measure (what is read, with its unit) and a summary (how it is pooled)
tip_stop_error
pendulum_arm.card("worst tip stop (mm)")
pendulum_arm.card("every stop settled")["value_type"]

# the five-bar's metrics, from the same building blocks
metrics_table(rigid_chain.metrics)
'''),
    ("one recording, offline and live", "Offline is the live code fed the whole recording at once. Live keeps a "
     "running total per metric between chunks; a stop still open at a chunk boundary stays open until the next "
     "chunk. Lag needs the whole recording, so it is offline only.", '''
run = pendulum_runs("real")[32]       # CAN 11, rung 0.9, a repeat
offline = Runs([run]).agg(*pendulum_arm.metrics)
offline.transpose(include_header=True, header_name="metric", column_names=["offline"])

# live: every metric that can run on chunks as they arrive
live_metrics = [metric for metric in pendulum_arm.metrics if metric.live]
live = Live(*live_metrics)
for chunk in chunks(run, rows=7):
    live.update(chunk)
live_table = live.result()
offline_part = offline.select(*live_table.columns)
both = offline_part.vstack(live_table)
both.transpose(include_header=True, header_name="metric",
               column_names=["offline", "live"])
live_table.equals(offline_part, within=1e-9)

# what each running summary holds after the last chunk (its last stop is still open)
live.memory()

# shared computations: each runs once per recording, whichever metric asks first
Runs([run]).explain(*pendulum_arm.metrics)
'''),
    ("runs to a deployment, gates applied", "One group_by per level: rung cells (station x rung), stations, the "
     "deployment. Stops are pooled over a cell's plays; tracking RMS is per run, then the median. The gates are "
     "Thresholds with owners, compared with the finished table.", '''
rungs = real.group_by("station", "rung").agg(*pendulum_arm.metrics)
len(rungs), len(rungs.columns)

# the gates: thresholds with owners, kept apart from the metrics
rack_gates.tip_stop
rack_gates.every_stop
rack_gates.tracking_rms
judged = rack_gates.judge(rungs)
verdict = ["rung", "median tip stop (mm)", "worst tip stop (mm)",
           "tracks", "median_stop_ok", "every_stop_ok"]
judged.filter(station="can11").select(*verdict)

cells = len(judged)
print("tracks:", judged["tracks"].sum(), "of", cells, "cells")
print("median stop within 1 mm:", judged["median_stop_ok"].sum(), "of", cells, "cells")
print("every stop within 1 mm:", judged["every_stop_ok"].sum(), "of", cells, "cells")
highest_rung(judged, "tracks", "median_stop_ok")

# up the roll-up: each station, then the whole deployment
by_station = {"tracking_deg": tracking_error.rms().per_run().median(),
              "median_stop_mm": tip_stop_error.median(),
              "worst_stop_mm": tip_stop_error.max(),
              "stops": tip_stop_error.count()}
real.group_by("station").agg(**by_station)
real.agg(**by_station)
'''),
    ("the twin's output (sim vs target)", "The same expressions on the twin's answers, joined beside the real "
     "numbers. Every number is the target against one source.", '''
twin = {"tracking_deg": tracking_error.rms().per_run().median(),
        "median_mm": tip_stop_error.median(), "worst_mm": tip_stop_error.max()}
real_rungs = real.group_by("station", "rung").agg(**twin)
sim_rungs = sim.group_by("station", "rung").agg(**twin)
real_vs_sim = real_rungs.join(sim_rungs, on=["station", "rung"], suffix="_sim")
real_vs_sim.filter(station="can11")

sim_median_ok = sim_rungs["median_mm"] <= rack_gates.tip_stop
sim_median_ok.sum(), len(sim_rungs)
'''),
    ("the chain, a view, a new metric", "The five-bar through the same building blocks; every CAN 11 stop as one "
     "view record sent to matplotlib, a W&B payload and Rerun; a new metric is one constructor call.", '''
chain = Runs(chain_runs("real") + chain_runs("sim"))
chain.group_by("source", "rung").agg(*rigid_chain.metrics)

# a view: every CAN 11 stop at the tip, one row per stop, then one record to three sinks;
# the measure carries the data-ui view that shows it
tip_stop_error.hint
can11_stops = real.filter(station="can11").explode(tip_stop_error)
len(can11_stops)
import pathlib
import _viz           # round 1's shared adapter: matplotlib, a W&B payload, Rerun calls
reports = pathlib.Path("/home/bam/robot_models/reports/media")
media = reports / "robot-models-metrics-api-proposals-2026-11-18"
media.mkdir(parents=True, exist_ok=True)
rung_labels = [f"{rung:g}" for rung in can11_stops["rung"]]
stop_numbers = list(range(len(can11_stops)))
stops_mm = can11_stops["tip stop error (mm)"].to_list()
view = _viz.View(title="CAN 11 · every stop at the tip, by rung", unit="mm",
                 hint=tip_stop_error.hint, x=stop_numbers, y=stops_mm, group=rung_labels,
                 target=1.0, target_label="Zach's 1 mm", x_label="stop")
figure, axes = _viz.plot(view, path=media / "r2-3-tip-stops.png")
payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
sorted(payload["summary"])
calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
len(calls)
rrd = _viz.write_rrd(calls, media / "r2-3-tip-stops.rrd")
rrd.name

# a new metric: one constructor call, then summarised like any other
velocity_error = RowError("velocity error", "deg/s", field="velocities", rows="moving",
                          description="measured minus target speed on each moving row")
real.group_by("station").agg(velocity_error.rms(), velocity_error.max())
'''),
    ("other inputs: repeats and a calibration", "Repeatability takes the repeats of one command; a performance "
     "gain takes held-out runs played with nominal and with fitted friction values. Both are summaries inside "
     "agg(), over runs filtered or grouped by their labels.", '''
repeats = real.filter(station="can11", rung=0.9, kind="repeat")
len(repeats)
repeats.agg(pendulum_arm.tip_stop_offset.repeatability(k=3))

nominal_runs = calibration.held_out_runs("nominal")
fitted_runs = calibration.held_out_runs("fitted")
held_out = Runs(nominal_runs + fitted_runs)
residual = integrated_servo.torque_residual.rms()
held_out.group_by("values").agg(residual)
held_out.agg(PerformanceGain(residual))

# what each kind of measure or summary takes in
{kind.__name__: kind.input for kind in MEASURE_KINDS}
'''),
]


def main() -> None:
    session = Session({"__name__": "__main__"})
    session.cell(*GLANCE, kind="glance")
    session.namespace = {"__name__": "__main__"}      # the session: a second fresh namespace, never this module's
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    MEDIA.mkdir(parents=True, exist_ok=True)
    main()
