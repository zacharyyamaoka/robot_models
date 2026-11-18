"""r2-1 · Keras / torchmetrics face: the round's scenario on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/r2-1_keras/scenario.py

Writes scenario.out.txt and session.json beside this file. The glance runs in its own fresh namespace
({"__name__": "__main__"}) and the session cells in a second one, so every name they use is imported or defined on
the page (round 1's hidden-globals failure). ``run()`` is what the parity check calls: the same API, no transcript.
"""

from __future__ import annotations

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROUND2 = HERE.parent
ROUND1 = ROUND2.parent / "metrics-api-2026-11-17"
for folder in (ROUND1, ROUND2, HERE):            # a reader's sys.path: this folder, round 2, round 1
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

import calibration  # noqa: E402
import fixture  # noqa: E402
from metrics import PerformanceGain, Repeatability, TrackingError  # noqa: E402
from part_metrics import (DEGREES, integrated_servo_metrics, pendulum_arm_metrics,  # noqa: E402
                          rigid_body_chain_metrics)
from rack_gates import highest_rung, judge  # noqa: E402
from rung_table import rung_table  # noqa: E402


def servo_metrics_at(tip_length_m: float):
    """rung_table makes metrics per cell with the cell's tip length; the servo is judged at its joint, so it ignores it."""

    return integrated_servo_metrics()


def play_numbers(play) -> tuple[dict, dict]:
    arm = pendulum_arm_metrics(tip_length_m=play.tip_length_m)
    servo = integrated_servo_metrics()
    arm.update(play.target, play.measured)
    servo.update(play.target, play.measured, plan=play.plan)
    return arm.compute(), servo.compute()


def rung_rows(table, servo_table=None) -> dict:
    rows = {}
    for (station, rung), numbers in table.rows.items():
        row = {"tracking_rms_deg": numbers["tracking RMS (deg)"], "tracking_peak_deg": numbers["tracking peak (deg)"],
               "stop_tool_mm": numbers["tip stop median (mm)"], "stop_tool_max_mm": numbers["tip stop worst (mm)"]}
        if servo_table is not None:
            row["settle_s"] = numbers["tip settle median (s)"]
            row["limiter_share"] = servo_table.rows[(station, rung)]["limiter share (%)"] / 100.0
        rows[f"{station}@{rung:g}"] = row
    return rows


def run() -> dict:
    real = fixture.pendulum_runs("real")
    sim = fixture.pendulum_runs("sim")

    real_plays = {}
    for play in real:
        arm, servo = play_numbers(play)
        real_plays[play.id] = {"tracking_rms_deg": arm["tracking RMS (deg)"],
                               "tracking_peak_deg": arm["tracking peak (deg)"], "stops": arm["stops"],
                               "overshoot_deg": servo["overshoot (deg)"], "lag_ms": servo["lag (ms)"]}

    table = rung_table(real, pendulum_arm_metrics)
    servo_table = rung_table(real, servo_metrics_at)
    faults: dict = {}
    for play in real:
        faults[(play.station, play.rung)] = faults.get((play.station, play.rung), 0) + int(play.faulted)
    cells = {key: {**table.rows[key], **servo_table.rows[key], "faults": faults[key]} for key in table.rows}
    verdicts = judge(cells)
    gate_names = {"tracking": "tracks", "tool_stop_median": "tip stop, median",
                  "tool_stop_worst": "tip stop, every stop"}
    gates = {key: [sum(cell[name] for cell in verdicts.values()), len(verdicts)] for key, name in gate_names.items()}

    sim_plays = {}
    for play in sim:
        arm = pendulum_arm_metrics(tip_length_m=play.tip_length_m)
        arm.update(play.target, play.measured)
        sim_plays[play.id] = {"tracking_rms_deg": arm.compute()["tracking RMS (deg)"]}
    sim_table = rung_table(sim, pendulum_arm_metrics)

    chain = rigid_body_chain_metrics()
    chain_values = {}
    for source in ("real", "sim"):
        for chain_run in fixture.chain_runs(source):
            chain["joints"].reset()
            chain["tool"].reset()
            chain["joints"].update(chain_run.target, chain_run.measured)
            chain["tool"].update(chain_run.tool_target, chain_run.tool_measured)
            joints, tool = chain["joints"].compute(), chain["tool"].compute()
            chain_values[chain_run.id] = {"positions/j2": joints["j2 tracking RMS (deg)"] / DEGREES,
                                          "positions/c1": joints["c1 tracking RMS (deg)"] / DEGREES,
                                          "tool_position_rms_m": tool["tool tracking RMS (mm)"] / 1000.0,
                                          "tool_position_max_m": tool["tool worst (mm)"] / 1000.0}

    recording = real[32]
    arm = pendulum_arm_metrics(tip_length_m=recording.tip_length_m)
    arm.update(recording.target, recording.measured)
    offline = arm.compute()
    arm.reset()
    for start in range(0, len(recording.measured), 7):
        arm.update(recording.target[start:start + 7], recording.measured[start:start + 7])
    live = arm.compute()
    pairs = {"tracking_rms_deg": "tracking RMS (deg)", "tracking_peak_deg": "tracking peak (deg)", "stops": "stops",
             "stop_tool_mm": "tip stop median (mm)", "stop_tool_max_mm": "tip stop worst (mm)"}
    live_block = {"recording": recording.id, "values": {key: [live[label], offline[label]] for key, label in pairs.items()}}

    velocity_error = TrackingError("velocity error RMS", statistic="rms", field="velocities", while_moving=True,
                                   unit="deg/s", scale=DEGREES)
    added = {}
    for play in real:
        velocity_error.reset()
        velocity_error.update(play.target, play.measured)
        added[play.id] = velocity_error.compute()

    repeats = [play for play in real
               if play.station == "can11" and play.rung == 0.9 and play.labels["kind"] == "repeat"]
    repeatability = Repeatability("repeatability", k=3.0)        # radians at the joint, as the check asks
    for play in repeats:
        repeatability.update(play.target, play.measured)

    residual = TrackingError("feed-forward residual RMS", statistic="rms", field="effort", across_runs="pooled",
                             unit="N·m")
    gain = PerformanceGain(residual)
    gain.update(calibration.held_out_runs("nominal"), calibration.held_out_runs("fitted"))

    servo = integrated_servo_metrics()
    chain = rigid_body_chain_metrics()
    every_metric = [*arm.values(), *servo.values()]
    return {
        "real_plays": real_plays,
        "real_rungs": rung_rows(table, servo_table),
        "sim_plays": sim_plays,
        "sim_rungs": rung_rows(sim_table),
        "highest_rung": highest_rung(verdicts),
        "gates": gates,
        "chain": chain_values,
        "live": live_block,
        "metric_cards": [arm["tip stop worst (mm)"].card(), arm["all stops settled"].card()],
        "declared": {"integrated_servo": list(servo),
                     "rigid_body_chain": [*chain["joints"], *chain["tool"]]},
        "added": {"name": velocity_error.label, "values": added},
        "aliases": [{"name": metric.label, "base": type(metric).__name__, "config": metric.config(),
                     "unit": metric.unit, "signal": metric.reads} for metric in every_metric],
        "inputs": {type(metric).__name__: metric.reads
                   for metric in [*every_metric, repeatability, gain]},
        "repeatability": {"group": "can11@0.9", "k": repeatability.k, "value": repeatability.compute(),
                          "repeats": len(repeats)},
        "performance_gain": {"alias": residual.label, "nominal": gain.nominal, "fitted": gain.fitted,
                             "gain": gain.compute()},
        "dedup": {"aliases": len(arm), "computed": len(arm.parts())},
    }


# The API at a glance: the contract's eight steps, each under one plain comment, every name imported on the page.
GLANCE = ("API at a glance", "Metric objects with the three verbs Keras and torchmetrics use: update() as rows "
          "arrive, compute() for the numbers, reset() to start over. The gate is a separate Threshold.", '''
# load the real runs: 252 recordings from the one-joint test rack (round 1's fixture.py)
from fixture import pendulum_runs
# from this folder: the arm's metrics, a table by station and rung, a pass/fail threshold
from part_metrics import pendulum_arm_metrics
from rung_table import rung_table
from threshold import Threshold
runs = pendulum_runs("real")
len(runs)

# the pendulum arm's metrics: a dict of metric objects, keyed by name and unit
metrics = pendulum_arm_metrics(tip_length_m=0.339)
print(metrics)

# one recording, offline: update() with the whole recording, then compute()
run = runs[32]          # station can11 at speed rung 0.9 (90 % of its top speed)
run
metrics.update(run.target, run.measured)
offline = metrics.compute()
offline

# the same recording, live: reset(), then update() with 7 rows at a time as they arrive
metrics.reset()
for start in range(0, len(run.measured), 7):
    metrics.update(run.target[start:start + 7], run.measured[start:start + 7])
live = metrics.compute()

# live and offline give the same numbers
live == offline

# many runs: fresh metrics for each station and rung (7 runs each), rolled up into a table
table = rung_table(runs, pendulum_arm_metrics)
print(table.for_station("can11"))

# Zach's 1 mm tip-stop gate: a threshold with its owner, kept outside the metrics
gate = Threshold(at_most=1.0, unit="mm", owner="Zach, 2026-11-11")

# its verdict for every station and rung, on the median stop and on the worst stop
print(gate.check(table, "tip stop median (mm)", "tip stop worst (mm)"))
''')

CELLS = [
    ("setup", "Imports first, then the real data: round 8's 252 plays on the rack, the twin's answer to each, and "
     "each station's tip length. Every later cell builds only on names defined here or above it.", '''
# the data, from round 1's folder: real and twin runs, the five-bar, a calibration
from fixture import chain_runs, pendulum_runs, stations
import calibration

# this folder: metric classes, each part type's metrics, the rung table, the gates
from metrics import PerformanceGain, Repeatability, StopError, TrackingError
from part_metrics import DEGREES, PART_TYPES, pendulum_arm_metrics
from part_metrics import integrated_servo_metrics, rigid_body_chain_metrics
from rung_table import rung_table
from rack_gates import GATES, TIP_STOP, highest_rung, judge

real_runs = pendulum_runs("real")
sim_runs = pendulum_runs("sim")
len(real_runs), len(sim_runs)

tip_lengths = {name: facts["tip_length_m"] for name, facts in stations().items()}
tip_lengths
'''),
    ("a part type's metrics, beside its models", "Each part type lists configured metric objects next to its models; "
     "one class serves several names (TrackingError for RMS and peak, StopError for median and worst), and the "
     "unit lives on the configured object, never in the math.", '''
servo_part = PART_TYPES["integrated_servo"]
servo_part["models"].keys()

servo = servo_part["metrics"]()
print(servo)

# CAN 11: the 339 mm pendulum
arm = pendulum_arm_metrics(tip_length_m=tip_lengths["can11"])
print(arm)

chain = rigid_body_chain_metrics()
print(chain["joints"])
print(chain["tool"])

# Zach's Metric fields: name, description, references; a number or a bool; no target
arm["tip stop worst (mm)"].card()
arm["all stops settled"].card()["value_type"]
'''),
    ("one recording, offline and live", "The same metrics fed the whole recording, and fed 7 rows at a time with a "
     "look mid-run: identical numbers. Seven metrics cost three computations: the five stop metrics share one "
     "stop finder (torchmetrics' compute groups).", '''
run = real_runs[32]       # station can11, rung 0.9, a repeat

# offline: the servo's metrics, where lag reads the plan as sent
servo.update(run.target, run.measured, plan=run.plan)
servo.compute()

# live, 7 rows at a time: compute() works mid-run, an open stop counted as if closed
arm.reset()
for start in range(0, len(run.measured), 7):
    arm.update(run.target[start:start + 7], run.measured[start:start + 7])
    if start in (70, 175, 280):
        rows_so_far = start + 7
        stops_so_far = arm.compute()["stops"]
        print(f"after {rows_so_far} rows: stops so far {stops_so_far}")
live = arm.compute()

arm.reset()
arm.update(run.target, run.measured)
arm.compute() == live

# 7 metrics, 3 computations per chunk: one tracking sum, one peak, one shared stop finder
len(arm), len(arm.parts())

# a second recording glued onto the first by mistake is refused, not silently merged
next_run = real_runs[33]
try:
    arm.update(next_run.target, next_run.measured)
except ValueError as refused:
    print(refused)
'''),
    ("many runs, the gates, the highest rung", "One fresh set of metrics per rung cell, fed its 7 plays; the gates "
     "read the table's numbers from outside. The median tip stop passes 36 of 36 cells; the worst stop fails 3 of "
     "CAN 11's rungs.", '''
table = rung_table(real_runs, pendulum_arm_metrics)
len(table)
print(table.for_station("can12"))

# the limiter share is the servo's: fresh servo metrics per cell (the tip does not matter)
def servo_metrics_at(tip_length_m):
    return integrated_servo_metrics()

servo_table = rung_table(real_runs, servo_metrics_at)

faults = {key: 0 for key in table.rows}
for play in real_runs:
    faults[(play.station, play.rung)] += int(play.faulted)

# each cell's numbers in one dict
cells = {}
for key in table.rows:
    cells[key] = {**table.rows[key], **servo_table.rows[key], "faults": faults[key]}

# each gate: its thresholds, each with an owner, read off the numbers from outside
for name, gate in GATES.items():
    for label, line in gate.items():
        print(f"{name:<22} {label} <= {line.at_most:g}   owner: {line.owner}")

verdicts = judge(cells)
for name in GATES:
    passed = sum(cell[name] for cell in verdicts.values())
    print(f"{name:<22} {passed} of {len(verdicts)} rung cells pass")

highest_rung(verdicts)
'''),
    ("the twin's output (sim vs target)", "The same metrics on the twin's answer to every play: the sim's own "
     "tracking and tip stops against the target. Comparing sim with real is fidelity, a separate family.", '''
sim_table = rung_table(sim_runs, pendulum_arm_metrics)
print(sim_table.for_station("can11"))

sim_verdicts = TIP_STOP.check(sim_table, "tip stop median (mm)", "tip stop worst (mm)")
sim_verdicts.passed("tip stop median (mm)"), sim_verdicts.passed("tip stop worst (mm)")
'''),
    ("the chain, a view, a new metric", "The five-bar through the same TrackingError class (joints and the tool); "
     "every CAN 11 stop as a standard view (matplotlib, the same points as a W&B payload and Rerun calls); a new "
     "metric is one constructor call.", '''
labels = [*chain["joints"], *chain["tool"]]
labels

for chain_run in chain_runs("real"):
    chain["joints"].reset()
    chain["tool"].reset()
    chain["joints"].update(chain_run.target, chain_run.measured)
    chain["tool"].update(chain_run.tool_target, chain_run.tool_measured)
    numbers = {**chain["joints"].compute(), **chain["tool"].compute()}
    columns = "".join(f"{numbers[label]:>9.3f}" for label in labels)
    print(f"{chain_run.labels['rung']:<28}{columns}")

# every CAN 11 stop at the tip: one metric on its own, fed 42 plays, each as its own run
mm_at_tip = 1000.0 * tip_lengths["can11"]
can11_worst = StopError("tip stop worst", statistic="worst", unit="mm", scale=mm_at_tip)
for play in real_runs:
    if play.station == "can11":
        can11_worst.update(play.target, play.measured, new_run=True)
len(can11_worst.values()), round(can11_worst.compute(), 3)

import pathlib
import _viz
reports = pathlib.Path("/home/bam/robot_models/reports/media")
media = reports / "robot-models-metrics-api-proposals-2026-11-18"
media.mkdir(parents=True, exist_ok=True)

view = can11_worst.view(title="CAN 11 · every stop at the tip", threshold=TIP_STOP)
view.hint
figure, axes = _viz.plot(view, path=media / "r2-1-tip-stops.png")
payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
payload["summary"]
calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
len(calls)
_viz.write_rrd(calls, media / "r2-1-tip-stops.rrd")

# a new metric: one constructor call, then the same three verbs
velocity_error = TrackingError("velocity error RMS", statistic="rms", field="velocities",
                               while_moving=True, unit="deg/s", scale=DEGREES)
velocity_error.update(run.target, run.measured)
velocity_error.compute()
'''),
    ("other inputs: repeats and a calibration", "Not every metric reads one recording: Repeatability takes one "
     "whole repeat per update(), PerformanceGain the held-out runs played with nominal and with fitted values.", '''
cell_runs = [play for play in real_runs if play.station == "can11" and play.rung == 0.9]
repeats = [play for play in cell_runs if play.labels["kind"] == "repeat"]
len(repeats)

repeatability = Repeatability("tip repeatability", k=3.0, unit="mm", scale=mm_at_tip)
for play in repeats:
    repeatability.update(play.target, play.measured)       # one whole repeat per update
repeatability.compute()

# the sealed friction ladder: held-out runs with datasheet values, then fitted ones
residual = TrackingError("feed-forward residual RMS", statistic="rms", field="effort",
                         across_runs="pooled", unit="N·m")
gain = PerformanceGain(residual)
gain.update(calibration.held_out_runs("nominal"), calibration.held_out_runs("fitted"))
gain.nominal, gain.fitted
gain.compute()
'''),
]


def main() -> None:
    from _session import Session

    session = Session({"__name__": "__main__"})
    session.cell(*GLANCE, kind="glance")
    session.namespace = {"__name__": "__main__"}        # the session's own fresh page, apart from the glance
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    main()
