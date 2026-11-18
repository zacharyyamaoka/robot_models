"""r2-4 · Checklist with targets: round 2's shared scenario on round 8's real runs, recorded as a REPL session.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/r2-4_checklist/scenario.py

Writes scenario.out.txt and session.json beside this file: GLANCE runs in a fresh ``{"__name__": "__main__"}``, the
session CELLS in a second fresh one, so every name on the page is imported or defined on the page. ``run()`` is what
the round's frozen parity check calls: the same checklists, no transcript.
"""

from __future__ import annotations

import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROUND1 = HERE.parent.parent / "metrics-api-2026-11-17"
for folder in (str(ROUND1), str(HERE.parent), str(HERE)):
    if folder not in sys.path:
        sys.path.insert(0, folder)

import calibration  # noqa: E402
import fixture  # noqa: E402
from checklist import Card, Check, in_chunks, performance_gain, repeatability, rms_tracking_error  # noqa: E402
from parts import INTEGRATED_SERVO, PENDULUM_ARM, RIGID_CHAIN  # noqa: E402
from rung_ladder import highest_passing_rung  # noqa: E402

import robot_metrics  # noqa: E402  (on the path through checklist: robot_metrics 0.1.0, read-only)

TRACKS = ["tracking RMS", "tracking peak", "limiter share", "faulted runs"]
MEDIAN_STOP = ["median tip stop", "tip settle"]      # Zach's 1 mm on the median stop, settled within 0.3 s
WORST_STOP = ["worst tip stop"]                      # the same 1 mm on every stop (proposed)


def run() -> dict:
    arm = PENDULUM_ARM.metrics
    real, sim = fixture.pendulum_runs("real"), fixture.pendulum_runs("sim")
    real_plays = arm.measure(real, by=["run"])
    sim_plays = arm.measure(sim, by=["run"])
    real_report = arm.check(real, by=["station", "rung"])
    sim_rungs = arm.measure(sim, by=["station", "rung"])

    def play_rows(measured) -> dict:
        return {key[0]: {"tracking_rms_deg": row["tracking RMS"], "tracking_peak_deg": row["tracking peak"],
                         "stops": int(row["stops"]), "overshoot_deg": row["overshoot"], "lag_ms": row["lag"]}
                for key, row in measured.items()}

    def rung_rows(measured) -> dict:
        return {f"{station}@{rung:g}": {
            "tracking_rms_deg": row["tracking RMS"], "tracking_peak_deg": row["tracking peak"],
            "stop_tool_mm": row["median tip stop"], "stop_tool_max_mm": row["worst tip stop"],
            "settle_s": row["tip settle"], "limiter_share": row["limiter share"] / 100.0}
            for (station, rung), row in measured.items()}

    groups = real_report.measured.groups()
    gates = {name: [sum(real_report.passing(checks).values()), len(groups)]
             for name, checks in (("tracking", TRACKS), ("tool_stop_median", MEDIAN_STOP),
                                  ("tool_stop_worst", WORST_STOP))}

    chain = {}
    rows = {row.name: row.measure for row in RIGID_CHAIN.metrics.checks}
    for chain_run in fixture.chain_runs("real") + fixture.chain_runs("sim"):
        chain[chain_run.id] = {"positions/j2": rows["j2 RMS"](chain_run), "positions/c1": rows["c1 RMS"](chain_run),
                               "tool_position_rms_m": rows["tool RMS"](chain_run),
                               "tool_position_max_m": rows["tool worst"](chain_run)}

    recording = real[32]
    live = arm.live()
    for chunk in in_chunks(recording, rows=7):
        live.update(chunk)
    live_values, offline = arm.measure(live), arm.measure(recording)
    pairs = {"tracking_rms_deg": "tracking RMS", "tracking_peak_deg": "tracking peak", "stops": "stops",
             "stop_tool_max_mm": "worst tip stop", "overshoot_deg": "overshoot"}

    velocity = Card("added", [Check("velocity RMS", measure=rms_tracking_error(of="velocity"), unit="deg/s")])
    collection = arm.collection(recording.tip_length_m)
    repeats = [play for play in real if play.station == "can11" and play.rung == 0.9 and play.labels["kind"] == "repeat"]
    friction_fit = performance_gain(rms_tracking_error(of="torque", while_moving=False))
    nominal, fitted = friction_fit.sides(calibration.held_out_runs("nominal"), calibration.held_out_runs("fitted"))
    stock = (robot_metrics.ErrorStat, robot_metrics.StopStat, robot_metrics.TimeOffset, robot_metrics.CodeShare,
             robot_metrics.Repeatability, robot_metrics.PerformanceGain)
    return {
        "real_plays": play_rows(real_plays),
        "real_rungs": rung_rows(real_report.measured),
        "sim_plays": {play_id: {"tracking_rms_deg": row["tracking_rms_deg"]}
                      for play_id, row in play_rows(sim_plays).items()},
        "sim_rungs": rung_rows(sim_rungs),
        "highest_rung": highest_passing_rung(real_report, TRACKS + MEDIAN_STOP),
        "gates": gates,
        "chain": chain,
        "live": {"recording": recording.id,
                 "values": {key: [live_values[name], offline[name]] for key, name in pairs.items()}},
        "metric_cards": [arm["worst tip stop"].card(), arm["faulted runs"].card()],
        "declared": {"integrated_servo": [row.name for row in INTEGRATED_SERVO.metrics.checks],
                     "rigid_body_chain": [row.name for row in RIGID_CHAIN.metrics.checks]},
        "added": {"name": "velocity RMS",
                  "values": {key[0]: row["velocity RMS"] for key, row in velocity.measure(real, by=["run"]).items()}},
        "aliases": [{"name": name, "base": type(alias.metric).__name__, "config": alias.metric.config(),
                     "unit": alias.unit, "signal": "/".join(alias.signal)} for name, alias in collection.aliases.items()],
        "dedup": {"aliases": len(collection.aliases), "computed": collection.computations},
        "inputs": {cls.__name__: cls.Input.__name__ for cls in stock},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": repeatability(k=3.0)(repeats),
                          "repeats": len(repeats)},
        "performance_gain": {"alias": "feed-forward residual RMS", "nominal": nominal, "fitted": fitted,
                             "gain": nominal - fitted},
    }


# WHY eight blank-line groups, each under a one-line comment: round 2's story, step for step, so the five glances
# compare row by row (contract: load, metrics, offline, live, equal, a table, the gate, its verdict).
GLANCE = ("API at a glance", "A checklist: each row is a check's name, how it is measured, its unit, its target and "
          "who set it. card.check() prints pass / fail per row; card.measure() gives the numbers alone.", '''
# load the real runs: round 8 of the 1-DOF test rack, 252 recordings (round 1's fixture.py)
from fixture import pendulum_runs
from checklist import Card, Check, in_chunks
from checklist import rms_tracking_error, median_stop_error, worst_stop_error, stop_count
runs = pendulum_runs("real")
len(runs)

# the pendulum arm's checklist: what is measured, its unit, its target and who set it
card = Card("pendulum arm", [
    Check("tracking RMS", measure=rms_tracking_error(), unit="deg",
          target="<= 0.5", owner="AI agent"),
    Check("median tip stop", measure=median_stop_error(at="tip"), unit="mm",
          target="<= 1.0", owner="Zach 2026-11-11"),
    Check("worst tip stop", measure=worst_stop_error(at="tip"), unit="mm",
          target="<= 1.0", owner="Zach 2026-11-11"),
    Check("stops", measure=stop_count(), unit="stops"),
])
print(card)

# one recording, offline: CAN 11 at rung 0.9, a repeat
run = runs[32]
card.check(run)

# the same recording, live: fed 7 rows at a time, as they would arrive
live = card.live()                  # keeps running sums and the stops so far, not the rows
for chunk in in_chunks(run, rows=7):
    live.update(chunk)
card.check(live)

# equal: live and offline give the same numbers
card.measure(live) == card.measure(run)

# many runs into a table by station and rung: CAN 11's six rungs
can11 = [run for run in runs if run.station == "can11"]
card.measure(can11, by=["station", "rung"])

# Zach's 1 mm tip-stop gate: the target and its owner sit on the row, not in the measure
print(card["worst tip stop"])

# its verdict per station and rung: the median tip stop passes everywhere, the worst does not
card.check(runs, by=["station", "rung"])
''')

MEDIA = "/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-18"

CELLS = [
    ("setup", "Imports first, then the real data: round 8's 252 real plays, the twin's answer to each, and the "
     "five-bar's speed ladder (real and sim). Every later cell builds only on these names.", '''
# the data: round 1's fixture reads round 8 (real), the twin's answers (sim) and the five-bar ladder
import pathlib
from fixture import pendulum_runs, chain_runs
import calibration
import _viz
from checklist import Card, Check, in_chunks, performance_gain, repeatability
from checklist import rms_tracking_error
from parts import INTEGRATED_SERVO, PENDULUM_ARM, RIGID_CHAIN
from rung_ladder import highest_passing_rung

real = pendulum_runs("real")
sim = pendulum_runs("sim")
five_bar = chain_runs("real") + chain_runs("sim")
len(real), len(sim), len(five_bar)
'''),
    ("a part's metrics, beside its models", "Each part type in parts.py holds its models (robot_models) and its "
     "checklist side by side. The pendulum arm's checklist is the servo's rows plus three tip rows.", '''
INTEGRATED_SERVO.models.keys()          # robot_models' model family for the servo

print(INTEGRATED_SERVO.metrics)

arm = PENDULUM_ARM.metrics
print(arm)

print(RIGID_CHAIN.metrics)

arm["worst tip stop"].card()            # Zach's Metric fields for one row
'''),
    ("one recording, offline and live", "The same checklist on one recording, whole and fed in chunks. A stop "
     "that straddles a chunk boundary is still found whole: chunks of one row give the same numbers.", '''
run = real[32]                          # CAN 11, rung 0.9, a repeat
arm.check(run)

live = arm.live()
for chunk in in_chunks(run, rows=7):
    live.update(chunk)
arm.measure(live) == arm.measure(run)
print(live.kept())                      # what the running measure holds between chunks

one_row = arm.live()                    # every chunk boundary falls inside a move or a stop
for chunk in in_chunks(run, rows=1):
    one_row.update(chunk)
whole, by_row = arm.measure(run), arm.measure(one_row)
by_row["stops"], by_row["worst tip stop"] == whole["worst tip stop"]
max(abs(by_row[name] - whole[name]) for name in whole)     # float round-off in the sums only
'''),
    ("runs to a deployment, the targets applied", "Every station and rung of round 8 against each row's target; "
     "the highest rung each station climbs; then the stations, and the deployment as all of them.", '''
by_rung = arm.check(real, by=["station", "rung"])
by_rung

tracks = ["tracking RMS", "tracking peak", "limiter share", "faulted runs"]
median_stop = ["median tip stop", "tip settle"]
highest_passing_rung(by_rung, tracks + median_stop)

by_station = arm.check(real, by=["station"])
by_station
deployment_passes = all(by_station.passing(tracks + median_stop).values())
deployment_passes
'''),
    ("the twin's output (sim vs target)", "The same checklist on the twin's answer to each of the 252 plays.", '''
arm.check(sim, by=["station", "rung"])

tip_stops = Card("tip stops", [arm["median tip stop"], arm["worst tip stop"]])
can11_sim = [run for run in sim if run.station == "can11"]
tip_stops.measure(can11_sim, by=["source", "rung"])
'''),
    ("the five-bar, a view, a new row", "The rigid chain through the same rows; every CAN 11 stop at the tip as "
     "a matplotlib plot, a W&B payload and a Rerun recording; a new metric is one more row in the list.", f'''
RIGID_CHAIN.metrics.measure(five_bar, by=["source", "rung"])

can11 = [run for run in real if run.station == "can11"]
title = "CAN 11 · every stop at the tip, by rung"
view = arm.view("worst tip stop", can11, by="rung", title=title)
len(view.y), view.hint

media = pathlib.Path("{MEDIA}")
media.mkdir(parents=True, exist_ok=True)
figure, axes = _viz.plot(view, path=media / "r2-4-tip-stops.png")
payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
payload["summary"]
calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
_viz.write_rrd(calls, media / "r2-4-tip-stops.rrd")

velocity = Check("velocity RMS", measure=rms_tracking_error(of="velocity"), unit="deg/s")
with_velocity = Card("pendulum arm", arm.checks + [velocity])
with_velocity.measure(run)["velocity RMS"]
'''),
    ("other inputs: repeats and a calibration", "A row can measure something other than one recording: the "
     "repeats of one command (ISO 9283 repeatability), or held-out runs played with nominal and fitted values.", '''
repeats = [run for run in can11 if run.rung == 0.9 and run.labels["kind"] == "repeat"]
len(repeats)
repeat_card = Card("CAN 11 at rung 0.9", [
    Check("tip repeatability", measure=repeatability(k=3, at="tip"), unit="mm"),
])
repeat_card.check(repeats)

friction_fit = performance_gain(rms_tracking_error(of="torque", while_moving=False))
nominal = calibration.held_out_runs("nominal")
fitted = calibration.held_out_runs("fitted")
friction_fit.sides(nominal, fitted)     # torque left to the feedback, N·m: nominal, fitted
friction_fit(nominal, fitted)           # N·m the fitted friction model saves
'''),
]


def main() -> None:
    from _session import Session

    session = Session({"__name__": "__main__"})
    session.cell(*GLANCE, kind="glance")
    session.namespace = {"__name__": "__main__"}     # WHY a second fresh dict: the session imports its own names
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    main()
