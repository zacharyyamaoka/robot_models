"""v3 · Signals and aliases: the shared scenario as one REPL session on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v3_signals/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs the viz extra: matplotlib, rerun-sdk). ``run()`` is what the
frozen check calls: the same API, no transcript.
"""

from __future__ import annotations

import math
import pathlib
import sys
from collections import defaultdict

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

import fixture  # noqa: E402
from _session import Session  # noqa: E402
from catalog import (DEGREES, INTEGRATED_SERVO, RIGID_BODY_CHAIN, RUNGS, SERVO_SIGNALS, VERDICTS,  # noqa: E402
                     pendulum_arm)
import calibration  # noqa: E402
from robot_msgs_metrics import (Alias, Check, CodeShare, CrossCorrelationLag, HeldOutPair, Limits,  # noqa: E402
                                PerformanceGain, Plan, Recording, Repeat, Repeatability, RootMeanSquare,
                                StopStatistic, aggregate, iqm)
from robot_msgs_metrics.plan import across_runs, display  # noqa: E402

MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
TIPS = {station: facts["tip_length_m"] for station, facts in fixture.stations().items()}
ARMS = {station: pendulum_arm(tip) for station, tip in TIPS.items()}
PLANS = {station: Plan.compile(arm.metrics, arm.signals) for station, arm in ARMS.items()}
FF_RESIDUAL = Alias("feed-forward residual RMS", RootMeanSquare(field="effort"), "joint", "N·m", 1.0)
SAME_CLOCK = {"joint": ("/joint/command", "/joint/state", "same_clock")}   # the sealed ladder logs both on one clock
VELOCITY_ERROR = Alias("velocity error RMS", RootMeanSquare(rows="moving", field="velocities"), "joint", "deg/s",
                       DEGREES, "median")


def recordings(source: str) -> list[Recording]:
    return [Recording.from_run(run) for run in fixture.pendulum_runs(source)]


def score(source: str) -> tuple[dict, dict, dict]:
    """Per run: the station's plan run offline. Per rung: each alias across its runs (its own `across` rule)."""

    results, groups = {}, defaultdict(list)
    for recording in recordings(source):
        station = recording.labels["station"]
        result = PLANS[station].run(recording)
        results[recording.id] = result
        groups[(station, recording.labels["rung"])].append(result)
    named = {station: {alias.name: alias for alias in arm.metrics} for station, arm in ARMS.items()}
    plays = {key: {name: display(named[r.labels["station"]][name], v) for name, v in r.values.items()}
             | {"faults": int(r.faulted)} for key, r in results.items()}
    rungs = {key: {alias.name: across_runs(alias, group) for alias in ARMS[key[0]].metrics}
             | {"faults": sum(r.faulted for r in group)} for key, group in groups.items()}
    return plays, rungs, groups


def verdicts(rungs: dict) -> dict:
    cells = {}
    for (station, rung), values in rungs.items():
        checks = {check.alias: check for check in ARMS[station].checks}
        cells[(station, rung)] = {}
        for name, aliases in VERDICTS.items():
            outcome = [checks[alias].limits.contains(values[alias]) for alias in aliases]
            cells[(station, rung)][name] = False if False in outcome else (None if None in outcome else True)
    highest = {}
    for station in sorted({s for s, _ in rungs}):
        reached = 0.0
        for rung in RUNGS:
            cell = cells.get((station, rung), {})
            if not (cell.get("tracks") and cell.get("tip stop (median)")):
                break
            reached = rung
        highest[station] = reached
    return {"cells": cells, "highest": highest}


def chain_values() -> dict:
    plan = Plan.compile(RIGID_BODY_CHAIN.metrics, RIGID_BODY_CHAIN.signals)
    out = {}
    for source in ("real", "sim"):
        for run in fixture.chain_runs(source):
            result = plan.run(Recording.from_run(run))
            out[run.id] = {alias.name: display(alias, result.values[alias.name]) for alias in RIGID_BODY_CHAIN.metrics}
    return out


def repeatability(station: str = "can11", rung: float = 0.9) -> tuple[Repeatability, int]:
    metric = Repeatability(k=3.0)
    repeats = [run for run in fixture.pendulum_runs("real")
               if run.station == station and run.rung == rung and run.labels["kind"] == "repeat"]
    for run in repeats:
        metric.update(Repeat((station, rung), run.target, run.measured))
    return metric, len(repeats)


def performance_gain() -> PerformanceGain:
    def recording(run):
        return Recording(run.id, {"/joint/command": run.target, "/joint/state": run.measured}, dict(run.labels))

    gain = PerformanceGain(FF_RESIDUAL, SAME_CLOCK)
    gain.update(HeldOutPair(tuple(recording(r) for r in calibration.held_out_runs("nominal")),
                            tuple(recording(r) for r in calibration.held_out_runs("fitted"))))
    return gain


def live_vs_offline(recording: Recording) -> tuple[dict, dict, Plan]:
    station = recording.labels["station"]
    offline = PLANS[station].run(recording).values
    live = Plan.compile(ARMS[station].metrics, ARMS[station].signals)
    for _, topic, message in recording.messages():          # one message at a time, in arrival order
        live.push(topic, message)
    live.finish()
    return offline, live.result(recording).values, live


def check_dict(real: tuple, sim: tuple, chain: dict, live: dict, plan: Plan) -> dict:
    def play_rows(plays):
        return {k: {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                    "stops": int(v["stops"]), "overshoot_deg": v["overshoot"], "lag_ms": v["lag"]}
                for k, v in plays.items()}

    def rung_rows(rungs):
        return {f"{s}@{r:g}": {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                               "stop_tool_mm": v["tip stop error"], "stop_tool_max_mm": v["tip stop worst"],
                               "settle_s": v["tip settle"], "limiter_share": v["limiter share"] / 100.0}
                for (s, r), v in rungs.items()}

    rp, repeats = repeatability()
    gain = performance_gain()
    judged = verdicts(real[1])
    gates = {key: [sum(bool(c[name]) for c in judged["cells"].values()), len(judged["cells"])]
             for key, name in (("tracking", "tracks"), ("tool_stop_median", "tip stop (median)"),
                               ("tool_stop_worst", "tip stop (every stop)"))}
    added = Plan.compile((VELOCITY_ERROR,), SERVO_SIGNALS)
    arm = ARMS["can11"]
    return {
        "real_plays": play_rows(real[0]), "real_rungs": rung_rows(real[1]),
        "sim_plays": {k: {"tracking_rms_deg": v["tracking_rms_deg"]} for k, v in play_rows(sim[0]).items()},
        "sim_rungs": rung_rows(sim[1]), "highest_rung": judged["highest"], "gates": gates,
        "chain": {k: {"positions/j2": v["j2 tracking RMS"] / DEGREES, "positions/c1": v["c1 tracking RMS"] / DEGREES,
                      "tool_position_rms_m": v["tool tracking RMS"] / 1e3, "tool_position_max_m": v["tool worst"] / 1e3}
                  for k, v in chain.items()},
        "live": live,
        "metric_cards": [a.card() for a in arm.metrics if a.name in ("tip stop error", "all stops settled")],
        "declared": {"integrated_servo": [a.name for a in INTEGRATED_SERVO.metrics],
                     "rigid_body_chain": [a.name for a in RIGID_BODY_CHAIN.metrics]},
        "added": {"name": VELOCITY_ERROR.name,
                  "values": {r.id: display(VELOCITY_ERROR, added.run(r).values[VELOCITY_ERROR.name])
                             for r in recordings("real")}},
        "aliases": [a.card() for a in arm.metrics],
        "dedup": {"aliases": len(arm.metrics), "computed": plan.computations},
        "inputs": {cls.__name__: cls.Input for cls in (RootMeanSquare, StopStatistic, CrossCorrelationLag, CodeShare,
                                                       Repeatability, PerformanceGain)},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": rp.compute(), "repeats": repeats},
        "performance_gain": {"alias": FF_RESIDUAL.name, "nominal": gain.sides[0], "fitted": gain.sides[1],
                             "gain": gain.compute()},
    }


def run() -> dict:
    recording = recordings("real")[0]
    offline, live, plan = live_vs_offline(recording)
    arm = ARMS[recording.labels["station"]]
    alias = {a.name: a for a in arm.metrics}
    pairs = {"tracking_rms_deg": "tracking RMS", "tracking_peak_deg": "tracking peak", "stops": "stops",
             "stop_tool_max_mm": "tip stop worst", "overshoot_deg": "overshoot"}
    live_block = {"recording": recording.id, "values": {
        key: [display(alias[name], live[name]), display(alias[name], offline[name])] for key, name in pairs.items()}}
    return check_dict(score("real"), score("sim"), chain_values(), live_block, plan)


def plays_of(station: str, rung: float) -> list[Recording]:
    """One rung cell's real plays, as the roll-up groups them."""

    return [recording for recording in recordings("real")
            if recording.labels["station"] == station and recording.labels["rung"] == rung]


# WHY the same seven headings in every variant: the five glances are compared side by side, row by row.
GLANCE = ("API at a glance", "The design's shape before any detail: a metric, an alias, a part type's .metrics, "
          "live, offline, a gate kept outside compute(), and a rung summarised across runs.", '''
# a metric: pure signal math in robot_msgs, no part, display unit or threshold
stop_worst = StopStatistic(of="error", statistic="max")

# an alias: the part's catalog adds a name, signal, display unit and across-runs rule
tip_stop_worst = Alias("tip stop worst", stop_worst, "joint", "mm", 339.0, "pooled:max")
tip_stop_worst

# a part type's metrics, beside its models: an alias catalog, compiled to one Plan
INTEGRATED_SERVO.models
arm = pendulum_arm(tip_length_m=0.339)
plan = Plan.compile(arm.metrics, arm.signals)
plan

# live: raw topics message by message, the aligner pairs them as they arrive
recording = Recording.from_run(fixture.pendulum_runs("real")[32])
for stamp, topic, message in recording.messages():
    plan.push(topic, message)
plan.finish()
live = plan.result(recording)
display(tip_stop_worst, live.values["tip stop worst"])

# offline: the same plan, each topic pushed whole
offline = plan.run(recording)
play_worst = display(tip_stop_worst, offline.values["tip stop worst"])
play_worst

# a gate: a catalog Check row (alias, Limits, owner), read on the display value
within_1mm = Check("tip stop worst", Limits(high=1.0), owner="Zach 2026-11-11")
within_1mm.limits.contains(play_worst)

# across runs: the alias's own rule over the rung's results, a bootstrap CI beside it
rung_results = [plan.run(play) for play in plays_of("can11", rung=0.9)]
aggregate(rung_results, tip_stop_worst)
rung_worst = across_runs(tip_stop_worst, rung_results)
within_1mm.limits.contains(rung_worst)
''')

CELLS = [
    ("a part's metrics", "robot_msgs.metrics holds only signal math; the part's catalog gives it meaning: an alias is "
     "a name, a signal, a display unit, how runs combine. The servo and the arm compile to one plan over one joint.", '''
print(INTEGRATED_SERVO.table())
INTEGRATED_SERVO.models

arm = pendulum_arm(tip_length_m=TIPS["can11"])    # CAN 11: the 339 mm heavy pendulum
print(arm.table())

print(RIGID_BODY_CHAIN.table())

Plan.compile(arm.metrics, arm.signals)

aliases = {alias.name: alias for alias in arm.metrics}
aliases["tip stop error"].card()
'''),
    ("one recording, offline and live", "Live, the two topics arrive message by message and the aligner holds each "
     "reply until the next command knot: the plan at the reply's instant, 1 tick late, identical to offline.", '''
run = fixture.pendulum_runs("real")[32]           # CAN 11, rung 0.9, a repeat
recording = Recording.from_run(run)
{topic: len(stream) for topic, stream in recording.topics.items()}

offline = PLANS["can11"].run(recording).values

live_plan = Plan.compile(arm.metrics, arm.signals)
for stamp, topic, message in recording.messages():
    live_plan.push(topic, message)
live_plan.finish()
live = live_plan.result(recording).values

names = ["tracking RMS", "tracking peak", "tip stop error", "tip stop worst",
         "tip settle", "overshoot", "lag", "stops"]
for name in names:
    offline_value = display(aliases[name], offline[name])
    live_value = display(aliases[name], live[name])
    print(f"{name:<15} offline {offline_value:10.6f}   live {live_value:10.6f}")

live_plan.aligners["joint"].max_held     # replies held for their next command knot
live_plan
'''),
    ("runs to a deployment, gates applied", "Each rung's aliases across its runs by their own rule, rliable-style "
     "intervals beside the point estimates, then the catalog's checks.", '''
real_plays, real_rungs, real_groups = score("real")
len(real_plays), len(real_rungs)

judged = verdicts(real_rungs)
mark = {True: "✓", False: "✗", None: "?"}
tracking = aliases["tracking RMS"]                # the median of the plays' RMS
for station in ("can11", "can12"):
    for rung in RUNGS:
        values = real_rungs[(station, rung)]
        estimate = aggregate(real_groups[(station, rung)], tracking)
        cell = judged["cells"][(station, rung)]
        marks = " ".join(mark[cell[name]] for name in VERDICTS)
        stop = f"{values['tip stop error']:.2f} (worst {values['tip stop worst']:.2f}) mm"
        print(f"{station}@{rung:<4g} RMS {str(estimate):<22} tip stop {stop}  {marks}")

cells = judged["cells"].values()
{name: sum(bool(cell[name]) for cell in cells) for name in VERDICTS}
judged["highest"]

checks = {check.alias: check for check in ARMS["can11"].checks}
checks["tip stop worst"]
worst_mm = real_rungs[("can11", 0.9)]["tip stop worst"]
checks["tip stop worst"].limits.contains(worst_mm)

can12_top = real_groups[("can12", 1.0)]
per_play_rms = [result.values["tracking RMS"] * DEGREES for result in can12_top]
[round(value, 2) for value in per_play_rms]
round(iqm(per_play_rms), 3)                       # for display; the gate reads the median
'''),
    ("the twin's output (sim vs target)", "The same plan on the twin's answers.", '''
sim_plays, sim_rungs, sim_groups = score("sim")

print("CAN 11 rung   RMS real °   RMS sim °   stop real mm   stop sim mm")
for rung in RUNGS:
    real_values = real_rungs[("can11", rung)]
    sim_values = sim_rungs[("can11", rung)]
    rms = (real_values["tracking RMS"], sim_values["tracking RMS"])
    stop = (real_values["tip stop error"], sim_values["tip stop error"])
    print(f"{rung:11.1f} {rms[0]:12.3f} {rms[1]:11.3f} {stop[0]:14.3f} {stop[1]:13.3f}")

sim_cells = verdicts(sim_rungs)["cells"].values()
sum(bool(cell["tip stop (median)"]) for cell in sim_cells)
'''),
    ("the chain, a view, a new metric", "The five-bar's catalog: RootMeanSquare on joints and on the tool. A view of "
     "every CAN 11 stop to matplotlib, W&B, Rerun; a new alias over an existing base.", '''
chain = chain_values()
for run_id, values in chain.items():
    rounded = {name: round(value, 3) for name, value in values.items()}
    print(run_id, rounded)

worst = aliases["tip stop worst"]
stops, groups = [], []
for rung in RUNGS:
    for result in real_groups[("can11", rung)]:
        for item in result.items["tip stop worst"]:
            stops.append(item * worst.factor)
            groups.append(f"{rung:g}")
len(stops)

import _viz
title = "CAN 11 · every stop at the tip, by rung"
shown = _viz.View(title=title, unit="mm", hint="distribution",
                  x=list(range(len(stops))), y=stops, group=groups, target=1.0,
                  target_label="Zach's 1 mm", x_label="stop", y_label="mm")
_viz.plot(shown, path=MEDIA / "v3-tip-stops.png")

payload = _viz.wandb_payload(shown, "can11/tip_stop_mm")
payload["summary"]

calls = _viz.rerun_calls(shown, "metrics/can11/tip_stop_mm")
len(calls)
_viz.write_rrd(calls, MEDIA / "v3-tip-stops.rrd")

VELOCITY_ERROR
faster = Plan.compile(arm.metrics + (VELOCITY_ERROR,), arm.signals)
faster
velocity = faster.run(recording).values["velocity error RMS"]
display(VELOCITY_ERROR, velocity)
'''),
    ("other inputs: repeats and a calibration", "Each class declares what update() takes; the Plan's AlignedPair is "
     "only the common case. The sealed ladder's streams share a clock, so its signal says same_clock.", '''
inputs = (RootMeanSquare, StopStatistic, CrossCorrelationLag,
          Repeatability, PerformanceGain)
{cls.__name__: cls.Input for cls in inputs}

repeatability_metric, repeats = repeatability("can11", 0.9)
repeats
tip_mm = 1e3 * TIPS["can11"]
repeatability_metric.compute() * tip_mm          # mm at the tip

outcome = calibration.outcome()             # a robot_msgs.calibration message
outcome.held_out

SAME_CLOCK
gain = performance_gain()
FF_RESIDUAL.name
gain.sides                         # nominal, fitted: N·m on the held-out runs
gain.compute()
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
