"""v3 · Signals and aliases: the shared scenario as one REPL session on round 8's real runs.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v3_signals/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs the viz extra). ``run()`` is what the
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
from robot_msgs_metrics import (Alias, CodeShare, CrossCorrelationLag, HeldOutPair, PerformanceGain, Plan,  # noqa: E402
                                Recording, Repeat, Repeatability, RootMeanSquare, StopStatistic, aggregate, iqm)
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


CELLS = [
    ("a part's metrics", "robot_msgs.metrics holds only signal math; the part's catalog gives it meaning: an alias is "
     "a name, a signal, a display unit, how runs combine. The servo and the arm compile to one plan over one joint.", '''
print(INTEGRATED_SERVO.table(), "\\n  models:", INTEGRATED_SERVO.models)
arm = pendulum_arm(tip_length_m=TIPS["can11"])
print(arm.table())
print(RIGID_BODY_CHAIN.table())
print(Plan.compile(arm.metrics, arm.signals))
card = arm.metrics[7].card()
{key: card[key] for key in ("name", "base", "config", "unit", "description", "references", "value_type")}
'''),
    ("one recording, offline and live", "Live, the two topics arrive message by message and the aligner holds each "
     "reply until the next command knot: the plan at the reply's instant, 1 tick late, identical to offline.", '''
recording = Recording.from_run(fixture.pendulum_runs("real")[32])     # CAN 11, rung 0.9, repeat
print({topic: len(stream) for topic, stream in recording.topics.items()}, "messages")
offline, live, plan = live_vs_offline(recording)
named = {alias.name: alias for alias in arm.metrics}
for name in ("tracking RMS", "tracking peak", "tip stop error", "tip stop worst", "tip settle", "overshoot", "lag", "stops"):
    print(f"{name:<15} offline {display(named[name], offline[name]):10.6f}   live {display(named[name], live[name]):10.6f}   {named[name].unit}")
print("the aligner held at most", plan.aligners["joint"].max_held, "reply waiting for its next knot")
print(plan)
'''),
    ("runs to a deployment, gates applied", "Each rung's aliases across its runs by their own rule, rliable-style "
     "intervals beside the point estimates, then the catalog's checks.", '''
real = score("real")
judged = verdicts(real[1])
mark = {True: "✓", False: "✗", None: "?"}
for station in ("can11", "can12"):
    for rung in RUNGS:
        v, group = real[1][(station, rung)], real[2][(station, rung)]
        rms = aggregate(group, ARMS[station].metrics[0])
        print(f"{station}@{rung:<4g} tracking RMS {str(rms):<22} peak {v['tracking peak']:.2f}° · tip stop {v['tip stop error']:.2f} (worst {v['tip stop worst']:.2f}) mm · "
              + " ".join(mark[judged["cells"][(station, rung)][n]] for n in VERDICTS))
print({name: sum(bool(c[name]) for c in judged["cells"].values()) for name in VERDICTS}, "of 36 ·", judged["highest"])
print("CAN 12 at 1.0, per-play RMS:", [round(r.values["tracking RMS"] * DEGREES, 2) for r in real[2][("can12", 1.0)]], "IQM", round(iqm([r.values["tracking RMS"] * DEGREES for r in real[2][("can12", 1.0)]]), 3))
'''),
    ("the twin's output (sim vs target)", "The same plan on the twin's answers.", '''
sim = score("sim")
for rung in RUNGS:
    r, s = real[1][("can11", rung)], sim[1][("can11", rung)]
    print(f"CAN 11 @{rung:.1f}: tracking RMS real {r['tracking RMS']:.3f}° sim {s['tracking RMS']:.3f}° · tip stop real {r['tip stop error']:.3f} sim {s['tip stop error']:.3f} mm")
print("sim tip stop (median):", sum(bool(c["tip stop (median)"]) for c in verdicts(sim[1])["cells"].values()), "of 36")
'''),
    ("the chain, a view, a new metric", "The five-bar's catalog: RootMeanSquare on joints and on the tool. A view of "
     "every CAN 11 stop to matplotlib, W&B, Rerun; a new alias over an existing base.", '''
for key, values in chain_values().items():
    print(f"{key:<34} " + " · ".join(f"{name} {value:.3f}" for name, value in values.items()))
import _viz
worst = next(a for a in ARMS["can11"].metrics if a.name == "tip stop worst")
stops = [item * worst.factor for rung in RUNGS for r in real[2][("can11", rung)] for item in r.items["tip stop worst"]]
groups = [f"{rung:g}" for rung in RUNGS for r in real[2][("can11", rung)] for _ in r.items["tip stop worst"]]
shown = _viz.View(title="CAN 11 · every stop at the tip, by rung", unit="mm", hint="distribution", x=list(range(len(stops))), y=stops, group=groups, target=1.0, target_label="Zach's 1 mm", x_label="stop", y_label="mm")
try:
    _viz.plot(shown, path=MEDIA / "v3-tip-stops.png"); print("plot ->", MEDIA / "v3-tip-stops.png")
except ImportError:
    print("plot: matplotlib not installed (the viz extra)")
print("W&B:", _viz.wandb_payload(shown, "can11/tip_stop_mm")["summary"])
print("Rerun ->", _viz.write_rrd(_viz.rerun_calls(shown, "metrics/can11/tip_stop_mm"), MEDIA / "v3-tip-stops.rrd"))
faster = Plan.compile(arm.metrics + (VELOCITY_ERROR,), arm.signals)
print(faster, "·", f"{display(VELOCITY_ERROR, faster.run(recording).values['velocity error RMS']):.3f} deg/s")
'''),
]


CELLS.append(("other inputs: repeats and a calibration", "Each class declares what update() takes; the Plan's "
    "AlignedPair is only the common case. The sealed ladder's streams share a clock, so its signal says same_clock.", '''
print({cls.__name__: cls.Input for cls in (RootMeanSquare, StopStatistic, CrossCorrelationLag, Repeatability, PerformanceGain)})
rp, repeats = repeatability("can11", 0.9)
print(f"CAN 11 @0.9: RP (k=3) over {repeats} real repeats: {rp.compute() * 1e3 * TIPS['can11']:.3f} mm at the tip")
outcome = calibration.outcome()                       # the record: a robot_msgs message, robot_msgs.calibration
gain = performance_gain()
print(f"PerformanceGain({FF_RESIDUAL.name!r}): nominal {gain.sides[0]:.4f}, fitted {gain.sides[1]:.4f}, gain {gain.compute():.4f} N·m")
'''))


def main() -> None:
    session = Session(dict(globals()))
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    MEDIA.mkdir(parents=True, exist_ok=True)
    main()
