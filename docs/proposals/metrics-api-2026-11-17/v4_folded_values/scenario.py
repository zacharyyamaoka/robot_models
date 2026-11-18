"""v4 · Folded values: the shared scenario as one REPL session on round 8's real runs.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v4_folded_values/scenario.py

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


CELLS = [
    ("a part's metrics", "Each metric is a frozen config (hashable: its identity); the part lists aliases; equal "
     "identities share one state, and every dwell metric on a signal shares one stop pass.", '''
print(INTEGRATED_SERVO.table(), "\\n  models:", INTEGRATED_SERVO.models)
arm = pendulum_arm(tip_length_m=TIPS["can11"])
print(arm.table())
print(RIGID_BODY_CHAIN.table())
print("same identity:", ErrorStat(statistic="rms", rows="moving") == arm.metrics[6].metric, "· hash equal:", hash(ErrorStat(statistic="rms", rows="moving")) == hash(arm.metrics[0].metric))
card = arm.metrics[7].card()
{key: card[key] for key in ("name", "base", "config", "unit", "description", "references", "value_type", "live", "version")}
'''),
    ("one recording, offline and live", "Live is a fold over 7-row chunks of new state VALUES; offline the same fold "
     "once. Stream is the torchmetrics face over it.", '''
run = Run.from_fixture(fixture.pendulum_runs("real")[32])        # CAN 11, rung 0.9, repeat
offline, live = live_vs_offline(run)
for name in ("tracking RMS", "tracking peak", "tip stop error", "tip stop worst", "tip settle", "overshoot", "lag", "stops"):
    print(f"{name:<15} offline {offline[name]:10.6f}   live {live[name]:10.6f}   |diff| {abs(offline[name] - live[name]):.1e}")
stream = Stream(ErrorStat(statistic="rms", rows="moving"))       # update / compute / reset, as torchmetrics
for chunk in run.chunks(7):
    stream.update(chunk.streams["target"], chunk.streams["state"])
print("Stream:", stream.compute() * DEGREES, "deg · its state is a value:", stream.state[:2])
print("stored:", {"alias": "tip stop error", "value": offline["tip stop error"], "unit": "mm", **{k: arm.metrics[7].card()[k] for k in ("base", "config", "version")}, "provenance": run.provenance, "rrd": run.links["rrd"]})
'''),
    ("runs to a deployment, gates applied", "Strata are station x rung; each verdict is OpenSLO objectives judged by "
     "RobotDx's own Result. The same alias carries the median gate and the every-stop gate.", '''
real = score("real")
verdicts = judged(real[1])
mark = {"pass": "✓", "fail": "✗", "not_run": "?"}
for (station, rung), stratum in sorted(real[1].items()):
    if station in ("can11", "can12"):
        v = stratum["values"]
        print(f"{station}@{rung:<4g} RMS {v['tracking RMS']:.2f}° peak {v['tracking peak']:.2f}° · tip stop {v['tip stop error']:.2f} (worst {v['tip stop worst']:.2f}) mm · " + " ".join(mark[verdicts["cells"][(station, rung)][n]] for n in VERDICTS))
print({name: sum(c[name] == "pass" for c in verdicts["cells"].values()) for name in VERDICTS}, "of 36 ·", verdicts["highest"])
station = summarize(COLLECTIONS["can11"], [r for r in runs("real") if r.labels["station"] == "can11"], ("station",))[("can11",)]
result = Objective("tip stop error", "le", 1.0, share=0.99, on="items", owner="proposed").judge(COLLECTIONS["can11"], station)
print("RobotDx Result:", result.verdict, result.title, "·", result.message)
settled = sum(COLLECTIONS["can11"].value(state, "all stops settled") for state in station["states"])
print(f"CAN 11 runs with every stop settled: {settled} of {len(station['states'])}, Wilson 95 % {wilson(settled, len(station['states']))}")
'''),
    ("the twin's output (sim vs target)", "The same collections on the twin's answers.", '''
sim = score("sim")
for rung in RUNGS:
    r, s = real[1][("can11", rung)]["values"], sim[1][("can11", rung)]["values"]
    print(f"CAN 11 @{rung:.1f}: tracking RMS real {r['tracking RMS']:.3f}° sim {s['tracking RMS']:.3f}° · tip stop real {r['tip stop error']:.3f} sim {s['tip stop error']:.3f} mm")
print("sim tip stop (median):", sum(c["tip stop (median)"] == "pass" for c in judged(sim[1])["cells"].values()), "of 36")
'''),
    ("the chain, a view, a new metric", "The five-bar through the same ErrorStat; a view of every CAN 11 stop to "
     "matplotlib, W&B and Rerun; a new alias is one more frozen config.", '''
for key, values in chain_values().items():
    print(f"{key:<34} " + " · ".join(f"{name} {value:.3f}" for name, value in values.items()))
import _viz
stops, groups = [], []
for rung in RUNGS:
    items = COLLECTIONS["can11"].items(real[1][("can11", rung)]["merged"], "tip stop error")
    stops += items; groups += [f"{rung:g}"] * len(items)
shown = _viz.View(title="CAN 11 · every stop at the tip, by rung", unit="mm", hint="distribution", x=list(range(len(stops))), y=stops, group=groups, target=1.0, target_label="Zach's 1 mm", x_label="stop", y_label="mm")
try:
    _viz.plot(shown, path=MEDIA / "v4-tip-stops.png"); print("plot ->", MEDIA / "v4-tip-stops.png")
except ImportError:
    print("plot: matplotlib not installed (the viz extra)")
print("W&B:", _viz.wandb_payload(shown, "can11/tip_stop_mm")["summary"], "· Rerun ->", _viz.write_rrd(_viz.rerun_calls(shown, "metrics/can11/tip_stop_mm"), MEDIA / "v4-tip-stops.rrd"))
faster = Collection(arm.metrics + (VELOCITY_ERROR,))
print(f"velocity error RMS {faster.values(faster.fold(run))['velocity error RMS']:.3f} deg/s · states {faster.computations}")
'''),
    ("other inputs: repeats and a calibration", "Each class declares its Input; a Run is the common case. "
     "Repeatability folds one real repeat at a time; PerformanceGain folds the alias's metric over both sides.", '''
print({cls.__name__: cls.Input for cls in (ErrorStat, Dwell, TimeOffset, Repeatability, PerformanceGain)})
rp, rp_state, repeats = repeatability("can11", 0.9)
print(f"CAN 11 @0.9: RP (k=3) over {repeats} real repeats: {rp.compute(rp_state) * 1e3 * TIPS['can11']:.3f} mm at the tip")
outcome = calibration.outcome()                       # the record: robot_msgs (a CalibrationOutcome message)
gain, state = performance_gain()
print(f"PerformanceGain(metric={gain.metric!r}): nominal {state[0]:.4f}, fitted {state[1]:.4f}, gain {gain.compute(state):.4f} N·m")
'''),
]


def main() -> None:
    session = Session(dict(globals()))
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    MEDIA.mkdir(parents=True, exist_ok=True)
    main()
