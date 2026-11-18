"""v2 · Family metrics: the shared scenario as one REPL session on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v2_family_metrics/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs the viz extra: matplotlib, rerun-sdk). ``run()`` is what the frozen check calls: the same API, no transcript.
"""

from __future__ import annotations

import math
import pathlib
import sys
HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

import fixture  # noqa: E402
from _session import Session  # noqa: E402
from families import (DEPLOYMENT, METRICS_BY_TYPE, RIGID_BODY_CHAIN_METRICS, RUNGS, SERVO_SIGNALS,  # noqa: E402
                      FeedbackShare, pendulum_arm)
from robot_models import IntegratedServo  # noqa: E402
import calibration  # noqa: E402
from robot_models_metrics import (DEGREES, MEMO, STOP_PASSES, Alias, CodeShare, Dwell, Episode, ErrorStat,  # noqa: E402
                                  Gate, HeldOutPair, Metrics, PerformanceGain, Repeat, Repeatability, TimeOffset,
                                  at_most, view)

MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
TIPS = {station: facts["tip_length_m"] for station, facts in fixture.stations().items()}
ARMS = {station: pendulum_arm(tip) for station, tip in TIPS.items()}
FF_RESIDUAL = Alias("feed-forward residual RMS", ErrorStat(statistic="rms", field="effort", across_runs="pool"),
                    "joint", "N·m", 1.0)
VELOCITY_ERROR = Alias("velocity error RMS", ErrorStat(statistic="rms", rows="moving", field="velocities"), "joint",
                       "deg/s", DEGREES)


def episodes(source: str) -> list[Episode]:
    return [Episode.from_run(run) for run in fixture.pendulum_runs(source)]


def score(source: str) -> tuple[dict, dict]:
    """Each play its own Metrics; each rung ONE Metrics fed its 7 plays (the dataset is in the object)."""

    plays, rungs = {}, {}
    for episode in episodes(source):
        station, rung = episode.labels["station"], episode.labels["rung"]
        plays[episode.id] = {**ARMS[station](episode), "faults": int(episode.faulted)}
        collection, faults = rungs.get((station, rung), (ARMS[station].clone(), 0))
        collection.update(episode)
        collection.end_run()
        rungs[(station, rung)] = (collection, faults + int(episode.faulted))
    return plays, {key: {**collection.compute(), "faults": faults} for key, (collection, faults) in rungs.items()}


def judged(rungs: dict) -> dict:
    cells = {key: DEPLOYMENT.judge(values) for key, values in rungs.items()}
    highest = {}
    for station in sorted({station for station, _ in rungs}):
        reached = 0.0
        for rung in RUNGS:
            cell = cells.get((station, rung), {})
            if not (cell.get("tracks") and cell.get("tip stop (median)")):
                break
            reached = rung
        highest[station] = reached
    return {"cells": cells, "highest": highest}


def summary(rungs: dict, verdicts: dict, stations: tuple[str, ...]) -> str:
    mark = {True: "✓", False: "✗", None: "?"}
    lines = [f"{'cell':<10}{'RMS°':>7}{'peak°':>7}{'tip stop mm (worst)':>21}{'oversh°':>9}{'lag ms':>8}   "
             + " · ".join(DEPLOYMENT.verdicts)]
    for (station, rung), values in sorted(rungs.items()):
        if station not in stations:
            continue
        cell = verdicts["cells"][(station, rung)]
        lines.append(f"{station}@{rung:<4g}{values['tracking RMS']:>7.2f}{values['tracking peak']:>7.2f}"
                     f"{values['tip stop error']:>13.2f} ({values['tip stop worst']:.2f})"
                     f"{values['overshoot']:>9.3f}{values['lag']:>8.2f}   "
                     + "        ".join(mark[cell[name]] for name in DEPLOYMENT.verdicts))
    for name, gates in DEPLOYMENT.verdicts.items():
        passed = sum(bool(cell[name]) for cell in verdicts["cells"].values())
        lines.append(f"{name}: {passed} of {len(verdicts['cells'])} · " + "; ".join(str(gate) for gate in gates))
    lines.append("highest rung tracked: " + ", ".join(f"{s} {f:g}" for s, f in verdicts["highest"].items()))
    return "\n".join(lines)


def chain_values() -> dict:
    return {run.id: RIGID_BODY_CHAIN_METRICS(Episode.from_run(run))
            for source in ("real", "sim") for run in fixture.chain_runs(source)}


def repeatability(station: str = "can11", rung: float = 0.9) -> tuple[Repeatability, int]:
    metric = Repeatability(k=3.0)
    repeats = [run for run in fixture.pendulum_runs("real")
               if run.station == station and run.rung == rung and run.labels["kind"] == "repeat"]
    for run in repeats:
        metric.update(Repeat((station, rung), run.target, run.measured))
    return metric, len(repeats)


def performance_gain() -> PerformanceGain:
    gain = PerformanceGain(FF_RESIDUAL, SERVO_SIGNALS)
    gain.update(HeldOutPair(tuple(Episode.from_run(r) for r in calibration.held_out_runs("nominal")),
                            tuple(Episode.from_run(r) for r in calibration.held_out_runs("fitted"))))
    return gain


def live_vs_offline(episode: Episode, rows: int = 7) -> tuple[dict, dict]:
    metrics = ARMS[episode.labels["station"]]
    offline = metrics(episode)
    live = metrics.clone()
    for chunk in episode.chunks(rows):
        live.update(chunk)
    live.end_run()
    return offline, live.compute()


def check_dict(real: tuple, sim: tuple, chain: dict, live: dict, dedup: dict) -> dict:
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
    verdicts = judged(real[1])
    gates = {key: [sum(bool(c[name]) for c in verdicts["cells"].values()), len(verdicts["cells"])]
             for key, name in (("tracking", "tracks"), ("tool_stop_median", "tip stop (median)"),
                               ("tool_stop_worst", "tip stop (every stop)"))}
    added = Metrics(VELOCITY_ERROR, signals=SERVO_SIGNALS)
    arm = ARMS["can11"]
    return {
        "real_plays": play_rows(real[0]), "real_rungs": rung_rows(real[1]),
        "sim_plays": {k: {"tracking_rms_deg": v["tracking_rms_deg"]} for k, v in play_rows(sim[0]).items()},
        "sim_rungs": rung_rows(sim[1]), "highest_rung": verdicts["highest"], "gates": gates,
        "chain": {k: {"positions/j2": v["j2 tracking RMS"] / DEGREES, "positions/c1": v["c1 tracking RMS"] / DEGREES,
                      "tool_position_rms_m": v["tool tracking RMS"] / 1e3, "tool_position_max_m": v["tool worst"] / 1e3}
                  for k, v in chain.items()},
        "live": live,
        "metric_cards": [a.card() for a in arm.aliases.values() if a.name in ("tip stop error", "all stops settled")],
        "declared": {name: list(metrics.aliases) for name, metrics in METRICS_BY_TYPE.items()},
        "added": {"name": VELOCITY_ERROR.name,
                  "values": {e.id: added(e)[VELOCITY_ERROR.name] for e in episodes("real")}},
        "aliases": [a.card() for a in arm.aliases.values()],
        "dedup": dedup,
        "inputs": {cls.__name__: cls.Input for cls in (ErrorStat, Dwell, TimeOffset, CodeShare, Repeatability,
                                                       PerformanceGain)},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": rp.compute(), "repeats": repeats},
        "performance_gain": {"alias": FF_RESIDUAL.name, "nominal": gain.sides[0], "fitted": gain.sides[1],
                             "gain": gain.compute()},
    }


def dedup_on(episode: Episode) -> dict:
    """The arm's 12 aliases on one recording after the servo's 6 ran on it: how many computations really ran."""

    MEMO.clear()
    STOP_PASSES.clear()
    IntegratedServo.metrics(episode)
    servo_misses = MEMO.misses
    ARMS[episode.labels["station"]](episode)
    return {"aliases": len(ARMS["can11"].aliases), "computed": MEMO.misses - servo_misses,
            "memo_hits": MEMO.hits, "stop_passes": STOP_PASSES.misses}


def run() -> dict:
    episode = episodes("real")[0]
    offline, live = live_vs_offline(episode)
    pairs = {"tracking_rms_deg": "tracking RMS", "tracking_peak_deg": "tracking peak", "stops": "stops",
             "stop_tool_max_mm": "tip stop worst", "overshoot_deg": "overshoot"}
    live_block = {"recording": episode.id, "values": {key: [live[name], offline[name]] for key, name in pairs.items()}}
    dedup = dedup_on(episode)
    return check_dict(score("real"), score("sim"), chain_values(), live_block, dedup)


def plays_of(station: str, rung: float) -> list[Episode]:
    """One rung cell's real plays, as the roll-up groups them."""

    return [episode for episode in episodes("real")
            if episode.labels["station"] == station and episode.labels["rung"] == rung]


# WHY the same seven headings in every variant: the five glances are compared side by side, row by row.
GLANCE = ("API at a glance", "The design's shape before any detail: a metric, an alias, a part type's .metrics, "
          "live, offline, a gate kept outside compute(), and a rung summarised across runs.", '''
# a metric: one base, its constructor is the config (a buffer + a functional twin)
stop_worst = Dwell(quantity="error", statistic="max")

# an alias: a name, the signal it reads, a display unit
tip_stop_worst = Alias("tip stop worst", stop_worst, "joint", unit="mm", factor=339.0)
tip_stop_worst

# a part type's metrics, beside its models: both on the ModelFamily
IntegratedServo
IntegratedServo.metrics
arm = pendulum_arm(tip_length_m=0.339)
list(arm.aliases)

# live: chunks appended to the buffer, compute() re-reads it
episode = Episode.from_run(fixture.pendulum_runs("real")[32])
live = arm.clone()
for chunk in episode.chunks(rows=7):
    live.update(chunk)
live.end_run()
live.compute()["tip stop worst"]

# offline: call the family's Metrics on the whole episode
offline = arm(episode)
offline["tip stop worst"]

# a gate: an alias against a robot_msgs Value that carries its source
within_1mm = Gate("tip stop worst", at_most(1.0, "mm", "Zach 2026-11-11"))
within_1mm.check(offline)

# across runs: one Metrics fed every play of the rung, run after run
rung = arm.clone()
for play in plays_of("can11", rung=0.9):
    rung.update(play)
    rung.end_run()
rung_values = rung.compute()
rung_values["tip stop worst"]
within_1mm.check(rung_values)
''')

CELLS = [
    ("a part's metrics", "The family IS the part type's models; its metrics are declared beside them. The arm is the "
     "servo's Metrics plus tip aliases; the chain's are keyed by type id until it has a family.", '''
IntegratedServo                                  # .models: the family itself
print(IntegratedServo.metrics.table("IntegratedServo"))

arm = pendulum_arm(tip_length_m=TIPS["can11"])    # CAN 11: the 339 mm heavy pendulum
print(arm.table("1-DOF pendulum arm (CAN 11, 339 mm)"))

chain_metrics = METRICS_BY_TYPE["rigid_body_chain"]
print(chain_metrics.table("rigid_body_chain"))

arm.aliases["tip stop error"].card()
'''),
    ("one recording, offline and live", "update() appends chunks and compute() runs the functional twin over the "
     "buffer, so live is offline by construction; the memo shows the arm's aliases that the servo already computed.", '''
run = fixture.pendulum_runs("real")[32]           # CAN 11, rung 0.9, a repeat
episode = Episode.from_run(run)

offline = arm(episode)

live = arm.clone()
for number, chunk in enumerate(episode.chunks(rows=7)):
    live.update(chunk)
    if number == 40:
        so_far = live.compute()["tip stop worst"]
        print(f"after 280 rows: worst stop so far {so_far:.3f} mm")
live.end_run()
live_values = live.compute()

names = ["tracking RMS", "tracking peak", "tip stop error", "tip stop worst",
         "tip settle", "overshoot", "lag", "stops"]
for name in names:
    print(f"{name:<15} offline {offline[name]:10.6f}   live {live_values[name]:10.6f}")

dedup_on(episode)

rms_config = ErrorStat.Config(statistic="rms", rows="moving")
one_run = [(episode.streams["target"], episode.streams["state"])]
ErrorStat.functional(rms_config, one_run) * DEGREES      # the functional twin, in degrees
'''),
    ("runs to a deployment, gates applied", "A rung is one Metrics object fed its plays; the deployment record holds the "
     "targets as robot_msgs Values with their source; verdicts read the computed values.", '''
real_plays, real_rungs = score("real")
len(real_plays), len(real_rungs)

verdicts = judged(real_rungs)
print(summary(real_rungs, verdicts, stations=("can11", "can12")))

every_stop = DEPLOYMENT.verdicts["tip stop (every stop)"]
every_stop
DEPLOYMENT.judge(real_rungs[("can11", 0.9)])
'''),
    ("the twin's output (sim vs target)", "The same Metrics on the twin's answer to every play.", '''
sim_plays, sim_rungs = score("sim")
sim_verdicts = judged(sim_rungs)

print("CAN 11 rung   RMS real °   RMS sim °   stop real mm   stop sim mm")
for rung in RUNGS:
    real_values = real_rungs[("can11", rung)]
    sim_values = sim_rungs[("can11", rung)]
    rms = (real_values["tracking RMS"], sim_values["tracking RMS"])
    stop = (real_values["tip stop error"], sim_values["tip stop error"])
    print(f"{rung:11.1f} {rms[0]:12.3f} {rms[1]:11.3f} {stop[0]:14.3f} {stop[1]:13.3f}")

sim_cells = sim_verdicts["cells"].values()
sum(bool(cell["tip stop (median)"]) for cell in sim_cells)
'''),
    ("the chain, a view, a new metric", "The five-bar's Metrics over the same ErrorStat base; a view of one alias to "
     "matplotlib, W&B and Rerun; a configured alias and a subclass added.", '''
chain = chain_values()
for run_id, values in chain.items():
    rounded = {name: round(value, 3) for name, value in values.items()}
    print(run_id, rounded)

station = ARMS["can11"].clone()
for play in episodes("real"):
    if play.labels["station"] == "can11":
        station.update(play)
        station.end_run()

import _viz
title = "CAN 11 · every stop at the tip, round 8"
shown = view(station, "tip stop worst", title, target=1.0)
_viz.plot(shown, path=MEDIA / "v2-tip-stops.png")

payload = _viz.wandb_payload(shown, "can11/tip_stop_mm")
payload["summary"]

calls = _viz.rerun_calls(shown, "metrics/can11/tip_stop_mm")
len(calls)
_viz.write_rrd(calls, MEDIA / "v2-tip-stops.rrd")

feedback_share = Alias("feedback share (model ff)", FeedbackShare(), "joint", "%", 100.0)
more = arm + (VELOCITY_ERROR, feedback_share)
more
more_values = more(episode)
more_values["velocity error RMS"]
more_values["feedback share (model ff)"]
'''),
    ("other inputs: repeats and a calibration", "Each class declares what update() takes; an Episode is only the "
     "common case. The gain reuses the alias by identity on the calibration's held-out runs.", '''
inputs = (ErrorStat, Dwell, TimeOffset, Repeatability, PerformanceGain)
{cls.__name__: cls.Input for cls in inputs}

repeatability_metric, repeats = repeatability("can11", 0.9)
repeats
tip_mm = 1e3 * TIPS["can11"]
repeatability_metric.compute() * tip_mm          # mm at the tip

outcome = calibration.outcome()                  # robot_models.metrics.CalibrationOutcome
outcome.held_out

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
