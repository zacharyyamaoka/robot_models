"""v1 · robot_metrics: the shared scenario as one REPL session on round 8's real runs.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v1_robot_metrics/scenario.py

Writes scenario.out.txt and session.json beside this file. The view cell draws with matplotlib and logs Rerun when
they are importable (the optional extras); run it with them through
``uv run --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python <this file>``.
``run()`` is what the frozen check calls: the same API, no transcript.
"""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import replace

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]

import fixture  # noqa: E402
from _session import Session  # noqa: E402
from parts import GATES, INTEGRATED_SERVO, RIGID_BODY_CHAIN, RUNGS, pendulum_arm  # noqa: E402
import calibration  # noqa: E402
from robot_metrics import (DEGREES, Alias, CodeShare, DwellError, ErrorStat, HeldOutPair, MetricCollection,  # noqa: E402
                           PerformanceGain, Repeat, Repeatability, RunRecording, TimeOffset)

MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
TIPS = {station: facts["tip_length_m"] for station, facts in fixture.stations().items()}
ARMS = {station: pendulum_arm(tip) for station, tip in TIPS.items()}
FF_RESIDUAL = Alias("feed-forward residual RMS", ErrorStat(statistic="rms", field="effort"), "joint", "N·m", 1.0)
VELOCITY_ERROR = Alias("velocity error RMS", ErrorStat(statistic="rms", rows="moving", field="velocities"), "joint",
                       "deg/s", DEGREES, "median")


def recordings(source: str) -> list[RunRecording]:
    return [RunRecording.from_run(run) for run in fixture.pendulum_runs(source)]


def score(source: str) -> tuple[dict, dict]:
    """Every play alone (offline); a rung is its plays' collections merged (merge_state), never re-read."""

    plays, rungs = {}, {}
    for recording in recordings(source):
        station, rung = recording.labels["station"], recording.labels["rung"]
        collection = ARMS[station].collection()
        values = {**collection(recording), "faults": int(recording.faulted)}
        plays[recording.id] = values
        merged, faults = rungs.get((station, rung), (ARMS[station].collection(), 0))
        rungs[(station, rung)] = (merged.merge_state(collection), faults + int(recording.faulted))
    return plays, {key: {**merged.compute(), "faults": faults} for key, (merged, faults) in rungs.items()}


def judged(rungs: dict) -> dict:
    """Each rung cell's gate verdicts, and each station's highest rung with every rung up to it tracking."""

    cells = {key: {gate.name: gate.judge(values) for gate in GATES} for key, values in rungs.items()}
    highest = {}
    for station in sorted({station for station, _ in rungs}):
        reached = 0.0
        for rung in RUNGS:
            verdicts = cells.get((station, rung), {})
            if not all(verdicts.get(name) and verdicts[name].verdict == "pass" for name in ("tracks", "tip stop (median)")):
                break
            reached = rung
        highest[station] = reached
    return {"cells": cells, "highest": highest}


def summary(rungs: dict, verdicts: dict, title: str) -> str:
    mark = {"pass": "✓", "fail": "✗", "not_run": "?"}
    lines = [title, f"{'station':<7}{'rung':>5}{'RMS°':>7}{'peak°':>7}{'stop mm (worst)':>17}{'settle s':>9}"
                    f"{'limit %':>8}{'oversh°':>8}{'lag ms':>7}  gates: tracks · stop median · every stop"]
    for (station, rung), values in sorted(rungs.items()):
        cell = verdicts["cells"][(station, rung)]
        lines.append(f"{station:<7}{rung:>5.1f}{values['tracking RMS']:>7.2f}{values['tracking peak']:>7.2f}"
                     f"{values['tip stop error']:>9.2f} ({values['tip stop worst']:>4.2f})"
                     f"{values['tip settle'] if values['tip settle'] is not None else math.inf:>9.2f}"
                     f"{values['limiter share']:>8.1f}{values['overshoot']:>8.3f}{values['lag']:>7.2f}  "
                     + "      ".join(mark[cell[gate.name].verdict] for gate in GATES))
    for gate in GATES:
        passed = sum(cell[gate.name].verdict == "pass" for cell in verdicts["cells"].values())
        owners = sorted({member.owner for member in gate.members})
        lines.append(f"gate {gate.name!r} ({', '.join(owners)}): {passed} of {len(verdicts['cells'])} rung cells")
    lines.append("highest rung tracked: " + ", ".join(f"{s} {f:g}" for s, f in verdicts["highest"].items()))
    return "\n".join(lines)


def chain_values() -> dict:
    out = {}
    for source in ("real", "sim"):
        for run in fixture.chain_runs(source):
            out[run.id] = RIGID_BODY_CHAIN.collection()(RunRecording.from_run(run))
    return out


def check_dict(real: tuple, sim: tuple, chain: dict, live: dict, collection: MetricCollection) -> dict:
    """The frozen check's format, built from the API's own values (display units converted back where it asks SI)."""

    def play_rows(plays: dict) -> dict:
        return {key: {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                      "stops": int(v["stops"]), "overshoot_deg": v["overshoot"], "lag_ms": v["lag"]}
                for key, v in plays.items()}

    def rung_rows(rungs: dict) -> dict:
        return {f"{s}@{r:g}": {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                               "stop_tool_mm": v["tip stop error"], "stop_tool_max_mm": v["tip stop worst"],
                               "settle_s": v["tip settle"], "limiter_share": v["limiter share"] / 100.0}
                for (s, r), v in rungs.items()}

    rp, repeats = repeatability()
    gain, nominal, fitted = performance_gain()
    verdicts = judged(real[1])
    gates = {name: [sum(c[gate].verdict == "pass" for c in verdicts["cells"].values()), len(verdicts["cells"])]
             for name, gate in (("tracking", "tracks"), ("tool_stop_median", "tip stop (median)"),
                                ("tool_stop_worst", "tip stop (every stop)"))}
    added = MetricCollection((VELOCITY_ERROR,), INTEGRATED_SERVO.signals)
    arm = ARMS["can11"]
    return {
        "real_plays": play_rows(real[0]), "real_rungs": rung_rows(real[1]),
        "sim_plays": {k: {"tracking_rms_deg": v["tracking_rms_deg"]} for k, v in play_rows(sim[0]).items()},
        "sim_rungs": rung_rows(sim[1]), "highest_rung": verdicts["highest"], "gates": gates,
        "chain": {key: {"positions/j2": v["j2 tracking RMS"] / DEGREES, "positions/c1": v["c1 tracking RMS"] / DEGREES,
                        "tool_position_rms_m": v["tool tracking RMS"] / 1e3, "tool_position_max_m": v["tool worst"] / 1e3}
                  for key, v in chain.items()},
        "live": live,
        "metric_cards": [alias.card() for alias in arm.metrics if alias.name in ("tip stop error", "all stops settled")],
        "declared": {"integrated_servo": [a.name for a in INTEGRATED_SERVO.metrics],
                     "rigid_body_chain": [a.name for a in RIGID_BODY_CHAIN.metrics]},
        "added": {"name": VELOCITY_ERROR.name,
                  "values": {r.id: added.clone()(r)[VELOCITY_ERROR.name] for r in recordings("real")}},
        "aliases": [alias.card() for alias in arm.metrics],
        "dedup": {"aliases": len(arm.metrics), "computed": collection.computations},
        "inputs": {cls.__name__: cls.Input for cls in (ErrorStat, DwellError, TimeOffset, CodeShare, Repeatability,
                                                       PerformanceGain)},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": rp.compute(), "repeats": repeats},
        "performance_gain": {"alias": FF_RESIDUAL.name, "nominal": nominal, "fitted": fitted, "gain": gain.compute()},
    }


def repeatability(station: str = "can11", rung: float = 0.9) -> tuple[Repeatability, int]:
    """Repeatability(k=3) fed one real same-input repeat at a time."""

    metric = Repeatability(k=3.0)
    repeats = [run for run in fixture.pendulum_runs("real")
               if run.station == station and run.rung == rung and run.labels["kind"] == "repeat"]
    for run in repeats:
        metric.update(Repeat(command=(station, rung), target=run.target, measured=run.measured))
    return metric, len(repeats)


def performance_gain() -> tuple[PerformanceGain, float, float]:
    gain = PerformanceGain(FF_RESIDUAL, INTEGRATED_SERVO.signals)
    pair = HeldOutPair(nominal=tuple(RunRecording.from_run(r) for r in calibration.held_out_runs("nominal")),
                       fitted=tuple(RunRecording.from_run(r) for r in calibration.held_out_runs("fitted")))
    gain.update(pair)
    nominal, fitted = gain.sides[-1]
    return gain, nominal, fitted


def live_vs_offline(recording: RunRecording, rows: int = 7) -> tuple[dict, dict, MetricCollection]:
    arm = ARMS[recording.labels["station"]]
    offline = arm.collection()(recording)
    live = arm.collection()
    for chunk in recording.chunks(rows):
        live.update(chunk)
    live.end_run()
    return offline, live.compute(), live


def run() -> dict:
    recording = recordings("real")[0]
    offline, live, collection = live_vs_offline(recording)
    pairs = {"tracking_rms_deg": "tracking RMS", "tracking_peak_deg": "tracking peak", "stops": "stops",
             "stop_tool_max_mm": "tip stop worst", "overshoot_deg": "overshoot"}
    live_block = {"recording": recording.id, "values": {key: [live[name], offline[name]] for key, name in pairs.items()}}
    return check_dict(score("real"), score("sim"), chain_values(), live_block, collection)


CELLS = [
    ("a part's metrics", "A part type lists configured stock metrics beside its models; the pendulum arm adds tip "
     "aliases over the SAME bases (the lever is a display factor), the chain the same ErrorStat on its tool.", '''
print(INTEGRATED_SERVO.table())
arm = pendulum_arm(tip_length_m=TIPS["can11"])          # CAN 11: the 339 mm heavy pendulum
print(arm.table())
print(RIGID_BODY_CHAIN.table())
card = arm.metrics[7].card()                            # "tip stop error": an alias of DwellError(statistic="median")
{key: card[key] for key in ("name", "base", "config", "unit", "description", "references", "value_type")}
'''),
    ("one recording, offline and live", "The same collection fed the whole recording, and fed it 7 rows at a time with "
     "a peek mid-run: identical numbers, and 12 aliases cost 5 computations (dwell metrics share one stop tracker).", '''
recording = RunRecording.from_run(fixture.pendulum_runs("real")[32])   # CAN 11, rung 0.9, repeat
offline = arm.collection()(recording)                   # update + end_run + compute
live = arm.collection()
for number, chunk in enumerate(recording.chunks(rows=7)):
    live.update(chunk)
    if number == 40:
        print(f"after 280 rows: tip stop worst so far {live.compute()['tip stop worst']:.3f} mm, stops {live.compute()['stops']:.0f}")
live.end_run()
for name in ("tracking RMS", "tracking peak", "tip stop error", "tip stop worst", "tip settle", "overshoot", "lag", "stops"):
    print(f"{name:<15} offline {offline[name]:10.6f}   live {live.compute()[name]:10.6f}   {arm.metrics[[a.name for a in arm.metrics].index(name)].unit}")
print(live, "· states fed per update:", live.computations)
print(live.to_msg("rack/can11").as_dict()["tip stop error [mm]"])
'''),
    ("runs to a deployment, gates applied", "Each play scored alone, each rung its plays merged, the gates judged on the "
     "values outside compute(). The median tip stop passes 36 of 36; the worst stop fails 3 CAN 11 rungs.", '''
real = score("real")                                    # 252 plays -> 36 rung cells
verdicts = judged(real[1])
print(summary({k: v for k, v in real[1].items() if k[0] in ("can11", "can12")}, {"cells": {k: v for k, v in verdicts["cells"].items() if k[0] in ("can11", "can12")}, "highest": verdicts["highest"]}, "real vs target · CAN 11 and CAN 12 shown, all six judged"))
print("deployment: gates", {g.name: sum(c[g.name].verdict == "pass" for c in verdicts["cells"].values()) for g in GATES}, "of 36")
'''),
    ("the twin's output (sim vs target)", "The same aliases on the twin's answer to every play: the sim's own tip stop "
     "and tracking, never compared with the real run here (that is fidelity).", '''
sim = score("sim")
sim_verdicts = judged(sim[1])
for rung in RUNGS:
    real_v, sim_v = real[1][("can11", rung)], sim[1][("can11", rung)]
    print(f"CAN 11 rung {rung:.1f}: tracking RMS real {real_v['tracking RMS']:.3f}° sim {sim_v['tracking RMS']:.3f}° · "
          f"tip stop real {real_v['tip stop error']:.3f} mm sim {sim_v['tip stop error']:.3f} mm")
print("sim tip stop gate (median):", sum(c["tip stop (median)"].verdict == "pass" for c in sim_verdicts["cells"].values()), "of 36")
'''),
    ("the chain, a view, a new metric", "The five-bar through the same ErrorStat base; one alias plotted to matplotlib, "
     "the same result as a W&B payload and Rerun calls; a new metric is one more configured alias.", '''
for key, values in chain_values().items():
    print(f"{key:<34} " + " · ".join(f"{name} {value:.3f}" for name, value in values.items()))
station = ARMS["can11"].collection()
for recording in recordings("real"):
    if recording.labels["station"] == "can11":
        station.update(recording)
        station.end_run()
member = station.members[ARMS["can11"].metrics[8].key()]   # tip stop worst: the station's 84 stops
import _viz
view = member.view(scale=1e3 * TIPS["can11"], unit="mm", title="CAN 11 · every stop at the tip, round 8")
view = _viz.View(**{**view.__dict__, "target": 1.0, "target_label": "Zach's 1 mm"})
try:
    _viz.plot(view, path=MEDIA / "v1-tip-stops.png"); print("plot ->", MEDIA / "v1-tip-stops.png")
except ImportError:
    print("plot: matplotlib not installed (the viz extra)")
payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
print("W&B:", payload["summary"], "·", len(payload["table"]["data"]), "table rows ·", payload["chart"])
calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
print("Rerun:", len(calls), "calls, first", calls[0], "->", _viz.write_rrd(calls, MEDIA / "v1-tip-stops.rrd"))
faster = replace(ARMS["can11"], metrics=ARMS["can11"].metrics + (VELOCITY_ERROR,))
print(VELOCITY_ERROR.name, f"{faster.collection()(recordings('real')[32])['velocity error RMS']:.3f} deg/s")
'''),
]


CELLS.append(("other inputs: repeats and a calibration", "A class declares what update() takes: Repeatability one "
    "real repeat at a time, PerformanceGain a held-out pair; RunRecording is only the common case.", '''
print({cls.__name__: cls.Input for cls in (ErrorStat, DwellError, TimeOffset, Repeatability, PerformanceGain)})
rp, repeats = repeatability("can11", 0.9)
print(f"CAN 11 @0.9: RP (k=3) over {repeats} real repeats, worst stop: {rp.compute() * 1e3 * TIPS['can11']:.3f} mm at the tip ({rp.compute() * DEGREES:.4f}°)")
try:
    rp.update(Repeat(command=("can11", 1.0), target=recording.streams["target"], measured=recording.streams["state"]))
except ValueError as refused:
    print("refused:", refused)
outcome = calibration.outcome()                       # the record: robot_metrics.records.CalibrationOutcome
print(outcome.model, {name: round(interval.value, 4) for name, interval in outcome.recovered.items()}, "fitted on", outcome.fitted_on)
gain, nominal, fitted = performance_gain()
print(f"PerformanceGain({FF_RESIDUAL.name!r}): nominal {nominal:.4f} N·m, fitted {fitted:.4f} N·m, gain {gain.compute():.4f} N·m on {outcome.held_out}")
'''))


def main() -> None:
    namespace = dict(globals())
    session = Session(namespace)
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    MEDIA.mkdir(parents=True, exist_ok=True)
    main()
