"""v5 · Part tree: the shared scenario as one REPL session on round 8's real runs.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v5_part_tree/scenario.py

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
from part_metrics import (LEVEL, Alias, CodeShare, Lag, Part, PerformanceGain, Repeatability, Segmenter,  # noqa: E402
                          StopEvents, TrackingError)
from parts import DEGREES, FIVE_BAR, RUNGS, SERVO_SIGNALS, rack, servo_aliases  # noqa: E402

MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
TIPS = {station: facts["tip_length_m"] for station, facts in fixture.stations().items()}
FF_RESIDUAL = Alias("feed-forward residual RMS", TrackingError(statistic="rms", field="effort"), "output", "N·m")
VELOCITY_ERROR = Alias("velocity error RMS", TrackingError(statistic="rms", field="velocities"), "output", "deg/s",
                       DEGREES, across="median")


def streams(run) -> dict:
    out = {"target": run.target, "state": run.measured}
    if run.plan is not None:
        out["plan"] = run.plan
    if run.tool_target is not None:
        out.update({"tool/target": run.tool_target, "tool/state": run.tool_measured})
    return out


def observe_all(source: str) -> Part:
    """The rack observes every play: each station's arm subtree takes its own runs (the rack's ledger is its arms')."""

    tree = rack(TIPS)
    for run in fixture.pendulum_runs(source):
        tree.child(f"{run.station} arm").observe(streams(run), {"id": run.id, "rung": run.rung, "kind": run.labels["kind"],
                                                                "source": run.source}, run.faulted)
    return tree


def tables(tree: Part) -> tuple[dict, dict]:
    plays, rungs = {}, {}
    for arm in tree.children:
        station, servo = arm.name.split()[0], arm.children[0]
        for arm_entry, servo_entry in zip(arm.ledger, servo.ledger):
            plays[arm_entry["labels"]["id"]] = {**servo.values([servo_entry]), **arm.values([servo_entry])}
        servo_groups = servo.groups(("rung",))
        for (rung,), entries in servo_groups.items():
            rungs[(station, rung)] = {**servo.values(entries), **arm.values(entries),
                                      "faults": sum(entry["faulted"] for entry in entries)}
    return plays, rungs


def verdicts(tree: Part, rungs: dict) -> dict:
    cells = {}
    for (station, rung), values in rungs.items():
        arm = tree.child(f"{station} arm")
        servo_level, servo_why = arm.children[0].level(values)
        tracks = servo_level == 0 and values["faults"] == 0
        median = values["tip stop error"] is not None and values["tip stop error"] <= 1.0 and \
            values["tip settle"] is not None and values["tip settle"] <= 0.3
        cells[(station, rung)] = {"tracks": tracks, "tip stop (median)": median,
                                  "tip stop (every stop)": values["tip stop worst"] <= 1.0,
                                  "level": max(servo_level, arm.level(values)[0]), "why": servo_why + arm.level(values)[1]}
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
    out = {}
    for source in ("real", "sim"):
        for run in fixture.chain_runs(source):
            tree = Part(FIVE_BAR.name, FIVE_BAR.models, FIVE_BAR.metrics, FIVE_BAR.signals, children=tuple(
                Part(c.name, c.models, c.metrics, c.signals) for c in FIVE_BAR.children))
            tree.observe(streams(run), {"id": run.id})
            out[run.id] = {**tree.values(), **{k: v for c in tree.children for k, v in c.values().items()}}
    return out


def live_vs_offline(run, rows: int = 7) -> tuple[dict, dict, Part]:
    station = run.station
    tree = rack(TIPS)
    arm = tree.child(f"{station} arm")
    arm.observe(streams(run), {"id": run.id, "rung": run.rung})
    offline = {**arm.children[0].values(), **arm.values()}
    live_tree = rack(TIPS)
    live = live_tree.child(f"{station} arm")
    live.begin()
    t = run.measured.t
    edges = list(t[rows - 1::rows])
    if not edges or edges[-1] < t[-1]:
        edges.append(t[-1])
    low = -float("inf")
    for high in edges:
        live.update({name: s[(s.t > low) & (s.t <= high)] for name, s in streams(run).items()})
        low = high
    values = {**live.children[0].values(live=True), **live.values(live=True)}
    return offline, values, live


def repeatability(station: str = "can11", rung: float = 0.9) -> tuple[float, int]:
    node = Part(f"{station} servo", None, servo_aliases(), SERVO_SIGNALS)
    for run in fixture.pendulum_runs("real"):
        if run.station == station and run.rung == rung and run.labels["kind"] == "repeat":
            node.observe(streams(run), {"command": (station, rung)})
    key = ("output", ("positions", 0, True))
    return Repeatability(k=3.0).compute([entry["events"][key] for entry in node.ledger]), len(node.ledger)


def performance_gain() -> tuple[float, float, float]:
    key = ("output", ("effort", 0, True))
    logs = []
    for which in ("nominal", "fitted"):
        node = Part("servo", None, (FF_RESIDUAL,), {"output": ("target", "state")})
        for run in calibration.held_out_runs(which):
            node.observe(streams(run), {"values": which})
        logs.append([event for entry in node.ledger for event in entry["events"][key]])
    gain = PerformanceGain(FF_RESIDUAL.metric)
    nominal, fitted = (FF_RESIDUAL.metric.compute(events) for events in logs)
    return nominal, fitted, gain.compute((logs[0], logs[1]))


def check_dict(real: tuple, sim: tuple, tree: Part, chain: dict, live: dict, live_node: Part) -> dict:
    def play_rows(plays):
        return {k: {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                    "stops": int(v["stops"]), "overshoot_deg": v["overshoot"], "lag_ms": v["lag"]}
                for k, v in plays.items()}

    def rung_rows(rungs):
        return {f"{s}@{r:g}": {"tracking_rms_deg": v["tracking RMS"], "tracking_peak_deg": v["tracking peak"],
                               "stop_tool_mm": v["tip stop error"], "stop_tool_max_mm": v["tip stop worst"],
                               "settle_s": v["tip settle"], "limiter_share": v["limiter share"] / 100.0}
                for (s, r), v in rungs.items()}

    judged = verdicts(tree, real[1])
    gates = {key: [sum(bool(c[name]) for c in judged["cells"].values()), len(judged["cells"])]
             for key, name in (("tracking", "tracks"), ("tool_stop_median", "tip stop (median)"),
                               ("tool_stop_worst", "tip stop (every stop)"))}
    added = {}
    for run in fixture.pendulum_runs("real"):
        node = Part("servo", None, (VELOCITY_ERROR,), {"output": ("target", "state")})
        node.observe(streams(run), {})
        added[run.id] = node.values()[VELOCITY_ERROR.name]
    rp, repeats = repeatability()
    nominal, fitted, gain = performance_gain()
    arm = tree.child("can11 arm")
    cards = [*arm.children[0].metrics, *arm.metrics]
    return {
        "real_plays": play_rows(real[0]), "real_rungs": rung_rows(real[1]),
        "sim_plays": {k: {"tracking_rms_deg": v["tracking_rms_deg"]} for k, v in play_rows(sim[0]).items()},
        "sim_rungs": rung_rows(sim[1]), "highest_rung": judged["highest"], "gates": gates,
        "chain": {k: {"positions/j2": v["j2 tracking RMS"] / DEGREES, "positions/c1": v["c1 tracking RMS"] / DEGREES,
                      "tool_position_rms_m": v["tool tracking RMS"] / 1e3, "tool_position_max_m": v["tool worst"] / 1e3}
                  for k, v in chain.items()},
        "live": live,
        "metric_cards": [a.card() for a in arm.metrics if a.name in ("tip stop error", "all stops settled")],
        "declared": {"integrated_servo": [a.name for a in servo_aliases()],
                     "rigid_body_chain": [a.name for a in FIVE_BAR.metrics] + [a.name for c in FIVE_BAR.children
                                                                               for a in c.metrics]},
        "added": {"name": VELOCITY_ERROR.name, "values": added},
        "aliases": [a.card() for a in cards],
        "dedup": {"aliases": len(cards), "computed": sum(node.computations for node in live_node.walk())},
        "inputs": {cls.__name__: cls.Input for cls in (TrackingError, StopEvents, Lag, CodeShare, Repeatability,
                                                       PerformanceGain)},
        "repeatability": {"group": "can11@0.9", "k": 3.0, "value": rp, "repeats": repeats},
        "performance_gain": {"alias": FF_RESIDUAL.name, "nominal": nominal, "fitted": fitted, "gain": gain},
    }


def run() -> dict:
    first = fixture.pendulum_runs("real")[0]
    offline, live, live_node = live_vs_offline(first)
    pairs = {"tracking_rms_deg": "tracking RMS", "tracking_peak_deg": "tracking peak", "stops": "stops",
             "stop_tool_max_mm": "tip stop worst", "overshoot_deg": "overshoot"}
    live_block = {"recording": first.id, "values": {key: [live[name], offline[name]] for key, name in pairs.items()}}
    real_tree, sim_tree = observe_all("real"), observe_all("sim")
    return check_dict(tables(real_tree), tables(sim_tree), real_tree, chain_values(), live_block, live_node)


def show(node: Part, values_by_node: dict, depth: int = 0) -> list[str]:
    level, why = values_by_node[node.name]
    lines = [f"{'  ' * depth}{LEVEL[level]:<6} {node.name}" + (f"  ({'; '.join(why[:2])})" if why else "")]
    for child in node.children:
        lines += show(child, values_by_node, depth + 1)
    return lines


CELLS = [
    ("a part's metrics", "The rack is a tree of RobotParts; each node lists aliases on ITS signal beside its models. "
     "The arm's tip aliases name the servo child: the servo computes once, the arm shows it through its lever.", '''
tree = rack(TIPS)
arm = tree.child("can11 arm")
for node in (arm.children[0], arm):
    print(f"{node.name}: models {getattr(node.models, '__name__', None) or node.models.type} · context {dict(node.context)}")
    for alias in node.metrics:
        print(f"  {alias.name:<20} {alias.metric!r:<55} {('child ' + alias.of_child + ' ') if alias.of_child else ''}{alias.signal} [{alias.unit or '-'}]")
print("five-bar:", FIVE_BAR.name, "->", [c.name for c in FIVE_BAR.children], "· its own:", [a.name for a in FIVE_BAR.metrics])
card = arm.metrics[1].card()
{key: card[key] for key in ("name", "base", "config", "unit", "signal", "description", "references", "value_type")}
'''),
    ("one recording, offline and live", "Live, the arm subtree takes 7-row chunks; each node's segmenter carries the "
     "open move or dwell across chunk boundaries; events are what the drill-down opens.", '''
run = fixture.pendulum_runs("real")[32]                  # CAN 11, rung 0.9, repeat
offline, live, node = live_vs_offline(run)
for name in ("tracking RMS", "tracking peak", "tip stop error", "tip stop worst", "tip settle", "overshoot", "lag", "stops"):
    print(f"{name:<15} offline {offline[name]:10.6f}   live {live[name]:10.6f}   |diff| {abs(offline[name] - live[name]):.1e}")
events = node.children[0].segmenters[("output", ("positions", 0, True))].events()
print(len(events), "events:", " · ".join(f"{e.kind} {e.t0:.2f}-{e.t1:.2f} s" + (f" stop {abs(e.stops[0].error) * 1e3 * TIPS['can11']:.3f} mm" if e.stops else "") for e in events))
print("segmentations per update:", {n.name: n.computations for n in node.walk()}, "for", len(node.metrics) + len(node.children[0].metrics), "aliases")
'''),
    ("runs to a deployment, gates applied", "Every play observed by its arm subtree; per rung each node's aliases; the "
     "rack's status is its worst child (diagnostic_aggregator), with the reason named.", '''
real_tree = observe_all("real")
real = tables(real_tree)
judged = verdicts(real_tree, real[1])
mark = {True: "✓", False: "✗"}
for (station, rung), v in sorted(real[1].items()):
    if station in ("can11", "can12"):
        c = judged["cells"][(station, rung)]
        print(f"{station}@{rung:<4g} RMS {v['tracking RMS']:.2f}° peak {v['tracking peak']:.2f}° · tip stop {v['tip stop error']:.2f} (worst {v['tip stop worst']:.2f}) mm · {mark[c['tracks']]} {mark[c['tip stop (median)']]} {mark[c['tip stop (every stop)']]} · {LEVEL[c['level']]}")
print({n: sum(bool(c[n]) for c in judged["cells"].values()) for n in ("tracks", "tip stop (median)", "tip stop (every stop)")}, "of 36 ·", judged["highest"])
status = {}                                              # each node judged on its own checks, worst over rungs
for arm in real_tree.children:
    station, servo = arm.name.split()[0], arm.children[0]
    servo_level = max((servo.level(real[1][(station, r)]) for r in RUNGS), key=lambda lw: lw[0])
    arm_own = max((arm.level(real[1][(station, r)]) for r in RUNGS), key=lambda lw: lw[0])
    status[servo.name] = servo_level
    status[arm.name] = (max(arm_own[0], servo_level[0]), arm_own[1])      # a parent is its worst child
status[real_tree.name] = (max(level for level, _ in status.values()), [])
print("\\n".join(show(real_tree, status)[:7]))
'''),
    ("the twin's output (sim vs target)", "The same tree on the twin's answers.", '''
sim_tree = observe_all("sim")
sim = tables(sim_tree)
for rung in RUNGS:
    r, s = real[1][("can11", rung)], sim[1][("can11", rung)]
    print(f"CAN 11 @{rung:.1f}: tracking RMS real {r['tracking RMS']:.3f}° sim {s['tracking RMS']:.3f}° · tip stop real {r['tip stop error']:.3f} sim {s['tip stop error']:.3f} mm")
print("sim tip stop (median):", sum(bool(c["tip stop (median)"]) for c in verdicts(sim_tree, sim[1])["cells"].values()), "of 36")
'''),
    ("the chain, a view, a new metric", "The five-bar is a node whose children are its joints; the same TrackingError "
     "on joints and tool. A view to matplotlib, W&B and Rerun; a new alias on the servo node.", '''
for key, values in chain_values().items():
    print(f"{key:<34} " + " · ".join(f"{name} {value:.3f}" for name, value in values.items()))
import _viz
arm = real_tree.child("can11 arm")
groups = arm.children[0].groups(("rung",))
stops = [item for rung in RUNGS for item in arm.items("tip stop error", groups[(rung,)])]
labels = [f"{rung:g}" for rung in RUNGS for _ in arm.items("tip stop error", groups[(rung,)])]
shown = _viz.View(title="CAN 11 · every stop at the tip, by rung", unit="mm", hint="distribution", x=list(range(len(stops))), y=stops, group=labels, target=1.0, target_label="Zach's 1 mm", x_label="stop", y_label="mm")
try:
    _viz.plot(shown, path=MEDIA / "v5-tip-stops.png"); print("plot ->", MEDIA / "v5-tip-stops.png")
except ImportError:
    print("plot: matplotlib not installed (the viz extra)")
print("W&B:", _viz.wandb_payload(shown, "can11/tip_stop_mm")["summary"], "· Rerun ->", _viz.write_rrd(_viz.rerun_calls(shown, "metrics/can11/tip_stop_mm"), MEDIA / "v5-tip-stops.rrd"))
node = Part("can11 servo", None, servo_aliases() + (VELOCITY_ERROR,), SERVO_SIGNALS)
node.observe(streams(run), {})
print(f"velocity error RMS {node.values()['velocity error RMS']:.3f} deg/s · segmentations {node.computations}")
'''),
    ("other inputs: repeats and a calibration", "Repeatability reads a node's event logs of one command; "
     "PerformanceGain two event logs of the same held-out runs. A recording is only the common case.", '''
print({cls.__name__: cls.Input for cls in (TrackingError, StopEvents, Lag, Repeatability, PerformanceGain)})
rp, repeats = repeatability("can11", 0.9)
print(f"CAN 11 @0.9: RP (k=3) over {repeats} real repeats: {rp * 1e3 * TIPS['can11']:.3f} mm at the tip")
outcome = calibration.outcome()                       # the record: robot_msgs (a CalibrationOutcome message)
nominal, fitted, gain = performance_gain()
print(f"PerformanceGain({FF_RESIDUAL.name!r}): nominal {nominal:.4f}, fitted {fitted:.4f}, gain {gain:.4f} N·m")
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
