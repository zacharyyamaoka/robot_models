"""v5 · Part tree: the shared scenario as one REPL session on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v5_part_tree/scenario.py

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
from parts import DEGREES, FIVE_BAR, RUNGS, SERVO_SIGNALS, pendulum_arm, rack, servo_aliases  # noqa: E402

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


def plays_of(station: str, rung: float) -> list:
    """One rung cell's real plays, as the roll-up groups them."""

    return [run for run in fixture.pendulum_runs("real") if run.station == station and run.rung == rung]


def stream_chunks(run, rows: int = 7) -> list[dict]:
    """The run's named streams as a live feed delivers them: ``rows`` state samples at a time (the data adapter)."""

    named = streams(run)
    times = run.measured.t
    edges = list(times[rows - 1::rows])
    if not edges or edges[-1] < times[-1]:
        edges.append(times[-1])
    chunks, low = [], -float("inf")
    for high in edges:
        chunks.append({name: stream[(stream.t > low) & (stream.t <= high)] for name, stream in named.items()})
        low = high
    return chunks


def metrics_table(node: Part) -> str:
    """A node's aliases as a table: name, configured metric, the child it reads (if any), signal, unit."""

    lines = [f"{node.name}: {len(node.metrics)} aliases"]
    for alias in node.metrics:
        source = f"child {alias.of_child} " if alias.of_child else ""
        lines.append(f"  {alias.name:<20} {alias.metric!r:<62} {source}{alias.signal} [{alias.unit or '-'}]")
    return "\n".join(lines)


def rung_table(rungs: dict, judged: dict, stations: tuple[str, ...]) -> str:
    """The rung cells of ``stations`` with their three verdicts and the arm's level (display only)."""

    mark = {True: "✓", False: "✗"}
    lines = ["station rung   RMS °  peak °  stop mm  worst mm   tracks · median · every stop   level"]
    for (station, rung), values in sorted(rungs.items()):
        if station not in stations:
            continue
        cell = judged["cells"][(station, rung)]
        marks = "        ".join(mark[bool(cell[name])] for name in ("tracks", "tip stop (median)",
                                                                   "tip stop (every stop)"))
        lines.append(f"{station:<7}{rung:5.1f}{values['tracking RMS']:8.2f}{values['tracking peak']:8.2f}"
                     f"{values['tip stop error']:9.2f}{values['tip stop worst']:10.2f}   {marks}      "
                     f"{LEVEL[cell['level']]}")
    return "\n".join(lines)


def worst(judgements: list[tuple[int, list[str]]]) -> tuple[int, list[str]]:
    """The worst (level, reasons) of several: a parent's level is its worst child's (diagnostic_aggregator)."""

    return max(judgements, key=lambda judgement: judgement[0])


# WHY the same seven headings in every variant: the five glances are compared side by side, row by row.
GLANCE = ("API at a glance", "The design's shape before any detail: a metric over move and dwell events, an alias "
          "that can read a child part, a node's .metrics beside its .models and .children, live through the subtree, "
          "offline as one observe(), checks judged into levels, and a rung as a group of the node's ledger.", '''
# a metric: a config over move and dwell events; this key names the shared segmenter
stop_worst = StopEvents(of="error", statistic="max")
stop_worst.segmenter()

# an alias: a name on a node's signal; an assembly's alias reads its child's events
tip_stop_worst = Alias("tip stop worst", stop_worst, "output", "mm", scale_by="lever_mm",
                       of_child="can11 servo")
tip_stop_worst

# a part type's metrics, beside its models and its children
arm = pendulum_arm("can11", tip_length_m=0.339)
arm.child("can11 servo").models
[alias.name for alias in arm.metrics]

# live: the arm's subtree takes chunks; each segmenter carries the open move or dwell
run = fixture.pendulum_runs("real")[32]
arm.begin()
for chunk in stream_chunks(run, rows=7):
    arm.update(chunk)
arm.value("tip stop worst", live=True)

# offline: one observe() per run, written to each node's ledger
arm.observe(streams(run), {"rung": run.rung})
arm.value("tip stop worst")

# a gate: checks declared on the node with their owners, judged into a level
arm.checks
level, reasons = arm.level(arm.values())
LEVEL[level], reasons

# across runs: a rung is a group of the node's ledger, every stop pooled
rung_arm = pendulum_arm("can11", tip_length_m=0.339)
for play in plays_of("can11", rung=0.9):
    rung_arm.observe(streams(play), {"rung": play.rung})
by_rung = rung_arm.child("can11 servo").groups(("rung",))
rung_values = rung_arm.values(by_rung[(0.9,)])
rung_values["tip stop worst"]
level, reasons = rung_arm.level(rung_values)
LEVEL[level], reasons
''')

CELLS = [
    ("a part's metrics", "The rack is a tree of RobotParts; each node lists aliases on ITS signal beside its models. "
     "The arm's tip aliases name the servo child: the servo computes once, the arm shows it through its lever.", '''
tree = rack(TIPS)
[arm.name for arm in tree.children]

arm = tree.child("can11 arm")
servo = arm.child("can11 servo")
servo.models
print(metrics_table(servo))

arm.models.type
arm.context
print(metrics_table(arm))

FIVE_BAR.name
[child.name for child in FIVE_BAR.children]
[alias.name for alias in FIVE_BAR.metrics]

aliases = {alias.name: alias for alias in arm.metrics}
aliases["tip stop error"].card()
'''),
    ("one recording, offline and live", "Live, the arm subtree takes 7-row chunks; each node's segmenter carries the "
     "open move or dwell across chunk boundaries; events are what the drill-down opens.", '''
run = fixture.pendulum_runs("real")[32]       # CAN 11, rung 0.9, a repeat
offline, live, live_arm = live_vs_offline(run)

names = ["tracking RMS", "tracking peak", "tip stop error", "tip stop worst",
         "tip settle", "overshoot", "lag", "stops"]
print(f"{'':<15} {'offline':>10} {'live':>10}")
for name in names:
    difference = abs(offline[name] - live[name])
    print(f"{name:<15} {offline[name]:10.6f} {live[name]:10.6f}  diff {difference:.1e}")

segment_key = ("output", ("positions", 0, True))
events = live_arm.child("can11 servo").segmenters[segment_key].events()
len(events)

tip_mm = 1e3 * TIPS["can11"]
for event in events:
    line = f"{event.kind:<5} {event.t0:6.2f} - {event.t1:5.2f} s"
    if event.stops:
        line += f"   stop {abs(event.stops[0].error) * tip_mm:.3f} mm"
    print(line)

{node.name: node.computations for node in live_arm.walk()}     # segmentations per update
len(live_arm.metrics) + len(live_arm.child("can11 servo").metrics)    # aliases they serve
'''),
    ("runs to a deployment, gates applied", "Every play observed by its arm subtree; per rung each node's aliases; the "
     "rack's status is its worst child (diagnostic_aggregator), with the reason named.", '''
real_tree = observe_all("real")
real_plays, real_rungs = tables(real_tree)
len(real_plays), len(real_rungs)

judged = verdicts(real_tree, real_rungs)
print(rung_table(real_rungs, judged, stations=("can11", "can12")))

gate_names = ("tracks", "tip stop (median)", "tip stop (every stop)")
cells = judged["cells"].values()
{name: sum(bool(cell[name]) for cell in cells) for name in gate_names}
judged["highest"]

real_tree.child("can11 arm").checks

status = {}
for arm in real_tree.children:
    station, servo = arm.name.split()[0], arm.children[0]
    rung_values = [real_rungs[(station, rung)] for rung in RUNGS]
    servo_status = worst([servo.level(values) for values in rung_values])
    arm_own = worst([arm.level(values) for values in rung_values])
    status[servo.name] = servo_status
    status[arm.name] = (max(arm_own[0], servo_status[0]), arm_own[1])    # its worst child
status[real_tree.name] = (max(level for level, _ in status.values()), [])
for line in show(real_tree, status):
    print(line)
'''),
    ("the twin's output (sim vs target)", "The same tree on the twin's answers.", '''
sim_tree = observe_all("sim")
sim_plays, sim_rungs = tables(sim_tree)

print("CAN 11 rung   RMS real °   RMS sim °   stop real mm   stop sim mm")
for rung in RUNGS:
    real_values = real_rungs[("can11", rung)]
    sim_values = sim_rungs[("can11", rung)]
    rms = (real_values["tracking RMS"], sim_values["tracking RMS"])
    stop = (real_values["tip stop error"], sim_values["tip stop error"])
    print(f"{rung:11.1f} {rms[0]:12.3f} {rms[1]:11.3f} {stop[0]:14.3f} {stop[1]:13.3f}")

sim_cells = verdicts(sim_tree, sim_rungs)["cells"].values()
sum(bool(cell["tip stop (median)"]) for cell in sim_cells)
'''),
    ("the chain, a view, a new metric", "The five-bar is a node whose children are its joints; the same TrackingError "
     "on joints and tool. A view to matplotlib, W&B and Rerun; a new alias on the servo node.", '''
chain = chain_values()
for run_id, values in chain.items():
    rounded = {name: round(value, 3) for name, value in values.items()}
    print(run_id, rounded)

can11_arm = real_tree.child("can11 arm")
by_rung = can11_arm.child("can11 servo").groups(("rung",))
stops, rung_labels = [], []
for rung in RUNGS:
    rung_stops = can11_arm.items("tip stop error", by_rung[(rung,)])
    stops += rung_stops
    rung_labels += [f"{rung:g}"] * len(rung_stops)
len(stops)

import _viz
title = "CAN 11 · every stop at the tip, by rung"
view = _viz.View(title=title, unit="mm", hint="distribution",
                 x=list(range(len(stops))), y=stops, group=rung_labels,
                 target=1.0, target_label="Zach's 1 mm", x_label="stop", y_label="mm")
_viz.plot(view, path=MEDIA / "v5-tip-stops.png")

payload = _viz.wandb_payload(view, "can11/tip_stop_mm")
payload["summary"]

calls = _viz.rerun_calls(view, "metrics/can11/tip_stop_mm")
_viz.write_rrd(calls, MEDIA / "v5-tip-stops.rrd")

VELOCITY_ERROR
with_velocity_aliases = servo_aliases() + (VELOCITY_ERROR,)
with_velocity = Part("can11 servo", None, with_velocity_aliases, SERVO_SIGNALS)
with_velocity.observe(streams(run), {})
with_velocity.value("velocity error RMS")
with_velocity.computations
'''),
    ("other inputs: repeats and a calibration", "Repeatability reads a node's event logs of one command; "
     "PerformanceGain two event logs of the same held-out runs. A recording is only the common case.", '''
inputs = (TrackingError, StopEvents, Lag, Repeatability, PerformanceGain)
{cls.__name__: cls.Input for cls in inputs}

repeatability_value, repeats = repeatability("can11", 0.9)
repeats
repeatability_value * tip_mm              # mm at the tip

outcome = calibration.outcome()           # a robot_msgs CalibrationOutcome message
outcome.model

FF_RESIDUAL.name
nominal, fitted, gain = performance_gain()
nominal, fitted                           # N·m on the held-out runs
gain
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
