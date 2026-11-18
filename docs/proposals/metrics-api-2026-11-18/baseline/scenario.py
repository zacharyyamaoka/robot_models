"""baseline · robot_metrics 0.1.0 exactly as published: round 2's comparison column, on round 8's real runs.

    uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/baseline/scenario.py

Writes scenario.out.txt and session.json beside this file (the view cell needs the viz extra), and the view's PNG and
.rrd under reports/media/ (gitignored). ``run()`` is what the frozen parity check calls: the same API, no transcript.

The API does not change here. ``robot_metrics`` is the published package read through a symlink
(``baseline/robot_metrics -> /home/bam/robot_metrics/src/robot_metrics``); importing this file asserts it is 0.1.0 and
that its src tree is byte-identical to tag v0.1.0 (the tag, not HEAD: the repo's README and website keep moving).
The only glue is ``fixture_runs.as_run``: round 1's fixture runs copied into 0.1.0's ``Run``.
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROUND2 = HERE.parent
ROUND1 = ROUND2.parent / "metrics-api-2026-11-17"
PUBLISHED = pathlib.Path("/home/bam/robot_metrics")
sys.path[:0] = [str(HERE), str(ROUND2), str(ROUND1)]

import calibration  # noqa: E402
import fixture  # noqa: E402
import robot_metrics  # noqa: E402
from _session import Session  # noqa: E402
from fixture_runs import as_run  # noqa: E402
from robot_metrics import (DEGREES, Alias, CodeShare, Collection, ErrorStat, HeldOutPair, PerformanceGain,  # noqa: E402
                           Repeat, Repeatability, StopStat, TimeOffset, catalog, highest_passing, summarize)

# WHY assert, not trust: this column is only a baseline if it is the release Zach saw, byte for byte
assert robot_metrics.__version__ == "0.1.0", robot_metrics.__version__
assert pathlib.Path(robot_metrics.__file__).resolve().is_relative_to(PUBLISHED / "src" / "robot_metrics")
_SRC_DIFF = subprocess.run(["git", "--no-optional-locks", "-C", str(PUBLISHED), "diff", "--quiet", "v0.1.0", "--",
                            "src"], capture_output=True, check=False)
assert _SRC_DIFF.returncode == 0, "robot_metrics/src differs from tag v0.1.0: not the published release"

# check names for the catalog's three gates (the frozen check's vocabulary, not the page's)
GATE_NAMES = {"tracking": "tracks", "tool_stop_median": "tip stop (median)", "tool_stop_worst": "tip stop (every stop)"}
FF_RESIDUAL = Alias("feed-forward residual RMS", ErrorStat(statistic="rms", field="effort"), unit="N·m")
VELOCITY_ERROR = Alias("velocity error RMS", ErrorStat(statistic="rms", rows="moving", field="velocities"),
                       unit="deg/s", scale=DEGREES, across="median")


def arm_collections() -> dict[str, Collection]:
    """One collection per station: the catalog's pendulum arm with that station's lever."""

    return {station: catalog.pendulum_arm(facts["tip_length_m"]).collection()
            for station, facts in fixture.stations().items()}


def play_rows(runs: list, collections: dict[str, Collection]) -> dict[str, dict]:
    out = {}
    for run in runs:
        collection = collections[run.labels["station"]]
        values = collection.values(collection.fold(run))
        out[run.id] = {"tracking_rms_deg": values["servo/tracking RMS"],
                       "tracking_peak_deg": values["servo/tracking peak"], "stops": int(values["servo/stops"]),
                       "overshoot_deg": values["servo/overshoot"], "lag_ms": values["servo/lag"]}
    return out


def station_strata(runs: list, collections: dict[str, Collection]) -> dict:
    out = {}
    for station, collection in collections.items():
        out.update(summarize(collection, [run for run in runs if run.labels["station"] == station],
                             by=("station", "rung")))
    return out


def rung_rows(strata: dict) -> dict[str, dict]:
    out = {}
    for (station, rung), stratum in strata.items():
        values = stratum.values
        share = values["servo/limiter share"]
        out[f"{station}@{float(rung):g}"] = {
            "tracking_rms_deg": values["servo/tracking RMS"], "tracking_peak_deg": values["servo/tracking peak"],
            "stop_tool_mm": values["tip stop error"], "stop_tool_max_mm": values["tip stop worst"],
            "settle_s": values["tip settle"], "limiter_share": None if share is None else share / 100.0}
    return out


def judged(strata: dict) -> dict:
    gates = catalog.pendulum_gates()
    return {key: {name: gate.judge(stratum).verdict for name, gate in gates.items()} for key, stratum in strata.items()}


def chain_rows() -> dict[str, dict]:
    collection = catalog.rigid_chain(("j2", "c1")).collection()
    out = {}
    for rung in fixture.chain_runs("real") + fixture.chain_runs("sim"):
        values = collection.values(collection.fold(as_run(rung)), si=True)
        out[rung.id] = {"positions/j2": values["j2/tracking RMS"], "positions/c1": values["c1/tracking RMS"],
                        "tool_position_rms_m": values["tool tracking RMS"], "tool_position_max_m": values["tool worst"]}
    return out


def live_block(run, collection: Collection, rows: int = 7) -> dict:
    offline = collection.values(collection.fold(run))
    state = collection.empty()
    for chunk in run.chunks(rows):
        state = collection.update(state, chunk)
    live = collection.values(collection.close(state))
    pairs = {"tracking_rms_deg": "servo/tracking RMS", "tracking_peak_deg": "servo/tracking peak",
             "stops": "servo/stops", "stop_tool_mm": "tip stop error", "stop_tool_max_mm": "tip stop worst",
             "overshoot_deg": "servo/overshoot", "lag_ms": "servo/lag"}
    return {"recording": run.id, "values": {key: [live[name], offline[name]] for key, name in pairs.items()}}


def repeatability_block(real: list, station: str = "can11", rung: float = 0.9) -> dict:
    metric = Repeatability(k=3.0)
    repeats = [Repeat((station, rung), run.streams["target"], run.streams["state"]) for run in real
               if run.labels["station"] == station and run.labels["rung"] == rung and run.labels["kind"] == "repeat"]
    return {"group": f"{station}@{rung:g}", "k": 3.0, "value": metric.compute(metric.fold(repeats)),
            "repeats": len(repeats)}


def performance_gain_block() -> dict:
    gain = PerformanceGain(metric=FF_RESIDUAL)
    pair = HeldOutPair(nominal=tuple(as_run(plateau) for plateau in calibration.held_out_runs("nominal")),
                       fitted=tuple(as_run(plateau) for plateau in calibration.held_out_runs("fitted")))
    state = gain.update(gain.empty(), pair)
    nominal, fitted = gain.sides(state)
    return {"alias": FF_RESIDUAL.name, "nominal": nominal, "fitted": fitted, "gain": gain.compute(state)}


def run() -> dict:
    real = [as_run(play) for play in fixture.pendulum_runs("real")]
    sim = [as_run(play) for play in fixture.pendulum_runs("sim")]
    collections = arm_collections()
    real_strata, sim_strata = station_strata(real, collections), station_strata(sim, collections)
    verdicts = judged(real_strata)
    can11 = collections["can11"]
    added = Collection([VELOCITY_ERROR])
    return {
        "real_plays": play_rows(real, collections),
        "real_rungs": rung_rows(real_strata),
        "sim_plays": {key: {"tracking_rms_deg": row["tracking_rms_deg"]}
                      for key, row in play_rows(sim, collections).items()},
        "sim_rungs": rung_rows(sim_strata),
        "highest_rung": highest_passing({key: cell["tracks"] == "pass" and cell["tip stop (median)"] == "pass"
                                         for key, cell in verdicts.items()}),
        "gates": {name: [sum(cell[gate] == "pass" for cell in verdicts.values()), len(verdicts)]
                  for name, gate in GATE_NAMES.items()},
        "chain": chain_rows(),
        "live": live_block(real[32], can11),
        "metric_cards": [can11.aliases[name].card() for name in ("tip stop error", "all stops settled")],
        "declared": {"integrated_servo": list(catalog.integrated_servo().collection().aliases),
                     "rigid_body_chain": list(catalog.rigid_chain(("j2", "c1")).collection().aliases)},
        "added": {"name": VELOCITY_ERROR.name,
                  "values": {run.id: added.values(added.fold(run))[VELOCITY_ERROR.name] for run in real}},
        "aliases": [{"name": name, "base": type(alias.metric).__name__, "config": alias.metric.config(),
                     "unit": alias.unit, "signal": "/".join(alias.signal)} for name, alias in can11.aliases.items()],
        "dedup": {"aliases": len(can11.aliases), "computed": can11.computations},
        "inputs": {cls.__name__: cls.Input.__name__ for cls in (ErrorStat, StopStat, TimeOffset, CodeShare,
                                                                Repeatability, PerformanceGain)},
        "repeatability": repeatability_block(real),
        "performance_gain": performance_gain_block(),
    }


# WHY this glance: 0.1.0's own landing-page glance (robot_metrics @ v0.1.0, website/gallery.py GLANCE), its statements
# and comments kept, extended to round 2's eight steps with 0.1.0's own calls. Nothing renamed: fold, close, strata
# and the Collection repr stay, because this column shows 0.1.0 as it is. Two changes, both the contract's: the data
# comes from round 1's fixture (0.1.0's page reads its own port of it), and each step sits under a plain heading.
GLANCE = ("API at a glance", "robot_metrics 0.1.0 as published: its own landing-page glance, extended to the eight "
          "steps with its own calls, nothing renamed. Every output is from CAN 11's play 32 and its 42 real runs.", '''
# load the real runs: round 8's 252 plays on the 1-DOF test rack, read from /archive
from fixture import pendulum_runs
from fixture_runs import as_run         # one play as 0.1.0's Run: its streams + labels
from robot_metrics import Objective, catalog, summarize

# the pendulum arm's metrics (CAN 11's arm, 339 mm to the tip), each with its unit
arm = catalog.pendulum_arm(tip_length_m=0.339).collection()
arm
{name: alias.unit for name, alias in arm.aliases.items()}

# one recording, offline: CAN 11, rung 0.9, a repeat
run = as_run(pendulum_runs("real")[32])
run
offline = arm.values(arm.fold(run))     # offline: the whole recording, one fold
offline["tip stop error"]               # mm, at the tip

# the same recording, live: fed in chunks of 7 rows as they would arrive
state = arm.empty()                     # live: the same fold, 7 samples at a time
for chunk in run.chunks(7):
    state = arm.update(state, chunk)
live = arm.values(arm.close(state))

# equal: live and offline give the same numbers
live == offline

# many runs into a table: CAN 11's 42 runs, one row per rung
can11_runs = [as_run(play) for play in pendulum_runs("real") if play.station == "can11"]
strata = summarize(arm, can11_runs, by=("station", "rung"))
for (station, rung), stratum in sorted(strata.items()):
    tail = stratum.tails["tip stop error"]
    print(f"rung {rung}: stops median {tail.median:.2f} mm, worst {tail.worst:.2f} mm")

# Zach's 1 mm tip-stop gate, outside the metric: on the median stop, and on every stop
median_stop = Objective("tip stop error", "<=", 1.0, unit="mm", on="value", owner="Zach")
every_stop = Objective("tip stop error", "<=", 1.0, unit="mm", share=1.0, on="items")
median_stop.title, every_stop.title

# its verdict per rung: the median stop, and every stop with the worst one
for (station, rung), stratum in sorted(strata.items()):
    median_verdict = median_stop.judge(stratum).verdict
    every_stop_result = every_stop.judge(stratum).message
    print(f"rung {rung}: median {median_verdict} · every stop: {every_stop_result}")
''')

CELLS = [
    ("setup: imports, then the real data", "Everything the session uses is imported here, and the data's origin is "
     "named: round 1's fixture and calibration readers, 0.1.0's public API, its two view sinks.", '''
import pathlib

import calibration                    # round 1: the friction ladder's held-out runs
from fixture import chain_runs, pendulum_runs, stations    # round 1: runs from /archive
from fixture_runs import as_run       # one fixture run as 0.1.0's Run: streams + labels
from robot_metrics import DEGREES, Alias, Collection, ErrorStat, HeldOutPair
from robot_metrics import CodeShare, PerformanceGain, Repeat, Repeatability, StopStat
from robot_metrics import TimeOffset, catalog, highest_passing, summarize
from robot_metrics.adapters import rerun as rerun_sink
from robot_metrics.adapters import wandb as wandb_sink
from robot_metrics.views import plot

# round 8 of the 1-DOF test rack: 252 real plays, and the twin's answer to each
real_runs = [as_run(play) for play in pendulum_runs("real")]
sim_runs = [as_run(play) for play in pendulum_runs("sim")]
len(real_runs), len(sim_runs)

# each station's lever (m): the tip moves tip length x the joint angle
tip_lengths = {name: facts["tip_length_m"] for name, facts in sorted(stations().items())}
tip_lengths

# the five-bar's speed ladder of 2026-11-10: the real arm's rungs, then its sim twin's
chain = [as_run(rung) for rung in chain_runs("real") + chain_runs("sim")]
len(chain)

# where the view's picture and .rrd go (gitignored)
REPORT_MEDIA = pathlib.Path("/home/bam/robot_models/reports/media")
MEDIA = REPORT_MEDIA / "robot-models-metrics-api-proposals-2026-11-18"
MEDIA.mkdir(parents=True, exist_ok=True)
'''),
    ("a part type's metrics", "The catalog: each row a name, a display unit and the stock metric behind it. The arm "
     "is the servo plus a lever, so its tip rows read the servo's results; one card carries Zach's Metric fields.", '''
servo = catalog.integrated_servo()
print(servo.table())

can11_arm = catalog.pendulum_arm(tip_length_m=tip_lengths["can11"])
print(can11_arm.table())

print(catalog.rigid_chain(("j2", "c1")).table())

can11 = can11_arm.collection()
can11.aliases["tip stop error"].card()
can11.aliases["all stops settled"].card()["value_type"]
'''),
    ("one recording, offline and live", "The same arm fed the whole recording (fold) and fed it 7 rows at a time "
     "(empty, update per chunk, close). Every stop lasts 25 rows or more, so 7-row chunks cut through each one.", '''
run = real_runs[32]                       # CAN 11, rung 0.9, a repeat
offline = can11.values(can11.fold(run))

state = can11.empty()
for chunk in run.chunks(7):
    state = can11.update(state, chunk)
live = can11.values(can11.close(state))

names = ["servo/tracking RMS", "servo/tracking peak", "tip stop error", "tip stop worst",
         "tip settle", "servo/overshoot", "servo/lag", "servo/stops"]
print(f"{'':<20} {'offline':>10} {'live':>10}")
for name in names:
    offline_value, live_value = offline[name], live[name]
    unit = can11.aliases[name].unit
    print(f"{name:<20} {offline_value:10.6f} {live_value:10.6f}  {unit}")
live == offline

# lag needs the whole run (a cross-correlation): 0.1.0 marks it end-of-run
can11.aliases["servo/lag"].metric.live
'''),
    ("runs to a deployment, gates applied", "One arm per station (each its own lever); summarize() groups each "
     "station's runs by rung. The catalog's three gates judge every rung cell, each objective with its owner.", '''
strata = {}
for station, tip_length_m in tip_lengths.items():
    station_arm = catalog.pendulum_arm(tip_length_m).collection()
    station_runs = [run for run in real_runs if run.labels["station"] == station]
    strata.update(summarize(station_arm, station_runs, by=("station", "rung")))
len(strata)

gates = catalog.pendulum_gates()
for name, gate in gates.items():
    for objective in gate.objectives:
        print(f"{name:<22} {objective.title:<36} {objective.owner}")

passed = {name: {} for name in gates}
for key, stratum in strata.items():
    for name, gate in gates.items():
        passed[name][key] = gate.judge(stratum).verdict == "pass"
for name, cells in passed.items():
    print(f"{name:<22} {sum(cells.values())} of {len(cells)} rung cells pass")

tracks_and_stops = {}
for key in strata:
    tracks_and_stops[key] = passed["tracks"][key] and passed["tip stop (median)"][key]
highest_passing(tracks_and_stops)

rungs = sorted({rung for station, rung in strata})
header = "worst stop mm"
for rung in rungs:
    header += f"{rung:>7}"
print(header)
for station in tip_lengths:
    row = f"{station:<13}"
    for rung in rungs:
        worst_mm = strata[(station, rung)].values["tip stop worst"]
        row += f"{worst_mm:7.2f}"
    print(row)
'''),
    ("the twin's output (sim vs target)", "The same arms and gates on the twin's answers to the same plays.", '''
sim_strata = {}
for station, tip_length_m in tip_lengths.items():
    station_arm = catalog.pendulum_arm(tip_length_m).collection()
    station_runs = [run for run in sim_runs if run.labels["station"] == station]
    sim_strata.update(summarize(station_arm, station_runs, by=("station", "rung")))

print("CAN 11 rung   RMS real °   RMS sim °   stop real mm   stop sim mm")
for rung in rungs:
    real_values = strata[("can11", rung)].values
    sim_values = sim_strata[("can11", rung)].values
    rms_real = real_values["servo/tracking RMS"]
    rms_sim = sim_values["servo/tracking RMS"]
    stop_real = real_values["tip stop error"]
    stop_sim = sim_values["tip stop error"]
    rms_text = f"{rms_real:12.3f} {rms_sim:11.3f}"
    stop_text = f"{stop_real:14.3f} {stop_sim:13.3f}"
    print(f"{rung:11.1f} {rms_text} {stop_text}")

median_gate = gates["tip stop (median)"]
sim_verdicts = [median_gate.judge(stratum).verdict for stratum in sim_strata.values()]
sim_verdicts.count("pass"), len(sim_verdicts)
'''),
    ("the chain, a view, a new metric", "The five-bar through the catalog's rigid chain; a view of every CAN 11 "
     "stop to matplotlib, the same result as a W&B payload and a Rerun .rrd; a new row is one more Alias.", '''
chain_metrics = catalog.rigid_chain(("j2", "c1")).collection()
for rung_run in chain:
    values = chain_metrics.values(chain_metrics.fold(rung_run))
    tool_mm, worst_mm = values["tool tracking RMS"], values["tool worst"]
    print(f"{rung_run.id:<34} tool RMS {tool_mm:6.2f} mm, worst {worst_mm:6.2f} mm")

states_by_rung = {}
for rung in rungs:
    states_by_rung[f"rung {rung:g}"] = strata[("can11", rung)].merged
stops_view = can11.view(
    "tip stop error",
    states_by_rung,
    title="CAN 11 · every stop at the tip, by rung (round 8, real)",
    target=1.0,
    target_label="Zach's 1 mm",
)
len(stops_view.y), stops_view.hint
figure, axes = plot(stops_view, path=MEDIA / "baseline-tip-stops.png")

payload = wandb_sink.payload(stops_view, "can11/tip_stop_mm")      # built, not sent
payload["summary"]
rrd_path = MEDIA / "baseline-tip-stops.rrd"
rrd = rerun_sink.write_rrd(stops_view, "metrics/can11/tip_stop_mm", rrd_path)
rrd.name

velocity_error = ErrorStat(statistic="rms", rows="moving", field="velocities")
velocity_alias = Alias("velocity error RMS", velocity_error, unit="deg/s", scale=DEGREES)
with_velocity = Collection([*can11.aliases.values(), velocity_alias])
velocity_values = with_velocity.values(with_velocity.fold(run))
velocity_values["velocity error RMS"]
can11.computations, with_velocity.computations
'''),
    ("other inputs: repeats and a calibration", "Each stock metric declares what it takes in. Repeatability takes "
     "one real repeat at a time; PerformanceGain takes held-out runs played with nominal and with fitted values.", '''
inputs = (ErrorStat, StopStat, TimeOffset, CodeShare, Repeatability, PerformanceGain)
{metric.__name__: metric.Input.__name__ for metric in inputs}

repeats = []
for repeat_run in real_runs:
    labels = repeat_run.labels
    same_cell = labels["station"] == "can11" and labels["rung"] == 0.9
    if same_cell and labels["kind"] == "repeat":
        streams = repeat_run.streams
        repeats.append(Repeat(("can11", 0.9), streams["target"], streams["state"]))
len(repeats)
repeatability = Repeatability(k=3.0)
repeatability_rad = repeatability.compute(repeatability.fold(repeats))
repeatability_rad * tip_lengths["can11"] * 1e3      # mm at the tip

residual_rms = ErrorStat(statistic="rms", field="effort")
residual = Alias("feed-forward residual RMS", residual_rms, unit="N·m")
nominal_runs = tuple(as_run(plateau) for plateau in calibration.held_out_runs("nominal"))
fitted_runs = tuple(as_run(plateau) for plateau in calibration.held_out_runs("fitted"))
gain = PerformanceGain(metric=residual)
held_out = HeldOutPair(nominal=nominal_runs, fitted=fitted_runs)
gain_state = gain.update(gain.empty(), held_out)
gain.sides(gain_state)                    # N·m on the held-out runs: nominal, fitted
gain.compute(gain_state)                  # nominal minus fitted
'''),
]


def main() -> None:
    # WHY two fresh namespaces (round 2's contract): round 1 ran every cell in this module's globals, so a page could
    # use a name it never imported and still "run"; here the glance and the session each start from nothing
    session = Session({"__name__": "__main__"})
    session.cell(*GLANCE, kind="glance")
    session.namespace = {"__name__": "__main__"}
    for label, caption, code in CELLS:
        session.cell(label, caption, code)
    session.write(HERE)


if __name__ == "__main__":
    main()
