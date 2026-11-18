# v3 · Signals and aliases: math in robot_msgs, meaning in the part's catalog

**Thesis.** "Everything is just signals": the metric classes are pure signal math in `robot_msgs.metrics`, beside
`compare`, and know no part type, display unit or threshold. A part's catalog gives them meaning as aliases. The
recording arrives as raw topics (the command on its ticks, the state as replies), and an `Aligner` pairs them live,
holding each reply until the next command knot arrives. Aliases compile into a `Plan` that runs each (signal, base,
config) once.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v3_signals/scenario.py
```

`check_scenario.py v3_signals` → **PASS** (scorecard parity at float round-off). The live aligner reproduces the
scorecard's `align: measured` target (the plan's Hermite at each reply's instant) exactly. Transcript:
[scenario.out.txt](scenario.out.txt).

## Where it lives and what it reads

- **Layering:** `robot_msgs (+ .metrics) <- robotparts' catalog` (aliases, Checks, owners: RobotDx's `catalog.py`
  moves here) `<- sim2real / RobotDx`.
- **Run data:** `Recording` = topics (MCAP-like), each a robot_msgs trajectory; `messages()` replays them in arrival
  order. A signal names its two topics and its alignment rule (`hermite` for the rack, `same_clock` for the five-bar
  and the sealed ladder).
- **Input types:** `RootMeanSquare`, `StopStatistic` an `AlignedPair`; `CrossCorrelationLag` the raw topics;
  `Repeatability` a `Repeat`; `PerformanceGain` a `HeldOutPair`.

## How it works

- **Streaming state:** the aligner holds at most one reply (a 1-tick latency, stated); metrics keep sufficient
  statistics, the `Stops` node the open dwell. One message pair (a knot and a reply) costs 0.66 ms.
- **Computed once:** `Plan.compile` deduplicates by (signal, identity); every `StopStatistic` reads one shared
  `Stops` node. The arm: 12 aliases, 11 nodes, 5 computations including 1 stop pass.
- **Gates:** `Check(alias, Limits, owner)` rows in the catalog, judged on the point estimates.
- **Across runs:** each alias says how runs combine (median, max, or `pooled:<statistic>` over stops and rows);
  `aggregate()` adds a seeded bootstrap interval and the IQM, rliable-style.

## What happens to the existing code

- **robotdx.diagnostics.stock:** its math moves down into `robot_msgs.metrics`; its `catalog.py` becomes robotparts'
  alias catalog. **sim2real.metrics.actuator:** deleted, callers use robot_msgs.metrics.
- **CalibrationOutcome** is a robot_msgs message (`robot_msgs.calibration`).

## Best when / loses when

**Best when** live streams arrive on their own clocks and alignment must be explicit, and the math should sit at the
lowest layer. **Loses when** robot_msgs grows a metrics framework it was not meant to hold (the prior-art note keeps
the math in `compare` and the classes in robot_metrics), and when an interval is read as a gate: the IQM trims
CAN 11's failing stops (the note's §1), so the gates here read point values, not the intervals.
