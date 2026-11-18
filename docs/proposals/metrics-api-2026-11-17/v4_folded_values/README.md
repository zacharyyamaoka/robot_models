# v4 · Folded values: the state is a value, the gate an occurrence objective

**Thesis.** The prior-art note's recommended core, grown to the whole scenario. A metric is a frozen dataclass
(identity = class + fields, hashable) whose state is an immutable value: `empty`, `update(state, chunk) -> state`,
`merge`, `close`, `compute`. Live is a fold over chunks, offline the same fold once, and runs, rungs and stations merge
with no extra code. Gates are OpenSLO occurrence objectives ("100 % of stops <= 1 mm", or a value gate), judged into
RobotDx's own `Result`.

```bash
uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v4_folded_values/scenario.py
```

`check_scenario.py v4_folded_values` → **PASS** (scorecard parity at float round-off). Transcript:
[scenario.out.txt](scenario.out.txt).

## Where it lives and what it reads

- **Layering:** a `robot_metrics` package on robot_msgs, published from io-diagnostics beside RobotDx (the metric half
  of `robotdx.diagnostics` lifted out, its parity tests in the same CI); robotparts lists aliases as `.metrics`.
- **Run data:** `Run`, the run-data MOC's recommendation in full: id, named streams, labels, provenance (real / sim /
  replay, carried though no metric reads it), outcome, links (`rrd` through the 8771 launcher, `wandb`).
- **Input types:** `ErrorStat`, `Dwell` an aligned pair (checked: same instants within `align_tolerance_s`);
  `TimeOffset` the plan and the state (it finds the offset, so it cannot demand alignment); `Repeatability` a `Repeat`;
  `PerformanceGain` a `HeldOutPair`.

## How it works

- **Streaming state:** tuples and frozen records; the open dwell's tracker is copied into each new state. Each metric
  declares `live` (lag: false, needs the run) and a `version` stored with its values. One live update of the arm's 12
  aliases costs 0.97 ms. `Stream(metric)` is the torchmetrics face (`update` / `compute` / `reset`).
- **Computed once:** the collection keys states by (signal, metric) (a frozen metric is its own key) and dwell metrics
  by (signal, rule): the arm's 12 aliases are 5 states.
- **Gates:** `Objective(alias, op, value, share, on="items" | "value", owner)`. The same alias "tip stop error" carries
  the scorecard's median gate (on its value) and Zach's 1 mm on every stop (on its items). Verdicts are RobotDx's
  `Result` (verdict, code, view hint), loaded from io-diagnostics' `core.py`, not copied.
- **Across runs:** `summarize(collection, runs, by=("station", "rung"))`: per stratum the merged states and the runs'
  own values; a tail per stratum for gates; Wilson intervals for bool metrics (CAN 11: 38 of 42 runs settled every
  stop, 95 % 0.78-0.96).

## What happens to the existing code

- **robotdx.diagnostics.stock:** its formulas become these classes, with one fix the note names: `limits` leave every
  config, so a new target never changes a metric's identity. RobotDx keeps `Limits`, `judge`, `Result`, records.
- **sim2real.metrics.actuator:** deleted, callers use robot_metrics. **CalibrationOutcome:** a robot_msgs message.

## Best when / loses when

**Best when** states must be stored, resumed, merged across machines or rounds, and gates must state tails honestly.
**Loses when** a new base costs four methods (`empty`, `update`, `merge`, `compute`), more ceremony than v1's
`add_state`, and when the open tracker is large (each update copies it).
