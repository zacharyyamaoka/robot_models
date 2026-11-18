# v1 · robot_metrics: torchmetrics, ported to numpy

**Thesis.** A literal port of torchmetrics' design: a `Metric` base class whose constructor is the config, states
declared with `add_state(name, default, reduce)` (sum / max / min / cat), `update` / `compute` / `reset` /
`merge_state`, wrappers (`PerRun`), and a `MetricCollection` with compute groups. It ships as its own `robot_metrics`
package on robot_msgs; robotparts' part type lists configured aliases as `.metrics` beside `.models`.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v1_robot_metrics/scenario.py
```

`check_scenario.py v1_robot_metrics` → **PASS**: every per-play and per-rung number equals round 8's scorecard within
2.2e-16 deg and 2.8e-17 mm, the sim side too, the five-bar's ladder to 1.9e-16 relative. Transcript:
[scenario.out.txt](scenario.out.txt).

## Where it lives and what it reads

- **Layering:** `robot_msgs <- robot_metrics <- robotparts` (`RobotPart.metrics`), with sim2real and RobotDx consuming.
  robot_models never imports it (Zach: "as we are in robot models all there really is is models").
- **Run data:** `RunRecording` = a stable id, a named map of robot_msgs trajectories (`target`, `state`, `plan`,
  `tool/target`, `tool/state`), labels, an `.rrd` pointer. This is the run-data MOC's recommendation (the run folder is
  the contract; streams are a named map; the alias names which pair it compares).
- **Input types:** each class declares `Input`: `ErrorStat` / `DwellError` a tracked pair, `Repeatability` a `Repeat`
  (one real repeat of one command, refused otherwise), `PerformanceGain` a `HeldOutPair`.

## How it works

- **Streaming state:** sufficient statistics (sum of squares, count, running max) plus the shared `StopTracker`, which
  carries the open dwell across chunk boundaries. Only statistics that cannot reduce keep a "cat" list (medians, the
  lag's cross-correlation), and each says so in its `memory` attribute. One live update of the arm's 12 aliases costs
  1.4 ms per 7-row chunk.
- **Computed once:** an alias's identity is (signal, base class, frozen config). Dwell metrics with one dwell rule on one
  signal share one `Dwells` state (torchmetrics' compute groups): the arm's 12 aliases cost 5 computations.
- **Gates:** `Diagnostic(alias, Limits, owner)` and `Gate(name, diagnostics)` judge the values outside `compute()`,
  with RobotDx's rule (strictly beyond a bound trips; no value is `not_run`, never green).
- **Across runs:** a rung is its plays' collections merged with `merge_state` (pooled stops are exact); `PerRun(metric,
  "median")` gives the scorecard's median of per-play RMS.
- **Views:** `Metric.plot()` returns `(fig, ax)`; the same `View` becomes a W&B table payload and Rerun calls.

## What happens to the existing code

- **robotdx.diagnostics.stock:** its compute half becomes robot_metrics classes (the config dataclass fields become
  constructor arguments, `scale` / `unit` / `limits` leave the config for the alias and the Diagnostic). RobotDx keeps
  `Limits`, `judge`, `Result` and records, and judges robot_metrics values.
- **sim2real.metrics.actuator:** deleted; its callers import `robot_metrics.functional`. Its stop rule and the
  pendulum's are one `DwellRule(include_last_dwell=...)`.
- **pendulum_score / traj_compare:** pendulum_score's per-play numbers are this collection; traj_compare keeps drawing
  and reads the chain's aliases.
- **CalibrationOutcome** lives in `robot_metrics.records`.

## Cost of adding

- **A metric:** usually one `Alias` over a stock base (the session adds velocity-error RMS in one line). New math is a
  `Metric` subclass with `setup` (states), `_update`, `compute`.
- **A part type:** one `PartType(name, models, signals, metrics)`.

## Best when / loses when

**Best when** you want torchmetrics' muscle memory exactly, and live monitors that run for hours (bounded state).
**Loses when** the state hides in attributes: merging needs every state's reduce rule right, `compute()` mid-run has to
deep-copy the open tracker, and `merge_state` refuses collections with different configs (stations with different tip
lengths merge only through their gates, not their states).
