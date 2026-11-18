# v5 · Part tree: each RobotPart judges its own output and reads its children's

**Thesis.** The deployment is a tree of RobotParts (the rack, its six pendulum arms, each arm's servo; the five-bar
and its two joints). Each node has `.models`, `.metrics` (aliases on its own signal) and `.children`. An assembly's
aliases read a child's computation through context (the arm's lever), so the servo computes and the arm shows: the
vault note's open question "a 1-joint chain judged at its tip, or a servo with a load term?" becomes "both". The state
is segment events (every move and dwell, with its statistics and its stop), the things a drill-down opens.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v5_part_tree/scenario.py
```

`check_scenario.py v5_part_tree` → **PASS** (scorecard parity within 2.2e-16 deg; the chain to 2.8e-15 relative, the
events sum rows one at a time). Transcript: [scenario.out.txt](scenario.out.txt).

## Where it lives and what it reads

- **Layering:** `robot_msgs <- robotparts` (RobotPart: `.models` from robot_models, `.metrics` here, `.children`)
  `<- sim2real / RobotDx`. The deployment's tree is its robot description's.
- **Run data:** named robot_msgs streams per run, routed to each node by its `signals`; a node's `ledger` keeps each
  run's labels and event logs.
- **Input types:** `TrackingError`, `StopEvents` read events; `Lag` and `CodeShare` the run's raw pairs (context);
  `Repeatability` a node's event logs of one command; `PerformanceGain` two event logs of the same held-out runs.
  Context (the part, its lever) is passed beside the data, as nuPlan's `compute(history, scenario)`.

## How it works

- **Streaming state:** each node owns one `Segmenter` per signal; the open move or dwell carries across chunks; a stop
  attaches to the dwell event that holds its first row. One live update of the arm subtree costs 0.71 ms. A metric
  used alone has its own `update` / `value` / `reset`.
- **Computed once:** composition is the dedup: the arm owns no segmentation (all 6 of its aliases read the servo's),
  the servo owns 2 (its output, and the raw plan / state for the lag). 12 aliases, 2 segmentations.
- **Gates:** `Check(alias, high, owner, level)` per node; levels OK / WARN / ERROR / STALE (diagnostic_msgs); a parent's
  level is its worst child's, and the reason is named at the node that failed.
- **Across runs:** `node.groups(by)` and `node.values(entries)`, each alias by its own rule.

## What happens to the existing code

- **robotdx.diagnostics.stock:** its catalog becomes node declarations in robotparts; its formulas move to the event
  metrics here, so RobotDx would depend on robotparts for them (the cost of this home).
- **sim2real.metrics.actuator:** deleted. **CalibrationOutcome:** a robot_msgs message.

## Best when / loses when

**Best when** the deployment page should read as the robot reads (rack → arm → servo), with every red cell naming the
part that failed and the event to open. **Loses when** a metric does not fit the move / dwell event model (a camera's
single-instant multi-DOF error), when the scenario's roll-ups need hand-written tables over the ledgers, and because
the formulas sit high in the stack (above robot_models), where RobotDx and sim2real must reach up for them.
