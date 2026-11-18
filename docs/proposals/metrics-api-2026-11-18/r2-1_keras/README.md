# r2-1 · Keras / torchmetrics face

Metric objects with the three verbs every ML engineer already knows: `metrics.update(target, measured)` as rows
arrive, `metrics.compute()` for the numbers, `metrics.reset()` to start over. A part's metrics are a dict of those
objects keyed by name and unit; the gate is a separate `Threshold` that reads the computed numbers from outside.

- Parity on round 8: **PASS** (`check_parity.py r2-1_keras`; worst deviation 2.2e-16 deg, 0 on stops, 2.8e-15 relative
  on the chain).
- Runs as shown: **2 of 2** blocks (the glance, 21 statements; the session, 86 statements) in a fresh `python -I`.
- The glance: 22 lines of code, the contract's 8 steps, nothing defined off the page.

## Direction: the face

```python
from fixture import pendulum_runs                   # round 1's folder: round 8's real runs
from part_metrics import pendulum_arm_metrics       # this folder

run = pendulum_runs("real")[32]                     # station can11, rung 0.9
metrics = pendulum_arm_metrics(tip_length_m=0.339)  # a dict: "tip stop worst (mm)" -> StopError(...)
metrics.update(run.target, run.measured)            # offline: the whole recording in one call
metrics.compute()                                   # {"tracking RMS (deg)": 0.146, ..., "stops": 2}
metrics.reset()                                     # forget everything
```

| file | what it holds |
|---|---|
| [metrics.py](metrics.py) | the metric classes (`TrackingError`, `StopError`, `SettleTime`, `AllStopsSettled`, `StopCount`, `Overshoot`, `CodeShare`, `Lag`, `Repeatability`, `PerformanceGain`) and `MetricDict` |
| [part_metrics.py](part_metrics.py) | each part type's metrics: `integrated_servo_metrics()`, `pendulum_arm_metrics(tip_length_m)`, `rigid_body_chain_metrics()`, and `PART_TYPES` (models beside metrics) |
| [rung_table.py](rung_table.py) | `rung_table(runs, make_metrics)`: one row per station and rung, fresh metrics per cell |
| [threshold.py](threshold.py) | `Threshold(at_most, unit, owner)` and its per-cell verdicts |
| [rack_gates.py](rack_gates.py) | the rack's three gates as thresholds with owners, and the highest rung per station |
| [scenario.py](scenario.py) | `run()` for the parity check, `GLANCE` and `CELLS` for the gallery |

Decisions taken inside the direction:
- **Every metric is one number or one bool**, as in Keras. The tip stop's median and worst are two `StopError` objects
  (`statistic="median"`, `statistic="worst"`), not one object with two outputs.
- **Units live on the configured object** (`unit="mm", scale=339.0`), never in the class or the math (the stop finder
  and the sums are SI). One class serves several names: `TrackingError` for RMS and peak, `StopError` for median,
  worst and p90.
- **`MetricDict` is a real `dict`** (so `list(metrics)`, `metrics["tip stop worst (mm)"]` and `.items()` work) with
  update / compute / reset added. It is torchmetrics' `MetricCollection`, named for what it is.
- **The one keyword beyond Keras:** `update(..., new_run=True)` starts the next recording inside the same metric. A
  chunk that goes back in time without it raises, so two recordings are never glued together silently (shown in the
  session). The glance never needs it.
- **Lag reads the plan as sent** (`update(run.plan, run.measured)`, or `MetricDict.update(..., plan=run.plan)`), not
  the target at each reply, because that is the oracle's definition.

## Prior art

- **keras.metrics**: `update_state(y_true, y_pred)` / `result()` / `reset_state()`, one number per metric, the
  constructor holds the config (`MeanSquaredError(name=...)`). Taken: the verbs, one number per object, constructor
  config, `name`. Not taken: Keras puts thresholds inside some metrics (`Precision(thresholds=0.5)`); here a threshold
  never enters a metric.
- **torchmetrics**: `update(preds, target)` / `compute()` / `reset()`; `MetricCollection` with compute groups (members
  with the same state are updated once); wrappers for per-sample statistics. Taken: the verbs, the collection as a
  dict, compute groups (the five stop metrics in a dict share one `StopFinder`, so 7 metrics cost 3 computations per
  chunk). Not taken: tensors, `add_state` / `merge_state`, `nn.Module`.
- **torchmetrics itself is not imported.** Round 1 measured it: 3.9 s and 450 MB at import, 2.7 GB of venv. numpy +
  robot_msgs does the work here; the core imports numpy, robot_msgs and the stdlib only (matplotlib and Rerun load
  only inside round 1's `_viz` sinks).

## Live and offline, honestly

- **Offline** is one `update(run.target, run.measured)` on the whole recording. **Live** is the same object fed chunks
  (`run.target[start:start + 7]`); `compute()` works at any time, mid-run included.
- **Exactly equal, not just close.** `live == offline` is `True` on CAN 11's play 32 for every metric. The RMS is a
  running sum in row order, so 7-row chunks and the whole recording add the same numbers in the same order; the stop
  rule is round 1's per-sample `stock.StopTracker`, so a stop that straddles a chunk boundary is found the same way.
- **Mid-run, the stop in progress counts as if the run ended now** (`compute()` closes a copy of the tracker; the
  state never changes). That is what makes the last `compute()` of a live run equal the offline value.
- **Memory.** Tracking keeps running sums: constant. Stop metrics keep one small record per stop (error, settle time
  per band asked for, overshoot) plus the rows of the move in progress; the move rows are dropped as each stop closes.
  A median needs every stop, so this grows with the stop count, never with the row count. `Lag` keeps every row it
  is given (a cross-correlation needs the run), and `Repeatability` keeps each repeat's stop errors; both say so.
- **Time alignment is stated, not done.** `update` refuses a target and measured that are not sampled at the same
  times ("align them first"); round 1's fixture puts the target at each reply's instant, the scorecard's own rule.
  A chunk that goes back in time without `new_run=True` is refused.
- **Many runs:** one fresh `MetricDict` per rung cell (`rung_table`), fed each of the cell's plays with
  `new_run=True`. Each metric combines runs by its own stated rule (`across_runs`, on every card): tracking RMS is
  the median of the plays' RMS, a peak the worst row of any play, every stop metric pools all the plays' stops (the
  rung's median stop is the median of all 14 stops, never a median of medians).

## Where it would live, and the three copies

- `metrics.py` → robot_metrics (this face replacing 0.1.0's empty / update / merge / close vocabulary); the stop rule
  → `robot_msgs.compare.stops`, beside `goal` and `estimate_time_offset`.
- `part_metrics.py` → robot_models, each function beside its family (`IntegratedServo.metrics` next to its 14 models);
  `PART_TYPES` here shows the pairing.
- **The three copies:** `robot_msgs.compare` stays the formula home (goal, overshoot, lag, pose error, and the stop rule
  once it lands there). `sim2real.metrics` and `robotdx.diagnostics.stock` call it instead of their own stop and RMS
  code, and lose those copies. **One copy stays:** `TrackingError` keeps its own running sum of squares, because
  `robot_msgs.compare.error_stats` takes a whole array and cannot be fed chunks. That is two RMS implementations, said
  out loud.

## Self-score (1 / 3 / 5 against the contract's anchors)

| id | weight | score | evidence |
|---|---|---|---|
| lg | 30 | 5 | every name on the glance is imported on it, with the data's origin in a comment; the verbs are update / compute / reset and a function call; outputs are a dict of numbers keyed "name (unit)", two small tables and `True`. Unexplained: "RMS" (not spelled out) and "repeat" in the run's echo |
| fr1 | 14 | 5 | each part type is one function returning configured metric objects with name, description, references and unit; `PART_TYPES` pairs them with the models. The pairing is a dict in a proposal module, not yet an attribute on the ModelFamily |
| fr2 | 14 | 3 | one `update` per recording, one `rung_table` call for runs → rung, `across_runs` stated per metric; station and deployment are thresholds over the table (`judge`, `highest_rung`), not one roll-up call |
| fr3 | 10.5 | 3 | live == offline exactly, stops safe across chunk boundaries, alignment stated and enforced; memory grows with the stop count (and with every row for Lag), so not bounded |
| fr4 | 10.5 | 5 | `Threshold(at_most, unit, owner)` never enters a metric; owners Zach / agent / proposed on every line; the verdict shows median and worst stop per cell |
| fr5 | 7 | 5 | a new metric is one constructor call (`TrackingError("velocity error RMS", field="velocities", ...)`, in the session); a part type is one function |
| fr6 | 7 | 3 | says where each copy goes, but keeps a streaming RMS beside `robot_msgs.compare.error_stats` |
| fr7 | 7 | 5 | `StopMetric.view()` carries the data-ui hint ("distribution"); one view goes to matplotlib, a W&B payload and an .rrd |

Total: **87.4 / 100** (Σ weight × score / 5). The dispatcher re-scores lg from the glance alone.

## What I'd change

- **Run boundaries are the seam Keras does not have.** `new_run=True` is plain, but it is one more thing to know. A
  recording type that knows its own start (a `Run` passed whole) would remove it for offline use; live would still
  need it.
- **One object per number** makes the dict long (7 entries for the arm) and repeats `scale=` on every tip metric. A
  small `tip(...)` helper in part_metrics would shorten it; I kept every constructor visible instead.
- **`MetricDict` sharing has a sharp edge:** once metrics share a stop finder, update the dict, not one member.
  torchmetrics has the same rule for compute groups; a member could refuse a direct update.
- **Bounded stop memory** is possible only by giving up the exact median (a streaming quantile sketch), which fails
  parity. I would keep it exact.
