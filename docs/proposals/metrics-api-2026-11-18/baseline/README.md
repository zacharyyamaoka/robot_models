# baseline · robot_metrics 0.1.0 exactly as published

Round 2's comparison column. The API does not change here: `robot_metrics` is the published package, read through a
symlink (`robot_metrics -> /home/bam/robot_metrics/src/robot_metrics`). Importing `scenario.py` asserts
`__version__ == "0.1.0"` and that `git diff --quiet v0.1.0 -- src` is clean, so the column is the tagged release
(2573912) byte for byte, whatever moves on the repo's HEAD.

**Verdict:** parity PASS on round 8 (zero deviation except 2.8e-17 mm on rung stops), both displayed blocks run as
shown (2 of 2), self-score **60.6 of 100**. It loses where this round leads: legibility at a glance scores 1. A newcomer
meets fold, state, close, strata, tails, aliases and a `Collection(15 aliases over 5 states)` repr before the first
verdict.

## The direction

- **The face:** round 1's v4 + v1 splice as released. A metric is a frozen settings object whose running total is a
  value: `empty` / `update(state, chunk)` / `close` / `merge` / `compute`. `fold(run)` does all of that for a whole
  recording. A part's rows are `Alias`es over stock metrics (`catalog.pendulum_arm(...)`), and `summarize(...,
  by=(...))` groups runs. Gates are OpenSLO-style `Objective`s judged outside the metric.
- **The glance:** 0.1.0's own landing-page glance (`website/gallery.py` GLANCE at v0.1.0) with its statements and
  comments kept, extended to the contract's eight steps using 0.1.0's own calls. Nothing is renamed. The working-tree
  rewording that robot_metrics' website is getting today (`state` -> `tally`, `strata` -> `by_rung`) is post-release, so
  this column does not use it.
- **The one piece of glue:** [fixture_runs.py](fixture_runs.py) `as_run(play)` copies round 1's fixture run into
  0.1.0's `Run` (the same streams under 0.1.0's names, station folded into the labels). 0.1.0's page reads the same
  plays through its own port of that fixture (`robot_metrics.datasets.onedof_round8`). This column reads the shared
  fixture instead, as the contract asks, so `pendulum_runs("real")[32]` is the same object in every glance. Same run id,
  same 0.23094052947652116 mm either way.

## Prior art

Round 1's v4 + v1 splice, which drew on these:
- **torchmetrics:** the `update` / `compute` / `reset` face (0.1.0's `CollectionStream`), `merge_state`, compute groups.
- **clu / torcheval:** state as an immutable value.
- **OpenSLO:** occurrence objectives ("100 % of stops <= 1 mm").
- **RobotDx:** owners, and `pass` / `fail` / `not_run`.
- **ISO 9283:** repeatability.

## How live and offline work, honestly

- **Offline:** `arm.values(arm.fold(run))` is one pass over the whole recording.
- **Live:** `state = arm.empty()`, then `state = arm.update(state, chunk)` per chunk, then `arm.values(arm.close(state))`.
  On CAN 11's play 32, fed 7 rows at a time, it gives `live == offline` -> `True`, equal bit for bit on all 15 numbers.
  The frozen check holds it to 1e-9.
- **Stops survive chunk boundaries:** yes. The stop tracker carries the open dwell across chunks as part of the state.
  Every stop lasts at least 0.25 s (25+ rows), so 7-row chunks cut through every one of them, and the stops still match
  the offline ones.
- **Memory is not bounded.** Measured after streaming play 32 (318 rows) in 7-row chunks: the lag metric
  (`TimeOffset`, a cross-correlation, `live=False`) holds both whole streams (636 samples) until `close`. The stop metric
  keeps each stop's move rows for settle and overshoot (607 samples). The error statistics hold 4 numbers each. So the
  state grows with the run, about 4x the run's rows here.
- **Time alignment:** 0.1.0 has an explicit `align` / `Aligner` step with a tolerance, and its own reader uses it (the
  hermite rule). In this column the fixture already puts the target at each reply's instant, so no metric aligns
  anything. `TimeOffset` reads the plan on its own ticks.

## Gates (G1-G5)

- **G1 parity:** `check_parity.py baseline` -> PASS. check_scenario.py md5 7d9ced14 (frozen). Max deviation:
  real_rungs.stop_tool_mm 2.78e-17, sim_rungs.stop_tool_mm 3.47e-18, everything else 0. Gates 31 / 36 / 33 of 36, highest
  rung CAN 11 1.0 and the rest 0.9, repeatability 0.387 mm at CAN 11's tip, gain 0.2220 -> 0.0042 N·m.
- **G2 runs as shown:** `check_standalone.py` on a one-variant page drawn from `session.json` with round 1's
  `flow_html` -> 2 of 2 PASS (glance 21 statements, session 96). A mutated output (0.231 for 0.23094052947652116)
  -> FAIL, so the check can fail. The round's gallery page is the dispatcher's to build.
- **G3 Metric fields:** `Alias.card()` has name, description, references, function, unit, `target: None` (the target
  lives on `Objective`), and `value_type` number or bool (session cell 2).
- **G4 live = offline:** above.
- **G5 light core:** 0.1.0 imports numpy, robot_msgs and the stdlib. matplotlib and rerun load only inside `plot` and
  `write_rrd`, and the W&B payload is built, never sent. No metric reads a model.

## Self-score against the contract's criteria

| id | weight | score | evidence |
|---|---|---|---|
| lg | 30 | 1 | Runs as shown and every name is imported, but fold, state / empty, close, strata, tails, aliases, `on="items"` / `share` and the `Collection(15 aliases over 5 states)` repr carry the meaning, unexplained (the list is in scores.json) |
| fr1 | 14 | 3 | `catalog.pendulum_arm()` lists the arm's 15 rows, each a stock metric with name, unit, description and references, but in `robot_metrics.catalog`, away from the part type. Wiring it into robotparts / robot_models is PEP 0001's next step |
| fr2 | 14 | 3 | One call per recording (`values(fold(run))`), and one `summarize(..., by=...)` per grouping with each alias's `across` rule (pool / median / max). A six-lever deployment is still a hand loop over six collections (session cell 4). Between 3 and 5; the scale has no 4 |
| fr3 | 10.5 | 3 | Exact (bit for bit) and stops safe across chunks, but the lag and stop states grow with the run (636 + 607 samples held for 318 rows) |
| fr4 | 10.5 | 5 | `Objective(alias, op, value, unit, share, on, owner)` judged outside the metric. The catalog's gates name owners (agent, Zach, proposed), and the median and every stop show side by side with the worst stop in the message |
| fr5 | 7 | 5 | A new metric is one `Alias(...)` over a stock metric (velocity error RMS, session cell 6). A part type is one catalog function returning `PartMetrics` |
| fr6 | 7 | 5 | PEP 0001 §7-8 says what happens to each copy: RobotDx's stock computes through `ErrorStat` / `StopStat` / `CodeShare` (started 11-17, byte-identical), sim2real.metrics.actuator is replaced, and the stop rule is proposed into `robot_msgs.compare` |
| fr7 | 7 | 5 | `collection.view()` -> `plot()` (matplotlib), the same `View` -> `adapters.wandb.payload` and `adapters.rerun.write_rrd`, `hint` carried (`distribution`) |

Total: Σ weight × score / 5 = **60.6**.

## What I'd change (without changing what 0.1.0 computes)

- **Lead with the face 0.1.0 already ships:** `stream = arm.stream()` with `update(chunk)` / `compute()` / `reset()`
  (`CollectionStream`) reads at a glance. `empty` / `update(state, chunk)` / `close` does not.
- **Rename the vocabulary, keep the math:** `fold(run)` -> `compute(run)` or `score(run)`; `state` -> a running total;
  `strata` -> groups; `tails` -> `stops` (n, median, worst); `on="items"` -> `every_stop=True`.
- **Readable reprs:** `Collection` should print its rows (name, unit, what it measures), and `summarize` should return
  something that prints as a table, so a glance never hand-writes a print loop.
- **A deployment in one call:** let `summarize` take the per-station lever from the run's context, so six stations need
  one collection, not a loop of six.
- **Bounded live state:** a windowed lag estimate for live use, and stops that keep only the dwell's tail and the settle
  crossing instead of every move row.

## Files

- [scenario.py](scenario.py): `run()` for the parity check, plus `GLANCE` and `CELLS`, recorded in two fresh namespaces
  with round 1's `_session.py`. Writes [scenario.out.txt](scenario.out.txt) and [session.json](session.json).
- [fixture_runs.py](fixture_runs.py): `as_run`, the field-for-field copy into 0.1.0's `Run`.
- [scores.json](scores.json): the self-score and the honest `lg_unexplained` list, read by the gallery.
- `robot_metrics`: the symlink to the published package. Never edited.
- The view's picture and `.rrd`: `reports/media/robot-models-metrics-api-proposals-2026-11-18/baseline-tip-stops.*`
  (gitignored, rewritten by every run).

Rerun the session:

```bash
uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/baseline/scenario.py
```
