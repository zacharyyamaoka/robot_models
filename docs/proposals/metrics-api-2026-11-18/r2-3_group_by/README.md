# r2-3 · Group-by expressions

**A metric is an expression: what is measured, then how it is summarised.** `tip_stop_error.max()` is "the worst
stop"; `runs.group_by("station", "rung").agg(tip_stop_error.median(), tip_stop_error.max(), tracking_error.rms())`
returns a readable table, and Zach's 1 mm gate is a comparison on that table, with its owner. Parity on round 8:
PASS, at float round-off ([check_parity.out.txt](check_parity.out.txt)). Standalone: 2 of 2 displayed blocks run as
shown in a fresh `python -I` ([check_standalone.out.txt](check_standalone.out.txt)).

## The direction and its prior art

- **Polars expressions** ([`group_by().agg()`](https://docs.pola.rs/user-guide/expressions/aggregation/)): a column
  expression (`pl.col("x").median()`) is evaluated per group. Here the "column" is a measure of each recording (its
  stops, its moving rows) and the expression is its summary. Polars' own verbs are kept: `group_by`, `agg`,
  `filter(**constraints)`, `with_columns`, `select`, `pivot`, `join`, `vstack`, `transpose`, `explode`, `explain`.
- **pandas named aggregation** (`df.groupby(...).agg(worst=("stop", "max"))`): `agg(worst_mm=tip_stop_error.max())`
  names a column; without a name the expression names itself ("worst tip stop (mm)").
- **dplyr `summarise`** (`group_by(station, rung) |> summarise(worst = max(stop))`): the same shape in R, and the
  reason "summarise" is the word the page uses.
- **Polars `LazyFrame.explain()`**: `Runs.explain(...)` prints which computations run over each recording and which
  metrics read each one (5 computations serve the arm's 11 metrics).

## The pieces

| file | what |
|---|---|
| [measures.py](measures.py) | the expressions: measures (`RowError`, `StopValue`, `ChannelCode`, `TimeOffset`, `RunFlag`) and how each is summarised (`.median()`, `.max()`, `.rms()`, `.mean()`, `.percentile(q)`, `.count()`, `.share()`, `.all()`, `.any()`, `.per_run()`, `.repeatability(k)`, `PerformanceGain`) |
| [runs_table.py](runs_table.py) | `Runs` (one row per recording), `Table`, `Column`, `Threshold`, `Live`, `chunks`: the Polars-shaped stand-in |
| [integrated_servo.py](integrated_servo.py), [pendulum_arm.py](pendulum_arm.py), [rigid_chain.py](rigid_chain.py) | each part type: `models` (robot_models) and beside it `metrics`, a list of expressions |
| [rack_gates.py](rack_gates.py) | the rack's Thresholds with owners (Zach, agent, proposed) and `judge(table)` |
| [rung_ladder.py](rung_ladder.py) | the one session helper: the highest rung each station reaches |
| [scenario.py](scenario.py) | `run()` for the parity check; `GLANCE` and `CELLS` recorded in two fresh namespaces |

A measure reads one recording and returns values: one per stop, one per moving row, or one per run, each with its
unit. A summary pools those values over the runs it is handed: `agg()` hands it a group's runs, `Live` hands it the
chunks of one recording. Adding a metric is one constructor call (`RowError("velocity error", "deg/s", ...)`) and
one line in a part's list; adding a part type is one module with `models` and `metrics`. No registry.

## Offline and live, honestly

- **One code path.** Each shared computation is a reader with `add(chunk)` and `finish()`; each summary a running
  total with `add(values)` and `value()`. Offline adds the whole recording once; live adds 7 rows at a time. On CAN 11
  play 32 every live-capable metric of the arm equals offline (`equals(..., within=1e-9)` is True; the parity check's
  Z2 row passes to 1e-9).
- **Memory between chunks** (`Live.memory()` prints it):
  - the stop finder holds the rows since the last stop: one move and one dwell, never the run;
  - an RMS or a share holds 2 numbers, a worst value, a count or a flag holds 1;
  - **a median (or any percentile) keeps every value it has seen**, because an exact median needs them all. For a
    median tip stop that is one float per stop (2 on play 32, 84 for all of CAN 11): it grows with the stops, not
    with the rows. A bounded sketch (P², t-digest) would not reproduce the scorecard's exact medians.
- **Stops are safe across chunk boundaries.** A dwell still open at a boundary stays open inside the stop finder until
  a later chunk closes it. `result()` counts an open stop as if it ended now, on a copy, so the feed goes on.
- **Time alignment is stated, not done here.** The fixture puts the target at each reply's own instant (the
  scorecard's `align: measured`), and `Live` assumes target and measured arrive on the same rows. A raw-topic aligner
  (round 1's v3 `Aligner`) would sit in front of `Live`.
- **Lag is offline only.** `estimate_time_offset` is a cross-correlation over the whole recording; `Live` refuses it
  by name instead of buffering the run silently.

**Pooled vs per run is stated per metric**, in the column name and in the card: stops are pooled over a cell's runs
("median tip stop (mm)": the median of every stop in the cell); tracking is per run, then the median across runs
("tracking RMS, median over runs (deg)", written `tracking_error.rms().per_run().median()`). The glance's
`tracking_error.rms()` is the pooled RMS over every moving row, the plain Polars meaning, so its numbers differ from
the scorecard's per-run median (CAN 11 rung 0.9: 0.140 deg pooled, 0.135 deg as the median run).

## Gates

A gate is a `Threshold(value, unit, owner=...)` compared with a finished table column:
`table["worst tip stop (mm)"] <= limit` is a True/False column, and `with_columns` puts it beside the numbers it
judges. The median and the worst stop sit side by side in the same table. Round 8: the median stop passes 36 of 36
cells, the worst stop 33 of 36 (CAN 11 rungs 0.4, 0.9, 1.0 fail Zach's 1 mm); tracking passes 31 of 36; highest rung
CAN 11 1.0, the rest 0.9, all as round 1.

## One formula in one place (fr6)

No per-recording number here has its own formula: the error is `robot_msgs.compare`'s, overshoot is
`robot_msgs.compare.goal`, lag is `robot_msgs.compare.estimate_time_offset`, and the stop rule is round 1's shared
`stock.StopTracker`. What this layer adds is pooling (median, max, RMS of values already measured). What happens to
each copy:
- **robot_msgs.compare** keeps the math and gains the stop kernel (`robot_msgs.compare.stops`, beside `goal`);
- **sim2real.metrics** becomes measure declarations over it (its actuator stops are a `StopValue`, its tracking a
  `RowError`), its formulas deleted;
- **robotdx.diagnostics.stock** keeps its records and Limits, the Limits become `Threshold`s on tables, and its
  tracking / stop configs become measures.

## The real thing would be a thin layer over Polars

Neither Polars nor pandas is in robot_models' venv, and the light-core gate (G5) allows numpy, robot_msgs and the
stdlib, so `runs_table.py` is a ~470-line stand-in with Polars' names. Over Polars:

| element | here | Polars | must survive |
|---|---|---|---|
| one row per recording | `Runs(runs)` | a `DataFrame`, list columns for each recording's values | labels as columns |
| a measure | `values_of(measure, run)`, cached | a list column computed once per recording | one pass per recording, shared |
| a summary | `tip_stop_error.median()` | `pl.col("tip_stop_error").explode().median()` | pooled vs per run stated in the name |
| per run, then across | `.rms().per_run().median()` | `group_by(..., "id").agg(...)` then `group_by(...).agg(pl.col(...).median())` | the scorecard's median of play RMS |
| live | `Live(...).update(chunk)` | no live mode that keeps an exact running result between appended chunks (its streaming engine runs one query over a source in batches) | exact equality with offline |
| a gate | `column <= Threshold(...)` | `pl.col(...) <= 1.0` | the owner beside the threshold |

## Self-score against the contract (1 / 3 / 5)

| id | weight | score | evidence |
|---|---|---|---|
| lg | 30 | 5 | 25 code lines in the contract's 8 steps; every name imported, the data's origin in the first comment; verbs are group_by / agg / filter / pivot / with_columns / update; every output a table with units, a number, True, or a sentence ("33 of 36 cells"). Borderline names are listed in scores.json |
| fr1 | 14 | 5 | each part type is a module: `models` (robot_models) and beside it `metrics`, a list of expressions, each a configured stock measure with name, description, references and unit (`pendulum_arm.card(...)`) |
| fr2 | 14 | 5 | one call per recording (`Runs([run]).agg(*metrics)`), one `group_by` per level (id, station x rung, station, the deployment's `agg`); pooled vs per run in every metric's name and card |
| fr3 | 10.5 | 3 | live = offline exactly, from one code path; stops safe across chunk boundaries; alignment stated. But a median keeps every stop, so memory grows with the stops (shown by `Live.memory()`), and lag is offline only |
| fr4 | 10.5 | 5 | Thresholds with owners (Zach / agent / proposed) outside every metric; the median and the worst stop side by side in one table, each with its own pass column |
| fr5 | 7 | 5 | a metric is one constructor call and one list line (the session adds velocity error); a part type is one module, no registry |
| fr6 | 7 | 5 | every per-recording number is robot_msgs.compare's or the shared stop kernel's; the README says what happens to each of the three copies |
| fr7 | 7 | 5 | one `_viz.View` of every CAN 11 stop to matplotlib, a W&B payload and a Rerun `.rrd`; the measure carries its data-ui hint (`tip_stop_error.hint`) |

Total: 30 + 14 + 14 + 6.3 + 10.5 + 7 + 7 + 7 = **95.8**.

## What I'd change

- **Build it on Polars** once a dependency is allowed outside the core: a `robot_metrics.polars` adapter whose
  measures produce list columns, so the summaries are real Polars expressions and any Polars user reads the page.
- **A bounded median as an opt-in**, named as an approximation (`tip_stop_error.median(approximate=True)`), for long
  live runs; the exact median stays the default, because the scorecard and the gates read exact medians.
- **Name the per-run step once per part.** `tracking_error.rms().per_run().median()` is exact but long; a part could
  declare `tracking_rms = tracking_error.rms().per_run()` and use `tracking_rms.median()` everywhere.
- **Put the aligner in front of `Live`** (round 1's v3 `Aligner`) so raw command and state topics can arrive apart.
- **Pass the run's frame of reference explicitly.** The tip measures read `Run.tip_length_m`, a station fact the
  recording carries; a part tree (round 1's v5) would pass it as context instead.
