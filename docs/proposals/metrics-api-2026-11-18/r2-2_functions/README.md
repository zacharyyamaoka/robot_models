# r2-2 · Plain functions, table out

**Every metric is a plain function: call it on a run and it returns a number in the unit its name ends with.** Many
runs go into a small table with `scorecard(runs, by=["station", "rung"], metrics=[...])`. A gate is a number
compared with a column of that table. Live, a `LiveScore` is fed chunks of rows and equals the functions bit for bit.

- **Parity (G1):** `check_parity.py r2-2_functions` passes. Worst deviation 2.2e-16 deg on tracking, 0 on stops,
  1.9e-16 relative on the five-bar ([check_parity.out.txt](check_parity.out.txt)).
- **Runs as shown (G2):** check_standalone passes 2 of 2 blocks (the glance and the session) in a fresh `python -I`
  ([check_standalone.out.txt](check_standalone.out.txt)). Round 2's gallery builder didn't exist yet, so the check read
  a stand-in page built from this folder's `session.json` with round 1's `flow_html`, the same markup the builder draws.

## The direction and its prior art

| here | prior art |
|---|---|
| `worst_tip_stop_mm(run)` returns a float | `sklearn.metrics.mean_squared_error(y_true, y_pred)` returns a float; the name says what it measures |
| `worst_tip_stop_mm(runs)` on a list of runs, with the rule for combining runs stated per metric | sklearn's `multioutput=` / `average=` arguments: how a metric combines is the metric's own decision |
| `scorecard(runs, by=["station", "rung"], metrics=[...])` returns a table | `df.groupby(["station", "rung"]).agg(...)` |
| `table["worst_tip_stop_mm"]`, `table["passes"] = [...]`, `table[["a", "b"]]` | pandas column get, set and select |
| `table.pivot(index="station", columns="rung", values=...)` | `DataFrame.pivot` / `pivot_table` |
| `metric_table(PENDULUM_ARM_METRICS)` | `df.describe()`: one call, one readable summary |

**The unit goes in the name, not in a returned `(value, unit)` pair.** A call then reads as one fact:
`worst_tip_stop_mm(run)` gives `0.330`. The value stays a plain float you can compare, sort and add, and a table column
gets its unit in its header for free. A pair would put the unit beside every number, and every comparison would have
to unpack it first. A count (`stop_count`) or a bool (`all_stops_settled`) has no unit, so its name has none.

pandas is not used. The frozen parity check forbids importing it, so `Table` is about 70 lines of stdlib: a list of
row dicts.

## The files

| file | what |
|---|---|
| [run_metrics.py](run_metrics.py) | the metric functions, `scorecard` and `Table`, `LiveScore` and `chunks`, and the unit-free math they share |
| [parts.py](parts.py) | each part type's models beside its list of metric functions: `PENDULUM_ARM_METRICS = [tracking_rms_deg, ...]` |
| [gates.py](gates.py) | the rack's gates as `(column, at most, owner)` rows, plus `passes(table, gate)` and `highest_rung` |
| [scenario.py](scenario.py) | `run()` for the parity check, plus `GLANCE` and `CELLS`, each recorded in a fresh namespace |
| [scenario.out.txt](scenario.out.txt), [session.json](session.json) | what ran and what it printed (`session.json` is the gallery's input) |

Built on round 1's `stock.py` (the stop rule), `robot_msgs.compare` (`goal`, `estimate_time_offset`, `ErrorStats`)
and round 1's `fixture.py`, `calibration.py` and `_viz.py`, all imported and none copied. robot_metrics 0.1.0 is not
used, so a reader's path needs nothing beyond the three folders.

## How offline and live work, honestly

- **One record per run.** Every pendulum metric reads `RunTotals`: six running sums plus three numbers per finished
  stop (signed error, settle time, overshoot). Offline, `totals_of(run)` fills it with the whole run as one chunk and
  remembers it, so the nine live-capable metrics read run 32's rows once (`passes_over_rows(run)` gives `1`). Live,
  `LiveScore.add(chunk)` fills the same record chunk by chunk, and `result()` calls the same functions on it.
- **Live equals offline bit for bit, not just within 1e-9.** The sum of squares is added one row at a time, in arrival
  order, so chunking can't regroup the additions. The session shows it for chunks of 7 rows and of 1 row:
  `(True, True)` on `==`.
- **Stops survive chunk boundaries.** The stop rule is round 1's per-sample `StopTracker`. A hold that straddles a
  chunk boundary is still open when the next chunk arrives, and `result()` mid-run closes a copy of it, so the record
  keeps going.
- **Memory is not constant.** It holds the six sums, three numbers per stop, and **the rows of the move in progress**:
  the stop rule needs the move into a stop for its overshoot and the hold's rows for its settle time. On run 32 that
  is 175 of its 318 rows at the end (`live.kept_in_memory()`). So memory stays bounded by the longest move plus its
  hold, not by the run. A run that never holds still would keep every row.
- **Time alignment is required, not done here.** `add()` checks that the target and measured rows share their
  instants, and raises if they don't. The fixture already samples the plan at each reply's own instant. Raw topics
  that arrive apart would need an aligner first (robot_metrics 0.1.0's `Aligner`, or robot_msgs.compare's `align`).
- **Whole-run metrics say so.** `lag_ms` (estimate_time_offset searches the whole recording), `faulted_runs` (the
  outcome is known at the end) and the five-bar's ErrorStats rows are `live=False`. `LiveScore([lag_ms])` raises
  "lag_ms needs the whole run" instead of quietly buffering the run.

## Many runs and the gates

- A metric decides how several runs combine (the "many runs" column of `metric_table`). Tracking RMS takes the median
  of the runs. The peak takes the largest. The stop metrics pool every stop of the runs, so a rung's median tip stop
  is the median of all its runs' stops (14 on each CAN 11 rung), not a median of medians.
- `scorecard(runs, by=["station", "rung"])` gives 36 rows, `by=["station"]` gives 6, `by=["deployment"]` gives 1.
  Each is the same functions called on bigger groups.
- Gates are rows in gates.py: `("worst_tip_stop_mm", 1.0, "proposed: Zach's 1 mm on every stop")`.
  `passes(table, gate)` compares columns with their limits and returns a list of bools. Round 8 gives:
  - tracks: 31 of 36 rung cells;
  - tip stop, median: 36 of 36;
  - tip stop, every stop: 33 of 36 (CAN 11's rungs 0.4, 0.9 and 1.0).

  The highest rung tracked is 1.0 on CAN 11 and 0.9 everywhere else. No metric function holds a threshold. The settle
  band (1 mm at the tip) is part of what "settled" means, so it lives in `RunTotals` and is stated on the function.

## One formula in one place

| formula | its one home | what happens to the copies |
|---|---|---|
| the stop rule (holds, error, settle) | round 1's `stock.py`, proposed as `robot_msgs.compare.stops` | sim2real `metrics.actuator.stops` and robotdx `stop_error` call it |
| overshoot | `robot_msgs.compare.goal` | unchanged |
| lag | `robot_msgs.compare.estimate_time_offset` | unchanged |
| the five-bar's joint and tool RMS, max | `robot_msgs.compare` ErrorStats | sim2real's and robotdx's copies call it |
| the pendulum's tracking RMS | **summed here**, row by row in `RunTotals` | still a second RMS beside ErrorStats (see below) |

## Self-score against the contract's criteria

| id | weight | score | evidence |
|---|---|---|---|
| lg | 30 | 5 | 25 code lines in the contract's eight steps. Every name is imported, and the data's origin is in the first comment. The verbs are a function call, `add`, `result()`, `scorecard(by=...)` and a list comprehension. Each output is a number or a small table. Soft spot: a single number's unit is in the function name, not printed beside it |
| fr1 | 14 | 5 | `PENDULUM_ARM_METRICS` sits beside `PENDULUM_ARM_MODELS` in parts.py. Each function carries its name, description (docstring), references, unit, base and config (`metric_card`). parts.py stands in for robot_models' part modules |
| fr2 | 14 | 5 | One call per recording: `worst_tip_stop_mm(run)`. One call per level: `scorecard(..., by=["station", "rung"] / ["station"] / ["deployment"])`. Pooled vs per-run is stated per metric in the "many runs" column |
| fr3 | 10.5 | 3 | Bit-exact live = offline at 7-row and 1-row chunks, stops safe across boundaries, alignment checked and stated. But memory holds the move in progress (175 of 318 rows on run 32), so it is bounded by the move, not constant |
| fr4 | 10.5 | 5 | `gate_mm = 1.0  # owner: Zach` on the glance, with median and worst side by side per rung. Every gate is a `(column, at most, owner)` row in gates.py, owned by Zach, the agent or a proposal |
| fr5 | 7 | 5 | A new metric is one plain function defined in the session and passed straight to `scorecard`. Adding it to a part is one name in its list. Making a new metric live also needs a running sum in `RunTotals` |
| fr6 | 7 | 3 | The stop rule, goal, lag and the chain's ErrorStats each have one home. The pendulum's tracking RMS is re-summed row by row here (for exact live = offline) instead of read from `robot_msgs.compare.ErrorStats` |
| fr7 | 7 | 5 | Every CAN 11 tip stop goes into one `View`, drawn to matplotlib (`media/tip-stops.png`), built as a W&B payload and written to a Rerun `.rrd`. The data-ui hint comes from the metric (`worst_tip_stop_mm.hint == "distribution"`) |

**Total 93.0** (Σ weight × score / 5).

lg, what a newcomer might not know from the glance alone: nothing I can name. "RMS" is spelled out in its docstring.
`LiveScore`, `chunks` and `scorecard` are each introduced by the plain comment above them. CAN 11, rung, stop and tip
are domain words, and a stop is defined in `stop_count`'s docstring. The dispatcher re-scores this.

## What I'd change

- **A stop tracker that keeps O(1) per stop:** a running overshoot, the hold's last 50 ms and its settle state,
  instead of the move's rows. That would make live memory truly bounded (fr3) and belongs in
  `robot_msgs.compare.stops`.
- **Read the tracking RMS from robot_msgs.** Give `robot_msgs.compare.ErrorStats` a running form that adds row by row
  and the pendulum's RMS stops being a second copy (fr6).
- **Show the unit beside a lone number.** A float whose repr reads `0.330 mm` was considered and rejected: it is a
  float that doesn't print like one, which surprises a reader more than a unit in the name does.
- **A weak cache.** `totals_of` remembers each run by identity for the life of the process. Fine for 504 runs, but it
  should let go of runs nobody holds.
- **Display names for columns,** for example `"worst tip stop (mm)"` for `worst_tip_stop_mm`. I kept the function's
  own name so a column always says which function made it.
