# r2-4 · Checklist with targets

A part's metrics are a short checklist. Every row reads as a sentence: its name, how it is measured, the unit it is
shown in, its target and who set the target. `card.check(run)` prints pass / fail per row with the measured value;
`card.check(runs, by=["station", "rung"])` says, per row, how many groups pass and names the ones that fail. A row
with no target is measured and shown as "observed".

```python
Check("worst tip stop", measure=worst_stop_error(at="tip"), unit="mm", target="<= 1.0", owner="Zach 2026-11-11")
```

**Zach's rule, kept literally:** "separate the calculation of the metric, and the checking of thresholds".
`measure=` computes one number in SI with no threshold anywhere inside it; the row carries the unit, the target and
the owner; the card does the comparing. A wrong unit fails when the row is built (`mm` on an angle raises).

## Files

| file | what |
|---|---|
| [checklist.py](checklist.py) | the face: `Check`, `Card`, `card.check` / `card.measure` / `card.live`, the measures (`rms_tracking_error`, `worst_stop_error`, `stop_count`, ...), `in_chunks` |
| [parts.py](parts.py) | each part type's checklist beside its models: `INTEGRATED_SERVO`, `PENDULUM_ARM`, `RIGID_CHAIN` (`.models`, `.metrics`) |
| [rung_ladder.py](rung_ladder.py) | `highest_passing_rung(report, checks)`: the highest rung with every rung below it passing |
| [scenario.py](scenario.py) | `run()` for the parity check, `GLANCE` and `CELLS` recorded in fresh namespaces |
| `session.json`, `scenario.out.txt` | the recorded session (round 1's `_session.py` format) |
| [scores.json](scores.json) | the self-score the gallery reads |
| [checks.out.txt](checks.out.txt) | the two gates as they ran on the final recording |

The arithmetic is robot_metrics 0.1.0's, imported read-only from `/home/bam/robot_metrics/src` (checklist.py appends
it to `sys.path` and asserts the version). Each row becomes a robot_metrics `Alias` over a stock metric, so rows that
share a computation share it (the three stop rows of the glance read one pass of the stop kernel). The data is round
1's `fixture.py`; the views are round 1's `_viz.py`. Nothing in robot_metrics, `src/`, round 1 or the contract changed.

## Prior art

- **Great Expectations**, `expect_column_values_to_be_between(column, min_value, max_value, mostly=...)`: an
  expectation suite is a list of expectations; its metrics compute an `observed_value`, the expectation only judges it,
  and validation returns one result per expectation. Here: a card is the suite, a `Check` the expectation, `measure=` the
  metric, and `report.results` lists one record per (group, row) the same way. GE's `mostly` is why the median and the
  worst stop are two rows: "worst tip stop <= 1.0" is `mostly=1.0` on every stop; "median tip stop <= 1.0" is
  `mostly=0.5`.
- **pytest-benchmark**: one table row per benchmark with its statistics side by side, and
  `--benchmark-compare-fail=mean:5%` thresholds that live outside the measurement. Here: `card.measure(..., by=...)`
  prints one row per group with each check's number in its unit.
- **OpenSLO** objectives (`displayName`, `op`, `value`): a target is an operator and a number beside the indicator,
  never inside it. Here: `target="<= 1.0"`, and the owner rides beside it (OpenSLO carries ownership as metadata).

## How live and offline work (honestly)

- **Offline** is one update with the whole recording; **live** is `live.update(chunk)` per chunk, through the same
  robot_metrics states. `card.check(live)` checks the chunks so far.
- **Equal:** on CAN 11's play 32 (the glance's recording), 7-row chunks give bit-identical numbers to offline
  (`card.measure(live) == card.measure(run)` is `True`). One-row chunks give the same stops and the same stop errors;
  the sums differ by 2.8e-17 (float round-off in a sum taken in another order). The parity check's G4 holds at 1e-9.
- **Stops survive chunk boundaries:** robot_metrics' stop kernel carries the open dwell across chunks as a value, so a
  stop split by a chunk edge is found whole. The session shows it with one-row chunks, where every boundary falls inside
  a move or a stop.
- **Memory is bounded for the sums, not for everything.** `live.kept()` prints what is held: tracking RMS and peak keep
  4 numbers, the limiter share 2; the stop rows keep every stop found so far (each with its move's rows, so overshoot
  can be read at the end) plus the rows of the move still open (175 rows at the end of play 32). `lag` keeps every row
  of the run, because a cross-correlation needs the whole run; it is the one end-of-run row and says so.
- **Time alignment** is the fixture's: the target is the plan at each reply's own instant. robot_metrics refuses a
  pair that is not on one clock (`check_aligned`), so an unaligned live feed fails loudly rather than scoring wrong.
- **Many runs:** a pooled row (stops, limiter share) pools every stop or row of the group; tracking RMS is the median
  of the runs' RMS, the peak the worst run; each measure states its rule (`Check.card()["across runs"]`). Tip numbers
  pool within one station only (six stations, six levers): grouping runs of two tip lengths raises.

## Results

- **Parity (G1): PASS** on round 8 at the frozen tolerances; worst deviation 2.8e-17 mm (`checks.out.txt`).
- **Runs as shown (G2): 2 of 2 PASS** (glance: 17 statements; session: 66), in a fresh `python -I` with only the
  variant folder, a round folder and round 1's folder on `sys.path`. Run on a one-variant `gallery-spec.json` drawn from
  `session.json` with round 1's `flow_html`, because round 2's `build_gallery.py` does not exist yet.
- The glance's answer: the median tip stop passes 36 of 36 groups; the worst stop passes 33 of 36 and fails CAN 11 at
  rungs 0.4 (1.293 mm), 0.9 (1.143 mm) and 1.0 (1.317 mm). Tracking RMS alone fails five stations' 1.0 rung.

## Self-score against the contract (1 / 3 / 5)

| id | weight | score | evidence |
|---|---|---|---|
| lg | 30 | 5 | 26 lines; imports first with the data's origin; verbs `check`, `measure`, `live`, `update`; each measure's plain meaning printed by `print(card)`; every output a table or `True`. Owners read "Zach 2026-11-11" and "AI agent": no term left to define |
| fr1 | 14 | 5 | `parts.py`: `INTEGRATED_SERVO.models` (robot_models' family) beside `INTEGRATED_SERVO.metrics` (its checklist); each row a configured stock measure with name, description, references and unit (`Check.card()`) |
| fr2 | 14 | 5 | `card.check(run)` per recording; `by=["station", "rung"]`, `by=["station"]` for the roll-ups; pooled vs per-run stated per measure. The deployment is "every station passes", one `all(...)` line, because tip numbers never pool across levers |
| fr3 | 10.5 | 3 | live = offline (bit-identical at 7-row chunks), stops safe across boundaries, alignment stated; memory grows with the stops kept and `lag` keeps the run |
| fr4 | 10.5 | 5 | target and owner on the row, nothing in the measure; median and worst stop are two visible rows; owner per row (Zach / AI agent / proposed) |
| fr5 | 7 | 5 | a new metric is one `Check(...)` in the list (`Card(name, arm.checks + [velocity])`); a new part type is one `Card` |
| fr6 | 7 | 3 | no fourth copy: the arithmetic is robot_metrics 0.1.0's; robot_msgs.compare keeps the error math, robot_metrics the stop rule, and sim2real.metrics and robotdx.diagnostics.stock would call it. Stated, not done here |
| fr7 | 7 | 5 | `card.view(row, runs, by=...)` -> round 1's View -> matplotlib PNG, a W&B payload, a Rerun `.rrd`; the data-ui hint comes from the stock metric (`distribution`) |

**Total 93.0** (Σ weight × score / 5).

The glance's owner for the worst tip stop is "Zach 2026-11-11", as the direction's example writes it. `parts.py` says
"proposed, Zach's 1 mm" for that row, because applying his 1 mm to the worst stop rather than the median is round 1's
proposal, not his ruling.

## What I'd change

- **Bound the stop memory.** Keep each stop's numbers (error, settle, overshoot) instead of its move's rows; that needs
  overshoot read when the stop closes, a robot_metrics change.
- **A bounded `lag`:** a running cross-correlation over a fixed window instead of the whole run.
- **Show "across runs" in `print(card)`.** The rule is stated per measure but only visible through `Check.card()`.
- **Named gates on the card.** The scorecard's "tracks" verdict is four rows passing together; today the session holds
  that as a list of names (`tracks = [...]`). A card could carry `gates={"tracks": [...]}`.
- **One call for the deployment.** `by=["station"]` plus an `all(...)` line is honest about the levers but is still a
  hand-written step.
- **Targets as strings** (`"<= 1.0"`) are parsed when the row is built, so a typo fails at once; `at_most(1.0)` would
  let an editor check it too.
- The view and the `.rrd` are written to the gitignored `reports/media/robot-models-metrics-api-proposals-2026-11-18/`,
  round 1's place for gallery media, outside this folder.
