# Metrics APIs, round 2: legible at a glance (2026-11-18)

Zach, 2026-11-18, on the public robot_metrics page (round 1's v4 + v1 splice, released as robot_metrics 0.1.0):
"I am not completely in love with this API right now I am not seeing what is going on at a glance. I don't know what
you mean by fold etc." Then: "Please make sure to redo the gallery again." Earlier the same day, on round 1's gallery:
"I cannot read this pseudo code because its not valid ... where did you import it from etc.."

Round 1 ([../metrics-api-2026-11-17/](../metrics-api-2026-11-17/README.md)) stays as the record. Its displayed code
fails the new standalone check: **0 of 10 blocks run as shown** (every glance and session stops at its first
statement with a NameError, because the harness ran the cells in scenario.py's hidden globals;
[check_standalone.out.txt](../metrics-api-2026-11-17/check_standalone.out.txt), commit 80c1b04).

This folder is a babble round: a baseline and four new directions, one shared dataset of real runs, three checks
frozen before any variant is built. **Stop line:** nothing in `src/`, robot_metrics, robot_msgs or robotparts changes
until Zach picks.

## The contract (frozen before any variant is built)

**Outcome, settled points, invariants, dataset and scenario: round 1's, unchanged.** A part type's performance metrics
(real or sim vs the target) from robot_msgs recordings, live or offline, rolled up from runs to a deployment, with the
gates applied outside the metric. Round 1's README holds the wording; nothing in it is re-litigated here.

**Imported from round 1, never copied.** Every variant puts round 1's folder on `sys.path` and imports from it:
- `fixture.py`: `pendulum_runs("real")` (round 8's 252 real plays), `pendulum_runs("sim")` (the twin's 252 answers),
  `chain_runs(...)` (the five-bar ladder), `stations()` (each station's tip length);
- `stock.py`: which rows move, and the stop rule as one per-sample tracker (live and offline share it);
- `calibration.py`: the sealed friction ladder's held-out runs (amendment 4);
- `_session.py` (the notebook harness) and `_viz.py` (matplotlib, the W&B payload, Rerun calls).

A variant may build on robot_metrics 0.1.0's math by importing it (never editing it), or on `stock.py` and
`robot_msgs.compare` directly. Either way the glance shows only the variant's own face.

### The glance: one story, the same in every variant

Each variant's "API at a glance" tells this story in this order, each step under a one-line `#` comment in plain
words, in about 25 lines of code (comments and blank lines not counted):

1. **load the real runs**: `from fixture import pendulum_runs` with a comment saying what it loads;
2. **the pendulum arm's metrics**: what is measured, each with its unit;
3. **one recording, offline**: `pendulum_runs("real")[32]` (CAN 11, rung 0.9, a repeat), so every glance shows the
   same numbers;
4. **the same recording, live**: fed in chunks of 7 rows as they would arrive;
5. **equal**: live and offline give the same numbers, shown;
6. **many runs into a table by station and rung**: a readable table (a 6 × 6 pivot, or CAN 11's six rows);
7. **Zach's 1 mm tip-stop gate**: the threshold, its owner, and that it sits outside the metric;
8. **its verdict**: per rung cell, with the median and the worst stop both visible (round 1's answer: the median
   passes 36 of 36 cells, the worst stop fails 3 CAN 11 rungs).

Rules for every displayed block, glance and session alike (Zach's two /rate bad notes of 2026-11-18,
[.claude/feedback.md](../../../.claude/feedback.md)):
- **It runs exactly as shown.** Every name is imported or defined on the page before it is used, and the data's
  origin is shown. No pseudo-code in a glance.
- **Readable output under each statement:** a number with its unit, or a small table. Never a framework repr such as
  `Collection(15 aliases over 5 states)`.
- **Plain words.** Verbs a Python user already knows. A term a Python user would not know gets a plain one-line
  meaning on the page at its first use; better is not needing one.
- Round 1's readability rules still hold: Pygments highlighting, one statement then its output, blank lines between
  ideas, lines of about 90 characters or less, domain names (never `o`, `b`, `tt`).
- The session exhibit opens with a visible **setup** cell (imports, then loading the real data). Later cells build
  only on names the page defined above them.

### Hard gates (pass/fail, never averaged)

- **G1 Parity on round 8**: [check_parity.py](check_parity.py) passes. It runs round 1's frozen `check_scenario.py`
  (md5 7d9ced14..., unedited) on this round's folders: the same scorecard oracle and tolerances, and the same
  structure rows (Z1 metric cards, Z2 live = offline, G6 one math function serving several named metrics, G7 units on
  the named metric and never in the math, G8 a shared computation done once, A4 other inputs). The keys of
  `run()`'s dict are the check's vocabulary, not the page's: a variant need not say "alias" or "base" anywhere.
- **G2 Runs as shown** (new): [check_standalone.py](../metrics-api-2026-11-17/check_standalone.py) is green on this
  round. It reruns every displayed block in a fresh `python -I` and compares each statement's output with the page.
- **G3 Zach's `Metric` fields**: name, description, references; a value that is a number or a bool; a target, when
  there is one, outside the computation.
- **G4 Live = offline**: a recording fed in chunks equals the offline value, to 1e-9.
- **G5 Light core**: numpy, robot_msgs and stdlib at runtime; matplotlib, W&B and Rerun only as lazy adapters; no
  metric reads a model; every number is the target vs one source.

Round 1's G2 (torchmetrics-shaped) is dropped. Three of this round's directions are deliberately not torchmetrics, and
r2-1 tests that shape on its merits.

### Weighted criteria (sum 100)

"Legible at a glance" leads at 30. Round 1's seven criteria are rescaled to 70 with their ratios kept (each × 0.7).

| id | criterion | weight | 1 | 3 | 5 |
|---|---|---|---|---|---|
| lg | **Legible at a glance** | 30 | coined jargon (fold, state, merge, close, strata, alias, collection, occurrence) or hidden setup | runs as shown and every name is imported, but two or three coined terms carry the meaning, or an output is a repr instead of a number or table | someone who has never seen the API reads only the glance and can say what each line does and what came out, without a single term defined. Every name is imported, the data's origin is shown, the verbs are ones a Python user knows (update / compute / reset, a plain function call, group_by / agg), and each output is a number with its unit or a small table |
| fr1 | A part's metrics read like its models | 14 | the rows are functions scattered over repos | a list exists, away from the part type | the part's metrics sit beside its models, each a configured stock metric with name, description, references and unit |
| fr2 | Recording in, summary out | 14 | per-run and roll-up are hand glue | per run is one call; roll-ups are hand-written | one call per recording, one for runs → rung → station → deployment, pooled vs per-run stated per metric |
| fr3 | Live = offline, honestly streaming | 10.5 | "live" re-runs the whole buffer, unstated | exact, but memory grows with the run | bounded memory, stops safe across chunk boundaries, time alignment handled or stated |
| fr4 | Gates stay outside the computation | 10.5 | thresholds inside the metric | metric + threshold, but owner or worst-vs-median hidden | threshold, owner (Zach / agent / proposed) and target outside the metric; median and worst both visible |
| fr5 | Adding a metric or a part type is small and local | 7 | several files and a registry | one class plus a registration | one function, one constructor call or one row |
| fr6 | One formula in one place | 7 | a fourth copy beside robot_msgs.compare, sim2real.metrics and robotdx.diagnostics.stock | says where the copies go, keeps two | the three copies become one, and the variant says what happens to each |
| fr7 | Standard views | 7 | numbers only | a plot | matplotlib, the same result to W&B and Rerun, the data-ui view hint carried |

**Evidence for lg** comes from the glance alone: list every name or output on it that a Python user would not know
and the page does not explain at first use. Domain words don't count (run, recording, station, rung, tip, stop,
settle, target, measured, mm, deg). Score from that list against the anchors, and quote the list in the gallery.

### The directions (assigned before any building)

| id | name | the face | live | many runs | the gate | prior art |
|---|---|---|---|---|---|---|
| baseline | robot_metrics 0.1.0 | exactly as published (`/home/bam/robot_metrics` @ tag v0.1.0, 2573912), its own glance made valid. The API does not change; this column is for comparison | as published | as published | as published | round 1's v4 + v1 splice |
| r2-1 | Keras / torchmetrics face | mutable metric objects: `metric.update(target, measured)`, `metric.compute()`, `metric.reset()` | `update` per chunk; offline is one `update` on the whole recording | one metric per rung cell, or `reset` between runs | a threshold object apart from the metric | Keras metrics, torchmetrics |
| r2-2 | Plain functions, table out | `tip_stop_error_mm(run)` returns a number; `scorecard(runs, by=["station", "rung"])` prints a table | a small bounded `LiveScore.add(chunk)` | `scorecard(..., by=[...])` | a column of the table against its target | sklearn.metrics, pandas |
| r2-3 | Group-by expressions | `runs.group_by("station", "rung").agg(tip_stop_error.median(), tip_stop_error.max(), tracking_error.rms())` returns a table | the same expressions over a growing frame of chunks, with memory bounded or stated | `group_by(...).agg(...)` | a filter or a pass/fail column on the table | Polars, pandas named aggregation |
| r2-4 | Checklist with targets | rows of name, how it is measured, unit, target and owner; `card.check(runs)` prints pass / fail per row | `card.check` on the chunks so far | the same rows, per station and rung | the target column; the measurement stays apart from the target | Great Expectations, pytest-benchmark |

### How a variant is laid out

- One folder per direction: `baseline/`, `r2-1_keras/`, `r2-2_functions/`, `r2-3_group_by/`, `r2-4_checklist/`. The
  gallery's variant id is the folder's prefix (`baseline`, `r2-1`, ...), which is how check_standalone finds it.
- **baseline** reads the published package through a symlink, `baseline/robot_metrics ->
  /home/bam/robot_metrics/src/robot_metrics`. Its `scenario.py` asserts `robot_metrics.__version__ == "0.1.0"` and
  that the checkout is exactly tag v0.1.0.
- `scenario.py` in each folder has:
  - `run() -> dict` for G1;
  - `GLANCE` and `CELLS`, recorded with round 1's `_session.py`. The glance runs in `{"__name__": "__main__"}`, and
    the session cells run in a second fresh dict (set `session.namespace` between them). Never `dict(globals())`:
    that hidden namespace is what failed round 1.
- Helpers the session uses live in a short, honestly named module in the variant folder, imported on the page.
- One `build_gallery.py` in this folder writes `gallery-spec.json` here and
  `/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-18.html`. Round 1's builder is imported
  for its highlighting and layout where it fits.
- Outputs are deterministic (no timings echoed), so the check can compare them.

### The checks

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/check_parity.py
```

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/check_standalone.py /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18 --path /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17 --python "uv run --offline --project /home/bam/robot_models --with matplotlib --with rerun-sdk==0.40.0 python"
```

The standalone gate runs in the same environment the scenarios record in (the view cells need matplotlib and
rerun-sdk; `uv run` keeps them out of the venv). A reader's `sys.path` holds the variant folder, this folder, and
round 1's folder. Nothing else.

Both checks were proven before any variant was built:
- `check_parity.py` on round 1's v4 folder: PASS. With no round-2 folders it exits 1.
- `check_standalone.py` on round 1: 0 of 10. On a control round, a self-contained block passed, and a wrong output
  and an undrawn `print` both failed.

## Result (2026-11-18, after all five were built and checked)

**Both hard checks pass on the built page:**
- `check_parity.py`: **5 of 5 PASS** on round 8. The worst deviation is 4.4e-16 deg on tracking and 2.8e-17 mm on stops. The gate counts and each station's highest rung match.
- `check_standalone.py`: **10 of 10** blocks run as shown in a fresh `python -I`. Round 1 managed 0 of 10.

| | variant | the face | legible (self → dispatcher) | total |
|---|---|---|---|---|
| r2-2 | [Plain functions, table out](r2-2_functions/README.md) | `worst_tip_stop_mm(run)` → a number; `scorecard(runs, by=[...])` → a table | 5 → 5 | **93.0** |
| r2-4 | [Checklist with targets](r2-4_checklist/README.md) | `Check(name, measure, unit, target, owner)`; `card.check(runs, by=[...])` → pass/fail per row | 5 → 5 | **93.0** |
| r2-3 | [Group-by expressions](r2-3_group_by/README.md) | `runs.group_by("station", "rung").agg(tip_stop_error.max(), ...)` | 5 → 4 | 89.8 |
| r2-1 | [Keras / torchmetrics face](r2-1_keras/README.md) | `metrics.update(target, measured)`, `compute()`, `reset()` | 5 → 5 | 87.4 |
| baseline | [robot_metrics 0.1.0](baseline/README.md) | `arm.values(arm.fold(run))`, `summarize`, `Objective` | 1 → 1 | 60.6 |

The fr1–fr7 scores are the builders' self-scores. The dispatcher re-scored only legibility, from each glance alone, in [dispatcher_lg.json](dispatcher_lg.json). r2-3 drops to 4 because a reader who doesn't use pandas needs `agg`, `pivot`, `with_columns` and `Runs([run])` explained.

Every variant scores 3 on fr3: an exact median keeps every stop, so memory grows with the stops. Each README says so.

**AI pick: r2-4's checklist, with r2-2's plain functions as its measures.** r2-2 and r2-4 tie at 93. Zach's question breaks the tie: "I am not seeing what is going on at a glance". r2-4's outputs answer "did it pass, against whose target?" in words, and its card reads as a spec. r2-2's code is the plainest of the five. The splice takes the best of both:
- each `Check`'s `measure` is one of r2-2's functions (`worst_tip_stop_mm(run)` → mm), which also works on its own;
- the card is the only place targets and owners live (Zach's rule: separate the metric from its threshold);
- r2-3's pivot is the overview table.

Fix before adopting:
- the tracking-RMS target (≤ 0.5 deg) is the agent's, so it's labelled `proposed`;
- `card.check(live)` stays, but the README states what memory a live check keeps.

Gallery: `/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-18.html`. It opens on Split, baseline against the pick; "All five" is a strip that scrolls sideways. Nothing in robot_metrics changes until Zach picks.
