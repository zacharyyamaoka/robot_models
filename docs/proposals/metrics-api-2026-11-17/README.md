# Five metrics APIs for the performance metrics per part type (2026-11-17)

Zach, 2026-11-17, on the vault note `Performance Metrics per Part Type — Proposed (2026-11-17)`: "This is a great
start for the outside-view metrics. Of course we can continue to change at some point. Can you please make 5
proposals for APIs on how these can be implemented?"

This folder is a babble round: five isolated, runnable prototypes, one shared dataset of real runs, one frozen check.
Nothing in `src/`, robot_msgs, sim2real, io-diagnostics, robotparts or bam_ws changes until Zach picks.

## The contract (frozen before any variant was built)

**Outcome all five serve:** a part type's performance metrics (real or sim vs the target) computed from robot_msgs
recordings, live or offline, rolled up from runs to a deployment, with the gates applied outside the metric.

**Settled, not re-litigated** (Zach, 3000-ft view and Sim2Real line 70-75): performance first, fidelity (real vs sim)
is a separate family and out of scope; runs are the atomic unit; metric → diagnostic (+ threshold) → gate (+ target);
ungated metrics are observed; metrics are model-free; one W&B project per deployment.

**Invariants:** robot_msgs in; SI inside, degrees shown; the generic rows call `robot_msgs.compare` (ErrorStats,
`goal`, `estimate_time_offset`) and the two rows it lacks come from one shared kernel ([stock.py](stock.py): which
rows move, the stop rule as a per-sample tracker); every core imports numpy + robot_msgs + stdlib only.

**Shared dataset** ([fixture.py](fixture.py), read-only on /archive and the twin's sim memo):
- `pendulum_runs("real")`: the 1-DOF rack's round 8, 252 plays (6 stations × 6 rungs × 5 repeats + 2 held out),
  paired runs split per board, each a `Run` of robot_msgs streams (target at each reply's instant, the response, the
  plan on its ticks) with its labels (station, rung, kind).
- `pendulum_runs("sim")`: the twin's answer to each of those 252 plays (round 8's twins, from the sim memo).
- `chain_runs("real" | "sim")`: the bam_fb five-bar's real speed ladder of 2026-11-10 (5 rungs, two joints and the
  L2 tip as a PoseTrajectory) and its sim twin.

**Scenario** (every variant's `scenario.py`, verbatim, on real data):
1. a part type's metrics: `RobotPart.metrics` beside `.models` for the integrated servo, the 1-DOF pendulum arm and
   a rigid chain;
2. one recording, offline and live: the same object fed the whole recording and fed it in chunks gives the same
   numbers (tracking RMS and peak in degrees, tip stop median and worst in mm, settle, limiter share, plus two rows
   pendulum_score lacks: overshoot via `robot_msgs.compare.goal` and lag via `estimate_time_offset`);
3. the dataset: runs → rung → station → deployment, the gates applied (Zach's: tip ≤ 1 mm at a settled stop; the
   rest labelled agent / proposed), a summary across the 252 runs, the highest rung each station tracks;
4. the same rows on the twin's output (sim vs target) for all six stations;
5. the rigid chain through the same API (joint RMS per joint, tool RMS and worst), a standard view per metric
   (`plot()` to matplotlib, the same result as a W&B payload and Rerun calls), and a new metric added in the session.

**Stop line:** the pendulum and the five-bar only; other part types are declared, not run. No hardware, no W&B
upload (payloads are built, not sent), Rerun is written to a local `.rrd` only when the SDK is present.

**Parity oracle (hard gate G1):** [check_scenario.py](check_scenario.py) compares each variant's numbers with the
round-8 scorecard (`scorecards/latest_real.json`: per play tracking RMS, peak, stop count; per rung tracking, peak,
stop median and worst, settle, limiter share; the sim side; the highest rung; the gate counts) and with the five-bar
ladder's own `summary.json`, at tolerances fixed before building: 1e-6 deg, 1e-6 mm, 1e-12 on shares, 1e-9
relative on the chain. Overshoot and lag are checked against independent oracles in the check itself. Only the check
reads the oracle files. Before any variant existed, a plain reference scorer passed with zero deviation everywhere,
and two wrong rulers failed: the plan linearly interpolated at each reply (play RMS off by up to 9.4e-4 deg) and the
rung stop as a mean of per-play medians (off by 3.7e-4 mm).

### Weighted criteria (sum 100)

| id | criterion | weight | 1 | 3 | 5 |
|---|---|---|---|---|---|
| fr1 | **A part's metrics read like its models** | 20 | the rows are functions scattered over repos | a list exists, away from the part type | `Part.metrics` beside `.models`, each a configured stock metric with name, description, references and unit |
| fr2 | **Recording in, summary out** | 20 | per-run and roll-up are hand glue | per run is one call; roll-ups are hand-written | one call per recording, one for runs → rung → station → deployment, pooled vs per-run stated per metric |
| fr3 | **Live = offline, honestly streaming** | 15 | "live" re-runs the whole buffer, unstated | exact, but the state grows with the run | bounded state, stops safe across chunk boundaries, time alignment handled or stated |
| fr4 | **Gates stay outside compute** | 15 | thresholds inside the metric | metric + threshold, but owner or worst-vs-median hidden | metric + Limits / target = diagnostic / gate, owner (Zach / agent / proposed) on each, median and worst both visible |
| fr5 | **Adding a metric or a part type is small and local** | 10 | several files and a registry | one class plus a registration | one subclass or one constructor call in the part's list |
| fr6 | **One formula in one place** | 10 | a fourth copy beside robot_msgs.compare, sim2real.metrics and robotdx.diagnostics.stock | says where the copies go, keeps two | the three copies become one, and the variant says what happens to each |
| fr7 | **Standard views** | 10 | numbers only | a plot | `plot()` to matplotlib, the same result to W&B and Rerun, the data-ui view hint carried |

The weights are the AI's reading: fr1 and fr2 are Zach's two asks this evening ("RobotPart.metrics -> metrics",
"pass in the Run Recording ... and the metrics come out"); fr3 is his "able to compute live and offline"; fr4 his
old rule "separate the calculation of the metric, and the checking of thresholds"; fr6 the coordinator's "the same
formulas live in three places today". Revise them and the ranking moves.

### Hard gates (pass/fail, never averaged)

- **G1 Parity:** `check_scenario.py` passes (above).
- **G2 torchmetrics-shaped:** a `Metric` base class; stock metrics are subclasses configured in the constructor;
  `update` / `compute` / `reset`; the same object fed a whole recording gives the offline number; a collection type;
  a functional twin for each stock metric.
- **G3 Zach's `Metric` fields:** name, description, references; a value that is a number or a bool; a target, when
  there is one, outside `compute()`.
- **G4 Live = offline:** a recording streamed in chunks equals the offline value (the check asserts it to 1e-9).
- **G5 Light, black-box core:** numpy + robot_msgs + stdlib at runtime (matplotlib, W&B and Rerun only as lazy
  adapters); no metric reads a model; real, sim and replay differ only by a label; every number is target vs one
  source (no sim-vs-real row).
- **G6 Math-only base metrics, meaning from the data** (Zach, "# Ok homing in on this..."): a base metric carries only
  its mathematical meaning; one RMS-error metric serves servo tracking, tip tracking and the chain's tool error by
  the streams it is given. Most metrics are a base class + a frozen config; a subclass only where config cannot say it.
- **G7 Aliases, identity and units:** a semantic name ("tip stop error", "T4 stop error p90") is an alias: a configured
  instance with a display name, a display unit and the signal it reads. A metric's identity is its base class + its
  frozen config; the alias is only a label. Units and degrees belong to the alias, never to the base math.
- **G8 Computed once:** two aliases with the same (base, config) on the same data compute once; the scenario shows it
  with a counter or a cache hit.

## Amendments (each before any variant was built; the criteria weights did not change after amendment 2)

1. **2026-11-17 20:03 MST, Zach's `Metric` design** (Daily Note Nov 17, "# pErformance metrics per type... where are
   they."): "everything is just signals", functional, live and offline, robot_msgs in and out, a `Metric` domain type
   with a name, description, references, a function and maybe a target, bool or number valued; run recording in,
   metrics out; summary statistics across runs (a for loop for now). → G3, G4 and fr2, fr3.
2. **2026-11-17 20:11 MST, the torchmetrics shape** ("# Ok torch metrics" and "# Ok homing in on this..."): "a base
   Metric class and the particular metrics are sub classes", constructor config, standard visualization (matplotlib,
   W&B, Rerun), a `robot_metrics` package, `RobotPart.models -> ModelFamily, RobotPart.metrics -> metrics`, "very
   similar to robot diagnostics ... separate the metric from the data", "the semantic meaning ... from the actual data
   you pass in ... we can always create alias for the metrics". → G2, fr1, fr6, fr7; the five directions re-allocated
   so each is torchmetrics-shaped and they differ on the remaining axes. Torch itself is not a runtime dependency.
3. **2026-11-17 20:15 MST, configurable math-only metrics and aliases** ("# Ok homing in on this...": "most metrics
   has the mathematical meaning but the semantic meaning ... from the actual data you end up passing into them";
   "Configurable metrics ... base metric type plus a configuration and then for those you cannot you can just extend
   it"; "100% agreed keep the target outside of compute"). → G6, G7, G8. His closing question, "What does run data
   look like? ... History, DataStream, Run, Episode, Result ... do we also carry in the rerun thing?", is answered
   per variant (each names its recording type and how it points at the run's `.rrd`).
4. **2026-11-17 21:05 MST, other inputs than a recording** (the Sim2Real Routines session, via the coordinator; v1-v3
   were built, v4-v5 not): every base class declares its input type; `Repeatability(k=3)` takes one real repeat at a
   time over round 8's same-input repeats (ISO 9283 RP at each stop); `PerformanceGain(metric=<alias>)` reuses an
   alias by identity on held-out runs played with nominal and with fitted values ([calibration.py](calibration.py):
   the earlier round's sealed friction ladder, robot_models' own fit) and returns the difference. The
   `CalibrationOutcome` record lives in robot_metrics or robot_msgs, never sim2real. Checked against independent
   oracles in the check. Same commit: a ruler fix that only narrows a false positive (G7 compared units as
   substrings and read the "s" of `StopStatistic` as seconds; it now compares whole CamelCase words).
5. **2026-11-17 21:10 MST, the prior-art note** (`Metric Library APIs — Prior Art (2026-11-17)`, §1, §4, §5). It
   arrived after v1-v3 were built and the criteria frozen, so it does not reweight them. Instead each variant gets a
   **prior-art check** in the gallery (P1 the torchmetrics shape without torch; P2 state as a mergeable value; P3
   Limits outside the config; P4 OpenSLO occurrence gates with a tail statistic per stratum, Wilson intervals for
   bools; P5 alignment as an explicit step with a tolerance; P6 context beside the data and provenance carried; P7
   a per-metric live flag, a version on stored values, units converted only at display), and v4 / v5 take the
   positions v1-v3 left open: v4 is the note's recommended core (state as a value, Limits out of RobotDx's configs,
   occurrence gates), v5 the part tree that passes context beside the data. "Live == offline" means within 1e-9
   (G4); v1-v3 happen to agree bit for bit on round 8, a summed state need not.

## The five directions (assigned before building; v4 re-allocated by amendment 5 before it was built)

| | name | where it lives (dependency direction) | streaming state | diagnostics and gates | across-run summary | prior art |
|---|---|---|---|---|---|---|
| v1 | robot_metrics | a new `robot_metrics` package on robot_msgs; robotparts' part type holds `.metrics` | sufficient statistics (sum, count, max) and a stop tracker carried across chunks | a `Diagnostic(metric, Limits)` wrapper | `merge_state` pools; a `PerRun(metric, "median")` wrapper | torchmetrics (`add_state`, `merge_state`, wrappers), Keras metrics |
| v2 | Family metrics | `robot_models.metrics`; `IntegratedServo.metrics` declared in the family body beside its models | a cat buffer: `update` appends, `compute` runs the functional twin | `Gate(target Value, owner)` on the deployment record | one object fed the rung's runs; the constructor's `across_runs=` | torchmetrics cat states, RobotDx's `TrackingErrorConfig.across_runs` |
| v3 | Signals and aliases | the math in `robot_msgs.metrics` (signals only); meaning bound by aliases in each part's catalog | raw topics: command and state arrive apart; an aligner pairs them live | an alias carries its Limits | rliable-style: per-run scores, median and IQM with bootstrap CIs | rliable, Hugging Face `evaluate` cards, evo |
| v4 | Folded values | io-diagnostics' stock math lifted into a `robot_metrics` on robot_msgs; RobotDx keeps `Limits` / `Result` / records, the Limits moved out of its configs (amendment 5) | state is an immutable value: `empty` / `update(state, chunk)` / `merge` / `compute`; live is a fold over chunks | OpenSLO occurrence gates (`op`, `value`, `share`) with a tail statistic per stratum; RobotDx `Result` per verdict | merge the states of a stratum; Wilson intervals for bool metrics | clu / torcheval, OpenSLO, RobotDx |
| v5 | Part tree | robotparts: `RobotPart.metrics`, and an assembly's metrics compose its children's (deployment → arm → servo); context (the part) passed beside the data | segment events: each move and dwell becomes an event with its own statistics | per-node status, a parent is its worst child; gates on nodes | per node, grouped by dataset labels, rolled up the tree | ROS 2 diagnostic_aggregator, nuPlan `compute(history, scenario)`, Drake / like_keras ports |

Considered, not built: **adopting torchmetrics itself.** Measured here: `import torchmetrics` takes 3.9 s and 450 MB
of RAM (numpy + `robot_msgs.compare`: 0.15 s, 41 MB), and its venv carries 2.7 GB (torch 804 MB, nvidia 1.9 GB);
tensors would sit between robot_msgs arrays and every metric. Its shape is adopted by all five instead.

## Result (2026-11-17, after all five were built and checked)

`check_scenario.py` → **5 of 5 PASS** ([check.out.txt](check.out.txt)). Every variant reproduces round 8's scorecard
at float round-off (worst: 2.2e-16 deg on tracking, 2.8e-17 mm on stops, the sim side the same), the five-bar
ladder's robot_msgs numbers to 2.8e-15 relative, the gate counts (tracking 31, tip stop median 36, every stop 33 of
36 rung cells) and each station's highest rung (CAN 11 1.0, the rest 0.9). Overshoot and lag match independent
oracles; `Repeatability(k=3)` on CAN 11's five real 0.9-rung repeats is 0.387 mm at the tip in all five; the sealed
ladder's `PerformanceGain` is 0.2220 → 0.0042 N·m (gain 0.2178 N·m).

| | variant | where the metrics live | streaming state | one live update (12 aliases) | AI score |
|---|---|---|---|---|---|
| v4 | [Folded values](v4_folded_values/README.md) | robot_metrics beside RobotDx, on robot_msgs | an immutable value, folded | 0.97 ms / 7 rows | **84** |
| v1 | [robot_metrics](v1_robot_metrics/README.md) | its own robot_metrics package, on robot_msgs | add_state attributes | 1.4 ms / 7 rows | 80 |
| v3 | [Signals and aliases](v3_signals/README.md) | robot_msgs.metrics + the part's catalog | raw topics, live aligner | 0.66 ms / 1 row | 78 |
| v2 | [Family metrics](v2_family_metrics/README.md) | robot_models, on the ModelFamily | cat buffer + functional | append 0.01 ms; compute 18 ms | 76 |
| v5 | [Part tree](v5_part_tree/README.md) | robotparts, the part tree | move / dwell events | 0.71 ms / 7 rows | 74 |

**AI pick: v4, 84.** Not a co-leader call (v1 is 4 points behind), but fragile: weight "reads like torchmetrics" or
the part's adjacency (fr1) up and v1 ties it. **Splice recommended:** v4's value state, occurrence objectives and
RobotDx Result, under v1's torchmetrics face (`Metric.plot()`, compute groups, `update` / `compute` / `reset` via
`Stream`), with v3's `Aligner` as the explicit alignment step for raw topics and v5's rule that an assembly reads its
child's computation (the arm shows its servo's stops through the lever). Gallery:
`/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-17.html`
([build_gallery.py](build_gallery.py), [capture_gallery.py](capture_gallery.py)).

Findings the round surfaced on the way:
- **The scorecard's ruler is reproducible from robot_msgs streams alone** once the target is the plan at each reply's
  instant; the plan linearly interpolated (robot_msgs.compare's own resampling) is off by up to 9.4e-4 deg.
- **Lag reads negative on CAN 11** (-0.16 to -1.35 ms per rung): each reply is stamped when its command was sent,
  after its tick, so the response "leads" the plan tick. It is the ruler's convention showing, not a fast joint.
- **Merging states needs equal configs:** the six stations have six tip lengths, so a deployment-level tip number
  cannot be one merged state; the deployment reads gates per stratum (v4) or worst children (v5).
