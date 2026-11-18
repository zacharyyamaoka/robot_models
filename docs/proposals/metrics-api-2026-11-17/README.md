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

## The five directions (assigned before building)

| | name | where it lives (dependency direction) | streaming state | diagnostics and gates | across-run summary | prior art |
|---|---|---|---|---|---|---|
| v1 | robot_metrics | a new `robot_metrics` package on robot_msgs; robotparts' part type holds `.metrics` | sufficient statistics (sum, count, max) and a stop tracker carried across chunks | a `Diagnostic(metric, Limits)` wrapper | `merge_state` pools; a `PerRun(metric, "median")` wrapper | torchmetrics (`add_state`, `merge_state`, wrappers), Keras metrics |
| v2 | Family metrics | `robot_models.metrics`; `IntegratedServo.metrics` declared in the family body beside its models | a cat buffer: `update` appends, `compute` runs the functional twin | `Gate(target Value, owner)` on the deployment record | one object fed the rung's runs; the constructor's `across_runs=` | torchmetrics cat states, RobotDx's `TrackingErrorConfig.across_runs` |
| v3 | Signals and aliases | the math in `robot_msgs.metrics` (signals only); meaning bound by aliases in each part's catalog | raw topics: command and state arrive apart; an aligner pairs them live | an alias carries its Limits | rliable-style: per-run scores, median and IQM with bootstrap CIs | rliable, Hugging Face `evaluate` cards, evo |
| v4 | RobotDx split | io-diagnostics: `robotdx.metrics` (stock/* ported) and `robotdx.diagnostics` (Limits, Result unchanged) | a bounded window: live is the last `window_s`, offline one window spanning the run | RobotDx's `Result`: judge, view hint, record v0.2, gate matrix | RobotDx's `center(per_run, across_runs)`, rollup over subjects | RobotDx, ROS diagnostic_updater, torchmetrics |
| v5 | Part tree | robotparts: `RobotPart.metrics`, and an assembly's metrics compose its children's (deployment → arm → servo) | segment events: each move and dwell becomes an event with its own statistics | per-node status, a parent is its worst child; gates on nodes | per node, grouped by dataset labels, rolled up the tree | ROS 2 diagnostic_aggregator, Drake / like_keras ports |

Considered, not built: **adopting torchmetrics itself.** Measured here: `import torchmetrics` takes 3.9 s and 450 MB
of RAM (numpy + `robot_msgs.compare`: 0.15 s, 41 MB), and its venv carries 2.7 GB (torch 804 MB, nvidia 1.9 GB);
tensors would sit between robot_msgs arrays and every metric. Its shape is adopted by all five instead.
