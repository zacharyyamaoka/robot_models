# v2 · Family metrics: `IntegratedServo.metrics`, beside its models

**Thesis.** The metrics are declared on the part type's ModelFamily, beside its models: `IntegratedServo.metrics` is a
`Metrics` collection in the family body. A metric's state is a cat buffer and its math one functional twin over
`(config, runs)` (RobotDx's own signature), so the class can never disagree with its function, and one object fed a
rung's runs computes the rung by the `across_runs=` rule in its constructor.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/v2_family_metrics/scenario.py
```

`check_scenario.py v2_family_metrics` → **PASS** (scorecard parity within 2.8e-17 mm and float round-off in degrees;
the chain to 1.9e-16 relative). Transcript: [scenario.out.txt](scenario.out.txt).

## Where it lives and what it reads

- **Layering:** `robot_msgs <- robot_models (models AND metrics, per type) <- robotparts` (a RobotPart reads both from
  its family) `<- sim2real / RobotDx`. A type with no ModelFamily yet (`rigid_body_chain`) is keyed by its type id.
  In the prototype the metrics are attached to the real `IntegratedServo` at import; in production they are a class
  attribute in `integrated_servo_family.py`.
- **Run data:** `Episode` (the MDP's word): id, named robot_msgs streams, labels, an `.rrd` pointer.
- **Input types:** `ErrorStat`, `Dwell` a tracked pair; `Repeatability` a `Repeat`; `PerformanceGain` a `HeldOutPair`.

## How it works

- **Streaming state:** `update()` appends the chunk; `compute()` concatenates and runs the functional. Exact by
  construction, a stop across chunks is simply in the buffer. The price: memory O(rows) and 18 ms per `compute()` on
  one pendulum play (an append costs 0.01 ms).
- **Computed once:** a memo keyed by (identity, which data: run, rows, chunk count) serves every consumer. The servo's
  6 aliases, then the arm's 12 on the same recording: 5 new computations, 7 memo hits, 1 stop pass. `Dwell` is one
  class with a `quantity` config (error, settle, overshoot, settled, count) instead of five subclasses.
- **Gates:** a `Deployment` record holds named verdicts of `Gate(alias, target)`, the target a robot_msgs `Value` with
  its range and `source` (who chose it), the noun sim2real's `gate()` already uses.
- **Across runs:** the constructor's `across_runs` (median, max, pool); Dwell quantities pool.
- **Views:** `view(collection, alias)` builds the stop or run points in the alias's display unit for the shared adapter.

## What happens to the existing code

- **robotdx.diagnostics.stock:** its functions move into `robot_models.metrics` with their `(config, runs)` signature
  unchanged, so the 89-entry catalog ports by renaming; RobotDx keeps Limits / Result / records.
- **sim2real.metrics.actuator:** a re-export of robot_models.metrics until its callers move.
- **CalibrationOutcome** lives in `robot_models.metrics` (beside the fit that produces it).

## Cost of adding

- **A metric:** one `Alias` (or a `Dwell(quantity=...)` config); new math is a subclass with one `functional`.
- **A part type:** a `metrics = Metrics(...)` line in its family body.

## Best when / loses when

**Best when** offline datasets are the main use and a type's models and metrics should be read in one place.
**Loses when** it runs live for long (the buffer grows and compute re-reads it), and it puts metrics into the models
package, against Zach's "all there really is is models" naming ruling and his later "a robot_metrics package".
