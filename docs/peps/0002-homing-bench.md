# PEP 0002: The homing bench's pure parts — the five metrics and the two seams

- **Status:** accepted, 2026-11-12
- **Follows:** PEP 0001 (the stop core). Decision report `/home/bam/bam_ws/reports/homing-subassembly-proposals-2026-11-12.html`, steps 3–4 of "What the default does next".
- **Layering:** pure parts here (`robot_msgs <- robot_models`). The kinematic plant, the procedures, the bench loop and the W&B leaderboard need a controller and a rig, so they live in sim2real (`sim2real.sim.homing`, branch `claude/homing-bench`), on top.

## Decision

1. **Seams (`robot_models.homing.protocols`).** `HomingPlant`: `reset(start)`, `tick(command, dt, defines)`, `truth()`. `HomingProcedure`: `plan(prior, so_far) -> JointCommandSequence | None` (collect) and `observe(prior, phases) -> HomingResult` (fit). `Prior` is what a procedure may know before it moves (stop map, stored backlash table, seek side, caps). `Phase` is one sequence as it ran. Typing only.
2. **Metrics (`robot_models.homing.metrics`).** Computed from outputs and truth only, never a procedure's code:

| Metric | Definition |
|---|---|
| `time_to_home_s` | tick clock, first command to done; median and p95 (nearest rank) over finished episodes |
| `reliability_pct` | done with no abort/timeout/refusal AND every stall it used truly on the CAD stop it meant (within 1.0°, `stall_proof`'s tolerance); every episode in the denominator |
| `any_start_pct` | reliable AND every joint's zero inside the rung gate |
| `backlash_error_deg` | per drive \|identified gap − hidden gap\|, worst drive per episode, p95 over reliable episodes; unidentified = `None` (shown '—'), never 0; `observable_pct` beside it; gate per drive max(1′, 25 % of truth) |
| `joint_angle_error_deg` | per joint \|identified zero − true zero\|, worst joint per episode, p95 over reliable episodes; an unzeroed joint is `missing`, never perfect |

No weighted sum: sort by reliability, then zero error, then time.

3. **Ruler first.** `tests/test_metrics_ruler.py` was committed against a deliberately wrong `metrics.py` (unfinished episodes dropped from the denominator, stalls not checked against their stop, mean for p95, unidentified gaps read as 0) and recorded red before the real one went in.
