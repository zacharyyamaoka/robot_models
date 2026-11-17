# v3 Composed terms: a Model is a sum of terms; a rung is a named preset

**Thesis.** A Model is a composition of small term models, one per slot (inertia, friction, ripple, drivetrain,
electrics, wear, correction), each of which runs on its own. `+` combines terms, and the model's parameters are the
union of theirs. A type declares its slots and the options for each slot. A ladder key such as `M3.stribeck` is the
name of one composition (`inertia:ideal + friction:stribeck`). Climbing the ladder means swapping or filling one slot,
and a swap pays only when the rung below it misses the rig's noise floor on plateaus neither fit saw (amendment 1).

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v3_terms/scenario.py
```

The amended check passes: `check_scenario.py v3_terms` prints `PASS  v3_terms`, case A → M3.stribeck (held-out
rms ÷ floor: M2 20.28, M3 1.00) and case B → M2.coulomb_viscous (M2 1.03, M3 1.57). The full session, case B included
(step 6), is in [scenario.out.txt](scenario.out.txt). Swapping this variant's rule for a training-rms rule, an
always-the-richer rule or an always-the-lower rule makes the check FAIL (probed in-process, not committed).

## The object model

```
Slots (servo)        the type: slot order, each slot's options, and the 14 presets (today's keys, in order)
├─ Term              one physics law that fills one slot; it runs its rung's own model.py
│    needs / reads / vocabulary / aliases / equation / references / falls_back_to (within its slot)
├─ Model             a sum of Terms, at most one per slot; .key is set when the sum matches a preset
│    + Term / + Model, .redeclare(friction="stribeck"), .fallback(), .changes(other)
│    .parameters (union of the terms' needs), .optional, .canonical(alias), .describe()
│    .torque(values, v=...), .from_datasheet(sheet), .fit(fit_log, start=...)
└─ Bound             a Model with values: .predict(v), .params, .rms (on its fit log), .score(log) (on any other)
compare(*bounds, on=test_log)   → Comparison: lowest first, held-out rms ÷ noise_floor(test_log), .chosen
```

Files:

- [composed_terms.py](composed_terms.py) is the generic core. It has 347 non-blank lines, over the 250-line target;
  the fit and the REPL printouts take up the extra. Amendment 1 added 27 (score, noise_floor, the held-out
  Comparison) and removed BIC.
- [servo_slots.py](servo_slots.py) is the type's one new table: which slot each rung fills (27 lines).
- [scenario.py](scenario.py) runs the shared scenario as a REPL session.

## What it borrows

- **Modelica `replaceable` / `redeclare`.** A slot is a replaceable component with a constrained set of options.
  `m2.redeclare(friction="stribeck")` swaps one option and keeps the rest. Options *replace* each other within a slot.
- **lmfit composite models.** `Model + Model` gives a single model whose parameter set is the union. `fit` belongs to
  that composite, and its result (here `Bound`, lmfit's `ModelResult`) predicts, reports its fit rms and scores
  itself on another log.
- **Drake Diagram of LeafSystems.** Each term is a leaf that runs alone; it calls `actuators/rungs/<word>/model.py`
  unchanged. The diagram (the Model) sums the leaves' torques. Terms in different slots *add*.

## Key decisions

1. **A slot holds a whole law, not a delta.** `friction:stribeck` covers Coulomb, viscous and the Stribeck drop.
   It is not `coulomb_viscous + stribeck_excess`, because LuGre and weakening-with-capture do not add onto Stribeck;
   they replace it. So terms add across slots and replace each other within a slot. M1's datasheet term is a
   *friction* option (its physics is the stated Coulomb + viscous). If it had a slot of its own, M2 would count
   friction twice.
2. **Presets and terms are derived from v1, then checked against it.** A term's needs are what the rungs in its own
   slot add down its v1 fallback. A preset takes the nearest option for each slot down that fallback. `Slots(...)`
   raises unless all 14 presets have v1's `parameters`, `optional` and `fallback` exactly. The type adds only
   `SLOT_OF`.
3. **The ladder's tree is derived from the slots.** `Model.down()` steps one term back within its slot (stribeck →
   coulomb_viscous → datasheet), or empties the slot, and it must land on exactly one preset. For all 14 presets this
   reproduces v1's `base` chain. An off-ladder sum can step down two ways; asking for its fallback says so instead of
   guessing.
4. **Fitting is on the composition, and the code works out how.** It nudges each parameter on the log to find which
   ones the log cannot move (the inertia, on velocity plateaus, is carried from `start`) and which ones the torque is
   linear in. The linear parameters are solved by least squares at each point of a log-spaced grid over the rest; the
   grid spans the log's own range when the unit matches an input, then zooms in on the best point (variable
   projection). It uses numpy only, with no per-rung fitting code. It fits on the log it is given: the scenario
   passes `rig_log(case, "fit")` only.
5. **A swap pays only on held-out data (amendment 1).** `compare(m2_fit, m3_fit, on=test_log)` orders the fits
   lowest first, scores each with `Bound.score(test_log)`, divides by `noise_floor(test_log)` (pooled within-plateau
   std, the rig's repeatability read from the log itself) and chooses the lowest at or under 1.5 × the floor; when
   none reaches it, `chosen` is None and the printout says to swap or fill another slot. The rule is sim2real's
   `select()` (`cards/schema.py`), mirrored, not imported (the runtime stays numpy + stdlib), so the rung chosen
   here is the one `select()` would choose from the same scores. It beats an information criterion here because
   the floor is observable on the rig and the ratio is the number sim2real already reports. BIC on the fit log is
   gone: it was the training-data choice the amendment rules out. Each row still names the slot it swapped, so the
   transcript reads "friction: coulomb_viscous → stribeck … 1.00 ← chosen".
6. **"Lowest" is fewest fitted parameters, then ladder order.** v3 can fit a sum no rung names (`m3 + ripple`),
   which has no place on the fallback chain; counting what the data had to set ranks it too. For M2 → M3 it is the
   fallback chain's own order, and ties fall back to the ladder's key order, as sim2real's `select()` walks it.
7. **The comparison refuses the fit log.** Each `Bound` keeps the samples it was fitted to, and `compare(..., on=)`
   raises if the held-out log shares any of them (step 5 shows the error). Judging on the fit plateaus would grade
   the optimiser, not the choice, and the API makes that the mistake you cannot make quietly.

## What the composed view brought out

- v1's `M4.drivetrain` adds `harmonics` again, a quantity the `ripple` term also owns. With composition it would be
  `ripple + drivetrain(backlash, torsional_stiffness)`. This prototype derives it exactly as v1 has it and does not
  change it.
- `M3.weakening_capture` has no code, so `M4.drivetrain` through `M6.residual` cannot run even where their own new
  terms might. The grid shows this column by column.
- `ripple/model.py` already reads `coulomb_friction` and `viscous_friction` with `.get(..., 0)`, so it works as a
  pure additive term with no edit. `stribeck + ripple` composes (`(m3 + servo.term("ripple")).parameters`), and it
  would run with the M4 cap lifted.

## Best when / loses when

- **Best when** effects are orthogonal and you want combinations no rung names: stribeck friction plus ripple, or
  dcmotor electrics on load friction. You get them with `+` or `redeclare` and no new rung. It also fits when the
  ladder is mostly "fill the next slot" (M4 to M6 are exactly that), and when a reviewer wants to see what changed
  between two rungs (`changes()` gives `friction: coulomb_viscous → stribeck`). The held-out comparison reads
  the same way: each row is one slot swap and its cost against the floor, so "this swap did not pay" (case B) is a
  row, not a separate report.
- **Loses when** effects inside one slot interact. "Stribeck *and* load-dependent friction" cannot be written as
  `stribeck + load`, because both are friction options, so it needs a new combined option (or nested slots, as
  Modelica nests replaceables). It also loses when one rung spans slots (v1's drivetrain repeats ripple's harmonics),
  and when the order of a rung's parameters matters beyond slot order. Composition gives only one way to step down,
  so a ladder whose fallback is not "undo the last slot change" fits poorly. The held-out rule needs a log with
  repeated plateaus to read its floor from; a log without `plateau` ids cannot be judged by this `compare` until the
  floor is stated some other way.

## Migration from v1

Nothing moves. This is an additive layer over today's ladder.

- **The v1 ruler** (`tests/test_ladders_acceptance.py`) is untouched: `Ladder`, `Rung` and `ladders()` stay as they
  are, and `Slots` reads them. Adding `Slots(LADDER, SLOT_OF)` to that suite pins the composed view to v1, since the
  constructor already raises on any difference.
- **robotparts vendoring** of `actuators/rungs/*` is untouched. Terms load each `model.py` by path and read
  `model.json` (inputs, term spellings such as `sigma2`), so no rung bytes or sha256 pins change.
- **sim2real `tests/test_robot_models_parity.py`** is untouched: keys, fallback chains and parameter lists are the
  same as v1's. The selection rule is sim2real's own (`select()`, limit 1.5), so a later parity test can pin
  `compare(...).chosen` against `select()` on the same held-out scores; nothing in sim2real has to change for it.
- **The fixture's new split** costs nothing in the core: `fit` takes whatever log it is handed, and only `compare`
  gained the `on=` argument and its need for `plateau` ids.
- **Later, if Zach picks it:** `SLOT_OF` becomes a `slot=` field on `Rung` (or a `slots.py` beside each type's
  `ladder.py`, found by the same walk as `ladders()`), and `Slots` is exported as `robot_models.slots`. That is one
  new field and one new module, with nothing renamed.

## The cost of a new M3 friction rung (say `M3.dahl`)

1. `actuators/rungs/dahl/model.py` and `model.json`, plus `model.js` for robotparts' parity. These are the runnable
   law and its inputs, exactly as today.
2. One `Rung(level=3, name="dahl", base="M2.coulomb_viscous", adds=(...), code="actuators/rungs/dahl",
   equation=..., references=...)` in `actuators/servo/ladder.py`, plus a `Parameter` for each new quantity.
3. One line in `SLOT_OF`: `"dahl": "friction"`.

The preset `M3.dahl`, its grid row, its fallback, `redeclare(friction="dahl")`, fitting (linear and nonlinear
parameters are found automatically) and `compare` all follow with no further code. Steps 2 and 3 would merge into one
declaration if `slot=` moves onto the rung.

## What is simulated or left out

- Only M0 to M3 run (the round's stop line). M4 to M7 are declared, and calling them raises `NotImplementedError` with
  the reason: the prototype's cap, or "no runnable code" for weakening_capture, drivetrain, envelope, wear, residual
  and learned.
- `predict` is the steady-state map (`dt=None`). LuGre's bristle state is not stepped through time.
- Vectorisation is a Python loop over the stdlib reference `model.py`. A numpy backend per term would be the next
  step.
- Fitting assumes at most two nonlinear parameters and that they are positive, because the grid is logarithmic.
  Optional parameters such as `stribeck_exponent` stay at the rung's default and are not fitted.
- The noise floor assumes each plateau is one held condition, so within-plateau spread is the rig's noise; drift
  inside a plateau would inflate it and make the rule too lenient. Only one log (velocity plateaus, split into fit
  and held-out halves) is used, so the inertia is carried from the datasheet and never identified.
- The leakage guard matches samples by exact float value. It catches the same log passed twice; it would not catch
  a re-measured copy of the fit plateaus.
- `composes` (the foc_controller delay loop) and `wears` are carried on the terms but not used.
