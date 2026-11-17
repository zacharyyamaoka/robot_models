# v2 — Records in a Family

**Thesis.** A `Model` is a frozen record (key, level, name, base, parameters with units, equation, references) that
also carries its physics; `model.bind(values)` gives a callable `BoundModel`. A `Family` (one per type) holds the
vocabulary, the aliases and the model tree, and so it is also what judges the tree: `family.compare(...)` scores fitted
models on held-out data and picks the lowest rung that reaches the rig's noise floor. It is the minimal-migration
option: `Ladder` becomes `Family` (literally a subclass of it), `Rung` becomes `Model`, and the rung's code folder is
attached as an object instead of named by string.

Run it: `/home/bam/robot_models/.venv/bin/python scenario.py` (transcript: [scenario.out.txt](scenario.out.txt));
check: `python check_scenario.py v2_records` → `PASS  v2_records` (case A: M2 20.28×, M3 1.00× the floor → M3.stribeck;
case B: M2 1.03×, M3 1.04× → M2.coulomb_viscous).

## The object model

```
families() -> {type: Family}                         today's ladders(), each read as a Family
Family(Ladder)                                        a v1 Ladder: vocabulary, aliases, packed, rungs, every v1 call
  ├─ family["M3.stribeck"] -> Model                   the record you hold
  ├─ .tree()  .children(key)  .siblings(key)          the tree; fallback/canonical/holds inherited from Ladder
  ├─ .values(datasheet | {name: number|Stated})       -> Values: canonical names -> SI floats (PyBaMM's ParameterValues)
  └─ .compare(*bound, observed=, repeats=, **inputs)  held-out data -> Comparison: rows lowest rung first, .floor,
                                                         .chosen (key), .best (the BoundModel to hand on)
Model (frozen)  key level name title base kind adds reads parameters optional equation references physics
  ├─ .bind(values) -> BoundModel                      checks what it needs, keeps what it holds
  ├─ .fit(observed, values=, grid=, **inputs)         -> BoundModel: varies model.json's required terms
  └─ .physics: Physics | None                         model.py's torque() + model.json's inputs/required/mujoco maps
BoundModel  model values(dict, SI) fitted rms samples   __call__(**inputs) numpy in/out; .mujoco()
noise_floor(observed, repeats)                        pooled within-repeat std: what the rig repeats to
```

The "what now" flow, from the transcript:

```python
values = servo.values(datasheet())                                      # datasheet -> SI values, once
m1 = servo["M1.datasheet"].bind(values)                                 # the datasheet model
m3 = stribeck.fit(fit_log["tau"], v=fit_log["v"], values=values)        # fit on the fit plateaus only
verdict = servo.compare(m1, m2, m3, observed=test_log["tau"], repeats=test_log["plateau"], v=test_log["v"])
verdict.chosen   # 'M3.stribeck' (servo B: 'M2.coulomb_viscous')
```

## Amendment 1: choosing on held-out data

The v0 build compared BIC on the log it had just fitted. Zach, 2026-11-17: "if doing a more complex model doesn't
actually improve test time performance you are just overfitting". Now:

- **Fit on the fit plateaus, judge on the held-out ones.** `Model.fit` is unchanged; the scenario hands it
  `rig_log(case, "fit")`. `Family.compare` takes the held-out log and computes each model's held-out rms itself.
- **The floor comes from the log, as data.** `repeats=log["plateau"]` labels the samples taken at one held condition;
  `noise_floor` is their pooled within-repeat standard deviation. Like the values (PyBaMM), the data stays apart from
  the model: compare is handed a log and its repeat labels, and nothing about the rig is stored on a record.
- **The rule is sim2real's** (`src/sim2real/cards/schema.py` `select()`, `TWINNED_LIMIT = 1.5`): the lowest rung whose
  held-out rms ÷ floor ≤ 1.5. `compare` sorts rows by depth in the tree (`len(family.fallback(key))`), same-depth
  siblings best first, so "lowest" is the tree's own order. That is why the comparison belongs to the Family and not
  to the Model or the BoundModel: only the Family knows the tree.
- **Why this rule and not another.** I checked the rules on the reference data before choosing:
  - *Training rms* fails servo B (M3 0.00398 < M2 0.004037 on the fit plateaus). A scratch copy of v2 with that rule
    gives `FAIL v2_trainrms … case B: chose 'M3.stribeck'`.
  - *Training BIC* (v0's rule) happens to pick M2 on servo B (−2745.1 vs −2741.1, Δ 4.0). But it is still graded on
    data the fit saw, and it only says which model is better, never whether either is good enough.
  - *Lowest held-out rms* also picks M2 on servo B, by 1.03 vs 1.04: a 1 % gap, so the next noise draw could flip it.
  - *The floor rule* picks M2 because M2 is already within 1.5× of what the rig repeats to, so there is nothing left
    for M3 to explain. It does not depend on that 1 % gap. It has an absolute stop, and it is the rule sim2real twins
    by, so a rung chosen here is the rung sim2real would select.
- **What the transcript shows** (steps 5 and 6): each candidate's fit rms beside its held-out rms and ÷ floor. On
  servo B, M3 trains better and tests worse. Its fit puts a Stribeck knee below the slowest fit plateau (breakaway
  −4.73 N·m at 0.0107 rad/s) to chase noise.

## Prior art borrowed

- **lmfit `Model`** ([docs](https://lmfit.github.io/lmfit-py/model.html)): wrap an existing function and take its
  parameters by name; `model.fit(data, x=...)` puts the data first and the independent variables as keywords; the fit
  returns a result that still evaluates. Here the wrapped function is today's `model.py` `torque()`, the parameter names
  come from its `model.json` (translated to the type's canonical names: LuGre's `sigma2` → `viscous_friction`), and
  `stribeck.fit(log["tau"], v=log["v"], values=values)` is lmfit's call shape.
- **PyBaMM** (model vs `ParameterValues`): a model is a symbolic thing with no numbers; values live in their own
  object and are bound later. Here `servo.values(datasheet)` is the `ParameterValues` (units converted once, by the
  datasheet's own `Stated.si`), and `model.bind(values)` is `process_model`. The same `values` feed M1, M2 and M3.
- **Variable projection** (Golub & Pereyra 1973) for the fit; **hold-out validation against a noise floor** for "the
  rung that pays", with sim2real's `select()` as the rule. BIC (Schwarz 1978) was v0's measure and is gone.

## Key decisions

1. **`Family` subclasses v1's `Ladder`.** Every v1 check and call (`fallback`, `canonical`, `parameters`, `holds`,
   `unpack`, `resolve`, `composed`) comes for free and unchanged; `ladders()` could hand out Families tomorrow with no
   consumer edit. The Family adds `family[key]`, `tree()`, `children()`, `siblings()`, `values()`, `compare()`.
2. **A Model's `base` stays a key; the Family owns the tree.** The record carries its resolved parameters (inherited
   ones included, each a `Parameter` with unit and definition), so `print(model)` describes it without the Family, but
   fallback and siblings are Family calls. That is the structural line against v1 (fallback = MRO): records are flat,
   the tree is data.
3. **Physics is attached, read from the unchanged rung folder.** `Physics.load` compiles `model.py` from source (an
   import would write `__pycache__/` into a folder robotparts pins byte for byte) and reads `model.json` for inputs,
   output, required terms, defaults and MuJoCo maps. Attached for M0–M3 only (the round's stop line); M4–M7 are records
   whose `bind(...)(...)` / `fit(...)` raise `NotImplementedError` saying so.
4. **`fit` lives on the Model and is generic.** It varies the terms `model.json` marks required, probes the physics to
   split them into linear terms (solved exactly by `lstsq`) and nonlinear ones (log-spaced grid over the logged speed
   range when the term has an input's unit, refined 4×; else `grid=`), and verifies the linear terms superpose at the
   answer. So a new rung's `model.py` brings no fitting code.
5. **`compare` lives on the Family and judges on held-out data (amendment 1).** Data first, by keyword, as `fit` takes
   it: `observed=`, `repeats=`, the inputs. It returns the rows lowest rung first, the floor, `.chosen` (the key) and
   `.best` (the chosen `BoundModel`, so the winner is handed on without a lookup: step 7 calls `verdict.best.mujoco()`).
   No rung within the limit gives `chosen = None` and says "climb, or look at the rig". `limit=` is a keyword
   (default 1.5) for a rig that wants a different one.

## Best when / loses when

- **Best when** the priority is landing without breaking anyone: three repos read `ladders()` today, and this is an
  additive module whose Family *is* a Ladder. Also when rungs keep arriving as robotparts-style folders
  (`model.py` + `model.json`): the record picks them up, and fitting and held-out selection come free. Also when the
  rung chosen in robot_models should be the one sim2real twins: both use the same rule.
- **Loses when** you want the Model to stand alone: it can describe and compute itself, but fallback, siblings and
  choosing a rung need its Family, and the parameters are names in a vocabulary, not typed attributes a type checker
  sees (`values` is a dict). It also loses when the fit needs physical bounds. The fitter is unbounded, so on servo B
  M3 lands on breakaway −4.73 N·m. The held-out choice discards that fit, but nothing flags it on the BoundModel.
  A rung whose physics is stateful (LuGre with `dt`) or not separable needs more than the probe: LuGre runs at steady
  state only here, and a jointly nonlinear term needs an explicit `grid=`.

## Migration from v1

- **Landing it:** add `src/robot_models/family.py` (this module less `RUNNABLE_UP_TO`). Nothing else changes. Later,
  optionally, `ladders()` returns `Family.from_ladder(...)` and type folders may write `LADDER = Family(...)`.
- **The v1 ruler (`tests/test_ladders_acceptance.py`):** untouched. Rung records, keys, bases, adds, `code` strings and
  every rung byte stay as they are; `isinstance(LADDER, Ladder)` still holds for a Family; the value scan judges type
  folders, not a root module. The fresh-interpreter import test picks up one more module (numpy is already a runtime
  dependency). The held-out rule adds no ruler test of its own in `src/`. The proposal's check is its ruler for now.
  A landing would port case B into `tests/` as the negative control.
- **robotparts' vendoring of `actuators/rungs/*`:** untouched. The folders are read in place and never imported, so no
  `__pycache__/` lands in them and the byte-identical and `RUNG_FILES` sha256 checks stay green.
- **sim2real `tests/test_robot_models_parity.py`:** untouched. It calls `ladders()`, `.keys`, `.rung()`, `.resolve()`,
  `.holds()`, `.composed()`: all inherited. `FLOOR_LIMIT = 1.5` duplicates sim2real's `TWINNED_LIMIT`. robot_models
  cannot import sim2real (the dependency runs the other way). So a landing would add one parity assertion
  `family_records.FLOOR_LIMIT == sim2real.cards.schema.TWINNED_LIMIT` to that test. sim2real may later fit and select
  through `family[key].fit(...)` and `family.compare(...)`, additively.

## Cost of a new rung (an M3 friction rung)

Exactly what v1 asks today, nothing more: one `Rung(level=3, name=..., base=..., adds=(...), code="actuators/rungs/<word>",
equation=..., references=(...))` entry in `actuators/servo/ladder.py` (plus a `Parameter` for any new quantity), and
the folder `actuators/rungs/<word>/` with `model.py` (`torque(terms, inputs, state, dt) -> (tau_r, state)`) and
`model.json` (its terms marked required or not, inputs, output, MuJoCo maps); robotparts' parity also wants `model.js`
and the ruler's sha256 table. The Family finds the folder through `code`. Binding, the card, fitting, the held-out
comparison against the floor (the new rung is placed by its depth in the tree) and the MuJoCo export all come free.
There is no registration and no fitting or scoring code.

## What is simulated or left out

- The Family is derived at runtime from today's `ladders()` instead of being declared in `src/`.
- Physics is attached for M0–M3 (ideal, datasheet, coulomb_viscous, stribeck, load, lugre); M4.ripple has a `model.py`
  but is not attached (stop line); M3.weakening_capture and M4+ have no runnable code.
- `Physics.__call__` vectorizes by looping the scalar stdlib `torque()` over samples (no numpy rewrite of the physics);
  state and `dt` are not threaded through, so LuGre runs at steady state.
- `Values` does not resolve aliases on read (it canonicalizes on build) and refuses a bare number for an alias with a
  non-SI unit (`noload_speed_rpm`), asking for a `Stated`.
- Fit uncertainties, parameter bounds (a negative breakaway is accepted), sim2real's coverage rule for residual and
  learned rungs (`RungScore.in_coverage`), cross-validation folds beyond one fit/test split, and composed types
  (`foc_controller`) are not modelled. Servo B's step reuses servo A's datasheet values for the terms a fit does not
  vary (inertia, gear ratio). The friction physics never reads them.
- Core size: 305 non-blank lines in `family_records.py` (282 before amendment 1; over the 250 aim; about 60 are
  docstrings and WHY comments).
