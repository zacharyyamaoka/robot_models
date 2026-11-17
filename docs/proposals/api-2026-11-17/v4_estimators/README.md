# v4 · Estimators that climb

**Thesis.** A Model is an estimator: `Model.from_datasheet(ds)`, `.fit(log)`, `.predict(v)`, `.rms(log)`,
`.score(log)`, `.get_params()`. A Family is model selection: `family.climb(datasheet, fit, test)` starts at M1 from
the datasheet, fits every rung that extends the current one on the fit plateaus, judges it on the held-out ones, and
climbs only while the current rung is still above 1.5× the rig's noise floor and a child beats it held out. The
ladder is the search order.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v4_estimators/scenario.py
```

Check (amendment 1): `PASS  v4_estimators`; case A chose M3.stribeck (held out ÷ floor: M2 20.28, M3 1.00), case B
chose M2.coulomb_viscous (M2 1.03, M3 1.57). The session is in [scenario.out.txt](scenario.out.txt), the code in
[estimators.py](estimators.py) (313 non-blank lines, docstrings and WHY comments included).

## "I have the datasheet and the rig, and I picked the models. What now?"

```python
m1 = Model.from_datasheet(ds)                                  # M1.datasheet: the stated values, in SI
m1.predict(1.0)                                                # 0.32 N·m
fit, test = rig_log("A", "fit"), rig_log("A", "test")          # odd plateaus to fit, even ones held out
m3 = Model.from_datasheet(ds, "M3.stribeck").fit(fit)          # the datasheet's values, then least squares
print(servo.climb(ds, fit, test))                              # which rung the held-out plateaus pay for
```

```
climb integrated_servo: fit on 250 samples, judged on 250 held out; rms in N·m, noise floor 0.00417; climb while held out > 1.5× floor
  rung                   k  fit rms held out ÷ floor  verdict
  M1.datasheet           0  0.24200  0.22197   53.22  start: the datasheet, nothing fitted
  M2.coulomb_viscous     2  0.06469  0.08458   20.28  lowest held-out → climb
  M3.stribeck            4  0.00395  0.00418    1.00  lowest held-out → climb
  M3.load                ·        ·        ·       ·  skipped: the log has no tau_load
  M4.ripple              ·        ·        ·       ·  skipped: declared, above the M0–M3 stop line
  M3.lugre               ·        ·        ·       ·  skipped: the log has no dt
  M3.weakening_capture   ·        ·        ·       ·  skipped: declared, no physics shipped
chosen M3.stribeck: the lowest rung within 1.5× the noise floor on held-out plateaus
```

The same call on servo B, a plain Coulomb + viscous servo (step 6), stops at M2. M3 fits its own plateaus a hair
better (0.00397 vs 0.00404) and is 1.5× worse on the ones it never saw (0.00662 vs 0.00433): it put a spurious
Stribeck hump (0.38 N·m breakaway at 1.16 rad/s) between the fit plateaus.

```
  M2.coulomb_viscous     2  0.00404  0.00433    1.03  lowest held-out → climb
  M3.stribeck            4  0.00397  0.00662    1.57  not needed: M2.coulomb_viscous reaches the floor
chosen M2.coulomb_viscous: the lowest rung within 1.5× the noise floor on held-out plateaus
```

## The object model

```
families() -> {type: Family}           built from robot_models.ladder.ladders(); nothing retyped
Family (one type's models, as its ladder)
  print(family)                        the tree: base → the rungs that extend it, ° marks a declared rung
  .model(key, **values) .models()      a Model per rung, unfitted
  .children(key) .siblings(key) .canonical(name)
  .climb(datasheet, fit, test) -> Climb   model selection up the tree, judged on held-out data
Model (an estimator for one rung: its family, its Rung, {canonical name: SI value})
  what it is   .key .needs .holds .free .fallback .inputs .describe()     from the Ladder and model.json
  verbs        Model.from_datasheet(ds, key)  .set_params(**any_name)  .get_params()
               .fit(log) -> self  .predict(v | log)  .rms(log)  .score(log)
  learned      .fitted_  .rms_  .n_samples_                               sklearn's trailing underscore
noise_floor(log)                       pooled within-plateau std: the held-out rms no rung can beat
Climb  .rows (rung, k, fit rms, held out, ÷ floor, verdict)  .fits {key: Model}  .held_out  .floor  .chosen  .stop
```

The torque always comes from `actuators/rungs/<word>/model.py`, loaded by path. Its inputs and term names come from
the rung's `model.json`, and term names resolve through the ladder's aliases, so LuGre's `sigma2` reads
`viscous_friction`.

## Prior art borrowed

- **scikit-learn's estimator API.** It supplies the verbs (`fit` returns self, `predict`, `score` as R²,
  `get_params`/`set_params`) and the convention that whatever `fit` learned ends in `_`.
- **scikit-learn's validation split** (`PredefinedSplit`, `validation_curve`). Fit on one set, score on another;
  model complexity is chosen where the validation score stops improving. Here the split is the rig's own: odd
  plateaus fit, even plateaus held out.
- **sim2real's selection rule** (`src/sim2real/cards/schema.py` `select()`, `TWINNED_LIMIT = 1.5`). The lowest rung
  whose held-out error ÷ the device's noise floor ≤ 1.5. The climb is that rule walked up a tree instead of a chain.
- **SysIdentPy's structure selection.** Its FROLS keeps adding regressors while a criterion improves. Here the
  candidates are whole rungs, the order is the ladder, and the criterion is held-out rms against the floor.
- **Levenberg–Marquardt** (Marquardt 1963), written out in numpy because the venv has no scipy.

## Decisions

1. **`get_params()` returns the physical values, stated and fitted alike, in SI.** scikit-learn separates
   hyperparameters from `coef_`. Here the values are what MuJoCo or sim2real gets handed, so they are the params, and
   `fitted_` says which ones the log set. Undo: rename `get_params` to `params_` and keep `get_params` for fit settings.
2. **The climb judges on held-out plateaus against the noise floor, not on training BIC** (amendment 1). The v0 climb
   ran `ΔBIC < −2` on the fit log. Measured on the fit split before the change, it did stop at M2 on servo B, but by
   3.2 BIC points (−2745.1 vs −2741.9), with M3's training rms *lower* (0.00397 vs 0.00404). Held out, the same two
   fits separate by 1.5× (1.03 vs 1.57). A rung that nests the one below always fits its own data at least as well, so
   only data it never saw can say no. The rule is sim2real's, so a rung chosen here is "twinned" by the same yardstick
   there. Undo: restore `Model.bic` and compare `ΔBIC` in `Family.climb` (both in git history).
3. **The climb stops at the first rung that reaches the floor, even if a child scores lower held out.** Below the
   floor there is nothing left but noise, so the lower rung wins (sim2real's "lowest rung" rule). Short of the floor
   it climbs only to a child that beats the current rung held out, Zach's rule: a richer model that does not improve
   test time performance is overfitting. Undo: drop the `reached` test in `Family.climb` to climb whenever a child
   improves held out.
4. **The noise floor is a function of the log (`noise_floor(test)`), not of a Model.** It is the held-out rms of the
   one estimator no rung can beat, the one that knows each plateau's own mean, so it belongs to the data. The climb
   computes it from the held-out log; the check's own oracle agrees (0.0042 N·m in both cases).
5. **The climb fits one rung past the one it stops at.** The table then shows why it stopped: on servo B, M3's 1.57
   sits next to M2's 1.03. A rung no log can feed (a missing input column, or physics that is declared but not
   runnable) is listed with the reason it was skipped, not dropped.
6. **One generic fit, no per-rung fitting code.**
   - Levenberg–Marquardt runs over `model.py`.
   - A value already held starts where it is: the datasheet's value, or the rung below's value during a climb.
   - A missing value starts from its unit. A torque starts at the log's torque scale. A speed is scanned across the
     log's speeds, because Stribeck's speed is the nonlinear term.
   - A term whose sign the log cannot see (Stribeck's speed enters squared) is reported positive.
7. **`scenario.run(case)` hands the check the climb's own fits**, so the check re-scores exactly the evidence the choice
   was made on. The case reaches the steps as the literal the user would type (`rig_log("A", "fit")`); nothing
   branches on it.

## Best when / loses when

**Best when** the question is "which rung does my data justify, and with which numbers": a rig log per unit split into
fit and held-out plateaus, a defensible pick, and values in SI ready for a simulator. The scenario's "what now"
becomes three calls, and the choice is explained by a table of held-out numbers, not by taste.

**Loses when:**
- **The job is browsing or authoring the type.** The ladder is reached through models and the family, and parameters
  are a dict, so type checkers see no fields.
- **The log has no repeated conditions.** The floor needs plateau ids with several samples each; a free-running
  trajectory has no within-plateau spread, so the climb would need another floor (a stationary segment, a
  repeat run) passed in.
- **The siblings need different experiments.** LuGre needs a time series and load friction needs `tau_load`, so one
  plateau log mostly prints "skipped".
- **The plateaus miss a term's speeds.** A feature at speeds no plateau visits (a Stribeck drop finished below
  0.03 rad/s) is invisible to both halves, so the climb picks the lower rung: right for the data, wrong for the
  servo. The held-out split guards against fitting noise, not against an experiment that never excites the term.
- **A higher rung pays only as part of a pair.** The greedy climb can miss it.

## Migration from v1

- **This is an additive layer.** It only reads `robot_models.ladder.ladders()` (Ladder, Rung, Parameter, Alias) and
  the rung folders. It would ship as `robot_models/estimators.py` beside `ladder.py`.
- **v1 ruler (`tests/test_ladders_acceptance.py`).** Untouched. The ladder data and the rung bytes it pins do not
  change. A new test file would hold the scenario's grade: M3 recovers a hidden Stribeck truth from the fit plateaus,
  and the climb picks M3 on a Stribeck servo and M2 on a Coulomb + viscous one, judged on held-out plateaus.
- **robotparts vendoring (`actuators/rungs/*`).** Untouched. `model.py` and `model.json` are read, never rewritten,
  so the sha256s in `model.json` and the ruler's byte check stay valid.
- **sim2real (`tests/test_robot_models_parity.py`).** Untouched today: it checks card names against
  `ladder.resolve()` and `holds()`, which are unchanged. The climb restates sim2real's `select()` rule, so the 1.5
  lives in two repos; if v4 is picked, that parity test gains one assertion, `estimators.FLOOR_LIMIT ==
  schema.TWINNED_LIMIT` (robot_models cannot import sim2real, so the parity test is where the two meet). Later,
  sim2real could take `model.get_params()` (canonical names, SI) instead of mapping fitted values itself, as an opt-in.

## Cost of adding a new M3 friction rung

The same as v1 today, and nothing more:

1. A `Rung(level=3, name=..., base=..., adds=..., code="actuators/rungs/<word>", equation=..., references=...)` in
   `actuators/servo/ladder.py`.
2. The `actuators/rungs/<word>/` folder, holding `model.py` (`torque(terms, inputs, state, dt)`) and `model.json`
   (`inputs`, `terms`).

It then appears in the tree, `from_datasheet` fills it, and `fit` fits its required terms above M1. The climb tries
it as a child of its base, and climbs to it only when the base is still above the floor and it is the child with the
lowest held-out rms. The one exception is a rung that least squares cannot fit (a special experiment, a bounded term).
That would need a per-rung `fit` override, and that hook is not built here.

## What is simulated or left out

- **Runnable physics stops at M0–M3**, per the stop line. M4–M7 raise `NotRunnable` and say why.
  `M3.weakening_capture` has no physics shipped.
- **`M3.lugre` has `predict` only.** It carries its state through a time series, but no log here has `dt`, so its
  fit is never exercised.
- **`predict` loops in Python** over the scalar stdlib `model.py`. Arrays go in and come out; the whole scenario
  (three M3 fits across the two servos) takes about 10 s.
- **The fixture's rig logs are simulated** (velocity plateaus plus i.i.d. noise, two servos). The variant never reads
  the hidden values, and never branches on which servo it was handed.
- **Not built:**
  - parameter uncertainty (covariance) and k-fold cross-validation: one fixed split, the rig's;
  - composition with other types (`M4.drivetrain` composes `foc_controller`);
  - sklearn's `clone` / `check_estimator` compliance; `fit(log)` takes a dict of columns, not `(X, y)`.
