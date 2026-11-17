# v4 · Estimators that climb

**Thesis.** A Model is an estimator: `Model.from_datasheet(ds)`, `.fit(log)`, `.predict(v)`, `.score(log)`,
`.get_params()`. A Family is model selection: `family.climb(datasheet, log)` starts at M1 from the datasheet, fits
every rung that extends the current one, and climbs only while the next rung pays for its added parameters. The
ladder is the search order.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v4_estimators/scenario.py
```

Check: `PASS  v4_estimators`. The session is in [scenario.out.txt](scenario.out.txt), the code in
[estimators.py](estimators.py) (295 non-blank lines, docstrings and WHY comments included).

## "I have the datasheet and the rig, and I picked the models. What now?"

```python
m1 = Model.from_datasheet(ds)                                  # M1.datasheet: the stated values, in SI
m1.predict(1.0)                                                # 0.32 N·m
m3 = Model.from_datasheet(ds, "M3.stribeck").fit(log)          # the datasheet's values, then least squares
print(servo.climb(ds, log))                                    # which rung this log pays for
```

```
climb integrated_servo on 500 samples: BIC = n·ln(RSS/n) + k·ln(n), climb while ΔBIC < −2
  rung                   k  rms N·m    BIC   ΔBIC  verdict
  M1.datasheet           0   0.2322  -1460      ·  start: the datasheet, nothing fitted
  M2.coulomb_viscous     2   0.0689  -2662  -1202  pays → climb
  M3.stribeck            4   0.0041  -5482  -2820  pays → climb
  M3.load                ·        ·      ·      ·  skipped: the log has no tau_load
  M4.ripple              ·        ·      ·      ·  skipped: declared, above the M0–M3 stop line
  M3.lugre               ·        ·      ·      ·  skipped: the log has no dt
  M3.weakening_capture   ·        ·      ·      ·  skipped: declared, no physics shipped
chosen M3.stribeck: nothing above it fits this log
```

Keep only the plateaus at 0.8 rad/s and above, and the same call stops at M2 (step 6): M3.stribeck fits the noise
just as well (rms 0.0039 for both) but costs +10 BIC.

## The object model

```
families() -> {type: Family}           built from robot_models.ladder.ladders(); nothing retyped
Family (one type's models, as its ladder)
  print(family)                        the tree: base → the rungs that extend it, ° marks a declared rung
  .model(key, **values) .models()      a Model per rung, unfitted
  .children(key) .siblings(key) .canonical(name)
  .climb(datasheet, log) -> Climb      model selection up the tree
Model (an estimator for one rung: its family, its Rung, {canonical name: SI value})
  what it is   .key .needs .holds .free .fallback .inputs .describe()     from the Ladder and model.json
  verbs        Model.from_datasheet(ds, key)  .set_params(**any_name)  .get_params()
               .fit(log) -> self  .predict(v | log)  .score(log)  .aic(log)  .bic(log)
  learned      .fitted_  .rms_  .n_samples_                               sklearn's trailing underscore
Climb  .rows (rung, k, rms, BIC, ΔBIC, verdict)  .fits {key: Model}  .chosen  .stop
```

The torque always comes from `actuators/rungs/<word>/model.py`, loaded by path. Its inputs and term names come from
the rung's `model.json`, and term names resolve through the ladder's aliases, so LuGre's `sigma2` reads
`viscous_friction`.

## Prior art borrowed

- **scikit-learn's estimator API.** It supplies the verbs (`fit` returns self, `predict`, `score` as R²,
  `get_params`/`set_params`) and the convention that whatever `fit` learned ends in `_`.
- **scikit-learn's `GaussianMixture.aic(X)` / `.bic(X)`.** Each estimator scores its own information criterion.
- **AIC/BIC selection** (Akaike 1974; Schwarz 1978). BIC = n·ln(RSS/n) + k·ln(n) for Gaussian residuals. The climb
  margin of 2 follows Kass & Raftery (1995), who call a BIC difference under 2 "not worth more than a bare mention".
- **SysIdentPy's structure selection.** Its FROLS keeps adding regressors while an information criterion improves.
  Here the candidate terms are whole rungs, and the order they are tried in is the ladder.
- **Levenberg–Marquardt** (Marquardt 1963), written out in numpy because the venv has no scipy.

## Decisions

1. **`get_params()` returns the physical values, stated and fitted alike, in SI.** scikit-learn separates
   hyperparameters from `coef_`. Here the values are what MuJoCo or sim2real gets handed, so they are the params, and
   `fitted_` says which ones the log set. Undo: rename `get_params` to `params_` and keep `get_params` for fit settings.
2. **k counts fitted values only.** A value the datasheet states costs the log nothing, so M1 is charged 0, and each
   rung pays for the terms it adds above M1. Optional terms (`stribeck_exponent`) keep their default and are not
   fitted. A rung is never charged for something it does not need.
3. **One generic fit, no per-rung fitting code.**
   - Levenberg–Marquardt runs over `model.py`.
   - A value already held starts where it is: the datasheet's value, or the rung below's value during a climb.
   - A missing value starts from its unit. A torque starts at the log's torque scale. A speed is scanned across the
     log's speeds, because Stribeck's speed is the nonlinear term.
   - A term whose sign the log cannot see (Stribeck's speed enters squared) is reported positive. Without this, the
     climb's warm-started fit came back with −0.249 rad/s where step 4 had +0.249.
4. **The climb is greedy over children.** At each step, every rung that extends the current one competes, and the
   best moves up only if ΔBIC < −2. A rung that this log cannot feed (a missing input column, or physics that is
   declared but not runnable) is listed with the reason it was skipped, not dropped.

## Best when / loses when

**Best when** the question is "which rung does my data justify, and with which numbers": a rig log per unit, a
defensible pick, and values in SI ready for a simulator. The scenario's "what now" becomes three calls, and the
choice is explained by a table, not by taste.

**Loses when:**
- **The job is browsing or authoring the type.** The ladder is reached through models and the family, and parameters
  are a dict, so type checkers see no fields.
- **The siblings need different experiments.** LuGre needs a time series and load friction needs `tau_load`, so one
  plateau log mostly prints "skipped".
- **The residuals are correlated.** Samples on one real plateau are not independent, so BIC over-rewards added terms
  unless you fit plateau means.
- **A higher rung pays only as part of a pair.** The greedy climb can miss it.

## Migration from v1

- **This is an additive layer.** It only reads `robot_models.ladder.ladders()` (Ladder, Rung, Parameter, Alias) and
  the rung folders. It would ship as `robot_models/estimators.py` beside `ladder.py`.
- **v1 ruler (`tests/test_ladders_acceptance.py`).** Untouched. The ladder data and the rung bytes it pins do not
  change. A new test file would hold the scenario's grade: M3 recovers a hidden Stribeck truth, and the climb picks
  M3 on the full log and M2 on fast plateaus only.
- **robotparts vendoring (`actuators/rungs/*`).** Untouched. `model.py` and `model.json` are read, never rewritten,
  so the sha256s in `model.json` and the ruler's byte check stay valid.
- **sim2real (`tests/test_robot_models_parity.py`).** Untouched. It checks card names against `ladder.resolve()` and
  `holds()`, which are unchanged. Later, sim2real could take `model.get_params()` (canonical names, SI) instead of
  mapping fitted values itself, as an opt-in.

## Cost of adding a new M3 friction rung

The same as v1 today, and nothing more:

1. A `Rung(level=3, name=..., base=..., adds=..., code="actuators/rungs/<word>", equation=..., references=...)` in
   `actuators/servo/ladder.py`.
2. The `actuators/rungs/<word>/` folder, holding `model.py` (`torque(terms, inputs, state, dt)`) and `model.json`
   (`inputs`, `terms`).

It then appears in the tree, `from_datasheet` fills it, and `fit` fits its required terms above M1. The climb tries
it as a child of its base and keeps it only if it pays. The one exception is a rung that least squares cannot fit
(a special experiment, a bounded term). That would need a per-rung `fit` override, and that hook is not built here.

## What is simulated or left out

- **Runnable physics stops at M0–M3**, per the stop line. M4–M7 raise `NotRunnable` and say why.
  `M3.weakening_capture` has no physics shipped.
- **`M3.lugre` has `predict` only.** It carries its state through a time series, but no log here has `dt`, so its
  fit is never exercised.
- **`predict` loops in Python** over the scalar stdlib `model.py`. Arrays go in and come out, but one M3 fit takes
  about 1–3 s.
- **The fixture's rig log is simulated** (velocity plateaus plus i.i.d. noise). The variant never reads the hidden
  values.
- **Not built:**
  - parameter uncertainty (covariance) and cross-validation;
  - composition with other types (`M4.drivetrain` composes `foc_controller`);
  - sklearn's `clone` / `check_estimator` compliance; `fit(log)` takes a dict of columns, not `(X, y)`.
