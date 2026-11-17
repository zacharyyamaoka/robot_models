# v5 Type kit

**Thesis.** The Type is the entry point and hands you everything it has: its Models (the ladder), its Datasheet class,
the Experiments (rig motions) that identify each Model, and its fidelity metric. A Model is the base object inside the
kit and names, per parameter, the experiment that identifies it, so "I have the datasheet and the rig log, what now?" is
`servo.identify(log, datasheet=ds)`: the kit gives values to every model the log and the datasheet cover, and names the
motion still to run for each one it cannot.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v5_kit/scenario.py
```

The transcript is [scenario.out.txt](scenario.out.txt); the core is [type_kit.py](type_kit.py) (330 non-blank lines, of
which about 75 print the tree, the model card and the tables); the check prints `PASS  v5_kit`.

## The object model

```
types                         Types: every type's kit by name; types.of(datasheet) picks it from the datasheet's class
└─ Kit  (types.integrated_servo)      a layer over today's v1 Ladder, which it keeps whole as kit.ladder
   ├─ models       Models {key: Model}, ladder order; print() draws the tree with each model's motions
   ├─ datasheet    IntegratedServo (the v1 Datasheet class)
   ├─ experiments  Experiments {id: Experiment(id, motion, identifies)}; .covered_by(log)
   ├─ fidelity     FidelityMetric("tau_r_rms"): RMS over the rig's within-plateau scatter, twinned at 1.5x
   ├─ canonical(name), stated(ds), from_datasheet(ds) -> Model, identify(log, datasheet) -> Identified
   └─ compare(found, log) -> Comparison(rows, chosen)
Model    rung, needs, reads, identified_by {parameter: Experiment}, base, physics (the rung's model.py), values
         .fallback  .experiments  .with_values(**si) -> Model  .predict(v) -> numpy  print() -> its card
```

A Model with no values is the declaration. `with_values` returns a new frozen Model, and that Model predicts. Fitted
models and the datasheet model are the same class.

## Prior art borrowed

- **sim2real `models/camera.py`, `CameraKit` and `kits_for`**: the kit shape (rungs + identify routines + fidelity
  metrics in one object per type id), the `FidelityMetric` record (what is compared, the floor it is divided by, how it
  is evaluated), and `TWINNED_LIMIT = 1.5`: a model is twinned within 1.5x the real thing's own floor.
- **Hugging Face `AutoModel`**: a name is enough to get everything (`types.integrated_servo`), and so is the config.
  `types.of(datasheet)` resolves the type from the datasheet's class, the way `AutoModel.from_config` resolves the
  architecture from `config.model_type`.
- **robotparts `model.json` `measured_by`**: each Experiment's id and motion are derived from it (`"slow plateaus below
  0.5 rad/s"` becomes `slow_plateaus`). The source is per rung and per term, so the same parameter is identified
  differently on different rungs: `coulomb_friction` is "reading the datasheet" on M1 and "velocity plateaus" on M2.
  Nothing is retyped. Names come through `Ladder.resolve`, so LuGre's `sigma2` and MuJoCo's packed `motorconst` land on
  the type's canonical names.

## Key decisions

1. **The experiment belongs to the (model, parameter) pair, read from model.json.** `identify()` asks whether this log is
   that motion (`Experiment.covers`), then whether the datasheet states the parameter. If neither, the model goes to
   `missing` with the step to take: `M3.lugre: run small_reversals`. The one new piece of data is `COVERAGE`, which says
   what counts as each motion: "both directions, 6-8 speeds" means 6 or more plateaus held both ways. Motions it cannot
   read say so instead of pretending.
2. **Fitting lives in the kit, and it calls the rung's own physics.** It is separable least squares. A friction rung is
   linear in its levels (N·m) and slopes (N·m·s/rad), and `lstsq` solves those with columns evaluated through
   `model.py`. Its one speed scale (rad/s) is scanned over the speeds the log held and then refined by golden section.
   Superposition is checked at the answer, so a rung the fitter cannot handle fails loudly. Optional terms
   (`stribeck_exponent`) are read or defaulted, never fitted, as robotparts' `required: false` says.
3. **The choice is made by the type's fidelity metric, not an information criterion.** The kit picks the lowest rung
   whose RMS is within 1.5x the rig's own within-plateau scatter: the climb stops where the rig can no longer tell the
   model from the part. On the fixture: M0 131x, M1 57x, M2 17x, M3.stribeck 0.99x, so M3 is chosen. M2's fitted
   `viscous_friction` comes out negative (-0.033): a constant-friction model bends its slope to imitate the missing
   Stribeck drop.
4. **The rung code is exec'd from source, never imported.** An import would leave `__pycache__/` in a vendored rung
   folder whose file set robotparts hashes (PEP 0004 §8.1).

## Best when / loses when

- **Best when** the question is "what do I run next?": a new part arrives, and the kit says which models the datasheet
  already gives, which rig motion unlocks which rung, and when to stop climbing. One object per type also matches how a
  robot assigns types (sim2real's kits), so other types can join without new idioms.
- **Loses when** you want the Model alone. It still has its physics and card, but its `identified_by` points at the
  kit's Experiment objects, so building one by hand outside a kit is awkward. Parameters are a `values` mapping, not
  typed fields, so type checkers see nothing (fr6), unlike v1's dataclass-per-rung direction. The kit is also only as
  rich as model.json: types with no rung folders (camera, conveyor, ...) get models and a tree, but no experiments and
  no fidelity metric yet.

## Migration from v1

Additive. Nothing in v1 moves.

- **v1 ruler** (`tests/test_ladders_acceptance.py`): unchanged. `Kit` wraps `Ladder` (`kit.ladder`), so every rung key,
  base, `parameters()`, alias and packed name is still v1's. Landing this means adding `src/robot_models/types.py`, plus
  one new test: every `measured_by` motion in a rung's model.json either has a `COVERAGE` entry or is declared
  unreadable.
- **robotparts vendoring of `actuators/rungs/*`**: unchanged. The kit reads model.json and model.py read-only and leaves
  no `__pycache__`, so robotparts' byte-for-byte sha256 sync still holds.
- **sim2real `tests/test_robot_models_parity.py`**: unchanged; it reads `ladders()`, which stays. Optional later step:
  sim2real's `CameraKit` and this `Kit` converge on one shape. Then sim2real's camera kits become robot_models kits plus
  its own identify routines, and `kits_for` keeps its members walk.

## Cost of a new M3 friction rung

1. Add a `Rung(level=3, name=..., base=..., adds=..., code="actuators/rungs/<word>", equation=..., references=...)` to
   `actuators/servo/ladder.py`, as today.
2. Add the folder `actuators/rungs/<word>/` with `model.py` (`torque(terms, inputs, state, dt)`) and `model.json`.
   Each term gets a `measured_by`. This is robotparts' format; vendor it as today.

The kit finds both with no registration: the model, its card, its place on the tree, its experiments (a new motion text
becomes a new Experiment automatically), `predict`, and `identify` once a log covers its motions. It costs one more
line only when the rung names a new motion that `identify` should read from a log: a `COVERAGE` entry. It needs a new
fitter only when the rung is not "linear in levels and slopes, plus one speed scale".

## What is simulated or left out

- The rig log is the fixture's simulation. Plateaus are found by the commanded speed repeating exactly (rounded to
  1e-6). A real log needs a plateau index.
- Only velocity-plateau logs are read (`velocity_plateaus`, `slow_plateaus`). Chirps, reversals, pendulum sweeps and
  board calibration are named, but their logs are not.
- The fidelity is in-sample: it is evaluated on the log that was fitted. sim2real evaluates on held-out data, and a
  held-out log goes in the same `compare(found, log)` slot.
- Physics runs for M0-M3 only: M0, M1, M2, M3.stribeck, M3.load (needs a `tau_load` input) and M3.lugre (steady state
  when no `dt` is given). M3.weakening_capture has no published physics. M4-M7 are declared, and calling `predict` on
  them raises "declared: this round runs M0-M3 only".
- `types` builds every kit when it is imported (about 0.4 s, including today's `ladders()` walk). In the package it would be `robot_models.types`. Here the
  module is `type_kit.py`, so that it does not shadow the stdlib's `types` on `sys.path`.
