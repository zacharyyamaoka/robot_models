# v8 Parameter sets: ModelFamily → Model (stateless physics) → Parameters (the values, as data)

**Thesis.** A Model class is pure physics composed of Parameter declarations; its values live in a separate, generated
`<Model>.Parameters` — a frozen, typed dataclass you can store, diff, JSON round-trip, compare and hand to MuJoCo — so
"which equation" and "which numbers" never share an object, and the classes are the one source today's `Ladder` is
derived from.

`check_scenario.py v8_parameter_sets` → **PASS** (case A chose M3.stribeck 1.00× floor; case B chose
M2.coulomb_viscous 1.03×, M3 1.57×). Session: [scenario.out.txt](scenario.out.txt); rendered tree:
[integrated_servo_tree.html](integrated_servo_tree.html).

## Object model

```
IntegratedServo(ModelFamily)              the type; iterate it, len() it, .models, .ladder(), .tree(), .fit(), .select_best()
├── class Ideal(Model, level=0, ...)      stateless: Parameter declarations + an optional numpy torque(p, v)
│   ├── .Parameters                       generated frozen dataclass (kw_only), fields = declarations, units in metadata
│   └── class Datasheet(Ideal, level=1, states=ServoDatasheet)
│       └── class CoulombViscous(..., requires=("coulomb_friction", "viscous_friction"))
│           └── class Stribeck(...)       Stribeck.Parameters subclasses CoulombViscous.Parameters
└── class Learned(Model, level=7)         Weights(4, ...) x2: same fit / predict / param_count outside
Parameters instance  .predict(v) .as_dict() .needed() .diff(other) .to_json() .mujoco() .info (FitInfo, compare=False)
```

| his call (ROUND2.md) | here |
|---|---|
| `IntegratedServo.models["M3.stribeck"]` / `.models.m3.stribeck` | both; `models.m3` is itself a `Models` group |
| `IntegratedServo.ladder()[0]` | `IntegratedServo.Ideal` (the class) |
| `IntegratedServo.tree().print()` / `.show()` | text tree (single-child runs on one line); `show(path)` writes a self-contained HTML page (no scripts, no network) plus mermaid source, returns the path, opens nothing |
| `for model in IntegratedServo: model.key, model.param_count` | the metaclass iterates the ladder; `param_count` = needed scalars (weights count each) |
| `model.fit(log); model.predict(v)` | `p = Stribeck.fit(log, start=m1)` → `Stribeck.Parameters`; `Stribeck.predict(p, v)` or `p.predict(v)` — **one spelling away**: `fit` returns values, it does not mutate the model |
| `IntegratedServo.fit(log); IntegratedServo.select_best(held_out)` | `fits = IntegratedServo.fit(log, start=m1)`; `IntegratedServo.select_best(fits, held_out)` — **one argument away**: the family is stateless, so you pass the fits |

## Decisions (each reversible)

- **Values never live on a class or a model instance.** No model instances exist; a `Parameters` set is the only
  holder of numbers. Prior art: PyBaMM `ParameterValues` (separate from models), scikit-learn `get_params` (`as_dict`),
  MuJoCo attribute export (`mujoco()` via the rung's `model.json` maps).
- **`Parameters` classes inherit along the ladder** (`Stribeck.Parameters` ⊂ `CoulombViscous.Parameters`), so an M3
  set IS an M2 set and `CoulombViscous.predict(m3_set, v)` is accepted at runtime: the vocabulary grows on both trees the same way.
- **Additive vocabulary is enforced at class creation**: reassigning an inherited name (`coulomb_friction = None`)
  raises `TypeError`; the only way to change an inherited parameter is `requires=` (optional → needed), which v1 already
  allows. A shared quantity is declared once (`HARMONICS`, `.when_stated()`); two different declarations of one name in
  a family raise.
- **One source.** `_derive()` builds a v1 `Ladder` from the classes on every registration and runs v1's own validators
  (bases, nothing added twice, datasheet = M1, aliases resolve). `IntegratedServo.drift(ladders()["integrated_servo"])`
  returns `[]`: keys, bases, adds, optional, kind, vocabulary set and every unit equal. A rung v1 refuses is rolled back,
  never left half-registered.
- **Physics is numpy, held to today's `model.py`**: a runnable class with `code=` is checked against the stdlib
  reference to 1e-12 on probe values when the class is defined (a drifted equation cannot be declared). Physics is
  per class, never inherited: `M3.lugre` cannot silently run Stribeck's equation.
- **Generic fitting, no per-rung `seeds()`**: Levenberg-Marquardt (numpy only); seeds come from each parameter's
  *unit* (rad/s across the logged speeds, N·m across the logged torques, damping from 0); `positive=True` parameters
  are searched in log space (stribeck_velocity cannot flip sign); parameters the log cannot move (inertia, ratio) must
  come from `start=` or the fit says so.
- **Batch fit climbs**: each rung starts from the fit below it merged over the datasheet set. Models with no physics are
  listed in `fits.skipped`, never dropped silently.
- **`select_best` is sim2real's rule**: held-out rms ÷ pooled within-plateau noise floor, the lowest rung (level, then
  param_count) within 1.5×. The black box (M7, 8 weights) competes in the same table.
- **Rung prose (title = docstring first line, equation, references, wears, composes) is per class, never inherited.**
- **Aliases, packed names and deployment-only names stay family-level tables**, passed in from today's
  `servo/ladder.py` for this prototype (they are not rung data).

## Best when / loses when

- **Best when** values travel: stored next to a rig log, diffed between two servos or two fit dates, serialized to a
  card, compared for equality, shipped to MuJoCo/sim2real — a `Parameters` set is plain frozen data with units in its
  field metadata. Also when one equation is fitted many times (fleet of units): the class is shared, the sets vary.
- **Loses when** you want the object you hold to *be* the fitted model (`m3.torque(v)` on one object, v6's instance);
  here it is `p.predict(v)` — fine — but `fit` cannot mutate, so his `model.fit(log)` literally returns a new set.
  **Type checkers do not see the generated `Parameters` fields** (made by `make_dataclass` at class creation); they see
  the `Parameter` declarations on the model class. Closing that needs a generated `.pyi` stub (not built). The family
  body also nests 14 classes in one `class` statement, so a type checker resolves `IntegratedServo.Stribeck` but a
  sibling reference inside a method would need the family prefix.

## Migration from v1

Additive: nothing in `src/` changes until picked. The servo family (`parameter_sets/servo.py`) replaces
`actuators/servo/ladder.py`'s `VOCABULARY` + `RUNGS`; its `LADDER` becomes `IntegratedServo.as_ladder()`, so
`ladders()`, the v1 ruler, robotparts' vendored rung folders (`model.py`/`model.js`/`model.json` bytes untouched — the
numpy physics is a checked second spelling, like `model.js`) and sim2real's parity test keep reading the same `Ladder`.
`discover()` would look for a `ModelFamily` subclass as well as `LADDER`.

## Cost of adding a rung

One class in the family body (or anywhere: subclassing a rung of a registered family registers it on definition):
its Parameter declarations, a docstring title, `equation`/`references`, and optionally a numpy `torque(p, v)` (plus
`code=` to hold it to a reference `model.py`). No `Rung` record, no `seeds()` override, no registry. Tried out of
scenario: a `class Tanh(IntegratedServo.CoulombViscous, level=3, name="tanh")` with one `positive=True` rad/s parameter
appeared in `as_ladder()`, `IntegratedServo.fit` and `select_best` with no other edit; a parameter named like an alias
(`sigma2`), a clashing redeclaration and a drifted equation were each refused and the family stayed at 14.

## What is simulated / not built

- The rig logs are fixture.py's simulated plateaus; the servo is illustrative.
- Physics runs for M0, M1, M2, M3.stribeck and M7.learned only (the round's stop line); M3.load needs `tau_load` and
  M3.lugre state, so both are declared, as is everything above M3.
- M7.learned is a per-direction 4-feature linear model in `[1, |v|, ln|v|, ln²|v|]`, a numpy stand-in for the MLP v1
  describes; it fits by the same generic LM.
- No `.pyi` stub, no `discover()` change, no robotparts/sim2real run.

## Files

- [parameter_sets/core.py](parameter_sets/core.py) — Parameter, Weights, FitInfo, ParameterSet, Model, ModelFamily,
  Models; parity against the reference (379 non-blank lines)
- [parameter_sets/fitting.py](parameter_sets/fitting.py) — LM, unit seeds, noise floor, Fits, Selection (101)
- [parameter_sets/views.py](parameter_sets/views.py) — Tree (text, mermaid, HTML), describe (67)
- [parameter_sets/servo.py](parameter_sets/servo.py) — the integrated_servo family, 14 classes (170)
- [scenario.py](scenario.py), [scenario.out.txt](scenario.out.txt)
