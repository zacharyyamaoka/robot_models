# v7 · Family per unit

**Thesis.** The ModelFamily *class* is the type and holds the Model classes. A ModelFamily *instance* is one physical
servo: it holds one instance of every Model with that unit's values. So the object you fit, choose on and hand to
sim2real/MuJoCo is the servo itself: `servo = IntegratedServo(datasheet=ds).fit(log)`, `servo.select_best(held_out)`,
`servo.best`.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v7_family_per_unit/scenario.py
```

Check (amendment 1): `PASS  v7_family_per_unit`. Case A chose M3.stribeck (held out ÷ floor: M2 20.28, M3 1.00).
Case B chose M2.coulomb_viscous (M2 1.03, M3 1.57). The session is in [scenario.out.txt](scenario.out.txt). The core
is [family_per_unit.py](family_per_unit.py): 473 non-blank lines, 421 without docstrings and comments, **over the
350 aim**. About 60 of those lines are the tree text and its HTML, 30 the `Models` accessor, 30 the numpy
Levenberg-Marquardt and 25 the selection table. The family is [servo_family.py](servo_family.py): 14 classes, 160
lines, equations and references included. [tree.html](tree.html) is what `IntegratedServo.tree().show(path)` writes.

## His access shape, as it runs

```python
from servo_family import IntegratedServo            # a ModelFamily: the type
IntegratedServo.models["M3.stribeck"]               # the class Stribeck
IntegratedServo.models.m3.stribeck                  # the same class; .m3 is itself a Models (stribeck, load, lugre, ...)
IntegratedServo.ladder()[0]                         # <model M0.ideal · class Ideal(IntegratedServoModel) · 2 params>
IntegratedServo.tree().print()                      # text tree: chains on one line, branches where siblings exist
IntegratedServo.tree().show("tree.html")            # writes a self-contained HTML tree with parameter tables, returns the path
for model in IntegratedServo: print(model.key, model.param_count)

servo = IntegratedServo(datasheet=ds, name="servo A")   # one unit: an instance of every Model, with the datasheet's values in SI
servo.models.m1.datasheet.predict(1.0)              # 0.32 N·m; numpy arrays in, numpy arrays out
servo.fit(fit_log)                                  # fits every model the log can identify, in place; returns the unit
servo.models.m3.stribeck.params                     # this unit's values
servo.models.m3.stribeck.fit(fit_log)               # one model, in place
Stribeck(output_inertia=4.2e-5, gear_ratio=10).fit(fit_log)   # a standalone model, no unit
servo.select_best(held_out)                         # the table; servo.best is the chosen model instance
servo.best.mujoco()                                 # {'armature', 'frictionloss', 'damping'}: by the rung's model.json maps
IntegratedServo.fit(log, datasheet=ds, name="B")    # the batch call on the type builds and returns a unit
```

## The object model

```
ModelFamily (class = the type; instance = one physical unit)
├─ class side    IntegratedServo.models / .ladder() / .tree() / iter / .canonical(alias) / .as_ladder() -> v1 Ladder
│  └─ Model classes   Ideal ─ Datasheet ─ CoulombViscous ─ Stribeck ─ LuGre ...   (each rung subclasses its base)
│     └─ Parameter    Stribeck.breakaway_torque -> Parameter(unit "N·m", definition, required)
└─ unit side     servo = IntegratedServo(datasheet=ds, name=...)
   ├─ servo.models.m3.stribeck   a Stribeck instance: .params .predict(v) .fit(log) .parity(log) .mujoco()
   ├─ servo.fit(log) -> servo    lowest rung first; each starts from the fitted rung below it; .status per model
   └─ servo.select_best(held_out) -> Selection; servo.best
```

- **Parameter** is a descriptor (a frozen dataclass with `__get__`). Read on the class it is the record: unit,
  definition, required or optional. Read on a model instance it is that unit's number. `model.parameter(name)` gives
  the record with `value` filled in, so value, unit and definition come back together. Type checkers see
  `servo[Stribeck].breakaway_torque` as a `float`.
- **Additive vocabulary, enforced at class creation.** In `__init_subclass__` a rung may add a Parameter, or promote an
  optional one to required (M1 states `coulomb_friction`; M2 needs it). Each of these raises before the class is
  registered: redeclaring a parameter the base already needs, making a required one optional, setting an inherited
  parameter to anything that is not a Parameter (removing it), and using an alias or an unknown name (with a
  did-you-mean hint). The scenario shows one of these errors, and shows the broken rung is not in the family.
- **A class is a rung by being declared.** `class X(IntegratedServo.Model or a rung, level=, name=)` registers itself on
  the family. No record has to be typed a second time, so nothing can drop out of the ladder.
  `IntegratedServo.as_ladder()` builds today's v1 `Ladder` from the classes, and `Ladder.__post_init__` re-runs all of
  v1's checks on it. `IntegratedServo.ladder_diff(ladders()["integrated_servo"])` returns `[]`: the keys, bases, adds
  and optional names match today's ladder exactly.
- **Runnable means "defines its own `torque`."** LuGre inherits Stribeck's `torque`. Running it under LuGre's name
  would be a silent lie, so a rung without its own `torque` is *declared*: you can browse it, and fitting skips it with
  a stated reason.

## Decisions I took

| | |
|---|---|
| Where values live | On a ModelFamily instance, one per physical unit, holding one Model instance per rung |
| Hybrid methods | `fit`, `ladder`, `tree`, `describe` and `parameter` run on the class (browse, or build a fresh instance) and on an instance (its values), through one small `both` descriptor |
| Physics | numpy `torque` on each rung class. `model.parity(log)` holds it to the package's stdlib `model.py`: max difference 0.0 on M3 |
| Fitting | One generic Levenberg-Marquardt. Starting values depend on the unit, not the rung: a torque starts at the median \|tau\|, a damping at 0, and a speed scale gets 6 starts across the logged speeds and is fitted as its log (it is positive). **No `seeds()` override per rung.** |
| What a fit moves | Only the needs the log's torque responds to, tested at the seed and at a jittered point. Inertia and ratio ride along from the datasheet. If a need is unknown and the log cannot see it, fitting refuses with "state it" instead of inventing a value |
| Climb | A unit fits lowest first, and each rung starts from the fitted rung below it |
| Selection | sim2real's rule: the lowest fitted rung in ladder order whose held-out rms ÷ the pooled within-plateau floor is ≤ 1.5. The table also shows the fit rms for contrast |
| Black box | `M7.learned` is a per-direction ridge regression on `[1, \|v\|, 5 Gaussian bumps in log\|v\|]`, 14 weights, numpy. It stands in for today's MLP and uses the same calls. It fits its training plateaus best (3.92 mN·m) and loses held out (4.49× floor on A, 2.55× on B) |
| Vocabulary | Aliases, packed names and deployment-only names are today's servo type data, imported. They are the type's dictionary, not the ladder's structure; in a migration they move onto the family class |
| `show()` | Writes HTML and returns the path. It never opens a window |

## Best when / loses when

**Best when** the thing you carry is a *servo*, not a model. sim2real twins one physical unit, the rig fits one unit,
and MuJoCo gets one unit's numbers. Fleets fall out naturally: `[IntegratedServo(datasheet=ds, name=sn).fit(log_sn)
for sn in serials]`, and each unit says which rung its own data paid for (A → M3, B → M2 from the same datasheet). The
batch call and its status table answer "what happened to every model" in one place.

**Loses when** you want values as plain data: to diff two fits, store one model's values without its unit, or share
values across units. A unit is a mutable object graph, and `fit` changes it in place. v8's Parameters sets win there.
Each unit also builds all 14 model instances, even the declared ones. And hybrid methods (`fit` on the class gives a
new instance, on an instance fits it in place) are one more thing to learn. The core also came in over its line
budget.

## Migration from v1

- The `Rung(...)` records in `actuators/servo/ladder.py` become classes like the ones in `servo_family.py`, and
  `LADDER = IntegratedServo.as_ladder()` keeps every v1 consumer working: `ladders()`, `fallback()`, `parameters()`,
  robotparts' vendored rung bytes (the rung folders are untouched) and sim2real's parity test. The `ladder_diff` guard
  is the test that proves the swap changed nothing.
- `actuators/rungs/*/model.py` stays as the stdlib ruler (robotparts' and JS parity). The numpy `torque` is checked
  against it by `parity` in a test, not retyped by hand without one.
- The name `IntegratedServo` is today's *datasheet* class. Here the family takes the name and the datasheet is
  imported as `IntegratedServoDatasheet`. In `src/` one of them must be renamed (suggested: `IntegratedServo.Datasheet`
  as a nested alias).

## Cost of adding a rung

One class: its Parameters, a docstring (the title), `equation`, `references`, and `code` if the package ships its
physics. Add a vectorized `torque` only if it should run. It is then in `models`, `ladder()`, `tree()`, `as_ladder()`,
every unit, the batch fit and `select_best`. No seeds, no registry, no second record. Its first fit starts from the
rung below it and the by-unit starting values.

## What is simulated

The rig logs come from the shared fixture (a hidden Stribeck truth for A, plain Coulomb + viscous for B, plus noise).
The datasheet is the fixture's illustrative servo. M0–M3.stribeck and M7 run. M3.load runs but the log has no
`tau_load`. M3.lugre, M3.weakening_capture and M4–M6 are declared only. M7 is a small numpy stand-in, not the MLP.
