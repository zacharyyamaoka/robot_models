# v1 · Subclass ladder

**Thesis.** A Model is a frozen dataclass whose fields ARE its parameters; a rung subclasses the rung it extends
(`class Stribeck(CoulombViscous)`), so a type's ladder is its class tree, the fallback is the MRO, and a value is an
instance (`Stribeck(coulomb_friction=..., ...)`) that type checkers and IDEs read field by field.

`check_scenario.py v1_subclass` (amendment 1) → **PASS**: case A chooses M3.stribeck (held-out rms ÷ floor: M2 20.28,
M3 1.00), case B chooses M2.coulomb_viscous (M2 1.03, M3 1.57). The whole transcript (both servos, 14
Levenberg–Marquardt fits) runs in about 1 s.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v1_subclass/scenario.py
```

Files: [subclass_ladder/model.py](subclass_ladder/model.py) (the generic `Model`, 337 non-blank lines),
[subclass_ladder/servo.py](subclass_ladder/servo.py) (integrated_servo as 15 classes, 71 lines),
[scenario.py](scenario.py), [scenario.out.txt](scenario.out.txt).

## The object model

```
Model                                   frozen-dataclass base (PEP 681 dataclass_transform); the generic machinery
└── ServoModel(Model, type="integrated_servo")       the TYPE: root class; tree(), ladder(), canonical(), compare()
    ├── Ideal(level=0)  output_inertia: float, gear_ratio: float           a field with no default: the rung needs it
    │   └── Datasheet(level=1, fields_from=IntegratedServo)               every datasheet field, `= None`: read if stated
    │       └── CoulombViscous(level=2)  coulomb_friction: float, ...     redeclared without default: needed from here
    │           ├── Stribeck(level=3)  breakaway_torque, stribeck_velocity; stribeck_exponent = None; seeds()
    │           │   ├── LuGre(3) → DcMotor(5)
    │           │   └── WeakeningCapture(3) → Drivetrain(4) → Envelope(5) → Wear(5) → Residual(6)
    │           ├── Load(3)
    │           └── Ripple(4)
    └── Learned(level=7)                                                   extends no model
class methods: fallback() = MRO as keys · needs() · reads() · parameter(name) · alternatives() · describe()
               from_datasheet(ds) · fit(log, start=) · seeds(log, start)        → instances (typed as Self)
               compare(models, held_out, fitted_on=) → Comparison: .chosen = lowest rung on the chain ≤ 1.5× floor
instance:      .coulomb_friction ... · torque(v, **inputs) · terms() · mujoco() · score(log) → rms [N·m]
module:        noise_floor(log): the pooled within-plateau std, read from the log itself
```

What the session shows (full transcript in [scenario.out.txt](scenario.out.txt)):

```
>>> print(Stribeck.describe())
M3.stribeck · class Stribeck(CoulombViscous) · Stribeck friction: breakaway falling to Coulomb with speed
  ...
>>> Stribeck.fallback()
('M3.stribeck', 'M2.coulomb_viscous', 'M1.datasheet', 'M0.ideal')
>>> fit, test = rig_log("A", "fit"), rig_log("A", "test")
>>> m2 = CoulombViscous.fit(fit, start=m1)
>>> m3 = Stribeck.fit(fit, start=m2)
>>> print(ServoModel.compare([m1, m2, m3], test, fitted_on=fit))
held-out plateaus · floor 4.17 mN·m · choose the lowest rung within 1.5× floor
rung                 fitted  fit rms  held-out rms  ÷ floor   [rms in mN·m]
M1.datasheet              0   242.00        221.97    53.22
M2.coulomb_viscous        2    64.69         84.58    20.28
M3.stribeck               4     3.95          4.18     1.00  <- chosen

# 6 · servo B: simpler wins   (the same calls on rig_log("B", ...))
held-out plateaus · floor 4.22 mN·m · choose the lowest rung within 1.5× floor
rung                 fitted  fit rms  held-out rms  ÷ floor   [rms in mN·m]
M1.datasheet              0    86.62         91.28    21.63
M2.coulomb_viscous        2     4.04          4.33     1.03  <- chosen
M3.stribeck               4     3.97          6.62     1.57
```

On servo B, M3 fits its own plateaus better than M2 (3.97 vs 4.04 mN·m) and predicts the held-out ones worse (6.62 vs
4.33): the overfit that a training-rms rule rewards, and the reason the choice is made on plateaus the fit never saw.

## Prior art borrowed

| From | What this takes |
|---|---|
| **Modelica `extends`** | A rung extends one base and inherits its parameters; it adds its own, and a *modification* tightens an inherited one (here: M2 redeclares M1's optional `coulomb_friction` with no default, so it becomes required). |
| **PyTorch `nn.Module` subclassing** | You define a model by subclassing; parameters are attributes; `forward` ↔ `torque`; `reset_parameters` ↔ `seeds()` (where a fit starts); `named_parameters` ↔ `needs()` / `terms()`. |
| **Dataclass inheritance + PEP 681 `dataclass_transform`** (pydantic, SQLModel, attrs) | The base applies `@dataclass(frozen=True, kw_only=True)` in `__init_subclass__`, so a rung class needs no decorator and pyright sees every hand-declared field as a typed, keyword-only constructor argument. |
| **Python's C3 MRO** | The fallback chain is the method resolution order read as rung keys. |
| sim2real `cards/schema.py` `select()` | The choice: the lowest rung whose held-out rms ÷ the rig's noise floor ≤ 1.5 (its `TWINNED_LIMIT`). |
| Hold-out validation; Levenberg–Marquardt | Fit on plateaus 1, 3, 5, 7, 9, judge on 2, 4, 6, 8, 10; `fit()` is numpy-only LM (no scipy in the runtime). |

## Key decisions

1. **The ladder is the class tree, found on definition.** `__init_subclass__` registers each rung with its type root,
   so a new rung is found by being declared: no registry file. `fallback()` is the MRO; same-level alternatives are the
   classes at the same `level` (Stribeck and Load are siblings under CoulombViscous; LuGre and WeakeningCapture are
   Stribeck's children, still M3).
2. **Fields are the parameters; the metadata is not retyped.** No default = needs, `= None` = reads when stated. Each
   field's `metadata["parameter"]` (unit, definition) is filled from v1's vocabulary by name, and a field named by an
   alias (`frictionloss_nm`) is refused at class creation, so one vocabulary per type holds (G4). M1's 33 optional
   fields come from `IntegratedServo` (`fields_from=`), because PEP 0003 §6 makes the datasheet's typed fields rung
   M1. **Cost:** that is the one place a type checker loses sight of fields (M0, M2 and up are declared by hand and
   checked).
3. **One copy of the physics and the prose.** `torque()` runs today's `actuators/rungs/<word>/model.py` by path,
   mapped over numpy arrays (a Python loop, not a numpy port); the reference's own term names (`sigma2`) resolve
   through the aliases. Title, equation and references are read from v1's rung record by key, and every class is
   checked against that record at import: fallback, needs (in v1's order) and optional match v1 for all 14 keys.
4. **Fitting is an alternative constructor.** `Stribeck.fit(log, start=m2)` returns a `Stribeck`: generic LM over
   the needed parameters the log actually moves (inertia and gear ratio never move a plateau sweep, so the
   datasheet's values ride along). A rung that adds a nonlinear term overrides `seeds()`; Stribeck tries six
   `stribeck_velocity` starts spread over the logged speeds.
5. **The choice is made on held-out plateaus, against the rig's floor, along the MRO (amendment 1).**
   `ServoModel.compare(models, held_out, fitted_on=fit)` scores each fitted instance on the plateaus it never saw
   (`m.score(log)` is a method on the Model), divides by `noise_floor(held_out)` (pooled within-plateau std, a
   property of the log, so a module function rather than a method of any model), and `.chosen` is the lowest rung
   within 1.5× that floor, or `None` (climb, or measure more).
   - **Why this rule, not BIC.** v0 picked the lowest BIC on the fit log. Held-out data makes the parsimony test
     direct (does the richer rung predict what it never saw?), the floor puts the bar in the rig's own units, and it
     is sim2real's rule, so a choice made here is the one sim2real would make from the same numbers. BIC always picks
     something; this rule can honestly say no rung is good enough.
   - **Why the class tree orders it.** "Lowest rung" needs an order; in this thesis it is the inheritance itself:
     `compare()` sorts by MRO length and refuses a set that is not one chain (`isinstance(upper, type(lower))`, a
     richer rung IS-A simpler one). `compare([m2, m3, load])` raises "M3.load does not extend M3.stribeck".
   - `fitted_on=` only adds a fit-rms column for contrast; nothing is chosen on it.

**Verified beyond the check:**
- **pyright 1.1.390** reports 0 errors on `model.py` and `servo.py`.
- **On a probe file**, pyright reports `m3.stribeck_velocity: float` and `Stribeck.fit(...) -> Stribeck`. It flags a
  missing `stribeck_velocity`, an alias keyword (`frictionloss_nm=`), and an assignment to a frozen field.
- **One thing it rejects** (a second probe): redeclaring an inherited `x: float | None = None` as `x: float` fails
  with "overrides a field of the same name but is missing a default value", even though `kw_only` makes it legal at
  run time. `servo.py` avoids this only because M1's fields are injected, so pyright never sees the optional
  version.

## Best when / loses when

**Best when** rungs really are refinements (each adds terms to its base, as friction does), the people adding them
write Python, and values flow into code: typed attributes, IDE completion, `dataclasses.replace`, `asdict`, keyword
errors caught before run time. Nesting is also what makes the held-out choice well posed: a subclass that reduces to
its base is exactly the case where training error can't choose and held-out error can.

**Loses when**
- **A rung combines two siblings.** Stribeck + load friction needs `class StribeckLoad(Stribeck, Load)`. C3
  linearizes that into one fallback (stribeck_load → stribeck → load → M2 ...), which may not be the fallback you mean.
- **Siblings can't be chosen between.** `compare()` only orders one chain, so "Stribeck or Load at M3?" is two
  comparisons against M2, and picking between two rungs that both reach the floor is left to you (sim2real's rule
  has the same gap: its ladder order breaks the tie).
- **The tree is forced.** `Learned` (M7) extends nothing, but it still has to subclass the type root to be in the
  ladder.
- **The consumer is not Python.** robotparts' JS and JSON readers, and sim2real's cards, can't see a class. The ladder
  has to be exported as records again (see Migration). A rung defined at run time from data needs `type(...)`.
- **"Optional below, needed above" fights the type checker.** That pattern is how v1 handles `coulomb_friction`,
  `viscous_friction`, `backlash`, `kt` and `resistance_line_to_line`. pyright treats it as an error on dataclass
  fields. Declaring M1's fields by hand, the pure form of this thesis, would turn on five such errors. You would need
  a `# pyright: ignore` on each, or a required-ness marker kept outside the default.
- **One namespace.** Parameters share the class namespace with the API (`key`, `level`, `torque`, `fit`). A future
  parameter with one of those names would collide. No current name does, but nothing checks for it.
- **Inheritance is opaque.** The fields a rung needs are spread across 4 to 9 class bodies. `describe()` and
  `needs()` gather them back together, but reading the source alone does not.

## Migration from v1

- **Now (this prototype, additive):** the classes sit beside v1's records and are checked against them at import.
  The selection rule is new API (`score`, `compare`, `noise_floor`) and touches no v1 record. Nothing else changes:
  - the v1 ruler `tests/test_ladders_acceptance.py`;
  - the rung bytes in `actuators/rungs/*`, and so robotparts' `tools/sync_actuator_rungs.py` vendoring and its
    `tests/core/test_actuator_rungs_vendored.py`;
  - sim2real's `tests/test_robot_models_parity.py`, which reads `ladders()` live through `holds()`, `rung()` and
    `fallback()`. Its rule 2 already checks that each sim2real card rung's base lies in the mapped rung's
    `fallback()`, and here `fallback()` is the MRO that `compare()` walks, so the two walk the same order. The 1.5
    limit is copied, not imported (robot_models never reads sim2real), so `SELECT_LIMIT == TWINNED_LIMIT` is the one
    assertion worth adding to that parity test when this lands.
- **Then (the classes become the source):** a two-line `actuators/servo/ladder.py` keeps `LADDER =
  Ladder.from_classes(ServoModel)`. That builds each `Rung` from the classes: `adds` from a class's own required
  fields, `optional` from its defaulted ones, `base` from `__bases__[0].key`. Title, equation and references move into
  the class body.
  - `ladders()` keeps its signature, so sim2real's parity test and the ruler's per-rung assertions (keys, bases, adds,
    optional, equation, references) run unchanged against the generated records.
  - robotparts vendors `model.py` and `model.json` bytes, which do not move.
  - The ruler only needs edits for what the classes don't hold yet: vocabulary, aliases, packed, deployment-only,
    wears and composes. Those either stay as module-level records in the type folder or become class attributes. That
    is a ruler edit, and it lands first, alone.

## Cost of a new M3 friction rung

In this prototype, a contributor writes:

1. A class in the type's models module (here `servo.py`), e.g.
   `class Dahl(CoulombViscous, level=3, name="dahl"):` with `dahl_stiffness: float` in its body.
2. While v1's records are the ruler, the matching `Rung(...)` record and a `Parameter` for `dahl_stiffness` in
   `actuators/servo/ladder.py`. The import-time guard refuses a class with no record.
3. To make it runnable, `actuators/rungs/dahl/model.py` and `model.json`.
4. Only if it adds a term the fit can't start linearly, a `seeds()` override (Stribeck's is 5 lines).

In the end state, step 2 shrinks to the `Parameter`, and the equation and references go in the class body.

Nothing for the choice: `Dahl.fit(fit, start=m2)` and `ServoModel.compare([m1, m2, dahl], test)` work as soon as the
class exists, because `compare()` orders by the MRO and the floor rule is the type's, not the rung's.

## What is simulated or left out

- **The rig:** the log is the fixture's simulated velocity-plateau sweep. M1 comes from the fixture's illustrative
  datasheet.
- **Rungs that don't run:**
  - M4–M7 are declared (fields, fallback, prose) and raise `NotImplementedError` on `torque()`.
  - `M3.weakening_capture` is declared only, because v1 ships no code for it.
  - `M3.lugre` runs only its steady-state sliding curve (no state is carried between calls).
  - `M3.load` runs with `tau_load=`.
- **Speed:** `torque()` calls the stdlib reference once per sample. That's fine for a 500-sample log, too slow for
  simulation.
- **Not yet classes:** composes (`M4.drivetrain` → foc_controller), wears, packed and deployment-only names are still
  read from v1's `Ladder`. Only integrated_servo is expressed as a class tree; the other 11 types are not.
- **The fit:** it frees needed parameters only, so `stribeck_exponent` stays at its default of 2, and it reports no
  parameter uncertainty. It is unbounded: on servo B the overfit M3 lands at `stribeck_velocity = −1.16` (with the
  exponent 2 the curve can't see the sign). Harmless to the choice, which rejects that M3, but a real fit wants a
  positivity bound per parameter.
- **The floor** is read from the held-out log's own repeats; it assumes each plateau is a steady hold. A log without a
  `plateau` id can't be compared.
