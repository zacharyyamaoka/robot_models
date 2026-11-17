# v1 · Subclass ladder

**Thesis.** A Model is a frozen dataclass whose fields ARE its parameters; a rung subclasses the rung it extends
(`class Stribeck(CoulombViscous)`), so a type's ladder is its class tree, the fallback is the MRO, and a value is an
instance (`Stribeck(coulomb_friction=..., ...)`) that type checkers and IDEs read field by field.

`check_scenario.py v1_subclass` → **PASS**. The whole session (7 Levenberg–Marquardt fits) runs in about 0.75 s.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v1_subclass/scenario.py
```

Files: [subclass_ladder/model.py](subclass_ladder/model.py) (the generic `Model`, 305 non-blank lines),
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
instance:      .coulomb_friction ... · torque(v, **inputs) · terms() · mujoco() · score(log)
```

What the session shows (full transcript in [scenario.out.txt](scenario.out.txt)):

```
>>> print(Stribeck.describe())
M3.stribeck · class Stribeck(CoulombViscous) · Stribeck friction: breakaway falling to Coulomb with speed
  ...
>>> Stribeck.fallback()
('M3.stribeck', 'M2.coulomb_viscous', 'M1.datasheet', 'M0.ideal')
>>> m3 = Stribeck.fit(log, start=m2)
>>> print(ServoModel.compare([m1, m2, m3], log))
rung                 fitted  rms [N·m]       BIC
M1.datasheet              0     0.2322   -1460.2
M2.coulomb_viscous        2     0.0689   -2662.1
M3.stribeck               4     0.0041   -5481.7  <- lowest BIC
```

## Prior art borrowed

| From | What this takes |
|---|---|
| **Modelica `extends`** | A rung extends one base and inherits its parameters; it adds its own, and a *modification* tightens an inherited one (here: M2 redeclares M1's optional `coulomb_friction` with no default, so it becomes required). |
| **PyTorch `nn.Module` subclassing** | You define a model by subclassing; parameters are attributes; `forward` ↔ `torque`; `reset_parameters` ↔ `seeds()` (where a fit starts); `named_parameters` ↔ `needs()` / `terms()`. |
| **Dataclass inheritance + PEP 681 `dataclass_transform`** (pydantic, SQLModel, attrs) | The base applies `@dataclass(frozen=True, kw_only=True)` in `__init_subclass__`, so a rung class needs no decorator and pyright sees every hand-declared field as a typed, keyword-only constructor argument. |
| **Python's C3 MRO** | The fallback chain is the method resolution order read as rung keys. |
| Schwarz 1978 (BIC), Levenberg–Marquardt | `compare()` picks the lowest BIC; `fit()` is numpy-only LM (no scipy in the runtime). |

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
   `stribeck_velocity` starts spread over the logged speeds. `compare()` is a classmethod on the type root.

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
errors caught before run time.

**Loses when**
- **A rung combines two siblings.** Stribeck + load friction needs `class StribeckLoad(Stribeck, Load)`. C3
  linearizes that into one fallback (stribeck_load → stribeck → load → M2 ...), which may not be the fallback you mean.
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
  Nothing else changes:
  - the v1 ruler `tests/test_ladders_acceptance.py`;
  - the rung bytes in `actuators/rungs/*`, and so robotparts' `tools/sync_actuator_rungs.py` vendoring and its
    `tests/core/test_actuator_rungs_vendored.py`;
  - sim2real's `tests/test_robot_models_parity.py`, which reads `ladders()` live through `holds()`, `rung()` and
    `fallback()`.
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
  parameter uncertainty.
