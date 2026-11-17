# v2 — Records in a Family

**Thesis.** A `Model` is a frozen record (key, level, name, base, parameters with units, equation, references) that
also carries its physics; `model.bind(values)` gives a callable `BoundModel`. A `Family` (one per type) holds the
vocabulary, the aliases and the model tree. It is the minimal-migration option: `Ladder` becomes `Family` (literally a
subclass of it), `Rung` becomes `Model`, and the rung's code folder is attached as an object instead of named by
string.

Run it: `/home/bam/robot_models/.venv/bin/python scenario.py` (transcript: [scenario.out.txt](scenario.out.txt));
check: `python check_scenario.py v2_records` → `PASS  v2_records`.

## The object model

```
families() -> {type: Family}                         today's ladders(), each read as a Family
Family(Ladder)                                        a v1 Ladder: vocabulary, aliases, packed, rungs, every v1 call
  ├─ family["M3.stribeck"] -> Model                   the record you hold
  ├─ .tree()  .children(key)  .siblings(key)          the tree; fallback/canonical/holds inherited from Ladder
  ├─ .values(datasheet | {name: number|Stated})       -> Values: canonical names -> SI floats (PyBaMM's ParameterValues)
  └─ .compare(*bound, observed=, **inputs)            -> Comparison: rms + BIC per model, .chosen
Model (frozen)  key level name title base kind adds reads parameters optional equation references physics
  ├─ .bind(values) -> BoundModel                      checks what it needs, keeps what it holds
  ├─ .fit(observed, values=, grid=, **inputs)         -> BoundModel: varies model.json's required terms
  └─ .physics: Physics | None                         model.py's torque() + model.json's inputs/required/mujoco maps
BoundModel  model values(dict, SI) fitted rms samples   __call__(**inputs) numpy in/out; .mujoco()
```

## Prior art borrowed

- **lmfit `Model`** ([docs](https://lmfit.github.io/lmfit-py/model.html)): wrap an existing function and take its
  parameters by name; `model.fit(data, x=...)` puts the data first and the independent variables as keywords; the fit
  returns a result that still evaluates. Here the wrapped function is today's `model.py` `torque()`, the parameter names
  come from its `model.json` (translated to the type's canonical names: LuGre's `sigma2` → `viscous_friction`), and
  `stribeck.fit(log["tau"], v=log["v"], values=values)` is lmfit's call shape.
- **PyBaMM** (model vs `ParameterValues`): a model is a symbolic thing with no numbers; values live in their own
  object and are bound later. Here `servo.values(datasheet)` is the `ParameterValues` (units converted once, by the
  datasheet's own `Stated.si`), and `model.bind(values)` is `process_model`. The same `values` feed M1, M2 and M3.
- **Variable projection** (Golub & Pereyra 1973) for the fit, and **BIC** (Schwarz 1978) for "the rung that pays".

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
4. **`fit` lives on the Model and is generic; `compare` lives on the Family.** The fit varies the terms `model.json`
   marks required, probes the physics to split them into linear terms (solved exactly by `lstsq`) and nonlinear ones
   (log-spaced grid over the logged speed range when the term has an input's unit, refined 4×; else `grid=`), and
   verifies the linear terms superpose at the answer. So a new rung's `model.py` brings no fitting code. Comparing is
   about the tree (is the higher rung worth its terms?), so it is the Family's.

## Best when / loses when

- **Best when** the priority is landing without breaking anyone: three repos read `ladders()` today, and this is an
  additive module whose Family *is* a Ladder. Also when rungs keep arriving as robotparts-style folders
  (`model.py` + `model.json`): the record picks them up and fitting comes free.
- **Loses when** you want the Model to stand alone: it can describe and compute itself, but fallback, siblings and
  comparison need its Family, and the parameters are names in a vocabulary, not typed attributes a type checker sees
  (`values` is a dict). A rung whose physics is stateful (LuGre with `dt`) or not separable needs more than the probe:
  LuGre runs at steady state only here, and a jointly nonlinear term needs an explicit `grid=`.

## Migration from v1

- **Landing it:** add `src/robot_models/family.py` (this module less `RUNNABLE_UP_TO`). Nothing else changes. Later,
  optionally, `ladders()` returns `Family.from_ladder(...)` and type folders may write `LADDER = Family(...)`.
- **The v1 ruler (`tests/test_ladders_acceptance.py`):** untouched. Rung records, keys, bases, adds, `code` strings and
  every rung byte stay as they are; `isinstance(LADDER, Ladder)` still holds for a Family; the value scan judges type
  folders, not a root module. The fresh-interpreter import test picks up one more module (numpy is already a runtime
  dependency).
- **robotparts' vendoring of `actuators/rungs/*`:** untouched. The folders are read in place and never imported, so no
  `__pycache__/` lands in them and the byte-identical and `RUNG_FILES` sha256 checks stay green.
- **sim2real `tests/test_robot_models_parity.py`:** untouched. It calls `ladders()`, `.keys`, `.rung()`, `.resolve()`,
  `.holds()`, `.composed()`: all inherited. sim2real may later fit through `family[key].fit(...)`, additively.

## Cost of a new rung (an M3 friction rung)

Exactly what v1 asks today, nothing more: one `Rung(level=3, name=..., base=..., adds=(...), code="actuators/rungs/<word>",
equation=..., references=(...))` entry in `actuators/servo/ladder.py` (plus a `Parameter` for any new quantity), and
the folder `actuators/rungs/<word>/` with `model.py` (`torque(terms, inputs, state, dt) -> (tau_r, state)`) and
`model.json` (its terms marked required or not, inputs, output, MuJoCo maps); robotparts' parity also wants `model.js`
and the ruler's sha256 table. The Family finds the folder through `code`; binding, the card, fitting, comparison and the
MuJoCo export come free. No registration and no fitting code.

## What is simulated or left out

- The Family is derived at runtime from today's `ladders()` instead of being declared in `src/`.
- Physics is attached for M0–M3 (ideal, datasheet, coulomb_viscous, stribeck, load, lugre); M4.ripple has a `model.py`
  but is not attached (stop line); M3.weakening_capture and M4+ have no runnable code.
- `Physics.__call__` vectorizes by looping the scalar stdlib `torque()` over samples (no numpy rewrite of the physics);
  state and `dt` are not threaded through, so LuGre runs at steady state.
- `Values` does not resolve aliases on read (it canonicalizes on build) and refuses a bare number for an alias with a
  non-SI unit (`noload_speed_rpm`), asking for a `Stated`.
- Fit uncertainties, held-out validation and composed types (`foc_controller`) are not modelled.
- Core size: 282 non-blank lines in `family_records.py` (over the 250 aim; about 50 are docstrings and WHY comments).
