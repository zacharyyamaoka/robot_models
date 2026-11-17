# v6 · Fitted instances

**Thesis.** A `ModelFamily` class holds its `Model` classes; a Model's `Parameter`s are class attributes (unit,
definition, info), a rung subclasses its base so vocabulary only grows (enforced at class creation), and values live
only on Model **instances**: `Stribeck(coulomb_friction=...)`, or what `Stribeck.fit(log)` returns. The classes are the
one source: today's `Ladder` record is derived from them and equals `ladders()["integrated_servo"]` exactly.

`check_scenario.py v6_fitted_instances` → **PASS**: case A chooses M3.stribeck (held-out rms ÷ floor: M2 17.18, M3
1.00), case B chooses M2.coulomb_viscous (M2 1.03, M3 1.57). The whole transcript runs in under 1 s.

```bash
/home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v6_fitted_instances/scenario.py
```

Files: [fitted_instances/model.py](fitted_instances/model.py) (the generic core: Parameter, Model, BlackBox,
ModelFamily, the batch results, the tree; 546 non-blank lines, over the 350 aim — see below),
[fitted_instances/servo.py](fitted_instances/servo.py) (integrated_servo as one family of 14 classes, 149 lines; the
type data that today's `actuators/servo/ladder.py` would be derived from), [scenario.py](scenario.py),
[scenario.out.txt](scenario.out.txt), [integrated_servo_tree.html](integrated_servo_tree.html) (what `.show()` wrote).

## The object model

```
ModelFamily  (metaclass: iterable, len)        class IntegratedServo(ModelFamily, type="integrated_servo")
├── .models            Models: ["M3.stribeck"], ["M3"], .m3.stribeck   (a level is a Group of classes)
├── .ladder(top=None)  every model in order; ladder(top) = top's fallback chain
├── .tree()            Tree: .print() text, .show(path) HTML + mermaid, .mermaid()
├── .fit(log, start=)  FamilyFit {key: instance} + .skipped {key: why}; prints as a table
├── .select_best(held_out, fits)  Selection: .chosen = lowest rung ≤ 1.5× noise floor (also fits.select_best(...))
└── .v1_ladder()       today's v1 Ladder record, DERIVED from the classes; .canonical(alias)
Model  (class = the type)                      class Stribeck(CoulombViscous, level=3, name="stribeck")
├── class: .key .param_count .parameters .own .fallback() .needs() .describe() .rung() .fit(log, start=) .from_datasheet()
└── instance: .coulomb_friction (a float) .values() .params .predict(v) .score(log) .mujoco()
Parameter(unit, definition, shape, note, required, value)   Parameter() = "the inherited one, now required"
BlackBox(Model)  weights, not Parameters; same fit / predict / param_count / score → IntegratedServo.Learned (M7)
```

## Decisions

1. **Classes nested in the family body.** `IntegratedServo.Stribeck` is the class; the family collects every Model
   in its namespace at creation, in declaration order by level, which IS today's ladder order. A rung declared later
   (`class Dahl(IntegratedServo.models.m2.coulomb_viscous, level=3, name="dahl")`) joins the family from its base, so
   nothing drops silently.
2. **Additive vocabulary, enforced.** At class creation a subclass that redefines an inherited parameter's unit or
   definition, shadows one with a non-Parameter (`breakaway_torque = None`), loosens a required one, uses an API name
   (`fit`, `key`...), names a parameter after a type alias (`frictionloss_nm`) or re-adds an inherited definition under
   a new name raises. The only change allowed to an inherited parameter is `x = Parameter()`: optional below,
   required from here (M1 states coulomb_friction, M2 needs it). Across branches the family refuses two definitions of
   one name (G4); the three names that first appear on two branches (harmonics, thermal_*) are module constants.
3. **One source, verified.** `Model.rung()` builds the v1 `Rung` from the class (adds = own required, optional = own
   optional, base = MRO, title = docstring, equation/references/note/wears/composes = class attributes, code = the
   rung folder if it exists); `v1_ladder()` adds the vocabulary (datasheet order first) and today's aliases, packed and
   deployment names. `IntegratedServo.v1_ladder() == ladders()["integrated_servo"]` is **True on the whole record**,
   not only keys, bases, adds and optional (scenario step 2).
4. **Values on instances.** The `Parameter` is a descriptor: `Stribeck.breakaway_torque` is the Parameter,
   `m3.breakaway_torque` the float, `m3.params["breakaway_torque"]` the Parameter with its value. Instances are frozen.
   A fitted instance leaves what its log cannot see unset rather than invented (`Stribeck.fit(log)` with no start has
   no inertia).
5. **Generic fit, no per-rung seeds.** `fit` runs the reference once with a recording terms dict to learn what the
   physics reads, frees the needed parameters that are read AND move the log (checked at a jittered point, so equal
   seeds can't hide stribeck_velocity), and seeds each from its **unit** (rad/s: a log-spaced sweep of the logged
   speeds; N·m: the median |tau|; N·m·s/rad: the tau-on-v slope) plus the climbed start. Levenberg–Marquardt in log
   space, so a fit keeps each parameter's sign (friction is never negative). What the log can't identify is a skip
   with a reason: LuGre's sigma0/sigma1 cancel at steady state; M3.load and M4.ripple need inputs the log lacks.
6. **Vectorized, one copy of the physics.** Today's `model.py` is loaded twice: as written, and with `math` and `_sign`
   swapped for numpy. Every call checks three samples of the array run against the stdlib run; a reference numpy can't
   run (LuGre's `(v > 0) - (v < 0)`) falls back to a loop, once. 100 000 samples of M3 predict in ~3 ms.
7. **Selection = sim2real's rule, in ladder order.** `select_best` ranks candidates by ladder position (siblings by
   declaration order, as sim2real's ladder order breaks ties), scores each on the held-out plateaus, and chooses the
   first within 1.5× the pooled within-plateau noise floor, else `None`. Unlike V1 it does not demand one fallback
   chain, so the black box (M7) competes with the same call.
8. **The black box (M7.learned)** is a per-direction cubic in v (8 weights, closed-form `lstsq`), a stand-in for
   today's MLP; it subclasses `BlackBox`, not the physics root, so it extends nothing (V1 had to force it under the
   type root). On servo A it fits its training plateaus to 10.1 mN·m and misses the held-out ones by 764 mN·m: the
   3 rad/s plateau lies outside the fitted 0.03–2 rad/s, and a polynomial extrapolates badly. The rule catches it.

## What V1's judges flagged, and what changed

| V1 flaw | here |
|---|---|
| every rung written twice (class + Rung record) | the class is the only copy; the record is derived and equals today's |
| a Rung without a class silently drops out | nothing to drop: records come from classes; a later subclass joins its family |
| a new nonlinear rung needs a `seeds()` override | seeds come from the parameter's unit; no rung overrides anything |
| `torque()` loops over scalar `model.py` | the same `model.py`, run on arrays with numpy swapped in, cross-checked per call |
| parameters share the namespace with the API, unchecked | a reserved-name list raises |

## Best when / loses when

**Best when** a type's models really are refinements of each other, the people writing rungs write Python, and you
want one place to read a model: its parameters, units, equation, references and physics are all on one class, and its
values are an ordinary object you can pass around, compare and hand to MuJoCo (`m3.mujoco()`) or sim2real
(`m3.values()`, canonical names in SI).

**Loses when**
- **You want to carry a unit around.** A servo is several fitted instances (one per rung) plus a `FamilyFit`; there
  is no single "this servo" object (that is v7's thesis).
- **You want values as storable data.** An instance is a Python object; serializing is `values()` plus the key
  (v8 makes the value set the first-class thing).
- **Type checkers.** `m3.breakaway_torque` is typed `float` through the descriptor's overloads, but constructor
  keywords are not checked statically (no `dataclass_transform`), and M1's 33 injected datasheet fields are invisible
  to a checker. **Not verified with pyright** (none installed here).
- **Nested classes are a long file.** `servo.py` is 149 lines in one class body; the payoff is that it is the whole
  type.
- **The core is big:** 546 non-blank lines against a 350 aim. Roughly: Parameter + additivity 100, Model 190,
  BlackBox 35, family 80, results and tree 110, physics runner and LM 90. Cutting `describe`, `show` and `mujoco`
  would save ~45; the rest is the feature set.
- **Equal-rung siblings.** `select_best` breaks ties by declaration order; "Stribeck or Load at M3?" is not asked.

## Migration from v1

- **Now (additive):** nothing in `src/`, the ruler, robotparts' vendored rung bytes or sim2real changes; the classes
  read today's `model.py`/`model.json` by path and are checked equal to `ladders()`.
- **Then:** `actuators/servo/ladder.py` shrinks to `LADDER = IntegratedServo.v1_ladder()` (the family moves into the
  type folder; ALIASES, PACKED, DEPLOYMENT move into its body verbatim). `ladders()`, `discover()` and every consumer
  see the same record — the equality already holds, so the v1 ruler `tests/test_ladders_acceptance.py`, sim2real's
  `tests/test_robot_models_parity.py` and robotparts' vendoring (`model.py`/`model.json` bytes don't move) keep
  passing unchanged. Worth adding to sim2real's parity test: `SELECT_LIMIT == TWINNED_LIMIT`.
- The other 11 types stay records until each gets its family; `ladders()` mixes both freely.

## Cost of a new M3 friction rung

1. One nested class in the family, e.g.
   `class Dahl(CoulombViscous, level=3, name="dahl"):` with a docstring title, `dahl_stiffness = Parameter("N·m/rad",
   "...")`, `equation = ...`, `references = (...)`.
2. To make it runnable, `actuators/rungs/dahl/model.py` and `model.json` (unchanged contract).

Nothing else: no record, no `seeds()`, no registry; fit, the batch fit, `select_best`, the tree and the derived v1
record pick it up. (The ruler's frozen key list would then fail on purpose: a new rung is a ruler edit.)

## What is simulated or left out

- The rig log is the fixture's simulated plateau sweep; M1 is the fixture's illustrative datasheet.
- Runnable: M0–M3 with reference physics (M3.lugre only at steady state, and skipped by the fit: sigma0/sigma1 can't be
  identified from plateaus); M3.load and M4.ripple run when given `tau_load` / `theta_rotor`. M3.weakening_capture,
  M4.drivetrain, M5.*, M6 are declared only. M7 is a numpy polynomial stand-in for the MLP.
- The fit frees needed parameters only (stribeck_exponent stays at 2) and reports no uncertainty. On servo A,
  M2's viscous_friction lands on the positivity bound (≈ 0): it was −0.065 in V1, compensating the Stribeck drop.
- `tree().show()` loads mermaid 10.9.1 from jsdelivr; the text tree is on the page as the offline fallback. Rendered
  headlessly and looked at.
