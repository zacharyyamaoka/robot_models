# M0.ideal — The ideal actuator

**no losses.** Family: ideal. Rung: 0, the ideal part. Category: actuator.

The ideal actuator is the zero every other model is measured from: the part's inertia and ratio, and no losses at all. Nobody should simulate a real robot with it. Its job is the ablation baseline: a rung's gain is how much better it tracks the real part than M0 does on the same runs.

**When to use it.** Use it as the first row of every ladder, and to see how much of a twin's error is friction at all.

## Equation

```
tau_r = 0; armature = output_inertia
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `output_inertia` | yes | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | chirps (or the datasheet) |
| `gear_ratio` | no | 1 | :1, 1 | reading the datasheet |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- none

## Inputs and state

- input `v` (rad/s): The output's angular speed.

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `ba6b33a4cbd2…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `533668d33e34…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | native | mujoco, release named by the fit | MuJoCo expresses it as the joint's armature; nothing runs per step. Maps: armature = `output_inertia`. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A stated model is fitted from nothing, so it has no fitted range.
- No losses of any kind: the model is a baseline, never a prediction of a real part.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("ideal", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0}, None, 0.001)
```

Defined since robotparts/2.5. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
