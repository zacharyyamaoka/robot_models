# M1.datasheet — The datasheet

**the stated values.** Family: datasheet. Rung: 1, the datasheet. Category: actuator.

The datasheet model holds what the maker states and nothing fitted: inertia, ratio, torque constant, and friction only if the datasheet gives it. The build checks every term against the part's own specs, so a model called datasheet can never carry a fitted number.

**When to use it.** Use it before anything is measured, and as the honest answer to "what does the maker claim?".

## Equation

```
tau_r = coulomb_friction*sign(v) + viscous_friction*v; armature = output_inertia, or gear_ratio^2*(rotor_inertia + gearbox_input_inertia)
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `output_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | reading the datasheet |
| `rotor_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | reading the datasheet |
| `gearbox_input_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | reading the datasheet |
| `gear_ratio` | no | 1 | :1, 1 | reading the datasheet |
| `kt` | no | N·m/A | N·m/A, mN·m/A | reading the datasheet |
| `coulomb_friction` | no | N·m | N·m, mN·m | reading the datasheet |
| `viscous_friction` | no | N·m·s/rad | N·m·s/rad, mN·m·s/rad | reading the datasheet |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- none

## Inputs and state

- input `v` (rad/s): The output's angular speed.

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `3ed5e7d8d90d…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `bb054894e901…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | native | mujoco, release named by the fit | The joint's armature, frictionloss and damping; a model stating rotor and gearbox inertia instead gives armature = gear_ratio^2 (rotor_inertia + gearbox_input_inertia). Maps: armature = `output_inertia`, frictionloss = `coulomb_friction`, damping = `viscous_friction`. Without `output_inertia`, armature = `gear_ratio`² × (`rotor_inertia` + `gearbox_input_inertia`), every one of `gear_ratio`, `rotor_inertia`, `gearbox_input_inertia` stated; `gearbox_input_inertia` counts as 0 only on drivetrain direct_drive or motor. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A stated model is fitted from nothing, so it has no fitted range.
- Every value is the maker's, as stated: the model is only as good as the datasheet.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("datasheet", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0}, None, 0.001)
```

Defined since robotparts/2.3. It is robotparts/2.3's `datasheet` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
