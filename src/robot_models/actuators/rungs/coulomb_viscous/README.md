# M2.coulomb_viscous — Coulomb and viscous friction

**Coulomb + viscous.** Family: fitted constant friction. Rung: 2, fitted constant friction. Category: actuator.

Constant (Coulomb) friction that flips with direction, plus a viscous drag that grows with speed. MuJoCo expresses both natively (frictionloss and damping), so it runs anywhere with no plugin. On the rack it is the first fitted rung, and the backend matters more than the terms: MuJoCo's default friction constraint creeps below about 0.4 rad/s on a light hinge, and a hard solreffriction fixes it.

**When to use it.** Use it as the first fitted model; measure it with velocity plateaus in both directions.

## Equation

```
tau_r = coulomb_friction*sign(v) + viscous_friction*v
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `coulomb_friction` | yes | N·m | N·m, mN·m | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `viscous_friction` | yes | N·m·s/rad | N·m·s/rad, mN·m·s/rad | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `output_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | chirps (or the datasheet) |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- none

## Inputs and state

- input `v` (rad/s): The output's angular speed.

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `e20d6f415822…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `5b8131ce0acd…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | native | mujoco, release named by the fit | frictionloss sticks at rest as a constraint (set friction_solref hard, or it creeps); damping is the viscous term. Maps: armature = `output_inertia`, frictionloss = `coulomb_friction`, damping = `viscous_friction`. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A fit states the range of `output_speed` (rad/s) it holds across: |v|, the output speed the terms were fitted or checked across.
- Friction does not depend on speed beyond the viscous slope, on load, on temperature or on how long the part rested.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("coulomb_viscous", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0}, None, 0.001)
```

Defined since robotparts/2.3. It is robotparts/2.3's `coulomb_viscous` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
