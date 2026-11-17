# M3.stribeck — Stribeck friction

**Coulomb + viscous + Stribeck.** Family: static friction. Rung: 3, friction that changes with state. Category: actuator.

Friction that is highest just as the output breaks away and falls to the Coulomb level as it speeds up (the Stribeck curve), plus viscous drag. In MuJoCo the breakaway level sticks natively and the plugin gives back the fall with speed each step.

**When to use it.** Use it when slow motion and starts from rest matter: the plateaus below 0.5 rad/s show the curve.

## Equation

```
tau_r = (coulomb_friction + (breakaway_torque - coulomb_friction)*exp(-(|v|/stribeck_velocity)^stribeck_exponent))*sign(v) + viscous_friction*v; stribeck_exponent is 2 when absent
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `coulomb_friction` | yes | N·m | N·m, mN·m | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `viscous_friction` | yes | N·m·s/rad | N·m·s/rad, mN·m·s/rad | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `breakaway_torque` | yes | N·m | N·m, mN·m | slow plateaus below 0.5 rad/s |
| `stribeck_velocity` | yes | rad/s | rad/s, rpm, deg/s | slow plateaus below 0.5 rad/s |
| `stribeck_exponent` | no | 1 | 1 | slow plateaus below 0.5 rad/s |
| `output_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | chirps (or the datasheet) |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- `stribeck_exponent`: Shape exponent d of the Stribeck curve, how sharply friction falls with speed; 2 when absent.

## Inputs and state

- input `v` (rad/s): The output's angular speed.

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `87f9bc8becd9…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `15bdb5ab166d…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | plugin | mujoco, release named by the fit | `robotparts.mujoco`. The static level sticks natively (frictionloss = breakaway_torque); the plugin adds back what friction falls by as the output speeds up, so sliding friction is the Stribeck curve. Maps: armature = `output_inertia`, frictionloss = `breakaway_torque`, damping = `viscous_friction`. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A fit states the range of `output_speed` (rad/s) it holds across: |v|, the output speed the terms were fitted or checked across.
- Friction depends on speed only: not on load, temperature or rest time.
- The curve holds across the speeds it was fitted on; below the slowest plateau the breakaway level is extrapolated.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("stribeck", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0}, None, 0.001)
```

Defined since robotparts/2.3. It is robotparts/2.3's `stribeck` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
