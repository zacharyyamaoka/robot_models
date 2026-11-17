# M3.lugre — LuGre friction

**Coulomb + viscous + Stribeck + bristle state.** Family: dynamic friction. Rung: 3, friction that changes with state. Category: actuator.

The LuGre dynamic friction model: the contact behaves like bristles that deflect before they slide, which gives pre-sliding motion, the Stribeck curve and frictional lag from one state variable. It is stateful, so every backend integrates z each step.

**When to use it.** Use it when small reversals and holding position matter more than the plateaus explain.

## Equation

```
dz/dt = v - sigma0*|v|*z/g(v), g(v) = coulomb_friction + (breakaway_torque - coulomb_friction)*exp(-(|v|/stribeck_velocity)^stribeck_exponent); tau_r = sigma0*z + sigma1*dz/dt + sigma2*v; stribeck_exponent is 2 when absent
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `coulomb_friction` | yes | N·m | N·m, mN·m | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `breakaway_torque` | yes | N·m | N·m, mN·m | slow plateaus below 0.5 rad/s |
| `stribeck_velocity` | yes | rad/s | rad/s, rpm, deg/s | slow plateaus below 0.5 rad/s |
| `sigma0` | yes | N·m/rad | N·m/rad | small reversals, recorded fast |
| `sigma1` | yes | N·m·s/rad | N·m·s/rad, mN·m·s/rad | small reversals, recorded fast |
| `sigma2` | yes | N·m·s/rad | N·m·s/rad, mN·m·s/rad | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `stribeck_exponent` | no | 1 | 1 | slow plateaus below 0.5 rad/s |
| `output_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | chirps (or the datasheet) |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- `sigma0`: LuGre bristle stiffness sigma0: output torque per radian of bristle deflection z.
- `sigma1`: LuGre bristle (micro-)damping sigma1: output torque per rad/s of bristle deflection rate.
- `sigma2`: LuGre viscous coefficient sigma2: output torque per rad/s of output speed (the other kinds' viscous_friction).
- `stribeck_exponent`: Shape exponent d of the Stribeck curve in g(v); 2 when absent.

## Inputs and state

- input `v` (rad/s): The output's angular speed.
- input `dt` (s): The step the simulator advances by (a stateful model integrates its state over it).
- state `z` (rad, starts at 0): The bristles' mean deflection.

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `63cb93d20325…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `e8e42ee33d8a…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | plugin | mujoco, release named by the fit | `robotparts.mujoco`. No native part: the plugin integrates z each step with the simulator's dt (semi-implicit, stable for any dt) and applies the whole torque. Maps: armature = `output_inertia`. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A fit states the range of `output_speed` (rad/s) it holds across: |v|, the output speed the terms were fitted or checked across.
- One bristle state for the whole drivetrain.
- Pre-sliding is fitted from small reversals; larger stiction effects outside their amplitude are extrapolated.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("lugre", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0}, None, 0.001)
```

Defined since robotparts/2.3. It is robotparts/2.3's `lugre` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
