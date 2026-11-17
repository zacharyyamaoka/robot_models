# M3.load — Load-dependent friction

**Coulomb + viscous + load.** Family: static friction. Rung: 3, friction that changes with state. Category: actuator.

Coulomb friction that grows with the load the gearbox carries (gear-mesh friction), plus viscous drag. The plugin raises the joint's friction level each step from the load torque it reads.

**When to use it.** Use it for geared joints that carry gravity or payload; measure it with pendulum sweeps at 2-3 masses.

## Equation

```
tau_r = (coulomb_friction + friction_load_coefficient*|tau_load|)*sign(v) + viscous_friction*v, tau_load the output load torque
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `coulomb_friction` | yes | N·m | N·m, mN·m | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `viscous_friction` | yes | N·m·s/rad | N·m·s/rad, mN·m·s/rad | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `friction_load_coefficient` | yes | 1 | 1 | pendulum sweeps with 2-3 masses |
| `output_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | chirps (or the datasheet) |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- none

## Inputs and state

- input `v` (rad/s): The output's angular speed.
- input `tau_load` (N·m): The load torque the part transmits at the output (what it carries besides its own friction).

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `03b5cb124ad4…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `d50eb2222039…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | plugin | mujoco, release named by the fit | `robotparts.mujoco`. The unloaded level sticks natively; the plugin raises the joint's frictionloss each step by friction_load_coefficient * |tau_load|. Maps: armature = `output_inertia`, frictionloss = `coulomb_friction`, damping = `viscous_friction`. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A fit states the range of `output_speed` (rad/s) it holds across: |v|, the output speed the terms were fitted or checked across.
- A fit states the range of `load_torque` (N·m) it holds across: |tau_load|, the load the friction-load coefficient was fitted across.
- Friction grows linearly with the load the gear mesh carries, the same in both directions.
- Valid across the loads it was fitted on.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("load", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0, "tau_load": 0.0}, None, 0.001)
```

Defined since robotparts/2.3. It is robotparts/2.3's `load_dependent` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
