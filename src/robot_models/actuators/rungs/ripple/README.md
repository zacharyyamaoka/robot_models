# M4.ripple — Rotor ripple map

**Coulomb + viscous + rotor-keyed ripple.** Family: grey box. Rung: 4, the drivetrain (ripple, backlash, compliance). Category: actuator.

Cogging and commutation ripple: a torque that repeats with the rotor's angle, written as harmonics, plus optional Coulomb and viscous friction. Because the magnets and windings turn with the rotor, a map in rotor angle holds wherever the joint goes.

**When to use it.** Use it when position holds and slow moves show a periodic error; one rotor-angle sweep fits it.

## Equation

```
tau_r = sum_k amplitude_k*sin(rotor_harmonic_k*theta_rotor + phase_k), theta_rotor = gear_ratio*theta_output in rad from the rotor zero the simulator uses (tau_r adds coulomb_friction*sign(v) + viscous_friction*v when stated)
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `harmonics` | yes | N·m | N·m, mN·m | a rotor-angle sweep (the plateaus, if they cover more than one rotor turn) |
| `coulomb_friction` | no | N·m | N·m, mN·m | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `viscous_friction` | no | N·m·s/rad | N·m·s/rad, mN·m·s/rad | velocity plateaus, both directions, 6-8 speeds, each at least one rotor turn |
| `output_inertia` | no | kg·m² | kg·m², kg·cm², kg·mm², g·cm², g·mm² | chirps (or the datasheet) |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- `harmonics`: The ripple harmonics: a list of {rotor_harmonic, amplitude, phase}, rotor_harmonic in cycles per rotor turn (an integer >= 1, each once), amplitude in the term's unit (>= 0, the torque at the output), phase in rad.

## Inputs and state

- input `v` (rad/s): The output's angular speed.
- input `theta_rotor` (rad): The rotor's angle, gear_ratio times the output angle from the rotor zero the simulator uses.

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| python | reference | — | `model.py` (sha256 `e30eb5e3ed7c…`). The contract's own ruler: stdlib Python, torque(terms, inputs, state, dt) -> (tau_r, state), terms in SI. |
| js | reference | — | `model.js` (sha256 `31f08577ba8c…`). The same equation as model.py as an ES module (torque(terms, inputs, state, dt) -> {tau, state}); tests/acceptance_v2/test_amendment_2_5.py holds the two to 1e-12. |
| mujoco | plugin | mujoco, release named by the fit | `robotparts.mujoco`. Friction natively; the plugin applies the ripple from the rotor angle each step. Maps: armature = `output_inertia`, frictionloss = `coulomb_friction`, damping = `viscous_friction`. |

**mujoco settings** a fit may state:

- `friction_solref`: 2 numbers. The joint's solreffriction (timeconst, dampratio): how hard frictionloss sticks; MuJoCo's soft default creeps below about 0.4 rad/s on a light hinge.
- `friction_solimp`: 5 numbers, in (0, inf). The joint's solimpfriction (dmin, dmax, width, midpoint, power), every entry > 0.

## Validity

- A fit states the range of `output_speed` (rad/s) it holds across: |v|, the output speed the terms were fitted or checked across.
- The ripple is periodic in the rotor angle and the same in both directions (cogging); direction-dependent ripple is not modelled.
- The harmonics hold wherever the joint goes, because the magnets turn with the rotor.

## Use it

```python
import importlib.util
spec = importlib.util.spec_from_file_location("ripple", "model.py")
model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
terms = {...}  # a fit's terms in SI (robotparts.models.si_terms(fit, definition) converts them)
tau_r, state = model.torque(terms, {"v": 0.0, "theta_rotor": 0.0}, None, 0.001)
```

Defined since robotparts/2.3. It is robotparts/2.3's `ripple_map` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
