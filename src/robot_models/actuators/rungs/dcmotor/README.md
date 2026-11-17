# M5.dcmotor — MuJoCo's DC motor

**DC motor electrics + heat + cogging + LuGre.** Family: motor electrics. Rung: 5, the motor's electrics and heat. Category: actuator.

MuJoCo's own DC motor actuator (3.16): the winding's electrical dynamics, the motor constant, heat, cogging and LuGre friction, with every parameter named and shaped as MuJoCo takes it. Only MuJoCo implements it.

**When to use it.** Use it when current, voltage and heat limits matter: fast moves, long duty, a drive near its ceiling.

## Equation

```
MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR (back-EMF, cogging, LuGre friction)
```

Output `tau_r` (N·m): The resisting torque at the output: what the motor must add to move the output as the inputs say. The joint feels -tau_r (a resisting torque opposes motion; ripple pushes and pulls with angle).

## Terms

| term | required | unit (SI) | units a fit may use | measured by |
|---|---|---|---|---|
| `motorconst` | yes | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `resistance` | yes | Ω | Ω, mΩ | measure-only board calibration, then the envelope ladder |
| `nominal` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `saturation` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `inductance` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `cogging` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `controller` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `thermal` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |
| `lugre` | no | engine-defined (the engine's own SI unit for each component) | `mujoco` (engine-defined) | measure-only board calibration, then the envelope ladder |

Declared terms (the rest are registry keys, defined in SCHEMA2.md §7.3):

- `motorconst`: MuJoCo dcmotor's motorconst (2 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `resistance`: MuJoCo dcmotor's resistance, the winding resistance the electrical model uses, as mjs_setToDCMotor takes it.
- `nominal`: MuJoCo dcmotor's nominal (3 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `saturation`: MuJoCo dcmotor's saturation (3 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `inductance`: MuJoCo dcmotor's inductance (2 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `cogging`: MuJoCo dcmotor's cogging (3 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `controller`: MuJoCo dcmotor's controller (6 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `thermal`: MuJoCo dcmotor's thermal (6 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.
- `lugre`: MuJoCo dcmotor's lugre (5 numbers), as mjs_setToDCMotor takes it; unit mujoco means MuJoCo's own (SI) units for each entry.

## Inputs and state

- no per-step inputs

## Backends

| backend | how | engine | what runs |
|---|---|---|---|
| mujoco | native | mujoco ≥ 3.16.0, release named by the fit | MuJoCo 3.16's dcmotor actuator (mjs_setToDCMotor) takes the terms as they are; no other backend implements it. Maps: motorconst = `motorconst`, resistance = `resistance`, nominal = `nominal`, saturation = `saturation`, inductance = `inductance`, cogging = `cogging`, controller = `controller`, thermal = `thermal`, lugre = `lugre`. |

## Validity

- A fit states the range of `output_speed` (rad/s) it holds across: |v|, the output speed the terms were fitted or checked across.
- A fit states the range of `current` (A) it holds across: |i|, the winding current the electrics were fitted across.
- MuJoCo's own DC motor equations, as its release documents them.

## Use it

Only MuJoCo implements this model: a fit's terms go into the dcmotor actuator as the backend's maps say.

Defined since robotparts/2.3. It is robotparts/2.3's `mujoco_dcmotor` kind. The contract is SCHEMA2.md §4.3.2; the fit format is `schemas/model.schema.json`.
