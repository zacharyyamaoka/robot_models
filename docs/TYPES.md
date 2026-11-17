> **Generated file.** Written by [`tools/make_type_docs.py`](../tools/make_type_docs.py) from each type folder's `ladder.py`: edit the ladders, not this file, then run `python tools/make_type_docs.py` from the repository root.

# The 12 types

Every part type robot_models models, 84 rungs in all. Each type is a ladder of models in a folder of its own under [`src/robot_models/`](../src/robot_models), and each folder's README.md shows its rung tree, its rungs with their equations and references, its vocabulary, aliases and the names that belong to a deployment instead.

```text
robot_msgs ← robot_models ← robot_parts ← robot_descriptions,   with sim2real on top
```

| type | what it is | folder | rungs | top physics rung(s) | residual, learned | datasheet class |
| --- | --- | --- | ---: | --- | --- | --- |
| [`integrated_servo`](../src/robot_models/actuators/servo/README.md) | A joint in one housing: a motor, its drive, a reducer or none, and an encoder. | [`actuators/servo/`](../src/robot_models/actuators/servo) | 14 | [`M5.dcmotor`](../src/robot_models/actuators/servo/README.md#rung-m5-dcmotor), [`M5.envelope`](../src/robot_models/actuators/servo/README.md#rung-m5-envelope), [`M5.wear`](../src/robot_models/actuators/servo/README.md#rung-m5-wear) | [`M6.residual`](../src/robot_models/actuators/servo/README.md#rung-m6-residual), [`M7.learned`](../src/robot_models/actuators/servo/README.md#rung-m7-learned) | [`IntegratedServo`](../src/robot_models/actuators/servo/integrated_servo.py) |
| [`bldc_motor`](../src/robot_models/actuators/servo/bldc/README.md) | A brushless motor alone, at its shaft. | [`actuators/servo/bldc/`](../src/robot_models/actuators/servo/bldc) | 9 | [`M5.dcmotor`](../src/robot_models/actuators/servo/bldc/README.md#rung-m5-dcmotor), [`M5.wear`](../src/robot_models/actuators/servo/bldc/README.md#rung-m5-wear) | [`M6.residual`](../src/robot_models/actuators/servo/bldc/README.md#rung-m6-residual), [`M7.learned`](../src/robot_models/actuators/servo/bldc/README.md#rung-m7-learned) | [`BldcMotor`](../src/robot_models/actuators/servo/bldc/bldc_motor.py) |
| [`foc_controller`](../src/robot_models/actuators/servo/foc_controller/README.md) | A field-oriented-control motor drive board. | [`actuators/servo/foc_controller/`](../src/robot_models/actuators/servo/foc_controller) | 6 | [`M2.ceiling_heat`](../src/robot_models/actuators/servo/foc_controller/README.md#rung-m2-ceiling_heat), [`M2.wear`](../src/robot_models/actuators/servo/foc_controller/README.md#rung-m2-wear) | [`M6.residual`](../src/robot_models/actuators/servo/foc_controller/README.md#rung-m6-residual), [`M7.learned`](../src/robot_models/actuators/servo/foc_controller/README.md#rung-m7-learned) | — |
| [`gearbox`](../src/robot_models/actuators/servo/gearbox/README.md) | A gear stage between a motor and its load. | [`actuators/servo/gearbox/`](../src/robot_models/actuators/servo/gearbox) | 8 | [`M4.drivetrain`](../src/robot_models/actuators/servo/gearbox/README.md#rung-m4-drivetrain), [`M4.wear`](../src/robot_models/actuators/servo/gearbox/README.md#rung-m4-wear) | [`M6.residual`](../src/robot_models/actuators/servo/gearbox/README.md#rung-m6-residual), [`M7.learned`](../src/robot_models/actuators/servo/gearbox/README.md#rung-m7-learned) | [`Gearbox`](../src/robot_models/actuators/servo/gearbox/gearbox.py) |
| [`rigid_body_chain`](../src/robot_models/arms/rigid_body_chain/README.md) | A chain of rigid links and joints, open or closed: a whole arm, a sub-assembly, a wrist, a five-bar. | [`arms/rigid_body_chain/`](../src/robot_models/arms/rigid_body_chain) | 10 | [`M3.compliance`](../src/robot_models/arms/rigid_body_chain/README.md#rung-m3-compliance), [`M3.wear`](../src/robot_models/arms/rigid_body_chain/README.md#rung-m3-wear) | [`M6.residual`](../src/robot_models/arms/rigid_body_chain/README.md#rung-m6-residual), [`M6.inertial_residual`](../src/robot_models/arms/rigid_body_chain/README.md#rung-m6-inertial_residual), [`M7.learned`](../src/robot_models/arms/rigid_body_chain/README.md#rung-m7-learned) | — |
| [`conveyor`](../src/robot_models/conveyors/conveyor/README.md) | A belt driven through a drum. | [`conveyors/conveyor/`](../src/robot_models/conveyors/conveyor) | 7 | [`M4.stretch`](../src/robot_models/conveyors/conveyor/README.md#rung-m4-stretch), [`M4.wear`](../src/robot_models/conveyors/conveyor/README.md#rung-m4-wear) | [`M6.residual`](../src/robot_models/conveyors/conveyor/README.md#rung-m6-residual), [`M7.learned`](../src/robot_models/conveyors/conveyor/README.md#rung-m7-learned) | — |
| [`gripper`](../src/robot_models/end_effectors/gripper/README.md) | An end effector that grasps, on a chain's flange. | [`end_effectors/gripper/`](../src/robot_models/end_effectors/gripper) | 6 | [`M2.contact`](../src/robot_models/end_effectors/gripper/README.md#rung-m2-contact), [`M2.wear`](../src/robot_models/end_effectors/gripper/README.md#rung-m2-wear) | [`M6.residual`](../src/robot_models/end_effectors/gripper/README.md#rung-m6-residual), [`M7.learned`](../src/robot_models/end_effectors/gripper/README.md#rung-m7-learned) | — |
| [`aruco_marker`](../src/robot_models/markers/aruco_marker/README.md) | A printed sheet carrying one or more ArUco markers rigidly. | [`markers/aruco_marker/`](../src/robot_models/markers/aruco_marker) | 4 | [`M3.curl`](../src/robot_models/markers/aruco_marker/README.md#rung-m3-curl) | — | — |
| [`camera`](../src/robot_models/sensors/camera/README.md) | A frame camera's colour image: projection, distortion, exposure. | [`sensors/camera/`](../src/robot_models/sensors/camera) | 6 | [`M3.rolling_shutter`](../src/robot_models/sensors/camera/README.md#rung-m3-rolling_shutter), [`M3.wear`](../src/robot_models/sensors/camera/README.md#rung-m3-wear) | [`M6.residual`](../src/robot_models/sensors/camera/README.md#rung-m6-residual) | — |
| [`encoder`](../src/robot_models/sensors/encoder/README.md) | An angle sensor on a shaft, such as a drive's rotor encoder. | [`sensors/encoder/`](../src/robot_models/sensors/encoder) | 4 | [`M2.eccentricity`](../src/robot_models/sensors/encoder/README.md#rung-m2-eccentricity), [`M2.wear`](../src/robot_models/sensors/encoder/README.md#rung-m2-wear) | — | — |
| [`hyperspectral_camera`](../src/robot_models/sensors/hyperspectral_camera/README.md) | A line-scan spectral camera over a belt. | [`sensors/hyperspectral_camera/`](../src/robot_models/sensors/hyperspectral_camera) | 5 | [`M3.pushbroom`](../src/robot_models/sensors/hyperspectral_camera/README.md#rung-m3-pushbroom), [`M3.wear`](../src/robot_models/sensors/hyperspectral_camera/README.md#rung-m3-wear) | — | — |
| [`depth_camera`](../src/robot_models/sensors/rgbd/README.md) | A stereo depth camera: depth on top of a colour stream. | [`sensors/rgbd/`](../src/robot_models/sensors/rgbd) | 5 | [`M2.depth_bias`](../src/robot_models/sensors/rgbd/README.md#rung-m2-depth_bias), [`M2.wear`](../src/robot_models/sensors/rgbd/README.md#rung-m2-wear) | [`M6.residual`](../src/robot_models/sensors/rgbd/README.md#rung-m6-residual) | — |
| **12 types** |   |   | **84** |   |   |   |

## The folder tree

- `actuators/`
  - [`servo/`](../src/robot_models/actuators/servo/README.md): `integrated_servo`
    - [`bldc/`](../src/robot_models/actuators/servo/bldc/README.md): `bldc_motor`
    - [`foc_controller/`](../src/robot_models/actuators/servo/foc_controller/README.md): `foc_controller`
    - [`gearbox/`](../src/robot_models/actuators/servo/gearbox/README.md): `gearbox`
- `arms/`
  - [`rigid_body_chain/`](../src/robot_models/arms/rigid_body_chain/README.md): `rigid_body_chain`
- `conveyors/`
  - [`conveyor/`](../src/robot_models/conveyors/conveyor/README.md): `conveyor`
- `end_effectors/`
  - [`gripper/`](../src/robot_models/end_effectors/gripper/README.md): `gripper`
- `markers/`
  - [`aruco_marker/`](../src/robot_models/markers/aruco_marker/README.md): `aruco_marker`
- `sensors/`
  - [`camera/`](../src/robot_models/sensors/camera/README.md): `camera`
  - [`encoder/`](../src/robot_models/sensors/encoder/README.md): `encoder`
  - [`hyperspectral_camera/`](../src/robot_models/sensors/hyperspectral_camera/README.md): `hyperspectral_camera`
  - [`rgbd/`](../src/robot_models/sensors/rgbd/README.md): `depth_camera`

## Across types

- [`integrated_servo` `M4.drivetrain`](../src/robot_models/actuators/servo/README.md#rung-m4-drivetrain) composes [`foc_controller` `M1.delay_loop`](../src/robot_models/actuators/servo/foc_controller/README.md#rung-m1-delay_loop)
- [`integrated_servo` `M5.envelope`](../src/robot_models/actuators/servo/README.md#rung-m5-envelope) composes [`foc_controller` `M2.ceiling_heat`](../src/robot_models/actuators/servo/foc_controller/README.md#rung-m2-ceiling_heat)

## Reading a rung key

A rung key is `M{level}.{name}`. The physics levels M0 to M5 are the actuator layers of [robotparts](#name-robotparts), where the numbering started: M0 the ideal model, M1 the datasheet, M2 Coulomb and viscous friction, M3 state-dependent friction, M4 the drivetrain, M5 electrics and heat. Every other type climbs its own physics over the same levels, from its simplest model at M0 (the ideal or nominal one). M6 is always the grey-box residual and M7 always the learned net; a rung named `wear` grows a parameter with use. Each rung extends one base and needs everything its base needs. Spec: [PEP 0004](peps/0004-model-ladders-by-type.md).

## Projects the notes name

The ladders' notes say where a rung or a name came from. These are the projects they name.

- <a id="name-cad2robot"></a>**Cad2Robot** ([cad2robot.com](https://cad2robot.com)): a pipeline from a robot's CAD to a MuJoCo simulation. Its exporter's motor and servo fields are aliases here.
- <a id="name-mujoco"></a>**MuJoCo** ([mujoco.org](https://mujoco.org)): the physics engine. Some rungs use its models as written (the servo's M5.dcmotor is its DC motor), and most packed names are that model's parameter arrays.
- <a id="name-robotparts"></a>**robotparts** ([robotparts.dev](https://robotparts.dev)): an open catalogue of robot parts. Its actuator models' layers are the physics levels M0 to M5 here, its eight actuator rungs moved here as the servo's code (actuators/rungs/), and its registry keys ("the registry") and old user-part keys are canonical names or aliases.
- <a id="name-sim2real"></a>**sim2real** ([github.com/zacharyyamaoka/sim2real](https://github.com/zacharyyamaoka/sim2real)): the outer API on top of this stack: twins and their calibration. Its type cards were the first source of most ladders here, and its twin names are aliases.
- <a id="name-the-rack-twin"></a>**the rack twin**: a fitted model of one lab test rack, kept with robotparts' lab tools and never published. Only its structure is used here: its terms and equations, never its fitted values.
