# robot_models

Generic physical models of robot part **types** (no maker, no SKU): each type is a ladder of models from ideal to
learned, and every rung states its parameters, its equation and its references.

```
robot_msgs ← robot_models ← robot_parts ← robot_descriptions,   with sim2real on top
```

- [robot_msgs](https://github.com/zacharyyamaoka/robot_msgs): the message types this package builds on.
- [sim2real](https://github.com/zacharyyamaoka/sim2real): the outer API (twins, calibration) over the stack.
- robot_parts (robotparts, private): the part catalogue, whose parts point at these types and rungs.
- Type data only: parameter names, SI units, definitions, equations. Never a value measured on one unit.

## Usage

```python
from robot_models.ladder import ladders, needs

servo = ladders()["integrated_servo"]
servo.parameters("M3.stribeck")     # what the rung needs, inherited names included:
# ('output_inertia', 'gear_ratio', 'coulomb_friction', 'viscous_friction', 'breakaway_torque', 'stribeck_velocity')
servo.holds("M3.stribeck")          # ... plus what it reads when a source states it
servo.fallback("M3.lugre")          # the chain it falls back down:
# ('M3.lugre', 'M3.stribeck', 'M2.coulomb_viscous', 'M1.datasheet', 'M0.ideal')
servo.canonical("frictionloss_nm")  # 'coulomb_friction': any other name resolves to the type's own
servo.unpack("thermal_r_c")         # ('thermal_resistance', 'thermal_time_constant'): a packed array
servo.rung("M3.stribeck").equation  # 'tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(...)'
needs(ladders(), "integrated_servo", "M4.drivetrain")  # {type: names}, the composed driver's included
```

- A rung key is `M{level}.{name}`: M0 ideal, M1 datasheet, M2 Coulomb + viscous, M3 state-dependent friction,
  M4 drivetrain, M5 electrics and heat, M6 grey-box residual, M7 learned net; `.wear` variants grow a parameter with use.
- Each ladder is a tree of explicit bases; a rung needs everything its base needs.
- Every other name for a quantity is an `Alias` (with a note when it is not the same quantity one for one) or a
  `Packed` array; what a deployment sets (gains, caps) is `DeploymentOnly`, never on a rung.
- Spec: [PEP 0004](docs/peps/0004-model-ladders-by-type.md).

## The 12 types (84 rungs)

| type | folder (`src/robot_models/…`) | rungs | what it is |
|---|---|---:|---|
| `integrated_servo` | `actuators/servo/` | 14 | A joint in one housing: motor, drive, reducer or none, encoder |
| `bldc_motor` | `actuators/servo/bldc/` | 9 | A brushless motor alone, at its shaft |
| `gearbox` | `actuators/servo/gearbox/` | 8 | A gear stage between a motor and its load |
| `foc_controller` | `actuators/servo/foc_controller/` | 6 | A field-oriented-control motor drive board |
| `encoder` | `sensors/encoder/` | 4 | An angle sensor on a shaft |
| `camera` | `sensors/camera/` | 6 | A frame camera's colour image: projection, distortion, exposure |
| `depth_camera` | `sensors/rgbd/` | 5 | A stereo depth camera: depth on top of a colour stream |
| `hyperspectral_camera` | `sensors/hyperspectral_camera/` | 5 | A line-scan spectral camera over a belt |
| `rigid_body_chain` | `arms/rigid_body_chain/` | 10 | A chain of rigid links and joints, open or closed: an arm, a wrist, a five-bar |
| `gripper` | `end_effectors/gripper/` | 6 | An end effector that grasps, on a chain's flange |
| `conveyor` | `conveyors/conveyor/` | 7 | A belt driven through a drum |
| `aruco_marker` | `markers/aruco_marker/` | 4 | A printed sheet carrying ArUco markers rigidly |

Each folder holds one `ladder.py`; `ladders()` finds them by walking the package (no registry). The integrated servo's
eight rung folders (`actuators/rungs/<rung>/`: `model.json`, a README, and Python + JavaScript code where the rung has
its own) are byte-pinned, so a downstream copy can be checked against them.

## Other modules

- `robot_models.datasheet`: typed part fields, and a part's values resolved from one Python file per raw source
  ([PEP 0003](docs/peps/0003-typed-datasheets-from-sources.md)). Stdlib only.
- `robot_models.actuators.servo`: the servo type tree (`IntegratedServo`, `BldcMotor`, `Gearbox`), the typed fields
  each ladder's M1 states.
- `robot_models.homing`: the homing stop core: stops, the backlash ladder, stall evidence, the pure fit, the bench's
  five metrics ([PEP 0001](docs/peps/0001-homing-stop-core.md), [PEP 0002](docs/peps/0002-homing-bench.md)).
  numpy + robot_msgs only; exact mesh contact (`homing.contact`, `homing.cad_bundle`) is the `contact` extra.
- The ladder core (`robot_models.ladder`) and the citable works (`robot_models.references`) are stdlib only.

## Install

```bash
pip install robot-models-lib
uv add robot-models-lib
```

- The PyPI name is robot-models-lib (PyPI refuses robot-models as too close to an existing robotmodels); you still `import robot_models`.
- robot-msgs 0.3.0 or later comes with it. The ladders and the datasheets import neither robot_msgs nor numpy.
- Mesh contact for homing: add the extra, `robot-models-lib[contact]`.
- Python 3.11 to 3.14.

## Tests

From a clone (`--no-sources` because `[tool.uv.sources]` names the author's local robot_msgs checkout):

```bash
uv run --isolated --no-sources --extra test python -m pytest -q --ignore=tests/test_fit_ruler.py --ignore=tests/test_stop_ruler.py
```

- That is what CI runs (`.github/workflows/test.yml`, Python 3.11–3.14).
- `tests/test_fit_ruler.py` and `tests/test_stop_ruler.py` read CAD bundles from a local, unpublished cache
  (`ROBOT_MODELS_CAD_BUNDLES`, default `~/.cache/cad2robot-build/bundles`); with that cache, drop the two `--ignore`s.
- `PYTHONDONTWRITEBYTECODE=1` keeps `__pycache__/` out of the rung folders, which robotparts vendors byte for byte.

## Design records

[docs/peps/](docs/peps/), each a decision with its undo:

- [0001](docs/peps/0001-homing-stop-core.md): the homing stop core lives in robot_models
- [0002](docs/peps/0002-homing-bench.md): the homing bench's pure parts: five metrics, two seams
- [0003](docs/peps/0003-typed-datasheets-from-sources.md): typed datasheets, resolved from one Python file per source
- [0004](docs/peps/0004-model-ladders-by-type.md): model ladders by type: one vocabulary per type, rungs as a tree of
  bases

## License

MIT, see [LICENSE](LICENSE).
