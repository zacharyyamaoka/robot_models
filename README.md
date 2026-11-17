# robot_models

Physical models and parameters of robot parts, the twin of robot_msgs: types, models, parameters
and pure fits that work on any data. Not ML models.

```
robot_msgs <- robot_models <- robot_description <- robotparts;   sim2real on top
```

- `robot_models.homing` -- the homing stop core (docs/peps/0001-homing-stop-core.md): stops,
  the B0/B1/B2 backlash ladder, stall evidence, the pure fit. numpy + robot_msgs only.
- `robot_models.homing.contact` / `cad_bundle` -- exact mesh first contact on coal (extra `contact`).
- `robot_models.datasheet` -- typed part fields and a part's values resolved from one Python file per raw
  source (docs/peps/0003-typed-datasheets-from-sources.md): `Stated`, `Source`, `Sources`, `Datasheet`,
  `Field`. Stdlib only.
- `robot_models.actuators.servo` -- the servo type tree, as robotparts' `user/parts/actuators/servo/`:
  `IntegratedServo`, `bldc.BldcMotor`, `gearbox.Gearbox`. Field names are robotparts registry keys.

Tests (the ruler needs the cad2robot bundle cache, `~/.cache/cad2robot-build/bundles`):

```bash
env -u VIRTUAL_ENV uv run --project /home/bam/robot_models --extra test pytest -q /home/bam/robot_models/tests
```

Private GitHub remote (`origin`, github.com/zacharyyamaoka/robot_models); making it public is Zach's call.
