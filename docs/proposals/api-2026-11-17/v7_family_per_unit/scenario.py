"""The shared scenario, typed with v7 "Family per unit": the family class is the type, a family instance is one servo.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/v7_family_per_unit/scenario.py

Each step is the lines a user types at the REPL; the runner executes them in one namespace and echoes each expression's
value the way the REPL does, so the printed session is exactly what these lines produce. Steps 1-6 are the workflow on
servo `<case>` ("A" when printed); step 7 is the negative control, servo B as a second unit. run(case) runs steps 1-6
on that servo's rig log and returns what check_scenario.py reads. `case` only ever reaches rig_log and the unit's name.
"""

from __future__ import annotations

import ast
import contextlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for folder in (HERE, HERE.parent):  # this variant's modules, and the shared fixture
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

STEPS = (
    # 1. The type: browse it without any values. Iterating the class walks its ladder.
    ("browse the type", """
>>> from servo_family import IntegratedServo
>>> IntegratedServo.tree().print()
>>> IntegratedServo.ladder()[0]
>>> for model in IntegratedServo: print(f"{model.key:<22}{model.param_count:>3}  {model.kind}")
>>> IntegratedServo.tree().show(Path(__file__).parent / "tree.html").name
"""),
    # 2. Pick M3.stribeck: by key or attribute (m3 is a group); a class attribute is the Parameter itself.
    ("pick M3.stribeck", """
>>> IntegratedServo.models["M3.stribeck"] is IntegratedServo.models.m3.stribeck
>>> IntegratedServo.models.m3
>>> Stribeck = IntegratedServo.models.m3.stribeck
>>> print(Stribeck.describe())
>>> Stribeck.breakaway_torque
>>> Stribeck.fallback()
>>> IntegratedServo.canonical("frictionloss_nm")
"""),
    # 3. One source: today's v1 Ladder is derived from the classes, and a subclass can only add vocabulary.
    ("one source, additive", """
>>> from robot_models.ladder import ladders
>>> IntegratedServo.ladder_diff(ladders()["integrated_servo"])
>>> IntegratedServo.as_ladder().keys == ladders()["integrated_servo"].keys
>>> from family_per_unit import Parameter
>>> try:
...     class Broken(Stribeck, level=3, name="broken"):
...         breakaway_torque = Parameter(optional=True)
... except TypeError as error:
...     print(error)
>>> "M3.broken" in IntegratedServo.models
"""),
    # 4. One physical unit from its datasheet: every model instance holds what the catalogue states, in SI.
    ("the unit: datasheet to M1, predict", """
>>> import numpy as np
>>> from fixture import datasheet, rig_log
>>> servo = IntegratedServo(datasheet=datasheet(), name="servo <case>")
>>> m1 = servo.models.m1.datasheet
>>> m1.predict(1.0)
>>> m1.predict(np.array([-1.0, 0.0, 0.5]))
"""),
    # 5. Fit the unit: every model the log can identify, lowest first, each climbing from the fitted rung below.
    ("fit the unit", """
>>> fit, test = rig_log("<case>", "fit"), rig_log("<case>", "test")
>>> servo.fit(fit)
>>> servo.models.m3.stribeck.params
>>> servo.models.m3.stribeck.parameter("breakaway_torque")
>>> servo[Stribeck].stribeck_velocity
>>> servo.models.m3.stribeck.parity(test)
>>> Stribeck(output_inertia=4.2e-5, gear_ratio=10).fit(fit)
"""),
    # 6. Choose on the held-out plateaus: the lowest rung within 1.5x the rig's own floor. M7 competes with the same calls.
    ("select_best on held out", """
>>> servo.select_best(test)
>>> servo.best
>>> servo.best.mujoco()
>>> servo.tree().print()
"""),
)

NEGATIVE_CONTROL = (
    # 7. Servo B, a second unit, the same calls: M3 fits its own plateaus better and predicts the held-out ones worse.
    ("servo B: simpler wins", """
>>> servo_b = IntegratedServo.fit(rig_log("B", "fit"), datasheet=datasheet(), name="servo B")
>>> servo_b.select_best(rig_log("B", "test"))
>>> servo_b.best.params
>>> [unit.best.key for unit in (servo, servo_b)]
"""),
)


def session(case: str = "A", show: bool = True, steps: tuple = STEPS) -> dict:
    """Run the steps in one namespace on servo `case`, echoing like the REPL. Returns the namespace."""
    namespace: dict = {"__file__": str(HERE / "scenario.py"), "Path": Path}
    sink = contextlib.nullcontext() if show else contextlib.redirect_stdout(io.StringIO())
    with sink:
        for number, (label, block) in enumerate(steps, 1):
            lines = [line[4:] for line in block.strip().replace("<case>", case).splitlines()]
            source = "\n".join(lines)
            print(f"\n# {number} · {label}")
            for node in ast.parse(source).body:
                for offset, line in enumerate(lines[node.lineno - 1:node.end_lineno]):
                    print(("... " if offset else ">>> ") + line)
                if isinstance(node, ast.Expr):
                    value = eval(compile(ast.Expression(node.value), "<scenario>", "eval"), namespace)
                    if value is not None:
                        print(repr(value))
                else:
                    exec(compile(ast.Module([node], []), "<scenario>", "exec"), namespace)
    return namespace


def run(case: str = "A", show: bool = False) -> dict:
    ns = session(case, show)
    family, servo = ns["IntegratedServo"], ns["servo"]
    return {
        "ladder": tuple(model.key for model in family),
        "fallback": family.models.m3.stribeck.fallback(),
        "alias": family.canonical("frictionloss_nm"),
        "m1_at_1": servo.models.m1.datasheet.predict(1.0),
        "fits": {key: {"params": servo.models[key].params} for key in ("M2.coulomb_viscous", "M3.stribeck")},
        "chosen": servo.best.key,
    }


if __name__ == "__main__":
    session("A", show=True, steps=STEPS + NEGATIVE_CONTROL)
