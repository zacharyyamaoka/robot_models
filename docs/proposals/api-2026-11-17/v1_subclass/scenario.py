"""The shared scenario, typed with v1 "Subclass ladder": a Model is a frozen dataclass, a rung is a subclass of its base.

    cd /home/bam/robot_models/docs/proposals/api-2026-11-17/v1_subclass && /home/bam/robot_models/.venv/bin/python scenario.py

Each step is the lines a user types at the REPL; the runner below executes them in one namespace and echoes each
expression's value the way the REPL does, so the printed session is exactly what these lines produce. Steps 1-5 are the
workflow: `<case>` is the servo whose rig log it runs on ("A" when printed). Step 6 is the transcript's negative control,
the same calls typed again on servo B. run(case) runs steps 1-5 on that servo and returns what check_scenario.py reads.
"""

from __future__ import annotations

import ast
import contextlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for folder in (HERE, HERE.parent):  # this variant's package, and the shared fixture
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

STEPS = (
    # 1. Browse the integrated_servo models as a ladder: the type is the root class, the ladder its subclass tree.
    ("browse the ladder", """
>>> from subclass_ladder.servo import ServoModel, Drivetrain
>>> print(ServoModel.tree())
>>> ServoModel.ladder()
>>> Drivetrain.runnable, Drivetrain.needs()[-3:]
"""),
    # 2. Pick M3.stribeck: its parameters are its fields, its fallback is its MRO, and an alias resolves on the type.
    ("pick M3.stribeck", """
>>> from subclass_ladder.servo import Stribeck
>>> print(Stribeck.describe())
>>> [klass.__name__ for klass in Stribeck.__mro__]
>>> Stribeck.fallback()
>>> ServoModel.canonical("frictionloss_nm")
"""),
    # 3. The datasheet model (M1): catalogue units in, SI fields out; predict the resisting torque at 1 rad/s.
    ("datasheet to M1, predict", """
>>> import numpy as np
>>> from fixture import datasheet, rig_log
>>> from subclass_ladder.servo import Datasheet
>>> m1 = Datasheet.from_datasheet(datasheet())
>>> m1
>>> m1.torque(1.0)
>>> m1.torque(np.array([-1.0, 0.0, 0.5]))
"""),
    # 4. Fit M2 and M3 on the fit plateaus only: each is an alternative constructor on its class, M3 climbs from M2.
    ("fit M2 and M3", """
>>> from subclass_ladder.servo import CoulombViscous
>>> fit, test = rig_log("<case>", "fit"), rig_log("<case>", "test")
>>> m2 = CoulombViscous.fit(fit, start=m1)
>>> m3 = Stribeck.fit(fit, start=m2)
>>> m2.coulomb_friction, m2.viscous_friction
>>> m3.coulomb_friction, m3.breakaway_torque, m3.stribeck_velocity, m3.viscous_friction
"""),
    # 5. Compare on the held-out plateaus; choose the lowest rung on the fallback chain that reaches the noise floor.
    ("held out, choose", """
>>> comparison = ServoModel.compare([m1, m2, m3], test, fitted_on=fit)
>>> print(comparison)
>>> chosen = comparison.chosen
>>> chosen.key, chosen.mujoco()
"""),
)

NEGATIVE_CONTROL = (
    # 6. Servo B, the same calls: M3 fits its training plateaus better and predicts the held-out ones worse.
    ("servo B: simpler wins", """
>>> fit, test = rig_log("B", "fit"), rig_log("B", "test")
>>> m2 = CoulombViscous.fit(fit, start=m1)
>>> m3 = Stribeck.fit(fit, start=m2)
>>> print(ServoModel.compare([m1, m2, m3], test, fitted_on=fit))
>>> m2.coulomb_friction, m2.viscous_friction
"""),
)


def session(case: str = "A", show: bool = True, steps: tuple = STEPS) -> dict:
    """Run the steps in one namespace on servo `case`, echoing like the REPL. Returns the namespace."""
    namespace: dict = {}
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
    servo, m1, m2, m3, test = ns["ServoModel"], ns["m1"], ns["m2"], ns["m3"], ns["test"]
    return {
        "ladder": servo.ladder(),
        "fallback": ns["Stribeck"].fallback(),
        "alias": servo.canonical("frictionloss_nm"),
        "m1_at_1": m1.torque(1.0),
        "fits": {model.key: {"params": model.terms(model.needs()), "rms": model.score(test).rms} for model in (m2, m3)},
        "chosen": ns["chosen"].key,
    }


if __name__ == "__main__":
    session("A", show=True, steps=STEPS + NEGATIVE_CONTROL)
