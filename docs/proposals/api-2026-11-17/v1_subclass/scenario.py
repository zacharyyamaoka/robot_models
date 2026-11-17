"""The shared scenario, typed with v1 "Subclass ladder": a Model is a frozen dataclass, a rung is a subclass of its base.

    cd /home/bam/robot_models/docs/proposals/api-2026-11-17/v1_subclass && /home/bam/robot_models/.venv/bin/python scenario.py

Each step is the lines a user types at the REPL; the runner below executes them in one namespace and echoes each
expression's value the way the REPL does, so the printed session is exactly what these lines produce.
run() returns what check_scenario.py reads.
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
>>> from subclass_ladder.servo import ServoModel
>>> print(ServoModel.tree())
>>> ServoModel.ladder()
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
    # 4. Fit M2 and M3 on the rig log: each is an alternative constructor on its class, and M3 climbs from M2.
    ("fit M2 and M3", """
>>> from subclass_ladder.servo import CoulombViscous
>>> log = rig_log()
>>> m2 = CoulombViscous.fit(log, start=m1)
>>> m3 = Stribeck.fit(log, start=m2)
>>> m2.coulomb_friction, m2.viscous_friction
>>> m3.coulomb_friction, m3.breakaway_torque, m3.stribeck_velocity, m3.viscous_friction
>>> m3.output_inertia, m3.gear_ratio
"""),
    # 5. Compare the rungs on the same log and choose the one that pays.
    ("compare and choose", """
>>> comparison = ServoModel.compare([m1, m2, m3], log)
>>> print(comparison)
>>> chosen = comparison.chosen
>>> chosen.key, isinstance(chosen, CoulombViscous)
"""),
    # 6. Hand it on: the values go to MuJoCo as joint attributes; a rung above the stop line is declared, not run.
    ("hand it on", """
>>> chosen.mujoco()
>>> from subclass_ladder.servo import Drivetrain
>>> Drivetrain.runnable, Drivetrain.needs()[-3:]
"""),
)


def session(show: bool = True) -> dict:
    """Run every step in one namespace, echoing like the REPL. Returns the namespace."""
    namespace: dict = {}
    sink = contextlib.nullcontext() if show else contextlib.redirect_stdout(io.StringIO())
    with sink:
        for number, (label, block) in enumerate(STEPS, 1):
            lines = [line[4:] for line in block.strip().splitlines()]
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


def run(show: bool = False) -> dict:
    ns = session(show)
    servo, m1, m2, m3, log = ns["ServoModel"], ns["m1"], ns["m2"], ns["m3"], ns["log"]
    return {
        "ladder": servo.ladder(),
        "fallback": ns["Stribeck"].fallback(),
        "alias": servo.canonical("frictionloss_nm"),
        "m1_at_1": m1.torque(1.0),
        "fits": {model.key: {"params": model.terms(model.needs()), "rms": model.score(log).rms} for model in (m2, m3)},
        "chosen": ns["chosen"].key,
    }


if __name__ == "__main__":
    session(show=True)
