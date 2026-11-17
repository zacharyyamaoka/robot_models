"""The shared scenario, typed against the REAL package: v6 "Fitted instances" as built in src/ (PEP 0004 amendment 2).

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/package_v6/scenario.py
    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/check_scenario.py package_v6

The same steps as the v6 prototype's scenario.py, with `from robot_models import IntegratedServo` (a PartType) in place
of the prototype's package, and the round-2 group calls on a level (`IntegratedServo.models.m3.fit`). No prototype code
is on the path: only this folder's parent, for the shared fixture.

Each step is the lines a user types at the REPL; the runner executes them in one namespace and echoes each
expression's value the way the REPL does, so the printed session is exactly what these lines produce. `<case>` is the
servo whose rig log a step runs on ("A" when printed). The last step, the negative control, types the same calls on
servo B. run(case) runs the workflow on that servo and returns what check_scenario.py reads.
"""

from __future__ import annotations

import ast
import contextlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE.parent) not in sys.path:  # the shared fixture only: the API under test is the installed robot_models
    sys.path.insert(0, str(HERE.parent))

STEPS = (
    # 1. Zach's access shape (ROUND2.md), as written.
    ("browse: the type, its models, its ladder and tree", """
>>> from robot_models import IntegratedServo
>>> IntegratedServo
>>> IntegratedServo.models
>>> IntegratedServo.models["M3.stribeck"]
>>> IntegratedServo.models.m3
>>> IntegratedServo.models.m3.stribeck is IntegratedServo.models["M3.stribeck"]
>>> IntegratedServo.ladder()[0]
>>> IntegratedServo.tree().print()
>>> IntegratedServo.tree().show().name
>>> IntegratedServo.models.m3.tree().print()
>>> IntegratedServo.Datasheet, IntegratedServo.models.m1.datasheet
>>> for model in IntegratedServo: print(f"{model.key:<22}{model.param_count:>3}  {'runs' if model.runnable() else 'declared'}")
"""),
    # 2. The classes are the one source: today's Ladder record is derived from them; vocabulary only grows.
    ("one source, additive vocabulary", """
>>> from robot_models.ladder import ladders
>>> today, derived = ladders()["integrated_servo"], IntegratedServo.v1_ladder()
>>> derived.keys == today.keys
>>> all((d.base, d.adds, d.optional) == (t.base, t.adds, t.optional) for d, t in zip(derived.rungs, today.rungs))
>>> derived == today
>>> from robot_models.part_type import Parameter
>>> try:
...     class Newtons(IntegratedServo.models.m3.stribeck, level=3, name="newtons"):
...         coulomb_friction = Parameter("N", "Coulomb friction, in newtons.")
... except TypeError as error:
...     print(error)
>>> try:
...     class Forgetful(IntegratedServo.models.m3.stribeck, level=3, name="forgetful"):
...         breakaway_torque = None
... except TypeError as error:
...     print(error)
>>> len(IntegratedServo)
"""),
    # 3. Pick M3.stribeck: the class is the type; its Parameters carry unit and definition; fallback is the MRO.
    ("pick M3.stribeck", """
>>> Stribeck = IntegratedServo.models.m3.stribeck
>>> print(Stribeck.describe())
>>> Stribeck.breakaway_torque
>>> Stribeck.fallback()
>>> IntegratedServo.canonical("frictionloss_nm")
"""),
    # 4. The datasheet model (M1): catalogue units in, an instance with SI values out; predict at 1 rad/s.
    ("datasheet to M1, predict", """
>>> import numpy as np
>>> from fixture import datasheet, rig_log
>>> m1 = IntegratedServo.models.m1.datasheet.from_datasheet(datasheet())
>>> m1
>>> m1.predict(1.0)
>>> m1.predict(np.array([-1.0, 0.0, 0.5]))
"""),
    # 5. Fit: one model (returns an instance), then the whole type, then one level (any Models group fits).
    ("fit: one model, then the type, then a level", """
>>> fit, test = rig_log("<case>", "fit"), rig_log("<case>", "test")
>>> m3 = Stribeck.fit(fit, start=m1)
>>> m3.breakaway_torque, m3.stribeck_velocity
>>> m3.params["stribeck_velocity"]
>>> m3.predict(np.linspace(-3, 3, 100_000)).shape
>>> fits = IntegratedServo.fit(fit, start=m1)
>>> print(fits)
>>> print(IntegratedServo.models.m3.fit(fit, start=m1))
"""),
    # 6. Choose on the held-out plateaus: the lowest rung within 1.5x the rig's noise floor (sim2real's rule).
    ("held out, choose", """
>>> best = IntegratedServo.select_best(test, fits)
>>> print(best)
>>> IntegratedServo.models.m3.select_best(test, fits).chosen
>>> chosen = best.chosen
>>> chosen.key, chosen.mujoco()
"""),
    # 7. The black box has the same outside: fit, predict, param_count, and it competes in select_best above.
    ("black box, same calls", """
>>> Learned = IntegratedServo.models.m7.learned
>>> net = Learned.fit(fit)
>>> net, net.param_count
>>> round(net.predict(1.0), 4), round(fits["M3.stribeck"].predict(1.0), 4)
>>> round(net.score(fit) * 1e3, 2), round(net.score(test) * 1e3, 2)
"""),
)

NEGATIVE_CONTROL = (
    # 8. Servo B, the same two calls: M3 fits its training plateaus better and predicts the held-out ones worse.
    ("servo B: simpler wins", """
>>> fit, test = rig_log("B", "fit"), rig_log("B", "test")
>>> print(IntegratedServo.fit(fit, start=m1).select_best(test))
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
    part_type, fits = ns["IntegratedServo"], ns["fits"]
    return {
        "ladder": tuple(model.key for model in part_type.ladder()),
        "fallback": ns["Stribeck"].fallback(),
        "alias": part_type.canonical("frictionloss_nm"),
        "m1_at_1": ns["m1"].predict(1.0),
        "fits": {key: {"params": {n: fits[key].values()[n] for n in fits[key].needs() if n in fits[key].values()}}
                 for key in ("M2.coulomb_viscous", "M3.stribeck")},
        "chosen": ns["chosen"].key,
    }


if __name__ == "__main__":
    session("A", show=True, steps=STEPS + NEGATIVE_CONTROL)
