"""The shared scenario, typed with v8 "Parameter sets": ModelFamily -> Model (stateless physics) -> Parameters (values).

    cd /home/bam/robot_models/docs/proposals/api-2026-11-17/v8_parameter_sets && \
        /home/bam/robot_models/.venv/bin/python scenario.py

Each step is the lines a user types at the REPL; the runner executes them in one namespace and echoes each expression's
value the way the REPL does, so the printed session is exactly what these lines produce (a line marked `# raises`
prints its exception). Steps 1-6 are the workflow on servo `<case>` ("A" when printed); step 7 is the negative control,
the same calls on servo B. run(case) runs steps 1-6 on that servo and returns what check_scenario.py reads.
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
    # 1. Browse: the family is the type; iterating it walks its ladder; tree() prints and renders it.
    ("browse the family", """
>>> from parameter_sets import IntegratedServo
>>> IntegratedServo.tree().print()
>>> IntegratedServo.tree().show(here / "integrated_servo_tree.html").name
>>> for model in IntegratedServo: print(model.key, model.param_count, model.runnable())
>>> IntegratedServo.ladder()[0]
>>> IntegratedServo.models.m3
"""),
    # 2. Pick M3.stribeck: a class of Parameter declarations; its values class is generated from them.
    ("pick M3.stribeck", """
>>> Stribeck = IntegratedServo.models["M3.stribeck"]
>>> Stribeck is IntegratedServo.models.m3.stribeck
>>> print(Stribeck.describe())
>>> Stribeck.breakaway_torque
>>> Stribeck.fallback()
>>> [klass.__qualname__ for klass in Stribeck.Parameters.__mro__[:4]]
>>> IntegratedServo.canonical("frictionloss_nm")
"""),
    # 3. One source: today's Ladder is derived from the classes; the vocabulary only grows.
    ("one source, additive only", """
>>> from robot_models.ladder import ladders
>>> IntegratedServo.drift(ladders()["integrated_servo"])
>>> class Smoother(Stribeck, level=4, name="smoother"):
...     coulomb_friction = None  # raises
>>> len(IntegratedServo)
"""),
    # 4. The datasheet model (M1): catalogue units in, an SI value set out; the model predicts from it.
    ("datasheet to M1, predict", """
>>> import numpy as np
>>> from fixture import datasheet, rig_log
>>> m1 = IntegratedServo.models.m1.datasheet.from_datasheet(datasheet())
>>> m1
>>> m1.predict(1.0)
>>> IntegratedServo.Datasheet.predict(m1, np.array([-1.0, 0.0, 0.5]))
"""),
    # 5. Fit on the fit plateaus only: one model, then the whole family (each rung climbs from the fit below it).
    ("fit", """
>>> fit_log, test_log = rig_log("<case>", "fit"), rig_log("<case>", "test")
>>> m3 = Stribeck.fit(fit_log, start=m1)
>>> m3.needed()
>>> m3.info
>>> fits = IntegratedServo.fit(fit_log, start=m1)
>>> print(fits)
>>> fits["M3.stribeck"].diff(fits["M2.coulomb_viscous"])
>>> IntegratedServo.load(fits["M3.stribeck"].to_json()) == fits["M3.stribeck"]
"""),
    # 6. Choose on the held-out plateaus: the lowest rung within 1.5x the rig's noise floor; the black box competes.
    ("held out, select_best", """
>>> selection = IntegratedServo.select_best(fits, test_log)
>>> print(selection)
>>> best = selection.best
>>> best.model.key, best.mujoco()
>>> best.predict(np.array([0.05, 0.5, 2.0]))
"""),
)

NEGATIVE_CONTROL = (
    # 7. Servo B, the same calls: M3 fits its training plateaus better and predicts the held-out ones worse.
    ("servo B: simpler wins", """
>>> fits_b = IntegratedServo.fit(rig_log("B", "fit"), start=m1)
>>> print(IntegratedServo.select_best(fits_b, rig_log("B", "test")))
>>> fits_b["M2.coulomb_viscous"].needed()
"""),
)


def session(case: str = "A", show: bool = True, steps: tuple = STEPS) -> dict:
    """Run the steps in one namespace on servo `case`, echoing like the REPL. Returns the namespace."""
    namespace: dict = {"here": HERE}
    sink = contextlib.nullcontext() if show else contextlib.redirect_stdout(io.StringIO())
    with sink:
        for number, (label, block) in enumerate(steps, 1):
            lines = [line[4:] for line in block.strip().replace("<case>", case).splitlines()]
            source = "\n".join(lines)
            print(f"\n# {number} · {label}")
            for node in ast.parse(source).body:
                text = lines[node.lineno - 1:node.end_lineno]
                for offset, line in enumerate(text):
                    print(("... " if offset else ">>> ") + line)
                try:
                    if isinstance(node, ast.Expr):
                        value = eval(compile(ast.Expression(node.value), "<scenario>", "eval"), namespace)
                        if value is not None:
                            print(repr(value))
                    else:
                        exec(compile(ast.Module([node], []), "<scenario>", "exec"), namespace)
                except Exception as error:
                    if "# raises" not in "\n".join(text):
                        raise
                    print(f"{type(error).__name__}: {error}")
    return namespace


def run(case: str = "A", show: bool = False) -> dict:
    ns = session(case, show)
    family, fits = ns["IntegratedServo"], ns["fits"]
    return {
        "ladder": tuple(model.key for model in family),
        "fallback": ns["Stribeck"].fallback(),
        "alias": family.canonical("frictionloss_nm"),
        "m1_at_1": ns["m1"].predict(1.0),
        "fits": {key: {"params": fits[key].needed()} for key in ("M2.coulomb_viscous", "M3.stribeck")},
        "chosen": ns["selection"].chosen,
    }


if __name__ == "__main__":
    session("A", show=True, steps=STEPS + NEGATIVE_CONTROL)
