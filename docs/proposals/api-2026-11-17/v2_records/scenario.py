"""The shared scenario with v2 "Records in a Family": "I have an accurate datasheet and the rig; I selected the models.
What now?"

Each step is the lines a user types at the REPL, run here verbatim: `python scenario.py` prints them with their output
(a line ending `# -> SomeError` is expected to raise, and the transcript shows the error as a REPL would).
"""

from __future__ import annotations

import contextlib
import io
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))   # the shared fixture
sys.path.insert(0, str(HERE))          # this variant's core

STEPS = (
    ("1. Browse the integrated_servo models as a ladder", """
from family_records import families    # robot_models.family once landed
servo = families()["integrated_servo"]
print(servo.tree())
servo.children("M2.coulomb_viscous")
"""),
    ("2. Pick M3.stribeck: parameters, units, equation, references, fallback; resolve an alias", """
stribeck = servo["M3.stribeck"]
print(stribeck)
servo.fallback("M3.stribeck")
servo.canonical("frictionloss_nm")
"""),
    ("3. The datasheet model (M1) from the datasheet: the resisting torque at 1 rad/s", """
from fixture import datasheet, rig_log
values = servo.values(datasheet())
values
m1 = servo["M1.datasheet"].bind(values)
m1(v=1.0)
m1(v=np.array([-1.0, 0.0, 1.0]))
"""),
    ("4. Fit M2.coulomb_viscous and M3.stribeck to the rig log", """
log = rig_log()
m2 = servo["M2.coulomb_viscous"].fit(log["tau"], v=log["v"], values=values)
m3 = stribeck.fit(log["tau"], v=log["v"], values=values)
print(m3)
round(m2.rms, 4), round(m3.rms, 4)
"""),
    ("5. Compare them and choose the rung that pays", """
verdict = servo.compare(m1, m2, m3, observed=log["tau"], v=log["v"])
verdict
verdict.chosen
"""),
    ("6. Hand it on: MuJoCo's joint attributes, a vectorized call; a declared rung says it does not run", """
m3.mujoco()
m3(v=np.array([0.05, 0.5, 2.0])).round(4)
servo["M4.drivetrain"].fit(log["tau"], v=log["v"], values=values)  # -> NotImplementedError
"""),
)


def _repl(code: str, namespace: dict) -> None:
    """Run lines as the REPL does: echo an expression's value, run a statement, show an expected error."""
    for line in filter(str.strip, code.strip().splitlines()):
        print(f">>> {line}")
        try:
            try:
                compiled, is_expression = compile(line, "<scenario>", "eval"), True
            except SyntaxError:
                compiled, is_expression = compile(line, "<scenario>", "exec"), False
            result = eval(compiled, namespace) if is_expression else exec(compiled, namespace)
            if is_expression and result is not None:
                print(repr(result))
        except Exception as error:
            if f"# -> {type(error).__name__}" not in line:
                raise
            print(f"{type(error).__name__}: {error}")


def run(echo: bool = False) -> dict:
    namespace = {"np": np}
    with contextlib.nullcontext() if echo else contextlib.redirect_stdout(io.StringIO()):
        for title, code in STEPS:
            print(f"\n## {title}")
            _repl(code, namespace)
    servo, m2, m3 = namespace["servo"], namespace["m2"], namespace["m3"]
    return {
        "ladder": servo.keys,
        "fallback": servo.fallback("M3.stribeck"),
        "alias": servo.canonical("frictionloss_nm"),
        "m1_at_1": namespace["m1"](v=1.0),
        "fits": {bound.model.key: {"params": {name: bound.values[name] for name in bound.fitted}, "rms": bound.rms}
                 for bound in (m2, m3)},
        "chosen": namespace["verdict"].chosen,
    }


if __name__ == "__main__":
    run(echo=True)
