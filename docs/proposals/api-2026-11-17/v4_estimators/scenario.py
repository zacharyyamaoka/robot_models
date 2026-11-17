"""The shared scenario with v4's API: "I have an accurate datasheet and the rig; I selected the models. What now?"

    /home/bam/robot_models/.venv/bin/python scenario.py       # prints the session, as a REPL would show it

Each step is the code a user types, kept as text so the transcript is exactly what runs: `run()` executes the steps
in one namespace, echoing each statement and the value of each bare expression, and returns what check_scenario.py
grades.
"""

from __future__ import annotations

import ast
import contextlib
import io
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]      # this variant's estimators.py, then the round's fixture.py

STEPS = (
    ("1 · Browse the integrated_servo models as a ladder", '''
from estimators import Model, families
servo = families()["integrated_servo"]       # the type's models, organized as its ladder: the climb's search order
print(servo)
ladder = list(servo.models())                # every key, in today's order: each one an unfitted Model
'''),
    ("2 · Pick M3.stribeck: what it needs, means and falls back on; resolve an alias", '''
stribeck = servo.model("M3.stribeck")
print(stribeck.describe())
servo.canonical("frictionloss_nm")                               # the rack twin's name for the quantity
servo.model("M3.stribeck", frictionloss_nm=0.3).get_params()     # set by an alias, held by the canonical name
'''),
    ("3 · The datasheet model: M1 from the stated values, then predict", '''
from fixture import datasheet, rig_log
ds = datasheet()                              # the catalogue's units: rpm, kg·cm², mN·m·s/rad
m1 = Model.from_datasheet(ds)                 # M1.datasheet by default: every stated value it holds, in SI
print(m1)
m1.predict(1.0)                               # tau_r at 1 rad/s [N·m]
'''),
    ("4 · Fit M2 and M3 to the rig log", '''
log = rig_log()                               # velocity plateaus, both directions: v [rad/s], tau [N·m]
m2 = Model.from_datasheet(ds, "M2.coulomb_viscous").fit(log)
m3 = Model.from_datasheet(ds, "M3.stribeck").fit(log)    # starts from the datasheet; the rest is scanned
print(m2)
print(m3)
m1.score(log), m2.score(log), m3.score(log)  # R², sklearn's score
'''),
    ("5 · Compare: climb the ladder while the next rung pays", '''
report = servo.climb(ds, log)                 # M1 from the datasheet, then fit every rung above, keep the best
print(report)
report.chosen
'''),
    ("6 · The same climb stops where the log stops paying", '''
fast = {column: values[abs(log["v"]) >= 0.8] for column, values in log.items()}  # only plateaus ≥ 0.8 rad/s
print(servo.climb(ds, fast))                  # no slow plateaus: the Stribeck drop is invisible, so M3 is not worth it
'''),
)


def _session(source: str, namespace: dict) -> None:
    """Run `source` as a REPL would: echo each statement (with its comments), print each bare expression's repr."""
    lines = source.strip("\n").splitlines()
    for node in ast.parse("\n".join(lines)).body:
        for number, line in enumerate(lines[node.lineno - 1:node.end_lineno]):
            print((">>> " if number == 0 else "... ") + line)
        if isinstance(node, ast.Expr):
            value = eval(compile(ast.Expression(node.value), "<scenario>", "eval"), namespace)
            if value is not None:
                print(repr(value))
        else:
            exec(compile(ast.Module([node], []), "<scenario>", "exec"), namespace)


def run(echo: bool = False) -> dict:
    namespace: dict = {}
    with contextlib.nullcontext() if echo else contextlib.redirect_stdout(io.StringIO()):
        for title, source in STEPS:
            print(f"\n# Step {title}")
            _session(source, namespace)
    fits = {key: {"params": model.get_params(), "rms": model.rms_}
            for key, model in (("M2.coulomb_viscous", namespace["m2"]), ("M3.stribeck", namespace["m3"]))}
    return {
        "ladder": namespace["ladder"],
        "fallback": namespace["stribeck"].fallback,
        "alias": namespace["servo"].canonical("frictionloss_nm"),
        "m1_at_1": namespace["m1"].predict(1.0),
        "fits": fits,
        "chosen": namespace["report"].chosen.key,
    }


if __name__ == "__main__":
    out = run(echo=True)
    print(f"\n# run() returns: chosen {out['chosen']}, M3 rms {out['fits']['M3.stribeck']['rms']:.4f} "
          f"< M2 rms {out['fits']['M2.coulomb_viscous']['rms']:.4f}")
