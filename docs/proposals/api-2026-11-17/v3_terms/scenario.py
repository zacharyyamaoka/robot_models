"""The shared scenario with v3 "Composed terms": typed as a REPL session, run here statement by statement.

    /home/bam/robot_models/.venv/bin/python scenario.py      # prints the session transcript (scenario.out.txt)

run() returns what check_scenario.py reads. Each STEPS block is what a user types; the runner echoes it with >>> and
lets expressions print themselves (compile mode "single", as the REPL does), so the transcript is the real output.
"""

from __future__ import annotations

import ast
import contextlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for folder in (HERE, HERE.parent):  # this variant's modules, then the shared fixture (fixture.py)
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

STEPS = (
    ("1. Browse the integrated_servo models: one row per preset (today's ladder keys), one column per slot", '''
from servo_slots import servo
print(servo)   # * = runs in this prototype (M0-M3); every other row is declared, not run
'''),
    ("2. Pick M3.stribeck: a preset is a named composition; climbing to it swaps one slot", '''
m3 = servo["M3.stribeck"]
m3 == servo["M2.coulomb_viscous"].redeclare(friction="stribeck")
print(m3.describe())   # each term's equation, what the model needs (units, meaning), what it reads, references
'''),
    ("2b. Its fallback, an alias, and a composition no rung names", '''
for lower in m3.fallback(): print(lower)   # each step down steps back or empties one slot
m3.canonical("frictionloss_nm")            # the rack twin's name for one of its quantities
m3 + servo.term("ripple")                  # lmfit-style +: the parameters are the union
(m3 + servo.term("ripple")).parameters[-1]
m3 + servo.term("lugre")                   # a slot holds one option
'''),
    ("3. The datasheet model (M1): stated values in SI, then the resisting torque at 1 rad/s", '''
import numpy as np
from fixture import datasheet, rig_log
m1 = servo["M1.datasheet"].from_datasheet(datasheet())   # catalogue units in: rpm, kg·cm², mN·m·s/rad
m1
m1.predict(1.0)
m1.predict(np.array([-1.0, 0.1, 1.0]))
servo["M3.stribeck"].from_datasheet(datasheet())        # a datasheet states no Stribeck curve
'''),
    ("4. Fit M2.coulomb_viscous, then climb one slot to M3.stribeck and fit again on the same rig log", '''
log = rig_log()   # velocity plateaus, both directions: v [rad/s], tau [N·m]
m2_fit = servo["M2.coulomb_viscous"].fit(log, start=m1)
m3_fit = m2_fit.model.redeclare(friction="stribeck").fit(log, start=m2_fit)
m2_fit
m3_fit
'''),
    ("5. Compare them: does the friction swap pay?", '''
from composed_terms import compare
verdict = compare(m2_fit, m3_fit)
print(verdict)
verdict.chosen
m3_fit.params   # SI values by canonical name, ready for MuJoCo or sim2real
'''),
)

# WHY only these: the API's own teaching errors print as the REPL shows them; anything else is a bug and raises.
SHOWN_ERRORS = (ValueError, KeyError, NotImplementedError)


def _session() -> dict:
    namespace: dict = {"__name__": "__session__"}
    for title, block in STEPS:
        print(f"\n# {title}")
        lines = block.strip("\n").splitlines()
        for node in ast.parse("\n".join(lines)).body:
            source = "\n".join(lines[node.lineno - 1: node.end_lineno])
            print(">>> " + source.replace("\n", "\n... "))
            try:
                exec(compile(source + "\n", "<session>", "single"), namespace)
            except SHOWN_ERRORS as error:
                print(f"{type(error).__name__}: {error.args[0] if error.args else ''}")
    return namespace


def run(echo: bool = False) -> dict:
    """The scenario's results in the shape check_scenario.py reads; quiet unless echo."""
    with contextlib.nullcontext() if echo else contextlib.redirect_stdout(io.StringIO()):
        session = _session()
    m2_fit, m3_fit = session["m2_fit"], session["m3_fit"]
    return {
        "ladder": list(session["servo"].keys()),
        "fallback": [model.key for model in session["m3"].fallback()],
        "alias": session["m3"].canonical("frictionloss_nm"),
        "m1_at_1": session["m1"].predict(1.0),
        "fits": {fit.model.key: {"params": fit.params, "rms": fit.rms} for fit in (m2_fit, m3_fit)},
        "chosen": session["verdict"].chosen,
    }


if __name__ == "__main__":
    run(echo=True)
