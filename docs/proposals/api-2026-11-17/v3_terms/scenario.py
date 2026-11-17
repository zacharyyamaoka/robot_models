"""The shared scenario with v3 "Composed terms": typed as a REPL session, run here statement by statement.

    /home/bam/robot_models/.venv/bin/python scenario.py      # prints the session transcript (scenario.out.txt)

run(case) returns what check_scenario.py reads: `case` is the fixture's opaque servo name, put in the session's
namespace and only ever passed through to rig_log. Each STEPS block is what a user types; the runner echoes it with >>>
and lets expressions print themselves (compile mode "single", as the REPL does), so the transcript is the real output.
The transcript adds CONTROL: steps 4 and 5 again, unchanged, on servo "B" (the negative control).
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
    ("4. Fit M2.coulomb_viscous, then climb one slot to M3.stribeck: both on the fit plateaus only", '''
case   # the servo on the rig: run(case) passes it in; step 6 reruns steps 4 and 5 on "B"
fit_log, test_log = rig_log(case, "fit"), rig_log(case, "test")   # plateaus 1, 3, 5, 7, 9 | 2, 4, 6, 8, 10 held out
m2_fit = servo["M2.coulomb_viscous"].fit(fit_log, start=m1)
m3_fit = m2_fit.model.redeclare(friction="stribeck").fit(fit_log, start=m2_fit)
m2_fit
m3_fit
'''),
    ("5. Does the friction swap pay? Judge on the held-out plateaus: the lowest rung that reaches the noise floor", '''
from composed_terms import compare
verdict = compare(m2_fit, m3_fit, on=test_log)   # rms on plateaus neither fit saw, ÷ the rig's own repeatability
print(verdict)
verdict.chosen
compare(m2_fit, m3_fit, on=fit_log)               # the fit plateaus would grade the optimiser, not the choice
m3_fit.params   # SI values by canonical name, ready for MuJoCo or sim2real
'''),
)

CONTROL = ("6. Servo B, the negative control: steps 4 and 5 unchanged, and the simpler rung wins", '''
case = "B"   # a second servo on the same rig
fit_log, test_log = rig_log(case, "fit"), rig_log(case, "test")
m2_fit = servo["M2.coulomb_viscous"].fit(fit_log, start=m1)
m3_fit = m2_fit.model.redeclare(friction="stribeck").fit(fit_log, start=m2_fit)
print(compare(m2_fit, m3_fit, on=test_log))   # M3 fits its own plateaus better and the held-out ones worse
''')

# WHY only these: the API's own teaching errors print as the REPL shows them; anything else is a bug and raises.
SHOWN_ERRORS = (ValueError, KeyError, NotImplementedError)


def _session(case: str, steps=STEPS) -> dict:
    namespace: dict = {"__name__": "__session__", "case": case}
    for title, block in steps:
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


def run(case: str = "A", echo: bool = False) -> dict:
    """The scenario's results in the shape check_scenario.py reads (steps 1-5 on `case`); quiet unless echo."""
    with contextlib.nullcontext() if echo else contextlib.redirect_stdout(io.StringIO()):
        session = _session(case)
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
    _session("A", (*STEPS, CONTROL))
