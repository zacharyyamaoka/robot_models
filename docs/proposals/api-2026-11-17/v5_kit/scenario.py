"""v5 Type kit: the round's shared scenario, typed as a user would at the REPL.

    /home/bam/robot_models/.venv/bin/python scenario.py      # prints the session (kept as scenario.out.txt)

SESSION below is the whole user-facing code; `run()` replays it (quietly for check_scenario.py) and returns what the
check grades. The replay is a ten-line echo REPL, so the transcript is exactly what the user typed and saw.
"""

from __future__ import annotations

import ast
import contextlib
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]   # type_kit here, the round's fixture one folder up

SESSION = '''
# 1a. Name the type and take its kit: its models, datasheet class, experiments and fidelity metric in one object
import numpy as np
from type_kit import types
from fixture import datasheet, rig_log
servo = types.of(datasheet())       # Hugging Face AutoModel's move: the datasheet's class names its type
servo                               # types.integrated_servo is the same kit

# 1b. Its models are its ladder; each names the rig motion that identifies what it adds (robotparts' measured_by)
print(servo.models)

# 2. Pick M3.stribeck: what it needs in what unit, the experiment identifying each, equation, references, fallback
stribeck = servo.models["M3.stribeck"]
print(stribeck)
[model.key for model in stribeck.fallback]
servo.canonical("frictionloss_nm")  # one vocabulary per type: the rack twin's name resolves to the type's own

# 3. The datasheet model: M1.datasheet holding the stated values in SI, then its resisting torque at 1 rad/s
m1 = servo.from_datasheet(datasheet())
{name: m1.values[name] for name in ("coulomb_friction", "viscous_friction", "max_output_speed")}
m1.predict(1.0)

# 4. The rig log: which of the kit's experiments it is, then every model the log and the datasheet identify
log = rig_log()
servo.experiments.covered_by(log)
found = servo.identify(log, datasheet=datasheet())
print(found)

# 5. Compare with the type's fidelity metric (RMS over the rig's own plateau scatter); choose the rung that pays
comparison = servo.compare(found, log)
print(comparison)

# 6. What now: hand the chosen model on (SI, numpy in and out); the kit names the motion that unlocks the next rung
chosen = comparison.chosen
dict(chosen.values)
chosen.predict(np.array([-1.0, -0.1, 0.1, 1.0]))
found.missing["M3.lugre"]
print(servo.experiments["small_reversals"])
'''


def replay(echo: bool) -> dict:
    """Run SESSION block by block, statement by statement, echoing each like the REPL when asked."""
    namespace: dict = {}
    for block in SESSION.strip().split("\n\n"):
        header, *code = block.splitlines()
        if echo:
            print(f"\n{header}")
        source = "\n".join(code)
        for node in ast.parse(source).body:
            if echo:
                print("\n".join(("... " if i else ">>> ") + line
                                for i, line in enumerate(code[node.lineno - 1:node.end_lineno])))
            if isinstance(node, ast.Expr):
                value = eval(compile(ast.Expression(node.value), "<session>", "eval"), namespace)
                if echo and value is not None:
                    print(repr(value))
            else:
                exec(compile(ast.Module([node], []), "<session>", "exec"), namespace)
    return namespace


def run() -> dict:
    with contextlib.redirect_stdout(io.StringIO()):     # the check wants the values, not the transcript
        session = replay(echo=False)
    found, comparison = session["found"], session["comparison"]
    rms = {row.key: row.error for row in comparison.rows}
    fitted = {key: {"params": {name: value for name, value in found[key].values.items()
                               if found[key].sources[name] != "datasheet"}, "rms": rms[key]}
              for key in ("M2.coulomb_viscous", "M3.stribeck")}
    return {
        "ladder": list(session["servo"].models),
        "fallback": [model.key for model in session["stribeck"].fallback],
        "alias": session["servo"].canonical("frictionloss_nm"),
        "m1_at_1": session["m1"].predict(1.0),
        "fits": fitted,
        "chosen": comparison.chosen.key,
    }


if __name__ == "__main__":
    import numpy

    numpy.set_printoptions(precision=4)
    replay(echo=True)
