"""Print a short robot_models API session, each call shown as a >>> line above its real result (the v1 report's capture d).

    /home/bam/robot_models/.venv/bin/python docs/report/api_session.py > <media dir>/term-d-api-session.txt

WHY eval the shown text: the >>> line is the exact expression that produced the result under it, so the capture cannot drift
from the code it claims to show.
"""

from pprint import pformat

from robot_models.ladder import ladders, needs

SESSION = (
    "from robot_models.ladder import ladders, needs",
    "sorted(ladders())",
    "servo = ladders()['integrated_servo']",
    "servo.keys",
    "servo.parameters('M3.stribeck')",
    "servo.fallback('M3.lugre')",
    "servo.canonical('frictionloss_nm')",
    "servo.resolve('thermal_r_c')",
    "needs(ladders(), 'integrated_servo', 'M4.drivetrain')",
)

namespace = {"ladders": ladders, "needs": needs}
for line in SESSION:
    print(">>>", line)
    try:
        value = eval(compile(line, "<session>", "eval"), namespace)
    except SyntaxError:
        exec(compile(line, "<session>", "exec"), namespace)
        continue
    print(pformat(value, width=110, compact=True))
