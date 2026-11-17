"""The ideal actuator (M0.ideal): the reference backend, stdlib only. robotparts/2.5, SCHEMA2.md §4.3.2.

torque(terms, inputs, state=None, dt=None) -> (tau_r, state). terms are SI values by name (a fit's terms converted
with the units its definition declares), inputs the model.json inputs in SI. tau_r is the resisting torque at the
output: the joint feels -tau_r. sign(0) is 0, so a model at rest resists nothing here; a backend that sticks at rest
(MuJoCo's frictionloss) does that part natively.
"""

import math

def torque(terms, inputs, state=None, dt=None):
    # WHY nothing: M0 is the ablation baseline; its inertia is the joint's armature, not a torque.
    return 0.0, dict(state or {})
