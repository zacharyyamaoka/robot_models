"""The datasheet (M1.datasheet): the reference backend, stdlib only. robotparts/2.5, SCHEMA2.md §4.3.2.

torque(terms, inputs, state=None, dt=None) -> (tau_r, state). terms are SI values by name (a fit's terms converted
with the units its definition declares), inputs the model.json inputs in SI. tau_r is the resisting torque at the
output: the joint feels -tau_r. sign(0) is 0, so a model at rest resists nothing here; a backend that sticks at rest
(MuJoCo's frictionloss) does that part natively.
"""

import math

def _sign(x):
    return (x > 0) - (x < 0)


def torque(terms, inputs, state=None, dt=None):
    v = inputs["v"]
    return terms.get("coulomb_friction", 0.0) * _sign(v) + terms.get("viscous_friction", 0.0) * v, dict(state or {})


def static_level(terms, inputs, state=None):
    """The level the friction holds at rest: the exact limit of the resisting torque's friction part as v -> 0+
    (judge r3 #2: a backend that sticks natively, MuJoCo's frictionloss, holds this, never a torque sampled at a small
    speed). The state is read, never advanced."""
    return float(terms.get("coulomb_friction", 0.0))
