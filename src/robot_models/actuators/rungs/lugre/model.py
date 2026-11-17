"""LuGre friction (M3.lugre): the reference backend, stdlib only. robotparts/2.5, SCHEMA2.md §4.3.2.

torque(terms, inputs, state=None, dt=None) -> (tau_r, state). terms are SI values by name (a fit's terms converted
with the units its definition declares), inputs the model.json inputs in SI. tau_r is the resisting torque at the
output: the joint feels -tau_r. sign(0) is 0, so a model at rest resists nothing here; a backend that sticks at rest
(MuJoCo's frictionloss) does that part natively.
"""

import math

def _g(terms, v):
    fc, fs, vs = terms["coulomb_friction"], terms["breakaway_torque"], terms["stribeck_velocity"]
    d = terms.get("stribeck_exponent", 2.0)
    return fc + (fs - fc) * math.exp(-((abs(v) / vs) ** d))


def torque(terms, inputs, state=None, dt=None):
    v = inputs["v"]
    s0, s1, s2 = terms["sigma0"], terms["sigma1"], terms["sigma2"]
    z = (state or {}).get("z", 0.0)
    g = _g(terms, v)
    if not dt:
        # WHY steady state without a step: dz/dt = 0 gives z = g(v) sign(v) / sigma0, the sliding curve.
        z_next = g * ((v > 0) - (v < 0)) / s0
        zdot = 0.0
    else:
        # WHY semi-implicit: the bristles are stiff (sigma0 ~ 1e2..1e5); an explicit step diverges at a simulator's dt.
        z_next = (z + v * dt) / (1.0 + s0 * abs(v) * dt / g)
        zdot = (z_next - z) / dt
    return s0 * z_next + s1 * zdot + s2 * v, {"z": z_next}
