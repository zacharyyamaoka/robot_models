// LuGre friction (M3.lugre): the reference backend as an ES module, the same equation as model.py.
// torque(terms, inputs, state, dt) -> {tau, state}; terms and inputs in SI; the joint feels -tau.

function g(terms, v) {
  const fc = terms.coulomb_friction, fs = terms.breakaway_torque, vs = terms.stribeck_velocity;
  const d = terms.stribeck_exponent ?? 2.0;
  return fc + (fs - fc) * Math.exp(-((Math.abs(v) / vs) ** d));
}

export function torque(terms, inputs, state = {}, dt = null) {
  const v = inputs.v;
  const s0 = terms.sigma0, s1 = terms.sigma1, s2 = terms.sigma2;
  const z = state.z ?? 0.0;
  const gv = g(terms, v);
  let zNext, zdot;
  if (!dt) {
    zNext = (gv * ((v > 0) - (v < 0))) / s0;
    zdot = 0.0;
  } else {
    zNext = (z + v * dt) / (1.0 + (s0 * Math.abs(v) * dt) / gv);
    zdot = (zNext - z) / dt;
  }
  return { tau: s0 * zNext + s1 * zdot + s2 * v, state: { z: zNext } };
}
