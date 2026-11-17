// Stribeck friction (M3.stribeck): the reference backend as an ES module, the same equation as model.py.
// torque(terms, inputs, state, dt) -> {tau, state}; terms and inputs in SI; the joint feels -tau.

const sign = (x) => (x > 0) - (x < 0);

export function torque(terms, inputs, state = {}, dt = null) {
  const v = inputs.v;
  const fc = terms.coulomb_friction, fs = terms.breakaway_torque, vs = terms.stribeck_velocity;
  const d = terms.stribeck_exponent ?? 2.0;
  const level = fc + (fs - fc) * Math.exp(-((Math.abs(v) / vs) ** d));
  return { tau: level * sign(v) + terms.viscous_friction * v, state: { ...state } };
}

// The level the friction holds at rest: the exact limit at v -> 0+ (see model.py's static_level).
export function static_level(terms, inputs = {}, state = {}) {
  return terms.breakaway_torque;
}
