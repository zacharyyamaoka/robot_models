// Rotor ripple map (M4.ripple): the reference backend as an ES module, the same equation as model.py.
// torque(terms, inputs, state, dt) -> {tau, state}; terms and inputs in SI; the joint feels -tau.

const sign = (x) => (x > 0) - (x < 0);

export function torque(terms, inputs, state = {}, dt = null) {
  const v = inputs.v, theta = inputs.theta_rotor;
  let tau = 0.0;
  for (const h of terms.harmonics) tau += h.amplitude * Math.sin(h.rotor_harmonic * theta + h.phase);
  tau += (terms.coulomb_friction ?? 0.0) * sign(v) + (terms.viscous_friction ?? 0.0) * v;
  return { tau, state: { ...state } };
}

// The level the friction holds at rest: the exact limit at v -> 0+ (see model.py's static_level).
export function static_level(terms, inputs = {}, state = {}) {
  return terms.coulomb_friction ?? 0.0;
}
