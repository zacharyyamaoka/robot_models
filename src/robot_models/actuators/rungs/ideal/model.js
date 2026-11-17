// The ideal actuator (M0.ideal): the reference backend as an ES module, the same equation as model.py.
// torque(terms, inputs, state, dt) -> {tau, state}; terms and inputs in SI; the joint feels -tau.

export function torque(terms, inputs, state = {}, dt = null) {
  return { tau: 0.0, state: { ...state } };
}
