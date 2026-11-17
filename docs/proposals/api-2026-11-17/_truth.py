"""The hidden ground truths behind fixture.rig_log() (feedback_sim_ground_truth_first). Only fixture.py and
check_scenario.py may import this file; a variant that reads it fails hard gate G3.

Two cases, named opaquely so a scenario cannot branch on them: one servo whose friction has a Stribeck dip, and a
negative control that is plain Coulomb + viscous (breakaway equal to Coulomb, so the dip vanishes). The control is
what shows a selection rule can pick the SIMPLER rung (amendment 1: a richer rung always wins on its own training
data, because M3.stribeck nests M2.coulomb_viscous).
"""

CASES = {
    "A": ({"coulomb_friction": 0.42, "breakaway_torque": 0.65, "stribeck_velocity": 0.25, "viscous_friction": 0.035},
          20261117),
    "B": ({"coulomb_friction": 0.38, "breakaway_torque": 0.38, "stribeck_velocity": 0.25, "viscous_friction": 0.030},
          20261118),
}
TRUTH = CASES["A"][0]
NOISE_STD = 0.004                 # N·m
