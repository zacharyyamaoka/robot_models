"""The hidden ground truth behind fixture.rig_log() (feedback_sim_ground_truth_first). Only fixture.py and
check_scenario.py may import this file; a variant that reads it fails hard gate G3."""

TRUTH = {
    "coulomb_friction": 0.42,     # N·m
    "breakaway_torque": 0.65,     # N·m
    "stribeck_velocity": 0.25,    # rad/s
    "viscous_friction": 0.035,    # N·m·s/rad
}
NOISE_STD = 0.004                 # N·m
SEED = 20261117
