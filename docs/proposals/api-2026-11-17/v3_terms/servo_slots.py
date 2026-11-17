"""integrated_servo as slots (v3): which slot each of today's rungs fills. Everything else is read from v1's ladder.

Seven slots, at most one option each per model: the joint's inertia, its friction law, rotor ripple, the drivetrain
(backlash and compliance), the electrics and heat, wear, and a learned correction. Today's 14 rung keys are the
presets: each is the nearest option per slot down its v1 fallback, and Slots checks that every preset needs, reads
and falls back exactly as v1's rung does.
"""

from composed_terms import Slots
from robot_models.ladder import ladders

SLOT_OF = {
    "ideal": "inertia",
    # WHY datasheet fills friction and is no slot of its own: its physics is the stated Coulomb + viscous, so a
    # separate slot would add that friction a second time under M2's fitted one.
    "datasheet": "friction",
    "coulomb_viscous": "friction",
    "stribeck": "friction",
    "load": "friction",
    "lugre": "friction",
    "weakening_capture": "friction",
    "ripple": "ripple",
    "drivetrain": "drivetrain",
    "dcmotor": "electrics",
    "envelope": "electrics",
    "wear": "wear",
    "residual": "correction",
    "learned": "correction",
}

servo = Slots(ladders()["integrated_servo"], SLOT_OF)
