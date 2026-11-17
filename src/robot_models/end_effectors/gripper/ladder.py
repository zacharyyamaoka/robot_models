"""A gripper: an end effector that grasps, on a chain's flange. Its ladder of models (PEP 0004).

Spec: PEP 0004 §5.11 and §6.10.

The climb: M0 places a rigid tool at the tool centre point; M1 maps the width command to the finger opening; M2 adds
the grip force and the contact friction that holds an object; M2.wear lets that friction change with grasp cycles. M6
and M7 are the learned rungs every type has.
"""

from __future__ import annotations

from robot_models.ladder import Alias, Ladder, Parameter, Rung, Wear
from robot_models.references import (
    ARCHARD_1953,
    HWANGBO_2019,
    LU_MEEKER_1993,
    MUJOCO_CONTACT,
    NGUYEN_TUONG_2010,
    PRATTICHIZZO_2016,
    SICILIANO_2009,
)

# --- the vocabulary, grouped by the rung that first reads it ---------------------------------------------------------

VOCABULARY = (
    # M0.tcp
    Parameter(
        name="tcp_offset",
        unit="m",
        shape="pose",
        definition="The tool centre point's offset from the flange the gripper mounts on.",
    ),
    # M1.width_map
    Parameter(
        name="finger_width_map",
        unit="m",
        shape="map",
        definition="Finger opening as a function of the width command.",
    ),
    # M2.contact
    # WHY each pad's normal force, not the jaws' sum: M2.contact's cone is per contact, so the force it reads is the
    # one at each pad; a datasheet that sums the jaws' forces states twice it, and read as one pad's it would make
    # the rung hold twice what it can.
    Parameter(
        name="grip_force_map",
        unit="N",
        shape="map",
        definition=("The normal force each finger pad presses the object with, as a function of the command and the "
                    "width: what a gauge squeezed between two jaws reads. A datasheet's gripping force summed over "
                    "the jaws is twice it on a two-jaw gripper."),
    ),
    Parameter(
        name="contact_friction",
        unit="",
        definition="Coefficient of friction between the finger pads and the object.",
    ),
    # M2.wear
    Parameter(
        name="contact_friction_change",
        unit="1/cycle",
        definition="How much contact_friction changes per grasp cycle (pad wear and glazing).",
    ),
)

# --- the rungs, M0 upward --------------------------------------------------------------------------------------------

RUNGS = (
    Rung(
        level=0,
        name="tcp",
        title="A rigid gripper at the tool centre point",
        base=None,
        adds=("tcp_offset",),
        equation="tool = flange · tcp_offset",
        references=(SICILIANO_2009,),
    ),
    Rung(
        level=1,
        name="width_map",
        title="Finger width map",
        base="M0.tcp",
        adds=("finger_width_map",),
        equation="width = finger_width_map(command)",
        references=(PRATTICHIZZO_2016,),
    ),
    Rung(
        level=2,
        name="contact",
        title="Grip force and contact friction",
        base="M1.width_map",
        adds=("grip_force_map", "contact_friction"),
        # WHY the Coulomb friction cone as the holding test: a pad grips while the tangential load stays inside it
        # (PEP 0004 §5.11); MUJOCO_CONTACT is the simulator's own statement of the same cone. WHY per contact: each
        # pad carries its own cone, so two pads hold about twice what one does.
        equation=("force = grip_force_map(command, width), each pad's normal force; the object holds while, at "
                  "each contact, |tangential| ≤ contact_friction·force (two pads hold about twice one pad's)"),
        references=(PRATTICHIZZO_2016, MUJOCO_CONTACT),
    ),
    Rung(
        level=2,
        name="wear",
        title="Finger pads that glaze with grasps",
        base="M2.contact",
        adds=("contact_friction_change",),
        wears=(
            Wear(
                parameter="contact_friction",
                usage="grasp cycles",
                usage_unit="cycle",
                law="linear",
                rate="contact_friction_change",
            ),
        ),
        equation="contact_friction(n) = contact_friction + contact_friction_change·n, n the grasp cycles so far",
        references=(ARCHARD_1953, LU_MEEKER_1993),
    ),
    Rung(
        level=6,
        name="residual",
        title="Grey-box residual on M2.wear",
        base="M2.wear",
        kind="residual",
        equation=("M2.wear + an MLP residual width from 3 samples of width command, measured width and finger "
                  "current"),
        references=(NGUYEN_TUONG_2010,),
    ),
    Rung(
        level=7,
        name="learned",
        title="Black-box network",
        base=None,
        kind="learned",
        equation="an MLP over 5 samples of width command, width and finger current → finger force",
        references=(HWANGBO_2019,),
    ),
)

# --- the same quantities under other names ---------------------------------------------------------------------------

ALIASES = (
    Alias(name="contact_friction_change_per_cycle", canonical="contact_friction_change"),
)

LADDER = Ladder(
    type="gripper",
    title="An end effector that grasps, on a chain's flange.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=ALIASES,
)
