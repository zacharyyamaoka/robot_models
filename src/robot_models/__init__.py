"""robot_models -- physical models and parameters of robot parts, the twin of robot_msgs.

Layering (Robot Models Layering decision, 2026-11-12)::

    robot_msgs <- robot_models <- robot_description <- robotparts;  sim2real on top

Only ``robot_models.homing`` exists so far (docs/peps/0001-homing-stop-core.md). The package
imports numpy and robot_msgs only; mesh contact is the ``contact`` extra.
"""

__version__ = "0.1.0"
