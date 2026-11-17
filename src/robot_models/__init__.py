"""robot_models -- physical models and parameters of robot parts, the twin of robot_msgs.

Layering (Robot Models Layering decision, 2026-11-12)::

    robot_msgs <- robot_models <- robot_description <- robotparts;  sim2real on top

``robot_models.homing`` (docs/peps/0001-homing-stop-core.md), ``robot_models.datasheet`` and the type tree
``robot_models.actuators.servo`` (docs/peps/0003-typed-datasheets-from-sources.md). The package imports numpy
and robot_msgs only (the datasheet and the types: the stdlib only); mesh contact is the ``contact`` extra.
"""

__version__ = "0.1.0"
