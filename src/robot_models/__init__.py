"""robot_models -- generic physical models of robot part types (no maker, no SKU): each type a ladder of
models from ideal to learned, every rung with its parameters, equation and references.

Layering (Robot Models Layering decision, 2026-11-12, in the order PEP 0004 corrected it to)::

    robot_msgs <- robot_models <- robot_parts <- robot_descriptions;  sim2real on top

``robot_models.ladder`` (the type ladders; ``ladders()`` finds each type's ``ladder.py``) and
``robot_models.references`` (the works the rungs cite), docs/peps/0004-model-ladders-by-type.md.
``robot_models.homing`` (docs/peps/0001-homing-stop-core.md), ``robot_models.datasheet`` and the type tree
``robot_models.actuators.servo`` (docs/peps/0003-typed-datasheets-from-sources.md). The package imports numpy
and robot_msgs only (the ladder core, the references, the datasheet and the types: the stdlib only); mesh
contact is the ``contact`` extra.

Each part type's models, a ModelFamily (PEP 0004 amendment 2), loaded on first use::

    from robot_models import IntegratedServo      # class IntegratedServo(ModelFamily): .m3.stribeck, .fit, .tree, ...
"""

# WHY a literal, kept equal to pyproject.toml's version (CI asserts it against the built metadata): reading
# importlib.metadata here costs ~11 ms on every import and returns a stale editable install's number.
__version__ = "0.3.0"

# WHY lazy (PEP 562): a model family runs on numpy, but the ladders and the datasheets import neither numpy nor
# robot_msgs (README), so `import robot_models.ladder` or a robotparts source file must not pay for the types.
_FAMILIES = {
    "ModelFamily": "robot_models.model_family",
    "IntegratedServo": "robot_models.actuators.servo.integrated_servo_family",
}


def __getattr__(name: str):
    if name in _FAMILIES:
        import importlib

        return getattr(importlib.import_module(_FAMILIES[name]), name)
    raise AttributeError(f"module 'robot_models' has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted([*globals(), *_FAMILIES])
