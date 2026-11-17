"""The units a datasheet field accepts, each with its factor to SI, spelled as robotparts' registry spells them.
The first unit of a family is the one a datasheet usually quotes, which a table shows on a row nobody states.

WHY the registry's spellings and factors (PEP 0003): one vocabulary. robotparts' user tests check that every typed
field named like a registry key accepts only units that key lists, with the same factors.

A source's own spelling ("N.M", "RPM", "rpm/v", "Pairs") is normalised to these when it is transcribed; the source's
words stay in `Stated.as_written`.
"""

from __future__ import annotations

import math

RPM = 2 * math.pi / 60

TORQUE = {"N·m": 1.0, "mN·m": 1e-3}
SPEED = {"rpm": RPM, "rad/s": 1.0, "deg/s": math.pi / 180}
SMALL_ANGLE = {"arcmin": math.pi / 10800, "deg": math.pi / 180, "rad": 1.0, "mrad": 1e-3}
VOLTAGE = {"V": 1.0}
CURRENT = {"A": 1.0}
POWER = {"W": 1.0}
MASS = {"kg": 1.0, "g": 1e-3}
LENGTH = {"mm": 1e-3, "m": 1.0}
FORCE = {"N": 1.0}
INERTIA = {"kg·cm²": 1e-4, "kg·m²": 1.0, "kg·mm²": 1e-6, "g·cm²": 1e-7, "g·mm²": 1e-9}
VELOCITY_CONSTANT = {"rpm/V": RPM}
TORQUE_CONSTANT = {"N·m/A": 1.0, "mN·m/A": 1e-3}
# WHY V/krpm to V·s/rad: 1 V per 1000 rpm is 1 / (1000 · 2π/60) V·s/rad
BACK_EMF_CONSTANT = {"V/krpm": 1 / (1000 * RPM), "V·s/rad": 1.0}
RESISTANCE = {"Ω": 1.0, "mΩ": 1e-3}
INDUCTANCE = {"mH": 1e-3, "H": 1.0, "µH": 1e-6}
DAMPING = {"N·m·s/rad": 1.0, "mN·m·s/rad": 1e-3}
RATIO = {":1": 1.0, "": 1.0}
FRACTION = {"%": 0.01, "": 1.0}
COUNT = {"": 1.0}

FAMILIES = (TORQUE, SPEED, SMALL_ANGLE, VOLTAGE, CURRENT, POWER, MASS, LENGTH, FORCE, INERTIA, VELOCITY_CONSTANT,
            TORQUE_CONSTANT, BACK_EMF_CONSTANT, RESISTANCE, INDUCTANCE, DAMPING, RATIO, FRACTION, COUNT)

# Every spelling above, once. A spelling in two families must mean the same factor ("" and ":1" are 1 everywhere;
# "%" is 0.01 only in FRACTION), so one table converts any stated value to SI.
FACTOR: dict[str, float] = {}
for _family in FAMILIES:
    for _unit, _factor in _family.items():
        if _unit in FACTOR and FACTOR[_unit] != _factor:
            raise AssertionError(f"unit {_unit!r} has two factors: {FACTOR[_unit]} and {_factor}")
        FACTOR[_unit] = _factor


def to_si(value: float, unit: str | None) -> float:
    """`value` in `unit`, converted to SI. A unit this module does not know refuses (never a silent factor of 1)."""
    if unit is None:
        unit = ""
    if unit not in FACTOR:
        raise ValueError(f"unknown unit {unit!r}; known: {', '.join(sorted(u for u in FACTOR if u))}")
    return value * FACTOR[unit]
