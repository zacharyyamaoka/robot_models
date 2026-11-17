"""An angle sensor on a shaft, such as a drive's rotor encoder: its ladder of models (PEP 0004), in `ladder.py`.

The type covers any sensor that reports a shaft's angle as a count: it quantises the angle, adds noise and a delay,
and can carry an angle error that repeats once per turn (an eccentric magnet or disc) that grows as its mount loosens.
"""
