"""The test rack's three gates, each a few thresholds read off a rung cell's numbers, never inside a metric.

    GATES["tip stop, median"]          -> {metric label: Threshold}   every threshold must pass
    judge(cells)                       -> {(station, rung): {gate name: True / False}}
    highest_rung(verdicts)             -> {station: the highest rung with every rung up to it tracking and stopping}

The rules are the round-8 scorecard's: "tracks" (agent: RMS <= 0.5 deg, peak <= 1.5 deg, limiter share <= 2 %, no
fault), "tip stop, median" (Zach's 1 mm on the median stop, 2026-11-11, with the agent's 0.3 s median settle), and
"tip stop, every stop" (proposed: Zach's 1 mm held against the worst stop).
"""

from __future__ import annotations

from typing import Any

from rung_table import station_order
from threshold import Threshold

TIP_STOP = Threshold(at_most=1.0, unit="mm", owner="Zach, 2026-11-11")

GATES = {
    "tracks": {
        "tracking RMS (deg)": Threshold(at_most=0.5, unit="deg", owner="agent"),
        "tracking peak (deg)": Threshold(at_most=1.5, unit="deg", owner="agent"),
        "limiter share (%)": Threshold(at_most=2.0, unit="%", owner="agent"),
        "faults": Threshold(at_most=0, owner="agent"),
    },
    "tip stop, median": {
        "tip stop median (mm)": TIP_STOP,
        "tip settle median (s)": Threshold(at_most=0.3, unit="s", owner="agent"),
    },
    "tip stop, every stop": {
        "tip stop worst (mm)": Threshold(at_most=1.0, unit="mm", owner="proposed: Zach's 1 mm on the worst stop"),
    },
}


def judge(cells: dict[tuple[str, float], dict[str, Any]]) -> dict[tuple[str, float], dict[str, bool]]:
    """Each rung cell against each gate: a gate passes when every one of its thresholds passes."""

    verdicts = {}
    for key, numbers in cells.items():
        verdicts[key] = {name: all(threshold.passes(numbers[label]) for label, threshold in gate.items())
                         for name, gate in GATES.items()}
    return verdicts


def highest_rung(verdicts: dict[tuple[str, float], dict[str, bool]]) -> dict[str, float]:
    """Per station, the highest rung such that it and every rung below it pass "tracks" and "tip stop, median"."""

    highest = {}
    for station in sorted({station for station, _ in verdicts}, key=station_order):
        reached = 0.0
        for rung in sorted(rung for name, rung in verdicts if name == station):
            cell = verdicts[(station, rung)]
            if not (cell["tracks"] and cell["tip stop, median"]):
                break
            reached = rung
        highest[station] = reached
    return highest
