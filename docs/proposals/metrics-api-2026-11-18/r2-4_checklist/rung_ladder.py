"""How high each station climbs the speed ladder: the highest rung such that it and every rung below it pass.

    highest_passing_rung(report, ["tracking RMS", "median tip stop"])   # {station: rung}

``report`` is ``card.check(runs, by=["station", "rung"])``. WHY "every rung up to it": a ladder is only as high as its
first failing step (round 8's rule for the highest rung a station tracks).
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any


def highest_passing_rung(report: Any, checks: Sequence[str]) -> dict[str, float]:
    if report.measured.by != ("station", "rung"):
        raise ValueError("highest_passing_rung reads a report grouped by station and rung")
    by_station: dict[str, list[tuple[float, bool]]] = {}
    for (station, rung), passed in report.passing(checks).items():
        by_station.setdefault(station, []).append((rung, passed))
    highest = {}
    for station, rungs in by_station.items():
        reached = 0.0
        for rung, passed in sorted(rungs):
            if not passed:
                break
            reached = rung
        highest[station] = reached
    return highest
