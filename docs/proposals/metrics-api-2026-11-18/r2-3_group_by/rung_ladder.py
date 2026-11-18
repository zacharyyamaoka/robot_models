"""One helper the session uses: the highest rung each station reaches with every rung up to it passing."""

from __future__ import annotations

from runs_table import Table


def highest_rung(table: Table, *passes: str) -> dict[str, float]:
    """Per station, the highest rung with every rung up to it True in each ``passes`` column (0.0: the first failed)."""

    reached: dict[str, float] = {}
    stopped: set[str] = set()
    for row in table.sort("station", "rung").rows():
        station = row["station"]
        reached.setdefault(station, 0.0)
        if station in stopped:
            continue
        if all(row[name] for name in passes):
            reached[station] = row["rung"]
        else:
            stopped.add(station)
    return reached
