"""Many runs into one table: one row per station and rung, each row from a fresh set of metrics.

    table = rung_table(runs, pendulum_arm_metrics)    # runs: fixture.pendulum_runs("real")
    print(table.for_station("can11"))                 # CAN 11's six rungs, one column each
    table.rows[("can11", 0.9)]                        # one rung cell's numbers, a plain dict

A rung cell is every run of one station at one speed rung (round 8: 7 plays per cell). The cell's metrics are made
fresh (``make_metrics(tip_length_m=...)``) and fed each of its runs in turn, each as its own run
(``new_run=True``); each metric then combines the runs by its own rule (``across_runs``: the median of the plays'
tracking RMS, every stop pooled for the stop numbers). This is torchmetrics' "one metric per group", written out.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable
from typing import Any


def rung_table(runs: Iterable[Any], make_metrics: Callable[..., Any]) -> "Table":
    """One row per (station, rung), in the order the runs first name them."""

    cells: dict[tuple[str, float], list[Any]] = {}
    for run in runs:
        cells.setdefault((run.station, run.rung), []).append(run)
    rows = {}
    for (station, rung), cell_runs in cells.items():
        metrics = make_metrics(tip_length_m=cell_runs[0].tip_length_m)
        for run in cell_runs:
            metrics.update(run.target, run.measured, plan=run.plan, new_run=True)
        rows[(station, rung)] = metrics.compute()
    return Table(rows)


def shown(value: Any) -> str:
    """A cell as text: 3 decimals, yes / no for a bool, "never" for a settle time that never came, "-" for none."""

    if value is None:
        return "-"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int):
        return str(value)
    if math.isinf(value):
        return "never"
    return f"{value:.3f}"


def station_order(station: str) -> tuple:
    """can1, can11, can12 ... by their number, not as text."""

    digits = "".join(character for character in station if character.isdigit())
    return (int(digits) if digits else 0, station)


class Table:
    """Rows keyed by (station, rung); each row is a plain dict of metric label -> number."""

    def __init__(self, rows: dict[tuple[str, float], dict[str, Any]]) -> None:
        self.rows = rows

    def stations(self) -> list[str]:
        return sorted({station for station, _ in self.rows}, key=station_order)

    def for_station(self, station: str) -> "Table":
        return Table({key: row for key, row in self.rows.items() if key[0] == station})

    def __len__(self) -> int:
        return len(self.rows)

    def __str__(self) -> str:
        """One station: a metric per line and a rung per column. Several: a station and rung per line."""

        if not self.rows:
            return "(no rows)"
        labels = list(next(iter(self.rows.values())))
        width = max(len(label) for label in labels)
        if len(self.stations()) == 1:
            station = self.stations()[0]
            rungs = sorted(rung for _, rung in self.rows)
            lines = [f"{station + ' · rung':<{width}}" + "".join(f"{rung:>8.1f}" for rung in rungs)]
            for label in labels:
                cells = "".join(f"{shown(self.rows[(station, rung)][label]):>8}" for rung in rungs)
                lines.append(f"{label:<{width}}{cells}")
            return "\n".join(lines)
        lines = [f"{'station':<8}{'rung':>5}  " + "  ".join(labels)]
        for (station, rung) in sorted(self.rows, key=lambda key: (station_order(key[0]), key[1])):
            row = self.rows[(station, rung)]
            cells = "  ".join(f"{shown(row[label]):>{len(label)}}" for label in labels)
            lines.append(f"{station:<8}{rung:>5.1f}  {cells}")
        return "\n".join(lines)
