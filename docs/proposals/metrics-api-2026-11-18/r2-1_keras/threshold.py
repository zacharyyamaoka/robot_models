"""A pass/fail line, kept apart from the metrics: the metric measures, the threshold judges.

    gate = Threshold(at_most=1.0, unit="mm", owner="Zach, 2026-11-11")
    gate.passes(0.62)                                              # True
    verdicts = gate.check(table, "tip stop median (mm)", "tip stop worst (mm)")
    print(verdicts)                                                # pass / FAIL per station and rung
    verdicts.passed("tip stop worst (mm)")                         # (33, 36): cells passing, cells judged

WHY outside the metric (Zach's rule: "separate the calculation of the metric, and the checking of thresholds"):
the same number can be judged by several lines with different owners (Zach's 1 mm, an agent's 0.3 s, a proposed
worst-stop rule) without recomputing anything. Keras puts thresholds inside some metrics (``Precision(thresholds=)``);
this proposal does not follow it there.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from rung_table import station_order


@dataclass(frozen=True)
class Threshold:
    """A number passes when it is at most ``at_most``. ``owner`` says who set the line and when."""

    at_most: float
    unit: str = ""
    owner: str = ""        # Zach / agent / proposed, with the date

    def passes(self, value: Any) -> bool:
        return value is not None and math.isfinite(float(value)) and float(value) <= self.at_most

    def check(self, table: Any, *labels: str) -> "Verdicts":
        """Each rung cell of ``table`` judged on each named column."""

        cells = {key: {label: self.passes(row[label]) for label in labels} for key, row in table.rows.items()}
        return Verdicts(self, labels, cells)


@dataclass(frozen=True)
class Verdicts:
    """pass / fail per rung cell and column; ``passed(label)`` counts the cells that pass."""

    threshold: Threshold
    labels: tuple[str, ...]
    cells: dict[tuple[str, float], dict[str, bool]]

    def passed(self, label: str) -> tuple[int, int]:
        return sum(cell[label] for cell in self.cells.values()), len(self.cells)

    def __str__(self) -> str:
        stations = sorted({station for station, _ in self.cells}, key=station_order)
        rungs = sorted({rung for _, rung in self.cells})
        names = " / ".join(label.split(" (")[0] for label in self.labels)
        rule = f"at most {self.threshold.at_most:g} {self.threshold.unit}".rstrip()
        width = 5 * len(self.labels)
        lines = [f"{rule} (owner: {self.threshold.owner}) · each cell: {names}",
                 f"{'rung':<8}" + "".join(f"{rung:>{width + 2}.1f}" for rung in rungs)]
        for station in stations:
            cells = []
            for rung in rungs:
                verdict = self.cells.get((station, rung), {})
                cells.append(" ".join("pass" if verdict.get(label) else "FAIL" for label in self.labels))
            lines.append(f"{station:<8}" + "".join(f"{cell:>{width + 2}}" for cell in cells))
        counts = [f"{label.split(' (')[0]} {passed} of {total}" for label in self.labels
                  for passed, total in [self.passed(label)]]
        lines.append("cells passing: " + " · ".join(counts))
        return "\n".join(lines)
