"""The rack's gates: a column of the scorecard table, the most it may be, and who owns that number.

    GATES["tip stop, every stop"]        -> [("worst_tip_stop_mm", 1.0, "proposed: ...")]
    passes(table, GATES["tracks"])       -> [True, False, ...]   one verdict per row of the table
    highest_rung(table, ["tracks", ...]) -> {"can11": 1.0, ...}  per station, the top of its passing rungs

No metric function knows these numbers. A gate reads the table a scorecard returns, so a new target never changes how
anything is measured, and the median and the worst stop are judged side by side.
"""

from __future__ import annotations

from typing import Any

ZACH = "Zach, 2026-11-11"
AGENT = "agent (pendulum_goal)"
PROPOSED = "proposed: Zach's 1 mm on every stop"

# each gate is a list of (column of the table, at most, owner); a row passes when every column is within its limit
GATES = {
    "tracks": [
        ("tracking_rms_deg", 0.5, AGENT),
        ("tracking_peak_deg", 1.5, AGENT),
        ("limited_rows_pct", 2.0, AGENT),
        ("faulted_runs", 0, AGENT),
    ],
    "tip stop, median": [
        ("median_tip_stop_mm", 1.0, ZACH),
        ("tip_settle_s", 0.3, AGENT),
    ],
    "tip stop, every stop": [
        ("worst_tip_stop_mm", 1.0, PROPOSED),
    ],
}


def passes(table: Any, gate: list[tuple[str, float, str]]) -> list[bool]:
    """Per row of the table: True when every column the gate names is at most its limit."""

    verdicts = []
    for row in table:
        verdicts.append(all(row[column] <= at_most for column, at_most, _owner in gate))
    return verdicts


def highest_rung(table: Any, verdict_columns: list[str]) -> dict[str, float]:
    """Per station, the highest rung with every rung up to it passing every named verdict column (0 if none)."""

    highest: dict[str, float] = {}
    for station in sorted({row["station"] for row in table}):
        reached = 0.0
        for row in sorted((row for row in table if row["station"] == station), key=lambda row: row["rung"]):
            if not all(row[column] for column in verdict_columns):
                break
            reached = row["rung"]
        highest[station] = reached
    return highest


def gate_rows() -> list[dict[str, Any]]:
    """Every gate as rows: which gate, which column, at most what, and whose number it is."""

    rows = []
    for name, gate in GATES.items():
        for column, at_most, owner in gate:
            rows.append({"gate": name, "column": column, "at most": at_most, "owner": owner})
    return rows


def target_of(metric_name: str) -> str | None:
    """The gates on one metric, as words (None when the metric is only observed)."""

    found = [f"<= {at_most:g} ({owner}, gate '{name}')" for name, gate in GATES.items()
             for column, at_most, owner in gate if column == metric_name]
    return "; ".join(found) or None
