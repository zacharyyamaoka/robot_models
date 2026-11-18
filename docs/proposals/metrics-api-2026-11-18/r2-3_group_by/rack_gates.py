"""The 1-DOF rack's gates: thresholds with owners, compared with a finished table, never inside a metric.

    judged = rack_gates.judge(runs.group_by("station", "rung").agg(*pendulum_arm.metrics))
    judged.filter(station="can11")       # three True / False columns beside the numbers they judge

``tracks`` and ``median_stop_ok`` are the round-8 scorecard's two verdicts; ``every_stop_ok`` is Zach's 1 mm applied
to the worst stop, which round 1 proposed.
"""

from __future__ import annotations

from runs_table import Table, Threshold

AGENT = "agent (pendulum_goal)"
ZACH = "Zach, 2026-11-11"

tracking_rms = Threshold(0.5, "deg", owner=AGENT)
tracking_peak = Threshold(1.5, "deg", owner=AGENT)
limited_rows = Threshold(2.0, "%", owner=AGENT)
tip_stop = Threshold(1.0, "mm", owner=ZACH)
tip_settle = Threshold(0.3, "s", owner=AGENT)
every_stop = Threshold(1.0, "mm", owner="proposed: Zach's 1 mm, on every stop")


def judge(table: Table) -> Table:
    """The table with three pass / fail columns added; the numbers stay as they were."""

    tracks = ((table["tracking RMS, median over runs (deg)"] <= tracking_rms)
              & (table["worst joint error (deg)"] <= tracking_peak)
              & (table["limited row share (%)"] <= limited_rows)
              & ~table["any faulted run"])
    median_stop_ok = (table["median tip stop (mm)"] <= tip_stop) & (table["median tip settle (s)"] <= tip_settle)
    every_stop_ok = table["worst tip stop (mm)"] <= every_stop
    return table.with_columns(tracks=tracks, median_stop_ok=median_stop_ok, every_stop_ok=every_stop_ok)
