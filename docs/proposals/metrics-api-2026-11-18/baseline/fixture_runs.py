"""Round 1's fixture runs as robot_metrics 0.1.0 ``Run`` records: a field-for-field copy, no computation.

    from fixture import pendulum_runs
    from fixture_runs import as_run
    run = as_run(pendulum_runs("real")[32])     # a robot_metrics.Run: named streams + labels

WHY this file exists: 0.1.0 scores its own ``Run`` (streams named "plan", "state", "target", "tool/target",
"tool/state", plus labels), and round 2's contract loads every variant's data from round 1's ``fixture.py``, whose
``Run`` has fields (``target``, ``measured``, ``plan``, ``station``...). 0.1.0's own landing page reads the same plays
through ``robot_metrics.datasets.onedof_round8.real_runs()``, its port of that fixture; this column reads the shared
fixture instead, so every round-2 glance starts from the same object, and this one line is the only glue it adds.
It renames nothing in robot_metrics and adds no metric.
"""

from __future__ import annotations

from typing import Any

from robot_metrics import Run


def as_run(play: Any) -> Run:
    """One fixture play (or chain rung, or held-out plateau) as 0.1.0's Run: the same streams under 0.1.0's names,
    the station folded into the labels a roll-up groups by, the source as provenance, a fault as the outcome."""

    # WHY this order: 0.1.0's own reader lists plan, state, target, so the Run prints as on its landing page
    streams = {} if play.plan is None else {"plan": play.plan}
    streams.update({"state": play.measured, "target": play.target})
    if play.tool_target is not None:
        streams["tool/target"] = play.tool_target
        streams["tool/state"] = play.tool_measured
    labels = {"deployment": play.deployment, "station": play.station, "part_type": play.part_type, **play.labels}
    return Run(play.id, streams, labels, provenance=play.source,
               outcome="faulted" if play.faulted else "complete")
