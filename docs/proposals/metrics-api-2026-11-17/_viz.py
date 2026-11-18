"""The standard views every variant's ``Metric.plot()`` goes through: one ``View`` record, three sinks.

    view = View(title=..., unit=..., hint="distribution", x=[...], y=[...], group=[...], target=1.0)
    fig, ax = plot(view, path="stops.png")        # matplotlib (lazy import; the 'viz' extra)
    wandb_payload(view, prefix="can11/tip_stop")   # what wandb.log / wandb.Table receive (built, never sent here)
    rerun_calls(view, "metrics/can11/tip_stop")    # (entity, archetype, args) for rerun; write_rrd() logs them when
                                                   # rerun-sdk is importable

``hint`` is the data-ui view contract (PEP 0010 §10, RobotDx core.HINTS): the same string tells data-ui which stock
view shows the number, so a Metric carries it instead of each sink guessing. WHY one adapter shared by all five: the
coordinator allows a thin adapter, and a view that differs per variant would make the comparison about plotting.
Nothing here imports matplotlib, wandb or rerun at module load: the runtime core stays numpy + robot_msgs.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

HINTS = ("gate_matrix", "threshold_band", "kpi_trend", "rollup_tree", "evidence", "distribution", "level_history",
         "path_overlay")


@dataclass(frozen=True)
class View:
    """What a metric's result looks like as data: points (x, y) in groups, the unit shown, the target line if any."""

    title: str
    unit: str
    hint: str
    x: Sequence[Any]
    y: Sequence[float]
    group: Sequence[str] = ()
    target: float | None = None
    target_label: str = ""
    x_label: str = ""
    y_label: str = ""
    extra: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.hint not in HINTS:
            raise ValueError(f"hint {self.hint!r} is not a data-ui view: {HINTS}")


def plot(view: View, ax: Any = None, path: str | Path | None = None) -> tuple[Any, Any]:
    """torchmetrics' ``Metric.plot`` contract: returns (fig, ax); draws into ``ax`` when given."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = (ax.figure, ax) if ax is not None else plt.subplots(figsize=(7.2, 3.4), dpi=130)
    groups = list(dict.fromkeys(view.group)) if view.group else [""]
    for name in groups:
        rows = [i for i in range(len(view.y)) if not view.group or view.group[i] == name]
        xs, ys = [view.x[i] for i in rows], [view.y[i] for i in rows]
        if view.hint == "distribution":
            ax.plot(xs, ys, "o", ms=3.5, alpha=0.8, label=name or None)
        else:
            ax.plot(xs, ys, "o-", ms=4, lw=1.4, label=name or None)
    if view.target is not None:
        ax.axhline(view.target, color="#c0392b", lw=1.2, ls="--", label=view.target_label or f"target {view.target:g}")
    ax.set_title(view.title, fontsize=10)
    ax.set_xlabel(view.x_label)
    ax.set_ylabel(view.y_label or view.unit)
    ax.grid(alpha=0.3)
    if len(groups) > 1 or view.target is not None:
        ax.legend(fontsize=7, ncol=min(4, len(groups) + 1), loc="best")
    fig.tight_layout()
    if path is not None:
        fig.savefig(path)
    return fig, ax


def wandb_payload(view: View, prefix: str) -> dict[str, Any]:
    """A wandb.Table of the points and the summary keys (W&B: one project per deployment, one run per scoring)."""

    columns = ["x", view.y_label or view.unit] + (["group"] if view.group else [])
    data = [[view.x[i], float(view.y[i])] + ([view.group[i]] if view.group else []) for i in range(len(view.y))]
    summary = {f"{prefix}/max": max(view.y) if view.y else None, f"{prefix}/n": len(view.y)}
    if view.target is not None:
        summary[f"{prefix}/target"] = view.target
    return {"table": {"columns": columns, "data": data}, "summary": summary,
            "chart": f"wandb.plot.scatter(table, 'x', {columns[1]!r}, title={view.title!r})", "hint": view.hint}


def rerun_calls(view: View, entity: str) -> list[tuple[str, str, dict[str, Any]]]:
    """Rerun log calls: one scalar series per group (time = the point's x), the target as a static series."""

    calls = []
    for i in range(len(view.y)):
        group = view.group[i] if view.group else "value"
        calls.append((f"{entity}/{group}", "Scalars", {"time": float(i), "scalars": [float(view.y[i])]}))
    if view.target is not None:
        calls.append((f"{entity}/target", "Scalars", {"time": 0.0, "scalars": [float(view.target)]}))
    return calls


def write_rrd(calls: list[tuple[str, str, dict[str, Any]]], path: str | Path, application: str = "robot_metrics") -> Path | None:
    """Log the calls to an .rrd when rerun-sdk is importable (the 'rerun' extra); None without it."""

    try:
        import rerun as rr
    except ImportError:
        return None
    rr.init(application, spawn=False)
    rr.save(str(path))
    for entity, archetype, args in calls:
        rr.set_time("index", sequence=int(args["time"]))
        rr.log(entity, getattr(rr, archetype)(args["scalars"]))
    return Path(path)
