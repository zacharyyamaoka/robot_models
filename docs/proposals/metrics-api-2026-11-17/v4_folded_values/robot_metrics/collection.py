"""Runs, aliases, the collection (compute groups), strata summaries and OpenSLO-style gates, all over value states.

    run = Run.from_fixture(r)                                  # the run folder's loader: named robot_msgs streams + links
    part = Collection(aliases)                                 # one state per (signal, metric); dwell metrics share one
    state = part.fold(run)                                     # offline; live: state = part.update(state, chunk) ...
    part.values(state)                                         # {alias: display value}
    strata = summarize(part, runs, by=("station", "rung"))     # per stratum: merged states, per-run values, tails
    Objective("tip stop error", "le", 1.0, share=1.0, on="items", owner="Zach")   # "100 % of stops <= 1 mm"

WHY OpenSLO's occurrence form for gates (op, value, target share; the prior-art note §1): "99 % of stops under 1 mm"
says what the scorecard's median rule hides (CAN 11's tail), and a gate reads a tail per stratum (station x rung),
never a pooled IQM. A gate's verdict is RobotDx's own ``Result`` (core.py loaded from io-diagnostics, not copied):
judge, code, view hint and evidence come with it, and it converts to RobotDx's record v0.2 unchanged.
"""

from __future__ import annotations

import importlib.util
import math
import statistics
from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from .core import Metric
from .metrics import Dwell

ROBOTDX_CORE = "/home/bam/io-diagnostics/robotdx/diagnostics/core.py"


def _robotdx_core() -> Any:
    spec = importlib.util.spec_from_file_location("robotdx_core", ROBOTDX_CORE)
    module = importlib.util.module_from_spec(spec)
    import sys

    sys.modules["robotdx_core"] = module       # a dataclass module must be importable by name
    spec.loader.exec_module(module)
    return module


rdx = _robotdx_core()


@dataclass(frozen=True)
class Run:
    """One run, as the run-data MOC recommends: a stable id, named robot_msgs streams, labels, provenance (real | sim |
    replay: carried even though no metric reads it), events and an outcome, and links out (the .rrd, the W&B run)."""

    id: str
    streams: Mapping[str, Any]
    labels: Mapping[str, Any]
    provenance: str = "real"
    outcome: str = "complete"
    links: Mapping[str, str] = field(default_factory=dict)

    @classmethod
    def from_fixture(cls, run: Any) -> Run:
        streams = {"target": run.target, "state": run.measured}
        if run.plan is not None:
            streams["plan"] = run.plan
        if run.tool_target is not None:
            streams.update({"tool/target": run.tool_target, "tool/state": run.tool_measured})
        return cls(run.id, streams, {"station": run.station, **run.labels}, provenance=run.source,
                   outcome="faulted" if run.faulted else "complete",
                   links={"rrd": f"http://127.0.0.1:8771/f/<token>/{run.id.rsplit('/', 1)[-1]}"})

    def chunks(self, rows: int) -> list[Run]:
        state = self.streams["state"]
        edges = state.t[rows - 1::rows].tolist()
        if not edges or edges[-1] < state.t[-1]:
            edges.append(float(state.t[-1]))
        out, low = [], -math.inf
        for high in edges:
            out.append(replace(self, streams={n: s[np.flatnonzero((s.t > low) & (s.t <= high))]
                                              for n, s in self.streams.items()}))
            low = high
        return out


@dataclass(frozen=True)
class Alias:
    name: str
    metric: Metric
    signal: tuple[str, str] = ("target", "state")
    unit: str = ""
    factor: float = 1.0
    across: str = "pool"           # pool (merge the stratum's states) | median | max (of its runs' own values)

    def card(self) -> dict[str, Any]:
        return {"name": self.name, "unit": self.unit, "signal": "/".join(self.signal), "target": None,
                "function": f"robot_metrics.{type(self.metric).__name__}.__call__", **self.metric.card()}


class Collection:
    """A part's aliases over value states: one state per (signal, metric), dwell metrics share one per (signal, rule)."""

    def __init__(self, aliases: Sequence[Alias]) -> None:
        self.aliases = {alias.name: alias for alias in aliases}
        self.groups: dict[tuple, Metric] = {}       # state key -> the metric that updates it
        self.key_of: dict[str, tuple] = {}
        for alias in aliases:
            metric = alias.metric
            key = (alias.signal, metric.rule) if isinstance(metric, Dwell) else (alias.signal, metric)
            self.groups.setdefault(key, metric)
            self.key_of[alias.name] = key

    @property
    def computations(self) -> int:
        return len(self.groups)

    def empty(self) -> dict:
        return {key: metric.empty() for key, metric in self.groups.items()}

    def update(self, state: dict, run: Run) -> dict:
        out = dict(state)
        for (signal, _), metric in self.groups.items():
            target, measured = run.streams[signal[0]], run.streams[signal[1]]
            if len(measured):
                out[(signal, _)] = metric.update(state[(signal, _)], target, measured)
        return out

    def close(self, state: dict) -> dict:
        return {key: self.groups[key].close(value) for key, value in state.items()}

    def merge(self, a: dict, b: dict) -> dict:
        return {key: self.groups[key].merge(a[key], b[key]) for key in a}

    def fold(self, run: Run) -> dict:
        return self.close(self.update(self.empty(), run))

    def value(self, state: dict, name: str) -> Any:
        alias = self.aliases[name]
        value = alias.metric.compute(state[self.key_of[name]])
        return value if value is None or isinstance(value, bool) else float(value) * alias.factor

    def items(self, state: dict, name: str) -> list[float]:
        alias = self.aliases[name]
        return [item * alias.factor for item in alias.metric.items(state[self.key_of[name]])]

    def values(self, state: dict) -> dict[str, Any]:
        return {name: self.value(state, name) for name in self.aliases}


@dataclass
class Stratum:
    key: tuple
    runs: list[Run] = field(default_factory=list)
    states: list[dict] = field(default_factory=list)


def summarize(collection: Collection, runs: Sequence[Run], by: Sequence[str]) -> dict[tuple, dict[str, Any]]:
    """Per stratum: each alias by its own `across` rule (merged states, or its runs' values), plus the run count."""

    strata: dict[tuple, Stratum] = {}
    for run in runs:
        key = tuple(run.labels[name] for name in by)
        stratum = strata.setdefault(key, Stratum(key))
        stratum.runs.append(run)
        stratum.states.append(collection.fold(run))
    out = {}
    for key, stratum in strata.items():
        merged = stratum.states[0]
        for state in stratum.states[1:]:
            merged = collection.merge(merged, state)
        values = {}
        for name, alias in collection.aliases.items():
            if alias.across == "pool":
                values[name] = collection.value(merged, name)
            else:
                per_run = [v for v in (collection.value(state, name) for state in stratum.states) if v is not None]
                values[name] = ({"median": statistics.median, "max": max}[alias.across](per_run) if per_run else None)
        values["faults"] = sum(run.outcome != "complete" for run in stratum.runs)
        out[key] = {"values": values, "merged": merged, "runs": stratum.runs, "states": stratum.states}
    return out


def wilson(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """The Wilson score interval for a share (a bool metric over runs), 95 % by default."""

    if trials == 0:
        return (0.0, 1.0)
    share = successes / trials
    centre = (share + z * z / (2 * trials)) / (1 + z * z / trials)
    half = z * math.sqrt(share * (1 - share) / trials + z * z / (4 * trials * trials)) / (1 + z * z / trials)
    return (centre - half, centre + half)


@dataclass(frozen=True)
class Objective:
    """An OpenSLO occurrence objective on an alias: ``share`` of its items (on="items": every stop, every row) or its
    value (on="value") must satisfy ``op value``. The target never enters compute()."""

    alias: str
    op: str             # le | lt | ge
    value: float
    share: float = 1.0
    on: str = "items"
    owner: str = "proposed"

    def judge(self, collection: Collection, stratum: dict) -> Any:
        passes = {"le": lambda x: x <= self.value, "lt": lambda x: x < self.value, "ge": lambda x: x >= self.value}[self.op]
        unit = collection.aliases[self.alias].unit if self.alias in collection.aliases else ""
        title = f"{self.alias} {self.op} {self.value:g} {unit}".strip()
        if self.on == "value":
            value = stratum["values"].get(self.alias)
            limits = rdx.Limits.at_most(self.value) if self.op in ("le", "lt") else rdx.Limits.at_least(self.value)
            return rdx.result({"objective": title, "owner": self.owner}, value=value, unit=unit, metric="value",
                              limits=limits, title=title, message=f"{self.alias} {value!r} ({self.owner})",
                              hint="threshold_band")
        items = collection.items(stratum["merged"], self.alias)
        ok = sum(passes(item) for item in items)
        observed = ok / len(items) if items else None
        verdict = None if items else rdx.NOT_RUN
        return rdx.result({"objective": f"{self.share:.0%} of {title}", "owner": self.owner}, value=observed, unit="1",
                          metric="share", limits=rdx.Limits.at_least(self.share), verdict=verdict,
                          code="NoItems" if not items else None,
                          title=f"{self.share:.0%} of {title}", message=f"{ok} of {len(items)} pass ({self.owner})",
                          hint="distribution")
