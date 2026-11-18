"""Aliases, the recording they read, and ``MetricCollection``: a part's metrics, each computation done once.

    Alias("tip stop error", DwellError(statistic="median"), signal="joint", unit="mm", factor=1e3 * tip)
    collection = MetricCollection(part.metrics, part.signals)
    collection.update(recording)            # or each chunk of it, live
    collection.end_run(); collection.compute()  -> {"tip stop error": 0.29, ...}   (display units)
    collection.computations                 # how many distinct states the last update fed (the dedup counter)

An alias is only a label: a display name, a display unit and factor, the signal it reads, how runs combine. Its
identity is (base class, frozen config, signal); aliases that share it share ONE metric. Dwell metrics with the same
dwell rule on the same signal also share one ``Dwells`` state (torchmetrics' compute groups), so "tip stop error",
"T4 stop error p90", the settle time and the overshoot are one pass over the stops.
"""

from __future__ import annotations

import copy
import math
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np
from robot_msgs.std.header import Header
from robot_msgs.std.metrics import Metrics

from .base import DwellMetric, PerRun
from .metric import Metric

DEGREES = 180.0 / math.pi


@dataclass(frozen=True)
class RunRecording:
    """One episode (Zach: "do the run, then pass in the Run Recording"): robot_msgs streams by name, the labels a
    roll-up groups by, and the run's ``.rrd`` for the drill-down (a pointer: the heavy data stays in its file)."""

    id: str
    streams: Mapping[str, Any]          # "target", "state", "plan", "tool/target", "tool/state": robot_msgs trajectories
    labels: Mapping[str, Any]           # deployment, station, rung, kind, source (real | sim)
    rrd: str | None = None
    faulted: bool = False

    @classmethod
    def from_run(cls, run: Any) -> RunRecording:
        streams = {"target": run.target, "state": run.measured}
        if run.plan is not None:
            streams["plan"] = run.plan
        if run.tool_target is not None:
            streams.update({"tool/target": run.tool_target, "tool/state": run.tool_measured})
        labels = {"deployment": run.deployment, "station": run.station, "source": run.source, **run.labels}
        return cls(run.id, streams, labels, faulted=run.faulted)

    def chunks(self, rows: int) -> Iterator[RunRecording]:
        """The recording as a live stream would deliver it: ``rows`` state samples at a time, every other stream cut
        at the same instants (a sample belongs to the chunk whose time window holds it)."""

        state = self.streams["state"]
        edges = [*state.t[rows - 1::rows].tolist()]
        if not edges or edges[-1] < state.t[-1]:
            edges.append(float(state.t[-1]))
        low = -math.inf
        for high in edges:
            part = {}
            for name, stream in self.streams.items():
                keep = (stream.t > low) & (stream.t <= high)
                part[name] = stream[np.flatnonzero(keep)]
            low = high
            yield replace(self, streams=part)


@dataclass(frozen=True)
class Alias:
    name: str
    metric: Metric
    signal: str = "joint"
    unit: str = ""                 # display unit; the base math stays in the signal's SI unit
    factor: float = 1.0            # SI -> display: DEGREES, 1e3 x tip for millimetres at a pendulum's tip, 1e3 for ms
    across_runs: str = "pool"      # pool: merge the states (a pooled median of every stop) | median | max: per run first
    owner: str = ""                # who chose the definition (the threshold's owner is the Diagnostic's)
    note: str = ""

    def key(self) -> tuple:
        wrapper = () if self.across_runs == "pool" else ("PerRun", self.across_runs)
        return (self.signal, self.metric.identity, wrapper)

    def card(self) -> dict[str, Any]:
        card = self.metric.card()
        return {"name": self.name, "base": card["base"], "config": card["config"], "unit": self.unit,
                "signal": self.signal, "across_runs": self.across_runs, "description": card["description"],
                "references": card["references"], "function": f"robot_metrics.{card['base']}.compute",
                "target": None, "value_type": card["value_type"]}


class MetricCollection:
    def __init__(self, aliases: Sequence[Alias], signals: Mapping[str, tuple[str, str]]) -> None:
        self.aliases = {alias.name: alias for alias in aliases}
        self.signals = dict(signals)
        self.members: dict[tuple, Metric] = {}
        self._groups: dict[tuple, list[Metric]] = {}  # (signal, dwell key) -> the dwell metrics sharing one state
        for alias in aliases:
            if alias.signal not in self.signals:
                raise KeyError(f"{alias.name}: no signal {alias.signal!r}; the part has {sorted(self.signals)}")
            key = alias.key()
            if key in self.members:
                continue
            metric = alias.metric.clone()
            member = metric if alias.across_runs == "pool" else PerRun(metric, reduce=alias.across_runs)
            self.members[key] = member
            if isinstance(metric, DwellMetric) and alias.across_runs == "pool":
                group = self._groups.setdefault((alias.signal, metric.dwell_key), [])
                if group:
                    metric.dwells = group[0].dwells   # WHY shared: one pass over the stops serves every dwell metric
                group.append(metric)
        self.computations = 0

    def _states(self) -> list[tuple[str, Metric]]:
        """Each distinct state once: a dwell group by its first member, every other member by itself."""

        seen, out = set(), []
        for key, member in self.members.items():
            state = id(member.dwells) if isinstance(member, DwellMetric) else id(member)
            if state not in seen:
                seen.add(state)
                out.append((key[0], member))
        return out

    def update(self, recording: RunRecording) -> None:
        states = self._states()
        for signal, member in states:
            target_name, measured_name = self.signals[signal]
            target, measured = recording.streams[target_name], recording.streams[measured_name]
            if len(measured):
                member.update(target, measured)
        self.computations = len(states)

    def end_run(self) -> None:
        for member in self.members.values():
            member.end_run()

    def compute(self) -> dict[str, Any]:
        out = {}
        for name, alias in self.aliases.items():
            value = self.members[alias.key()].compute()
            out[name] = value if (value is None or isinstance(value, bool)) else float(value) * alias.factor
        return out

    def __call__(self, recording: RunRecording) -> dict[str, Any]:
        """One recording, offline: update, end the run, compute."""

        self.update(recording)
        self.end_run()
        return self.compute()

    def reset(self) -> None:
        for member in self.members.values():
            member.reset()
        for group in self._groups.values():
            for metric in group[1:]:
                metric.dwells = group[0].dwells

    def clone(self) -> MetricCollection:
        return MetricCollection(list(self.aliases.values()), self.signals)

    def merge_state(self, other: MetricCollection) -> MetricCollection:
        done = set()
        for key, member in self.members.items():
            state = id(member.dwells) if isinstance(member, DwellMetric) else id(member)
            if state in done:
                continue
            done.add(state)
            member.merge_state(other.members[key])
        return self

    def to_msg(self, path: str, stamp: float = 0.0) -> Metrics:
        """The values as a robot_msgs ``Metrics`` message (display units in the names): robot_msgs in, robot_msgs out."""

        values = {f"{name} [{alias.unit}]" if alias.unit else name: (np.nan if v is None else float(v))
                  for (name, alias), v in zip(self.aliases.items(), self.compute().values())}
        return Metrics.from_mapping(values, header=Header(stamp=stamp), path=path)

    def __repr__(self) -> str:
        return f"MetricCollection({len(self.aliases)} aliases over {len(self._states())} computations)"
