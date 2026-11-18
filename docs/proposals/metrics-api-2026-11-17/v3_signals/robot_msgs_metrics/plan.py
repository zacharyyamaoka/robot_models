"""Aliases compile into a Plan: each (signal, base, config) once, each stop pass once; the same Plan runs live or offline.

    plan = Plan.compile(aliases, signals={"joint": ("/joint/command", "/joint/state")})
    print(plan)                          # 12 aliases -> 6 computations + 1 stop pass
    for time, topic, message in recording.messages():    # live: one message at a time, arrival order
        plan.push(topic, message)
    plan.finish(); plan.values()         # offline is plan.run(recording): whole topics through the same push

Like a query planner's common-subexpression step (or Dask's graph dedup): the alias table is the query, the plan is
what runs. WHY the plan and not the collection holds the dedup: an alias set is data (the part's catalog), and the
same catalog compiled for two parts that share a joint (the servo and the arm) is one plan over that joint.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from .align import Aligner, Recording
from .metric import Metric, StopStatistic, Stops, reduce_items


@dataclass(frozen=True)
class Alias:
    """A semantic name over a math-only metric: the signal it reads, how it is shown, how runs combine.

    ``across`` is "median" | "max" (over the runs' own values) or "pooled:<statistic>" (over every run's items: stops,
    rows) - the statistic a dataset needs is declared where the meaning is, not in the math."""

    name: str
    metric: Metric
    signal: str
    unit: str = ""
    factor: float = 1.0
    across: str = "median"

    def card(self) -> dict[str, Any]:
        return {"name": self.name, "unit": self.unit, "signal": self.signal, "target": None, "across": self.across,
                "function": f"robot_msgs.metrics.{type(self.metric).__name__}.functional", **self.metric.card()}


@dataclass(frozen=True)
class Limits:
    low: float = -math.inf
    high: float = math.inf

    def contains(self, value: Any) -> bool | None:
        if value is None or (isinstance(value, float) and not math.isfinite(value)):
            return None
        return self.low <= float(value) <= self.high

    def __str__(self) -> str:
        return f"<= {self.high:g}" if math.isinf(self.low) else f">= {self.low:g}"


@dataclass(frozen=True)
class Check:
    """A diagnostic or gate in a part's catalog: alias + Limits + who owns the number (RobotDx's catalog row)."""

    alias: str
    limits: Limits
    owner: str


@dataclass
class RunResult:
    id: str
    labels: Mapping[str, Any]
    values: dict[str, Any]
    items: dict[str, list[float]]
    faulted: bool = False


@dataclass
class Plan:
    aliases: dict[str, Alias]
    signals: dict[str, tuple[str, ...]]   # name -> (command topic, state topic[, rule])
    nodes: dict[tuple, Metric] = field(default_factory=dict)
    stops: dict[tuple, Stops] = field(default_factory=dict)
    aligners: dict[str, Aligner] = field(default_factory=dict)

    @classmethod
    def compile(cls, aliases: Sequence[Alias], signals: Mapping[str, tuple[str, str]]) -> Plan:
        plan = cls({alias.name: alias for alias in aliases}, dict(signals))
        for alias in aliases:
            key = (alias.signal, alias.metric.identity)
            if key in plan.nodes:
                continue
            node = alias.metric.clone()
            if isinstance(node, StopStatistic):
                node.source = plan.stops.setdefault((alias.signal, node.rule), Stops(**node.rule.__dict__))
            plan.nodes[key] = node
        plan.reset()
        return plan

    def reset(self) -> None:
        self.aligners = {name: Aligner(*spec) for name, spec in self.signals.items()}
        for node in [*self.nodes.values(), *self.stops.values()]:
            node.reset()

    @property
    def computations(self) -> int:
        """Distinct states fed per update: plain nodes plus stop passes (a StopStatistic is a read of its Stops)."""

        return sum(not isinstance(node, StopStatistic) for node in self.nodes.values()) + len(self.stops)

    def _feed(self, signal: str, pair: tuple[Any, Any] | None) -> None:
        if pair is None:
            return
        for (name, _), node in self.nodes.items():
            if name == signal and node.reads == "aligned" and not isinstance(node, StopStatistic):
                node.update(*pair)
        for (name, _), node in self.stops.items():
            if name == signal:
                node.update(*pair)

    def push(self, topic: str, message: Any) -> None:
        """One message (or a chunk of one topic), live."""

        for signal, aligner in self.aligners.items():
            command, state = self.signals[signal][:2]
            if topic in (command, state):
                for (name, _), node in self.nodes.items():
                    if name == signal and node.reads == "raw":
                        node.update(message if topic == command else None, message if topic == state else None)
                self._feed(signal, aligner.push(topic, message))

    def finish(self) -> None:
        for signal, aligner in self.aligners.items():
            self._feed(signal, aligner.finish())
        for node in [*self.nodes.values(), *self.stops.values()]:
            node.finish()

    def run(self, recording: Recording) -> RunResult:
        """Offline: every topic pushed whole (commands first), then finish. The live path with bigger messages."""

        self.reset()
        for signal, spec in self.signals.items():
            self.push(spec[0], recording.topics[spec[0]])
            self.push(spec[1], recording.topics[spec[1]])
        self.finish()
        return self.result(recording)

    def result(self, recording: Recording) -> RunResult:
        values, items = {}, {}
        for name, alias in self.aliases.items():
            node = self.nodes[(alias.signal, alias.metric.identity)]
            values[name] = node.compute()
            items[name] = node.items()
        return RunResult(recording.id, dict(recording.labels), values, items, recording.faulted)

    def __repr__(self) -> str:
        return (f"Plan({len(self.aliases)} aliases -> {len(self.nodes)} nodes, {self.computations} computations "
                f"incl. {len(self.stops)} stop pass{'es' if len(self.stops) != 1 else ''})")


def display(alias: Alias, value: Any) -> Any:
    return value if value is None or isinstance(value, bool) else float(value) * alias.factor


def across_runs(alias: Alias, results: Sequence[RunResult]) -> Any:
    """The alias over a group of runs, in its display unit: per-run values combined, or every run's items pooled."""

    if alias.across.startswith("pooled:"):
        pooled = [item for result in results for item in result.items[alias.name]]
        statistic = alias.across.split(":", 1)[1]
        return display(alias, reduce_items(pooled, statistic, empty=0.0 if statistic == "max" else None))
    values = [result.values[alias.name] for result in results if result.values[alias.name] is not None]
    if not values:
        return None
    import statistics

    return display(alias, {"median": statistics.median, "max": max, "mean": statistics.fmean}[alias.across](values))
