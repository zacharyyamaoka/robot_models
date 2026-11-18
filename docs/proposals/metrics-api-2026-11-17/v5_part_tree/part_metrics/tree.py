"""The part tree: each RobotPart evaluates its own metrics on its own signal; an assembly reads its children's.

    servo = Part("can11 servo", IntegratedServo, metrics=servo_aliases(), signals={"output": ("target", "state")})
    arm = Part("can11 arm", ladders()["rigid_body_chain"], metrics=tip_aliases(), children=(servo,),
               context={"lever_m": 0.339})                  # an assembly: its aliases read the child's events
    rack = Part("one-dof-test-rack", None, children=arms, checks=...)
    rack.observe(recording)                  # every node updates once per signal it owns (a child is never re-read)
    rack.status()                            # diagnostic_aggregator: a node's level is the worst of its own and children's

WHY the tree supplies the hierarchy (the prior-art note §3: MetricCollection is a flat bag): the vault note's question
"is the pendulum a 1-joint chain judged at its tip, or a servo with a load term?" becomes "both": the arm is a node
whose child is the servo, and its tip aliases are the servo's computation shown through the lever (context, not data).
"""

from __future__ import annotations

import math
import statistics
from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from .metric import Event, Metric, Segmenter

OK, WARN, ERROR, STALE = 0, 1, 2, 3          # diagnostic_msgs/DiagnosticStatus levels
LEVEL = {OK: "OK", WARN: "WARN", ERROR: "ERROR", STALE: "STALE"}


@dataclass(frozen=True)
class Alias:
    """A name for a configured metric on a node's signal; ``of_child`` reads that child's signal instead (no recompute).
    ``scale`` is a context key (the lever) or a number: the display conversion is the node's, never the math's."""

    name: str
    metric: Metric
    signal: str = "output"
    unit: str = ""
    factor: float = 1.0
    scale_by: str = ""              # a context key multiplied into the factor ("lever_m")
    across: str = "pool"            # pool (every run's events) | median | max (of the runs' own values)
    of_child: str = ""

    def card(self) -> dict[str, Any]:
        return {"name": self.name, "unit": self.unit, "signal": f"{self.of_child + '/' if self.of_child else ''}{self.signal}",
                "target": None, "function": f"robotparts.metrics.{type(self.metric).__name__}.compute",
                **self.metric.card()}


@dataclass(frozen=True)
class Check:
    alias: str
    high: float
    owner: str
    level: int = ERROR


@dataclass
class Part:
    name: str
    models: Any
    metrics: Sequence[Alias] = ()
    signals: Mapping[str, tuple[str, str]] = field(default_factory=dict)
    children: Sequence[Part] = ()
    context: Mapping[str, Any] = field(default_factory=dict)
    checks: Sequence[Check] = ()
    ledger: list[dict] = field(default_factory=list)            # one entry per observed run: labels + event logs
    segmenters: dict[tuple, Segmenter] = field(default_factory=dict)
    path: str = ""

    def __post_init__(self) -> None:
        for child in self.children:
            child.path = f"{self.path or self.name}/{child.name}"

    # ---- the tree ------------------------------------------------------------------------------------------------
    def child(self, name: str) -> Part:
        return next(child for child in self.children if child.name == name)

    def walk(self):
        yield self
        for child in self.children:
            yield from child.walk()

    def _keys(self) -> set[tuple]:
        return {(alias.signal, alias.metric.segmenter()) for alias in self.metrics if not alias.of_child}

    @property
    def computations(self) -> int:
        """Segmentations this node owns (its children's are theirs): what one update costs here."""

        return len(self._keys())

    # ---- one run, live or offline --------------------------------------------------------------------------------
    def begin(self) -> None:
        for node in self.walk():
            node.segmenters = {key: Segmenter(*key[1]) for key in node._keys()}

    def update(self, streams: Mapping[str, Any]) -> None:
        """A chunk: every node feeds each segmenter it owns ONCE, from the streams named by its signals."""

        for node in self.walk():
            for (signal, _), segmenter in node.segmenters.items():
                target, measured = node.signals[signal]
                if len(streams[measured]):
                    segmenter.update(streams[target], streams[measured])

    def end(self, labels: Mapping[str, Any], faulted: bool = False) -> None:
        for node in self.walk():
            for segmenter in node.segmenters.values():
                segmenter.close()
            node.ledger.append({"labels": dict(labels), "faulted": faulted,
                                "events": {key: s.closed for key, s in node.segmenters.items()},
                                "pairs": {key: s.extra for key, s in node.segmenters.items()}})

    def observe(self, streams: Mapping[str, Any], labels: Mapping[str, Any], faulted: bool = False) -> None:
        self.begin()
        self.update(streams)
        self.end(labels, faulted)

    # ---- values ----------------------------------------------------------------------------------------------------
    def _source(self, alias: Alias) -> tuple[Part, tuple]:
        node = self.child(alias.of_child) if alias.of_child else self
        return node, (alias.signal, alias.metric.segmenter())

    def _factor(self, alias: Alias) -> float:
        return alias.factor * (float(self.context[alias.scale_by]) if alias.scale_by else 1.0)

    def _display(self, alias: Alias, value: Any) -> Any:
        return value if value is None or isinstance(value, bool) else float(value) * self._factor(alias)

    def value(self, name: str, entries: Sequence[dict] | None = None, live: bool = False) -> Any:
        """An alias over some of the ledger's runs (default: the last), or over the open run when ``live``."""

        alias = next(a for a in self.metrics if a.name == name)
        node, key = self._source(alias)
        if live:
            segmenter = node.segmenters[key]
            return self._display(alias, alias.metric.compute(segmenter.events(), {"pairs": segmenter.extra}))
        entries = entries if entries is not None else node.ledger[-1:]
        if alias.across == "pool":
            events = [event for entry in entries for event in entry["events"][key]]
            pairs = [pair for entry in entries for pair in entry["pairs"][key]]
            return self._display(alias, alias.metric.compute(events, {"pairs": pairs}))
        per_run = [alias.metric.compute(entry["events"][key], {"pairs": entry["pairs"][key]}) for entry in entries]
        per_run = [value for value in per_run if value is not None]
        if not per_run:
            return None
        return self._display(alias, {"median": statistics.median, "max": max}[alias.across](per_run))

    def values(self, entries: Sequence[dict] | None = None, live: bool = False) -> dict[str, Any]:
        return {alias.name: self.value(alias.name, entries, live) for alias in self.metrics}

    def items(self, name: str, entries: Sequence[dict]) -> list[float]:
        alias = next(a for a in self.metrics if a.name == name)
        node, key = self._source(alias)
        events = [event for entry in entries for event in entry["events"][key]]
        return [item * self._factor(alias) for item in alias.metric.items(events)]

    def groups(self, by: Sequence[str]) -> dict[tuple, list[dict]]:
        out = defaultdict(list)
        for entry in self.ledger:
            out[tuple(entry["labels"][key] for key in by)].append(entry)
        return dict(out)

    # ---- status ----------------------------------------------------------------------------------------------------
    def level(self, values: Mapping[str, Any]) -> tuple[int, list[str]]:
        worst, why = OK, []
        for check in self.checks:
            value = values.get(check.alias)
            if value is None or (isinstance(value, float) and not math.isfinite(value)):
                worst = max(worst, STALE)
                why.append(f"{check.alias}: no value")
            elif float(value) > check.high:
                worst = max(worst, check.level)
                why.append(f"{check.alias} {float(value):.3g} > {check.high:g} ({check.owner})")
        return worst, why
