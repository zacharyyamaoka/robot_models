"""Aliases, the ``Metrics`` collection a family declares, the ``Episode`` it reads, and a deployment's gates.

    class IntegratedServo(ModelFamily, id="integrated_servo"):        # robot_models' family body (production)
        ...models...
        metrics = Metrics(Alias("tracking RMS", ErrorStat(statistic="rms", rows="moving"), unit="deg", ...), ...)

    IntegratedServo.metrics(episode)                       # one recording, offline
    collection = IntegratedServo.metrics.clone(); collection.update(chunk) ...; collection.compute()   # live
    Deployment("one-dof-test-rack", gates=...).judge(values)   # targets: robot_msgs Values, outside compute()

A target is a robot_msgs ``Value`` with its range and its ``source`` (who chose it), the same noun sim2real's
``gate(value, target)`` and a datasheet already use (Zach, 2026-11-10: "a datasheet value and a gate target are the
same noun").
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

import numpy as np
from robot_msgs.space import Value, meta

from .metric import Metric

DEGREES = 180.0 / math.pi


@dataclass(frozen=True)
class Episode:
    """One run (the MDP's word): robot_msgs streams by name, labels, and the ``.rrd`` it was recorded to."""

    id: str
    streams: Mapping[str, Any]
    labels: Mapping[str, Any]
    rrd: str | None = None
    faulted: bool = False

    @classmethod
    def from_run(cls, run: Any) -> Episode:
        streams = {"target": run.target, "state": run.measured}
        if run.plan is not None:
            streams["plan"] = run.plan
        if run.tool_target is not None:
            streams.update({"tool/target": run.tool_target, "tool/state": run.tool_measured})
        return cls(run.id, streams, {"station": run.station, "source": run.source, **run.labels}, faulted=run.faulted)

    def chunks(self, rows: int) -> Iterator[Episode]:
        state = self.streams["state"]
        edges = state.t[rows - 1::rows].tolist()
        if not edges or edges[-1] < state.t[-1]:
            edges.append(float(state.t[-1]))
        low = -math.inf
        for high in edges:
            yield replace(self, streams={name: stream[np.flatnonzero((stream.t > low) & (stream.t <= high))]
                                         for name, stream in self.streams.items()})
            low = high


@dataclass(frozen=True)
class Alias:
    name: str
    metric: Metric
    signal: str = "joint"
    unit: str = ""
    factor: float = 1.0
    owner: str = ""

    def key(self) -> tuple:
        return (self.signal, self.metric.identity)

    def card(self) -> dict[str, Any]:
        card = self.metric.card()
        return {"name": self.name, "unit": self.unit, "signal": self.signal, "target": None,
                "function": f"robot_models.metrics.{card['base']}.functional", **card}


class Metrics:
    """A family's metrics: aliases over configured base metrics, one member per (signal, identity)."""

    def __init__(self, *aliases: Alias, signals: Mapping[str, tuple[str, str]]) -> None:
        self.aliases = {alias.name: alias for alias in aliases}
        self.signals = dict(signals)
        self.members: dict[tuple, Metric] = {}
        for alias in aliases:
            self.members.setdefault(alias.key(), alias.metric.clone())

    def __add__(self, more: Sequence[Alias]) -> Metrics:
        return Metrics(*self.aliases.values(), *more, signals=self.signals)

    def clone(self) -> Metrics:
        return Metrics(*self.aliases.values(), signals=self.signals)

    def update(self, episode: Episode) -> None:
        for (signal, _), member in self.members.items():
            target_name, measured_name = self.signals[signal]
            measured = episode.streams[measured_name]
            if len(measured):
                member.update(episode.streams[target_name], measured,
                              key=f"{episode.labels.get('source', '')}:{episode.id}:{signal}")

    def end_run(self) -> None:
        for member in self.members.values():
            member.end_run()

    def reset(self) -> None:
        for member in self.members.values():
            member.reset()

    def compute(self) -> dict[str, Any]:
        out = {}
        for name, alias in self.aliases.items():
            value = self.members[alias.key()].compute()
            out[name] = value if (value is None or isinstance(value, bool)) else float(value) * alias.factor
        return out

    def __call__(self, episode: Episode) -> dict[str, Any]:
        fresh = self.clone()
        fresh.update(episode)
        fresh.end_run()
        return fresh.compute()

    def table(self, title: str) -> str:
        width = max(len(name) for name in self.aliases)
        lines = [f"{title}.metrics: {len(self.aliases)} aliases over {len(self.members)} configured metrics"]
        for alias in self.aliases.values():
            lines.append(f"  {alias.name:<{width}}  {alias.metric!r:<70} on {alias.signal:<7} [{alias.unit or '-'}]")
        return "\n".join(lines)

    def __repr__(self) -> str:
        return f"Metrics({len(self.aliases)} aliases, {len(self.members)} members)"


@dataclass(frozen=True)
class Gate:
    alias: str
    target: Value          # robot_msgs Value: the bound, its unit, its source

    def check(self, values: Mapping[str, Any]) -> bool | None:
        value = values.get(self.alias)
        if value is None or (isinstance(value, float) and not math.isfinite(value)):
            return None
        bounds = meta(self.target)
        return bool(bounds["low"] <= float(value) <= bounds["high"])

    def __str__(self) -> str:
        bounds = meta(self.target)
        side = f"<= {bounds['high']:g}" if math.isinf(bounds["low"]) else f">= {bounds['low']:g}"
        return f"{self.alias} {side} {bounds['unit'] or ''} ({bounds['source']})"


def at_most(limit: float, unit: str, source: str) -> Value:
    return Value(limit, unit, low=-math.inf, high=limit, source=source)


@dataclass(frozen=True)
class Deployment:
    """What a deployment holds itself to: named verdicts, each every one of its gates (W&B: one project per deployment)."""

    name: str
    verdicts: Mapping[str, tuple[Gate, ...]]

    def judge(self, values: Mapping[str, Any]) -> dict[str, bool | None]:
        out = {}
        for name, gates in self.verdicts.items():
            checks = [gate.check(values) for gate in gates]
            out[name] = False if False in checks else (None if None in checks else True)
        return out


def view(collection: Metrics, alias_name: str, title: str = "", target: float | None = None) -> Any:
    """The points behind an alias's value in its display unit: a Dwell's every stop, any other metric's every run."""

    import _viz

    from .base import Dwell, stops_of

    alias = collection.aliases[alias_name]
    member = collection.members[alias.key()]
    if isinstance(member, Dwell):
        keys = ["|".join(map(str, part)) for part in member.data_key()]
        found = [stop for key, (target_stream, measured) in zip(keys, member._joined())
                 for stop in stops_of(member.config, key, target_stream, measured)]
        points = [value * alias.factor for value in Dwell.quantities(member.config, found)]
        x_label = "stop"
    else:
        points = [type(member).functional(member.config, [run]) * alias.factor for run in member._joined()]
        x_label = "run"
    return _viz.View(title=title or alias.name, unit=alias.unit, hint=member.hint, x=list(range(len(points))),
                     y=points, target=target, target_label=f"target {target:g} {alias.unit}" if target is not None else "",
                     x_label=x_label, y_label=alias.unit)
