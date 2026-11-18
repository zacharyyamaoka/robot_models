"""The v5 metrics: torchmetrics' shape over SEGMENT EVENTS, evaluated by the part that owns the signal.

    segments = Segmenter()                       # one per (part, signal): cuts the stream into moves and dwells
    segments.update(target, measured)            # live chunks or the whole run; a segment open at a chunk's end carries
    TrackingError(statistic="rms").compute(segments.events())     # metrics read events, never raw rows twice

A move event holds its moving rows' (sum e^2, n, max |e|); a dwell event its max |e| and, when it is a stop, the stop
(stock.StopTracker). WHY events: they are what a drill-down opens (each has t0, t1 for Rerun's timeline and a W&B
row), every metric of a signal reads one segmentation (the duplicate-work rule), and a stop that spans two chunks is
an open event, not a special case. Context (the part: its lever, its joint) is passed beside the data, as nuPlan's
compute(history, scenario): a metric never looks it up.
"""

from __future__ import annotations

import copy
import math
import statistics
from dataclasses import asdict, astuple, dataclass, field
from typing import Any, ClassVar

import numpy as np
from robot_msgs.compare import compare, estimate_time_offset, goal
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory, concatenate

import stock


@dataclass
class Event:
    kind: str                 # move | dwell
    t0: float
    t1: float
    sum_squares: float = 0.0  # moving rows only
    count: int = 0
    max_abs: float = 0.0      # every row
    stops: list = field(default_factory=list)   # a dwell event's stops (stock.Stop), matched by the dwell's start


class Segmenter:
    """One signal's events. ``update`` takes aligned (target, measured) chunks; ``close`` ends the run."""

    def __init__(self, field_name: str = "positions", joint: int = 0, include_last_dwell: bool = True) -> None:
        self.field_name, self.joint, self.include_last_dwell = field_name, joint, include_last_dwell
        self.closed: list[Event] = []
        self.open: Event | None = None
        self.tracker: stock.StopTracker | None = None
        self.updates = 0
        self.extra: list[tuple[Any, Any]] = []     # the raw pairs, for metrics that need the whole run (lag)

    def update(self, target: Any, measured: Any) -> None:
        self.updates += 1
        self.extra.append((target, measured))
        if self.field_name == "raw":      # a signal only whole-run metrics read (the lag's plan and state)
            return
        error = compare(target, measured).error
        values = (np.linalg.norm(error.positions, axis=1) if isinstance(error, PoseTrajectory)
                  else np.asarray(getattr(error, self.field_name)[:, self.joint], float))
        has_speed = isinstance(target, JointTrajectory) and target.velocities is not None
        moving = stock.moving(target, self.joint) if has_speed else np.ones(len(values), bool)
        t = np.asarray(measured.t, float)
        for row in range(len(values)):
            kind = "move" if moving[row] else "dwell"
            if self.open is None or self.open.kind != kind:
                if self.open is not None:
                    self.closed.append(self.open)
                self.open = Event(kind, float(t[row]), float(t[row]))
            event = self.open
            event.t1 = float(t[row])
            if np.isfinite(values[row]):
                event.max_abs = max(event.max_abs, abs(float(values[row])))
                if kind == "move":
                    event.sum_squares += float(values[row]) ** 2
                    event.count += 1
        if has_speed and self.field_name == "positions":
            if self.tracker is None:
                self.tracker = stock.StopTracker(joint=target.joint_names[self.joint],
                                                 include_last_dwell=self.include_last_dwell)
            self._attach(self.tracker.push_many(measured.t, measured.positions[:, self.joint],
                                                target.positions[:, self.joint], target.velocities[:, self.joint]))

    def _attach(self, found: tuple) -> None:
        """A stop closes when its dwell ends; it belongs to the dwell event that holds the dwell's first row."""

        for stop in found:
            start = float(stop.move_t[stop.move_end])
            for event in reversed([*self.closed, *([self.open] if self.open else [])]):
                if event.kind == "dwell" and event.t0 <= start <= event.t1:
                    event.stops.append(stop)
                    break
            else:
                raise AssertionError(f"a stop at {start} s has no dwell event")

    def close(self) -> None:
        if self.tracker is not None:
            self._attach(self.tracker.finish())
            self.tracker = None
        if self.open is not None:
            self.closed.append(self.open)
            self.open = None

    def events(self) -> list[Event]:
        """Every event so far; an open dwell's stop as if the run ended now (reading never mutates the state)."""

        if self.tracker is None and self.open is None:
            return list(self.closed)
        probe = copy.deepcopy(self)
        probe.close()
        return probe.closed


class Metric:
    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    references: ClassVar[tuple[str, ...]] = ()
    value_type: ClassVar[str] = "number"
    Input: ClassVar[str] = "Events"        # what compute() reads; update() feeds the node's Segmenter
    live: ClassVar[bool] = True
    hint: ClassVar[str] = "kpi_trend"
    Config: ClassVar[type]

    def __init__(self, **config: Any) -> None:
        self.config = self.Config(**config)

    @property
    def identity(self) -> tuple:
        return type(self).__name__, astuple(self.config)

    def segmenter(self) -> tuple:
        """Which segmentation this metric reads (field, joint, last-dwell rule): metrics sharing it share one."""

        return (getattr(self.config, "field", "positions"), getattr(self.config, "joint", 0),
                getattr(self.config, "include_last_dwell", True))

    def compute(self, events: list[Event], context: Any = None) -> Any:
        raise NotImplementedError

    def items(self, events: list[Event], context: Any = None) -> list[float]:
        value = self.compute(events, context)
        return [] if value is None else [float(value)]

    # ---- torchmetrics' face when used alone (in a tree the node's Segmenter holds the state and is shared) ----------
    def update(self, target: Any, measured: Any) -> None:
        if getattr(self, "_own", None) is None:
            self._own = Segmenter(*self.segmenter())
        self._own.update(target, measured)

    def reset(self) -> None:
        self._own = None

    def value(self, context: Any = None) -> Any:
        own = getattr(self, "_own", None)
        if own is None:
            return None
        return self.compute(own.events(), {"pairs": own.extra, **(context or {})})

    def functional(self, target: Any, measured: Any, context: Any = None) -> Any:
        segmenter = Segmenter(*self.segmenter())
        segmenter.update(target, measured)
        segmenter.close()
        return self.compute(segmenter.closed, context)

    def card(self) -> dict[str, Any]:
        return {"base": type(self).__name__, "config": ", ".join(f"{k}={v!r}" for k, v in asdict(self.config).items()),
                "description": self.description, "references": list(self.references), "value_type": self.value_type,
                "input": self.Input, "live": self.live}

    def __repr__(self) -> str:
        return f"{type(self).__name__}({', '.join(f'{k}={v!r}' for k, v in asdict(self.config).items())})"


@dataclass(frozen=True)
class TrackingConfig:
    statistic: str = "rms"      # rms (moving rows) | max_abs (every row)
    field: str = "positions"
    joint: int = 0


class TrackingError(Metric):
    name = "tracking error"
    description = "target - measured: RMS over the moving rows of every move, or the largest |e| of the run."
    references = ("robot_msgs.compare.ErrorStats", "evo APE")
    Config = TrackingConfig

    def compute(self, events: list[Event], context: Any = None) -> float | None:
        if self.config.statistic == "max_abs":
            return max((event.max_abs for event in events), default=None)
        count = sum(event.count for event in events)
        return math.sqrt(sum(event.sum_squares for event in events) / count) if count else None


@dataclass(frozen=True)
class StopConfig:
    of: str = "error"           # error | settle | settled | overshoot | count
    statistic: str = "median"
    band: float = math.inf
    include_last_dwell: bool = True
    joint: int = 0


class StopEvents(Metric):
    name = "stop events"
    description = "A quantity of every stop event (a dwell >= 0.25 s at zero commanded speed a move led into)."
    references = ("bam_rack_twin T4", "MATLAB stepinfo", "robot_msgs.compare.goal")
    Config = StopConfig
    hint = "distribution"

    @property
    def value_type(self) -> str:  # type: ignore[override]
        return "bool" if self.config.of == "settled" else "number"

    def items(self, events: list[Event], context: Any = None) -> list[float]:
        found = [stop for event in events for stop in event.stops]
        of = self.config.of
        if of == "error":
            return [abs(stop.error) for stop in found]
        if of == "settle":
            return [stock.settle_time(stop, self.config.band) for stop in found]
        if of == "settled":
            return [float(math.isfinite(stock.settle_time(stop, self.config.band))) for stop in found]
        if of == "overshoot":
            return [goal(*stop.move(), band=math.inf)[f"positions/{stop.joint}"].overshoot for stop in found]
        return [1.0 for _ in found]

    def compute(self, events: list[Event], context: Any = None) -> Any:
        return reduce_items(self.items(events), self.config.statistic, 0.0 if self.config.of == "overshoot" else None)


def reduce_items(values: list[float], statistic: str, empty: Any = None) -> Any:
    if statistic == "count":
        return float(len(values))
    if statistic == "all":
        return all(value == 1.0 for value in values)
    if not values:
        return empty
    if statistic == "median":
        middle = statistics.median(values)
        return None if math.isinf(middle) else float(middle)
    if statistic == "max":
        kept = [v for v in values if math.isfinite(v)]
        return max(kept) if kept else None
    if statistic == "p90":
        return float(np.percentile(values, 90.0))
    if statistic == "mean":
        return float(np.mean(values))
    raise ValueError(statistic)


@dataclass(frozen=True)
class LagConfig:
    max_offset_s: float = 0.25


class Lag(Metric):
    name = "lag"
    description = "How late the response runs behind the command (estimate_time_offset of the run's plan and state)."
    references = ("robot_msgs.compare.estimate_time_offset",)
    Input = "the run's raw pairs (plan, state)"
    Config = LagConfig
    live = False

    def segmenter(self) -> tuple:
        return ("raw", 0, True)

    def compute(self, events: Any, context: Any = None) -> float | None:
        pairs = context["pairs"] if context else []
        if not pairs:
            return None
        target = concatenate([p[0] for p in pairs]) if len(pairs) > 1 else pairs[0][0]
        measured = concatenate([p[1] for p in pairs]) if len(pairs) > 1 else pairs[0][1]
        return estimate_time_offset(target, measured, max_offset_s=self.config.max_offset_s).offset_s


@dataclass(frozen=True)
class CodeConfig:
    codes: tuple[int, ...] = (97, 98, 99, 102)
    channel: str = "fault"


class CodeShare(Metric):
    name = "code share"
    description = "The share of state rows whose code channel holds one of the codes."
    references = ("moteus fault and limiter codes", "RobotDx stock.fault_codes")
    Input = "the run's raw pairs (the state's code channel)"
    Config = CodeConfig

    def items(self, events: Any, context: Any = None) -> list[float]:
        pairs = context["pairs"] if context else []
        return [float(flag) for _, measured in pairs
                for flag in np.isin(np.asarray(measured.channels[self.config.channel]).astype(int), self.config.codes)]

    def compute(self, events: Any, context: Any = None) -> float | None:
        flags = self.items(events, context)
        return float(np.mean(flags)) if flags else None


@dataclass(frozen=True)
class RepeatabilityConfig:
    k: float = 3.0


class Repeatability(Metric):
    name = "repeatability"
    description = "ISO 9283 RP at each stop event over same-input repeats: mean(l) + k S_l, the worst stop index."
    references = ("ISO 9283:1998 §7.2.2",)
    Input = "Repeats (one node's event logs of one command)"
    Config = RepeatabilityConfig
    live = False

    def compute(self, logs: list[list[Event]], context: Any = None) -> float | None:  # type: ignore[override]
        errors = [[stop.error for event in events for stop in event.stops] for events in logs]
        if len(errors) < 2 or len({len(e) for e in errors}) != 1:
            return None
        matrix = np.asarray(errors)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return float(np.max(distance.mean(axis=0) + self.config.k * distance.std(axis=0, ddof=1)))


@dataclass(frozen=True)
class GainConfig:
    sign: str = "lower_is_better"


class PerformanceGain(Metric):
    name = "performance gain"
    description = "An alias's metric on held-out runs with nominal and with fitted values; nominal minus fitted."
    references = ("hold-out validation",)
    Input = "HeldOutPair (two event logs of the same held-out runs)"
    Config = GainConfig
    live = False

    def __init__(self, metric: Metric, **config: Any) -> None:
        self.metric = metric
        super().__init__(**config)

    def compute(self, pair: tuple[list[Event], list[Event]], context: Any = None) -> float | None:  # type: ignore[override]
        nominal, fitted = (self.metric.compute(events) for events in pair)
        return None if nominal is None else nominal - fitted
