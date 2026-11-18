"""The stock base metrics: mathematical meaning only. What they measure comes from the streams they are given.

    ErrorStat(statistic="rms", rows="moving")   one error signal's statistic: servo tracking, tip tracking, tool error
    DwellError(statistic="median")              the error left at every settled stop (the T4 rule, stock.StopTracker)
    SettleTime(band=..., statistic="median")    from a move's end to inside a band for good
    Overshoot(statistic="max")                  past a stop's target along its move (robot_msgs.compare.goal)
    AllSettled(band=...)                        bool: did every stop get inside the band
    TimeOffset()                                how late the response runs (robot_msgs.compare.estimate_time_offset)
    CodeShare(codes=(97, 98, 99, 102))          share of rows whose code channel holds one of the codes

The dwell metrics share one ``Dwells`` state: in a collection, every dwell metric on the same signal with the same dwell
rule reads ONE tracker (torchmetrics' compute groups), so "tip stop error", "T4 stop error p90", the settle time and
the overshoot cost one pass. Functional twins: ``robot_metrics.functional``.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from robot_msgs.compare import compare, error_stats, estimate_time_offset, goal
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory, concatenate

import stock

from .metric import Metric, finite

ROBOT_MSGS = "robot_msgs.compare (ErrorStats: evo's rmse/max + tracking's p95 and final)"
T4 = "bam_rack_twin acceptance T4 (stop_accuracy: dwell >= 0.25 s, error = mean of the last 50 ms)"
STEPINFO = "MATLAB stepinfo (settling time, overshoot); ROS 2 joint_trajectory_controller goal check"
ISO9283 = "ISO 9283:1998 (manipulating industrial robots: performance criteria)"


def error_signal(target: Any, measured: Any, field_name: str = "positions", joint: int = 0) -> np.ndarray:
    """target - measured on the measured run's samples, through robot_msgs.compare (one column, or a pose's distance)."""

    error = compare(target, measured).error
    if isinstance(error, PoseTrajectory):
        return np.linalg.norm(error.positions, axis=1)
    return np.asarray(getattr(error, field_name)[:, joint], float)


def _rows(target: Any, rows: str) -> np.ndarray | slice:
    if rows == "all" or not isinstance(target, JointTrajectory) or target.velocities is None:
        return slice(None)
    return stock.moving(target)


@dataclass(frozen=True)
class ErrorStatConfig:
    statistic: str = "rms"        # rms | max_abs | mean_abs (mergeable); p95_abs keeps every sample ("cat")
    rows: str = "all"             # all | moving (rows whose commanded speed is not zero)
    field: str = "positions"      # positions | velocities | effort (a JointTrajectory); a PoseTrajectory: its distance
    joint: int = 0


class ErrorStat(Metric):
    name = "error statistic"
    description = "One statistic of the error signal target - measured over the rows it is told to read."
    references = (ROBOT_MSGS, "scikit-learn mean_squared_error; evo APE")
    Config = ErrorStatConfig

    def setup(self) -> None:
        if self.config.statistic not in ("rms", "max_abs", "mean_abs", "p95_abs"):
            raise ValueError(f"statistic {self.config.statistic!r}")
        self.memory = "O(rows)" if self.config.statistic == "p95_abs" else "O(1)"
        self.add_state("sum_squares", 0.0, "sum")
        self.add_state("sum_abs", 0.0, "sum")
        self.add_state("count", 0, "sum")
        self.add_state("max_abs", 0.0, "max")
        self.add_state("samples", [], "cat")

    def _update(self, target: Any, measured: Any) -> None:
        error = error_signal(target, measured, self.config.field, self.config.joint)[_rows(target, self.config.rows)]
        error = finite(error)
        if not error.size:
            return
        stats = error_stats(error)  # WHY robot_msgs: its rms and max_abs are the ones every report already prints
        self.sum_squares += stats.rms ** 2 * stats.n
        self.sum_abs += float(np.sum(np.abs(error)))
        self.count += stats.n
        self.max_abs = max(self.max_abs, stats.max_abs)
        if self.config.statistic == "p95_abs":
            self.samples = [*self.samples, *error.tolist()]

    def compute(self) -> float | None:
        if not self.count:
            return None
        return {"rms": lambda: math.sqrt(self.sum_squares / self.count), "max_abs": lambda: self.max_abs,
                "mean_abs": lambda: self.sum_abs / self.count,
                "p95_abs": lambda: float(np.percentile(np.abs(self.samples), 95.0))}[self.config.statistic]()


@dataclass(frozen=True)
class DwellRule:
    """The stop rule's own config (what makes two dwell states the same)."""

    include_last_dwell: bool = True   # the pendulum counts its last dwell; sim2real / RobotDx's T4 drop it (parking)
    joint: int = 0


class Dwells:
    """The shared state of every dwell metric: the stops closed so far (pooled) and the tracker of the open run."""

    def __init__(self, rule: DwellRule) -> None:
        self.rule = rule
        self.stops: list[stock.Stop] = []
        self.tracker: stock.StopTracker | None = None
        self.updates = 0

    def update(self, target: JointTrajectory, measured: JointTrajectory) -> None:
        self.updates += 1
        if self.tracker is None:
            self.tracker = stock.StopTracker(joint=target.joint_names[self.rule.joint],
                                             include_last_dwell=self.rule.include_last_dwell)
        j = self.rule.joint
        self.stops += self.tracker.push_many(measured.t, measured.positions[:, j], target.positions[:, j],
                                             target.velocities[:, j])

    def end_run(self) -> None:
        if self.tracker is not None:
            self.stops += self.tracker.finish()
            self.tracker = None

    def closed(self) -> list[stock.Stop]:
        """Every stop so far, the open dwell included as if the run ended now (compute never mutates the state)."""

        if self.tracker is None:
            return list(self.stops)
        import copy

        return [*self.stops, *copy.deepcopy(self.tracker).finish()]


class DwellMetric(Metric):
    """A metric of the stops: its state is a ``Dwells`` that a collection may share between members."""

    hint = "distribution"
    memory = "O(stops)"
    rule_fields: tuple[str, ...] = ("include_last_dwell", "joint")

    def setup(self) -> None:
        self.dwells = Dwells(DwellRule(**{k: getattr(self.config, k) for k in self.rule_fields}))

    @property
    def dwell_key(self) -> tuple:
        return (Dwells.__name__, self.dwells.rule)

    def _update(self, target: Any, measured: Any) -> None:
        self.dwells.update(target, measured)

    def end_run(self) -> None:
        self.dwells.end_run()

    def reset(self) -> None:
        super().reset()
        self.dwells = Dwells(self.dwells.rule)

    def merge_state(self, other: Metric) -> Metric:
        if other.identity != self.identity:
            raise ValueError(f"cannot merge {other.identity} into {self.identity}")
        self.dwells.stops = [*self.dwells.closed(), *other.dwells.closed()]
        self.dwells.tracker = None
        return self

    def values(self) -> list[float]:
        raise NotImplementedError

    def view(self, scale: float = 1.0, unit: str = "", title: str = "") -> Any:
        import _viz

        values = self.values()
        return _viz.View(title=title or self.name, unit=unit, hint=self.hint, x=list(range(len(values))),
                         y=[v * scale for v in values], x_label="stop", y_label=unit)


def _summary(values: list[float], statistic: str) -> float | None:
    if statistic == "count":
        return float(len(values))
    if not values:
        return None
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
    raise ValueError(f"statistic {statistic!r}")


@dataclass(frozen=True)
class DwellErrorConfig:
    statistic: str = "median"   # median | max | p90 | mean | count, over |error| of every stop (pooled over runs)
    include_last_dwell: bool = True
    joint: int = 0


class DwellError(DwellMetric):
    name = "dwell error"
    description = ("The error left at every stop: |mean(measured - target)| over the last 50 ms of each dwell of at "
                   "least 0.25 s at zero commanded speed that a move led into; one statistic over the pooled stops.")
    references = (T4, ISO9283 + " pose accuracy")
    Config = DwellErrorConfig

    def values(self) -> list[float]:
        return [abs(stop.error) for stop in self.dwells.closed()]

    def compute(self) -> float | None:
        return _summary(self.values(), self.config.statistic)


@dataclass(frozen=True)
class SettleTimeConfig:
    band: float = math.inf     # in the signal's unit: inside it for good is settled
    statistic: str = "median"
    include_last_dwell: bool = True
    joint: int = 0


class SettleTime(DwellMetric):
    name = "settle time"
    description = "From each move's end to the first row after which the error stays inside the band; inf: never."
    references = (STEPINFO, T4)
    Config = SettleTimeConfig
    unit = "s"

    def values(self) -> list[float]:
        return [stock.settle_time(stop, self.config.band) for stop in self.dwells.closed()]

    def compute(self) -> float | None:
        return _summary(self.values(), self.config.statistic)


@dataclass(frozen=True)
class AllSettledConfig:
    band: float = math.inf
    include_last_dwell: bool = True
    joint: int = 0


class AllSettled(DwellMetric):
    name = "all settled"
    description = "True when every stop got inside the band and stayed there by the dwell's end."
    references = (STEPINFO,)
    Config = AllSettledConfig
    value_type = "bool"
    higher_is_better = True
    unit = ""

    def values(self) -> list[float]:
        return [float(math.isfinite(stock.settle_time(stop, self.config.band))) for stop in self.dwells.closed()]

    def compute(self) -> bool:
        return all(value == 1.0 for value in self.values())


@dataclass(frozen=True)
class OvershootConfig:
    statistic: str = "max"
    include_last_dwell: bool = True
    joint: int = 0


class Overshoot(DwellMetric):
    name = "overshoot"
    description = "How far the response went past each stop's target along the move into it (0 if it never did)."
    references = (STEPINFO, ROBOT_MSGS + " goal()")
    Config = OvershootConfig

    def values(self) -> list[float]:
        out = []
        for stop in self.dwells.closed():
            reference, measured = stop.move()
            out.append(goal(reference, measured, band=math.inf)[f"positions/{stop.joint}"].overshoot)
        return out

    def compute(self) -> float | None:
        values = self.values()
        return _summary(values, self.config.statistic) if values else 0.0


@dataclass(frozen=True)
class TimeOffsetConfig:
    max_offset_s: float = 0.25


class TimeOffset(Metric):
    Input = "TrackedPair (plan, state)"
    name = "time offset"
    description = "How late the response's clock runs behind the command (cross-correlation, then refined)."
    references = (ROBOT_MSGS + " estimate_time_offset", "trajectopy time-shift estimation")
    Config = TimeOffsetConfig
    unit = "s"
    memory = "O(rows) (a cross-correlation needs the run)"

    def setup(self) -> None:
        self.add_state("runs", [], "cat")          # finished runs' offsets
        self.open: list[tuple[Any, Any]] = []      # the open run's chunks

    def _update(self, target: Any, measured: Any) -> None:
        self.open.append((target, measured))

    def _offset(self, chunks: list) -> float:
        target = concatenate([chunk[0] for chunk in chunks]) if len(chunks) > 1 else chunks[0][0]
        measured = concatenate([chunk[1] for chunk in chunks]) if len(chunks) > 1 else chunks[0][1]
        return estimate_time_offset(target, measured, max_offset_s=self.config.max_offset_s).offset_s

    def end_run(self) -> None:
        if self.open:
            self.runs = [*self.runs, self._offset(self.open)]
            self.open = []

    def reset(self) -> None:
        super().reset()
        self.open = []

    def compute(self) -> float | None:
        values = [*self.runs, *([self._offset(self.open)] if self.open else [])]
        return float(statistics.median(values)) if values else None


@dataclass(frozen=True)
class CodeShareConfig:
    codes: tuple[int, ...] = (97, 98, 99, 102)   # moteus: max power, back-EMF, current, torque cap
    channel: str = "fault"


class CodeShare(Metric):
    Input = "TrackedPair (only the state's code channel is read)"
    name = "code share"
    description = "The share of rows whose code channel holds one of the codes."
    references = ("moteus fault and limiter codes (mjbots reference)", "RobotDx stock.fault_codes")
    Config = CodeShareConfig
    unit = "1"

    def setup(self) -> None:
        self.add_state("hits", 0, "sum")
        self.add_state("rows", 0, "sum")

    def _update(self, target: Any, measured: Any) -> None:
        codes = np.asarray(measured.channels[self.config.channel]).astype(int)
        self.hits += int(np.isin(codes, self.config.codes).sum())
        self.rows += int(codes.size)

    def compute(self) -> float | None:
        return self.hits / self.rows if self.rows else None


@dataclass(frozen=True)
class PerRunConfig:
    reduce: str = "median"


class PerRun(Metric):
    """A wrapper (torchmetrics' wrappers): the inner metric's value per run, then ``reduce`` over the runs. WHY: a rung's
    tracking RMS is the median of its plays' RMS, not the RMS of the pooled rows (the scorecard's rule)."""

    name = "per run"
    description = "The wrapped metric computed on each run alone, combined over runs by `reduce`."
    references = ("torchmetrics wrappers (Running, BootStrapper)", "RobotDx TrackingErrorConfig.across_runs")
    Config = PerRunConfig

    def __init__(self, inner: Metric, **config: Any) -> None:
        self.inner = inner
        super().__init__(**config)

    def setup(self) -> None:
        self.add_state("per_run", [], "cat")

    @property
    def identity(self) -> tuple:
        return type(self).__name__, (self.inner.identity, *super().identity[1])

    def clone(self) -> Metric:
        return PerRun(self.inner.clone(), reduce=self.config.reduce)

    def _update(self, target: Any, measured: Any) -> None:
        self.inner.update(target, measured)

    def end_run(self) -> None:
        if self.inner.updates:
            self.inner.end_run()
            self.per_run = [*self.per_run, self.inner.compute()]
            self.inner.reset()

    def merge_state(self, other: Metric) -> Metric:
        if other.identity != self.identity:
            raise ValueError(f"cannot merge {other.identity} into {self.identity}")
        open_run = [other.inner.compute()] if other.inner.updates else []
        self.per_run = [*self.per_run, *other.per_run, *open_run]
        return self

    def compute(self) -> float | None:
        values = [v for v in self.per_run if v is not None]
        if self.inner.updates:
            values.append(self.inner.compute())
        if not values:
            return None
        return {"median": statistics.median, "max": max, "mean": statistics.fmean}[self.config.reduce](values)

    def __repr__(self) -> str:
        return f"PerRun({self.inner!r}, reduce={self.config.reduce!r})"


# ---- amendment 4: other inputs than a recording's tracked pair ---------------------------------------------------


@dataclass(frozen=True)
class Repeat:
    """One repeat of a command: what it repeats (its identity) and the pair it produced."""

    command: tuple
    target: JointTrajectory
    measured: JointTrajectory


@dataclass(frozen=True)
class RepeatabilityConfig:
    k: float = 3.0               # ISO 9283 RP = mean(l) + k * S_l
    include_last_dwell: bool = True


class Repeatability(Metric):
    """ISO 9283 pose repeatability at the stops of same-input repeats: for each stop index, l = |e - mean(e)| over the
    repeats' signed stop errors, RP = mean(l) + k * std(l, ddof=1); the value is the worst stop index (signal unit)."""

    name = "repeatability"
    description = "How far same-input repeats land from their own mean at each stop: ISO 9283 RP, the worst stop."
    references = (ISO9283 + " §7.2.2 pose repeatability RP", "robot_msgs.compare.aggregate (ISO 9283 over a grid)")
    Input = "Repeat"
    Config = RepeatabilityConfig
    hint = "distribution"
    memory = "O(repeats x stops)"
    live = False

    def setup(self) -> None:
        self.add_state("errors", [], "cat")
        self.command: tuple | None = None

    def update(self, repeat: Repeat) -> None:  # type: ignore[override]
        if self.command is not None and repeat.command != self.command:
            raise ValueError(f"{repeat.command} is not a repeat of {self.command}: a repeatability needs one input")
        self.command = repeat.command
        self.updates += 1
        found = stock.stops(repeat.target, repeat.measured, include_last_dwell=self.config.include_last_dwell)
        self.errors = [*self.errors, [stop.error for stop in found]]

    def compute(self) -> float | None:
        if len(self.errors) < 2 or len({len(errors) for errors in self.errors}) != 1:
            return None
        matrix = np.asarray(self.errors)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return float(np.max(distance.mean(axis=0) + self.config.k * distance.std(axis=0, ddof=1)))


@dataclass(frozen=True)
class HeldOutPair:
    """The same held-out runs played twice: with the nominal values and with the fitted ones."""

    nominal: tuple
    fitted: tuple


@dataclass(frozen=True)
class PerformanceGainConfig:
    sign: str = "lower_is_better"    # gain = nominal - fitted (lower_is_better) or fitted - nominal


class PerformanceGain(Metric):
    """What a calibration bought, on a part-type alias: the alias on held-out runs with nominal values, again with
    fitted values, the difference. It reuses the alias by identity: both sides are a clone of the alias's metric."""

    name = "performance gain"
    description = "An alias computed on held-out runs with nominal and with fitted values; nominal minus fitted."
    references = ("hold-out validation (the round's held-out plays)", "robot_models select_best (held-out rms)")
    Input = "HeldOutPair"
    Config = PerformanceGainConfig
    live = False

    def __init__(self, alias: Any, signals: Any, **config: Any) -> None:
        self.alias, self.signals = alias, signals
        super().__init__(**config)

    @property
    def identity(self) -> tuple:
        return type(self).__name__, (self.alias.key(), *super().identity[1])

    def setup(self) -> None:
        self.add_state("sides", [], "cat")

    def update(self, pair: HeldOutPair) -> None:  # type: ignore[override]
        from .collection import MetricCollection

        self.updates += 1
        sides = []
        for runs in (pair.nominal, pair.fitted):
            collection = MetricCollection((self.alias,), self.signals)
            for recording in runs:
                collection.update(recording)
                collection.end_run()
            sides.append(collection.compute()[self.alias.name])
        self.sides = [*self.sides, tuple(sides)]

    def compute(self) -> float | None:
        if not self.sides:
            return None
        nominal, fitted = self.sides[-1]
        return nominal - fitted if self.config.sign == "lower_is_better" else fitted - nominal
