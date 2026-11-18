"""The stock base metrics of ``robot_models.metrics``: math only, one functional each over (config, runs).

    ErrorStat(statistic="rms", rows="moving", across_runs="median")
    Dwell(quantity="error" | "settle" | "overshoot" | "settled" | "count", statistic=..., band=...)
    TimeOffset(across_runs="median")
    CodeShare(codes=(97, 98, 99, 102))

WHY one ``Dwell`` class with a ``quantity`` and not four subclasses: Zach's "most metrics you can get with a base metric
type plus a configuration"; the stops are found once per run (a second-level memo, ``STOP_PASSES``) and every quantity
reads them. A subclass is for what config cannot say (see the scenario's ``FeedbackShare``).
"""

from __future__ import annotations

import math
import statistics
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from robot_msgs.compare import compare, error_stats, estimate_time_offset, goal
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory

import stock

from .metric import Memo, Metric

STOP_PASSES = Memo()  # stop detection, once per (rule, run): every Dwell quantity reads the same stops
ROBOT_MSGS = "robot_msgs.compare (ErrorStats)"
T4 = "bam_rack_twin acceptance T4 (dwell >= 0.25 s, error = mean of its last 50 ms)"
STEPINFO = "MATLAB stepinfo; ROS 2 joint_trajectory_controller goal check"


def _across(values: Sequence[float], rule: str) -> float | None:
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {"median": statistics.median, "max": max, "mean": statistics.fmean}[rule](values)


def error_signal(target: Any, measured: Any, field: str, joint: int) -> np.ndarray:
    error = compare(target, measured).error
    if isinstance(error, PoseTrajectory):
        return np.linalg.norm(error.positions, axis=1)
    return np.asarray(getattr(error, field)[:, joint], float)


@dataclass(frozen=True)
class ErrorStatConfig:
    statistic: str = "rms"          # rms | max_abs | p95_abs | mean_abs
    rows: str = "all"               # all | moving
    field: str = "positions"
    joint: int = 0
    across_runs: str = "median"     # median | max | mean | pool (the statistic over every run's rows at once)


class ErrorStat(Metric):
    name = "error statistic"
    description = "One statistic of target - measured over the rows it is told to read, combined over runs."
    references = (ROBOT_MSGS, "RobotDx stock.tracking_error", "scikit-learn mean_squared_error")
    Config = ErrorStatConfig

    @classmethod
    def functional(cls, config: ErrorStatConfig, runs: Sequence[tuple[Any, Any]]) -> float | None:
        errors = []
        for target, measured in runs:
            error = error_signal(target, measured, config.field, config.joint)
            if config.rows == "moving" and isinstance(target, JointTrajectory) and target.velocities is not None:
                error = error[stock.moving(target)]
            errors.append(error[np.isfinite(error)])
        if config.across_runs == "pool":
            errors = [np.concatenate(errors)] if errors else []
        values = [getattr(error_stats(error), config.statistic) if error.size else None for error in errors]
        if config.statistic == "mean_abs":
            values = [float(np.mean(np.abs(error))) if error.size else None for error in errors]
        return _across(values, "max" if config.across_runs == "pool" else config.across_runs)


@dataclass(frozen=True)
class DwellConfig:
    quantity: str = "error"          # error (|stop error|) | settle (s) | overshoot | settled (bool) | count
    statistic: str = "median"        # median | max | p90 | all (settled) | sum (count)
    band: float = math.inf           # in the signal's unit; settle and settled only
    include_last_dwell: bool = True
    joint: int = 0


def stops_of(config: DwellConfig, key: str, target: Any, measured: Any) -> tuple[stock.Stop, ...]:
    rule = (config.include_last_dwell, config.joint)
    return STOP_PASSES.get((rule, key, len(measured)), lambda: stock.stops(
        target, measured, joint=config.joint, include_last_dwell=config.include_last_dwell))


class Dwell(Metric):
    name = "dwell"
    description = ("A quantity of every stop (a dwell >= 0.25 s at zero commanded speed on one target that a move led "
                   "into), pooled over the runs: its error over the last 50 ms, its settle time, its overshoot.")
    references = (T4, STEPINFO, "ISO 9283 pose accuracy")
    Config = DwellConfig
    hint = "distribution"

    @classmethod
    def functional(cls, config: DwellConfig, runs: Sequence[tuple[Any, Any]], keys: Sequence[str] = ()) -> Any:
        keys = list(keys) or [f"anonymous-{id(measured)}" for _, measured in runs]
        found = [stop for key, (target, measured) in zip(keys, runs) for stop in stops_of(config, key, target, measured)]
        return cls.summarize(config, found)

    @staticmethod
    def quantities(config: DwellConfig, found: Sequence[stock.Stop]) -> list[float]:
        if config.quantity == "error":
            return [abs(stop.error) for stop in found]
        if config.quantity == "settle":
            return [stock.settle_time(stop, config.band) for stop in found]
        if config.quantity == "settled":
            return [float(math.isfinite(stock.settle_time(stop, config.band))) for stop in found]
        if config.quantity == "overshoot":
            return [goal(*stop.move(), band=math.inf)[f"positions/{stop.joint}"].overshoot for stop in found]
        if config.quantity == "count":
            return [1.0 for _ in found]
        raise ValueError(f"quantity {config.quantity!r}")

    @classmethod
    def summarize(cls, config: DwellConfig, found: Sequence[stock.Stop]) -> Any:
        values = cls.quantities(config, found)
        if config.statistic == "all":
            return all(value == 1.0 for value in values)
        if config.statistic == "sum":
            return float(sum(values))
        if not values:
            return 0.0 if config.quantity == "overshoot" else None
        if config.statistic == "median":
            middle = statistics.median(values)
            return None if math.isinf(middle) else float(middle)
        if config.statistic == "max":
            kept = [v for v in values if math.isfinite(v)]
            return max(kept) if kept else None
        if config.statistic == "p90":
            return float(np.percentile(values, 90.0))
        raise ValueError(f"statistic {config.statistic!r}")

    def compute(self) -> Any:
        if not self.runs:
            return None
        keys = ["|".join(map(str, part)) for part in self.data_key()]
        return self._memo((self.identity, self.data_key()), lambda: type(self).functional(self.config, self._joined(), keys))

    @staticmethod
    def _memo(key: tuple, make: Any) -> Any:
        from .metric import MEMO

        return MEMO.get(key, make)

    @property
    def value_type(self) -> str:  # type: ignore[override]
        return "bool" if self.config.quantity == "settled" else "number"


@dataclass(frozen=True)
class TimeOffsetConfig:
    max_offset_s: float = 0.25
    across_runs: str = "median"


class TimeOffset(Metric):
    Input = "TrackedPair (plan, state)"
    name = "time offset"
    description = "How late the response's clock runs behind the command."
    references = ("robot_msgs.compare.estimate_time_offset", "trajectopy")
    Config = TimeOffsetConfig

    @classmethod
    def functional(cls, config: TimeOffsetConfig, runs: Sequence[tuple[Any, Any]]) -> float | None:
        return _across([estimate_time_offset(t, m, max_offset_s=config.max_offset_s).offset_s for t, m in runs],
                       config.across_runs)


@dataclass(frozen=True)
class CodeShareConfig:
    codes: tuple[int, ...] = (97, 98, 99, 102)
    channel: str = "fault"


class CodeShare(Metric):
    Input = "TrackedPair (the state's code channel)"
    name = "code share"
    description = "The share of rows, over every run, whose code channel holds one of the codes."
    references = ("moteus fault and limiter codes", "RobotDx stock.fault_codes")
    Config = CodeShareConfig

    @classmethod
    def functional(cls, config: CodeShareConfig, runs: Sequence[tuple[Any, Any]]) -> float | None:
        codes = [np.asarray(measured.channels[config.channel]).astype(int) for _, measured in runs]
        rows = sum(code.size for code in codes)
        return sum(int(np.isin(code, config.codes).sum()) for code in codes) / rows if rows else None


# ---- amendment 4: other inputs ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class Repeat:
    command: tuple
    target: Any
    measured: Any


@dataclass(frozen=True)
class RepeatabilityConfig:
    k: float = 3.0
    include_last_dwell: bool = True


class Repeatability(Metric):
    name = "repeatability"
    description = "ISO 9283 RP at each stop over same-input repeats (mean(l) + k S_l); the worst stop index."
    references = ("ISO 9283:1998 §7.2.2", "robot_msgs.compare.aggregate")
    Input = "Repeat"
    Config = RepeatabilityConfig
    live = False

    def update(self, repeat: Repeat) -> None:  # type: ignore[override]
        if self.runs and repeat.command != self.runs[0][0]:
            raise ValueError(f"{repeat.command} is not a repeat of {self.runs[0][0]}")
        self.runs.append((repeat.command, [(repeat.target, repeat.measured)]))

    @classmethod
    def functional(cls, config: RepeatabilityConfig, runs: Sequence[tuple[Any, Any]]) -> float | None:
        errors = [[stop.error for stop in stock.stops(t, m, include_last_dwell=config.include_last_dwell)]
                  for t, m in runs]
        if len(errors) < 2 or len({len(e) for e in errors}) != 1:
            return None
        matrix = np.asarray(errors)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return float(np.max(distance.mean(axis=0) + config.k * distance.std(axis=0, ddof=1)))

    def compute(self) -> float | None:
        return type(self).functional(self.config, [chunks[0] for _, chunks in self.runs])


@dataclass(frozen=True)
class HeldOutPair:
    nominal: tuple
    fitted: tuple


@dataclass(frozen=True)
class PerformanceGainConfig:
    sign: str = "lower_is_better"


class PerformanceGain(Metric):
    """The alias on held-out runs with nominal values, again with fitted ones: nominal - fitted. Reuses the alias by
    identity, so the memo serves a side another consumer already computed."""

    name = "performance gain"
    description = "An alias on held-out runs played with nominal and with fitted values; nominal minus fitted."
    references = ("hold-out validation", "robot_models select_best")
    Input = "HeldOutPair"
    Config = PerformanceGainConfig
    live = False

    def __init__(self, alias: Any, signals: Any, **config: Any) -> None:
        self.alias, self.signals = alias, signals
        super().__init__(**config)
        self.sides: tuple[float, float] | None = None

    def update(self, pair: HeldOutPair) -> None:  # type: ignore[override]
        from .metrics import Metrics

        sides = []
        for episodes in (pair.nominal, pair.fitted):
            collection = Metrics(self.alias, signals=self.signals)
            for episode in episodes:
                collection.update(episode)
                collection.end_run()
            sides.append(collection.compute()[self.alias.name])
        self.sides = (sides[0], sides[1])

    def compute(self) -> float | None:
        if self.sides is None:
            return None
        nominal, fitted = self.sides
        return nominal - fitted if self.config.sign == "lower_is_better" else fitted - nominal
