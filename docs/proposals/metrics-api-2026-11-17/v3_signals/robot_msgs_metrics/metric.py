"""Signal metrics: torchmetrics' Metric with numpy inside, and no idea what the signal means.

    RootMeanSquare(rows="moving")             sqrt(mean(e^2)) of e = target - measured     state: (sum e^2, n)
    MaxAbsolute()                             max |e|                                      state: running max
    Stops()                                   the stops of the pair (stock.StopTracker)    state: open dwell + closed stops
    StopStatistic(of="error", statistic=...)  a statistic of every stop's error / settle / overshoot (reads a Stops)
    CrossCorrelationLag()                     estimate_time_offset of the raw command vs state   state: the run (cat)
    CodeShare(codes=...)                      share of state rows whose code channel holds a code  state: (hits, rows)

Each metric's ``items()`` is what a pooled across-run statistic reads (per-stop values, per-row flags); ``compute()`` is
the one-run value. Functional twins: ``Metric.functional(target, measured, **config)``.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import asdict, astuple, dataclass
from typing import Any, ClassVar

import numpy as np
from robot_msgs.compare import compare, estimate_time_offset, goal
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory, concatenate

import stock


class Metric:
    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    references: ClassVar[tuple[str, ...]] = ()
    value_type: ClassVar[str] = "number"
    hint: ClassVar[str] = "kpi_trend"
    reads: ClassVar[str] = "aligned"      # aligned (target, measured) pairs | raw (the command and state topics)
    Input: ClassVar[str] = "AlignedPair"  # what update() takes (the Plan's aligner makes it from a Recording)
    live: ClassVar[bool] = True
    Config: ClassVar[type]

    def __init__(self, **config: Any) -> None:
        self.config = self.Config(**config)
        self.reset()

    def reset(self) -> None:
        raise NotImplementedError

    def update(self, target: Any, measured: Any) -> None:
        raise NotImplementedError

    def finish(self) -> None:
        """The run ended (a pair never spans two runs)."""

    def compute(self) -> Any:
        raise NotImplementedError

    def items(self) -> list[float]:
        value = self.compute()
        return [] if value is None else [float(value)]

    @classmethod
    def functional(cls, target: Any, measured: Any, **config: Any) -> Any:
        metric = cls(**config)
        metric.update(target, measured)
        metric.finish()
        return metric.compute()

    @property
    def identity(self) -> tuple:
        return type(self).__name__, astuple(self.config)

    def clone(self) -> Metric:
        return type(self)(**asdict(self.config))

    def card(self) -> dict[str, Any]:
        return {"base": type(self).__name__, "config": ", ".join(f"{k}={v!r}" for k, v in asdict(self.config).items()),
                "description": self.description, "references": list(self.references), "value_type": self.value_type}

    def __repr__(self) -> str:
        return f"{type(self).__name__}({', '.join(f'{k}={v!r}' for k, v in asdict(self.config).items())})"


def _error(target: Any, measured: Any, field: str, joint: int, rows: str) -> np.ndarray:
    error = compare(target, measured).error
    values = (np.linalg.norm(error.positions, axis=1) if isinstance(error, PoseTrajectory)
              else np.asarray(getattr(error, field)[:, joint], float))
    if rows == "moving" and isinstance(target, JointTrajectory) and target.velocities is not None:
        values = values[stock.moving(target, joint)]
    return values[np.isfinite(values)]


@dataclass(frozen=True)
class ErrorConfig:
    rows: str = "all"
    field: str = "positions"
    joint: int = 0


class RootMeanSquare(Metric):
    name = "root mean square"
    description = "sqrt(mean(e^2)) of e = target - measured over the rows it reads."
    references = ("robot_msgs.compare.ErrorStats.rms", "evo APE rmse", "scikit-learn root_mean_squared_error")
    Config = ErrorConfig

    def reset(self) -> None:
        self.sum_squares, self.count = 0.0, 0

    def update(self, target: Any, measured: Any) -> None:
        error = _error(target, measured, self.config.field, self.config.joint, self.config.rows)
        self.sum_squares += float(np.sum(error * error))
        self.count += int(error.size)

    def compute(self) -> float | None:
        return math.sqrt(self.sum_squares / self.count) if self.count else None


class MaxAbsolute(Metric):
    name = "max absolute"
    description = "The largest |e| of e = target - measured."
    references = ("robot_msgs.compare.ErrorStats.max_abs", "evo APE max")
    Config = ErrorConfig

    def reset(self) -> None:
        self.peak: float | None = None

    def update(self, target: Any, measured: Any) -> None:
        error = _error(target, measured, self.config.field, self.config.joint, self.config.rows)
        if error.size:
            self.peak = max(self.peak or 0.0, float(np.max(np.abs(error))))

    def compute(self) -> float | None:
        return self.peak


@dataclass(frozen=True)
class StopRule:
    include_last_dwell: bool = True
    joint: int = 0


class Stops(Metric):
    """A shared node: every StopStatistic on one signal with one rule reads ONE of these."""

    name = "stops"
    description = "Every dwell (>= 0.25 s at zero commanded speed on one target) that a move led into."
    references = ("bam_rack_twin T4 stop rule", "stock.StopTracker")
    Config = StopRule

    def reset(self) -> None:
        self.tracker: stock.StopTracker | None = None
        self.closed: list[stock.Stop] = []

    def update(self, target: Any, measured: Any) -> None:
        if self.tracker is None:
            self.tracker = stock.StopTracker(joint=target.joint_names[self.config.joint],
                                             include_last_dwell=self.config.include_last_dwell)
        j = self.config.joint
        self.closed += self.tracker.push_many(measured.t, measured.positions[:, j], target.positions[:, j],
                                              target.velocities[:, j])

    def finish(self) -> None:
        if self.tracker is not None:
            self.closed += self.tracker.finish()
            self.tracker = None

    def compute(self) -> list[stock.Stop]:
        if self.tracker is None:
            return list(self.closed)
        import copy

        return [*self.closed, *copy.deepcopy(self.tracker).finish()]


@dataclass(frozen=True)
class StopStatisticConfig:
    of: str = "error"            # error | settle | overshoot | settled | count
    statistic: str = "median"    # median | max | p90 | all | count
    band: float = math.inf       # the signal's unit (settle, settled)
    include_last_dwell: bool = True
    joint: int = 0


class StopStatistic(Metric):
    name = "stop statistic"
    description = "A statistic over every stop of one quantity: |error| (last 50 ms), settle time, overshoot."
    references = ("bam_rack_twin T4", "MATLAB stepinfo", "robot_msgs.compare.goal", "ISO 9283 pose accuracy")
    Config = StopStatisticConfig
    hint = "distribution"

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self.source: Stops | None = None    # bound by the Plan to its shared Stops node

    @property
    def rule(self) -> StopRule:
        return StopRule(self.config.include_last_dwell, self.config.joint)

    @property
    def value_type(self) -> str:  # type: ignore[override]
        return "bool" if self.config.of == "settled" else "number"

    def reset(self) -> None:
        pass

    def update(self, target: Any, measured: Any) -> None:
        raise TypeError("a StopStatistic reads a Stops node; the Plan feeds that node, never this metric")

    def items(self) -> list[float]:
        found = self.source.compute() if self.source is not None else []
        if self.config.of == "error":
            return [abs(stop.error) for stop in found]
        if self.config.of == "settle":
            return [stock.settle_time(stop, self.config.band) for stop in found]
        if self.config.of == "settled":
            return [float(math.isfinite(stock.settle_time(stop, self.config.band))) for stop in found]
        if self.config.of == "overshoot":
            return [goal(*stop.move(), band=math.inf)[f"positions/{stop.joint}"].overshoot for stop in found]
        if self.config.of == "count":
            return [1.0 for _ in found]
        raise ValueError(f"of {self.config.of!r}")

    def compute(self) -> Any:
        return reduce_items(self.items(), self.config.statistic, empty=0.0 if self.config.of == "overshoot" else None)


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
    raise ValueError(f"statistic {statistic!r}")


@dataclass(frozen=True)
class LagConfig:
    max_offset_s: float = 0.25


class CrossCorrelationLag(Metric):
    name = "cross-correlation lag"
    description = "How late the state topic runs behind the command topic (robot_msgs.compare.estimate_time_offset)."
    references = ("robot_msgs.compare.estimate_time_offset", "scipy.signal.correlation_lags", "trajectopy")
    Config = LagConfig
    reads = "raw"
    Input = "RawTopics (command, state)"
    live = False

    def reset(self) -> None:
        self.command: list[Any] = []
        self.state: list[Any] = []

    def update(self, command: Any, state: Any) -> None:
        if command is not None and len(command):
            self.command.append(command)
        if state is not None and len(state):
            self.state.append(state)

    def compute(self) -> float | None:
        if not self.command or not self.state:
            return None
        return estimate_time_offset(concatenate(self.command), concatenate(self.state),
                                    max_offset_s=self.config.max_offset_s).offset_s


@dataclass(frozen=True)
class CodeShareConfig:
    codes: tuple[int, ...] = (97, 98, 99, 102)
    channel: str = "fault"


class CodeShare(Metric):
    name = "code share"
    description = "The share of state rows whose code channel holds one of the codes."
    references = ("moteus fault and limiter codes", "RobotDx stock.fault_codes")
    Config = CodeShareConfig

    def reset(self) -> None:
        self.flags: list[float] = []

    def update(self, target: Any, measured: Any) -> None:
        codes = np.asarray(measured.channels[self.config.channel]).astype(int)
        self.flags += np.isin(codes, self.config.codes).astype(float).tolist()

    def items(self) -> list[float]:
        return list(self.flags)

    def compute(self) -> float | None:
        return float(np.mean(self.flags)) if self.flags else None


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
    description = "ISO 9283 RP at each stop over same-input repeats: mean(l) + k S_l, the worst stop index."
    references = ("ISO 9283:1998 §7.2.2", "robot_msgs.compare.aggregate")
    Input = "Repeat"
    Config = RepeatabilityConfig
    live = False

    def reset(self) -> None:
        self.command: tuple | None = None
        self.errors: list[list[float]] = []

    def update(self, repeat: Repeat) -> None:  # type: ignore[override]
        if self.command is not None and repeat.command != self.command:
            raise ValueError(f"{repeat.command} is not a repeat of {self.command}")
        self.command = repeat.command
        self.errors.append([stop.error for stop in stock.stops(repeat.target, repeat.measured,
                                                               include_last_dwell=self.config.include_last_dwell)])

    def compute(self) -> float | None:
        if len(self.errors) < 2 or len({len(e) for e in self.errors}) != 1:
            return None
        matrix = np.asarray(self.errors)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return float(np.max(distance.mean(axis=0) + self.config.k * distance.std(axis=0, ddof=1)))


@dataclass(frozen=True)
class HeldOutPair:
    nominal: tuple
    fitted: tuple


@dataclass(frozen=True)
class PerformanceGainConfig:
    sign: str = "lower_is_better"


class PerformanceGain(Metric):
    """An alias on held-out recordings played with nominal and with fitted values; nominal - fitted. The alias's node
    is compiled once and fed every held-out run of a side (its sufficient statistics pool)."""

    name = "performance gain"
    description = "An alias on held-out runs with nominal and with fitted values; nominal minus fitted."
    references = ("hold-out validation", "robot_models select_best")
    Input = "HeldOutPair"
    Config = PerformanceGainConfig
    live = False

    def __init__(self, alias: Any, signals: Any, **config: Any) -> None:
        self.alias, self.signals = alias, signals
        super().__init__(**config)

    def reset(self) -> None:
        self.sides: tuple[float, float] | None = None

    def update(self, pair: HeldOutPair) -> None:  # type: ignore[override]
        from .align import Aligner
        from .plan import Plan, display

        sides = []
        for recordings in (pair.nominal, pair.fitted):
            plan = Plan.compile((self.alias,), self.signals)
            node = plan.nodes[(self.alias.signal, self.alias.metric.identity)]
            for recording in recordings:      # the node is NOT reset between runs: its (sum e^2, n) pools them
                for spec in self.signals.values():
                    plan.push(spec[0], recording.topics[spec[0]])
                    plan.push(spec[1], recording.topics[spec[1]])
                plan.finish()
                plan.aligners = {name: Aligner(*spec) for name, spec in self.signals.items()}   # a fresh pairing per run
            sides.append(display(self.alias, node.compute()))
        self.sides = (sides[0], sides[1])

    def compute(self) -> float | None:
        if self.sides is None:
            return None
        return self.sides[0] - self.sides[1] if self.config.sign == "lower_is_better" else self.sides[1] - self.sides[0]
