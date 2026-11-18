"""Metric objects with the three verbs Keras and torchmetrics already use: update, compute, reset.

    stop_worst = StopError("tip stop worst", statistic="worst", unit="mm", scale=339.0)
    stop_worst.update(run.target, run.measured)      # offline: the whole recording in one call
    stop_worst.compute()                             # a plain float in the metric's unit (mm here)
    stop_worst.reset()                               # forget everything
    for start in range(0, len(run.measured), 7):     # live: the same metric fed 7 rows at a time
        stop_worst.update(run.target[start:start + 7], run.measured[start:start + 7])

Prior art, copied on purpose: Keras metrics (``update_state`` / ``result`` / ``reset_state``) and torchmetrics
(``update`` / ``compute`` / ``reset``; ``MetricCollection`` and its compute groups, which ``MetricDict`` follows).
torchmetrics itself is not imported: it costs 3.9 s and 450 MB at import (round 1's measurement); numpy does the work.

How a metric reads its input. ``update(target, measured)`` takes two robot_msgs streams sampled at the same times
(round 1's fixture puts the target at each reply's instant; a stream that is not aligned is refused, never
resampled here). Rows arrive in time order. Successive calls continue one run; ``new_run=True`` starts the next run
inside the same metric (a rung cell's plays, one after another), and a chunk that goes back in time without it is
an error, so two recordings are never silently glued together.

How several runs combine is each metric's own, stated rule (``across_runs``): tracking RMS is the median of the
runs' RMS (the scorecard's rule), a peak is the worst row of any run, and every stop metric pools the stops of all
the runs (the rung's median stop is the median of all its stops, never a median of medians).

Memory. Tracking keeps running sums (constant memory). Stop metrics keep one small record per stop (its error, its
settle time at each band asked for, its overshoot) plus the rows of the move in progress; a median needs every stop,
so that list grows with the stop count, never with the row count. ``Lag`` and ``Repeatability`` keep whole runs and
say so.

The math is not new: stops come from round 1's ``stock.StopTracker`` (the proposed ``robot_msgs.compare.stops``),
overshoot from ``robot_msgs.compare.goal``, lag from ``robot_msgs.compare.estimate_time_offset``, a pose's error from
``robot_msgs.compare.compare``. Units live on the configured object (``unit``, ``scale``); the math stays in SI.
"""

from __future__ import annotations

import copy
import math
import statistics
from collections.abc import Iterable, Sequence
from typing import Any

import numpy as np
from robot_msgs.compare import compare, estimate_time_offset, goal
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory, concatenate

import stock  # round 1's stop rule, one row at a time (live and offline share it)

ROBOT_MSGS = "robot_msgs.compare (ErrorStats, goal, estimate_time_offset)"
T4 = "bam_rack_twin acceptance T4: a stop is a dwell of >= 0.25 s; its error is the mean of the last 50 ms"
STEPINFO = "MATLAB stepinfo (settling time, overshoot)"
ISO9283 = "ISO 9283:1998, performance criteria for industrial robots"
KERAS = "keras.metrics (update_state / result / reset_state)"
TORCHMETRICS = "torchmetrics (update / compute / reset, MetricCollection compute groups)"


class Metric:
    """One number from robot_msgs streams: ``update`` as rows arrive, ``compute`` at any time, ``reset`` to start over.

    Every metric carries Zach's fields (name, description, references) and is a number or a bool. A target or
    threshold never lives here: see ``threshold.Threshold``.
    """

    description = ""
    references: tuple[str, ...] = ()
    value_type = "number"
    view_hint = "kpi_trend"          # which data-ui view shows it (round 1's _viz.HINTS)
    reads = "target, measured"       # the two streams update() takes
    across_runs = "pooled"           # how several runs in one metric combine

    def __init__(self, name: str, *, unit: str = "", scale: float = 1.0) -> None:
        self.name, self.unit, self.scale = name, unit, float(scale)
        self.reset()

    @property
    def label(self) -> str:
        """The name with its unit, the key a MetricDict files it under: "tip stop worst (mm)"."""

        return f"{self.name} ({self.unit})" if self.unit else self.name

    def update(self, target: Any, measured: Any, *, new_run: bool = False) -> None:
        raise NotImplementedError

    def compute(self) -> Any:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def config(self) -> dict[str, Any]:
        """The constructor settings that decide the math (not the name, unit or scale)."""

        return {}

    def card(self) -> dict[str, Any]:
        """Zach's Metric fields, as a record."""

        return {"name": self.label, "description": self.description, "references": list(self.references),
                "function": f"{type(self).__name__}({_arguments(self.config())})", "unit": self.unit,
                "target": None, "value_type": self.value_type, "reads": self.reads,
                "across_runs": self.across_runs, "view_hint": self.view_hint}

    def __repr__(self) -> str:
        return f"{type(self).__name__}({_arguments(self.config())})"


def _arguments(config: dict[str, Any]) -> str:
    return ", ".join(f"{key}={value!r}" for key, value in config.items())


class _Clock:
    """Refuses rows that go back in time inside one run (two recordings glued together by mistake)."""

    def __init__(self) -> None:
        self.last_t: float | None = None

    def advance(self, t: np.ndarray, new_run: bool) -> None:
        if new_run:
            self.last_t = None
        if self.last_t is not None and float(t[0]) <= self.last_t:
            raise ValueError(f"rows went back in time ({float(t[0]):.3f} s after {self.last_t:.3f} s): pass "
                             "new_run=True for the next recording, or reset() the metric")
        self.last_t = float(t[-1])


def error_rows(target: Any, measured: Any, field: str = "positions", joint: int = 0) -> np.ndarray:
    """measured - target on each row: one joint's column, or a pose's position distance (robot_msgs.compare)."""

    if isinstance(target, PoseTrajectory):
        return np.linalg.norm(compare(target, measured).error.positions, axis=1)
    if len(target) != len(measured) or not np.array_equal(target.t, measured.t):
        raise ValueError("target and measured must be sampled at the same times: align them first "
                         "(robot_msgs.compare.align), this metric never resamples")
    return np.asarray(getattr(measured, field)[:, joint], float) - np.asarray(getattr(target, field)[:, joint], float)


class TrackingError(Metric):
    """The error target vs measured on every row: its RMS or its peak."""

    description = ("measured - target on each row; statistic='rms' is the root mean square, statistic='peak' the "
                   "largest |error|; while_moving=True reads only the rows whose commanded speed is not zero.")
    references = (ROBOT_MSGS, "scikit-learn mean_squared_error", KERAS)

    def __init__(self, name: str, *, statistic: str = "rms", while_moving: bool = False, field: str = "positions",
                 joint: int = 0, across_runs: str = "median", unit: str = "", scale: float = 1.0) -> None:
        if statistic not in ("rms", "peak"):
            raise ValueError(f"statistic is 'rms' or 'peak', not {statistic!r}")
        if across_runs not in ("median", "pooled"):
            raise ValueError(f"across_runs is 'median' (of the runs' values) or 'pooled' (every row), not {across_runs!r}")
        self.statistic, self.while_moving, self.field, self.joint = statistic, while_moving, field, joint
        self.across_runs = "max" if statistic == "peak" else across_runs
        super().__init__(name, unit=unit, scale=scale)

    def config(self) -> dict[str, Any]:
        config: dict[str, Any] = {"statistic": self.statistic}
        if self.while_moving:
            config["while_moving"] = True
        if self.field != "positions":
            config["field"] = self.field
        if self.joint:
            config["joint"] = self.joint
        if self.statistic == "rms" and self.across_runs != "median":
            config["across_runs"] = self.across_runs
        return config

    def reset(self) -> None:
        self.clock = _Clock()
        self.finished: list[float] = []          # one value per finished run
        self.pooled_squares, self.pooled_count = 0.0, 0
        self._start_run()

    def _start_run(self) -> None:
        self.squares, self.count, self.peak = 0.0, 0, 0.0

    def _run_value(self) -> float:
        return math.sqrt(self.squares / self.count) if self.statistic == "rms" else self.peak

    def update(self, target: Any, measured: Any, *, new_run: bool = False) -> None:
        if not len(measured):
            return
        if new_run and self.count:
            self.finished.append(self._run_value())
            self._start_run()
        self.clock.advance(np.asarray(measured.t, float), new_run)
        error = error_rows(target, measured, self.field, self.joint)
        if self.while_moving:
            error = error[stock.moving(target, self.joint)]
        error = error[np.isfinite(error)]
        # WHY a running sum in row order: 7-row chunks and the whole recording then add the same numbers in the same
        # order, so live and offline agree exactly, not just to 1e-15.
        for square in (error * error).tolist():
            self.squares += square
            self.pooled_squares += square
        self.count += int(error.size)
        self.pooled_count += int(error.size)
        if error.size:
            self.peak = max(self.peak, float(np.max(np.abs(error))))

    def compute(self) -> float | None:
        values = self.finished + ([self._run_value()] if self.count else [])
        if not values:
            return None
        if self.statistic == "peak":
            return self.scale * max(values)
        if self.across_runs == "pooled":
            return self.scale * math.sqrt(self.pooled_squares / self.pooled_count)
        return self.scale * statistics.median(values)


class StopRecord:
    """What is kept of one settled stop: its signed error (rad), its settle time per band asked for, its overshoot."""

    __slots__ = ("error", "settle_s", "overshoot")

    def __init__(self, error: float, settle_s: dict[tuple[float, float], float], overshoot: float | None) -> None:
        self.error, self.settle_s, self.overshoot = error, settle_s, overshoot


class StopFinder:
    """The stops of one or more runs, found one row at a time with round 1's stock.StopTracker.

    Several stop metrics may share one finder (a MetricDict arranges it), so the stops are found once per row however
    many metrics read them. Each closed stop is kept as a StopRecord; the move rows behind it are dropped.
    """

    def __init__(self, *, joint: int = 0, include_last_dwell: bool = True) -> None:
        self.joint, self.include_last_dwell = joint, include_last_dwell
        self.bands: set[tuple[float, float]] = set()     # (within, scale) pairs a settle metric asked for
        self.wants_overshoot = False
        self.reset()

    def reset(self) -> None:
        self.records: list[StopRecord] = []
        self.tracker: stock.StopTracker | None = None
        self.clock = _Clock()
        self.chunks_read = 0

    def update(self, target: JointTrajectory, measured: JointTrajectory, *, new_run: bool = False) -> None:
        if not len(measured):
            return
        if new_run and self.tracker is not None:
            self.records += self._records(self.tracker.finish())
            self.tracker = None
        self.clock.advance(np.asarray(measured.t, float), new_run)
        if len(target) != len(measured) or not np.array_equal(target.t, measured.t):
            raise ValueError("target and measured must be sampled at the same times: align them first")
        if self.tracker is None:
            self.tracker = stock.StopTracker(joint=target.joint_names[self.joint],
                                             include_last_dwell=self.include_last_dwell)
        column = self.joint
        found = self.tracker.push_many(np.asarray(measured.t, float), np.asarray(measured.positions[:, column], float),
                                       np.asarray(target.positions[:, column], float),
                                       np.asarray(target.velocities[:, column], float))
        self.records += self._records(found)
        self.chunks_read += 1

    def stops(self) -> list[StopRecord]:
        """Every stop so far; the dwell in progress counts as if the run ended now (compute never changes the state)."""

        if self.tracker is None:
            return list(self.records)
        return self.records + self._records(copy.deepcopy(self.tracker).finish())

    def _records(self, found: Sequence[stock.Stop]) -> list[StopRecord]:
        records = []
        for stop in found:
            settle = {(band, scale): stock.settle_time(stop, band, scale) for band, scale in self.bands}
            overshoot = None
            if self.wants_overshoot:
                reference, measured = stop.move()
                overshoot = goal(reference, measured, band=math.inf)[f"positions/{stop.joint}"].overshoot
            records.append(StopRecord(stop.error, settle, overshoot))
        return records


class StopMetric(Metric):
    """A metric of the stops. On its own it owns a StopFinder; inside a MetricDict it shares one."""

    across_runs = "pooled (every stop)"
    view_hint = "distribution"      # data-ui's stock view for a spread of values (round 1's _viz.HINTS)
    references = (T4, ISO9283 + " pose accuracy")

    def __init__(self, name: str, *, unit: str = "", scale: float = 1.0, include_last_dwell: bool = True) -> None:
        self.finder = StopFinder(include_last_dwell=include_last_dwell)
        self.ask(self.finder)
        super().__init__(name, unit=unit, scale=scale)

    def ask(self, finder: StopFinder) -> None:
        """Tell a finder what this metric needs it to keep for every stop."""

    def update(self, target: Any, measured: Any, *, new_run: bool = False) -> None:
        self.finder.update(target, measured, new_run=new_run)

    def reset(self) -> None:
        self.finder.reset()

    def values(self) -> list[Any]:
        """One value per stop, in this metric's unit: what compute() summarises."""

        raise NotImplementedError

    def view(self, title: str = "", threshold: Any = None) -> Any:
        """Every stop as a standard view (round 1's _viz.View: matplotlib, a W&B payload, Rerun calls). The
        threshold, when given, is drawn as a line; it still never enters compute()."""

        import _viz   # round 1's shared adapter; matplotlib and rerun load only inside its sinks

        values = [float(value) for value in self.values()]
        return _viz.View(title=title or self.label, unit=self.unit, hint=self.view_hint, x=list(range(len(values))),
                         y=values, target=None if threshold is None else threshold.at_most,
                         target_label="" if threshold is None else f"{threshold.at_most:g} {threshold.unit} "
                                                                   f"({threshold.owner})",
                         x_label="stop", y_label=self.unit)


class StopError(StopMetric):
    """The error left at each settled stop, summarised over every stop."""

    description = ("|mean(measured - target)| over the last 50 ms of each stop (a dwell of at least 0.25 s at zero "
                   "commanded speed that a move led into); statistic='median', 'worst' or 'p90' over every stop.")

    def __init__(self, name: str, *, statistic: str = "median", unit: str = "", scale: float = 1.0,
                 include_last_dwell: bool = True) -> None:
        if statistic not in ("median", "worst", "p90"):
            raise ValueError(f"statistic is 'median', 'worst' or 'p90', not {statistic!r}")
        self.statistic = statistic
        super().__init__(name, unit=unit, scale=scale, include_last_dwell=include_last_dwell)

    def config(self) -> dict[str, Any]:
        return {"statistic": self.statistic}

    def values(self) -> list[float]:
        return [self.scale * abs(record.error) for record in self.finder.stops()]

    def compute(self) -> float | None:
        values = self.values()
        if not values:
            return None
        if self.statistic == "median":
            return float(statistics.median(values))
        if self.statistic == "worst":
            return max(values)
        return float(np.percentile(values, 90.0))


class StopCount(StopMetric):
    """How many settled stops the runs had."""

    description = "The number of stops: dwells of at least 0.25 s at zero commanded speed that a move led into."

    def values(self) -> list[int]:
        return [1 for _ in self.finder.stops()]

    def compute(self) -> int:
        return len(self.finder.stops())


class SettleTime(StopMetric):
    """From each move's end until the error stays within ``within`` (in ``within_unit``) for good."""

    description = ("Seconds from each move's end to the first row after which |error| stays within the limit; 0 if "
                   "within from the start, inf if never within by the dwell's end; the median over every stop.")
    references = (STEPINFO, T4)

    def __init__(self, name: str, *, within: float, within_unit: str = "", scale: float = 1.0,
                 include_last_dwell: bool = True) -> None:
        self.within, self.within_unit = float(within), within_unit
        self.within_scale = float(scale)    # radians at the joint -> within_unit (mm at a tip: 1000 x its length)
        super().__init__(name, unit="s", scale=1.0, include_last_dwell=include_last_dwell)

    def ask(self, finder: StopFinder) -> None:
        finder.bands.add((self.within, self.within_scale))

    def config(self) -> dict[str, Any]:
        return {"within": self.within, "within_unit": self.within_unit} if self.within_unit else {"within": self.within}

    def values(self) -> list[float]:
        return [record.settle_s[(self.within, self.within_scale)] for record in self.finder.stops()]

    def compute(self) -> float | None:
        values = self.values()
        return float(statistics.median(values)) if values else None    # inf: the median stop never settled


class AllStopsSettled(SettleTime):
    """True when every stop got within the limit and stayed there."""

    description = "True when every stop got within the limit by the end of its dwell and stayed there."
    value_type = "bool"

    def __init__(self, name: str, *, within: float, within_unit: str = "", scale: float = 1.0,
                 include_last_dwell: bool = True) -> None:
        super().__init__(name, within=within, within_unit=within_unit, scale=scale,
                         include_last_dwell=include_last_dwell)
        self.unit = ""

    def compute(self) -> bool:
        return all(math.isfinite(value) for value in self.values())


class Overshoot(StopMetric):
    """How far the response went past each stop's target along the move into it; the largest, 0 with none."""

    description = "How far past each stop's target the joint went on the move into it (robot_msgs.compare.goal)."
    references = (STEPINFO, ROBOT_MSGS)

    def ask(self, finder: StopFinder) -> None:
        finder.wants_overshoot = True

    def values(self) -> list[float]:
        return [self.scale * record.overshoot for record in self.finder.stops()]

    def compute(self) -> float:
        return max(self.values(), default=0.0)


class CodeShare(Metric):
    """The share of rows whose code channel holds one of ``codes`` (moteus limiter codes, by default)."""

    description = "The share of rows whose code channel holds one of the codes, over every row of every run."
    references = ("moteus fault and limiter codes (mjbots reference)", "RobotDx stock.fault_codes")

    def __init__(self, name: str, *, codes: tuple[int, ...] = (97, 98, 99, 102), channel: str = "fault",
                 unit: str = "", scale: float = 1.0) -> None:
        self.codes, self.channel = tuple(codes), channel
        super().__init__(name, unit=unit, scale=scale)

    def config(self) -> dict[str, Any]:
        return {"codes": self.codes}

    def reset(self) -> None:
        self.clock = _Clock()
        self.hits, self.rows = 0, 0

    def update(self, target: Any, measured: Any, *, new_run: bool = False) -> None:
        if not len(measured):
            return
        self.clock.advance(np.asarray(measured.t, float), new_run)
        codes = np.asarray(measured.channels[self.channel]).astype(int)
        self.hits += int(np.isin(codes, self.codes).sum())
        self.rows += int(codes.size)

    def compute(self) -> float | None:
        return self.scale * self.hits / self.rows if self.rows else None


class Lag(Metric):
    """How late the response runs behind the plan as sent: robot_msgs.compare.estimate_time_offset.

    It reads the plan on its own ticks (``update(run.plan, run.measured)``), not the target at each reply. A
    cross-correlation needs the whole run, so this metric keeps every row it is given: offline in spirit, even when
    fed in chunks. Across runs: the median of the runs' lags.
    """

    description = "The time shift that best lines the response up with the plan (cross-correlation, refined)."
    references = (ROBOT_MSGS, "trajectopy time-shift estimation")
    reads = "plan, measured"
    across_runs = "median"

    def reset(self) -> None:
        self.plan_clock, self.measured_clock = _Clock(), _Clock()
        self.finished: list[float] = []
        self.plan_rows: list[JointTrajectory] = []
        self.measured_rows: list[JointTrajectory] = []

    def update(self, plan: JointTrajectory, measured: JointTrajectory, *, new_run: bool = False) -> None:
        if new_run and self.measured_rows:
            self.finished.append(self._offset())
            self.plan_rows, self.measured_rows = [], []
        if len(plan):
            self.plan_clock.advance(np.asarray(plan.t, float), new_run)
            self.plan_rows.append(plan)
        if len(measured):
            self.measured_clock.advance(np.asarray(measured.t, float), new_run)
            self.measured_rows.append(measured)

    def _offset(self) -> float:
        plan = concatenate(self.plan_rows) if len(self.plan_rows) > 1 else self.plan_rows[0]
        measured = concatenate(self.measured_rows) if len(self.measured_rows) > 1 else self.measured_rows[0]
        return estimate_time_offset(plan, measured).offset_s

    def compute(self) -> float | None:
        values = self.finished + ([self._offset()] if self.measured_rows else [])
        return self.scale * float(statistics.median(values)) if values else None


class Repeatability(Metric):
    """ISO 9283 pose repeatability at the stops of same-input repeats; each update() is one whole repeat.

    For each stop index: l = |e - mean(e)| over the repeats' signed stop errors, RP = mean(l) + k * std(l, ddof=1).
    The value is the worst stop index, in the metric's unit. Keeps every repeat's stop errors.
    """

    description = "How far same-input repeats land from their own mean at each stop (ISO 9283 RP, the worst stop)."
    references = (ISO9283 + " section 7.2.2, pose repeatability RP",)
    reads = "one whole repeat per update (target, measured)"
    across_runs = "each update is one repeat"

    def __init__(self, name: str, *, k: float = 3.0, unit: str = "", scale: float = 1.0) -> None:
        self.k = float(k)
        super().__init__(name, unit=unit, scale=scale)

    def config(self) -> dict[str, Any]:
        return {"k": self.k}

    def reset(self) -> None:
        self.errors: list[list[float]] = []

    def update(self, target: JointTrajectory, measured: JointTrajectory, *, new_run: bool = True) -> None:
        self.errors.append([stop.error for stop in stock.stops(target, measured)])

    def compute(self) -> float | None:
        if len(self.errors) < 2 or len({len(errors) for errors in self.errors}) != 1:
            return None
        matrix = np.asarray(self.errors)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return self.scale * float(np.max(distance.mean(axis=0) + self.k * distance.std(axis=0, ddof=1)))


class PerformanceGain(Metric):
    """What a calibration bought: one metric on held-out runs played with nominal values, again with fitted values.

    ``update(nominal_runs, fitted_runs)``; ``compute()`` is nominal minus fitted (lower is better). Each side is a
    fresh copy of the given metric, fed each run as its own run.
    """

    description = "A metric on held-out runs with nominal values minus the same metric with fitted values."
    references = ("hold-out validation", "robot_models select_best (held-out RMS)")
    reads = "(runs played with nominal values, runs played with fitted values)"
    across_runs = "each side pools its runs as the metric says"

    def __init__(self, metric: Metric) -> None:
        self.metric = metric
        super().__init__(f"{metric.name} gain", unit=metric.unit)

    def config(self) -> dict[str, Any]:
        return {"metric": self.metric}

    def reset(self) -> None:
        self.nominal: float | None = None
        self.fitted: float | None = None

    def side(self, runs: Iterable[Any]) -> float | None:
        metric = copy.deepcopy(self.metric)
        metric.reset()
        for run in runs:
            metric.update(run.target, run.measured, new_run=True)
        return metric.compute()

    def update(self, nominal_runs: Iterable[Any], fitted_runs: Iterable[Any], *, new_run: bool = False) -> None:
        self.nominal, self.fitted = self.side(nominal_runs), self.side(fitted_runs)

    def compute(self) -> float | None:
        if self.nominal is None or self.fitted is None:
            return None
        return self.nominal - self.fitted


class MetricDict(dict):
    """A dict of metrics keyed by their label, with update / compute / reset run on every one.

    torchmetrics' MetricCollection, compute groups included: the stop metrics in one dict share one StopFinder, so a
    chunk's stops are found once however many stop metrics read them. Update the dict, not a member, once they share.
    ``plan=`` feeds the metrics that read the plan as sent (``Lag``); without it they stay empty (None).
    """

    def __init__(self, metrics: Iterable[Metric]) -> None:
        metrics = list(metrics)
        super().__init__((metric.label, metric) for metric in metrics)
        if len(self) != len(metrics):
            raise ValueError("two metrics share a label")
        shared: dict[tuple[int, bool], StopFinder] = {}
        for metric in metrics:
            if isinstance(metric, StopMetric):
                rule = (metric.finder.joint, metric.finder.include_last_dwell)
                finder = shared.setdefault(rule, StopFinder(joint=rule[0], include_last_dwell=rule[1]))
                metric.ask(finder)
                metric.finder = finder
        self.finders = list(shared.values())

    def parts(self) -> list[Any]:
        """What one update actually runs: each shared stop finder once, then every metric that is not a stop metric."""

        return [*self.finders, *(metric for metric in self.values() if not isinstance(metric, StopMetric))]

    def update(self, target: Any, measured: Any, *, plan: Any = None, new_run: bool = False) -> None:
        for part in self.parts():
            if getattr(part, "reads", "") == "plan, measured":
                if plan is not None:
                    part.update(plan, measured, new_run=new_run)
            else:
                part.update(target, measured, new_run=new_run)

    def compute(self) -> dict[str, Any]:
        return {label: metric.compute() for label, metric in self.items()}

    def reset(self) -> None:
        for part in self.parts():
            part.reset()

    def __str__(self) -> str:
        width = max(len(label) for label in self)
        return "\n".join(f"{label:<{width}}  {metric!r}" for label, metric in self.items())

    def __repr__(self) -> str:
        return f"MetricDict({list(self)!r})"
