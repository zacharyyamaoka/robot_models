"""What a recording is measured for, and how each measure is summarised: the expressions of the group-by face.

    tip_stop_error = StopValue("tip stop error", "mm", short="tip stop", of="error", at="tip")
    tip_stop_error.median()                       # an expression: the median of every stop in the runs it is given
    tip_stop_error.max()                          # the worst stop
    tracking_error.rms()                          # every moving row of the runs, pooled
    tracking_error.rms().per_run().median()       # each run's RMS first, then the median of those

A **measure** turns one recording into values: one per stop, one per moving row, or one per run. A **summary** pools
those values over the runs of a group (median, max, rms, mean, share, count, all, any). ``Runs.group_by(...).agg(...)``
(runs_table.py) hands each summary its group's runs; ``Live`` hands it chunks of one recording as they arrive.

Every measure reads its recording through one shared computation (a "pass": the compare of target and measured, the
stop finder, a channel, ...). A pass runs once per recording and every measure reading it picks its values from the
result, so ten metrics over one recording cost four or five passes (``Runs.explain`` prints which).

WHY offline is the live code fed one chunk (Zach, 2026-11-17: "it doesn't matter if the sensor stream is online or
offline"): each pass is a reader with ``add(chunk)`` and ``finish()``, and each summary a running total with
``add(values)`` and ``value()``. Offline adds the whole recording once; live adds 7 rows at a time. One code path, so
the two agree to float round-off.

WHY no number from a model and no threshold here (G3, G5; Zach: "separate the calculation of the metric, and the
checking of thresholds"): a measure reads only the recording's streams and labels, and a gate is a ``Threshold``
compared with a table column afterwards (runs_table.py).

The arithmetic is not new: the error is ``robot_msgs.compare``'s, the stops are round 1's ``stock.StopTracker`` (the
proposed ``robot_msgs.compare.stops``), overshoot is ``robot_msgs.compare.goal`` and lag
``robot_msgs.compare.estimate_time_offset``. This module only names the readings and pools them.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, fields
from typing import Any, ClassVar

import numpy as np
from robot_msgs.compare import compare, estimate_time_offset, goal
from robot_msgs.trajectory import PoseTrajectory, concatenate

from stock import StopTracker, moving, settle_time

DEGREES = 180.0 / math.pi
LIMITER_CODES = (97, 98, 99, 102)     # moteus: the drive's torque, power, voltage or current limiter was active

# --------------------------------------------------------------------------- the shared passes, run once per recording

_PASSES: dict[tuple, list] = {}       # (recording, pass) -> everything the pass found in it
_VALUES: dict[tuple, np.ndarray] = {}  # (recording, measure) -> the measure's values in it
COMPUTED: Counter = Counter()          # recording -> how many passes actually ran over it


def recording_key(run: Any) -> tuple:
    return (run.deployment, run.source, run.id)


def values_of(measure: Measure, run: Any) -> np.ndarray:
    """One measure's values in one whole recording; its pass runs once, whichever measure asks first."""

    key = (recording_key(run), measure)
    if key not in _VALUES:
        pass_key = (recording_key(run), measure.pass_key)
        if pass_key not in _PASSES:
            reader = measure.reader()
            _PASSES[pass_key] = reader.add(run) + reader.finish()
            COMPUTED[recording_key(run)] += 1
        _VALUES[key] = measure.pick(_PASSES[pass_key], run)
    return _VALUES[key]


class _CompareReader:
    """robot_msgs.compare on each chunk: the error (reference - measured) on the measured rows, and the reference."""

    def __init__(self, streams: tuple[str, str]) -> None:
        self.streams = streams

    def add(self, chunk: Any) -> list:
        reference, measured = (getattr(chunk, name) for name in self.streams)
        if reference is None or measured is None or not len(measured):
            return []
        comparison = compare(reference, measured)
        if comparison.dropped:
            raise ValueError(f"compare dropped {comparison.dropped} rows: the two streams are not on one clock")
        return [(reference, comparison.error)]

    def finish(self) -> list:
        return []


class _StopReader:
    """Round 1's StopTracker, one row at a time: it holds only the rows since the last stop, so its memory is one move
    and one dwell, never the run. A dwell still open at a chunk boundary stays open until the next chunk."""

    def __init__(self, joint: int) -> None:
        self.joint, self.tracker = joint, None

    def add(self, chunk: Any) -> list:
        target, measured, joint = chunk.target, chunk.measured, self.joint
        if self.tracker is None:
            self.tracker = StopTracker(joint=target.joint_names[joint])
        return list(self.tracker.push_many(np.asarray(measured.t, float), np.asarray(measured.positions[:, joint], float),
                                           np.asarray(target.positions[:, joint], float),
                                           np.asarray(target.velocities[:, joint], float)))

    def finish(self) -> list:
        return list(self.tracker.finish()) if self.tracker is not None else []


class _ChannelReader:
    def __init__(self, channel: str, codes: tuple[int, ...]) -> None:
        self.channel, self.codes = channel, codes

    def add(self, chunk: Any) -> list:
        if not len(chunk.measured):
            return []
        codes = np.asarray(chunk.measured.channels[self.channel]).astype(int)
        return [np.isin(codes, self.codes)]

    def finish(self) -> list:
        return []


class _WholeRunReader:
    """For a reading that needs the whole recording (a cross-correlation): it keeps every chunk until finish()."""

    def __init__(self, compute: Any) -> None:
        self.compute, self.chunks = compute, []

    def add(self, chunk: Any) -> list:
        self.chunks.append(chunk)
        return []

    def finish(self) -> list:
        return [self.compute(self.chunks)] if self.chunks else []


# --------------------------------------------------------------------------- measures: what a recording is read for


@dataclass(frozen=True)
class Measure:
    """What is measured, its unit, and what it means. Its methods make the expressions that summarise it."""

    name: str                           # the Python name with spaces: "tip stop error" is tip_stop_error
    unit: str
    description: str = ""
    references: tuple[str, ...] = ()
    short: str = ""                     # the word a table column uses ("tip stop"); the name when empty
    input: ClassVar[str] = "Run: target and measured"
    live: ClassVar[bool] = True
    streams: ClassVar[tuple[str, ...]] = ("target", "measured")
    hint: ClassVar[str] = "distribution"     # the data-ui view that shows its values (PEP 0010 section 10)

    @property
    def code(self) -> str:
        return self.name.replace(" ", "_")

    @property
    def column(self) -> str:
        return self.short or self.name

    @property
    def config(self) -> dict[str, Any]:
        """The fields that change the arithmetic (its identity), without its name, unit or words."""

        skip = {"name", "unit", "description", "references", "short"}
        return {item.name: getattr(self, item.name) for item in fields(self) if item.name not in skip}

    @property
    def pass_text(self) -> str:
        raise NotImplementedError

    def __repr__(self) -> str:
        unit = f" ({self.unit})" if self.unit else ""
        return f"{self.name}{unit}: {self.description}"

    # how it is summarised: each returns an expression for agg() or Live()
    def median(self) -> Summary:
        return Summary(self, "median")

    def max(self, if_none: float | None = None) -> Summary:
        return Summary(self, "max", if_none)

    def min(self, if_none: float | None = None) -> Summary:
        return Summary(self, "min", if_none)

    def rms(self) -> Summary:
        return Summary(self, "rms")

    def mean(self) -> Summary:
        return Summary(self, "mean")

    def percentile(self, q: float) -> Summary:
        return Summary(self, f"p{q:g}")

    def count(self) -> Summary:
        return Summary(self, "count")

    def share(self) -> Summary:
        return Summary(self, "share")

    def all(self) -> Summary:
        return Summary(self, "all")

    def any(self) -> Summary:
        return Summary(self, "any")

    def repeatability(self, k: float = 3.0) -> Repeatability:
        return Repeatability(self, k)


@dataclass(frozen=True, repr=False, kw_only=True)
class RowError(Measure):
    """|reference - measured| on each row, from robot_msgs.compare: one joint's field, or a pose's position error."""

    field: str = "positions"            # positions | velocities | effort; a pose: its position error
    rows: str = "moving"                # moving (the target's speed is not zero) | all
    joint: int = 0
    scale: float = DEGREES              # radians to the unit shown (1e3 for metres to mm, 1.0 for N·m)
    streams: tuple[str, str] = ("target", "measured")

    @property
    def pass_key(self) -> tuple:
        return ("compare", self.streams)

    @property
    def pass_text(self) -> str:
        return f"compare({', '.join(self.streams)})"

    def reader(self) -> _CompareReader:
        return _CompareReader(self.streams)

    def pick(self, items: list, run: Any) -> np.ndarray:
        values = []
        for reference, error in items:
            if isinstance(error, PoseTrajectory):
                column = np.linalg.norm(error.positions, axis=1)
            else:
                column = np.asarray(getattr(error, self.field)[:, self.joint], float)
            if self.rows == "moving":
                column = column[moving(reference, self.joint)]
            values.append(self.scale * np.abs(column[np.isfinite(column)]))
        return np.concatenate(values) if values else np.empty(0)


@dataclass(frozen=True, repr=False, kw_only=True)
class StopValue(Measure):
    """One value per stop: a dwell >= 0.25 s at zero target speed on one target that a move led into.

    ``of``: "error" |measured - target| over the dwell's last 50 ms, "offset" the same signed, "settle" the time to stay
    within ``band``, "settled" whether it did by the dwell's end (1 or 0), "overshoot" past the target along the move
    (robot_msgs.compare.goal). ``at``: "tip" in mm (the joint angle times the run's lever length) or "joint" in degrees.
    """

    of: str = "error"
    at: str = "joint"
    band: float | None = None           # in the measure's unit: 1.0 is 1 mm at the tip
    joint: int = 0

    @property
    def pass_key(self) -> tuple:
        return ("stops", self.joint)

    @property
    def pass_text(self) -> str:
        return "stops(target, measured)"

    def reader(self) -> _StopReader:
        return _StopReader(self.joint)

    def scale_of(self, run: Any) -> float:
        return 1e3 * run.tip_length_m if self.at == "tip" else DEGREES

    def pick(self, items: list, run: Any) -> np.ndarray:
        scale = self.scale_of(run)
        if self.of == "error":
            values = [scale * abs(stop.error) for stop in items]
        elif self.of == "offset":
            values = [scale * stop.error for stop in items]
        elif self.of in ("settle", "settled"):
            settles = [settle_time(stop, self.band, scale) for stop in items]
            values = settles if self.of == "settle" else [float(math.isfinite(s)) for s in settles]
        elif self.of == "overshoot":
            values = [scale * goal(*stop.move(), band=math.inf)[f"positions/{stop.joint}"].overshoot for stop in items]
        else:
            raise ValueError(f"of is error, offset, settle, settled or overshoot, not {self.of!r}")
        return np.asarray(values, float)


@dataclass(frozen=True, repr=False, kw_only=True)
class ChannelCode(Measure):
    """1 or 0 on each row: does the measured channel hold one of the codes?"""

    channel: str = "fault"
    codes: tuple[int, ...] = LIMITER_CODES
    input: ClassVar[str] = "Run: the measured stream's channels"

    @property
    def pass_key(self) -> tuple:
        return ("channel", self.channel)

    @property
    def pass_text(self) -> str:
        return f"the {self.channel} channel"

    def reader(self) -> _ChannelReader:
        return _ChannelReader(self.channel, self.codes)

    def pick(self, items: list, run: Any) -> np.ndarray:
        return np.concatenate(items).astype(float) if items else np.empty(0)


@dataclass(frozen=True, repr=False, kw_only=True)
class TimeOffset(Measure):
    """One value per run: how far the response runs behind the command (robot_msgs.compare.estimate_time_offset)."""

    scale: float = 1e3                  # seconds to ms
    input: ClassVar[str] = "Run: plan and measured, the whole recording"
    live: ClassVar[bool] = False        # a cross-correlation needs the whole run
    hint: ClassVar[str] = "kpi_trend"
    streams: ClassVar[tuple[str, ...]] = ("plan", "measured")

    @property
    def pass_key(self) -> tuple:
        return ("time offset", self.streams)

    @property
    def pass_text(self) -> str:
        return "estimate_time_offset(plan, measured)"

    def reader(self) -> _WholeRunReader:
        def offset(chunks: list) -> float:
            plan = next(chunk.plan for chunk in chunks if chunk.plan is not None)
            measured = concatenate([chunk.measured for chunk in chunks]) if len(chunks) > 1 else chunks[0].measured
            return float(estimate_time_offset(plan, measured).offset_s)
        return _WholeRunReader(offset)

    def pick(self, items: list, run: Any) -> np.ndarray:
        return self.scale * np.asarray(items, float)


@dataclass(frozen=True, repr=False, kw_only=True)
class RunFlag(Measure):
    """One value per run, 1 or 0: a yes/no fact the recording carries (faulted: the runner stopped it early)."""

    attribute: str = "faulted"
    input: ClassVar[str] = "Run: its labels"
    hint: ClassVar[str] = "evidence"

    @property
    def pass_key(self) -> tuple:
        return ("run", self.attribute)

    @property
    def pass_text(self) -> str:
        return f"run.{self.attribute}"

    def reader(self) -> _WholeRunReader:
        return _WholeRunReader(lambda chunks: float(bool(getattr(chunks[0], self.attribute))))

    def pick(self, items: list, run: Any) -> np.ndarray:
        return np.asarray(items, float)


# --------------------------------------------------------------------------- summaries: how the values are pooled

POOLED = "every value in the group's runs, pooled"
COLUMN = {"median": "median {s}", "max": "worst {s}", "min": "best {s}", "rms": "{s} RMS", "mean": "mean {s}",
          "count": "{s} count", "share": "{s} share", "all": "every {s}", "any": "any {s}"}
PHRASE = {"median": "the median of", "max": "the largest of", "min": "the smallest of", "rms": "the RMS of",
          "mean": "the mean of", "count": "the number of", "share": "the share (%) of rows set in",
          "all": "true when every value is set in", "any": "true when any value is set in"}


def _percentile(how: str) -> float | None:
    return float(how[1:]) if how.startswith("p") and how[1:].replace(".", "", 1).isdigit() else None


class _Running:
    """A summary's running total: add(values) as they arrive, value() at any time. What it keeps is listed by
    holds(): two or three numbers for rms / max / mean / share, but every value for a median or a percentile."""

    def __init__(self, how: str, if_none: float | None) -> None:
        self.how, self.if_none = how, if_none
        self.kept: list[np.ndarray] = []           # median / percentile: every value (exact, so it grows)
        self.total, self.squares, self.count = 0.0, 0.0, 0
        self.extreme: float | None = None
        self.flag = self.how == "all"

    def add(self, values: np.ndarray) -> None:
        values = np.asarray(values, float)
        if not values.size:
            return
        self.count += int(values.size)
        if self.how == "median" or _percentile(self.how) is not None:
            self.kept.append(values)
        elif self.how in ("max", "min"):
            found = float(np.max(values) if self.how == "max" else np.min(values))
            better = max if self.how == "max" else min
            self.extreme = found if self.extreme is None else better(self.extreme, found)
        elif self.how == "rms":
            self.squares += float(np.sum(values * values))
        elif self.how in ("mean", "share"):
            self.total += float(np.sum(values))
        elif self.how == "all":
            self.flag = self.flag and bool(np.all(values == 1.0))
        elif self.how == "any":
            self.flag = self.flag or bool(np.any(values == 1.0))

    def value(self) -> Any:
        if self.how == "count":
            return self.count
        if self.how in ("all", "any"):
            return self.flag
        if not self.count:
            return self.if_none
        if self.how == "median":
            return float(np.median(np.concatenate(self.kept)))
        if _percentile(self.how) is not None:
            return float(np.percentile(np.concatenate(self.kept), _percentile(self.how)))
        if self.how in ("max", "min"):
            return self.extreme
        if self.how == "rms":
            return math.sqrt(self.squares / self.count)
        share = self.total / self.count
        return 100.0 * share if self.how == "share" else share

    def holds(self) -> str:
        if self.kept:
            count = sum(part.size for part in self.kept)
            return f"{count} value{'s' if count != 1 else ''}"
        return {"rms": "2 numbers", "mean": "2 numbers", "share": "2 numbers", "max": "1 number", "min": "1 number",
                "count": "1 number", "all": "1 flag", "any": "1 flag"}[self.how]


@dataclass(frozen=True, repr=False)
class Summary:
    """An expression: one measure, pooled one way over the runs it is given. Its value is a number or a bool."""

    measure: Measure
    how: str
    if_none: float | None = None

    @property
    def name(self) -> str:
        return f"{self.core} ({self.unit})" if self.unit else self.core

    @property
    def core(self) -> str:
        word = COLUMN.get(self.how, f"{self.how} {{s}}")
        return word.format(s=self.measure.column)

    @property
    def unit(self) -> str:
        if self.how == "share":
            return "%"
        return "" if self.how in ("count", "all", "any") else self.measure.unit

    @property
    def value_type(self) -> str:
        return "bool" if self.how in ("all", "any") else "number"

    @property
    def live(self) -> bool:
        """Can it run on chunks as they arrive? (lag cannot: it needs the whole recording)"""

        return self.measure.live

    @property
    def phrase(self) -> str:
        quantile = _percentile(self.how)
        return f"the {quantile:g}th percentile of" if quantile is not None else PHRASE[self.how]

    @property
    def pooling(self) -> str:
        return f"{self.phrase} {POOLED}"

    def config(self) -> dict[str, Any]:
        return {**self.measure.config, "summary": self.how}

    def __repr__(self) -> str:
        quantile = _percentile(self.how)
        call = f"percentile({quantile:g})" if quantile is not None else f"{self.how}()"
        if self.if_none is not None:
            call = f"{self.how}(if_none={self.if_none!r})"
        return f"{self.measure.code}.{call}"

    def start(self) -> _Running:
        return _Running(self.how, self.if_none)

    def evaluate(self, runs: tuple) -> Any:
        running = self.start()
        for run in runs:
            running.add(values_of(self.measure, run))
        return running.value()

    def per_run(self) -> PerRun:
        return PerRun(self)


@dataclass(frozen=True, repr=False)
class PerRun:
    """A summary taken on each run alone: one value per run, summarised again across runs (``.median()``)."""

    inner: Summary

    def median(self) -> AcrossRuns:
        return AcrossRuns(self.inner, "median")

    def max(self) -> AcrossRuns:
        return AcrossRuns(self.inner, "max")

    def mean(self) -> AcrossRuns:
        return AcrossRuns(self.inner, "mean")

    def __repr__(self) -> str:
        return f"{self.inner!r}.per_run()"


class _AcrossRunsRunning:
    """Live over one recording: the inner summary's running total, and the outer summary of that one value."""

    def __init__(self, inner: _Running, how: str) -> None:
        self.inner, self.how = inner, how

    def add(self, values: np.ndarray) -> None:
        self.inner.add(values)

    def value(self) -> Any:
        outer = _Running(self.how, None)
        one = self.inner.value()
        if one is not None:
            outer.add(np.asarray([one], float))
        return outer.value()

    def holds(self) -> str:
        return self.inner.holds()


@dataclass(frozen=True, repr=False)
class AcrossRuns:
    """``tracking_error.rms().per_run().median()``: each run's RMS, then the median of those per-run values."""

    inner: Summary
    how: str

    @property
    def measure(self) -> Measure:
        return self.inner.measure

    @property
    def unit(self) -> str:
        return self.inner.unit

    @property
    def name(self) -> str:
        unit = f" ({self.unit})" if self.unit else ""
        return f"{self.inner.core}, {self.how} over runs{unit}"

    @property
    def value_type(self) -> str:
        return "number"

    @property
    def live(self) -> bool:
        return self.inner.live

    @property
    def pooling(self) -> str:
        return f"{PHRASE[self.how]} one value per run, each run's being {self.inner.phrase} its own values"

    def config(self) -> dict[str, Any]:
        return {**self.inner.config(), "across_runs": self.how}

    def __repr__(self) -> str:
        return f"{self.inner!r}.per_run().{self.how}()"

    def start(self) -> _AcrossRunsRunning:
        return _AcrossRunsRunning(self.inner.start(), self.how)

    def evaluate(self, runs: tuple) -> Any:
        running = _Running(self.how, None)
        for run in runs:
            one = self.inner.evaluate((run,))
            if one is not None:
                running.add(np.asarray([one], float))
        return running.value()


# --------------------------------------------------------------------------- summaries over other inputs (amendment 4)


@dataclass(frozen=True, repr=False)
class Repeatability:
    """ISO 9283 pose repeatability over same-input repeats: at each stop, mean(l) + k * std(l), where l is each
    repeat's distance from the repeats' mean stop; the worst stop. Its input is a group of repeats of one command, so it
    only makes sense in agg() over runs filtered to one command (kind="repeat", one station, one rung)."""

    measure: Measure
    k: float = 3.0
    input: ClassVar[str] = "same-input repeats: the runs of one command"
    live: ClassVar[bool] = False
    value_type: ClassVar[str] = "number"

    @property
    def unit(self) -> str:
        return self.measure.unit

    @property
    def name(self) -> str:
        return f"{self.measure.column} repeatability, k={self.k:g} ({self.unit})"

    @property
    def pooling(self) -> str:
        return "ISO 9283 RP over the group's runs, matched stop by stop"

    def config(self) -> dict[str, Any]:
        return {**self.measure.config, "k": self.k}

    def __repr__(self) -> str:
        return f"{self.measure.code}.repeatability(k={self.k:g})"

    def evaluate(self, runs: tuple) -> float | None:
        per_repeat = [values_of(self.measure, run) for run in runs]
        if len(per_repeat) < 2 or len({len(values) for values in per_repeat}) != 1:
            return None
        matrix = np.asarray(per_repeat)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return float(np.max(distance.mean(axis=0) + self.k * distance.std(axis=0, ddof=1)))


@dataclass(frozen=True, repr=False)
class PerformanceGain:
    """A summary on held-out runs played with the nominal values, minus the same summary with the fitted ones. Its
    input is runs labelled by which values played them (``label``), so ``held_out.agg(PerformanceGain(...))``.

    WHY (Zach, 3000-ft view, 2026-11-17): "Fitting a model is only useful if it gives us better performance on the
    things that actually matter": the gain is the performance metric itself, on runs the fit never saw."""

    inner: Summary
    label: str = "values"
    nominal: str = "nominal"
    fitted: str = "fitted"
    input: ClassVar[str] = "held-out runs, played with nominal and with fitted values"
    live: ClassVar[bool] = False
    value_type: ClassVar[str] = "number"

    @property
    def measure(self) -> Measure:
        return self.inner.measure

    @property
    def unit(self) -> str:
        return self.inner.unit

    @property
    def name(self) -> str:
        unit = f" ({self.unit})" if self.unit else ""
        return f"{self.inner.core} gain, {self.nominal} - {self.fitted}{unit}"

    @property
    def pooling(self) -> str:
        return f"{self.inner.pooling}, on the {self.nominal} runs minus on the {self.fitted} runs"

    def config(self) -> dict[str, Any]:
        return {**self.inner.config(), "gain": f"{self.nominal} - {self.fitted}"}

    def __repr__(self) -> str:
        return f"PerformanceGain({self.inner!r})"

    def sides(self, runs: tuple) -> tuple[Any, Any]:
        def side(which: str) -> Any:
            return self.inner.evaluate(tuple(run for run in runs if run.labels.get(self.label) == which))
        return side(self.nominal), side(self.fitted)

    def evaluate(self, runs: tuple) -> float | None:
        nominal, fitted = self.sides(runs)
        return None if nominal is None or fitted is None else nominal - fitted


MEASURE_KINDS = (RowError, StopValue, ChannelCode, TimeOffset, RunFlag, Repeatability, PerformanceGain)


def card(expression: Any) -> dict[str, Any]:
    """Zach's Metric fields for one expression: name, description, references, the function, unit, target, value type.
    ``target`` is always None here: a target is a Threshold beside the table, never part of the metric."""

    measure = expression.measure
    return {"name": expression.name, "description": f"{measure.description}; {expression.pooling}",
            "references": list(measure.references), "function": repr(expression), "unit": expression.unit,
            "target": None, "value_type": expression.value_type}
