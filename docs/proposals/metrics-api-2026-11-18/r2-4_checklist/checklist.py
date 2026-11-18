"""Checklist with targets (round 2, r2-4): a part's metrics as a short checklist whose rows read as sentences.

    card = Card("pendulum arm", [
        Check("worst tip stop", measure=worst_stop_error(at="tip"), unit="mm", target="<= 1.0", owner="Zach"),
        Check("stops", measure=stop_count(), unit="stops"),        # no target: measured, shown as "observed"
    ])
    card.check(run)                            # one recording: a pass / fail table, one row per check
    card.check(runs, by=["station", "rung"])   # many: every group against each target, the failing groups named
    card.measure(run)                          # the numbers alone: {check name: value in the row's unit}
    live = card.live()                         # the same rows on the chunks so far
    live.update(chunk)

A row has five parts: its name, how it is measured, the unit it is shown in, its target and who set the target. A row
without a target is measured and shown as observed.

WHY a measure never holds a threshold (Zach, robot-diagnostics: "separate the calculation of the metric, and the
checking of thresholds"): a measure turns recordings into one number in SI units; the row says which unit to show it
in, what to hold it to and who said so; the card does the comparing. Great Expectations splits the same way: its
metrics compute an ``observed_value`` and the expectation (``expect_column_values_to_be_between``) only judges it.

WHY the arithmetic is robot_metrics 0.1.0's (imported read-only from its checkout, never edited): the round-2 brief
allows it, robot_metrics already reproduces round 8 at float round-off, and a variant that re-derived the stop rule
would be a fourth copy of it. This file is only the checklist face over it.
"""

from __future__ import annotations

import math
import re
import sys
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import Any

ROBOT_METRICS_SRC = "/home/bam/robot_metrics/src"
# WHY appended, never inserted: a reader's own robot_metrics (the baseline column's symlink) wins if it is on the path
if ROBOT_METRICS_SRC not in sys.path:
    sys.path.append(ROBOT_METRICS_SRC)

import robot_metrics  # noqa: E402
from robot_metrics import (CodeShare, ErrorStat, HeldOutPair, PerformanceGain, Repeat, Repeatability,  # noqa: E402
                           StopStat, TimeOffset)

if robot_metrics.__version__ != "0.1.0":
    raise ImportError(f"checklist is built on robot_metrics 0.1.0, found {robot_metrics.__version__}")

DEGREES = 180.0 / math.pi
LIMITER_CODES = (97, 98, 99, 102)    # moteus: rows the limiter or a fault shaped (round 8's limiter share)

# WHY units live on the row, never in a measure (Zach 2026-11-11: SI in code and data, degrees and mm on screen): each
# measure says what kind of number it returns in SI; a row may show it in any unit of that kind and no other.
UNITS: dict[str, dict[str, float]] = {
    "angle": {"rad": 1.0, "deg": DEGREES, "mrad": 1e3},
    "length": {"m": 1.0, "mm": 1e3},
    "angular speed": {"rad/s": 1.0, "deg/s": DEGREES},
    "torque": {"N·m": 1.0},
    "time": {"s": 1.0, "ms": 1e3},
    "share": {"%": 100.0, "share": 1.0},
    "count": {"stops": 1.0, "runs": 1.0, "count": 1.0},
}
SI_UNIT = {quantity: next(unit for unit, factor in units.items() if factor == 1.0) for quantity, units in UNITS.items()}
ACROSS_WORDS = {"pool": "pooled over every stop or row", "median": "median of the runs",
                "max": "worst of the runs", "count": "how many runs"}


# --------------------------------------------------------------------------------------------- how a number is measured


@dataclass(frozen=True)
class Measure:
    """How one number comes out of recordings, in SI units, with no unit to show and no target.

    ``says`` is the plain meaning; ``across`` how runs combine (pooled, median of runs, worst of runs, a count);
    ``metric`` the robot_metrics stock metric that does the arithmetic (None: a fact read off each run)."""

    call: str
    says: str
    quantity: str
    across: str = "pool"
    metric: Any = None
    signal: tuple[str, str] = ("target", "state")
    at_tip: bool = False
    tip_band_m: float | None = None
    takes: str = "a recording"
    value_type: str = "number"     # per run: a number, or a bool (yes / no)

    def __repr__(self) -> str:
        return self.call

    def metric_for(self, tip_length_m: float | None) -> Any:
        """The stock metric for runs of one tip length: a settle band at the tip is that many joint radians."""

        if self.tip_band_m is None:
            return self.metric
        if not tip_length_m:
            raise ValueError(f"{self.call} needs the run's tip length")
        return StopStat(of=self.metric.of, statistic=self.metric.statistic, band=self.tip_band_m / tip_length_m)

    @property
    def live(self) -> bool:
        return self.metric is None or bool(self.metric.live)

    def __call__(self, *data: Any) -> Any:
        """The number in SI: of one recording, a list of runs (combined by ``across``), the repeats of one command,
        or (nominal runs, fitted runs) for a performance gain."""

        if self.takes == "repeats of one command":
            return _repeatability(self, as_runs(data[0]))
        if self.takes == "nominal and fitted runs":
            nominal, fitted = self.sides(*data)
            return None if nominal is None or fitted is None else nominal - fitted
        alone = Card("one measure", [Check("value", measure=self, unit=SI_UNIT[self.quantity])])
        return alone.measure(data[0])["value"]

    def sides(self, nominal: Any, fitted: Any) -> tuple[Any, Any]:
        """A performance gain's two sides: the measure on the nominal runs and on the fitted runs (SI)."""

        if self.takes != "nominal and fitted runs":
            raise TypeError(f"{self.call} is not a performance gain")
        gain = self.metric
        state = gain.update(gain.empty(), HeldOutPair(tuple(as_runs(nominal)), tuple(as_runs(fitted))))
        return gain.sides(state)


ERROR_OF = {    # what a tracking error is of: (the stream field it reads, the kind of number it is)
    "position": ("positions", "angle"),
    "velocity": ("velocities", "angular speed"),
    "torque": ("effort", "torque"),
    "tool": ("positions", "length"),       # the tool's position: robot_msgs.compare's distance between two poses
}


def _signal(of: str) -> tuple[str, str]:
    return ("tool/target", "tool/state") if of == "tool" else ("target", "state")


def rms_tracking_error(of: str = "position", *, joint: int = 0, while_moving: bool = True,
                       at: str = "joint") -> Measure:
    """RMS of measured - target, of the position (or the velocity, the torque, the tool's position)."""

    field_name, quantity = ERROR_OF[of]
    at_tip = at == "tip"
    when = (" while moving" if while_moving else "") + (f", joint {joint}" if joint else "")
    says = f"RMS of measured - target{' at the tip' if at_tip else ''}{when}" if of == "position" else \
        f"RMS of {of} error{when}"
    call = _call("rms_tracking_error", of=(of, "position"), joint=(joint, 0), while_moving=(while_moving, True),
                 at=(at, "joint"))
    metric = ErrorStat(statistic="rms", rows="moving" if while_moving else "all", field=field_name, joint=joint)
    return Measure(call, says, "length" if at_tip else quantity, across="median", metric=metric, signal=_signal(of),
                   at_tip=at_tip)


def peak_tracking_error(of: str = "position", *, joint: int = 0) -> Measure:
    """The largest |measured - target| of the run, every row."""

    field_name, quantity = ERROR_OF[of]
    call = _call("peak_tracking_error", of=(of, "position"), joint=(joint, 0))
    says = "largest |measured - target|" if of == "position" else f"largest {of} error"
    return Measure(call, says, quantity, across="max", metric=ErrorStat(statistic="max_abs", field=field_name,
                                                                        joint=joint), signal=_signal(of))


def median_stop_error(at: str = "joint") -> Measure:
    """The median |stop error| over every stop (a stop: the arm held still on one target for 0.25 s or more)."""

    return _stop_error(_call("median_stop_error", at=(at, "joint")), "median stop error", "median", at)


def worst_stop_error(at: str = "joint") -> Measure:
    """The largest |stop error| over every stop."""

    return _stop_error(_call("worst_stop_error", at=(at, "joint")), "largest stop error", "max", at)


def stop_error_percentile(percent: float, at: str = "joint") -> Measure:
    """A percentile of |stop error| over every stop (95: the p95)."""

    call = _call("stop_error_percentile", percent=(percent, None), at=(at, "joint"))
    return _stop_error(call, f"p{percent:g} stop error", f"p{percent:g}", at)


def _stop_error(call: str, says: str, statistic: str, at: str) -> Measure:
    """A stop error pooled over every stop: in joint radians, or at the tip (times the run's tip length)."""

    at_tip = at == "tip"
    return Measure(call, says + (" at the tip" if at_tip else ""), "length" if at_tip else "angle", across="pool",
                   metric=StopStat(of="error", statistic=statistic), at_tip=at_tip)


def stop_count() -> Measure:
    return Measure("stop_count()", "number of stops", "count", "pool", StopStat(of="count", statistic="count"))


def settle_time(at: str = "tip", within_mm: float = 1.0) -> Measure:
    """The median stop's time from the end of its move until the tip stays within ``within_mm``."""

    if at != "tip":
        raise ValueError("settle_time is read at the tip (its band is in mm)")
    return Measure(_call("settle_time", at=(at, "tip"), within_mm=(within_mm, 1.0)),
                   f"median time to settle within {within_mm:g} mm at the tip", "time", "pool",
                   StopStat(of="settle", statistic="median"), tip_band_m=within_mm / 1e3)


def overshoot() -> Measure:
    return Measure("overshoot()", "largest overshoot past a stop's target", "angle", "pool",
                   StopStat(of="overshoot", statistic="max"))


def limiter_share(codes: tuple[int, ...] = LIMITER_CODES) -> Measure:
    return Measure("limiter_share()", "share of rows the limiter or a fault shaped", "share", "pool",
                   CodeShare(codes=codes))


def lag() -> Measure:
    """How late the reply runs behind the plan (robot_msgs.compare.estimate_time_offset), per run; needs the whole run."""

    return Measure("lag()", "how late the reply runs behind the plan", "time", "median", TimeOffset(),
                   ("plan", "state"))


def faulted() -> Measure:
    """Whether a run faulted: a yes / no per run; over runs, how many faulted."""

    return Measure("faulted()", "runs that faulted (yes / no per run)", "count", across="count", value_type="bool")


def repeatability(k: float = 3.0, at: str = "joint") -> Measure:
    """ISO 9283 pose repeatability over repeats of one command: mean(l) + k * std(l) at each stop, the worst stop."""

    tip = at == "tip"
    return Measure(_call("repeatability", k=(k, 3.0), at=(at, "joint")),
                   f"ISO 9283 repeatability{' at the tip' if tip else ''}, worst stop", "length" if tip else "angle",
                   "pool", Repeatability(k=k), at_tip=tip, takes="repeats of one command")


def performance_gain(measure: Measure) -> Measure:
    """The measure on held-out runs played with nominal values minus the same with fitted values."""

    return Measure(f"performance_gain({measure.call})", f"gain in: {measure.says}", measure.quantity, "pool",
                   PerformanceGain(metric=measure.metric, signal=measure.signal), measure.signal,
                   takes="nominal and fitted runs")


def _call(name: str, **arguments: tuple[Any, Any]) -> str:
    """How the measure is written, with only the arguments that differ from their defaults."""

    shown = [f"{key}={value!r}".replace("'", '"') for key, (value, default) in arguments.items() if value != default]
    return f"{name}({', '.join(shown)})"


def _repeatability(measure: Measure, runs: Sequence[Any]) -> float | None:
    metric = measure.metric
    state = metric.empty()
    for run in runs:
        command = (run.labels.get("station"), run.labels.get("rung"))
        state = metric.update(state, Repeat(command, run.streams["target"], run.streams["state"]))
    value = metric.compute(state)
    if value is None or not measure.at_tip:
        return value
    return value * _one_tip(runs, measure.call)


# --------------------------------------------------------------------------------------------- a row and a card


TARGET = re.compile(r"^\s*(<=|>=|==|<|>)\s*(-?\d+(?:\.\d*)?(?:e-?\d+)?)\s*$")
COMPARE = {"<=": lambda a, b: a <= b, ">=": lambda a, b: a >= b, "==": lambda a, b: a == b,
           "<": lambda a, b: a < b, ">": lambda a, b: a > b}


@dataclass(frozen=True)
class Check:
    """One row of a checklist: its name, how it is measured, the unit it is shown in, its target and who set it."""

    name: str
    measure: Measure
    unit: str
    target: str | None = None
    owner: str | None = None

    def __post_init__(self) -> None:
        units = UNITS.get(self.measure.quantity, {})
        if self.unit not in units:
            raise ValueError(f"{self.name}: {self.measure.call} is a {self.measure.quantity}; "
                             f"show it in {' or '.join(units)}, not {self.unit!r}")
        if self.target is not None and not TARGET.match(self.target):
            raise ValueError(f"{self.name}: a target reads like '<= 1.0', not {self.target!r}")

    @property
    def factor(self) -> float:
        return UNITS[self.measure.quantity][self.unit]

    def passes(self, value: Any) -> bool | None:
        """True / False against the target (a missing value never passes); None when the row has no target."""

        if self.target is None:
            return None
        if value is None:
            return False
        operator, number = TARGET.match(self.target).groups()
        return bool(COMPARE[operator](float(value), float(number)))

    def card(self) -> dict[str, Any]:
        """Zach's Metric fields: name, description, references, function, unit, target, value type."""

        metric = self.measure.metric
        references = list(metric.references) if metric is not None else ["round 1 fixture: Run.faulted (moteus fault)"]
        references.append("Great Expectations: expect_column_values_to_be_between (the target is applied apart)")
        return {"name": self.name, "description": self.measure.says, "references": references,
                "function": f"checklist.{self.measure.call}", "unit": self.unit,
                "target": None if self.target is None else f"{self.target} {self.unit}", "owner": self.owner,
                "value_type": self.measure.value_type,
                "across runs": ACROSS_WORDS[self.measure.across]}

    def __str__(self) -> str:
        held = f"target {self.target}, set by {self.owner}" if self.target else "no target: observed"
        return f"{self.name}: {self.measure.call} in {self.unit}, {held}"


class Card:
    """A part's checklist: its rows, in order."""

    def __init__(self, part: str, checks: Iterable[Check]) -> None:
        self.part = part
        self.checks = list(checks)
        names = [check.name for check in self.checks]
        if len(set(names)) != len(names):
            raise ValueError(f"{part}: two checks share a name")
        self._collections: dict[Any, Any] = {}

    def __getitem__(self, name: str) -> Check:
        for check in self.checks:
            if check.name == name:
                return check
        raise KeyError(f"{self.part} has no check {name!r}: {[check.name for check in self.checks]}")

    def __len__(self) -> int:
        return len(self.checks)

    def __str__(self) -> str:
        header = ("check", "how it is measured", "unit", "target")
        rows = [(check.name, check.measure.says, check.unit,
                 f"{check.target} ({check.owner})" if check.target else "none: observed") for check in self.checks]
        return f"{self.part}: {len(self.checks)} checks\n" + _table([header, *rows])

    def __repr__(self) -> str:
        return f"Card({self.part!r}, {len(self.checks)} checks)"

    # ---- measuring: numbers only, no targets ---------------------------------------------------------------------
    def measure(self, data: Any, by: Sequence[str] | None = None) -> Measured:
        """Each row's number in its unit. ``data``: one recording, a list of runs, or a ``live`` running measure.
        With ``by``, one row of numbers per group of runs (``by=["station", "rung"]``)."""

        if isinstance(data, Live):
            return Measured(data.values(), card=self, counts={(): data.runs}, title=data.title())
        runs = as_runs(data)
        if by is None:
            return Measured(self._values(runs), card=self, counts={(): len(runs)}, title=_title(runs))
        groups = group_runs(runs, by)
        values = {key: self._values(members) for key, members in groups.items()}
        return Measured(values, card=self, by=tuple(by), counts={key: len(members) for key, members in groups.items()},
                        title=f"{len(runs)} recordings in {len(groups)} groups by {', '.join(by)}")

    # ---- checking: each number against its row's target ------------------------------------------------------------
    def check(self, data: Any, by: Sequence[str] | None = None) -> Report:
        """The measured numbers against each row's target: pass, FAIL, or observed when the row has no target."""

        return Report(self, self.measure(data, by))

    def live(self) -> Live:
        """A running measure: ``update(chunk)`` as rows arrive; check or measure it at any time."""

        return Live(self)

    # ---- the stock metrics underneath, one set per tip length --------------------------------------------------------
    def collection(self, tip_length_m: float | None) -> Any:
        """robot_metrics' Collection over this card's recording rows: equal (metric, signal) pairs compute once."""

        if tip_length_m not in self._collections:
            aliases = []
            for check in self.checks:
                measure = check.measure
                if measure.metric is None or measure.takes != "a recording":
                    continue
                scale = check.factor * (tip_length_m if measure.at_tip else 1.0)
                if measure.at_tip and not tip_length_m:
                    raise ValueError(f"{check.name}: {measure.call} needs each run's tip length")
                aliases.append(robot_metrics.Alias(check.name, measure.metric_for(tip_length_m), measure.signal,
                                                   check.unit, scale, measure.across))
            self._collections[tip_length_m] = robot_metrics.Collection(aliases)
        return self._collections[tip_length_m]

    def _tip(self, runs: Sequence[Any]) -> float | None:
        needs = [check for check in self.checks if check.measure.at_tip or check.measure.tip_band_m]
        if not needs:
            return None
        return _one_tip(runs, needs[0].name)

    def _values(self, runs: Sequence[Any]) -> dict[str, Any]:
        collection = self.collection(self._tip(runs))
        stratum = robot_metrics.summarize(collection, runs, by=())[()] if collection.aliases and runs else None
        out: dict[str, Any] = {}
        for check in self.checks:
            measure = check.measure
            if measure.takes == "repeats of one command":
                value = _repeatability(measure, runs)
                out[check.name] = None if value is None else value * check.factor
            elif measure.takes == "nominal and fitted runs":
                raise TypeError(f"{check.name}: call {measure.call} with (nominal runs, fitted runs)")
            elif measure.metric is None:
                out[check.name] = sum(run.outcome != "complete" for run in runs)
            else:
                out[check.name] = stratum.values[check.name] if stratum else None
        return out

    def occurrences(self, name: str, runs: Any, by: str) -> tuple[list[float], list[str]]:
        """Every occurrence behind a pooled row (each stop), in the row's unit, labelled by the ``by`` group."""

        values, labels = [], []
        for key, members in group_runs(as_runs(runs), [by]).items():
            collection = self.collection(self._tip(members))
            stratum = robot_metrics.summarize(collection, members, by=())[()]
            found = list(stratum.occurrences.get(name, ()))
            values += found
            labels += [_label(key[0])] * len(found)
        return values, labels

    def view(self, name: str, runs: Any, by: str, title: str) -> Any:
        """The row's occurrences as round 1's shared View (matplotlib, a W&B payload, Rerun), with its target line."""

        import _viz

        check = self[name]
        values, labels = self.occurrences(name, runs, by)
        target = float(TARGET.match(check.target).group(2)) if check.target else None
        return _viz.View(title=title, unit=check.unit, hint=check.measure.metric.hint, x=list(range(len(values))),
                         y=values, group=labels, target=target, target_label=f"{check.target} ({check.owner})",
                         x_label="stop" if check.measure.metric.hint == "distribution" else "run",
                         y_label=check.unit)


# --------------------------------------------------------------------------------------------- live


class Live:
    """The card's rows on the chunks so far. It keeps each stock metric's running state (sums, counts, the stops
    found, the rows of the move still open), never the rows already scored, except ``lag``, which needs the whole run."""

    def __init__(self, card: Card) -> None:
        self.card = card
        self.collection: Any = None
        self.state: Any = None
        self.chunks = 0
        self.rows = 0
        self.runs = 0
        self.faulted = False

    def update(self, chunk: Any) -> None:
        recording = as_recording(chunk)
        if self.collection is None:
            self.collection = self.card.collection(self.card._tip([recording]))
            self.state = self.collection.empty()
            self.runs = 1
        self.state = self.collection.update(self.state, recording)
        self.chunks += 1
        self.rows += len(recording.streams["state"])
        self.faulted = self.faulted or recording.outcome != "complete"

    def values(self) -> dict[str, Any]:
        if self.collection is None:
            return {check.name: None for check in self.card.checks}
        closed = self.collection.values(self.collection.close(self.state))
        out = {}
        for check in self.card.checks:
            if check.measure.metric is None:
                out[check.name] = int(self.faulted)
            else:
                out[check.name] = closed.get(check.name)
        return out

    def title(self) -> str:
        return f"live: {self.chunks} chunks, {self.rows} rows so far"

    def kept(self) -> str:
        """What the running measure holds right now, per stock metric (one line each)."""

        lines = []
        for key in (self.collection.groups if self.collection else ()):
            names = [name for name, alias_key in self.collection.key_of.items() if alias_key == key]
            lines.append((", ".join(names), _kept(self.state[key])))
        return _table([("rows of the card", "kept between chunks"), *lines])


def _kept(state: Any) -> str:
    if isinstance(state, robot_metrics.stops.StopsState):
        open_rows = 0 if state.open is None else len(state.open.since)
        found = f"{len(state.closed)} stop{'' if len(state.closed) == 1 else 's'}"
        return f"{found} + {open_rows} rows of the open move"
    if isinstance(state, tuple) and len(state) == 3 and isinstance(state[1], tuple):
        rows = sum(len(part) for part in state[2])
        return f"every row so far ({rows}): needs the whole run"
    if isinstance(state, tuple):
        return f"{len(state)} numbers"
    return type(state).__name__


# --------------------------------------------------------------------------------------------- results as tables


class Measured(dict):
    """The numbers: ``{check name: value}`` for one group, ``{group: {check name: value}}`` with ``by``. A dict, so
    ``==`` compares the values; it prints as a table, each number in its row's unit."""

    def __init__(self, values: Mapping, *, card: Card, by: tuple[str, ...] = (), counts: Mapping | None = None,
                 title: str = "") -> None:
        super().__init__(values)
        self.card, self.by, self.counts, self.title = card, by, dict(counts or {}), title

    def groups(self) -> dict[tuple, dict[str, Any]]:
        return dict(self) if self.by else {(): dict(self)}

    def __repr__(self) -> str:
        checks = self.card.checks
        if not self.by:
            return _table([(check.name, f"{_shown(self[check.name], check)} {check.unit}") for check in checks])
        header = (*self.by, "runs", *[check.name if check.unit in check.name else f"{check.name} ({check.unit})"
                                      for check in checks])
        rows = [(*[_label(part) for part in key], self.counts.get(key, ""),
                 *[_shown(values[check.name], check) for check in checks])
                for key, values in sorted(self.items(), key=lambda item: _sort_key(item[0]))]
        return _table([header, *rows], right=set(range(len(self.by), len(header))))


class Report:
    """The checklist's verdicts: each measured value against its row's target. One recording or group prints one line
    per row; groups print, per row, how many groups pass and which fail, with their measured value."""

    def __init__(self, card: Card, measured: Measured) -> None:
        self.card, self.measured = card, measured

    def result(self, name: str, group: tuple = ()) -> str:
        verdict = self.card[name].passes(self.measured.groups()[group][name])
        return "observed" if verdict is None else ("pass" if verdict else "FAIL")

    def passing(self, names: Sequence[str]) -> dict[tuple, bool]:
        """Per group: True when every named row passes."""

        return {group: all(self.result(name, group) == "pass" for name in names) for group in self.measured.groups()}

    @property
    def results(self) -> list[dict[str, Any]]:
        """One record per (group, row), the way Great Expectations lists its validation results."""

        out = []
        for group, values in self.measured.groups().items():
            for check in self.card.checks:
                out.append({"group": dict(zip(self.measured.by, group)), "check": check.name,
                            "measured": values[check.name], "unit": check.unit, "target": check.target,
                            "owner": check.owner, "result": self.result(check.name, group)})
        return out

    def __repr__(self) -> str:
        title = f"{self.card.part} · {self.measured.title}"
        if not self.measured.by:
            values = self.measured.groups()[()]
            rows = [("check", "measured", "target", "owner", "result")]
            rows += [(check.name, f"{_shown(values[check.name], check)} {check.unit}", check.target or "-",
                      check.owner or "-", self.result(check.name)) for check in self.card.checks]
            return title + "\n" + _table(rows)
        groups = self.measured.groups()
        rows = [("check", "target", "owner", "passing", "failing groups")]
        for check in self.card.checks:
            if check.target is None:
                rows.append((check.name, "-", "-", "observed", ""))
                continue
            failing = [f"{' '.join(_label(part) for part in group)}: {_shown(values[check.name], check)} {check.unit}"
                       for group, values in groups.items() if self.result(check.name, group) == "FAIL"]
            if len(failing) > 6:
                failing = failing[:5] + [f"... and {len(failing) - 5} more"]
            passed = sum(self.result(check.name, group) == "pass" for group in groups)
            rows.append((check.name, check.target, check.owner or "-", f"{passed} of {len(groups)}",
                         failing[0] if failing else ""))
            rows += [("", "", "", "", line) for line in failing[1:]]
        return title + "\n" + _table(rows)


# --------------------------------------------------------------------------------------------- recordings in


def as_recording(run: Any) -> Any:
    """Round 1's fixture Run (or a robot_metrics Run) as robot_metrics' Run: the same streams under its names, the
    labels a group-by reads (station, rung, kind, source, run, tip length), a fault as the outcome."""

    if isinstance(run, robot_metrics.Run):
        return run
    streams = {"target": run.target, "state": run.measured}
    if run.plan is not None:
        streams["plan"] = run.plan
    if run.tool_target is not None:
        streams["tool/target"], streams["tool/state"] = run.tool_target, run.tool_measured
    labels = {"station": run.station, **run.labels, "source": run.source, "run": run.id,
              "tip_length_m": run.tip_length_m}
    return robot_metrics.Run(run.id, streams=streams, labels=labels, provenance=run.source,
                             outcome="faulted" if run.faulted else "complete")


def as_runs(data: Any) -> list[Any]:
    if isinstance(data, (list, tuple)):
        return [as_recording(run) for run in data]
    return [as_recording(data)]


def in_chunks(run: Any, rows: int = 7) -> tuple[Any, ...]:
    """The recording as a live feed would deliver it: ``rows`` replies at a time, every stream cut at the same instants."""

    return as_recording(run).chunks(rows)


def group_runs(runs: Sequence[Any], by: Sequence[str]) -> dict[tuple, list[Any]]:
    groups: dict[tuple, list[Any]] = {}
    for run in runs:
        groups.setdefault(tuple(run.labels.get(name) for name in by), []).append(run)
    return dict(sorted(groups.items(), key=lambda item: _sort_key(item[0])))


def _one_tip(runs: Sequence[Any], what: str) -> float:
    tips = {run.labels.get("tip_length_m") for run in runs}
    if len(tips) != 1 or None in tips:
        # WHY refuse instead of pooling: six stations have six levers, so a millimetre at one tip is not one at another
        raise ValueError(f"{what}: a tip number pools runs of one tip length only; group by station")
    return float(tips.pop())


def _title(runs: Sequence[Any]) -> str:
    if len(runs) == 1:
        labels = runs[0].labels
        rung = labels.get("rung")
        return f"1 recording: {labels.get('station')}, rung {_label(rung)}"
    return f"{len(runs)} recordings"


def _shown(value: Any, check: Check) -> str:
    if value is None:
        return "none"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if check.measure.quantity == "count":
        return f"{int(value)}"
    return f"{value:.3f}"


def _label(part: Any) -> str:
    if not isinstance(part, float):
        return str(part)
    text = f"{part:g}"
    return text if "." in text or "e" in text else text + ".0"     # a rung reads 1.0, not 1


def _sort_key(key: tuple) -> tuple:
    # WHY by station number: "can2" before "can11", as a person reads the rack
    return tuple((0, int(match.group(1)), part) if isinstance(part, str) and (match := re.fullmatch(r"\D+(\d+)", part))
                 else (1, 0, part) if isinstance(part, str) else (2, part, "") for part in key)


def _table(rows: Sequence[Sequence[Any]], right: set[int] | None = None) -> str:
    cells = [[str(cell) for cell in row] for row in rows]
    widths = [max(len(row[column]) for row in cells if column < len(row)) for column in range(len(cells[0]))]
    lines = []
    for row in cells:
        parts = [cell.rjust(widths[column]) if right and column in right else cell.ljust(widths[column])
                 for column, cell in enumerate(row)]
        lines.append("  ".join(parts).rstrip())
    return "\n".join(lines)


__all__ = ["Card", "Check", "Live", "Measure", "Measured", "Report", "as_recording", "faulted", "in_chunks", "lag",
           "limiter_share", "median_stop_error", "overshoot", "peak_tracking_error", "performance_gain",
           "repeatability", "rms_tracking_error", "settle_time", "stop_count", "stop_error_percentile",
           "worst_stop_error"]
