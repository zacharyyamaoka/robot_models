"""Plain metric functions over robot_msgs recordings: call one on a run, get back a number in the unit its name says.

    median_tip_stop_mm(run)                      -> 0.29     one recording
    median_tip_stop_mm(runs)                     -> 0.31     several recordings: all their stops together
    scorecard(runs, by=["station", "rung"], metrics=[...])   -> Table: one row per group, one column per metric
    LiveScore(metrics).add(chunk) ... .result()  -> dict     the same numbers while a run is still recording
    chunks(run, rows=7)                          -> the run in pieces of 7 rows, in the order they arrived

Prior art: sklearn.metrics, where ``mean_squared_error(y_true, y_pred)`` is a function that returns a float and its
name says what it measures; pandas for the table (``groupby(...).agg``, ``pivot_table``, ``df["column"]``).

WHY the unit is the end of the name (not a returned ``(value, unit)`` pair): a call then reads as one fact,
``worst_tip_stop_mm(run)`` -> ``1.07``, the value is a plain float you can compare, sort and add, and a table's column
header carries its unit for free. The pair would put the unit beside every number and make every comparison unpack it.

How live and offline agree: every pendulum metric reads one small record per run, ``RunTotals`` (a few running sums and
three numbers per stop), filled row by row. Offline the whole run is one chunk, read once and remembered
(``totals_of``), so every metric on one run shares one read of its rows. Live, ``LiveScore`` fills the same record chunk by chunk
and calls the same functions on it. The sums run row by row in arrival order, so live equals offline bit for bit.

Where each formula comes from (one formula in one place): the tracking error is measured - target on rows that share
an instant (robot_msgs.compare's ``reference - measured`` with the sign flipped, which RMS and |x| ignore); the stop
rule is round 1's ``stock.py`` (proposed home: ``robot_msgs.compare.stops``); overshoot is ``robot_msgs.compare.goal``;
lag is ``robot_msgs.compare.estimate_time_offset``; the five-bar reads ``robot_msgs.compare``'s own ErrorStats.
Nothing here holds a target or a threshold: the gates live in gates.py and read the table.
"""

from __future__ import annotations

import collections
import copy
import dataclasses
import math
import statistics
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import Any

import numpy as np
from robot_msgs.compare import compare, estimate_time_offset, goal

import stock

DEGREES = 180.0 / math.pi
LIMITER_CODES = (97, 98, 99, 102)   # moteus: the power, voltage, current and torque limiters
SETTLE_BAND_MM = 1.0                # what "settled" means: within 1 mm at the tip (the definition, not a gate)
ROBOTDX = "robotdx.diagnostics.stock (io-diagnostics main a7fc658)"


# --------------------------------------------------------------------------- what a metric function carries

def metric_info(*, unit: str, many_runs: str, base: str, references: tuple[str, ...], config: dict | None = None,
                live: bool = True, signal: str = "target, measured", hint: str = "kpi_trend") -> Callable:
    """Attach a metric's facts to a plain function: its unit, how several runs combine, where the formula comes from.

    ``base`` names the unit-free math the function scales into its unit (one base serves several metrics);
    ``live`` says whether LiveScore can compute it chunk by chunk; ``hint`` is the data-ui view that shows it."""

    def attach(function: Callable) -> Callable:
        function.unit = unit
        function.many_runs = many_runs
        function.base = base
        function.config = dict(config or {})
        function.references = tuple(references)
        function.live = live
        function.signal = signal
        function.hint = hint
        return function

    return attach


def metric_card(function: Callable) -> dict[str, Any]:
    """A metric function's facts as one record: name, description, unit, references (Zach's Metric fields)."""

    returns = str(function.__annotations__.get("return", "float"))
    return {"name": function.__name__, "description": function.__doc__, "unit": getattr(function, "unit", ""),
            "many runs": getattr(function, "many_runs", ""), "live": getattr(function, "live", False),
            "value": "bool" if returns == "bool" else "number",
            "references": list(getattr(function, "references", ())),
            "function": f"{function.__module__}.{function.__name__}"}


def metric_table(functions: list[Callable]) -> Table:
    """A part's metric functions as a table: name, unit, how several runs combine, and whether it runs live."""

    rows = []
    for function in functions:
        card = metric_card(function)
        rows.append({"metric": card["name"], "unit": card["unit"], "many runs": card["many runs"],
                     "live": card["live"]})
    return Table(rows)


# --------------------------------------------------------------------------- one run's running totals

@dataclass(frozen=True)
class StopResult:
    """One finished stop, kept as three numbers (the rows behind it are let go)."""

    error_rad: float        # signed: measured - target, the mean over the hold's last 50 ms
    settle_s: float         # from the move's end until the tip stays within 1 mm (inf: it never did)
    overshoot_rad: float    # how far it ran past the target on the way in (robot_msgs.compare.goal)


class RunTotals:
    """What every pendulum metric reads about one run: a few running sums and three numbers per stop.

    Filled with ``add(target, measured)``: offline the whole run is one chunk, live each chunk as it arrives. Besides
    the sums and the stops it keeps only the rows of the move in progress, because the stop rule needs the move that
    led into a stop. A stop is a hold of at least 0.25 s at zero commanded speed on one target (stock.py).
    """

    def __init__(self, run: Any) -> None:
        self.source, self.run_id, self.station = run.source, run.id, run.station
        self.labels = dict(run.labels)
        self.tip_length_m = run.tip_length_m
        self.faulted = bool(run.faulted)
        self.moving_squares = 0.0      # sum of (measured - target)^2 over the rows that move, radians^2
        self.moving_rows = 0
        self.largest_error = 0.0       # the largest |measured - target| so far, radians
        self.rows = 0
        self.limited_rows = 0          # rows where a moteus limiter was active
        self.stops: list[StopResult] = []
        # WHY 1e-3 / tip: 1 mm at the tip in the joint's radians, written as the scorecard writes it
        band = 1e-3 * SETTLE_BAND_MM / run.tip_length_m if run.tip_length_m else math.inf
        self._tracker: stock.StopTracker | None = stock.StopTracker(band=band, joint=run.target.joint_names[0])

    def add(self, target: Any, measured: Any) -> None:
        if not np.array_equal(target.t, measured.t):
            raise ValueError("target and measured must sit on the same instants: align them before scoring")
        error = np.asarray(measured.positions[:, 0], float) - np.asarray(target.positions[:, 0], float)
        finite = np.isfinite(error)
        moving = stock.moving(target) & finite
        # WHY one row at a time: the same additions in the same order whether the rows came as one chunk or many,
        # so live and offline agree bit for bit (a numpy sum per chunk would regroup them)
        for square in (error[moving] ** 2).tolist():
            self.moving_squares += square
        self.moving_rows += int(np.count_nonzero(moving))
        if finite.any():
            self.largest_error = max(self.largest_error, float(np.max(np.abs(error[finite]))))
        codes = measured.channels["fault"] if "fault" in measured.channels else np.zeros(len(measured))
        self.rows += len(measured)
        self.limited_rows += int(np.count_nonzero(np.isin(np.asarray(codes).astype(int), LIMITER_CODES)))
        finished = self._tracker.push_many(np.asarray(measured.t, float), np.asarray(measured.positions[:, 0], float),
                                           np.asarray(target.positions[:, 0], float),
                                           np.asarray(target.velocities[:, 0], float))
        self.stops += [self._numbers(stop) for stop in finished]

    def finished(self) -> RunTotals:
        """A copy with the hold in progress closed, as if the run ended now; this record keeps going."""

        done = copy.copy(self)
        last = copy.deepcopy(self._tracker).finish() if self._tracker is not None else ()
        done.stops = self.stops + [self._numbers(stop) for stop in last]
        done._tracker = None
        return done

    def rows_in_progress(self) -> int:
        return len(self._tracker.since) if self._tracker is not None else 0

    @staticmethod
    def _numbers(stop: stock.Stop) -> StopResult:
        reached = goal(*stop.move(), band=math.inf)[f"positions/{stop.joint}"].overshoot
        return StopResult(stop.error, stop.settle_s, float(reached))


_TOTALS: dict[int, tuple[Any, RunTotals]] = {}
_PASSES: collections.Counter = collections.Counter()


def as_list(runs: Any) -> list:
    """A run on its own, or several: always a list."""

    return list(runs) if isinstance(runs, (list, tuple)) else [runs]


def totals_of(runs: Any) -> list[RunTotals]:
    """Each run's totals: its rows read once, then remembered (a RunTotals passes straight through)."""

    found = []
    for run in as_list(runs):
        if isinstance(run, RunTotals):
            found.append(run)
            continue
        remembered = _TOTALS.get(id(run))
        if remembered is None or remembered[0] is not run:
            totals = RunTotals(run)
            totals.add(run.target, run.measured)
            _PASSES[(run.source, run.id)] += 1
            remembered = _TOTALS[id(run)] = (run, totals.finished())
        found.append(remembered[1])
    return found


def passes_over_rows(run: Any) -> int:
    """How many times this run's rows were read (each metric after the first reuses the totals)."""

    return _PASSES[(run.source, run.id)]


# --------------------------------------------------------------------------- the unit-free math (radians, seconds)

def moving_error_rms(totals: RunTotals) -> float:
    """RMS of measured - target over the rows that move, radians."""

    return math.sqrt(totals.moving_squares / totals.moving_rows) if totals.moving_rows else math.nan


def stop_errors(totals: RunTotals) -> list[float]:
    """|measured - target| at each stop, radians at the joint."""

    return [abs(stop.error_rad) for stop in totals.stops]


def _median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else math.nan


# --------------------------------------------------------------------------- the pendulum's metrics

@metric_info(unit="deg", many_runs="median over the runs", base="moving_error_rms", config={"rows": "moving"},
             references=("robot_msgs.compare.ErrorStats", ROBOTDX + " tracking_error"))
def tracking_rms_deg(runs: Any) -> float:
    """root mean square of measured - target while the arm moves, in degrees"""

    per_run = [DEGREES * moving_error_rms(totals) for totals in totals_of(runs)]
    return _median(per_run)


@metric_info(unit="mm", many_runs="median over the runs", base="moving_error_rms", config={"rows": "moving"},
             references=("robot_msgs.compare.ErrorStats", ROBOTDX + " tracking_error"))
def tip_tracking_rms_mm(runs: Any) -> float:
    """RMS of measured - target while the arm moves, at the tip, in mm"""

    per_run = [1e3 * totals.tip_length_m * moving_error_rms(totals) for totals in totals_of(runs)]
    return _median(per_run)


@metric_info(unit="deg", many_runs="largest over the runs", base="largest_error", config={"rows": "all"},
             references=("robot_msgs.compare.ErrorStats.max_abs", ROBOTDX + " tracking_error"))
def tracking_peak_deg(runs: Any) -> float:
    """the largest |measured - target| in the run, in degrees"""

    return max(DEGREES * totals.largest_error for totals in totals_of(runs))


def tip_stop_errors_mm(runs: Any) -> list[float]:
    """Every stop's |measured - target| at the tip, in mm, in the order the stops happened."""

    errors = []
    for totals in totals_of(runs):
        errors += [1e3 * totals.tip_length_m * error for error in stop_errors(totals)]
    return errors


@metric_info(unit="mm", many_runs="median over all their stops", base="stop_errors", config={"statistic": "median"},
             references=("bam_rack_twin T4 stop accuracy", ROBOTDX + " stop_error"), hint="distribution")
def median_tip_stop_mm(runs: Any) -> float:
    """median distance of the tip from its target at the stops, in mm"""

    return _median(tip_stop_errors_mm(runs))


@metric_info(unit="mm", many_runs="largest over all their stops", base="stop_errors", config={"statistic": "max"},
             references=("bam_rack_twin T4 stop accuracy", ROBOTDX + " stop_error"), hint="distribution")
def worst_tip_stop_mm(runs: Any) -> float:
    """largest distance of the tip from its target at any stop, in mm"""

    errors = tip_stop_errors_mm(runs)
    return max(errors) if errors else math.nan


@metric_info(unit="s", many_runs="median over all their stops", base="settle_times", config={"band_mm": SETTLE_BAND_MM},
             references=("bam_rack_twin T4 settle", "robot_msgs.compare.goal settle"), hint="distribution")
def tip_settle_s(runs: Any) -> float:
    """median time for the tip to get within 1 mm after a move, in seconds"""

    settle_times = [stop.settle_s for totals in totals_of(runs) for stop in totals.stops]
    return _median(settle_times)


@metric_info(unit="", many_runs="all of their stops", base="settle_times", config={"band_mm": SETTLE_BAND_MM},
             references=("bam_rack_twin T4 settle",), hint="gate_matrix")
def all_stops_settled(runs: Any) -> bool:
    """True when the tip got within 1 mm at every stop"""

    return all(math.isfinite(stop.settle_s) for totals in totals_of(runs) for stop in totals.stops)


@metric_info(unit="", many_runs="total over the runs", base="stop_count", references=("stock.py stop rule",))
def stop_count(runs: Any) -> int:
    """how many stops: holds of 0.25 s or more on one target"""

    return sum(len(totals.stops) for totals in totals_of(runs))


@metric_info(unit="deg", many_runs="largest over all their stops", base="stop_overshoots", config={"statistic": "max"},
             references=("robot_msgs.compare.goal overshoot",))
def overshoot_deg(runs: Any) -> float:
    """how far the arm ran past a stop's target on the way in, the largest, in degrees"""

    overshoots = [stop.overshoot_rad for totals in totals_of(runs) for stop in totals.stops]
    return DEGREES * max([0.0] + overshoots)


@metric_info(unit="%", many_runs="share of all their rows", base="limited_rows", config={"codes": LIMITER_CODES},
             references=("moteus fault codes 97, 98, 99, 102", ROBOTDX + " fault_codes"))
def limited_rows_pct(runs: Any) -> float:
    """share of rows where a moteus limiter (power, voltage, current, torque) was on, in %"""

    every_totals = totals_of(runs)
    rows = sum(totals.rows for totals in every_totals)
    return 100.0 * sum(totals.limited_rows for totals in every_totals) / rows if rows else math.nan


@metric_info(unit="", many_runs="how many runs faulted", base="faulted_runs", live=False,
             references=("the runner's outcome per board",))
def faulted_runs(runs: Any) -> int:
    """how many of the runs ended in a fault"""

    return sum(totals.faulted for totals in totals_of(runs))


@metric_info(unit="ms", many_runs="median over the runs", base="time_offset", live=False, signal="plan, measured",
             references=("robot_msgs.compare.estimate_time_offset",))
def lag_ms(runs: Any) -> float:
    """how late the response runs behind the plan, in ms (negative: ahead)"""

    offsets = []
    for run in as_list(runs):
        if isinstance(run, RunTotals) or run.plan is None:
            raise TypeError("lag_ms needs the whole run with its plan; score it when the run has ended")
        offsets.append(1e3 * estimate_time_offset(run.plan, run.measured).offset_s)
    return _median(offsets)


# --------------------------------------------------------------------------- the five-bar's metrics (whole runs)

def _joint_stats(run: Any, joint: str) -> Any:
    return compare(run.target, run.measured).metrics[f"positions/{joint}"]


def _tool_stats(run: Any) -> Any:
    return compare(run.tool_target, run.tool_measured).metrics["position"]


@metric_info(unit="deg", many_runs="median over the runs", base="error_stats",
             config={"statistic": "rms", "joint": "j2"}, live=False, references=("robot_msgs.compare.ErrorStats",))
def j2_tracking_rms_deg(runs: Any) -> float:
    """RMS of measured - target at joint j2 over the whole run, in degrees"""

    return _median([DEGREES * _joint_stats(run, "j2").rms for run in as_list(runs)])


@metric_info(unit="deg", many_runs="median over the runs", base="error_stats",
             config={"statistic": "rms", "joint": "c1"}, live=False, references=("robot_msgs.compare.ErrorStats",))
def c1_tracking_rms_deg(runs: Any) -> float:
    """RMS of measured - target at joint c1 over the whole run, in degrees"""

    return _median([DEGREES * _joint_stats(run, "c1").rms for run in as_list(runs)])


@metric_info(unit="mm", many_runs="median over the runs", base="error_stats", config={"statistic": "rms"},
             live=False, signal="tool target, tool measured", references=("robot_msgs.compare.ErrorStats", "evo APE"))
def tool_tracking_rms_mm(runs: Any) -> float:
    """RMS distance of the L2 tip from its target over the whole run, in mm"""

    return _median([1e3 * _tool_stats(run).rms for run in as_list(runs)])


@metric_info(unit="mm", many_runs="largest over the runs", base="error_stats", config={"statistic": "max_abs"},
             live=False, signal="tool target, tool measured", references=("robot_msgs.compare.ErrorStats", "evo APE"))
def worst_tool_error_mm(runs: Any) -> float:
    """largest distance of the L2 tip from its target in the run, in mm"""

    return max(1e3 * _tool_stats(run).max_abs for run in as_list(runs))


# --------------------------------------------------------------------------- other inputs: repeats, a calibration

def repeatability_rad(repeats: list, k: float = 3.0) -> float:
    """ISO 9283 pose repeatability over runs of the same command: at each stop, the mean distance of the repeats from
    their average plus k standard deviations of it; the worst stop, radians at the joint (nan unless every repeat has
    the same stops)."""

    errors = [[stop.error_rad for stop in totals.stops] for totals in totals_of(repeats)]
    if len(errors) < 2 or len({len(per_repeat) for per_repeat in errors}) != 1:
        return math.nan
    matrix = np.asarray(errors)
    distance = np.abs(matrix - matrix.mean(axis=0))
    return float(np.max(distance.mean(axis=0) + k * distance.std(axis=0, ddof=1)))


@metric_info(unit="mm", many_runs="the runs are the repeats", base="repeatability", config={"k": 3.0}, live=False,
             signal="target, measured of each repeat", references=("ISO 9283:1998 7.2.2", ROBOTDX + " pose_accuracy"))
def tip_repeatability_mm(repeats: list, k: float = 3.0) -> float:
    """ISO 9283 repeatability at the tip over runs of the same command, the worst stop, in mm"""

    return 1e3 * totals_of(repeats)[0].tip_length_m * repeatability_rad(repeats, k)


@metric_info(unit="N·m", many_runs="every row of the runs together", base="residual_rms", config={"field": "effort"},
             live=False, references=("hold-out validation", "robot_models select_best"))
def feedforward_residual_rms_nm(runs: Any) -> float:
    """RMS of measured torque - feed-forward torque over every sample, in N·m"""

    residual = np.concatenate([run.measured.effort[:, 0] - run.target.effort[:, 0] for run in as_list(runs)])
    return float(math.sqrt(np.mean(residual ** 2)))


def performance_gain(metric: Callable, before_runs: Any, after_runs: Any) -> float:
    """How much a metric drops from one set of runs to another (before - after), in the metric's own unit."""

    return metric(before_runs) - metric(after_runs)


# --------------------------------------------------------------------------- many runs: a table

def _label(run: Any, name: str) -> Any:
    return getattr(run, name) if hasattr(run, name) else run.labels[name]


def scorecard(runs: Any, *, by: list[str], metrics: list[Callable]) -> Table:
    """One row per group of runs (``by=["station", "rung"]``), one column per metric: each metric called on its group.

    ``by=[]`` puts every run in one row. A metric decides how several runs combine (its ``many_runs``), so a rung's
    median tip stop is the median of all its runs' stops, not a median of medians."""

    groups: dict[tuple, list] = {}
    for run in as_list(runs):
        key = tuple(_label(run, name) for name in by)
        groups.setdefault(key, []).append(run)
    rows = []
    for key in sorted(groups):
        row: dict[str, Any] = dict(zip(by, key))
        row["runs"] = len(groups[key])
        for metric in metrics:
            row[metric.__name__] = metric(groups[key])
        rows.append(row)
    return Table(rows, keys=by)


def _shown(value: Any, is_key: bool) -> str:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return "-" if value is None else str(value)
    if isinstance(value, float):
        if is_key:
            return repr(value)      # a label such as a rung: 0.9 and 1.0, as written
        return f"{value:.3f}" if math.isfinite(value) else str(value)
    return str(value)


class Table:
    """A small table: a list of row dicts. ``table["column"]`` is that column's values (full precision),
    ``table["new"] = values`` adds a column, ``table[["a", "b"]]`` keeps two columns; printing aligns it, floats
    to 3 decimals."""

    def __init__(self, rows: list[dict], keys: list[str] | tuple = ()) -> None:
        self.rows = [dict(row) for row in rows]
        self.keys = tuple(keys)

    @property
    def columns(self) -> list:
        names: dict = {}
        for row in self.rows:
            names.update(dict.fromkeys(row))
        return list(names)

    def __len__(self) -> int:
        return len(self.rows)

    def __iter__(self) -> Iterator[dict]:
        return iter(self.rows)

    def __getitem__(self, column: Any) -> Any:
        if isinstance(column, list):
            return Table([{name: row.get(name) for name in column} for row in self.rows],
                         keys=[name for name in self.keys if name in column])
        return [row[column] for row in self.rows]

    def __setitem__(self, column: str, values: list) -> None:
        values = list(values)
        if len(values) != len(self.rows):
            raise ValueError(f"{len(values)} values for {len(self.rows)} rows")
        for row, value in zip(self.rows, values):
            row[column] = value

    def pivot(self, index: str, columns: str, values: str) -> Table:
        """pandas' pivot: one row per ``index`` value, one column per ``columns`` value, the cells from ``values``."""

        out: dict[Any, dict] = {}
        for row in self.rows:
            out.setdefault(row[index], {index: row[index]})[row[columns]] = row[values]
        return Table([out[key] for key in sorted(out)], keys=[index])

    def __repr__(self) -> str:
        columns = self.columns
        header = [_shown(column, True) for column in columns]
        body = [[_shown(row.get(column), column in self.keys) for column in columns] for row in self.rows]
        widths = [max([len(header[at])] + [len(line[at]) for line in body]) for at in range(len(columns))]
        left = [all(isinstance(row.get(column), str) for row in self.rows) for column in columns]

        def line(cells: list[str]) -> str:
            parts = [cell.ljust(width) if is_left else cell.rjust(width)
                     for cell, width, is_left in zip(cells, widths, left)]
            return "  ".join(parts).rstrip()

        return "\n".join([line(header)] + [line(cells) for cells in body])


# --------------------------------------------------------------------------- live

def chunks(run: Any, rows: int = 7) -> Iterator[Any]:
    """The run cut into pieces of ``rows`` rows, in order, as a recorder would hand them over while it runs."""

    for start in range(0, len(run.measured), rows):
        end = start + rows
        yield dataclasses.replace(run, target=run.target[start:end], measured=run.measured[start:end], plan=None)


class LiveScore:
    """One run's metrics while it records: ``add`` each chunk of rows as it arrives, ``result()`` at any time.

    It fills the same RunTotals the offline functions read, so ``result()`` after the last chunk equals calling each
    function on the whole run. Memory: six running sums, the rows of the move in progress, and three numbers per
    finished stop; never the whole run."""

    def __init__(self, metrics: list[Callable]) -> None:
        whole_run_only = [metric.__name__ for metric in metrics if not getattr(metric, "live", False)]
        if whole_run_only:
            raise ValueError(f"{', '.join(whole_run_only)} needs the whole run: score it when the run has ended")
        self.metrics = list(metrics)
        self.totals: RunTotals | None = None

    def add(self, chunk: Any) -> None:
        if self.totals is None:
            self.totals = RunTotals(chunk)
        elif (chunk.source, chunk.id) != (self.totals.source, self.totals.run_id):
            raise ValueError(f"this LiveScore is scoring {self.totals.run_id}, not {chunk.id}")
        self.totals.add(chunk.target, chunk.measured)

    def result(self) -> dict[str, Any]:
        if self.totals is None:
            raise ValueError("no rows yet: add a chunk first")
        finished = self.totals.finished()
        return {metric.__name__: metric(finished) for metric in self.metrics}

    def kept_in_memory(self) -> dict[str, int]:
        """What this LiveScore holds right now, counted."""

        if self.totals is None:
            return {"running sums": 0, "rows of the move in progress": 0, "finished stops (3 numbers each)": 0}
        return {"running sums": 6, "rows of the move in progress": self.totals.rows_in_progress(),
                "finished stops (3 numbers each)": len(self.totals.stops)}
