"""Runs and result tables with Polars' method names (group_by, agg, filter, with_columns, select, pivot, join), numpy
and the stdlib inside.

    runs = Runs(pendulum_runs("real"))                    # one row per recording; its labels are the columns
    table = runs.group_by("station", "rung").agg(tip_stop_error.median(), tip_stop_error.max(), tracking_error.rms())
    table.filter(station="can11")                         # Polars: DataFrame.filter(**constraints)
    table.with_columns(passes=table["worst tip stop (mm)"] <= Threshold(1.0, "mm", owner="Zach, 2026-11-11"))

    live = Live(tip_stop_error.median(), tracking_error.rms())     # the same expressions, fed chunks as they arrive
    for chunk in chunks(run, rows=7):
        live.update(chunk)
    live.result()                                         # a one-row table, the same numbers as offline

WHY not Polars itself: the light-core rule (G5: numpy, robot_msgs and the stdlib at runtime), and neither Polars nor
pandas is in robot_models' venv. The real thing would be a thin layer over Polars: a frame with one row per recording,
list columns holding each recording's values (its stops, its moving rows), and each summary a Polars expression
(``pl.col("tip_stop_error").explode().median()``). This file is the ~300-line stand-in that keeps Polars' names, so the
page reads as Polars would.

WHY a Threshold and not a number inside the metric (Zach, 2026-11-11, "separate the calculation of the metric, and the
checking of thresholds"): a gate is a comparison of a finished table column with a Threshold that carries its owner,
so changing a target never changes a metric, and the median and the worst stop sit side by side in the same table.
"""

from __future__ import annotations

import copy
import dataclasses
import math
import operator
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass
from typing import Any

LABELS = ("id", "station", "source", "deployment", "part_type")   # Run attributes that are columns, beside its labels


# --------------------------------------------------------------------------- a gate's threshold, with its owner


@dataclass(frozen=True)
class Threshold:
    """A limit and who owns it. Compare a table column with it: ``table["worst tip stop (mm)"] <= limit``."""

    value: float
    unit: str
    owner: str

    def __float__(self) -> float:
        return float(self.value)

    def __repr__(self) -> str:
        return f"{self.value!r} {self.unit} (owner: {self.owner})"


# --------------------------------------------------------------------------- columns and tables


class Column:
    """One named column of a table. Comparing it with a number or a Threshold gives a column of True / False."""

    def __init__(self, name: str, values: Iterable[Any]) -> None:
        self.name, self.values = name, list(values)

    def _compare(self, other: Any, compare: Callable[[Any, Any], bool]) -> Column:
        bound = other.values if isinstance(other, Column) else [other] * len(self.values)
        bound = [float(value) if isinstance(value, Threshold) else value for value in bound]
        return Column(self.name, [value is not None and limit is not None and bool(compare(value, limit))
                                  for value, limit in zip(self.values, bound)])

    def __le__(self, other: Any) -> Column:
        return self._compare(other, operator.le)

    def __lt__(self, other: Any) -> Column:
        return self._compare(other, operator.lt)

    def __ge__(self, other: Any) -> Column:
        return self._compare(other, operator.ge)

    def __gt__(self, other: Any) -> Column:
        return self._compare(other, operator.gt)

    def __and__(self, other: Column) -> Column:
        return Column(self.name, [bool(a) and bool(b) for a, b in zip(self.values, other.values)])

    def __or__(self, other: Column) -> Column:
        return Column(self.name, [bool(a) or bool(b) for a, b in zip(self.values, other.values)])

    def __invert__(self) -> Column:
        return Column(self.name, [not bool(value) for value in self.values])

    def sum(self) -> int | float:
        present = [value for value in self.values if value is not None]
        total = sum(present)
        return int(total) if all(isinstance(value, (bool, int)) for value in present) else float(total)

    def max(self) -> Any:
        present = [value for value in self.values if value is not None]
        return max(present) if present else None

    def to_list(self) -> list:
        return list(self.values)

    def __getitem__(self, index: int) -> Any:
        return self.values[index]

    def __iter__(self) -> Iterator[Any]:
        return iter(self.values)

    def __len__(self) -> int:
        return len(self.values)

    def __repr__(self) -> str:
        return f"{self.name}: [{', '.join(_cell(value, _decimals(self.values)) for value in self.values)}]"


def _decimals(values: list) -> int | None:
    """None prints a float as written (a rung, 0.9); 3 decimals otherwise."""

    floats = [value for value in values if isinstance(value, float) and math.isfinite(value)]
    return None if floats and all(round(value, 2) == value for value in floats) else 3


def _cell(value: Any, decimals: int | None) -> str:
    if value is None:
        return "-"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return "inf" if value > 0 else "-inf"
        return repr(value) if decimals is None else f"{value:.{decimals}f}"
    return str(value)


class Table:
    """A small table: named columns of equal length. Printing it shows it."""

    def __init__(self, columns: dict[str, list]) -> None:
        lengths = {len(values) for values in columns.values()}
        if len(lengths) > 1:
            raise ValueError(f"columns of unequal length: {lengths}")
        self._columns = {name: list(values) for name, values in columns.items()}

    @property
    def columns(self) -> list[str]:
        return list(self._columns)

    def __len__(self) -> int:
        return len(next(iter(self._columns.values()), []))

    def __getitem__(self, name: str) -> Column:
        if name not in self._columns:
            raise KeyError(f"no column {name!r}; the columns are {self.columns}")
        return Column(name, self._columns[name])

    def rows(self) -> list[dict[str, Any]]:
        return [{name: values[row] for name, values in self._columns.items()} for row in range(len(self))]

    def item(self) -> Any:
        """The one value of a one-row, one-column table (Polars' DataFrame.item())."""

        if len(self) != 1 or len(self._columns) != 1:
            raise ValueError(f"item() needs a 1 x 1 table, not {len(self)} x {len(self._columns)}")
        return next(iter(self._columns.values()))[0]

    def _take(self, keep: list[bool]) -> Table:
        return Table({name: [value for value, kept in zip(values, keep) if kept]
                      for name, values in self._columns.items()})

    def filter(self, *conditions: Column, **equal_to: Any) -> Table:
        """The rows where every True/False column is True and every named column equals its value."""

        keep = [True] * len(self)
        for condition in conditions:
            keep = [a and bool(b) for a, b in zip(keep, condition.values)]
        for name, wanted in equal_to.items():
            keep = [a and value == wanted for a, value in zip(keep, self._columns[name])]
        return self._take(keep)

    def with_columns(self, **columns: Any) -> Table:
        """A copy with these columns added or replaced: a Column, a list, or one value for every row."""

        out = dict(self._columns)
        for name, values in columns.items():
            if isinstance(values, Column):
                values = values.values
            elif not isinstance(values, (list, tuple)):
                values = [values] * len(self)
            out[name] = list(values)
        return Table(out)

    def select(self, *names: str) -> Table:
        return Table({name: self._columns[name] for name in names})

    def sort(self, *names: str) -> Table:
        order = sorted(range(len(self)), key=lambda row: tuple(_sort_key(self._columns[n][row]) for n in names))
        return Table({name: [values[row] for row in order] for name, values in self._columns.items()})

    def pivot(self, on: str, *, index: str, values: str) -> Table:
        """One row per ``index`` value, one column per ``on`` value, holding ``values`` (Polars' pivot)."""

        rows = sorted(set(self._columns[index]), key=_sort_key)
        heads = sorted(set(self._columns[on]), key=_sort_key)
        found = {(row[index], row[on]): row[values] for row in self.rows()}
        out = {index: rows}
        for head in heads:
            out[str(head)] = [found.get((row, head)) for row in rows]
        return Table(out)

    def join(self, other: Table, on: list[str], suffix: str = " (right)") -> Table:
        """This table's rows with the other's columns beside them, matched on ``on``; a clashing name gets ``suffix``."""

        lookup = {tuple(row[name] for name in on): row for row in other.rows()}
        extra = [name for name in other.columns if name not in on]
        out = dict(self._columns)
        for name in extra:
            target = name + suffix if name in out else name
            out[target] = [lookup.get(tuple(row[key] for key in on), {}).get(name) for row in self.rows()]
        return Table(out)

    def vstack(self, other: Table) -> Table:
        """This table's rows, then the other's (the same columns)."""

        if self.columns != other.columns:
            raise ValueError(f"vstack needs the same columns: {self.columns} vs {other.columns}")
        return Table({name: values + other._columns[name] for name, values in self._columns.items()})

    def transpose(self, *, include_header: bool = False, header_name: str = "column",
                  column_names: list[str] | None = None) -> Table:
        """Rows become columns (Polars' transpose): a wide one-row table reads as a list of name and value."""

        names = column_names or [f"column_{row}" for row in range(len(self))]
        out: dict[str, list] = {header_name: self.columns} if include_header else {}
        for row, name in enumerate(names):
            out[name] = [values[row] for values in self._columns.values()]
        return Table(out)

    def equals(self, other: Table, within: float = 0.0) -> bool:
        """Same columns, same rows; numbers equal to within ``within``."""

        if self.columns != other.columns or len(self) != len(other):
            return False
        for name in self.columns:
            for mine, theirs in zip(self._columns[name], other._columns[name]):
                if isinstance(mine, float) and isinstance(theirs, float):
                    if math.isfinite(mine) and math.isfinite(theirs):
                        if abs(mine - theirs) > within:
                            return False
                    elif mine != theirs:
                        return False
                elif mine != theirs:
                    return False
        return True

    def __repr__(self) -> str:
        if not self._columns:
            return "(empty table)"
        texts, right = {}, {}
        for name, values in self._columns.items():
            decimals = _decimals(values)
            texts[name] = [_cell(value, decimals) for value in values]
            right[name] = not all(isinstance(value, str) for value in values if value is not None)
        widths = {name: max([len(name), *map(len, cells)]) for name, cells in texts.items()}

        def line(cells: dict[str, str]) -> str:
            parts = [cells[name].rjust(widths[name]) if right[name] else cells[name].ljust(widths[name])
                     for name in self._columns]
            return "  ".join(parts).rstrip()

        lines = [line({name: name for name in self._columns})]
        lines += [line({name: texts[name][row] for name in self._columns}) for row in range(len(self))]
        return "\n".join(lines)


def _sort_key(value: Any) -> tuple:
    return (value is None, 0 if value is None else value)


def metrics_table(metrics: Iterable[Any]) -> Table:
    """A part's metrics as a table: the column each one makes, the expression behind it, and whether it runs live."""

    metrics = list(metrics)
    return Table({"metric": [metric.name for metric in metrics], "expression": [repr(metric) for metric in metrics],
                  "live": [metric.live for metric in metrics]})


# --------------------------------------------------------------------------- runs: one row per recording


def _named(expressions: tuple, named: dict[str, Any]) -> dict[str, Any]:
    out = {expression.name: expression for expression in expressions}
    if len(out) != len(expressions):
        raise ValueError("two expressions share a name: name one with agg(name=expression)")
    for name, expression in named.items():
        if name in out:
            raise ValueError(f"two expressions are named {name!r}")
        out[name] = expression
    return out


class Runs:
    """Recordings as the rows of a table: each run's labels are its columns (station, rung, kind, source, id)."""

    def __init__(self, runs: Iterable[Any]) -> None:
        self.runs = tuple(runs)

    @staticmethod
    def label(run: Any, name: str) -> Any:
        return getattr(run, name) if name in LABELS else run.labels.get(name)

    def __len__(self) -> int:
        return len(self.runs)

    def __iter__(self) -> Iterator[Any]:
        return iter(self.runs)

    def filter(self, **equal_to: Any) -> Runs:
        return Runs(run for run in self.runs if all(self.label(run, name) == value for name, value in equal_to.items()))

    def group_by(self, *keys: str) -> GroupBy:
        return GroupBy(self, keys)

    def agg(self, *expressions: Any, **named: Any) -> Table:
        """Every expression over all these runs together: a one-row table."""

        return Table({name: [expression.evaluate(self.runs)] for name, expression in _named(expressions, named).items()})

    def explode(self, measure: Any, *labels: str) -> Table:
        """One row per value of ``measure`` (each stop, each moving row), with the run's labels beside it."""

        from measures import values_of

        labels = labels or ("station", "rung", "kind")
        column = f"{measure.name} ({measure.unit})" if measure.unit else measure.name
        out: dict[str, list] = {name: [] for name in (*labels, column)}
        for run in self.runs:
            values = values_of(measure, run)
            for name in labels:
                out[name] += [self.label(run, name)] * len(values)
            out[column] += [float(value) for value in values]
        return Table(out)

    def explain(self, *expressions: Any, **named: Any) -> Plan:
        """Which computations run over each recording, and which metrics read each one (Polars' LazyFrame.explain)."""

        by_pass: dict[tuple, tuple[str, list[str]]] = {}
        for name, expression in _named(expressions, named).items():
            measure = expression.measure
            by_pass.setdefault(measure.pass_key, (measure.pass_text, []))[1].append(name)
        return Plan(len(_named(expressions, named)), list(by_pass.values()))

    def labels(self, *names: str) -> Table:
        names = names or ("station", "rung", "kind", "source")
        table = {name: [self.label(run, name) for run in self.runs] for name in names}
        table["rows"] = [len(run.measured) for run in self.runs]
        return Table(table)

    def __repr__(self) -> str:
        shown = Runs(self.runs[:3]).labels()
        more = f"\n... {len(self.runs) - 3} more" if len(self.runs) > 3 else ""
        return f"{len(self.runs)} runs, one row per recording:\n{shown!r}{more}"


@dataclass(frozen=True)
class Plan:
    """What explain() prints: the computations over one recording and the metrics each serves."""

    metrics: int
    passes: list[tuple[str, list[str]]]

    def __repr__(self) -> str:
        width = max(len(text) for text, _ in self.passes)
        room = 96 - width - 6
        lines = [f"{len(self.passes)} computations over each recording serve {self.metrics} metrics:"]
        for text, names in self.passes:
            wrapped = [names[0]]
            for name in names[1:]:          # never split a metric's name across lines
                if len(wrapped[-1]) + 2 + len(name) <= room:
                    wrapped[-1] += ", " + name
                else:
                    wrapped[-1] += ","
                    wrapped.append(name)
            lines.append(f"  {text:<{width}}  -> {wrapped[0]}")
            lines += [f"  {'':<{width}}     {rest}" for rest in wrapped[1:]]
        return "\n".join(lines)


class GroupBy:
    def __init__(self, runs: Runs, keys: tuple[str, ...]) -> None:
        self.runs, self.keys = runs, keys

    def agg(self, *expressions: Any, **named: Any) -> Table:
        """One row per group (sorted by its keys), one column per expression over that group's runs."""

        groups: dict[tuple, list] = {}
        for run in self.runs:
            groups.setdefault(tuple(Runs.label(run, key) for key in self.keys), []).append(run)
        order = sorted(groups, key=lambda key: tuple(_sort_key(value) for value in key))
        out = {key: [group[index] for group in order] for index, key in enumerate(self.keys)}
        for name, expression in _named(expressions, named).items():
            out[name] = [expression.evaluate(tuple(groups[group])) for group in order]
        return Table(out)


# --------------------------------------------------------------------------- live: the same expressions on chunks


def chunks(run: Any, rows: int = 7) -> Iterator[Any]:
    """The recording in pieces of ``rows`` rows, as a live feed would deliver it (each piece is a Run of those rows)."""

    for start in range(0, len(run.measured), rows):
        piece = slice(start, start + rows)
        yield dataclasses.replace(
            run, target=run.target[piece], measured=run.measured[piece], plan=None,
            tool_target=run.tool_target[piece] if run.tool_target is not None else None,
            tool_measured=run.tool_measured[piece] if run.tool_measured is not None else None)


class Live:
    """The same expressions as agg(), over one recording arriving in chunks: update(chunk), then result() any time.

    Each computation keeps only what it needs between chunks: the stop finder holds the rows since the last stop (one
    move and one dwell), an RMS holds two numbers, a worst value one. A median keeps every value it has seen (an exact
    median needs them all), so its memory grows with the stops. memory() lists what each one holds now."""

    def __init__(self, *expressions: Any, **named: Any) -> None:
        self.expressions = _named(expressions, named)
        offline_only = [name for name, expression in self.expressions.items() if not expression.live]
        if offline_only:
            raise ValueError(f"{offline_only} need the whole recording; compute them offline with Runs([run]).agg()")
        self.readers: dict[tuple, Any] = {}
        self.running = {name: expression.start() for name, expression in self.expressions.items()}
        self.last_chunk = None

    def update(self, chunk: Any) -> None:
        self._feed(self.readers, self.running, chunk, finish=False)
        self.last_chunk = chunk

    def _feed(self, readers: dict, running: dict, chunk: Any, finish: bool) -> None:
        found = {}
        for expression in self.expressions.values():
            measure = expression.measure
            if measure.pass_key not in found:
                if measure.pass_key not in readers:
                    readers[measure.pass_key] = measure.reader()
                reader = readers[measure.pass_key]
                found[measure.pass_key] = reader.finish() if finish else reader.add(chunk)
        for name, expression in self.expressions.items():
            measure = expression.measure
            running[name].add(measure.pick(found[measure.pass_key], chunk))

    def result(self) -> Table:
        """What the rows so far give: a stop still open counts as if it ended now (on a copy; the feed goes on)."""

        if self.last_chunk is None:
            return Table({name: [None] for name in self.expressions})
        readers, running = copy.deepcopy(self.readers), copy.deepcopy(self.running)
        self._feed(readers, running, self.last_chunk, finish=True)
        return Table({name: [running[name].value()] for name in self.expressions})

    def memory(self) -> Table:
        """What each running summary holds right now: a few numbers, or every value for a median."""

        return Table({"metric": list(self.running), "holds": [running.holds() for running in self.running.values()]})
