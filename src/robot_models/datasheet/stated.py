"""`Stated`: what one source states for one field, written the way the source writes it.

    rated_torque = Stated(1.05, "N·m", "额定扭矩(Nominal Torque)")
    peak_torque = Stated({"GDZ34": 3.82, "GDS34": 3.47}, "N·m", "堵转扭矩(Stall Torque)")   # one value per column

WHY a class of its own and not robotparts' `Spec` or dataclass_values' `Value` (PEP 0003 §4): a `Spec` repeats its
source on every value (`source_url=`, `source_kind=`), which is the "no longer linked" Zach rejected; in a source file
the source is written once, in its header. A `Value` IS the number, for configs, and has no place for the source's
own label or a per-column value. `Stated` is the transcription; the catalogue's `Spec` is generated from it, never
typed twice.
"""

from __future__ import annotations

import urllib.parse
from dataclasses import KW_ONLY, dataclass, field, replace
from typing import Any

from .units import to_si

SIDES = ("output", "rotor")
BOUNDS = ("max", "min")


@dataclass(frozen=True)
class Stated:
    """One value as one source states it.

    - `value`: a number, a text, or `{column: value}` when the source has one column per variant (a driver board,
      a winding). A value the same in every column is written once.
    - `unit`: robotparts' registry spelling (`N·m`, `rpm`, `kg·cm²`; `""` for a count). The source's own spelling
      goes in `as_written` or `note`.
    - `as_written`: the source's own label for the value. It is also the anchor back into the raw file (a text quote),
      so `where` is only needed when the label cannot find it (a drawing with no text layer, a repeated label).
    - `where`: a source_msgs compact selector (`page=1&xywh=percent:18,76,7,2.6`, `table=1&row=7&col=3`).
    - `at`: `"output"` when the source states a rotor-side quantity (Kv, Kt, rotor inertia) after the reducer.
    - `basis`: how an estimate was made. `bound`: `"max"` for "< 60 dB".

    `name`, `column` and `source` are set by the loader, never written by hand.
    """

    value: Any
    unit: str | None = None
    as_written: str | None = None
    _: KW_ONLY
    where: str | None = None
    note: str | None = None
    at: str | None = None
    basis: str | None = None
    bound: str | None = None
    name: str | None = field(default=None, compare=False)
    column: str | None = field(default=None, compare=False)
    source: Any = field(default=None, compare=False, repr=False)
    derived_from: Stated | None = field(default=None, compare=False, repr=False)

    def __post_init__(self) -> None:
        if self.at is not None and self.at not in SIDES:
            raise ValueError(f"at={self.at!r}: say 'output' or 'rotor'")
        if self.bound is not None and self.bound not in BOUNDS:
            raise ValueError(f"bound={self.bound!r}: say 'max' or 'min'")
        if isinstance(self.value, dict) and not self.value:
            raise ValueError("a per-column value needs at least one column")

    # --- columns ---------------------------------------------------------------------------------------------

    @property
    def by_column(self) -> bool:
        """True when the source gives a different value per column (`{"GDZ34": 3.82, "GDS34": 3.47}`)."""
        return isinstance(self.value, dict)

    def for_column(self, column: str) -> Stated:
        """This value as one column states it. A value written once holds for every column."""
        if not self.by_column:
            return replace(self, column=column)
        if column not in self.value:
            raise KeyError(f"{self.name or 'this value'} has no column {column!r}; it has {', '.join(self.value)}")
        return replace(self, value=self.value[column], column=column)

    # --- reading it ------------------------------------------------------------------------------------------

    @property
    def is_number(self) -> bool:
        return isinstance(self.value, (int, float)) and not isinstance(self.value, bool)

    @property
    def si(self) -> float:
        """The value in SI units (rated_torque in N·m, rotor_inertia in kg·m², speeds in rad/s)."""
        if not self.is_number:
            raise TypeError(f"{self.name or 'this value'} is {self.value!r}, not one number")
        return to_si(self.value, self.unit)

    @property
    def anchor(self) -> str | None:
        """Where the value is in its raw file, as a source_msgs compact selector: `where` if written, else the
        source's own label as a text quote (Text Fragments syntax, `text=...`)."""
        if self.where:
            return self.where
        if self.as_written:
            return "text=" + urllib.parse.quote(self.as_written, safe="")
        return None

    @property
    def is_derived(self) -> bool:
        return self.derived_from is not None

    def text(self) -> str:
        """The value and its unit, compactly: `3.82 N·m`, `GDZ34 3.82 · GDS34 3.47`, `< 60 dB`."""
        return self.value_text() + unit_text(self.unit)

    def value_text(self) -> str:
        """The value without its unit: `3.82`, `GDZ34 3.82 · GDS34 3.47`, `< 60`."""
        if self.by_column:
            return " · ".join(f"{column} {_number(value)}" for column, value in self.value.items())
        mark = "< " if self.bound == "max" else "> " if self.bound == "min" else ""
        return mark + _number(self.value)

    def __str__(self) -> str:
        return self.text()


def _number(value: Any) -> str:
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        return f"{value:.6g}"
    if isinstance(value, tuple):
        return " – ".join(_number(item) for item in value)
    return str(value)


def unit_text(unit: str | None) -> str:
    """A unit as it follows a number: ` N·m`, `:1` (no space), nothing for a count."""
    if not unit:
        return ""
    return unit if unit == ":1" else " " + unit
