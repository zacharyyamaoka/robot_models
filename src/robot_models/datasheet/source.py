"""`Source`, one raw source's header, and `Sources`, the loader of a part's `sources/*.py`.

A part folder holds one `sources/<name>.py` per raw source (a saved page, a PDF, a seller's list, our own estimates).
Each reads like a transcription: the header once, then one plain assignment per value, in the source's own order:

    source = Source("SteadyWin product page", kind="manufacturer", url="https://...", file="product_page.html",
                    sha256="...", read_on="2026-11-16", columns=("GDZ34", "GDS34"))

    nominal_voltage = Stated(24, "V", "额定电压(Nominal Voltage)")
    rated_torque = Stated({"GDZ34": 1.05, "GDS34": 1}, "N·m", "额定扭矩(Nominal Torque)")

    extra = {"noise": Stated(60, "dB", "噪音(Noise)", bound="max")}   # what the type has no field for

Loaded, `source.nominal_voltage` is the `Stated` (with `.as_written` and `.source`), `source["GDS34"]` is the
GDS34 column, and `source.rated_torque` on a two-column source refuses until a column is named.

WHY the .py IS the cache (Zach, 2026-11-16: "I want to cache the result ... I like python as ground truth"): the raw
file is parsed once, by a person or an extractor, into this file; nothing re-parses it at run time. WHY `sha256`:
a cache needs a key, or a re-captured raw file silently disagrees with its parse; the user tests compare it.
"""

from __future__ import annotations

import hashlib
import importlib.util
import pathlib
import types
from collections.abc import Iterator, Mapping
from dataclasses import KW_ONLY, dataclass, field, replace
from typing import Any

from .stated import Stated

# robotparts registry `source_kinds`, less "private" (a publishing flag, not a kind of evidence)
KINDS = ("manufacturer", "distributor", "retailer", "inspected", "derived", "estimated")


class ColumnError(AttributeError):
    """A value read from a source with one column per variant, without naming the column, where the columns differ."""


@dataclass(frozen=True)
class Source:
    """The header of one raw source: what it is, where its bytes are, and how it is laid out.

    - `kind`: robotparts' source kinds (manufacturer, distributor, retailer, inspected, derived, estimated).
    - `file`: the raw file(s) beside the part, relative to the part folder; the first is the one `sha256` pins.
    - `columns`: the source's variant columns (driver boards, windings, units on a rack), when it has several.
    - `applies_to`: the model the document is for, when not this part (a sibling's drawing). Such a source is shown,
      never preferred: a value is taken from it only by an explicit pick.

    `name` (the file's stem), `column`, `folder`, `values` and `extra` are set by the loader.
    """

    title: str
    _: KW_ONLY
    kind: str
    url: str | None = None
    file: str | tuple[str, ...] | None = None
    sha256: str | None = None
    read_on: str | None = None
    columns: tuple[str, ...] = ()
    applies_to: str | None = None
    where: str | None = None
    note: str | None = None
    name: str | None = None
    column: str | None = None
    folder: pathlib.Path | None = field(default=None, compare=False, repr=False)
    values: Mapping[str, Stated] = field(default_factory=dict, compare=False, repr=False)
    extra: Mapping[str, Stated] = field(default_factory=dict, compare=False, repr=False)

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f"Source({self.title!r}): kind {self.kind!r} is not one of {', '.join(KINDS)}")
        if self.column is not None and self.column not in self.columns:
            raise KeyError(f"{self.title}: no column {self.column!r}; it has {', '.join(self.columns) or 'none'}")

    def __hash__(self) -> int:
        return hash((self.name, self.title, self.column))

    # --- the files -------------------------------------------------------------------------------------------

    @property
    def files(self) -> tuple[str, ...]:
        if self.file is None:
            return ()
        return (self.file,) if isinstance(self.file, str) else tuple(self.file)

    @property
    def path(self) -> pathlib.Path | None:
        """The first raw file on disk (it may be absent: raw files are git-ignored)."""
        return self.folder / self.files[0] if self.folder is not None and self.files else None

    def file_sha256(self) -> str | None:
        """The sha256 of the first raw file as it is on disk now, or None when it is not here."""
        path = self.path
        if path is None or not path.is_file():
            return None
        return hashlib.sha256(path.read_bytes()).hexdigest()

    # --- reading it ------------------------------------------------------------------------------------------

    @property
    def label(self) -> str:
        """`steadywin_page·GDS34`: the name a table column and a pick use."""
        return f"{self.name}·{self.column}" if self.column else str(self.name)

    @property
    def is_related(self) -> bool:
        return self.applies_to is not None

    def __getitem__(self, column: str) -> Source:
        """This source's `column` only: every value as that column states it."""
        if column not in self.columns:
            raise KeyError(f"{self.label} has no column {column!r}; it has {', '.join(self.columns) or 'none'}")
        view = replace(self, column=column)
        _fill(view, {name: stated.for_column(column) for name, stated in self.values.items()},
              {name: stated.for_column(column) for name, stated in self.extra.items()})
        return view

    def states(self, name: str) -> Stated | None:
        """What this source states for `name` (a typed field or an extra name), or None. On a source with columns,
        read without one, a value that differs by column refuses: name the column."""
        stated = self.values.get(name) or self.extra.get(name)
        if stated is None:
            return None
        if stated.by_column and self.column is None:
            raise ColumnError(f"{self.label} states {name} per column ({stated.text()}): name one, "
                              f"{self.name}[{next(iter(stated.value))!r}].{name}")
        return stated

    def __getattr__(self, name: str) -> Stated:
        # only reached when normal lookup fails, so a field of Source itself always wins
        if name.startswith("_"):
            raise AttributeError(name)
        values = self.__dict__.get("values", {})
        extra = self.__dict__.get("extra", {})
        if name not in values and name not in extra:
            raise AttributeError(f"{self.label} states no {name!r}")
        return self.states(name)

    def statements(self) -> Iterator[Source]:
        """Itself once per column (or once, when it has none): every view a table shows."""
        if not self.columns or self.column is not None:
            yield self
        else:
            for column in self.columns:
                yield self[column]


def _fill(source: Source, values: dict[str, Stated], extra: dict[str, Stated]) -> None:
    """Bind the values to their source. WHY object.__setattr__: a Stated points at its Source and the Source holds
    its Stateds, so one of the two is filled after it is built; nothing else ever mutates them."""
    bound = {name: replace(stated, name=name, source=source) for name, stated in values.items()}
    bound_extra = {name: replace(stated, name=name, source=source) for name, stated in extra.items()}
    object.__setattr__(source, "values", types.MappingProxyType(bound))
    object.__setattr__(source, "extra", types.MappingProxyType(bound_extra))


def load_source(path: pathlib.Path, part_folder: pathlib.Path | None = None) -> Source:
    """One `sources/<name>.py`, run and checked: it has a `source = Source(...)`, every value is a `Stated`, and a
    per-column value names exactly the header's columns."""
    path = pathlib.Path(path)
    module_name = f"robot_models_source_{abs(hash(str(path.resolve())))}_{path.stem}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    header = getattr(module, "source", None)
    if not isinstance(header, Source):
        raise ValueError(f"{path}: needs `source = Source(...)` (its header)")
    values: dict[str, Stated] = {}
    for name, value in vars(module).items():
        if name.startswith("_") or name in ("source", "extra") or isinstance(value, (types.ModuleType, type)):
            continue
        if callable(value):
            continue
        if not isinstance(value, Stated):
            raise ValueError(f"{path.name}: {name} = {value!r} is not a Stated(...); a source file holds only values")
        values[name] = value
    extra = getattr(module, "extra", {}) or {}
    if not isinstance(extra, dict) or not all(isinstance(value, Stated) for value in extra.values()):
        raise ValueError(f"{path.name}: extra must be {{name: Stated(...)}}")
    for name in set(values) & set(extra):
        raise ValueError(f"{path.name}: {name} is both a value and an extra key")
    for name, stated in {**values, **extra}.items():
        if stated.by_column and set(stated.value) != set(header.columns):
            raise ValueError(f"{path.name}: {name} gives columns {', '.join(stated.value)} but the header has "
                             f"{', '.join(header.columns) or 'none'}")
    source = replace(header, name=path.stem, folder=pathlib.Path(part_folder) if part_folder else path.parent.parent)
    _fill(source, values, dict(extra))
    return source


class Sources:
    """Every `sources/*.py` beside a part file, by name: `sources = Sources(__file__)`, then `sources.steadywin_page`.

    Iterating gives them in name order. A part folder with no `sources/` has none.
    """

    def __init__(self, part_file: str | pathlib.Path):
        part_file = pathlib.Path(part_file).resolve()
        self.folder = part_file.parent
        directory = self.folder / "sources"
        paths = sorted(directory.glob("*.py")) if directory.is_dir() else []
        self._by_name: dict[str, Source] = {path.stem: load_source(path, self.folder) for path in paths
                                            if not path.name.startswith("_")}

    def __iter__(self) -> Iterator[Source]:
        return iter(self._by_name.values())

    def __len__(self) -> int:
        return len(self._by_name)

    def __getitem__(self, name: str) -> Source:
        return self._by_name[name]

    def __getattr__(self, name: str) -> Source:
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self._by_name[name]
        except KeyError:
            raise AttributeError(f"no sources/{name}.py beside {self.folder.name}; have {', '.join(self._by_name)}") from None

    def names(self) -> tuple[str, ...]:
        return tuple(self._by_name)


def is_source(value: Any) -> bool:
    return isinstance(value, Source)
