"""`Source`, one raw source's header, and `Sources`, the loader of a part's `sources/*.py`.

A part folder holds one `sources/<name>.py` per raw source (a saved page, a PDF, a seller's list, our own estimates),
beside the raw file it was read from, which carries the same name (`sources/steadywin_page.html` +
`sources/steadywin_page.py`). Each reads like a transcription: the header once, then the part type called with one
keyword per value, in the source's own order:

    source = Source("SteadyWin product page", kind="manufacturer", role="product_page", url="https://...",
                    file="steadywin_page.html", sha256="...", read_on="2026-11-16", columns=("GDZ34", "GDS34"))

    values = IntegratedServo(
        nominal_voltage=Stated(24, "V", "额定电压(Nominal Voltage)"),
        rated_torque=Stated({"GDZ34": 1.05, "GDS34": 1}, "N·m", "额定扭矩(Nominal Torque)"),
        extra={"noise": Stated(60, "dB", "噪音(Noise)", bound="max")},   # what the type has no field for
    )

WHY the type is called with keywords (Zach, 2026-11-16: "To get type completion you should be filling in these values
from the models"; proposal 1 of five, robotparts reports/source-file-proposals-2026-11-16.html): the editor completes
each field name from the type's own signature and marks an unknown one, and the type stays a frozen dataclass.

A header with no `values` is a raw file catalogued but not transcribed yet (a photo, a CAD file, a manual nobody has
read for values): it still says what the file is (`role`), where it came from and which bytes it is.

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
import re
import types
from collections.abc import Iterator, Mapping
from dataclasses import KW_ONLY, dataclass, field, replace
from typing import Any, Literal, get_args

from .stated import Stated

# robotparts registry `source_kinds`, less "private" (a publishing flag, not a kind of evidence)
KINDS = ("manufacturer", "distributor", "retailer", "inspected", "derived", "estimated")

# What a raw file IS, robotparts user/README.md's file-name vocabulary. WHY a header field and no longer the file name
# (Zach, 2026-11-16: "the file names should be the exact same for the sources ... so they show up next to each other
# in the file viewer"): the name now pairs the raw file with its .py, so what the file is moves into the header.
Role = Literal["datasheet", "2d_drawing", "cad", "product_page", "manual", "seller_chat", "photo"]
ROLES: tuple[str, ...] = get_args(Role)


class ColumnError(AttributeError):
    """A value read from a source with one column per variant, without naming the column, where the columns differ."""


@dataclass(frozen=True)
class Source:
    """The header of one raw source: what it is, where its bytes are, and how it is laid out.

    - `kind`: robotparts' source kinds (manufacturer, distributor, retailer, inspected, derived, estimated): who says.
    - `role`: what the raw document is (`ROLES`: datasheet, 2d_drawing, cad, product_page, manual, seller_chat, photo),
      one or several (a spec sheet that is also the drawing). None for a source that is no document (our estimates).
    - `file`: the raw file(s), relative to the part's sources/ folder and named like the .py (`steadywin_page.html`,
      `steadywin_page(spec_table).html`); the first is the one `sha256` pins.
    - `columns`: the source's variant columns (driver boards, windings, units on a rack), when it has several.
    - `applies_to`: the model the document is for, when not this part (a sibling's drawing). Such a source is shown,
      never preferred: a value is taken from it only by an explicit pick.

    `name` (the file's stem), `column`, `folder`, `values`, `extra` and `type` are set by the loader.
    """

    title: str
    _: KW_ONLY
    kind: str
    role: Role | tuple[Role, ...] | None = None
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
    type: Any = field(default=None, compare=False, repr=False)

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise ValueError(f"Source({self.title!r}): kind {self.kind!r} is not one of {', '.join(KINDS)}")
        for role in self.roles:
            if role not in ROLES:
                raise ValueError(f"Source({self.title!r}): role {role!r} is not one of {', '.join(ROLES)}")
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
    def roles(self) -> tuple[str, ...]:
        if self.role is None:
            return ()
        return (self.role,) if isinstance(self.role, str) else tuple(self.role)

    @property
    def path(self) -> pathlib.Path | None:
        """The first raw file on disk (it may be absent: raw files are git-ignored)."""
        return self.folder / "sources" / self.files[0] if self.folder is not None and self.files else None

    @property
    def states_anything(self) -> bool:
        """False for a header with no values yet: a raw file catalogued, not transcribed."""
        return bool(self.values or self.extra)

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
    """One `sources/<name>.py`, run and checked: it has a `source = Source(...)`, at most one `values = <Type>(...)`
    whose keywords are that type's fields (the type checks each unit and kind as it is built), and a per-column value
    names exactly the header's columns."""
    from .datasheet import Datasheet  # WHY here: datasheet.py imports this module

    path = pathlib.Path(path)
    module_name = f"robot_models_source_{abs(hash(str(path.resolve())))}_{path.stem}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except TypeError as error:
        raise TypeError(f"sources/{path.name}: {_with_hint(error)}") from error
    except ValueError as error:
        raise ValueError(f"sources/{path.name}: {error}") from error
    header = getattr(module, "source", None)
    if not isinstance(header, Source):
        raise ValueError(f"{path}: needs `source = Source(...)` (its header)")
    for name, value in vars(module).items():
        if name.startswith("_") or name in ("source", "values") or isinstance(value, (types.ModuleType, type)):
            continue
        if callable(value):
            continue
        if isinstance(value, Stated):
            raise ValueError(f"sources/{path.name}: {name} = Stated(...) at the top level; write it as a keyword of "
                             f"the type, `values = <Type>({name}=Stated(...), ...)` (PEP 0003 amendment 1)")
        raise ValueError(f"sources/{path.name}: {name} = {value!r}; a source file holds `source = Source(...)` and "
                         f"`values = <Type>(...)` only")
    sheet = getattr(module, "values", None)
    if sheet is not None and not isinstance(sheet, Datasheet):
        raise ValueError(f"sources/{path.name}: values must be a part type called with its fields, "
                         f"`values = IntegratedServo(peak_torque=Stated(...), ...)`, not {type(sheet).__name__}")
    values: dict[str, Stated] = {}
    extra: dict[str, Stated] = {}
    if sheet is not None:
        values = {name: getattr(sheet, name) for name in sheet.typed_fields() if getattr(sheet, name) is not None}
        extra = dict(sheet.extra)
    for name, stated in {**values, **extra}.items():
        if stated.by_column and set(stated.value) != set(header.columns):
            raise ValueError(f"{path.name}: {name} gives columns {', '.join(stated.value)} but the header has "
                             f"{', '.join(header.columns) or 'none'}")
    source = replace(header, name=path.stem, folder=pathlib.Path(part_folder) if part_folder else path.parent.parent,
                     type=type(sheet) if sheet is not None else None)
    _fill(source, values, extra)
    return source


def _with_hint(error: TypeError) -> str:
    """`IntegratedServo.__init__() got an unexpected keyword argument 'nominal_votlage'`, plus the nearest field and
    where a custom value goes: the editor marks the same typo before it ever runs."""
    import difflib

    text = str(error)
    found = re.search(r"(\w+)\.__init__\(\) got an unexpected keyword argument '(\w+)'", text)
    if not found:
        return text
    from .datasheet import Datasheet

    owner = next((cls for cls in _subclasses(Datasheet) if cls.__name__ == found[1]), None)
    fields = list(owner.typed_fields()) if owner else []
    near = difflib.get_close_matches(found[2], fields, n=1)
    hint = f" (did you mean {near[0]}?)" if near else ""
    return (f"{found[2]} is not a field of {found[1]}{hint}; a value the type has no field for goes in "
            f"extra={{...}}")


def _subclasses(cls: type) -> list[type]:
    out = []
    for sub in cls.__subclasses__():
        out += [sub, *_subclasses(sub)]
    return out


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
