"""`Datasheet`: a part type's typed fields, and a part's values resolved from its sources.

A type is a frozen dataclass of typed fields, one per parameter, each declared with `Field(units, definition)`:

    @dataclass(frozen=True)
    class IntegratedServo(BldcMotor):
        gear_ratio: Stated | None = Field(RATIO, "Rotor turns per output turn ...", group=PERFORMANCE)

A part's datasheet is an instance, resolved from its sources by an explicit rule:

    datasheet = IntegratedServo.from_sources(sources, prefer=[steadywin["GDZ34"], sources.estimates],
                                             pick={"kt": steadywin["GDS34"]})

- **prefer**: the first source in the list that states a field gives it.
- **pick**: one line per field taken from elsewhere (the part file says why, beside it).
- **never silently**: every other statement stays readable (`statements`), and `disagreements()` lists each field whose
  sources differ from the chosen value, which the table marks.

WHY typed fields named as robotparts' registry keys (PEP 0003): one vocabulary. A name the registry lacks is the name
the catalogue already uses for it (`rated_phase_current`, `max_axial_load`); robotparts' user tests hold both rules.
WHY unknown names refuse: a typo (`nominal_votlage`) must not land quietly among the custom values; a custom value is
written in the source's `extra = {...}`.
"""

from __future__ import annotations

import dataclasses
import difflib
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, replace
from typing import Any, ClassVar

from .source import ColumnError, Source, Sources
from .stated import Stated

KEY_SPECS = "Key specs"
PERFORMANCE = "Performance"
ELECTRICAL = "Electrical"
MECHANICAL = "Mechanical"
INTERFACES = "Interfaces"
EXTRA = "As the source names them"
# robotparts.dev's datasheet sections, in its order (build/site_pages.py SPEC_GROUPS)
GROUPS = (KEY_SPECS, PERFORMANCE, ELECTRICAL, MECHANICAL, INTERFACES, EXTRA)


@dataclass(frozen=True)
class FieldSpec:
    """What a typed field accepts. `units` maps each accepted spelling to its SI factor; the first is the display
    unit. `side` and `reflect` are for a quantity defined on the rotor that a geared part's source may state at the
    output: value_at_rotor = value_at_output × gear_ratio ** reflect (Kv +1, Kt −1, inertia −2)."""

    units: Mapping[str, float]
    definition: str
    group: str
    kind: str = "number"  # number | integer | text
    positive: bool = False
    side: str | None = None
    reflect: int = 0

    @property
    def unit(self) -> str:
        return next(iter(self.units), "")


def Field(units: Mapping[str, float], definition: str, *, group: str, kind: str = "number", positive: bool = False,
          side: str | None = None, reflect: int = 0) -> Any:
    """Declare a typed field of a part type. WHY a capitalised function returning a dataclass field: the pydantic /
    SQLModel spelling (`x: T = Field(...)`), read as a declaration."""
    if group not in GROUPS or group in (KEY_SPECS, EXTRA):
        raise ValueError(f"group {group!r}: one of {', '.join(GROUPS[1:-1])} (key specs are the type's KEY_SPECS)")
    spec = FieldSpec(dict(units), definition, group, kind, positive, side, reflect)
    # WHY kw_only: a source file names every value (`peak_torque=Stated(...)`), so the editor's signature and
    # completion list the fields by name, and a bare positional Stated can never land in the wrong field
    return field(default=None, kw_only=True, metadata={"datasheet": spec})


@dataclass(frozen=True)
class Disagreement:
    """A field whose sources differ from the value chosen for it."""

    name: str
    chosen: Stated
    others: tuple[Stated, ...]

    def text(self) -> str:
        others = ", ".join(f"{stated.source.label} {stated.text()}" for stated in self.others)
        return f"{self.name}: chose {self.chosen.text()} ({self.chosen.source.label}); differs: {others}"


@dataclass(frozen=True)
class Datasheet:
    """The base of every part type: its typed fields (declared by the subclass), the custom values (`extra`), and,
    when resolved from sources, the sources and the rule that chose each value."""

    KEY_SPECS: ClassVar[tuple[str, ...]] = ()

    extra: Mapping[str, Stated] = field(default_factory=dict, kw_only=True)
    # WHY init=False: set only by from_sources, never written in a source file, so they stay out of the signature the
    # editor shows while he types a source's values
    sources: tuple[Source, ...] = field(default=(), init=False, compare=False, repr=False)
    prefer: tuple[Source, ...] = field(default=(), init=False, compare=False, repr=False)
    picks: Mapping[str, Source] = field(default_factory=dict, init=False, compare=False, repr=False)

    # --- the type ---------------------------------------------------------------------------------------------

    @classmethod
    def type_name(cls) -> str:
        """The part type this datasheet states, as a message names it: `IntegratedServoDatasheet` -> "IntegratedServo".

        WHY (PEP 0004 amendment 2): the datasheet class moved off the type's bare name to `<Type>Datasheet` (the bare
        name is the type's ModelFamily); a refusal still names the field the way the type's vocabulary does
        (`IntegratedServo.rated_torque`), so the messages robotparts and its editor show did not change."""
        return cls.__name__.removesuffix("Datasheet") or cls.__name__

    @classmethod
    def typed_fields(cls) -> dict[str, FieldSpec]:
        """Every typed field of this type, {name: FieldSpec}, in declaration order."""
        return {f.name: f.metadata["datasheet"] for f in dataclasses.fields(cls) if "datasheet" in f.metadata}

    @classmethod
    def group_of(cls, name: str) -> str:
        if name in cls.KEY_SPECS:
            return KEY_SPECS
        spec = cls.typed_fields().get(name)
        return spec.group if spec else EXTRA

    @classmethod
    def rows(cls) -> list[tuple[str, list[str]]]:
        """The typed field names grouped as robotparts.dev groups them: Key specs first, in KEY_SPECS order."""
        fields = cls.typed_fields()
        grouped: dict[str, list[str]] = {group: [] for group in GROUPS}
        grouped[KEY_SPECS] = [name for name in cls.KEY_SPECS if name in fields]
        for name, spec in fields.items():
            if name not in cls.KEY_SPECS:
                grouped[spec.group].append(name)
        return [(group, names) for group, names in grouped.items() if names]

    # --- checking a value -------------------------------------------------------------------------------------

    def __post_init__(self) -> None:
        fields = self.typed_fields()
        for name, spec in fields.items():
            stated = getattr(self, name)
            if stated is not None:
                check_value(self.type_name(), name, spec, stated)
        for name, stated in self.extra.items():
            if name in fields:
                raise ValueError(f"{self.type_name()}: extra {name!r} is a typed field; set it as one")
            if not isinstance(stated, Stated):
                raise TypeError(f"{self.type_name()}: extra {name!r} must be a Stated, not {stated!r}")

    # --- resolving it from sources ----------------------------------------------------------------------------

    @classmethod
    def from_sources(cls, sources: Sources | Iterable[Source], *, prefer: Iterable[Source],
                     pick: Mapping[str, Source] | None = None) -> Datasheet:
        """Each typed field from the first source in `prefer` that states it, unless `pick` names its source.

        Every source is checked against the type first: a name that is not a typed field refuses (with the nearest
        field name), so a custom value must be in the source's `extra`. A source written for another model
        (`applies_to`) is never preferred, only picked.
        """
        everything = tuple(sources)
        prefer = tuple(prefer)
        pick = dict(pick or {})
        fields = cls.typed_fields()
        for source in everything:
            check_source(cls, source)
        for source in prefer:
            if source.is_related:
                raise ValueError(f"{source.label} is for {source.applies_to}, not this part: pick a field from it "
                                 f"explicitly instead of preferring it")
        for name, source in pick.items():
            if name not in fields and not any(name in s.extra for s in everything):
                raise KeyError(f"pick names {name!r}, which is no field of {cls.type_name()} and no source's extra")
            if source.states(name) is None:
                raise KeyError(f"pick takes {name} from {source.label}, which states no {name}")

        def chosen(name: str) -> Stated | None:
            if name in pick:
                return pick[name].states(name)
            for source in prefer:
                stated = source.states(name)
                if stated is not None:
                    return stated
            return None

        values = {name: chosen(name) for name in fields}
        extra_names = [name for source in everything for name in source.extra]
        extra = {name: chosen(name) for name in dict.fromkeys(extra_names)}
        extra = {name: stated for name, stated in extra.items() if stated is not None}
        gear_ratio = values.get("gear_ratio")
        for name, spec in fields.items():
            stated = values[name]
            if stated is not None and stated.at is not None and spec.side is not None and stated.at != spec.side:
                values[name] = reflected(name, spec, stated, gear_ratio)
        sheet = cls(**values, extra=extra)
        for name, value in (("sources", everything), ("prefer", prefer), ("picks", pick)):
            object.__setattr__(sheet, name, value)  # the frozen instance's bookkeeping, filled once, right here
        return sheet

    # --- reading every statement ------------------------------------------------------------------------------

    def chosen(self, name: str) -> Stated | None:
        """The value chosen for a typed field or an extra name."""
        if name in self.typed_fields():
            return getattr(self, name)
        return self.extra.get(name)

    def statements(self, name: str) -> list[Stated]:
        """Every source's (and column's) statement of `name`, in table order: preferred sources first."""
        found = []
        for source in self.columns():
            stated = source.states(name)
            if stated is not None:
                found.append(stated)
        return found

    def stating(self) -> list[Source]:
        """The sources that state at least one value: a header with no values (a photo, a CAD file, a manual not read
        yet) catalogues a raw file and is not a column."""
        return [source for source in self.sources if source.states_anything]

    def columns(self) -> list[Source]:
        """One entry per source and column, in table order: the preferred sources in their order, then the rest by
        name; a source's columns in the source's own order. Only sources that state something."""
        by_name = {source.name: source for source in self.stating()}
        names = dict.fromkeys([source.name for source in self.prefer] + sorted(by_name))
        return [view for name in names if name in by_name for view in by_name[name].statements()]

    def gaps(self) -> list[str]:
        """The typed fields no source states."""
        return [name for name in self.typed_fields() if getattr(self, name) is None]

    def disagreements(self) -> list[Disagreement]:
        """Each field whose sources state something other than the chosen value (compared in SI, on the field's
        side of the reducer), with every differing statement. A source for another model (`applies_to`) is shown
        beside the values but never counted: a sibling's winding is expected to differ."""
        found = []
        fields = self.typed_fields()
        for name in [*fields, *self.extra]:
            chosen = self.chosen(name)
            if chosen is None:
                continue
            spec = fields.get(name)
            others = tuple(stated for stated in self.statements(name) if not stated.source.is_related
                           and not same(self._comparable(spec, stated), self._comparable(spec, chosen)))
            if others:
                found.append(Disagreement(name, chosen, others))
        return found

    def agrees(self, name: str, stated: Stated) -> bool:
        chosen = self.chosen(name)
        if chosen is None:
            return False
        spec = self.typed_fields().get(name)
        return same(self._comparable(spec, stated), self._comparable(spec, chosen))

    def _comparable(self, spec: FieldSpec | None, stated: Stated) -> Any:
        """A statement as a comparable value: SI, carried to the field's side of the reducer (a derived value is
        already there); a text as itself; a number in a unit nobody can convert as (value, unit)."""
        if not stated.is_number:
            return stated.value
        try:
            value = stated.si
        except ValueError:
            return (stated.value, stated.unit)
        if spec is not None and spec.side is not None and stated.at not in (None, spec.side):
            ratio = getattr(self, "gear_ratio", None)
            if ratio is None:
                return (stated.value, stated.unit, stated.at)
            value *= ratio.si ** spec.reflect
        return value


def same(a: Any, b: Any) -> bool:
    if isinstance(a, float) and isinstance(b, float):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-15)
    return a == b


def reflected(name: str, spec: FieldSpec, stated: Stated, gear_ratio: Stated | None) -> Stated:
    """A rotor-side field a source states at the output, carried to the rotor through the gear ratio, marked as
    derived from the statement (which the table still shows as written)."""
    if gear_ratio is None:
        raise ValueError(f"{name} is stated at the {stated.at} ({stated.text()}) but no source gives gear_ratio")
    ratio = gear_ratio.si
    factor = ratio ** spec.reflect if stated.at == "output" else ratio ** -spec.reflect
    value = float(f"{stated.value * factor:.12g}")
    basis = (f"{stated.text()} at the {stated.at} ({stated.source.label if stated.source else 'source'}) "
             f"× gear_ratio {ratio:g}{'' if spec.reflect == 1 else f'^{spec.reflect}'}")
    return replace(stated, value=value, at=None, basis=basis, derived_from=stated)


def check_value(type_name: str, name: str, spec: FieldSpec, stated: Any) -> None:
    """One typed field's value: a single Stated, in one of the field's units, of its kind; else a loud refusal."""
    where = f"{type_name}.{name}"
    if not isinstance(stated, Stated):
        raise TypeError(f"{where} must be a Stated(...), not {stated!r}")
    if stated.by_column:
        # a source's statement may give one value per column (a driver board, a winding): each is checked alone. A
        # resolved datasheet never holds one: from_sources reads each source through one column.
        for column in stated.value:
            check_value(type_name, name, spec, stated.for_column(column))
        return
    unit = stated.unit or ""
    if unit not in spec.units:
        accepted = ", ".join(repr(u) for u in spec.units) or "none"
        raise ValueError(f"{where}: unit {unit!r} is not one this field takes ({accepted}); {spec.definition}")
    if spec.kind in ("number", "integer"):
        if not stated.is_number:
            raise TypeError(f"{where} is {stated.value!r}: a number is needed")
        if spec.kind == "integer" and not float(stated.value).is_integer():
            raise ValueError(f"{where} is {stated.value!r}: a whole number is needed")
        if spec.positive and stated.value <= 0:
            raise ValueError(f"{where} is {stated.value!r}: it must be > 0")
    elif spec.kind == "text" and not isinstance(stated.value, str):
        raise TypeError(f"{where} is {stated.value!r}: a text is needed")


def check_source(cls: type[Datasheet], source: Source) -> None:
    """Every value a source states is a typed field of `cls` in an accepted unit; every extra key is not. A source
    written as another type must be one `cls` extends (a BldcMotor statement can feed an IntegratedServo)."""
    fields = cls.typed_fields()
    if source.type is not None and not issubclass(cls, source.type):
        raise TypeError(f"sources/{source.name}.py is written as {source.type.type_name()}(...), but this part is "
                        f"{cls.type_name()}, which does not extend it")
    for name, stated in source.values.items():
        if name not in fields:
            near = difflib.get_close_matches(name, list(fields), n=1)
            hint = f" (did you mean {near[0]}?)" if near else ""
            raise KeyError(f"sources/{source.name}.py: {name} is not a field of {cls.type_name()}{hint}; "
                           f"a value the type has no field for goes in extra = {{...}}")
        spec = fields[name]
        for column in source.columns or (None,):
            value = stated.for_column(column) if column and stated.by_column else stated
            check_value(f"sources/{source.name}.py", name, spec, value)
        if stated.at is not None and spec.side is None:
            raise ValueError(f"sources/{source.name}.py: {name} says at={stated.at!r}, but {cls.type_name()}.{name} "
                             f"is defined on one side only ({spec.definition})")
    for name in source.extra:
        if name in fields:
            raise KeyError(f"sources/{source.name}.py: extra {name!r} is a typed field of {cls.type_name()}; "
                           f"write it as `{name} = Stated(...)`")


__all__ = [
    "ELECTRICAL",
    "EXTRA",
    "GROUPS",
    "INTERFACES",
    "KEY_SPECS",
    "MECHANICAL",
    "PERFORMANCE",
    "ColumnError",
    "Datasheet",
    "Disagreement",
    "Field",
    "FieldSpec",
    "check_source",
    "check_value",
    "reflected",
]
