"""A part type's ladder of models (PEP 0004): which parameters each rung needs, and which rung it falls back on.

    from robot_models.ladder import ladders
    servo = ladders()["integrated_servo"]
    servo.parameters("M3.stribeck")     # what the rung needs, inherited ones included (its required names)
    servo.holds("M3.stribeck")          # ... plus what it reads when stated (its optional names)
    servo.fallback("M3.lugre")          # ("M3.lugre", "M3.stribeck", "M2.coulomb_viscous", "M1.datasheet", "M0.ideal")
    servo.canonical("frictionloss_nm")  # "coulomb_friction": another name for a quantity resolves to the type's own
    servo.unpack("thermal_r_c")         # ("thermal_resistance", "thermal_time_constant"): a packed array's quantities

- **One vocabulary per type.** A type lists its parameters once (`vocabulary`); a rung names the ones it adds. Any other
  name for the same quantity is an `Alias`, with a note whenever it is not the same quantity one for one; a name that
  packs several quantities into one array is a `Packed`, never an alias of one of them.
- **A ladder is a tree of bases.** Each rung extends one base and needs everything its base needs; the base chain is its
  fallback. Rung keys are composed from explicit fields (`M{level}.{name}`) and never parsed.
- **Required or optional.** `adds` are what the rung's model cannot run without; `optional` are what it reads when a
  source states them (a default or another route covers them otherwise). A name optional below may become required
  above (M1 states coulomb_friction, M2 needs it); nothing is added twice down one chain.
- **Each rung shows its equation and its references.** `equation` and `references` on every rung.
- **Type data only.** Parameter names, units, definitions and equations: never a value measured on one unit, never a
  maker. What a deployment sets (gains, caps, the station's gravity) is listed as `DeploymentOnly`, not put on a rung.

Each type folder holds one `ladder.py` with `LADDER = Ladder(...)`; `ladders()` finds them by walking the package, so no
hand-kept registry exists for concurrent edits to collide on. Stdlib only, like robot_models.datasheet.
"""

from __future__ import annotations

import difflib
import importlib
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

KINDS = ("physics", "residual", "learned")
# WHY fixed levels for the two learned kinds (sim2real decision-rung-keys-2026-11-05, kept by PEP 0004): M6 is always
# the grey-box residual and M7 always the black-box net, on every type, so "M6" means the same thing everywhere.
RESIDUAL_LEVEL = 6
LEARNED_LEVEL = 7
TOP_PHYSICS_LEVEL = 5

WEAR_LAWS = {
    "linear": "p(u) = p0 + k·u",
    "sqrt": "p(u) = p0 + k·√u",
    "archard": "p(u) = p0 + k·Σ(load·distance): wear grows with the load carried times the distance slid",
}

DEPLOYMENT_ROLES = {
    "config": "a setting a deployment writes (a gain, a cap, a rate): read from its config, never fitted to the type",
    "assembly": "a property of where the part is mounted (the station's gravity, a base frame): the description's",
}

# WHY refuse these endings (robotparts SCHEMA2: "Names carry no units"; PEP 0004 decision 1): the unit lives on the
# Parameter, so one quantity cannot get a second name per unit (frictionloss_nm, armature_kg_m2). Direction words that
# look like units (fov_h, resolution_v) are deliberately not in the list.
UNIT_SUFFIXES = (
    "_nm", "_nm_a", "_nm_rad", "_nm_s_rad", "_n_m", "_n_s_m", "_kg_m2", "_kg", "_rad_s", "_rad", "_deg", "_rpm",
    "_mm", "_ms", "_hz", "_ohm", "_px", "_pct", "_per_h", "_per_sqrt_h", "_per_km", "_per_cycle", "_per_nm_rev",
    "_s", "_m", "_n",
)

# WHY a table for kv alone: robotparts' registry spells kv only in rpm/V; its SI unit is rad/(s·V) (registry si_unit).
SI_SPELLING = {"rpm/V": "rad/(s·V)"}

_WORD = re.compile(r"^[a-z][a-z0-9_]*$")


def _word(what: str, text: object) -> None:
    if not isinstance(text, str) or not _WORD.match(text):
        raise ValueError(f"{what} {text!r} must be a snake_case word: lower-case letters, digits and _")


def _hint(name: str, known) -> str:
    near = difflib.get_close_matches(name, list(known), n=1)
    return f" (did you mean {near[0]!r}?)" if near else ""


def _ordered(names) -> tuple[str, ...]:
    """Names in first-seen order, each once."""
    return tuple(dict.fromkeys(names))


# --- the records -----------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Reference:
    """Where to learn a model: a real, citable work (sim2real's cards check DOIs and URLs; this keeps their shape)."""

    title: str               # authors (year). Title. Venue volume(issue):pages
    url: str = ""            # "" when the work has no stable URL
    kind: str = "paper"      # paper | book | chapter | standard | doc | code | preprint
    doi: str = ""
    note: str = ""           # what in it the rung uses

    def __post_init__(self) -> None:
        if not self.title.strip():
            raise ValueError("a reference needs its title: authors (year). Title. Venue")


@dataclass(frozen=True)
class Parameter:
    """One quantity in a type's vocabulary: its canonical name, its SI unit and what it means."""

    name: str                # canonical: a registry key, a typed field, a rung term, else sim2real's name less its unit
    unit: str                # SI ("" for a ratio or a count); a wear rate is per its usage unit (N·m/h, N/km)
    definition: str          # one line
    shape: str = "scalar"    # scalar | integer | text | pair | list | map | pose | per joint | per link
    note: str = ""

    def __post_init__(self) -> None:
        _word("parameter", self.name)
        for suffix in UNIT_SUFFIXES:
            if self.name.endswith(suffix):
                raise ValueError(f"parameter {self.name!r} carries a unit in its name ({suffix!r}): names carry no "
                                 f"units; name it {self.name[: -len(suffix)]!r} with unit=... and list "
                                 f"{self.name!r} as an Alias")
        if not isinstance(self.unit, str):
            raise TypeError(f"parameter {self.name}: unit must be a str ('' when dimensionless), not {self.unit!r}")
        if not self.definition.strip():
            raise ValueError(f"parameter {self.name}: a one-line definition is required")
        if not self.shape.strip():
            raise ValueError(f"parameter {self.name}: shape is 'scalar' unless it says otherwise")


@dataclass(frozen=True)
class Wear:
    """A wear rung's statement that `parameter` grows with use: p(u) = law(p0, rate, u). The rung adds `rate`."""

    parameter: str           # the parameter that wears (the rung holds it)
    usage: str               # deployment hours | vibration hours | belt distance | torque-weighted revolutions ...
    usage_unit: str          # h | km | N·m·rev | cycle
    law: str                 # a WEAR_LAWS key
    rate: str                # the rate parameter, one the rung adds

    def __post_init__(self) -> None:
        _word("wear parameter", self.parameter)
        _word("wear rate", self.rate)
        if self.law not in WEAR_LAWS:
            raise ValueError(f"wear of {self.parameter}: law {self.law!r} is not one of {', '.join(WEAR_LAWS)}")
        if not self.usage.strip() or not self.usage_unit.strip():
            raise ValueError(f"wear of {self.parameter}: say what it wears with (usage) and in what unit (usage_unit)")


@dataclass(frozen=True)
class Alias:
    """Another name for ONE canonical parameter: an old key, a sim2real twin name, an atlas or exporter column."""

    name: str
    canonical: str
    note: str = ""           # required in spirit whenever the alias is not the same quantity one for one (side, phase
    #                          vs line to line, a level vs a difference, as mounted vs the part alone)
    # WHY a unit on the alias (PEP 0004, the physics review): od_mm, camera_latency_ms or an unsuffixed old key can
    # otherwise be converted only by parsing the name. "" means the canonical parameter's own SI unit.
    unit: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip() or self.name != self.name.strip():
            raise ValueError(f"alias {self.name!r}: a non-empty name with no surrounding spaces")
        _word(f"alias {self.name}'s canonical", self.canonical)
        if not isinstance(self.unit, str):
            raise TypeError(f"alias {self.name}: unit is a str ('' for the canonical SI unit), not {self.unit!r}")


@dataclass(frozen=True)
class Packed:
    """A name for an array that packs several canonical quantities: MuJoCo's `thermal`, the rack twin's `thermal_r_c`.

    WHY not an Alias of its first entry (PEP 0004, the fidelity review): `thermal_r_c -> thermal_resistance` would sort
    a (K/W, s) pair into a scalar's row of a parameter × source table. `canonical()` refuses a packed name; `unpack()`
    names what it carries.
    """

    name: str
    holds: tuple[str, ...]   # the canonical quantities it carries
    note: str = ""           # what else it carries, its unit, where its layout is defined
    ordered: bool = False    # True when `holds` is its full entry order; False when it carries at least these

    def __post_init__(self) -> None:
        object.__setattr__(self, "holds", tuple(self.holds))
        if not isinstance(self.name, str) or not self.name.strip() or self.name != self.name.strip():
            raise ValueError(f"packed {self.name!r}: a non-empty name with no surrounding spaces")
        if not self.holds:
            raise ValueError(f"packed {self.name}: name the canonical quantities it carries in holds=(...)")
        for name in self.holds:
            _word(f"packed {self.name} holds", name)
        if len(set(self.holds)) != len(self.holds):
            raise ValueError(f"packed {self.name} names a quantity twice: {self.holds}")


@dataclass(frozen=True)
class DeploymentOnly:
    """A name some model uses that is not the type's: a deployment sets it (config) or the mount gives it (assembly)."""

    name: str
    role: str                # a DEPLOYMENT_ROLES key
    why: str                 # where it belongs instead

    def __post_init__(self) -> None:
        if self.role not in DEPLOYMENT_ROLES:
            raise ValueError(f"deployment-only {self.name}: role {self.role!r} is not one of "
                             f"{', '.join(DEPLOYMENT_ROLES)} (a driver's parameter belongs to the foc_controller type)")
        if not self.why.strip():
            raise ValueError(f"deployment-only {self.name}: say where it belongs instead")


@dataclass(frozen=True)
class Rung:
    """One model on a type's ladder. It needs everything its base needs plus `adds`; its fallback is its base."""

    level: int               # robotparts' layer: 0 ideal, 1 datasheet, 2 Coulomb + viscous, 3 state-dependent friction,
    #                          4 drivetrain, 5 electrics and heat; 6 the residual, 7 the learned net
    name: str                # a word unique among the rungs of its level: "stribeck", "wear"
    title: str               # one line
    base: str | None         # the key of the rung it extends; None for the root (M0) and for a learned net
    adds: tuple[str, ...] = ()       # required: new over its base, or optional below and required from here
    kind: str = "physics"
    # WHY optional (PEP 0004; robotparts' model.json marks each term required or not): the coverage consumer counts
    # what a rung needs. M1.datasheet states every typed field but needs none of them beyond M0, as robotparts' own
    # M1 marks all its terms required: false; stribeck_exponent is 2 when absent.
    optional: tuple[str, ...] = ()
    code: str = ""           # a folder under robot_models with the runnable equation, e.g. "actuators/rungs/stribeck"
    wears: tuple[Wear, ...] = ()
    # WHY pairs and not copies (PEP 0004 decision 4): a rung that needs another type's parameters (an integrated servo
    # needs its driver's delay) names that type's rung; the parameters stay in the other type's vocabulary.
    composes: tuple[tuple[str, str], ...] = ()
    equation: str = ""       # the model, in the parameters' canonical names (or the source's own, said so)
    references: tuple[Reference, ...] = ()
    note: str = ""

    @property
    def key(self) -> str:
        # WHY composed, never parsed (sim2real CONTRACT "never parse a rung key"): level and name are the fields.
        return f"M{self.level}.{self.name}"

    def __post_init__(self) -> None:
        for name in ("adds", "optional", "wears", "composes", "references"):
            object.__setattr__(self, name, tuple(getattr(self, name)))
        object.__setattr__(self, "composes", tuple(tuple(pair) for pair in self.composes))
        if isinstance(self.level, bool) or not isinstance(self.level, int) or not 0 <= self.level <= LEARNED_LEVEL:
            raise ValueError(f"rung {self.name!r}: level {self.level!r} must be an int from 0 to {LEARNED_LEVEL}")
        _word("rung name", self.name)
        key = self.key
        if not self.title.strip():
            raise ValueError(f"{key}: a one-line title is required")
        if self.kind not in KINDS:
            raise ValueError(f"{key}: kind {self.kind!r} is not one of {', '.join(KINDS)}")
        expected = {"residual": RESIDUAL_LEVEL, "learned": LEARNED_LEVEL}.get(self.kind)
        if expected is not None and self.level != expected:
            raise ValueError(f"{key}: a {self.kind} rung is M{expected} on every type; set level={expected}")
        if self.kind == "physics" and self.level > TOP_PHYSICS_LEVEL:
            raise ValueError(f"{key}: physics rungs stop at M{TOP_PHYSICS_LEVEL}; M6 is the residual, M7 the net")
        if self.base is None and self.kind == "physics" and self.level != 0:
            raise ValueError(f"{key}: only the root M0 has no base; name the rung it extends, e.g. base='M0.ideal'")
        if self.base is not None and self.kind == "learned":
            raise ValueError(f"{key}: a learned net extends no model and needs none of its parameters: base=None")
        if self.base is None and self.kind == "residual":
            raise ValueError(f"{key}: a residual corrects a physics rung: name it as base")
        if self.base is not None and not isinstance(self.base, str):
            raise TypeError(f"{key}: base is a rung key (a str) or None, not {self.base!r}")
        if self.kind != "physics" and (self.adds or self.optional):
            raise ValueError(f"{key}: a {self.kind} rung's weights are not named parameters: adds=(), optional=()")
        if self.kind != "physics" and self.wears:
            raise ValueError(f"{key}: only a physics rung wears")
        for name in (*self.adds, *self.optional):
            _word(f"{key} adds", name)
        if len(set(self.adds)) != len(self.adds) or len(set(self.optional)) != len(self.optional):
            raise ValueError(f"{key} names a parameter twice: adds {self.adds}, optional {self.optional}")
        both = sorted(set(self.adds) & set(self.optional))
        if both:
            raise ValueError(f"{key}: {', '.join(both)} is both required (adds) and optional; pick one")
        if self.code and (self.code.startswith("/") or "\\" in self.code or ".." in self.code.split("/")):
            raise ValueError(f"{key}: code {self.code!r} is a folder relative to robot_models, e.g. "
                             f"'actuators/rungs/stribeck'")
        for pair in self.composes:
            if len(pair) != 2:
                raise ValueError(f"{key}: composes holds (type, rung key) pairs, not {pair!r}")
            _word(f"{key} composes type", pair[0])
        for reference in self.references:
            if not isinstance(reference, Reference):
                raise TypeError(f"{key}: references holds Reference(...)s (robot_models.references), not "
                                f"{reference!r}")


# --- the ladder ------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Ladder:
    """A type's vocabulary and its rungs. Built once per type folder, in that folder's `ladder.py` as `LADDER`."""

    type: str                # the type id: "integrated_servo", "depth_camera"
    title: str               # one line: what the type is, with no maker in it
    vocabulary: tuple[Parameter, ...]
    rungs: tuple[Rung, ...]
    aliases: tuple[Alias, ...] = ()
    packed: tuple[Packed, ...] = ()
    deployment: tuple[DeploymentOnly, ...] = ()
    # WHY (PEP 0003 §6, PEP 0004 decision 3): where a type has a Datasheet class, its typed fields ARE rung M1.datasheet
    datasheet: type | None = None
    note: str = ""

    def __post_init__(self) -> None:
        _word("type", self.type)
        if not self.title.strip():
            raise ValueError(f"{self.type}: a one-line title is required")
        for name in ("vocabulary", "rungs", "aliases", "packed", "deployment"):
            object.__setattr__(self, name, tuple(getattr(self, name)))
        self._check_vocabulary()
        self._check_rungs()
        self._check_names()
        if self.datasheet is not None:
            self._check_datasheet()

    # --- reading it ------------------------------------------------------------------------------------------------

    @property
    def keys(self) -> tuple[str, ...]:
        return tuple(rung.key for rung in self.rungs)

    @property
    def names(self) -> tuple[str, ...]:
        """The canonical parameter names, in vocabulary order."""
        return tuple(parameter.name for parameter in self.vocabulary)

    def rung(self, key: str) -> Rung:
        for rung in self.rungs:
            if rung.key == key:
                return rung
        raise KeyError(f"{self.type} has no rung {key!r}{_hint(key, self.keys)}; its rungs: {', '.join(self.keys)}")

    def fallback(self, key: str) -> tuple[str, ...]:
        """The rung and every rung below it through its bases, down to the root: what it falls back on, in order."""
        chain = [key]
        rung = self.rung(key)
        while rung.base is not None:
            chain.append(rung.base)
            rung = self.rung(rung.base)
        return tuple(chain)

    def parameters(self, key: str) -> tuple[str, ...]:
        """Every parameter the rung NEEDS, inherited ones included: the root's first, the rung's own last.

        WHY required only (PEP 0004 decision 8, robotparts' coverage KPI): "the specific terms we are looking for", so a
        part that states no flange size still reaches M2.coulomb_viscous.
        """
        return _ordered(name for step in reversed(self.fallback(key)) for name in self.rung(step).adds)

    def optional(self, key: str) -> tuple[str, ...]:
        """What the rung reads when a source states it, through its bases, less what it needs."""
        needed = set(self.parameters(key))
        return _ordered(name for step in reversed(self.fallback(key)) for name in self.rung(step).optional
                        if name not in needed)

    def holds(self, key: str) -> tuple[str, ...]:
        """Everything the rung reads: what it needs, then what it reads when stated."""
        return (*self.parameters(key), *self.optional(key))

    def composed(self, key: str) -> dict[str, str]:
        """{type: rung key} of the other types this rung needs, through its bases; the nearest rung's choice wins."""
        found: dict[str, str] = {}
        for step in self.fallback(key):
            for other, rung_key in self.rung(step).composes:
                found.setdefault(other, rung_key)
        return found

    def canonical(self, name: str) -> str:
        """The type's own name for `name`: itself when canonical, the alias's target otherwise."""
        if name in self.names:
            return name
        for alias in self.aliases:
            if alias.name == name:
                return alias.canonical
        packed = self.packing(name)
        if packed is not None:
            raise KeyError(f"{name!r} packs {', '.join(packed.holds)} into one array, so it is no single parameter of "
                           f"{self.type}: unpack it with ladder.unpack({name!r})")
        known = [*self.names, *(alias.name for alias in self.aliases)]
        raise KeyError(f"{name!r} is neither a parameter nor an alias of {self.type}{_hint(name, known)}")

    def unpack(self, name: str) -> tuple[str, ...]:
        """The canonical quantities a packed name carries."""
        packed = self.packing(name)
        if packed is None:
            raise KeyError(f"{name!r} is not a packed name of {self.type}{_hint(name, [p.name for p in self.packed])}")
        return packed.holds

    def resolve(self, name: str) -> tuple[str, ...]:
        """The canonical names behind any name the type knows: (canonical,) for a parameter or an alias, the
        quantities a packed name carries. A deployment-only name or an unknown one raises KeyError."""
        return self.unpack(name) if self.packing(name) is not None else (self.canonical(name),)

    def parameter(self, name: str) -> Parameter:
        canonical = self.canonical(name)
        return next(parameter for parameter in self.vocabulary if parameter.name == canonical)

    def alias(self, name: str) -> Alias | None:
        return next((alias for alias in self.aliases if alias.name == name), None)

    def packing(self, name: str) -> Packed | None:
        return next((packed for packed in self.packed if packed.name == name), None)

    # --- checking it -----------------------------------------------------------------------------------------------

    def _check_vocabulary(self) -> None:
        seen: set[str] = set()
        for parameter in self.vocabulary:
            if not isinstance(parameter, Parameter):
                raise TypeError(f"{self.type}: the vocabulary holds Parameter(...)s, not {parameter!r}")
            if parameter.name in seen:
                raise ValueError(f"{self.type}: {parameter.name} is in the vocabulary twice; one quantity, one entry")
            seen.add(parameter.name)

    def _check_rungs(self) -> None:
        by_key: dict[str, Rung] = {}
        for rung in self.rungs:
            if not isinstance(rung, Rung):
                raise TypeError(f"{self.type}: rungs holds Rung(...)s, not {rung!r}")
            if rung.key in by_key:
                raise ValueError(f"{self.type}: two rungs are {rung.key}; a level's rungs need different names")
            by_key[rung.key] = rung
        for rung in self.rungs:
            if rung.base is None:
                continue
            if rung.base not in by_key:
                raise KeyError(f"{self.type} {rung.key}: base {rung.base!r} is no rung of this ladder"
                               f"{_hint(rung.base, by_key)}; base is the key of the rung it extends")
            if by_key[rung.base].level > rung.level:
                raise ValueError(f"{self.type} {rung.key}: its base {rung.base} is a higher layer; a rung falls back "
                                 f"down the ladder, never up")
        roots = [rung.key for rung in self.rungs if rung.base is None and rung.kind == "physics"]
        if len(roots) != 1:
            raise ValueError(f"{self.type}: a ladder has one root, M0 with base=None; found {roots or 'none'}")
        for rung in self.rungs:  # WHY before anything walks the bases: a loop would never end
            path, step = [rung.key], rung
            while step.base is not None:
                if step.base in path:
                    loop = " -> ".join([*path[path.index(step.base):], step.base])
                    raise ValueError(f"{self.type}: the bases loop ({loop}); every chain of bases ends at the root "
                                     f"{roots[0]}")
                path.append(step.base)
                step = by_key[step.base]
        vocabulary = set(self.names)
        for rung in self.rungs:
            for name in (*rung.adds, *rung.optional):
                if name not in vocabulary:
                    raise KeyError(f"{self.type} {rung.key} adds {name!r}, which is not in the {self.type} "
                                   f"vocabulary{_hint(name, vocabulary)}; add a Parameter for it, or, if it is "
                                   f"another name for one, add the canonical name and list {name!r} as an Alias")
            needed = set(self.parameters(rung.base)) if rung.base else set()
            stated = set(self.optional(rung.base)) if rung.base else set()
            for name in rung.adds:
                if name in needed:
                    raise ValueError(f"{self.type} {rung.key} adds {name}, which {self._owner(rung.base, name)} "
                                     f"below it already adds: a rung adds only what is new over its base; drop it")
            for name in rung.optional:
                if name in needed or name in stated:
                    raise ValueError(f"{self.type} {rung.key} lists {name} as optional, which "
                                     f"{self._owner(rung.base, name)} below it already holds; drop it")
            holds = needed | stated | set(rung.adds) | set(rung.optional)
            for wear in rung.wears:
                if wear.rate not in rung.adds:
                    raise ValueError(f"{self.type} {rung.key}: wear rate {wear.rate} must be a parameter this rung "
                                     f"adds")
                if wear.parameter not in holds or wear.parameter == wear.rate:
                    raise ValueError(f"{self.type} {rung.key}: {wear.parameter} wears, but the rung does not hold it "
                                     f"(its parameters: {', '.join(sorted(holds))})")
        on_a_rung = {name for rung in self.rungs for name in (*rung.adds, *rung.optional)}
        orphans = [name for name in self.names if name not in on_a_rung]
        if orphans:
            raise ValueError(f"{self.type}: {', '.join(orphans)} no rung adds; the vocabulary is what the ladder "
                             f"holds, so put each on the rung that first reads it (optional when it only states it) "
                             f"or drop it")

    def _owner(self, base: str, name: str) -> str:
        return next(step for step in self.fallback(base)
                    if name in self.rung(step).adds or name in self.rung(step).optional)

    def _check_names(self) -> None:
        vocabulary = set(self.names)
        seen: set[str] = set()
        for alias in self.aliases:
            if not isinstance(alias, Alias):
                raise TypeError(f"{self.type}: aliases holds Alias(...)es, not {alias!r}")
            if alias.name in vocabulary:
                raise ValueError(f"{self.type}: {alias.name} is canonical, so it cannot also be an alias")
            if alias.canonical not in vocabulary:
                raise KeyError(f"{self.type}: alias {alias.name} points at {alias.canonical!r}, which is not in the "
                               f"vocabulary{_hint(alias.canonical, vocabulary)}")
            if alias.name in seen:
                raise ValueError(f"{self.type}: alias {alias.name} is listed twice")
            seen.add(alias.name)
        for packed in self.packed:
            if not isinstance(packed, Packed):
                raise TypeError(f"{self.type}: packed holds Packed(...)s, not {packed!r}")
            if packed.name in vocabulary or packed.name in seen:
                raise ValueError(f"{self.type}: {packed.name} is a parameter or an alias of the type, so it cannot "
                                 f"also be a packed name")
            for name in packed.holds:
                if name not in vocabulary:
                    raise KeyError(f"{self.type}: packed {packed.name} holds {name!r}, which is not in the "
                                   f"vocabulary{_hint(name, vocabulary)}")
            seen.add(packed.name)
        for other in self.deployment:
            if not isinstance(other, DeploymentOnly):
                raise TypeError(f"{self.type}: deployment holds DeploymentOnly(...)s, not {other!r}")
            if other.name in vocabulary or other.name in seen:
                raise ValueError(f"{self.type}: {other.name} is a parameter, an alias or a packed name of the type, "
                                 f"so it is not deployment-only")
            seen.add(other.name)
        for rung in self.rungs:
            for other, _ in rung.composes:
                if other == self.type:
                    raise ValueError(f"{self.type} {rung.key} composes its own type; a rung's base says that")

    def _check_datasheet(self) -> None:
        fields = self.datasheet.typed_fields()
        for name, spec in fields.items():
            if name not in self.names:
                raise KeyError(f"{self.type}: the {self.datasheet.__name__} field {name} is not in the vocabulary; "
                               f"start it from datasheet_vocabulary({self.datasheet.__name__})")
            unit, expected = self.parameter(name).unit, si_unit(spec.units)
            if unit != expected:
                raise ValueError(f"{self.type}: {name} has unit {unit!r}, but {self.datasheet.__name__}.{name} is "
                                 f"{expected!r} in SI")
        if "M1.datasheet" not in self.keys:
            raise KeyError(f"{self.type}: a type with a Datasheet class has rung M1.datasheet, its typed fields")
        held = set(self.holds("M1.datasheet"))
        if held != set(fields):
            missing, extra = sorted(set(fields) - held), sorted(held - set(fields))
            raise ValueError(f"{self.type}: M1.datasheet holds exactly the {self.datasheet.__name__} fields, needed "
                             f"or optional (missing {missing or 'none'}; not fields: {extra or 'none'})")


# --- the datasheet as vocabulary -------------------------------------------------------------------------------------


def si_unit(units: Mapping[str, float]) -> str:
    """The SI spelling of a datasheet field's unit family: "" when dimensionless, else the spelling with factor 1."""
    if units.get("") == 1.0:
        return ""
    for unit, factor in units.items():
        if factor == 1.0:
            return unit
    for unit in units:
        if unit in SI_SPELLING:
            return SI_SPELLING[unit]
    raise ValueError(f"no SI spelling among {list(units)}: add one to SI_SPELLING")


def datasheet_vocabulary(datasheet: type) -> tuple[Parameter, ...]:
    """A Datasheet class's typed fields as Parameters, in declaration order: rung M1's names, units and definitions
    taken from the class, so the datasheet and the ladder can never hold two spellings of one field."""
    shapes = {"number": "scalar", "integer": "integer", "text": "text"}
    return tuple(Parameter(name, si_unit(spec.units), spec.definition, shapes[spec.kind])
                 for name, spec in datasheet.typed_fields().items())


# --- finding every type's ladder -------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Found:
    ladder: Ladder
    module: str              # "robot_models.actuators.servo.ladder"
    folder: str              # the type folder, relative to the package: "actuators/servo" (a consumer's join key)


def discover(root: Path | None = None, package: str = "robot_models") -> dict[str, Found]:
    """Every type's ladder, found by walking the package for `ladder.py` files, {type: Found}.

    WHY a walk and not a registry (PEP 0004): each new type adds one folder and touches no shared file. The checks
    that span ladders run here: one ladder per type, and every composed (type, rung) exists.
    """
    root = Path(__file__).resolve().parent if root is None else Path(root)
    # WHY: the import system caches each folder's listing; a type folder made since (a lane's, a test's) would be
    # missed when its parent's mtime has not moved past the cache, which made discovery flaky within one second.
    importlib.invalidate_caches()
    found: dict[str, Found] = {}
    for path in sorted(root.rglob("ladder.py")):
        folder = path.parent.relative_to(root)
        if not folder.parts:  # WHY: this module is the package's own ladder.py, the core, not a type's
            continue
        if any(part.startswith((".", "__")) for part in folder.parts):
            continue
        module_name = ".".join((package, *folder.parts, "ladder"))
        module = importlib.import_module(module_name)
        ladder = getattr(module, "LADDER", None)
        if not isinstance(ladder, Ladder):
            raise TypeError(f"{module_name} defines no LADDER = Ladder(...); each type folder's ladder.py holds one")
        if ladder.type in found:
            raise ValueError(f"two ladders are type {ladder.type!r}: {found[ladder.type].module} and {module_name}")
        found[ladder.type] = Found(ladder, module_name, folder.as_posix())
    for item in found.values():
        for rung in item.ladder.rungs:
            for other, rung_key in rung.composes:
                if other not in found:
                    raise KeyError(f"{item.ladder.type} {rung.key} composes type {other!r}, which no ladder "
                                   f"is{_hint(other, found)}")
                found[other].ladder.rung(rung_key)
    return found


def ladders(root: Path | None = None, package: str = "robot_models") -> dict[str, Ladder]:
    """{type: Ladder} for every type folder."""
    return {name: item.ladder for name, item in discover(root, package).items()}


def needs(found: Mapping[str, Ladder], type_name: str, key: str) -> dict[str, tuple[str, ...]]:
    """{type: parameters} a rung needs, its own type first, then each type it composes (at the composed rung).

    WHY a function over the ladders (the breakage review): `parameters()` is one type's; a servo's driver delay sits
    in foc_controller's vocabulary, and the coverage consumer must not walk `composed()` by hand.
    """
    ladder = found[type_name]
    result = {type_name: ladder.parameters(key)}
    for other, rung_key in ladder.composed(key).items():
        result[other] = found[other].parameters(rung_key)
    return result


__all__ = [
    "DEPLOYMENT_ROLES",
    "KINDS",
    "LEARNED_LEVEL",
    "RESIDUAL_LEVEL",
    "TOP_PHYSICS_LEVEL",
    "UNIT_SUFFIXES",
    "WEAR_LAWS",
    "Alias",
    "DeploymentOnly",
    "Found",
    "Ladder",
    "Packed",
    "Parameter",
    "Reference",
    "Rung",
    "Wear",
    "datasheet_vocabulary",
    "discover",
    "ladders",
    "needs",
    "si_unit",
]
