"""v7 "Family per unit": three types. A ModelFamily CLASS is the type; a ModelFamily INSTANCE is one physical unit.

    class IntegratedServo(ModelFamily, type_name="integrated_servo", vocabulary=..., datasheet=...)
    class Stribeck(CoulombViscous, level=3, name="stribeck"):     # a rung subclasses its base: vocabulary only grows
        breakaway_torque = Parameter()                            # required; Parameter(optional=True) reads when stated
    IntegratedServo.models.m3.stribeck                            # the class: browse units, definitions, equation
    servo = IntegratedServo(datasheet=ds).fit(log)                # one unit: one instance of every Model
    servo.models.m3.stribeck.params; servo.select_best(held_out); servo.best

The classes are the one source: `IntegratedServo.as_ladder()` derives today's v1 `Ladder` from them. Runtime: numpy,
stdlib and robot_models only.
"""

from __future__ import annotations

import copy
import dataclasses
import difflib
import html
import importlib.util
import json
import tempfile
from collections.abc import Callable, Iterator, Mapping
from functools import cache, partial
from pathlib import Path
from typing import Any, Generic, TypeVar, overload

import numpy as np

import robot_models
from robot_models.ladder import Ladder, Rung

PACKAGE = Path(robot_models.__file__).resolve().parent
# WHY 1.5 x the floor: sim2real's selection rule (cards/schema.py TWINNED_LIMIT), so a unit picks what sim2real would.
SELECT_LIMIT = 1.5
SCALE_UNITS = {"rad/s", "s"}  # a speed or time scale: positive by its meaning, so fitted as its log
Log = Mapping[str, np.ndarray]  # a rig log: v [rad/s], tau [N·m], plateau (an id per held speed and direction)
T = TypeVar("T")
M = TypeVar("M", bound="Model")


class both:
    """A method that runs on the type (browse, no values) and on a unit or model instance (its values)."""

    def __init__(self, function: Callable) -> None:
        self.function = function

    def __get__(self, instance: Any, owner: type) -> Callable:
        return partial(self.function, owner if instance is None else instance)


@dataclasses.dataclass(frozen=True)
class Parameter(Generic[T]):
    """One quantity of a Model: unit and definition from the family's vocabulary, plus this unit's value if bound.

    On the class (`Stribeck.breakaway_torque`) it is this record; on a model instance it is the number."""

    optional: bool = False
    name: str = ""
    unit: str = ""
    definition: str = ""
    value: T | None = None

    def __set_name__(self, owner: type, name: str) -> None:
        object.__setattr__(self, "name", name)

    @overload
    def __get__(self, instance: None, owner: type) -> Parameter[T]: ...
    @overload
    def __get__(self, instance: Model, owner: type) -> T: ...
    def __get__(self, instance, owner):  # WHY a descriptor: type checkers see `servo[Stribeck].breakaway_torque: float`
        return self if instance is None else instance._values.get(self.name)

    def __repr__(self) -> str:
        value = "" if self.value is None else f" = {self.value:.6g}"
        return f"{self.name}{value} [{self.unit or '1'}] ({'optional' if self.optional else 'required'}): " \
               f"{self.definition}"


class ModelMeta(type):
    def __repr__(cls) -> str:
        return f"<model {cls.key} · class {cls.__name__}({cls.__bases__[0].__name__}) · {cls.param_count} params>"


class Model(metaclass=ModelMeta):
    """A rung: a class composed of Parameters. Subclass `Family.Model` for a root, a rung for one above it."""

    family: type[ModelFamily]
    key: str
    level: int
    kind: str = "physics"
    inputs: tuple[str, ...] = ("v",)     # what predict reads from a log besides the parameters
    code: str = ""                       # the package's stdlib reference physics, the ruler `parity` holds us to
    equation, note = "", ""
    references: tuple = ()
    wears: tuple = ()
    composes: tuple = ()
    _params: dict[str, Parameter] = {}

    def __init_subclass__(cls, *, level: int | None = None, name: str | None = None, kind: str = "physics",
                          fields_from: Any = None, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if level is None:  # the family's own root (Family.Model): no rung, no parameters
            return
        family, inherited = cls.family, dict(cls._params)
        declared = {n: p for n, p in vars(cls).items() if isinstance(p, Parameter)}
        if fields_from is not None:  # WHY (PEP 0003 §6): a datasheet's typed fields ARE M1, read, never retyped
            declared |= {n: Parameter(True, n) for n in fields_from.typed_fields() if n not in inherited}
        for n in inherited:  # WHY additive (Zach, round 2): a rung may promote optional -> required, never remove
            if n in vars(cls) and not isinstance(vars(cls)[n], Parameter):
                raise TypeError(f"M{level}.{name} sets inherited parameter {n} to {vars(cls)[n]!r}: a rung only adds")
        adds, optional = [], []
        for n, p in declared.items():
            family.canonical(n, strict=True)
            if n in inherited and (p.optional or not inherited[n].optional):
                raise TypeError(f"M{level}.{name} redeclares {n}, which its base already "
                                f"{'needs' if not inherited[n].optional else 'reads'}: a rung only adds")
            spec = family.parameter(n)
            filled = Parameter(p.optional, n, spec.unit, spec.definition)
            setattr(cls, n, filled)
            inherited[n] = filled
            (optional if p.optional else adds).append(n)
        if kind != "physics" and declared:
            raise TypeError(f"M{level}.{name}: a {kind} model's weights are not vocabulary parameters")
        cls.key, cls.level, cls.name, cls.kind, cls._params = f"M{level}.{name}", level, name, kind, inherited
        cls.adds, cls.optional = tuple(adds), tuple(optional)
        for prose in ("code", "equation", "note", "references", "wears", "composes"):  # WHY own only: a residual
            setattr(cls, prose, vars(cls).get(prose, type(getattr(Model, prose))()))   # does not wear as its base does
        cls.base = next((k.key for k in cls.__mro__[1:] if "key" in vars(k)), None)
        cls.title = (cls.__doc__ or "").strip().splitlines()[0] if cls.__doc__ else cls.key
        # WHY own torque only: LuGre inherits Stribeck's torque; running that as LuGre would be a silent lie
        cls.runnable = "torque" in vars(cls)
        cls.param_count = vars(cls).get("param_count", len(cls.needs()))
        if cls.key in family._classes:
            raise ValueError(f"{family.type_name} already has {cls.key}")
        family._classes[cls.key] = cls  # WHY on definition: a rung exists by being declared; no record to forget

    # --- the class: browse -----------------------------------------------------------------------------------------

    @classmethod
    def needs(cls) -> tuple[str, ...]:
        return tuple(n for n, p in cls._params.items() if not p.optional)

    @classmethod
    def reads(cls) -> tuple[str, ...]:
        return tuple(n for n, p in cls._params.items() if p.optional)

    @classmethod
    def fallback(cls) -> tuple[str, ...]:
        """This rung and every rung below it: the MRO, read as keys."""
        return tuple(k.key for k in cls.__mro__ if "key" in vars(k))

    @classmethod
    def as_rung(cls) -> Rung:
        return Rung(level=cls.level, name=cls.name, title=cls.title, base=cls.base, adds=cls.adds, kind=cls.kind,
                    optional=cls.optional, code=cls.code, wears=cls.wears, composes=cls.composes,
                    equation=cls.equation, references=cls.references, note=cls.note)

    @both
    def describe(self) -> str:
        """The model card: own parameters with unit and definition (and this unit's values on an instance)."""
        cls = self if isinstance(self, type) else type(self)
        lines = [f"{cls.key} · class {cls.__name__}({cls.__bases__[0].__name__}) · {cls.title}", f"  {cls.equation}"]
        lines += [f"  {'reads' if n in cls.optional else 'needs'} {self.parameter(n)!r}"
                  for n in (*cls.adds, *cls.optional[:3])]
        lines.append(f"  inherits {', '.join(n for n in cls.needs() if n not in cls.adds)}; reads "
                     f"{len(cls.reads()) - len(cls.optional[:3])} more when stated")
        lines.append("  cites " + "; ".join(r.title.split(")")[0] + ")" for r in cls.references))
        return "\n".join(lines + [f"  falls back {' -> '.join(cls.fallback()[1:]) or 'nowhere'}"])

    # --- an instance: this unit's values ---------------------------------------------------------------------------

    def __init__(self, unit: ModelFamily | None = None, **values: Any) -> None:
        self.unit, self.fitted, self.fit_rms, self.fitted_count = unit, False, None, 0
        self._values = {self.family.canonical(n): v for n, v in values.items() if v is not None}

    @both
    def parameter(self, name: str) -> Parameter:
        spec = (self if isinstance(self, type) else type(self))._params[self.family.canonical(name)]
        return spec if isinstance(self, type) else dataclasses.replace(spec, value=self._values.get(spec.name))

    @property
    def params(self) -> dict[str, Any]:
        """{canonical name: SI value} of what this rung needs: what it was fitted to, what MuJoCo and sim2real read."""
        return {n: self._values[n] for n in self.needs() if n in self._values}

    def missing(self) -> list[str]:
        return [n for n in self.needs() if n not in self._values]

    def torque(self, v: np.ndarray, **inputs: np.ndarray) -> np.ndarray:
        raise NotImplementedError(f"{self.key} is declared (parameters, fallback, prose): no physics in this prototype")

    def predict(self, v: Any, **inputs: Any) -> Any:
        """The resisting torque tau_r [N·m] at output speed v [rad/s]: numpy in, numpy out (a float for a float)."""
        if self.missing():
            raise ValueError(f"{self.key} has no {', '.join(self.missing())} yet: fit it, or state them")
        out = np.asarray(self.torque(np.asarray(v, float), **{k: np.asarray(x, float) for k, x in inputs.items()}))
        return float(out) if out.ndim == 0 else out

    def rms(self, log: Log) -> float:
        residual = self.predict(log["v"], **{k: log[k] for k in self.inputs if k != "v"}) - log["tau"]
        return float(np.sqrt(np.mean(residual ** 2)))

    def _with(self, values: dict) -> Model:
        clone = copy.copy(self)
        clone._values = {**self._values, **values}
        return clone

    def free(self, log: Log) -> list[str]:
        """The needs the log's torque responds to: inertia and ratio never move a velocity-plateau sweep. Tested at a
        jittered point too, so a seed that hides a term (breakaway = coulomb hides the speed scale) cannot fool it."""
        jitter = self._with({n: self._values[n] * (1.1 + 0.07 * i) for i, n in enumerate(self.needs())})
        return [n for n in self.needs() if any(
            not np.array_equal(at.predict(log["v"]), at._with({n: at._values[n] * 1.01 + 1e-6}).predict(log["v"]))
            for at in (self, jitter))]

    @both
    def fit(self, log: Log) -> Model:
        """Fit this model in place on `log` and return it. On the class: a fresh instance, fitted."""
        model = self() if isinstance(self, type) else self
        model._solve(log)
        model.fitted, model.fit_rms = True, model.rms(log)
        return model

    def _solve(self, log: Log) -> None:
        """Generic least squares. WHY seeds by unit, not by rung: a new rung needs no seeds() override (round 1)."""
        best = None
        for seed in _seeds(type(self), log, self._values):
            start = self._with(seed)
            free = start.free(log)
            unseen = [n for n in self.needs() if n not in self._values and n not in free]
            if unseen:
                raise ValueError(f"the log cannot see {', '.join(unseen)}: state it")
            if not free:
                best = (seed, 0.0)
                break

            # WHY a speed scale is fitted as its log: it is positive, and Stribeck's (|v|/vs)² cannot tell vs from -vs
            scale = np.array([type(self)._params[n].unit in SCALE_UNITS for n in free])

            def values(x: np.ndarray, free: list = free, scale: np.ndarray = scale) -> dict:
                return dict(zip(free, np.where(scale, np.exp(x), x).tolist()))

            def residual(x: np.ndarray, start: Model = start) -> np.ndarray:
                return start._with(values(x)).predict(log["v"]) - log["tau"]

            x0 = np.array([np.log(seed[n]) if s else seed[n] for n, s in zip(free, scale)], float)
            x, cost = _least_squares(residual, x0)
            if best is None or cost < best[1]:
                best = ({**seed, **values(x)}, cost)
        self._values, self.fitted_count = best[0], len(free)

    def parity(self, log: Log) -> float:
        """Max |this numpy torque - the package's stdlib model.py| over the log's speeds [N·m]: the one physics."""
        module = _reference(self.code)[0]
        ruler = [module.torque(self._values, {"v": float(v)})[0] for v in log["v"]]
        return float(np.max(np.abs(self.predict(log["v"]) - ruler)))

    def mujoco(self) -> dict[str, float]:
        """The joint attributes MuJoCo reads, by the rung's model.json maps: no glue on the sim2real side."""
        maps = _reference(self.code)[1]["backends"]["mujoco"].get("maps", {})
        return {attribute: self._values[n] for attribute, n in maps.items() if n in self._values}

    def __repr__(self) -> str:
        where = f" of {self.unit.name}" if self.unit is not None and self.unit.name else ""
        values = ", ".join(f"{n}={v:.4g}" for n, v in self.params.items())
        return f"{type(self).__name__}{where}({values}){'' if self.fitted else ' (not fitted)'}"


def _seeds(cls: type[Model], log: Log, known: dict) -> Iterator[dict]:
    """Starts for a fit. A torque starts at the log's median torque, a damping at 0; a speed scale gets one start per
    stretch of the logged speeds (the curve is not linear in it, so one start can settle in the wrong valley)."""
    tau, speeds = np.abs(log["tau"]), np.abs(log["v"][log["v"] != 0])
    by_unit = {"N·m": float(np.median(tau)), "N·m·s/rad": 0.0}
    start = {n: known.get(n, by_unit.get(cls._params[n].unit, 1.0)) for n in cls.needs()}
    grid = [n for n in cls.needs() if n not in known and cls._params[n].unit == "rad/s"]
    for speed in np.geomspace(speeds.min(), speeds.max(), 6) if grid else [None]:
        yield {**known, **start, **{n: float(speed) for n in grid}}


class Models(Mapping):
    """Models by key (`["M3.stribeck"]`), by class (`[Stribeck]`, typed), by attribute (`.m3.stribeck`); `.m3` is
    the group of alternatives at one level, itself a Models."""

    def __init__(self, entries: dict[str, Any]) -> None:
        self._entries = entries

    @overload
    def __getitem__(self, key: type[M]) -> M: ...
    @overload
    def __getitem__(self, key: str) -> Any: ...
    def __getitem__(self, key):
        return self._entries[key.key if isinstance(key, type) else key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._entries)

    def __len__(self) -> int:
        return len(self._entries)

    def __getattr__(self, name: str) -> Any:
        if name in self._entries:
            return self._entries[name]
        group = {k.split(".")[1]: m for k, m in self._entries.items() if k.lower().startswith(name + ".")}
        if not group:
            raise AttributeError(f"no model or level {name!r} in {', '.join(self)}")
        return Models(group)

    def __repr__(self) -> str:
        return f"Models({', '.join(self)})"


class FamilyMeta(type):
    """The type side: `for model in IntegratedServo`, `IntegratedServo.models`."""

    def __iter__(cls) -> Iterator[type[Model]]:
        return iter(cls.ladder())

    @property
    def models(cls) -> Models:  # a unit's own `models` (set in __init__) shadows this: instances, not classes
        return Models({k.key: k for k in cls.ladder()})


class ModelFamily(metaclass=FamilyMeta):
    """A type of part. The class holds its Model classes; an instance is one physical unit holding one of each."""

    type_name: str
    Model: type[Model]
    _classes: dict[str, type[Model]]

    def __init_subclass__(cls, *, type_name: str, vocabulary: tuple, aliases: tuple = (), packed: tuple = (),
                          deployment: tuple = (), datasheet: type | None = None, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        cls.type_name, cls.vocabulary, cls.aliases, cls.packed, cls.deployment = (
            type_name, vocabulary, aliases, packed, deployment)
        cls.datasheet, cls._classes = datasheet, {}
        cls._names = {p.name: p for p in vocabulary}
        cls._alias = {a.name: a.canonical for a in aliases}
        cls.Model = ModelMeta(f"{cls.__name__}Model", (Model,), {"family": cls, "__module__": cls.__module__})

    @classmethod
    def canonical(cls, name: str, strict: bool = False) -> str:
        """The type's own name for `name`; strict (a class body) refuses an alias, so one quantity has one spelling."""
        if name in cls._names:
            return name
        if name in cls._alias and not strict:
            return cls._alias[name]
        hint = f"; it is an alias of {cls._alias[name]!r}" if name in cls._alias else \
            "".join(f" (did you mean {m!r}?)" for m in difflib.get_close_matches(name, cls._names, n=1))
        raise KeyError(f"{name!r} is not a canonical {cls.type_name} parameter{hint}")

    @classmethod
    def parameter(cls, name: str):
        return cls._names[cls.canonical(name)]

    @classmethod
    def as_ladder(cls) -> Ladder:
        """Today's v1 Ladder, derived from the classes (Ladder's own checks run on it)."""
        return Ladder(type=cls.type_name, title=(cls.__doc__ or "").strip(), vocabulary=cls.vocabulary,
                      rungs=tuple(k.as_rung() for k in cls.ladder()), aliases=cls.aliases, packed=cls.packed,
                      deployment=cls.deployment, datasheet=cls.datasheet)

    @classmethod
    def ladder_diff(cls, today: Ladder) -> list[str]:
        """Where the derived ladder differs from `today` on keys, bases, adds and optional: [] is the same."""
        mine = cls.as_ladder()
        out = [] if mine.keys == today.keys else [f"keys {mine.keys} vs {today.keys}"]
        for a, b in zip(mine.rungs, today.rungs):
            out += [f"{a.key} {f}" for f in ("base", "adds", "optional") if getattr(a, f) != getattr(b, f)]
        return out

    # --- one unit ---------------------------------------------------------------------------------------------------

    def __init__(self, datasheet: Any = None, name: str = "") -> None:
        stated = {}
        for field, spec in (type(datasheet).typed_fields().items() if datasheet is not None else ()):
            value = getattr(datasheet, field)
            if value is not None:
                stated[field] = value.si if spec.kind == "number" else value.value
        self.name, self.datasheet, self.best, self.status = name, datasheet, None, {}
        self.models = Models({k.key: k(self, **{n: v for n, v in stated.items() if n in k._params})
                              for k in type(self).ladder()})

    def __iter__(self) -> Iterator[Model]:
        return iter(self.models.values())

    def __getitem__(self, key: type[M]) -> M:
        return self.models[key]

    @both
    def ladder(self) -> tuple:
        """The models, lowest level first (definition order within a level): classes on the type, instances on a unit."""
        if isinstance(self, type):
            return tuple(sorted(self._classes.values(), key=lambda k: k.level))
        return tuple(self.models.values())

    @both
    def tree(self) -> Tree:
        return Tree(self)

    @both
    def fit(self, log: Log, **unit: Any) -> ModelFamily:
        """Fit every model the log can identify, lowest first, each starting from the fitted rung below it (the
        climb); return the unit. On the type: a new unit, built from `**unit` (datasheet=, name=). Nothing is skipped
        silently: each model's `status` says what happened to it."""
        unit = self(**unit) if isinstance(self, type) else self
        for model in unit:
            below = unit.models[model.base] if model.base else None
            if below is not None and below.fitted:
                model._values |= {n: v for n, v in below._values.items() if n in model._params}
            lacks = [k for k in model.inputs if k not in log]
            if not model.runnable:
                unit.status[model.key] = "declared, no physics here"
            elif lacks:
                unit.status[model.key] = f"skipped: the log has no {', '.join(lacks)}"
            else:
                try:
                    model.fit(log)
                    unit.status[model.key] = (f"fitted {model.fitted_count} of {model.param_count}"
                                              if model.fitted_count else "stated: nothing the log moves")
                except ValueError as error:
                    unit.status[model.key] = f"skipped: {error}"
        return unit

    @both
    def select_best(self, held_out: Log, limit: float = SELECT_LIMIT) -> Selection:
        """The lowest fitted rung whose held-out rms / the rig's noise floor <= limit; it becomes `unit.best`."""
        if isinstance(self, type):
            raise TypeError(f"{self.__name__} is the type and has no values: select on a unit, "
                            f"{self.__name__}(datasheet=ds).fit(log).select_best(held_out)")
        rows = [(m, m.rms(held_out)) for m in self if m.fitted]
        selection = Selection(self.name, rows, noise_floor(held_out), limit)
        self.best = selection.chosen
        return selection

    def __repr__(self) -> str:
        lines = [f"{type(self).__name__} unit {self.name!r}: {sum(m.fitted for m in self)} of {len(self.models)} "
                 f"models fitted" + (f", best {self.best.key}" if self.best else "")]
        for m in self:
            rms = f"fit rms {1e3 * m.fit_rms:7.2f} mN·m" if m.fitted else ""
            lines.append(f"  {m.key:<22}{self.status.get(m.key, 'not fitted'):<32}{rms}")
        return "\n".join(lines)


def noise_floor(log: Log) -> float:
    """What the rig repeats to [N·m]: the pooled within-plateau standard deviation of tau, from the log itself."""
    plateaus = np.unique(log["plateau"])
    spread = np.concatenate([log["tau"][log["plateau"] == p] - log["tau"][log["plateau"] == p].mean()
                             for p in plateaus])
    return float(np.sqrt(spread @ spread / (spread.size - plateaus.size)))


@dataclasses.dataclass(frozen=True)
class Selection:
    """Fitted rungs scored on held-out plateaus against the floor. WHY held out (amendment 1): on its own fit data a
    rung never loses to one it nests, so only plateaus it never saw can say a richer rung pays."""

    unit: str
    rows: list            # (model, held-out rms [N·m]) in ladder order
    floor: float          # N·m
    limit: float

    @property
    def chosen(self) -> Model | None:
        return next((m for m, rms in self.rows if rms / self.floor <= self.limit), None)

    def __repr__(self) -> str:
        lines = [f"{self.unit} · held-out floor {1e3 * self.floor:.2f} mN·m · lowest rung within {self.limit}× floor",
                 f"  {'model':<20}{'params':>7}{'fit rms':>9}{'held out':>10}{'÷ floor':>9}   [mN·m]"]
        for m, rms in self.rows:
            mark = "  <- best" if m is self.chosen else ""
            lines.append(f"  {m.key:<20}{m.param_count:>7}{1e3 * m.fit_rms:>9.2f}{1e3 * rms:>10.2f}"
                         f"{rms / self.floor:>9.2f}{mark}")
        return "\n".join(lines + ([] if self.chosen else ["  no rung reaches the floor: climb, or measure more"]))


class Tree:
    """The models as their subclass tree. A ladder is one line up and down (a -> b: b subclasses a); a tree branches."""

    def __init__(self, owner: Any) -> None:
        self.owner, self.unit = owner, None if isinstance(owner, type) else owner
        self.nodes = owner.ladder()

    def _children(self, node: Any) -> list:
        key = None if node is None else node.key
        return [n for n in self.nodes if n.base == key]

    def _label(self, node: Any) -> str:
        return node.key + ("" if self.unit is None else "*" if node is self.unit.best else "" if node.fitted else "°")

    def __str__(self) -> str:
        family = self.owner if self.unit is None else type(self.owner)
        head = f"{family.type_name} · {len(self.nodes)} models" + \
            (f" · unit {self.unit.name!r} (* best, ° not fitted)" if self.unit else " (a -> b: b subclasses a)")
        lines = [head]

        def draw(node: Any, prefix: str, last: bool) -> None:
            chain, below = [self._label(node)], self._children(node)
            while len(below) == 1:
                chain.append(self._label(below[0]))
                below = self._children(below[0])
            lines.append(prefix + ("└── " if last else "├── ") + " -> ".join(chain))
            for i, child in enumerate(below):
                draw(child, prefix + ("    " if last else "│   "), i == len(below) - 1)

        roots = self._children(None)
        for i, root in enumerate(roots):
            draw(root, "", i == len(roots) - 1)
        return "\n".join(lines)

    def print(self) -> None:
        print(self)

    def show(self, path: str | Path | None = None) -> Path:
        """A self-contained HTML page: the tree with each model's own parameters (and this unit's values). Writes the
        file and returns its path; never opens a window (WHY: Zach's focus is not ours to take)."""
        path = Path(path or Path(tempfile.gettempdir()) / f"{str(self).split(' ')[0]}-tree.html")

        def card(node: Any) -> str:
            rows = "".join(f"<tr><td>{n}</td><td>{html.escape(p.unit or '1')}</td><td>{'' if p.optional else 'needs'}"
                           f"</td><td>{'' if self.unit is None or p.value is None else f'{p.value:.5g}'}</td>"
                           f"<td>{html.escape(p.definition)}</td></tr>"
                           for n in (*node.adds, *node.optional) for p in [node.parameter(n)])
            kids = "".join(f"<li>{card(c)}</li>" for c in self._children(node))
            return (f"<details open><summary><b>{self._label(node)}</b> {html.escape(node.title)} · "
                    f"{node.param_count} params</summary><code>{html.escape(node.equation)}</code>"
                    f"{f'<table>{rows}</table>' if rows else ''}{f'<ul>{kids}</ul>' if kids else ''}</details>")

        body = "".join(f"<li>{card(root)}</li>" for root in self._children(None))
        path.write_text(f"<!doctype html><meta charset=utf-8><title>{html.escape(str(self).splitlines()[0])}</title>"
                        "<style>body{font:14px system-ui;margin:2em}ul{list-style:none;border-left:1px solid #ccc;"
                        "padding-left:1.2em}td{padding:0 .6em;border-bottom:1px solid #eee;font-size:12px}"
                        f"code{{color:#555;font-size:12px}}</style><h3>{html.escape(str(self).splitlines()[0])}</h3>"
                        f"<ul>{body}</ul>")
        return path


@cache
def _reference(code: str) -> tuple[Any, dict]:
    """The package's reference physics and definition for a rung: actuators/rungs/<word>/model.py and model.json."""
    if not code:
        raise NotImplementedError("this model has no reference physics in the package")
    folder = PACKAGE / code
    spec = importlib.util.spec_from_file_location(f"v7_reference_{folder.name}", folder / "model.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, json.loads((folder / "model.json").read_text())


def _least_squares(residual: Callable[[np.ndarray], np.ndarray], x: np.ndarray,
                   iterations: int = 100) -> tuple[np.ndarray, float]:
    """Levenberg-Marquardt with a forward-difference Jacobian: numpy only (the runtime has no scipy)."""
    r = residual(x)

    def safe(point: np.ndarray) -> np.ndarray:
        with np.errstate(all="ignore"):
            out = residual(point)
        return out if np.all(np.isfinite(out)) else np.full_like(r, np.inf)

    cost, damping = float(r @ r), 1e-3
    for _ in range(iterations):
        steps = 1e-6 * np.maximum(np.abs(x), 1e-3)
        jacobian = np.column_stack([(safe(x + np.eye(x.size)[i] * steps[i]) - r) / steps[i] for i in range(x.size)])
        normal, gradient = jacobian.T @ jacobian, jacobian.T @ r
        while damping < 1e12:
            step = np.linalg.solve(normal + damping * np.diag(np.diag(normal) + 1e-12), -gradient)
            trial_r = safe(x + step)
            trial = float(trial_r @ trial_r)
            if np.isfinite(trial) and trial < cost:
                break
            damping *= 10
        else:
            return x, cost
        done = cost - trial < 1e-12 * cost
        x, r, cost, damping = x + step, trial_r, trial, max(damping / 10, 1e-9)
        if done:
            break
    return x, cost
