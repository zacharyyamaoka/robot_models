"""v1 "Subclass ladder": a Model is a frozen dataclass whose fields ARE its parameters, and a rung subclasses its base.

    class Stribeck(CoulombViscous, level=3, name="stribeck"):   # see servo.py: the type is the root class
        breakaway_torque: float                                  # no default: the rung needs it
        stribeck_exponent: float | None = None                   # a default: read when stated
    Stribeck.fallback()   Stribeck(output_inertia=..., ...)   Stribeck.fit(log, start=m2)   m3.torque(v)

Units, definitions, titles, equations and references are not retyped: field metadata and class prose are read from
v1's records (robot_models.ladder) by name and key, and each class is checked against its v1 rung record.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import inspect
import json
import math
from collections.abc import Callable, Iterable, Iterator, Mapping
from functools import cache
from pathlib import Path
from typing import Any, ClassVar, Self, dataclass_transform

import numpy as np

import robot_models
from robot_models.ladder import Ladder, Parameter, Reference, ladders

PACKAGE = Path(robot_models.__file__).resolve().parent
# WHY the contract's stop line: M0-M3 run; M4-M7 are declared (fields, fallback, prose) and refuse to run.
TOP_RUNNABLE_LEVEL = 3
KIND_ANNOTATION = {"number": "float | None", "integer": "int | None", "text": "str | None"}
Log = Mapping[str, np.ndarray]  # a rig log: {"v": output speed [rad/s], "tau": torque the motor added [N·m]}


def _ordered(names: Iterable[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(names))


# WHY dataclass_transform (PEP 681, the pydantic / SQLModel pattern): every subclass becomes a frozen, keyword-only
# dataclass without a decorator, and type checkers see each rung's parameters as typed constructor arguments.
@dataclass_transform(frozen_default=True, kw_only_default=True)
class Model:
    """The base object. Subclass it once per type (`type=`), then once per rung (`level=`, `name=`)."""

    __dataclass_fields__: ClassVar[dict[str, dataclasses.Field[Any]]]  # every subclass is a dataclass
    type_name: ClassVar[str]                        # set on the type's root class
    v1: ClassVar[Ladder]                            # v1's records: the vocabulary, aliases and each rung's prose
    key: ClassVar[str]                              # set on each rung class: "M3.stribeck"
    level: ClassVar[int]
    title: ClassVar[str]
    equation: ClassVar[str]
    references: ClassVar[tuple[Reference, ...]]
    runnable: ClassVar[bool]

    def __init_subclass__(cls, *, type: str | None = None, level: int | None = None, name: str | None = None,
                          fields_from: Any = None, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if type is not None:
            cls.type_name, cls.v1, cls._rungs = type, ladders()[type], []
        elif level is None or name is None:
            raise TypeError(f"{cls.__name__}: a model class names its rung, e.g. "
                            f"class {cls.__name__}(CoulombViscous, level=3, name='stribeck')")
        else:
            cls._declare_rung(level, name, fields_from)
        dataclasses.dataclass(frozen=True, kw_only=True, repr=False)(cls)

    @classmethod
    def _declare_rung(cls, level: int, name: str, fields_from: Any) -> None:
        key = f"M{level}.{name}"
        record = cls.v1.rung(key)
        own = dict(inspect.get_annotations(cls))
        if fields_from is not None:
            # WHY (PEP 0003 §6): a datasheet's typed fields ARE rung M1, so they are read from its class, never typed twice
            inherited = {field.name for field in dataclasses.fields(cls)}
            for field_name, spec in fields_from.typed_fields().items():
                if field_name not in inherited and field_name not in own:
                    own[field_name] = KIND_ANNOTATION[spec.kind]
                    setattr(cls, field_name, None)
        adds, optional = [], []
        for field_name in own:
            if field_name not in cls.v1.names:  # one spelling per quantity: an alias is no field name
                raise KeyError(f"{key}: {field_name!r} is not a canonical parameter of {cls.type_name}")
            default = cls.__dict__.get(field_name, dataclasses.MISSING)
            (adds if default is dataclasses.MISSING else optional).append(field_name)
            setattr(cls, field_name, dataclasses.field(default=default,
                                                       metadata={"parameter": cls.v1.parameter(field_name)}))
        cls.__annotations__ = own
        base = next((klass.key for klass in cls.__mro__[1:] if "key" in vars(klass)), None)
        # WHY check against v1's record (the migration guard): while both exist they cannot drift apart
        if (base, set(adds), set(optional)) != (record.base, set(record.adds), set(record.optional)):
            raise TypeError(f"{cls.__name__} ({key}) is not v1's record: base {base} vs {record.base}, needs "
                            f"{sorted(adds)} vs {sorted(record.adds)}, reads {sorted(optional)} vs "
                            f"{sorted(record.optional)}")
        cls.key, cls.level, cls.word, cls.kind, cls.code = key, level, name, record.kind, record.code
        cls.title, cls.equation, cls.references = record.title, record.equation, record.references
        cls.runnable = record.kind == "physics" and bool(record.code) and level <= TOP_RUNNABLE_LEVEL
        cls._adds, cls._optional = tuple(adds), tuple(optional)
        cls._rungs.append(cls)  # WHY on definition: a new rung is found by being declared, no registry to edit

    # --- the type: its ladder is its subclass tree ------------------------------------------------------------

    @classmethod
    def rungs(cls) -> tuple[type[Model], ...]:
        """The type's rung classes: by level, then in the order they were declared."""
        return tuple(sorted(cls._rungs, key=lambda rung: rung.level))

    @classmethod
    def ladder(cls) -> tuple[str, ...]:
        return tuple(rung.key for rung in cls.rungs())

    @classmethod
    def rung(cls, key: str) -> type[Model]:
        return next(rung for rung in cls.rungs() if rung.key == key)

    @classmethod
    def canonical(cls, name: str) -> str:
        """The type's own name for `name` (one vocabulary per type): an alias resolves to its canonical."""
        return cls.v1.canonical(name)

    @classmethod
    def tree(cls) -> str:
        """The subclass tree. A run of single children prints on one line (a -> b: b subclasses a)."""
        root = next(klass for klass in cls.__mro__ if "type_name" in vars(klass))
        lines = [f"{root.type_name} · class {root.__name__} · {len(root._rungs)} models (a -> b: b subclasses a)"]

        def children(node: type) -> list[type[Model]]:
            return [rung for rung in root.rungs() if rung.__bases__[0] is node]

        def draw(node: type[Model], prefix: str, last: bool) -> None:
            chain, below = [node.key], children(node)
            while len(below) == 1:
                chain.append(below[0].key)
                below = children(below[0])
            lines.append(prefix + ("└── " if last else "├── ") + " -> ".join(chain))
            for index, child in enumerate(below):
                draw(child, prefix + ("    " if last else "│   "), index == len(below) - 1)

        top = children(root)
        for index, child in enumerate(top):
            draw(child, "", index == len(top) - 1)
        return "\n".join(lines)

    # --- one rung: its parameters are its fields, its fallback its MRO ----------------------------------------

    @classmethod
    def fallback(cls) -> tuple[str, ...]:
        """The rung and every rung below it, in order: the MRO, read as rung keys."""
        return tuple(klass.key for klass in cls.__mro__ if "key" in vars(klass))

    @classmethod
    def needs(cls) -> tuple[str, ...]:
        """Every parameter the rung cannot run without (fields with no default): the root's first, its own last."""
        return _ordered(name for klass in reversed(cls.__mro__) for name in vars(klass).get("_adds", ()))

    @classmethod
    def reads(cls) -> tuple[str, ...]:
        """What it reads when stated (fields that default to None), less what it needs."""
        needed = set(cls.needs())
        return _ordered(name for klass in reversed(cls.__mro__) for name in vars(klass).get("_optional", ())
                        if name not in needed)

    @classmethod
    def parameter(cls, name: str) -> Parameter:
        """A field's unit and definition, from its dataclass metadata."""
        fields = {field.name: field for field in dataclasses.fields(cls)}
        return fields[cls.canonical(name)].metadata["parameter"]

    @classmethod
    def alternatives(cls) -> tuple[str, ...]:
        """The other rungs at this level: what else could stand here."""
        return tuple(rung.key for rung in cls.rungs() if rung.level == cls.level and rung is not cls)

    @classmethod
    def describe(cls) -> str:
        own_needs, own_reads = vars(cls)["_adds"], vars(cls)["_optional"]

        def unit(name: str) -> str:
            return cls.parameter(name).unit or "1"

        lines = [f"{cls.key} · class {cls.__name__}({cls.__bases__[0].__name__}) · {cls.title}",
                 f"  {cls.equation}"]
        lines += [f"  needs {name} [{unit(name)}]: {cls.parameter(name).definition}" for name in own_needs]
        if len(own_reads) <= 3:
            lines += [f"  reads {name} [{unit(name)}]: {cls.parameter(name).definition}" for name in own_reads]
        else:
            lines.append(f"  reads {len(own_reads)} when stated: {', '.join(own_reads[:4])}, ...")
        inherited = [name for name in cls.needs() if name not in own_needs]
        more = len([name for name in cls.reads() if name not in own_reads])
        lines.append("  inherits " + ", ".join(f"{name} [{unit(name)}]" for name in inherited)
                     + f"; reads {more} more when stated")
        lines.append("  cites " + "; ".join(reference.title.split(")")[0] + ")" for reference in cls.references))
        runs = f"{cls.code}/model.py" if cls.runnable else "declared, not runnable here"
        lines.append(f"  runs {runs} · also at M{cls.level}: {', '.join(cls.alternatives()) or 'none'}")
        return "\n".join(lines)

    # --- values: an instance is a model with its numbers ------------------------------------------------------

    @classmethod
    def from_datasheet(cls, datasheet: Any, column: str | None = None) -> Self:
        """The model with every field the datasheet states, in SI (a per-column value takes `column`)."""
        names, values = {field.name for field in dataclasses.fields(cls)}, {}
        for name, spec in type(datasheet).typed_fields().items():
            stated = getattr(datasheet, name)
            if name in names and stated is not None:
                stated = stated.for_column(column) if stated.by_column else stated
                values[name] = stated.si if spec.kind == "number" else stated.value
        return cls(**values)

    def terms(self, names: Iterable[str] | None = None) -> dict[str, Any]:
        """{canonical name: SI value} of what this model states: what model.py, sim2real and scipy read."""
        stated = {field.name: getattr(self, field.name) for field in dataclasses.fields(self)}
        stated = {name: value for name, value in stated.items() if value is not None}
        return stated if names is None else {name: stated[name] for name in names}

    def __repr__(self) -> str:
        return f"{type(self).__name__}({', '.join(f'{name}={value!r}' for name, value in self.terms().items())})"

    @classmethod
    def _reference(cls) -> tuple[Any, dict]:
        if not cls.runnable:
            why = "the prototype runs M0-M3 only" if cls.code else "v1 ships no reference physics for it"
            raise NotImplementedError(f"{cls.key} is declared, not runnable here: {why}")
        return _load(cls.code)

    def torque(self, v: Any, **inputs: Any) -> Any:
        """The resisting torque tau_r [N·m] at output speed v [rad/s] (plus the rung's other inputs, e.g. tau_load)."""
        module, definition = type(self)._reference()
        terms = self.terms()
        for term in definition["terms"]:  # the reference's own term names (LuGre's sigma2) resolve through aliases
            if term not in terms and term not in self.v1.names and self.canonical(term) in terms:
                terms[term] = terms[self.canonical(term)]
        names = ("v", *inputs)
        columns = np.broadcast_arrays(*(np.asarray(value, dtype=float) for value in (v, *inputs.values())))
        rows = zip(*(column.ravel() for column in columns))
        # WHY a loop over the stdlib reference, not a numpy rewrite: the contract keeps one copy of the physics
        out = np.fromiter((module.torque(terms, dict(zip(names, map(float, row))))[0] for row in rows), float,
                          count=columns[0].size).reshape(columns[0].shape)
        return float(out) if out.ndim == 0 else out

    def mujoco(self) -> dict[str, float]:
        """The joint attributes MuJoCo reads (armature, frictionloss, damping), as the rung's model.json maps them."""
        maps = type(self)._reference()[1]["backends"]["mujoco"].get("maps", {})
        return {attribute: getattr(self, name) for attribute, name in maps.items() if getattr(self, name) is not None}

    # --- fitting and comparing --------------------------------------------------------------------------------

    @classmethod
    def seeds(cls, log: Log, start: dict[str, Any]) -> Iterator[dict[str, Any]]:
        """Where a fit starts. A rung that adds a term with no linear seed overrides this (nn.Module's
        reset_parameters, in spirit)."""
        yield dict(start)

    @classmethod
    def fit(cls, log: Log, start: Model | None = None) -> Self:
        """Least squares on the rig log over every parameter the log moves; the rest keep `start`'s values.

        WHY a start (the climb): an M3 starts from the fitted M2 below it, and the datasheet's inertia and ratio, which
        a friction log cannot see, ride along unchanged."""
        names = {field.name for field in dataclasses.fields(cls)}
        start_values = {} if start is None else {n: value for n, value in start.terms().items() if n in names}
        best: tuple[dict[str, Any], float] | None = None
        for seed in cls.seeds(log, start_values):
            missing = [name for name in cls.needs() if seed.get(name) is None]
            if missing:
                raise ValueError(f"{cls.key}.fit has no start for {', '.join(missing)}: pass start= a model that "
                                 f"states them (Datasheet.from_datasheet(...) or the rung below, fitted)")
            free = [name for name in cls.needs() if cls._moves(seed, name, log)]

            def residual(x: np.ndarray, seed: dict = seed, free: list = free) -> np.ndarray:
                return cls(**{**seed, **dict(zip(free, x))}).torque(log["v"]) - log["tau"]

            solution, cost = _least_squares(residual, np.array([seed[name] for name in free], dtype=float))
            if best is None or cost < best[1]:
                best = ({**seed, **dict(zip(free, solution.tolist()))}, cost)
        if best is None:
            raise ValueError(f"{cls.key}.seeds gave no start")
        return cls(**best[0])

    @classmethod
    def _moves(cls, values: dict[str, Any], name: str, log: Log) -> bool:
        """Does the log's torque respond to `name`? Inertia and gear ratio never move a velocity-plateau sweep."""
        bumped = {**values, name: values[name] * 1.01 + 1e-6}
        return not np.array_equal(cls(**values).torque(log["v"]), cls(**bumped).torque(log["v"]))

    def score(self, log: Log) -> Score:
        residual = self.torque(log["v"]) - log["tau"]
        n, rss = residual.size, float(residual @ residual)
        fitted = sum(type(self)._moves(self.terms(), name, log) for name in self.needs())
        return Score(self.key, fitted, math.sqrt(rss / n), n * math.log(rss / n) + fitted * math.log(n))

    @classmethod
    def compare(cls, models: Iterable[Model], log: Log) -> Comparison:
        models = tuple(models)
        return Comparison(models, tuple(model.score(log) for model in models))


@dataclasses.dataclass(frozen=True)
class Score:
    key: str
    fitted: int              # parameters the log moves (the BIC's k)
    rms: float               # N·m
    bic: float


@dataclasses.dataclass(frozen=True)
class Comparison:
    """Rungs scored on one log. WHY BIC (Schwarz 1978): a rung pays only when its drop in residual beats ln(n) for
    each parameter it fits, so a richer rung has to earn its place."""

    models: tuple[Model, ...]
    scores: tuple[Score, ...]

    @property
    def chosen(self) -> Model:
        return min(zip(self.scores, self.models), key=lambda pair: pair[0].bic)[1]

    def __str__(self) -> str:
        lines = [f"{'rung':<20}{'fitted':>7}{'rms [N·m]':>11}{'BIC':>10}"]
        for score in self.scores:
            mark = "  <- lowest BIC" if score.key == self.chosen.key else ""
            lines.append(f"{score.key:<20}{score.fitted:>7}{score.rms:>11.4f}{score.bic:>10.1f}{mark}")
        return "\n".join(lines)


@cache
def _load(code: str) -> tuple[Any, dict]:
    """Today's runnable physics, by path: actuators/rungs/<word>/model.py and the model.json beside it."""
    folder = PACKAGE / code
    spec = importlib.util.spec_from_file_location(f"subclass_ladder_reference_{folder.name}", folder / "model.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, json.loads((folder / "model.json").read_text())


def _least_squares(residual: Callable[[np.ndarray], np.ndarray], x: np.ndarray,
                   iterations: int = 100) -> tuple[np.ndarray, float]:
    """Levenberg-Marquardt with a forward-difference Jacobian: numpy only (the runtime has no scipy)."""
    r = residual(x)  # the start lies on the model's domain; a trial step may not

    def safe(point: np.ndarray) -> np.ndarray:
        try:
            return residual(point)
        except (ArithmeticError, ValueError, TypeError):  # a step off the model's domain (a zero speed scale)
            return np.full_like(r, np.inf)

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
