"""v8 "Parameter sets": a ModelFamily holds Model classes; a Model class is STATELESS physics composed of Parameter
declarations; the values live apart, in a typed Parameters set generated per Model (PyBaMM's ParameterValues, kept
typed per model).

    class IntegratedServo(ModelFamily, type="integrated_servo", ...):
        class Stribeck(CoulombViscous, level=3, name="stribeck"):
            breakaway_torque = Parameter("N·m", "Static friction at the output ...")
            @staticmethod
            def torque(p, v): ...                                    # numpy, vectorized
    p = IntegratedServo.Stribeck.fit(log, start=m1)                 # -> Stribeck.Parameters: frozen, JSON, diffable
    IntegratedServo.Stribeck.predict(p, v) == p.predict(v)
"""

from __future__ import annotations

import dataclasses
import importlib.util
import inspect
import itertools
import json
from collections.abc import Iterator, Mapping
from functools import cache
from pathlib import Path
from typing import Any, ClassVar

import numpy as np

import robot_models
from robot_models.ladder import Ladder, Parameter as LadderParameter, Rung, datasheet_vocabulary

from .fitting import Fits, Log, Selection, _get, _indices, _least_squares, _moves, _seeds, _unpack, noise_floor
from .views import Tree, describe

PACKAGE = Path(robot_models.__file__).resolve().parent
# WHY 1.5 x the floor: sim2real's rule (cards/schema.py select()), so the rung chosen here is the one it would twin.
SELECT_LIMIT = 1.5
RUNG_PROSE = {"kind": "physics", "code": "", "equation": "", "references": (), "composes": (), "wears": ()}


# --- declarations: what a Model is composed of ---------------------------------------------------------------------


@dataclasses.dataclass(frozen=True)
class Parameter:
    """One named quantity of a type: its SI unit, what it means, and whether a model needs it or reads it if stated."""

    unit: str
    definition: str
    optional: bool = False       # read when stated; the model runs without it
    default: Any = None          # what the physics uses when an optional one is unstated
    shape: str = "scalar"        # scalar | integer | text | list (v1's shapes)
    positive: bool = False       # > 0 by definition: a fit searches its logarithm, so it cannot change sign
    name: str = ""               # set from the class body by __set_name__

    def __set_name__(self, owner: type, name: str) -> None:
        object.__setattr__(self, "name", name)

    @classmethod
    def from_datasheet(cls, datasheet: type, name: str) -> Parameter:
        """A datasheet field as a declaration: its unit and definition are the typed field's, never retyped."""
        field = next(f for f in datasheet_vocabulary(datasheet) if f.name == name)
        return cls(field.unit, field.definition, shape=field.shape, name=name)

    def when_stated(self) -> Parameter:
        """The same quantity, optional on this rung (one vocabulary: share the declaration, never retype it)."""
        return dataclasses.replace(self, optional=True)


@dataclasses.dataclass(frozen=True)
class Weights:
    """A black box's weights: counted and fitted like parameters, but no named quantity of the type's vocabulary."""

    count: int
    definition: str
    name: str = ""

    def __set_name__(self, owner: type, name: str) -> None:
        object.__setattr__(self, "name", name)


@dataclasses.dataclass(frozen=True)
class FitInfo:
    """How a value set came to be. Kept beside the values (compare=False): two fits of the same numbers are equal."""

    fitted: tuple[str, ...]      # what the log moved
    count: int                   # scalars fitted (a black box's weights each count)
    samples: int
    rms: float                   # on the log it was fitted to [N·m]; never chosen on
    start: str | None = None     # the key of the value set it started from


# --- Parameters: the values, as data ---------------------------------------------------------------------------------


class ParameterSet:
    """The base of every generated `<Model>.Parameters`: a frozen, keyword-only dataclass of one model's values in SI.

    WHY values apart from the model (PyBaMM ParameterValues, scikit-learn get_params): a value set is data you store,
    diff, serialize and hand to MuJoCo or sim2real; the model class stays pure physics that never holds a number."""

    model: ClassVar[type[Model]]
    info: FitInfo | None

    def __post_init__(self) -> None:
        for name, declared in self.model.declared.items():
            value = getattr(self, name)
            if isinstance(declared, Weights):
                value = tuple(float(w) for w in value)
                if len(value) != declared.count:
                    raise ValueError(f"{self.model.key}: {name} holds {declared.count} weights, not {len(value)}")
            elif value is not None and declared.shape in ("scalar", "integer"):
                value = int(value) if declared.shape == "integer" else float(value)
            object.__setattr__(self, name, value)

    def as_dict(self) -> dict[str, Any]:
        """What this set states, by canonical name (sklearn's get_params): what model.py, sim2real and JSON read."""
        return {name: getattr(self, name) for name in self.model.declared if getattr(self, name) is not None}

    def needed(self) -> dict[str, Any]:
        return {name: getattr(self, name) for name in self.model.needs()}

    def predict(self, v: Any, **inputs: Any) -> Any:
        """The light bound pair: these values through their own model's physics."""
        return self.model.predict(self, v, **inputs)

    def diff(self, other: ParameterSet) -> dict[str, tuple[Any, Any]]:
        """{name: (mine, theirs)} wherever two sets disagree, across models too (an M3 set against an M2 set)."""
        mine, theirs = self.as_dict(), other.as_dict()
        return {name: (mine.get(name), theirs.get(name)) for name in dict.fromkeys([*mine, *theirs])
                if mine.get(name) != theirs.get(name)}

    def to_json(self) -> str:
        units = {name: getattr(self.model.declared[name], "unit", "") for name in self.as_dict()}
        info = dataclasses.asdict(self.info) if self.info else None
        return json.dumps({"family": self.model.family.type_name, "model": self.model.key, "values": self.as_dict(),
                           "units": units, "info": info})

    def mujoco(self) -> dict[str, float]:
        """The joint attributes MuJoCo reads (armature, frictionloss, damping), as the rung's model.json maps them."""
        maps = _reference(self.model.code)[1]["backends"]["mujoco"].get("maps", {}) if self.model.code else {}
        return {attribute: getattr(self, name) for attribute, name in maps.items()
                if getattr(self, name, None) is not None}

    def __repr__(self) -> str:
        return f"{self.model.__name__}.Parameters({', '.join(f'{k}={v!r}' for k, v in self.as_dict().items())})"


# --- Model: a class of Parameter declarations and its physics, never a value -----------------------------------------


class Model:
    """Stateless physics. Subclass once per rung (`level=`, `name=`); a rung subclasses the rung it extends."""

    family: ClassVar[Any] = None
    declared: ClassVar[dict[str, Parameter | Weights]] = {}
    Parameters: ClassVar[type[ParameterSet]] = ParameterSet
    kind: ClassVar[str] = "physics"
    code: ClassVar[str] = ""                 # the folder of today's reference model.py / model.json
    equation: ClassVar[str] = ""
    references: ClassVar[tuple] = ()
    composes: ClassVar[tuple] = ()
    wears: ClassVar[tuple] = ()

    def __init_subclass__(cls, *, level: int, name: str, requires: tuple[str, ...] = (), states: type | None = None,
                          **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if len(cls.__bases__) != 1:
            raise TypeError(f"{cls.__name__}: a rung extends exactly one base")
        base = cls.__bases__[0]
        inherited = dict(base.declared)
        # WHY refuse at class creation (Zach: "we only ever want vocabulary to be additive"): a subclass that reassigns
        # an inherited name would remove or retype it, so the rung below would no longer be inside this one.
        for attribute in vars(cls):
            if attribute in inherited:
                raise TypeError(f"{cls.__name__}: {attribute} is {base.__name__}'s; vocabulary only grows. To need an "
                                f"inherited optional one here, pass requires=({attribute!r},)")
        own = {n: d for n, d in vars(cls).items() if isinstance(d, (Parameter, Weights))}
        if states is not None:  # WHY (PEP 0003 §6): a datasheet's typed fields ARE rung M1, read from its class
            for field in datasheet_vocabulary(states):
                if field.name not in inherited:
                    own[field.name] = Parameter(field.unit, field.definition, optional=True, shape=field.shape,
                                                name=field.name)
                    setattr(cls, field.name, own[field.name])
        promoted = {}
        for attribute in requires:
            if not getattr(inherited.get(attribute), "optional", False):
                raise TypeError(f"{cls.__name__}: requires={attribute!r} must name an optional parameter it inherits")
            promoted[attribute] = dataclasses.replace(inherited[attribute], optional=False)
        cls.declared = {**inherited, **promoted, **own}
        for attribute, default in RUNG_PROSE.items():  # WHY own, never inherited: M6 does not wear what M5 wears
            setattr(cls, attribute, vars(cls).get(attribute, default))
        cls.level, cls.name, cls.key = level, name, f"M{level}.{name}"
        cls.title = (cls.__doc__ or "").strip().splitlines()[0] if cls.__doc__ else ""
        cls.base = base if base is not Model else None
        cls.adds = (*promoted, *(n for n, d in own.items() if isinstance(d, Parameter) and not d.optional))
        cls.optional = tuple(n for n, d in own.items() if isinstance(d, Parameter) and d.optional)
        cls.param_count = sum(d.count if isinstance(d, Weights) else 1 for d in cls.declared.values()
                              if not getattr(d, "optional", False))
        cls.Parameters = _parameters_class(cls, base, {**promoted, **own})
        if "torque" in vars(cls) and cls.code:
            _check_parity(cls)
        if base.family is not None:  # WHY on definition: a rung is found by being declared, no registry to edit
            base.family._register(cls)

    # --- reading the class ---------------------------------------------------------------------------------------

    @classmethod
    def fallback(cls) -> tuple[str, ...]:
        """The rung and every rung below it, in order: the MRO, read as rung keys."""
        return tuple(klass.key for klass in cls.__mro__ if "key" in vars(klass))

    @classmethod
    def needs(cls) -> tuple[str, ...]:
        return tuple(n for n, d in cls.declared.items() if not getattr(d, "optional", False))

    @classmethod
    def runnable(cls) -> bool:
        return "torque" in vars(cls)  # WHY own physics only: M3.lugre must never run M3.stribeck's equation

    @classmethod
    def describe(cls) -> str:
        return describe(cls)

    # --- the physics: values in, torque out ---------------------------------------------------------------------

    @classmethod
    def predict(cls, params: ParameterSet, v: Any, **inputs: Any) -> Any:
        """The resisting torque tau_r [N·m] at output speed v [rad/s], vectorized over v."""
        if not cls.runnable():
            raise NotImplementedError(f"{cls.key} is declared, not runnable here: no physics attached")
        if not isinstance(params, cls.Parameters):
            raise TypeError(f"{cls.key}.predict takes {cls.__name__}.Parameters (or a richer rung's), not "
                            f"{type(params).__qualname__}")
        with np.errstate(all="ignore"):
            out = cls.torque(params, np.asarray(v, dtype=float), **inputs)
        return float(out) if np.ndim(out) == 0 else out

    @classmethod
    def from_datasheet(cls, datasheet: Any, column: str | None = None) -> ParameterSet:
        """The values a datasheet states for this model, in SI (a per-column value takes `column`)."""
        values = {}
        for name, spec in type(datasheet).typed_fields().items():
            stated = getattr(datasheet, name)
            if name in cls.declared and stated is not None:
                stated = stated.for_column(column) if stated.by_column else stated
                values[name] = stated.si if spec.kind == "number" else stated.value
        return cls.Parameters(**values)

    @classmethod
    def fit(cls, log: Log, start: ParameterSet | tuple[ParameterSet, ...] = ()) -> ParameterSet:
        """Least squares on the log over every needed parameter the log moves; the rest keep `start`'s values (several
        value sets merge, the last winning: the datasheet's, then the fitted rung below).

        WHY seeds from units, not a per-rung override: a speed [rad/s] is tried across the logged speeds, a torque
        [N·m] across the logged torques, a damping from zero, so a new rung fits with no fitting code of its own."""
        starts = start if isinstance(start, tuple) else (start,)
        known = {n: v for p in starts for n, v in p.as_dict().items() if n in cls.declared}
        missing = [n for n in cls.needs() if n not in known]
        best: tuple[float, dict] | None = None
        for combo in itertools.product(*(_seeds(cls.declared[n], log) for n in missing)):
            values = {**known, **dict(zip(missing, combo))}
            free = [(n, i) for n in cls.needs() for i in _indices(cls.declared[n]) if _moves(cls, values, n, log)]
            unseen = [n for n in missing if n not in {f for f, _ in free}]
            if unseen:
                raise ValueError(f"{cls.key}.fit: this log cannot see {', '.join(unseen)}; pass start= a value set "
                                 f"that states them (e.g. Datasheet.from_datasheet(...))")

            def residual(x: np.ndarray, values: dict = values, free: list = free) -> np.ndarray:
                return cls.predict(cls.Parameters(**_unpack(cls, values, free, x)), log["v"]) - log["tau"]

            x, cost = _least_squares(residual, np.array([_get(cls, values, f) for f in free], dtype=float))
            if best is None or cost < best[0]:
                best = (cost, _unpack(cls, values, free, x), tuple(dict.fromkeys(n for n, _ in free)), len(free))
        cost, values, fitted, count = best
        rms = float(np.sqrt(cost / len(log["v"])))
        info = FitInfo(fitted, count, len(log["v"]), rms, starts[-1].model.key if starts else None)
        return cls.Parameters(**values, info=info)


def _parameters_class(model: type[Model], base: type, new: dict[str, Parameter | Weights]) -> type[ParameterSet]:
    """`<Model>.Parameters`: a frozen dataclass of the model's values that SUBCLASSES its base rung's, so an M3 set is
    an M2 set too (the base's physics reads it) and the vocabulary grows along both trees the same way."""
    specs = []
    for name, d in new.items():
        metadata = {"declared": d, "unit": getattr(d, "unit", "")}
        if isinstance(d, Weights):
            specs.append((name, tuple[float, ...], dataclasses.field(metadata=metadata)))
        elif d.optional:
            specs.append((name, Any, dataclasses.field(default=d.default, metadata=metadata)))
        else:
            specs.append((name, str if d.shape == "text" else Any, dataclasses.field(metadata=metadata)))
    if base is Model:
        specs.append(("info", FitInfo | None, dataclasses.field(default=None, compare=False, repr=False)))
    bases = (base.Parameters,) if base is not Model else (ParameterSet,)
    klass = dataclasses.make_dataclass("Parameters", specs, bases=bases, frozen=True, kw_only=True, repr=False)
    klass.__qualname__, klass.__module__, klass.model = f"{model.__qualname__}.Parameters", model.__module__, model
    return klass


# --- the family: the type, its models, and the batch calls -----------------------------------------------------------


class _FamilyType(type):
    def __iter__(cls) -> Iterator[type[Model]]:
        return iter(cls.ladder())

    def __len__(cls) -> int:
        return len(cls._models)


class ModelFamily(metaclass=_FamilyType):
    """A type: its Model classes, declared in its body, and the calls that deal with all of them at once."""

    _models: ClassVar[list[type[Model]]]

    def __init_subclass__(cls, *, type: str, title: str, datasheet: type | None = None, aliases: tuple = (),
                          packed: tuple = (), deployment: tuple = (), **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        cls.type_name, cls.title, cls._models = type, title, []
        cls._v1 = dict(title=title, aliases=aliases, packed=packed, deployment=deployment, datasheet=datasheet)
        for value in list(vars(cls).values()):
            if inspect.isclass(value) and issubclass(value, Model) and value is not Model:
                value.family = cls
                cls._models.append(value)
        cls._derive()

    @classmethod
    def _register(cls, model: type[Model]) -> None:
        model.family = cls
        cls._models.append(model)
        try:
            cls._derive()
        except Exception:  # WHY: a rung v1's checks refuse must not stay half-added to the family
            cls._models.remove(model)
            cls._derive()
            raise

    @classmethod
    def _derive(cls) -> None:
        """Today's v1 Ladder, derived from the classes and validated by v1's own checks (one vocabulary, bases,
        nothing added twice, datasheet = M1). WHY derived: the classes are the one source; the Ladder is a view."""
        vocabulary: dict[str, Parameter] = {}
        for model in cls._models:
            for name, d in model.declared.items():
                if isinstance(d, Parameter):
                    seen = vocabulary.setdefault(name, d)
                    if (seen.unit, seen.definition) != (d.unit, d.definition):
                        raise TypeError(f"{cls.type_name}: {name} is declared twice, differently (one vocabulary "
                                        f"per type: share the declaration, use .when_stated())")
        rungs = [Rung(level=m.level, name=m.name, title=m.title, base=m.base and m.base.key, adds=m.adds,
                      kind=m.kind, optional=m.optional, code=m.code, wears=m.wears, composes=m.composes,
                      equation=m.equation, references=m.references) for m in cls.ladder()]
        cls._ladder = Ladder(type=cls.type_name, rungs=rungs, **cls._v1, vocabulary=tuple(
            LadderParameter(n, d.unit, d.definition, d.shape) for n, d in vocabulary.items()))
        cls.models = Models(cls.ladder())

    @classmethod
    def ladder(cls) -> tuple[type[Model], ...]:
        """The models in rung order: by level, then as declared."""
        return tuple(sorted(cls._models, key=lambda model: model.level))

    @classmethod
    def as_ladder(cls) -> Ladder:
        return cls._ladder

    @classmethod
    def drift(cls, today: Ladder) -> list[str]:
        """How the derived Ladder differs from `today` (keys, bases, adds, optional, kind, vocabulary, units): []."""
        mine, problems = cls._ladder, []
        if mine.keys != today.keys:
            problems.append(f"keys {mine.keys} != {today.keys}")
        for key in set(mine.keys) & set(today.keys):
            a, b = mine.rung(key), today.rung(key)
            problems += [f"{key}.{f}: {getattr(a, f)} != {getattr(b, f)}" for f in ("base", "adds", "optional", "kind")
                         if (set if f in ("adds", "optional") else str)(getattr(a, f)) !=
                         (set if f in ("adds", "optional") else str)(getattr(b, f))]
        if set(mine.names) != set(today.names):
            problems.append(f"vocabulary differs: {sorted(set(mine.names) ^ set(today.names))}")
        problems += [f"{n} unit {mine.parameter(n).unit!r} != {today.parameter(n).unit!r}"
                     for n in set(mine.names) & set(today.names) if mine.parameter(n).unit != today.parameter(n).unit]
        return problems

    @classmethod
    def canonical(cls, name: str) -> str:
        return cls._ladder.canonical(name)

    @classmethod
    def tree(cls) -> Tree:
        return Tree(cls)

    @classmethod
    def load(cls, text: str) -> ParameterSet:
        """A value set back from its JSON: the model key says which Parameters class reads it."""
        data = json.loads(text)
        info = FitInfo(**{**data["info"], "fitted": tuple(data["info"]["fitted"])}) if data["info"] else None
        return cls.models[data["model"]].Parameters(**data["values"], info=info)

    @classmethod
    def fit(cls, log: Log, start: ParameterSet | None = None) -> Fits:
        """Fit every model the log can drive, up the ladder: each starts from its base's fit (the climb), else
        `start`. Models with no physics attached are listed in .skipped with the reason, never dropped silently."""
        fits = Fits()
        for model in cls.ladder():
            if not model.runnable():
                fits.skipped[model.key] = "declared, no physics attached"
                continue
            base_fit = next((fits[k] for k in model.fallback()[1:] if k in fits), None)
            fits[model.key] = model.fit(log, start=tuple(p for p in (start, base_fit) if p is not None))
        return fits

    @classmethod
    def select_best(cls, fits: Mapping[str, ParameterSet], held_out: Log, limit: float = SELECT_LIMIT) -> Selection:
        """Score every fit on held-out plateaus against the rig's own noise floor; choose the lowest rung (level,
        then fewest parameters) within `limit` x the floor (amendment 1: a richer rung pays only off its fit data)."""
        order = sorted(fits.values(), key=lambda p: (p.model.level, p.model.param_count))
        rows = tuple((p, float(np.sqrt(np.mean((p.predict(held_out["v"]) - held_out["tau"]) ** 2)))) for p in order)
        return Selection(rows, noise_floor(held_out), limit)


class Models(Mapping):
    """The family's models by key (`models["M3.stribeck"]`) and by attribute (`models.m3.stribeck`)."""

    def __init__(self, models: tuple[type[Model], ...]) -> None:
        self._by_key = {model.key: model for model in models}

    def __getitem__(self, key: str) -> type[Model]:
        return self._by_key[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._by_key)

    def __len__(self) -> int:
        return len(self._by_key)

    def __getattr__(self, attribute: str) -> Any:
        if attribute[:1] == "m" and attribute[1:].isdigit():  # "m3": the grouping of a level's models
            return Models(tuple(m for m in self._by_key.values() if m.level == int(attribute[1:])))
        found = [m for m in self._by_key.values() if m.name == attribute]
        if len(found) != 1:
            raise AttributeError(f"no single model named {attribute!r} here; models: {', '.join(self._by_key)}")
        return found[0]

    def __repr__(self) -> str:
        return f"Models({', '.join(self._by_key)})"


@cache
def _reference(code: str) -> tuple[Any, dict]:
    """Today's stdlib physics and its model.json: actuators/rungs/<word>/, the contract's ruler."""
    folder = PACKAGE / code
    spec = importlib.util.spec_from_file_location(f"parameter_sets_reference_{folder.name}", folder / "model.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, json.loads((folder / "model.json").read_text())


def _check_parity(model: type[Model]) -> None:
    """WHY at class creation: the numpy physics is a second spelling of today's model.py (as model.js is), so it is
    held to it to 1e-12 on probe values the moment the class exists; a drifted rung cannot be defined."""
    numbers = [n for n, d in model.declared.items() if isinstance(d, Parameter) and d.shape == "scalar"]
    terms = {n: 0.05 * (i + 2) for i, n in enumerate(numbers)}
    speeds = np.array([-2.0, -0.3, 0.0, 0.05, 1.7])
    reference = np.array([_reference(model.code)[0].torque(terms, {"v": float(v)})[0] for v in speeds])
    mine = model.predict(model.Parameters(**terms), speeds)
    if not np.allclose(mine, reference, rtol=1e-12, atol=1e-12):
        raise TypeError(f"{model.key}: its numpy physics drifted from {model.code}/model.py ({mine} vs {reference})")
