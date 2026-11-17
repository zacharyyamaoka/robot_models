"""ModelFamily -> Model -> Parameter: a part type's models as classes; values live on Model INSTANCES.

Round 2's pick, v6 "Fitted instances" (docs/proposals/api-2026-11-17/), named as PEP 0004 amendment 2 records:

    class IntegratedServo(ModelFamily, id="integrated_servo"):           # a part type's models: one family
        class Stribeck(CoulombViscous, level=3, name="stribeck"):        # a rung subclasses its base
            breakaway_torque = Parameter("N·m", "Static friction ...")   # new: unit + definition, required
            coulomb_friction = Parameter()                               # inherited: only ever tightened
    IntegratedServo.m3.stribeck                                          # a Model class; .m3 is a ModelFamily too
    m3 = IntegratedServo.m3.stribeck.fit(log, start=m1)                  # a fit returns an instance
    IntegratedServo.fit(log, start=m1).select_best(held_out)             # batch: any family, a level's too

The classes are the one source of the type's models: `IntegratedServo.v1_ladder()` builds today's `Ladder` record from
them, equal to `ladders()["integrated_servo"]`. numpy + stdlib only; importing `robot_models.ladder` or a datasheet
never imports this module (robot_models/__init__.py loads the types lazily).
"""

from __future__ import annotations

import builtins
import dataclasses
import html
import itertools
import json
import re
import tempfile
import types
from collections.abc import Callable, Iterable, Iterator, Mapping
from functools import cache
from pathlib import Path
from types import SimpleNamespace
from typing import Any, ClassVar, overload

import numpy as np

import robot_models
from robot_models import ladder as v1

PACKAGE = Path(robot_models.__file__).resolve().parent
RUNG_CODE = "actuators/rungs"
# WHY 1.5 x the floor: sim2real's select() (cards/schema.py TWINNED_LIMIT), so both repos choose the same rung.
SELECT_LIMIT = 1.5
Log = Mapping[str, np.ndarray]  # a rig log: v [rad/s], tau [N·m], plateau (an id per held speed)
# WHY reserved: parameters share the class namespace with the API (V1's "one namespace" flaw), so a clash raises.
API_NAMES = frozenset({"key", "level", "word", "kind", "title", "equation", "references", "note", "wears", "composes",
                       "code", "family", "parameters", "own", "param_count", "fitted_count", "fallback", "needs",
                       "describe", "rung", "runnable", "inputs", "from_datasheet", "fit", "predict", "score",
                       "values", "params", "mujoco", "weights", "degree"})
LEVEL = re.compile(r"[mM](\d+)")


@dataclasses.dataclass(frozen=True)
class Parameter:
    """One quantity: unit, definition and info on the class; its value on an instance (`m3.params[name].value`).
    `Parameter()` (no unit) names a parameter the base already holds and makes it required; it never redefines one."""

    unit: str | None = None
    definition: str = ""
    shape: str = "scalar"
    note: str = ""
    required: bool = True
    value: Any = None
    name: str = ""

    def __set_name__(self, owner: type, name: str) -> None:
        object.__setattr__(self, "name", name)

    @overload
    def __get__(self, model: None, owner: type) -> Parameter: ...
    @overload
    def __get__(self, model: Model, owner: type) -> float: ...
    def __get__(self, model: Model | None, owner: type) -> Any:
        return self if model is None else model._values.get(self.name)  # class: the Parameter; instance: its value

    def optional(self) -> Parameter:
        return dataclasses.replace(self, required=False)

    def record(self) -> v1.Parameter:
        return v1.Parameter(self.name, self.unit or "", self.definition, self.shape, self.note)

    def __repr__(self) -> str:
        value = "" if self.value is None else f" = {self.value!r}"
        return (f"Parameter({self.name}{value} [{self.unit or '1'}], {'required' if self.required else 'optional'}: "
                f"{self.definition})")


class _ModelType(type):
    def __repr__(cls) -> str:
        key = vars(cls).get("key")
        return f"<Model {key} · class {cls.__name__} · {cls.param_count} params>" if key else super().__repr__()


class Model(metaclass=_ModelType):
    """A model class: its Parameters are class attributes. An instance holds one value per parameter.

    WHY analytical and black box look the same (Zach, round 2): every Model, physics or learned, answers `fit`,
    `predict`, `score` and `param_count`, so `select_best` compares them with one call."""

    key: ClassVar[str]
    level: ClassVar[int]
    word: ClassVar[str]
    title: ClassVar[str]
    code: ClassVar[str]
    kind: ClassVar[str] = "physics"
    family: ClassVar[Any] = None                 # the type's ModelFamily, once the rung joins it
    _datasheet: ClassVar[type | None] = None     # the datasheet class whose fields this rung reads (M1)
    parameters: ClassVar[dict[str, Parameter]] = {}
    own: ClassVar[tuple[str, ...]] = ()          # what this rung adds or tightens, in declaration order
    param_count: ClassVar[int] = 0
    fitted_count: int = 0                        # on a fitted instance: how many numbers the log chose

    def __init_subclass__(cls, *, level: int | None = None, name: str | None = None, datasheet: type | None = None,
                          **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if level is None or name is None:  # an abstract helper (BlackBox): no rung of its own
            return
        cls.level, cls.word, cls.key = level, name, f"M{level}.{name}"
        inherited, own = dict(cls.parameters), {}
        if datasheet is not None:  # WHY (PEP 0003 §6): a datasheet's typed fields ARE rung M1, read from its class
            cls._datasheet = datasheet
            for record in v1.datasheet_vocabulary(datasheet):
                if record.name not in inherited:
                    own[record.name] = Parameter(record.unit, record.definition, record.shape, record.note, False,
                                                 name=record.name)
                    setattr(cls, record.name, own[record.name])
        for attr, value in list(vars(cls).items()):
            if attr in inherited and not isinstance(value, Parameter):
                raise TypeError(f"{cls.key}: `{attr} = {value!r}` removes an inherited parameter; vocabulary only grows")
            if isinstance(value, Parameter) and attr not in own:
                if attr in API_NAMES:
                    raise TypeError(f"{cls.key}: a parameter cannot be called {attr!r}, the Model API uses that name")
                own[attr] = cls._additive(attr, value, inherited)
                setattr(cls, attr, own[attr])
        cls.parameters, cls.own = {**inherited, **own}, tuple(own)
        cls.title = (vars(cls).get("__doc__") or "").strip()
        cls.code = f"{RUNG_CODE}/{name}" if (PACKAGE / RUNG_CODE / name).is_dir() else ""
        cls.param_count = cls._count()
        if cls.family is not None:  # WHY: a rung declared after its family joins it; nothing drops silently
            cls.family._add(cls)

    @classmethod
    def _additive(cls, name: str, new: Parameter, inherited: dict[str, Parameter]) -> Parameter:
        """WHY (Zach, round 2: "we only ever want vocabulary to be additive"): add, or tighten; never redefine."""
        old = inherited.get(name)
        if old is None:
            if new.unit is None:
                raise TypeError(f"{cls.key}: {name} = Parameter() names no inherited parameter; give unit and definition")
            same = [p.name for p in inherited.values() if p.definition == new.definition]
            if same:
                raise TypeError(f"{cls.key}: {name} renames inherited {same[0]} (same definition); keep the old name")
            return dataclasses.replace(new, name=name)
        if new.unit is not None and (new.unit, new.definition or old.definition) != (old.unit, old.definition):
            raise TypeError(f"{cls.key}: {name} redefines inherited [{old.unit}] {old.definition!r}; vocabulary only "
                            f"grows: write `{name} = Parameter()` to require it")
        if old.required or not new.required:
            raise TypeError(f"{cls.key}: {name} is already held below; a rung can only make an optional one required")
        return dataclasses.replace(old, required=True)

    # --- the class: what the model is ---------------------------------------------------------------------------

    @classmethod
    def _count(cls) -> int:
        return len(cls.needs())

    @classmethod
    def needs(cls) -> tuple[str, ...]:
        return tuple(name for name, parameter in cls.parameters.items() if parameter.required)

    @classmethod
    def fallback(cls) -> tuple[str, ...]:
        """This model and every model it extends, top first: the MRO read as keys."""
        return tuple(klass.key for klass in cls.__mro__ if "key" in vars(klass))

    @classmethod
    def rung(cls) -> v1.Rung:
        """Today's v1 record for this class, derived: the class is the source."""
        own, chain = vars(cls).get, cls.fallback()
        return v1.Rung(cls.level, cls.word, cls.title, chain[1] if len(chain) > 1 else None,
                       adds=tuple(n for n in cls.own if cls.parameters[n].required), kind=cls.kind,
                       optional=tuple(n for n in cls.own if not cls.parameters[n].required), code=cls.code,
                       wears=own("wears", ()), composes=own("composes", ()), equation=own("equation", ""),
                       references=own("references", ()), note=own("note", ""))

    @classmethod
    def runnable(cls) -> bool:
        return cls.kind == "physics" and bool(cls.code) and (PACKAGE / cls.code / "model.py").is_file()

    @classmethod
    def inputs(cls) -> tuple[str, ...]:
        """What predict needs besides v, from the rung's model.json (no dt is the steady state)."""
        return tuple(name for name in _reference(cls.code)[2]["inputs"] if name not in ("v", "dt"))

    @classmethod
    def describe(cls) -> str:
        p, record = cls.parameters, cls.rung()
        lines = [f"{cls.key} · class {cls.__name__}({cls.__bases__[0].__name__}) · {cls.title}", f"  {record.equation}"]
        lines += [f"  adds {n} [{p[n].unit or '1'}] {'required' if p[n].required else 'optional'}: {p[n].definition}"
                  for n in cls.own]
        inherited = [f"{n} [{p[n].unit or '1'}]" for n in cls.needs() if n not in cls.own]
        more = sum(1 for n in p if n not in cls.own and not p[n].required)
        siblings = [m.word for m in (cls.family if cls.family is not None else ()) if m.level == cls.level and m is not cls]
        runs = f"{cls.code}/model.py, numpy-vectorized" if cls.runnable() else "declared, not runnable here"
        return "\n".join([*lines, f"  inherits {', '.join(inherited) or 'nothing'}; + {more} optional, read when stated",
                          f"  param_count {cls.param_count} · runs {runs} · also at M{cls.level}: "
                          f"{', '.join(siblings) or 'none'}",
                          "  cites " + "; ".join(r.title.split(")")[0] + ")" for r in record.references)])

    # --- an instance: the model with its values ----------------------------------------------------------------

    def __init__(self, **values: Any) -> None:
        unknown = [name for name in values if name not in self.parameters]
        if unknown:
            raise TypeError(f"{self.key} has no parameter {unknown[0]!r} "
                            f"(canonical: {_canonical(self.family, unknown[0])!r})")
        missing = [name for name in self.needs() if values.get(name) is None]
        if missing:
            raise TypeError(f"{self.key} needs {', '.join(missing)}")
        object.__setattr__(self, "_values", {n: v for n, v in values.items() if v is not None})

    @classmethod
    def _of(cls, values: Mapping[str, Any], fitted_count: int = 0) -> Any:
        """WHY lenient: a fit leaves what its log cannot see (inertia, on a friction sweep) unset, not invented."""
        model = object.__new__(cls)
        object.__setattr__(model, "_values", {n: v for n, v in values.items() if n in cls.parameters and v is not None})
        object.__setattr__(model, "fitted_count", fitted_count)
        return model

    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError(f"{self.key}'s values are fixed: make a new instance, type(m)(**{{**m.values(), ...}})")

    def values(self) -> dict[str, Any]:
        """{canonical name: SI value}: what model.py, sim2real and MuJoCo read, no glue."""
        return dict(self._values)

    @property
    def params(self) -> dict[str, Parameter]:
        return {n: dataclasses.replace(p, value=self._values.get(n)) for n, p in self.parameters.items()}

    def __repr__(self) -> str:
        return f"{type(self).__name__}({', '.join(f'{n}={v:.6g}' for n, v in self._values.items())})"

    @classmethod
    def from_datasheet(cls, datasheet: Any, column: str | None = None) -> Any:
        """Every parameter the datasheet states, converted to SI."""
        values = {}
        for name, spec in type(datasheet).typed_fields().items():
            stated = getattr(datasheet, name)
            if name in cls.parameters and stated is not None:
                stated = stated.for_column(column) if stated.by_column else stated
                values[name] = stated.si if spec.kind == "number" else stated.value
        return cls(**values)

    def predict(self, v: Any, **inputs: Any) -> Any:
        """The resisting torque tau_r [N·m] at output speed v [rad/s]: numpy in, numpy out."""
        if not self.runnable():
            raise NotImplementedError(f"{self.key} is declared, not runnable here (no reference physics)")
        return self._torque(self._values, {"v": v, **inputs})

    @classmethod
    def _torque(cls, values: Mapping[str, Any], inputs: Mapping[str, Any]) -> Any:
        terms = values if isinstance(values, _Reading) else dict(values)
        for term in _reference(cls.code)[2]["terms"]:  # the reference's own symbols (LuGre's sigma2) via aliases
            if term not in terms and _canonical(cls.family, term) in terms:
                terms[term] = terms[_canonical(cls.family, term)]
        return _run(cls.code, terms, inputs)

    def score(self, log: Log) -> float:
        """rms error on `log` [N·m]."""
        error = self.predict(log["v"], **{name: log[name] for name in self.inputs()}) - log["tau"]
        return float(np.sqrt(np.mean(error ** 2)))

    def mujoco(self) -> dict[str, float]:
        """The joint attributes MuJoCo reads, as the rung's model.json maps them."""
        maps = _reference(self.code)[2]["backends"]["mujoco"].get("maps", {})
        return {attribute: self._values[name] for attribute, name in maps.items() if name in self._values}

    # --- fitting: generic, no per-rung seeds -------------------------------------------------------------------

    @classmethod
    def fit(cls, log: Log, start: Model | Mapping[str, Any] | None = None) -> Any:
        """Least squares over every needed parameter the log moves; the rest keep `start`'s values or stay unset."""
        if not cls.runnable():
            raise NotFittable("declared, no reference physics here")
        lacking = [name for name in cls.inputs() if name not in log]
        if lacking:
            raise NotFittable(f"needs input {', '.join(lacking)}, which the log lacks")
        given = {n: v for n, v in dict(start.values() if isinstance(start, Model) else start or {}).items()
                 if n in cls.parameters}
        values = {**{n: _unit_seeds(cls.parameters[n].unit, log)[0] for n in cls.needs()}, **given}
        reading, inputs = _Reading(values), {"v": log["v"], **{n: log[n] for n in cls.inputs()}}
        cls._torque(reading, inputs)
        reads = {_canonical(cls.family, name) for name in reading.read}
        # WHY read AND moved: inertia is never read by a friction curve; LuGre's sigma0 is read but cancels at steady state
        # WHY a jittered probe: equal seeds (breakaway = Coulomb) would hide stribeck_velocity
        probe = {**values, **{n: values[n] * (1.1 + 0.1 * i) for i, n in enumerate(cls.needs())}}
        free = [n for n in cls.needs() if n in reads and cls._moves(probe, n, inputs)]
        stuck = [n for n in cls.needs() if n in reads and n not in free and n not in given]
        if stuck:
            raise NotFittable(f"this log cannot identify {', '.join(stuck)}")
        values = {n: v for n, v in values.items() if n in reads or n in given}
        # WHY the start's value AND the unit's seeds: a climbed value can sit on a bound (M2's viscous at 0 on servo A)
        starts = itertools.product(*(dict.fromkeys([*([given[n]] if n in given else []),
                                                    *_unit_seeds(cls.parameters[n].unit, log)]) for n in free))
        best: tuple[float, dict[str, Any]] = (np.inf, values)
        for seed in starts:
            signs = np.where(np.array(seed) < 0, -1.0, 1.0)

            def solved(x: np.ndarray, signs: np.ndarray = signs) -> dict[str, Any]:  # WHY log space: keeps each sign
                return {**values, **dict(zip(free, (signs * np.exp(np.clip(x, -60, 60))).tolist()))}

            x, cost = _least_squares(lambda x: cls._torque(solved(x), inputs) - log["tau"],
                                     np.log(np.maximum(np.abs(seed), 1e-9)))
            best = min(best, (cost, solved(x)), key=lambda pair: pair[0])
        return cls._of(best[1], fitted_count=len(free))

    @classmethod
    def _moves(cls, values: dict[str, Any], name: str, inputs: dict[str, Any]) -> bool:
        bumped = {**values, name: values[name] * 1.01 + 1e-6}
        return not np.allclose(cls._torque(values, inputs), cls._torque(bumped, inputs), rtol=1e-9, atol=0)


class NotFittable(Exception):
    """Why a batch fit skipped a model: a reason, not a failure."""


class BlackBox(Model):
    """A numpy black box with a physics model's outside: fit, predict, param_count, score. Weights, not Parameters."""

    degree: ClassVar[int] = 3

    def __init__(self, weights: Iterable[float]) -> None:
        object.__setattr__(self, "_values", {})
        object.__setattr__(self, "weights", np.asarray(weights, dtype=float))
        object.__setattr__(self, "fitted_count", self.weights.size)

    @classmethod
    def _count(cls) -> int:
        return 2 * (cls.degree + 1)  # WHY per direction: friction is odd in v, a polynomial is not

    @classmethod
    def runnable(cls) -> bool:
        return True

    @classmethod
    def inputs(cls) -> tuple[str, ...]:
        return ()

    @classmethod
    def fit(cls, log: Log, start: Any = None) -> Any:
        v, tau = log["v"], log["tau"]
        return cls(np.concatenate([np.linalg.lstsq(np.vander(v[side], cls.degree + 1), tau[side], rcond=None)[0]
                                   for side in (v > 0, v < 0)]))

    def predict(self, v: Any, **inputs: Any) -> Any:
        v, half = np.asarray(v, dtype=float), self.degree + 1
        out = np.where(v > 0, np.polyval(self.weights[:half], v), np.where(v < 0, np.polyval(self.weights[half:], v), 0.0))
        return float(out) if out.ndim == 0 else out

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.weights.size} weights, a degree-{self.degree} polynomial per direction)"


# --- a family of models: the type's, or one level's --------------------------------------------------------------


class _FamilyMeta(type):
    """WHY a metaclass: a family is a class, `class IntegratedServo(ModelFamily, id=...)`, and its models are reached
    on the class itself, as Zach wrote them: `IntegratedServo["M3.stribeck"]`, `IntegratedServo.m3.stribeck`,
    `for model in IntegratedServo`."""

    def __getitem__(cls, key: str) -> Any:
        """`IntegratedServo["M3.stribeck"]` is a Model class; `IntegratedServo["M3"]` the M3 family."""
        models = cls._models()
        if key in models:
            return models[key]
        level = LEVEL.fullmatch(key)
        if level:
            return cls._subfamily(int(level[1]))
        raise KeyError(f"{key!r}: no model or level in {cls._name()}; keys are {', '.join(models)}")

    def __getattr__(cls, name: str) -> Any:
        """`IntegratedServo.m3` is the M3 family; `.m3.stribeck` the model whose word is stribeck in that family."""
        if name.startswith("_"):
            raise AttributeError(name)
        level = LEVEL.fullmatch(name)
        try:
            if level:
                return cls._subfamily(int(level[1]))
        except KeyError as error:
            raise AttributeError(str(error)) from None
        found = [model for model in cls._models().values() if model.word == name]
        if len(found) == 1:
            return found[0]
        if found:
            raise AttributeError(f"{name!r} names {', '.join(m.key for m in found)} in {cls._name()}: write "
                                 f".m<level>.{name}")
        raise AttributeError(f"{cls._name()} has no model {name!r}; words here: {', '.join(m.word for m in cls)}")

    def __iter__(cls) -> Iterator[type[Model]]:
        return iter(cls._models().values())

    def __len__(cls) -> int:
        return len(cls._models())

    def __contains__(cls, item: object) -> bool:
        models = cls._models()
        return item in models or (isinstance(item, type) and models.get(getattr(item, "key", None)) is item)

    def __dir__(cls) -> list[str]:
        return sorted({*super().__dir__(), *(m.word for m in cls), *(f"m{m.level}" for m in cls)})

    def __call__(cls, *args: Any, **kwargs: Any) -> Any:
        # WHY refuse: values live on Model instances (v6); a call to the family means nothing yet, so it never
        # silently builds one, and a later meaning has no first one to retire.
        raise TypeError(f"{cls._name()} is a model family, not a value: a model's values are "
                        f"{cls._name()}.m<level>.<word>(...) or what its .fit(log) returns")

    def __repr__(cls) -> str:
        if vars(cls).get("_level") is not None:
            return f"<ModelFamily {cls._name()} · {len(cls)}: {', '.join(m.word for m in cls)}>"
        if "_own" in vars(cls):
            return f"<ModelFamily {cls.__name__} · {cls.id} · {len(cls)} models: {', '.join(cls.keys())}>"
        return super().__repr__()


class ModelFamily(metaclass=_FamilyMeta):
    """A family of Model classes in ladder order, and the batch calls over them; a part type's whole set is one,
    declared as a class whose body nests its models, and every level of it (`IntegratedServo.m3`) is one too.

        IntegratedServo["M3.stribeck"], IntegratedServo.m3.stribeck     # one Model class
        IntegratedServo.m3, IntegratedServo["M3"]                       # the M3 family
        IntegratedServo.fit(log), .select_best(held_out, fits), .ladder(top=), .tree(); for model in IntegratedServo

    WHY one kind of object at every level (2026-11-17): Zach's "a parent class that holds a bunch of single models and
    kinda helps us do compact batch operations", and "M1 models yet again a Model grouping". WHY "ModelFamily" and no
    `.models` (Zach, 2026-11-17: "As we are in robot models all there really is is models"): robot_models knows
    ModelFamily -> Model -> Parameter only; the type that joins a datasheet to its models lives in robotparts (PEP 0004
    amendment 2). "Models" was renamed for readability: "to make Models clearly different from Model".
    """

    id: ClassVar[str]
    aliases: ClassVar[tuple] = ()
    packed: ClassVar[tuple] = ()
    deployment: ClassVar[tuple] = ()
    _root: ClassVar[Any] = None          # a level's family: the type's family it reads; None on the type's own
    _level: ClassVar[int | None] = None

    def __init_subclass__(cls, *, id: str | None = None, level: int | None = None, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if level is not None:  # a level's family (_subfamily): it holds nothing of its own
            return
        if id is None:
            raise TypeError(f"class {cls.__name__}(ModelFamily) needs id=, the type's id (as Ladder.type)")
        cls.id, cls._own, cls._levels = id, {}, {}
        found = [v for v in vars(cls).values() if isinstance(v, builtins.type) and issubclass(v, Model) and "key" in vars(v)]
        for model in sorted(found, key=lambda model: model.level):  # stable: declaration order within a level
            cls._add(model)

    # --- what the family holds --------------------------------------------------------------------------------

    @classmethod
    def _models(cls) -> dict[str, type[Model]]:
        """WHY computed on access: a level's family sees a rung declared after it was first read."""
        own = vars(cls._root or cls).get("_own")
        if own is None:
            return {}
        return own if cls._level is None else {key: m for key, m in own.items() if m.level == cls._level}

    @classmethod
    def _add(cls, model: type[Model]) -> None:
        if cls._level is not None:
            raise TypeError(f"{model.key}: a rung joins its type's family, not the level {cls._name()}")
        calls = {name for name in {*dir(ModelFamily), *vars(_FamilyMeta), "id"} if not name.startswith("_")}
        if model.word in calls:
            raise TypeError(f"{model.key}: the word {model.word!r} is a ModelFamily call; name the rung otherwise")
        vocabulary = {p.name: p for m in cls for p in m.parameters.values()}
        for name in model.own:  # WHY one vocabulary per type (G4): a name means one thing on every branch
            old, new = vocabulary.get(name), model.parameters[name]
            if name in {alias.name for alias in cls.aliases}:
                raise TypeError(f"{model.key}: {name} is an alias of the type, never a parameter name")
            if old is not None and (old.unit, old.definition, old.shape) != (new.unit, new.definition, new.shape):
                raise TypeError(f"{model.key}: {name} [{new.unit}] differs from its other branch's [{old.unit}]")
        model.family = cls
        cls._own[model.key] = model
        cls.__dict__.get("_v1") and delattr(cls, "_v1")  # a new rung changes the derived record

    @classmethod
    def _subfamily(cls, level: int) -> Any:
        if not any(m.level == level for m in cls):
            raise KeyError(f"{cls._name()} has no M{level} models; levels: "
                           f"{', '.join(sorted({f'M{m.level}' for m in cls}, key=lambda s: int(s[1:])))}")
        root = cls._root or cls
        if level not in root._levels:  # WHY cached: `IntegratedServo.m3 is IntegratedServo.m3`
            root._levels[level] = _FamilyMeta(f"{root.__name__}.m{level}", (ModelFamily,),
                                              {"_root": root, "_level": level, "id": root.id, "__doc__": root.__doc__},
                                              level=level)
        return root._levels[level]

    @classmethod
    def _name(cls) -> str:
        return cls.__name__

    @classmethod
    def keys(cls) -> tuple[str, ...]:
        return tuple(cls._models())

    @classmethod
    def values(cls) -> tuple[type[Model], ...]:
        return tuple(cls._models().values())

    @classmethod
    def items(cls) -> tuple[tuple[str, type[Model]], ...]:
        return tuple(cls._models().items())

    # --- the batch calls --------------------------------------------------------------------------------------

    @classmethod
    def ladder(cls, top: type[Model] | None = None) -> tuple[type[Model], ...]:
        """Every model in ladder order; with `top`, its fallback chain (one line, top first), within this family."""
        models = cls._models()
        return tuple(models[key] for key in (top.fallback() if top else models) if key in models)

    @classmethod
    def tree(cls) -> Tree:
        return Tree(cls)

    @classmethod
    def fit(cls, log: Log, start: Model | None = None) -> Fits:
        """Fit every runnable model in the family, climbing: each starts from its nearest fitted base and `start`.
        A model this log cannot fit is a skip with its reason, never a failure."""
        result = Fits(cls, log)
        for model in cls:
            base = next((result[key].values() for key in model.fallback()[1:] if key in result), {})
            try:  # WHY merged: the climb carries the user's start (the datasheet) past rungs that drop its fields
                result[model.key] = model.fit(log, start={**(start.values() if start else {}), **base})
            except NotFittable as reason:
                result.skipped[model.key] = str(reason)
        return result

    @classmethod
    def select_best(cls, held_out: Log, among: Iterable[Model] | Mapping[str, Model],
                    limit: float = SELECT_LIMIT) -> Selection:
        """Among the fitted instances of this family's models: the lowest rung whose held-out rms ÷ the rig's noise
        floor ≤ limit (sim2real's rule), else None. `IntegratedServo.fit(log).select_best(held_out)` is the same."""
        order = list((cls._root or cls).keys())
        candidates = among.values() if isinstance(among, Mapping) else among
        ranked = sorted((m for m in candidates if type(m) in cls), key=lambda m: order.index(m.key))
        return Selection(tuple(ranked), tuple(m.score(held_out) for m in ranked), noise_floor(held_out), limit)

    # --- the type's record ------------------------------------------------------------------------------------

    @classmethod
    def v1_ladder(cls) -> v1.Ladder:
        """Today's `Ladder` record of the type, derived from the classes (the classes are the one source). The
        datasheet class it names is the one the M1 rung reads its fields from (`datasheet=`)."""
        root = cls._root or cls
        if "_v1" not in vars(root):
            datasheet = next((vars(m)["_datasheet"] for m in root if vars(m).get("_datasheet")), None)
            vocabulary = {p.name: p.record() for m in root for p in m.parameters.values()}
            first = [p.name for p in v1.datasheet_vocabulary(datasheet)] if datasheet else []
            order = [*first, *(name for name in vocabulary if name not in first)]
            root._v1 = v1.Ladder(root.id, (root.__doc__ or "").strip(), tuple(vocabulary[n] for n in order),
                                 tuple(m.rung() for m in root), root.aliases, root.packed, root.deployment, datasheet)
        return root._v1

    @classmethod
    def canonical(cls, name: str) -> str:
        return _canonical(cls._root or cls, name)


# --- what the batch calls return ---------------------------------------------------------------------------------


class Fits(dict):
    """{key: fitted instance}, `.skipped` {key: why}; prints as a table; `.select_best(held_out)` chooses."""

    def __init__(self, family: type[ModelFamily], log: Log) -> None:
        super().__init__()
        self.family, self.log, self.skipped = family, log, {}

    def select_best(self, held_out: Log, limit: float = SELECT_LIMIT) -> Selection:
        return self.family.select_best(held_out, self, limit)

    def __str__(self) -> str:
        lines = [f"{self.family._name()}.fit · {self.log['v'].size} samples · {len(self)} fitted, {len(self.skipped)} "
                 f"skipped", f"{'model':<22}{'params':>7}{'fitted':>7}{'fit rms':>9}  [mN·m]"]
        for model in self.family:
            fitted = self.get(model.key)
            lines.append(f"{model.key:<22}{model.param_count:>7}" + (
                f"{fitted.fitted_count:>7}{1e3 * fitted.score(self.log):>9.2f}" if fitted else
                f"{'-':>7}{'-':>9}  skipped: {self.skipped[model.key]}"))
        return "\n".join(lines)


@dataclasses.dataclass(frozen=True)
class Selection:
    """Fitted models in ladder order, scored on held-out plateaus against the rig's noise floor."""

    models: tuple[Model, ...]
    rms: tuple[float, ...]   # N·m, held out
    floor: float             # N·m
    limit: float

    @property
    def chosen(self) -> Model | None:
        """WHY held out (check_scenario.py amendment 1, Zach: "if doing a more complex model doesn't actually improve
        test time performance you are just overfitting"): the first rung that predicts unseen plateaus to the floor."""
        return next((m for m, rms in zip(self.models, self.rms) if rms / self.floor <= self.limit), None)

    def __str__(self) -> str:
        lines = [f"held-out plateaus · floor {1e3 * self.floor:.2f} mN·m · the lowest rung within {self.limit}× floor",
                 f"{'model':<22}{'params':>7}{'held-out rms':>14}{'÷ floor':>9}"]
        lines += [f"{m.key:<22}{m.param_count:>7}{1e3 * rms:>14.2f}{rms / self.floor:>9.2f}"
                  + ("  <- chosen" if m is self.chosen else "") for m, rms in zip(self.models, self.rms)]
        return "\n".join(lines + ([] if self.chosen else ["no rung reaches the floor: climb, or measure more"]))


class Tree:
    """A family's class tree: `.print()` as text, `.show()` writes an HTML page with a mermaid diagram."""

    def __init__(self, family: type[ModelFamily]) -> None:
        self.family = family

    def _parent(self, model: type[Model]) -> str | None:
        """The nearest model it extends within this family: a level's tree is its own roots."""
        return next((key for key in model.fallback()[1:] if key in self.family.keys()), None)

    def _children(self, key: str | None) -> list[type[Model]]:
        return [m for m in self.family if self._parent(m) == key]

    def text(self) -> str:
        """A run of single children is one line (a -> b: b subclasses a); the tree branches where the type does."""
        lines = [f"{self.family.id} · {self.family._name()} · {len(self.family)} models (a -> b: b subclasses a)"]

        def draw(node: type[Model], prefix: str, last: bool) -> None:
            chain, below = [node.key], self._children(node.key)
            while len(below) == 1:
                chain.append(below[0].key)
                below = self._children(below[0].key)
            lines.append(prefix + ("└── " if last else "├── ") + " -> ".join(chain))
            for index, child in enumerate(below):
                draw(child, prefix + ("    " if last else "│   "), index == len(below) - 1)

        top = self._children(None)
        for index, model in enumerate(top):
            draw(model, "", index == len(top) - 1)
        return "\n".join(lines)

    def print(self) -> None:
        print(self.text())

    def mermaid(self) -> str:
        def node(m: type[Model]) -> str:
            adds = "".join(f"<br/>+ {n} [{m.parameters[n].unit or '1'}]" for n in m.own[:4])
            more = f"<br/>+ {len(m.own) - 4} more" if len(m.own) > 4 else ""
            return f'  {m.key.replace(".", "_")}["<b>{m.key}</b> · {m.param_count} params{adds}{more}"]' + (
                "" if m.runnable() else ":::declared")

        edges = [f"  {m.key.replace('.', '_')} --> {parent.replace('.', '_')}" for m in self.family
                 if (parent := self._parent(m))]
        return "\n".join(["graph BT", *map(node, self.family), *edges, "  classDef declared stroke-dasharray: 4 3"])

    def show(self, path: str | Path | None = None) -> Path:
        """Write a page (mermaid from a pinned CDN, the text tree as its fallback) and return its path; opens nothing."""
        name = self.family._name()
        path = Path(path or Path(tempfile.gettempdir()) / f"{name.replace('.', '_')}_tree.html")
        rows = "".join(f"<tr><td>{m.key}<td>{n}<td>{p.unit or '1'}<td>{'required' if p.required else 'optional'}"
                       f"<td>{html.escape(p.definition)}" for m in self.family for n, p in m.parameters.items()
                       if n in m.own)
        path.write_text(f"""<!doctype html><meta charset="utf-8"><title>{name}</title>
<style>body{{font:14px system-ui;margin:24px}}td{{padding:2px 8px}}tr:nth-child(even){{background:#f2f2f2}}</style>
<h1>{name} · {len(self.family)} models</h1><pre class="mermaid">{self.mermaid()}</pre>
<pre>{html.escape(self.text())}</pre><h2>What each model adds</h2><table>{rows}</table>
<script type="module">import m from "https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.esm.min.mjs";
m.initialize({{startOnLoad: true}});</script>""", encoding="utf-8")
        return path


# --- helpers -----------------------------------------------------------------------------------------------------


def noise_floor(log: Log) -> float:
    """What the rig repeats to [N·m]: the pooled within-plateau standard deviation of tau, read from the log itself."""
    plateaus = np.unique(log["plateau"])
    spread = np.concatenate([log["tau"][log["plateau"] == p] - log["tau"][log["plateau"] == p].mean() for p in plateaus])
    return float(np.sqrt(spread @ spread / (spread.size - plateaus.size)))


def _canonical(family: Any, name: str) -> str:
    try:
        return family.v1_ladder().canonical(name) if family is not None else name
    except (KeyError, ValueError):
        return name


def _unit_seeds(unit: str | None, log: Log) -> list[float]:
    """WHY seeds from the unit (V1 needed a seeds() per rung): a unit says which scale of the log a parameter lives on."""
    v, tau = log["v"], log["tau"]
    if unit == "rad/s":
        return list(np.geomspace(np.abs(v[v != 0]).min(), np.abs(v).max(), 6))
    return {"N·m": [float(np.median(np.abs(tau)))], "N·m·s/rad": [abs(float(v @ tau / (v @ v)))]}.get(unit, [1.0])


class _Reading(dict):
    """A terms dict that records what the physics reads: an unread parameter cannot be fitted from this log."""

    def __init__(self, values: Mapping[str, Any]) -> None:
        super().__init__(values)
        self.read: set[str] = set()

    def __getitem__(self, name: str) -> Any:
        self.read.add(name)
        return super().__getitem__(name)

    def get(self, name: str, default: Any = None) -> Any:
        self.read.add(name)
        return super().get(name, default)


NUMPY_MATH = SimpleNamespace(exp=np.exp, sin=np.sin, cos=np.cos, sqrt=np.sqrt, tanh=np.tanh, log=np.log, pi=np.pi)
_LOOPED: set[str] = set()


def _load(path: Path, name: str) -> types.ModuleType:
    """A rung's model.py run from its text. WHY not an import: robotparts vendors the rung folders byte for byte,
    and an import would leave __pycache__/ beside them."""
    module = types.ModuleType(name)
    module.__file__ = str(path)
    exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"), module.__dict__)  # noqa: S102 (our own rung)
    return module


@cache
def _reference(code: str) -> tuple[Callable | None, Callable | None, dict]:
    """Today's model.py twice: as written (stdlib) and with numpy swapped in for math and _sign (vectorized)."""
    folder = PACKAGE / code
    modules = [_load(folder / "model.py", f"robot_models_rung_{folder.name}") for _ in range(2)] \
        if (folder / "model.py").is_file() else []
    if modules:
        modules[1].math, modules[1]._sign = NUMPY_MATH, np.sign
    scalar, vector = (modules[0].torque, modules[1].torque) if modules else (None, None)
    return scalar, vector, json.loads((folder / "model.json").read_text(encoding="utf-8"))


def _run(code: str, terms: Mapping[str, Any], inputs: Mapping[str, Any]) -> Any:
    """WHY one copy of the physics, vectorized: run the reference on whole arrays, checked against its scalar self on
    three samples every call; a reference numpy cannot run (LuGre's bool arithmetic) falls back to a loop, once."""
    scalar, vector, _ = _reference(code)
    columns = dict(zip(inputs, np.broadcast_arrays(*(np.asarray(x, dtype=float) for x in inputs.values()))))
    shape, size = next(iter(columns.values())).shape, next(iter(columns.values())).size

    def one(i: int) -> float:
        return scalar(terms, {name: float(column.flat[i]) for name, column in columns.items()})[0]

    out = None
    if code not in _LOOPED:
        try:
            with np.errstate(all="ignore"):
                out = np.broadcast_to(np.asarray(vector(terms, columns)[0], dtype=float), shape).ravel()
            if not all(np.isclose(out[i], one(i), rtol=1e-12, atol=1e-15) for i in {0, size // 2, size - 1} if size):
                out = None
        except (TypeError, ValueError, AttributeError):
            out = None
        if out is None:
            _LOOPED.add(code)
    out = (np.fromiter((one(i) for i in range(size)), float, count=size) if out is None else out).reshape(shape)
    return float(out) if out.ndim == 0 else out


def _least_squares(residual: Callable[[np.ndarray], np.ndarray], x: np.ndarray, iterations: int = 200) -> tuple:
    """Levenberg-Marquardt, forward-difference Jacobian: numpy only (the runtime has no scipy)."""
    size = residual(x).size

    def safe(point: np.ndarray) -> np.ndarray:  # a step off the model's domain costs "infinitely" much
        try:
            with np.errstate(all="ignore"):
                r = residual(point)
        except ArithmeticError:
            return np.full(size, 1e100)
        return r if np.all(np.abs(r) < 1e100) else np.full(size, 1e100)

    r, damping = safe(x), 1e-3
    cost = float(r @ r)
    for _ in range(iterations if x.size else 0):
        jacobian = np.column_stack([(safe(x + 1e-7 * e) - r) / 1e-7 for e in np.eye(x.size)])
        normal, gradient = jacobian.T @ jacobian, jacobian.T @ r
        while damping < 1e12:
            step = np.linalg.solve(normal + damping * np.diag(np.diag(normal) + 1e-12), -gradient)
            trial = safe(x + step)
            if float(trial @ trial) < cost:
                break
            damping *= 10
        else:
            break
        improvement, x, r, cost = cost - float(trial @ trial), x + step, trial, float(trial @ trial)
        damping = max(damping / 10, 1e-12)
        if improvement < 1e-14 * max(cost, 1e-30):
            break
    return x, cost


__all__ = ["API_NAMES", "BlackBox", "Fits", "Model", "ModelFamily", "NotFittable", "Parameter", "SELECT_LIMIT",
           "Selection", "Tree", "noise_floor"]
