"""v2 "Records in a Family": a Model is a frozen record that carries its physics; a Family is one type's ladder of them.

A Family IS today's Ladder (a subclass: every v1 call keeps working) plus its Models, derived rung by rung. A Model's
physics is today's actuators/rungs/<word>/model.py + model.json, attached as an object instead of a folder named by
string. Runnable: M0-M3 only (the round's stop line); M4-M7 are declared records and say so when run.
"""

from __future__ import annotations

import itertools
import json
import math
import re
import types
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, fields
from pathlib import Path

import numpy as np

import robot_models
from robot_models.ladder import Ladder, Parameter, Reference, Rung, ladders

PACKAGE = Path(robot_models.__file__).resolve().parent
# WHY 3: the babble round's stop line. M4.ripple has a model.py, but nothing above M3 is attached in this prototype.
RUNNABLE_UP_TO = 3
COARSE_POINTS, REFINE_POINTS, REFINE_ROUNDS = 61, 21, 4   # the first grid over a nonlinear term, then finer ones
# WHY a margin: a Stribeck knee just outside the slowest plateau still bends the slowest samples, so look past them.
SPAN_MARGIN = 3.0


def _named(parameter: Parameter) -> str:
    return f"{parameter.name} [{parameter.unit}]" if parameter.unit else parameter.name


def _cited(reference: Reference) -> str:
    """'Authors (year)…' and its link; the full title stays on model.references."""
    head = re.match(r"^.*?\(\d{4}\)", reference.title)
    return f"{head.group(0)}… {reference.url}" if head else f"{reference.title} {reference.url}"


@dataclass(frozen=True)
class Physics:
    """A model's runnable equation: its folder's model.py torque() and what its model.json says about it."""

    folder: str                          # relative to robot_models: "actuators/rungs/stribeck"
    torque: Callable = field(repr=False)  # torque(terms, inputs, state, dt) -> (tau_r, state): scalar, stdlib
    inputs: Mapping[str, str]            # input name -> SI unit
    output: tuple[str, str]              # ("tau_r", "N·m")
    terms: Mapping[str, str]             # canonical name -> the name model.py reads (LuGre reads "sigma2")
    required: tuple[str, ...]            # canonical names the equation cannot run without: what a fit varies
    mujoco: Mapping[str, str]            # MuJoCo joint attribute -> canonical name

    @classmethod
    def load(cls, folder: str, canonical: Callable[[str], str]) -> Physics:
        path, module = PACKAGE / folder, types.ModuleType(f"physics_{Path(folder).name}")
        # WHY compile + exec, not import: an import writes __pycache__/ into a folder robotparts vendors byte for byte.
        exec(compile((path / "model.py").read_text(), str(path / "model.py"), "exec"), module.__dict__)
        spec = json.loads((path / "model.json").read_text())
        terms, maps = spec["terms"], spec["backends"].get("mujoco", {}).get("maps", {})
        return cls(folder, module.torque, {name: item["unit"] for name, item in spec["inputs"].items()},
                   (spec["output"]["name"], spec["output"]["unit"]), {canonical(term): term for term in terms},
                   tuple(canonical(term) for term, item in terms.items() if item.get("required")),
                   {attribute: canonical(term) for attribute, term in maps.items()})

    def __call__(self, values: Mapping[str, float], **inputs) -> np.ndarray:
        """The output for every sample: numpy arrays in, one numpy array out."""
        terms = {term: values[name] for name, term in self.terms.items() if name in values}
        arrays = np.broadcast_arrays(*(np.asarray(value, float) for value in inputs.values()))
        rows = zip(*(array.ravel().tolist() for array in arrays)) if arrays else [()]
        out = [self.torque(terms, dict(zip(inputs, row)))[0] for row in rows]   # a missing input: model.py's KeyError
        return np.reshape(np.array(out, float), arrays[0].shape if arrays else ())


@dataclass(frozen=True)
class Model:
    """One model of a type: what it needs (with units), its equation, its references and, when attached, its physics."""

    family: str                          # the type id it belongs to
    level: int
    name: str
    title: str
    base: str | None                     # the key of the model it extends: the Family walks these
    kind: str                            # physics | residual | learned
    adds: tuple[Parameter, ...]          # needed, new over its base
    reads: tuple[Parameter, ...]         # its own optional names: read when stated
    parameters: tuple[Parameter, ...]    # everything it needs, inherited first
    optional: tuple[Parameter, ...]      # everything it reads when stated, inherited first
    equation: str
    references: tuple[Reference, ...]
    physics: Physics | None = None

    @property
    def key(self) -> str:
        return f"M{self.level}.{self.name}"

    @property
    def runs(self) -> bool:
        return self.physics is not None

    def bind(self, values: Mapping[str, float]) -> BoundModel:
        """This model with values (PyBaMM's process_model): the ones it holds are kept; a missing needed one raises."""
        missing = [_named(p) for p in self.parameters if p.name not in values]
        if missing:
            raise KeyError(f"{self.key} needs {', '.join(missing)}: not in the values")
        return BoundModel(self, {p.name: float(values[p.name]) for p in (*self.parameters, *self.optional)
                                 if p.name in values})

    def fit(self, observed, *, values: Mapping[str, float] | None = None, grid: Mapping | None = None,
            **inputs) -> BoundModel:
        """Fit the terms the equation needs (lmfit's Model.fit: data first, inputs by name); the rest come from values.
        WHY separable least squares (Golub & Pereyra 1973) probed on the physics itself: linear terms are solved by
        lstsq, the others grid-searched, so a new rung's model.py brings no fitting code of its own."""
        physics, observed, given = self._physics(), np.asarray(observed, float).ravel(), dict(values or {})
        free = list(physics.required)
        held = {name: given[name] for name in physics.terms if name in given and name not in free}
        linear, nonlinear = _split(physics, free, held, inputs)
        spans = {name: np.asarray(grid[name], float) if grid and name in grid else self._span(name, inputs)
                 for name in nonlinear}

        def solve(point: dict[str, float]) -> tuple[float, np.ndarray]:
            known = {**held, **point}
            offset = physics({**known, **dict.fromkeys(linear, 0.0)}, **inputs).ravel()
            basis = np.stack([physics({**known, **{n: float(n == name) for n in linear}}, **inputs).ravel() - offset
                              for name in linear], axis=-1) if linear else np.zeros((observed.size, 0))
            coefficients = np.linalg.lstsq(basis, observed - offset, rcond=None)[0]
            return float(np.sum((observed - offset - basis @ coefficients) ** 2)), coefficients

        best = _search(spans, lambda point: solve(point)[0])
        rss, coefficients = solve(best)
        fitted = {**best, **{name: float(value) for name, value in zip(linear, coefficients)}}
        residual = observed - physics({**held, **fitted}, **inputs).ravel()
        # WHY check: the probe shows each linear term is affine alone; this proves they superpose at the answer.
        if not math.isclose(float(residual @ residual), rss, rel_tol=1e-6, abs_tol=1e-12):
            raise ValueError(f"{self.key} is not jointly linear in {', '.join(linear)}: give each a grid=")
        rms = float(np.sqrt(np.mean(residual ** 2)))
        return BoundModel(self, self.bind({**given, **fitted}).values, tuple(free), rms, observed.size)

    def _span(self, name: str, inputs: Mapping) -> np.ndarray:
        # WHY a term with an input's unit is searched across that input's logged range: a stribeck_velocity far
        # outside the speeds you ran cannot be seen in the data anyway.
        unit = next(p.unit for p in (*self.parameters, *self.optional) if p.name == name)
        for input_name, input_unit in self._physics().inputs.items():
            if input_unit == unit and input_name in inputs:
                speeds = np.abs(np.asarray(inputs[input_name], float))
                speeds = speeds[speeds > 0]
                return np.geomspace(speeds.min() / SPAN_MARGIN, speeds.max() * SPAN_MARGIN, COARSE_POINTS)
        raise ValueError(f"{self.key}: {name} [{unit}] enters nonlinearly; give it fit(..., grid={{{name!r}: ...}})")

    def _physics(self) -> Physics:
        if self.physics is None:
            why = (f"physics is attached for M0-M{RUNNABLE_UP_TO} only (the round's stop line)"
                   if self.level > RUNNABLE_UP_TO else "it has no model.py (v1 names no code for it)")
            raise NotImplementedError(f"{self.key} is declared, not runnable here: {why}")
        return self.physics

    def __repr__(self) -> str:
        return f"Model({self.key})"

    def __str__(self) -> str:
        """The card: what it needs, what it means, where it comes from, and what computes it."""
        inherited = [p for p in self.parameters if p not in self.adds]
        physics = self.physics and (
            f"{self.physics.folder}/model.py: torque({', '.join(f'{n} [{u}]' for n, u in self.physics.inputs.items())})"
            f" -> {self.physics.output[0]} [{self.physics.output[1]}]; a fit varies {', '.join(self.physics.required)}")
        return "\n".join([
            f"{self.key}  {self.title}",
            f"  base      {self.base or 'none: a root'}",
            f"  inherits  {', '.join(map(_named, inherited)) or 'nothing'}",
            *(f"  {'adds' if i == 0 else '':<9} {_named(p)}: {p.definition}" for i, p in enumerate(self.adds)),
            f"  optional  {', '.join(map(_named, self.reads)) or 'nothing of its own'} "
            f"(+{len(self.optional) - len(self.reads)} inherited, read when stated)",
            f"  equation  {self.equation}",
            *(f"  {'refs' if i == 0 else '':<9} {_cited(r)}" for i, r in enumerate(self.references)),
            f"  physics   {physics or 'declared, not runnable here'}"])


@dataclass(frozen=True)
class BoundModel:
    """A Model with values, callable on inputs. A fit also records what it fitted and how well."""

    model: Model
    values: dict[str, float]             # canonical name -> SI: a plain dict, so scipy, JSON or sim2real take it as is
    fitted: tuple[str, ...] = ()
    rms: float | None = None
    samples: int = 0

    def __call__(self, **inputs):
        out = self.model._physics()(self.values, **inputs)
        return float(out) if out.ndim == 0 else out

    def mujoco(self) -> dict[str, float]:
        """The values as the MuJoCo joint attributes this model's model.json maps them to."""
        return {attribute: self.values[name] for attribute, name in self.model._physics().mujoco.items()
                if name in self.values}

    def __str__(self) -> str:
        how = f"fitted on {self.samples} samples, rms {self.rms:.4g}" if self.fitted else "bound to stated values"
        extra = [p.name for p in self.model.optional if p.name in self.values]
        return "\n".join([f"{self.model.key} {how}", *(
            f"  {p.name:<18} {self.values[p.name]:<10.5g} {p.unit:<10} {'fitted' if p.name in self.fitted else 'given'}"
            for p in self.model.parameters), *([f"  + {len(extra)} optional values it reads: {', '.join(extra)}"]
                                               if extra else [])])


class Values(dict):
    """A type's parameter values in SI by canonical name, kept apart from any model (PyBaMM's ParameterValues)."""

    def __init__(self, family: Family, si: Mapping[str, float]):
        super().__init__(si)
        self.family = family

    def __repr__(self) -> str:
        rows = (f"  {name:<18} {value:<12.6g} {self.family.parameter(name).unit}" for name, value in self.items())
        return "\n".join([f"Values({self.family.type}, {len(self)} stated, SI):", *map(str.rstrip, rows)])


@dataclass(frozen=True)
class Family(Ladder):
    """A type's vocabulary, aliases and model tree. WHY a Ladder subclass: v1's checks and calls (fallback, canonical,
    parameters, holds, unpack) carry over unchanged, so ladders() could hand out Families with no consumer edit."""

    models: Mapping[str, Model] = field(default_factory=dict, compare=False, repr=False)

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "models", {rung.key: self._model(rung) for rung in self.rungs})

    @classmethod
    def from_ladder(cls, ladder: Ladder) -> Family:
        return cls(**{f.name: getattr(ladder, f.name) for f in fields(Ladder)})

    def _model(self, rung: Rung) -> Model:
        runs = rung.code and rung.level <= RUNNABLE_UP_TO and (PACKAGE / rung.code / "model.py").is_file()
        records = lambda names: tuple(map(self.parameter, names))  # noqa: E731
        return Model(self.type, rung.level, rung.name, rung.title, rung.base, rung.kind, records(rung.adds),
                     records(rung.optional), records(self.parameters(rung.key)), records(self.optional(rung.key)),
                     rung.equation, rung.references, Physics.load(rung.code, self.canonical) if runs else None)

    def __getitem__(self, key: str) -> Model:
        return self.models[self.rung(key).key]  # WHY via rung(): its KeyError names the near miss and every key

    def children(self, key: str) -> tuple[str, ...]:
        """The models that extend this one: where you can climb from it."""
        return tuple(k for k, model in self.models.items() if model.base == self[key].key)

    def siblings(self, key: str) -> tuple[str, ...]:
        """The other models on the same base: the alternatives to this one."""
        base = self[key].base
        return tuple(k for k in self.children(base) if k != key) if base else ()

    def tree(self) -> str:
        """The models as a tree of bases in today's order. A chain with no branch prints on one line, a → b meaning
        b extends a; * marks a model whose physics is attached here."""
        lines = [f"{self.type}: {self.title}"]

        def walk(key: str, prefix: str, branch: str, below: str) -> None:
            chain = [key]
            while len(self.children(chain[-1])) == 1:
                chain.append(self.children(chain[-1])[0])
            lines.append(prefix + branch + " → ".join(k + "*" * self.models[k].runs for k in chain))
            kids = self.children(chain[-1])
            for i, kid in enumerate(kids):
                last = i == len(kids) - 1
                walk(kid, prefix + below, "└── " if last else "├── ", "    " if last else "│   ")

        for root in (k for k, model in self.models.items() if model.base is None):
            walk(root, "", "", "")
        runs = sum(model.runs for model in self.models.values())
        return "\n".join([*lines, f"* runnable here: {runs} of {len(self.models)} (physics attached for "
                                  f"M0-M{RUNNABLE_UP_TO} only); the rest are declared records"])

    def values(self, source) -> Values:
        """Values from a datasheet (its stated numbers, in SI) or from {any name the type knows: number or Stated}."""
        if hasattr(source, "typed_fields"):
            source = {name: getattr(source, name) for name in source.typed_fields()}
        si = {}
        for name, value in source.items():
            if getattr(value, "is_number", True) is False or value is None:
                continue
            if not hasattr(value, "si") and getattr(self.alias(name), "unit", ""):
                raise ValueError(f"{name} is in {self.alias(name).unit}: pass Stated(value, unit), not a bare number")
            si[self.canonical(name)] = float(getattr(value, "si", value))   # a Stated converts by its own unit
        return Values(self, si)

    def compare(self, *bound: BoundModel, observed, **inputs) -> Comparison:
        """Score bound models on one log. BIC = n·ln(RSS/n) + k·ln(n) (Schwarz 1978), k the fitted terms: a higher
        model must buy each term it adds with a lower residual."""
        observed, rows = np.asarray(observed, float).ravel(), []
        for each in bound:
            if each.model.family != self.type:
                raise ValueError(f"{each.model.key} is a {each.model.family} model, not {self.type}")
            n, rss, k = observed.size, float(np.sum((observed - np.ravel(each(**inputs))) ** 2)), len(each.fitted)
            rows.append((each.model.key, k, math.sqrt(rss / n), n * math.log(rss / n) + k * math.log(n)))
        return Comparison(tuple(sorted(rows, key=lambda row: row[3])))


@dataclass(frozen=True)
class Comparison:
    rows: tuple[tuple[str, int, float, float], ...]   # (key, fitted terms, rms, BIC), best BIC first

    @property
    def chosen(self) -> str:
        return self.rows[0][0]

    def __repr__(self) -> str:
        lines = [f"{'model':<20} {'fitted':>6} {'rms':>10} {'BIC':>10}"]
        lines += [f"{key:<20} {k:>6} {rms:>10.4g} {bic:>10.1f}" for key, k, rms, bic in self.rows]
        if len(self.rows) > 1:
            lines.append(f"chosen: {self.chosen}, BIC {self.rows[1][3] - self.rows[0][3]:.1f} below {self.rows[1][0]}")
        return "\n".join(lines)


def _split(physics: Physics, free: list[str], held: dict, inputs: Mapping) -> tuple[list[str], list[str]]:
    """Which free terms enter the equation linearly (lstsq solves them) and which need a search."""
    # WHY distinct probe values: equal levels (coulomb = breakaway) would flatten the Stribeck knee and hide its speed.
    probe = dict(zip(free, np.random.default_rng(0).uniform(0.5, 1.5, len(free)).tolist()))
    linear, nonlinear = [], []
    for name in free:
        try:
            a, b, c = (physics({**held, **probe, name: probe[name] * k}, **inputs) for k in (1, 2, 3))
        except ArithmeticError:
            nonlinear.append(name)
            continue
        if np.allclose(a, b) and np.allclose(b, c):
            raise ValueError(f"this log does not move {name}: state it in values= instead of fitting it")
        (linear if np.allclose(c - b, b - a, rtol=1e-9, atol=1e-12) else nonlinear).append(name)
    return linear, nonlinear


def _search(spans: Mapping[str, np.ndarray], score: Callable[[dict], float]) -> dict[str, float]:
    """Grid search over the nonlinear terms, then finer log-spaced grids between the best point's neighbours."""
    names, best = list(spans), {}
    for _ in range(REFINE_ROUNDS + 1 if names else 0):
        best = min((dict(zip(names, map(float, point))) for point in itertools.product(*spans.values())), key=score)
        index = {name: int(np.argmin(np.abs(spans[name] - best[name]))) for name in names}
        spans = {name: np.geomspace(span[max(index[name] - 1, 0)], span[min(index[name] + 1, len(span) - 1)],
                                    REFINE_POINTS) for name, span in spans.items()}
    return best


def families() -> dict[str, Family]:
    """{type: Family} for every type folder: today's ladders, each read as a Family of Models."""
    return {name: Family.from_ladder(ladder) for name, ladder in ladders().items()}
