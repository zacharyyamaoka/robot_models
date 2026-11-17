"""v5 "Type kit": name a type and it hands you everything it has (Zach, 2026-11-11: "robot types I can assign those
generic types and then they get all of the stuff for those types kinda like automatically").

    servo = types.integrated_servo          # or types.of(datasheet): the datasheet's class names its type
    servo.models, servo.datasheet, servo.experiments, servo.fidelity     # the kit
    found = servo.identify(fit_log, datasheet=ds); servo.compare(found, held_out_log).chosen

Nothing is retyped: rungs, parameters, aliases, equations and references come from robot_models.ladder; the experiment
that identifies each parameter, its defaults and the physics come from actuators/rungs/<word>/model.json and model.py.
"""

from __future__ import annotations

import json
import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

import robot_models
from robot_models.ladder import Ladder, Parameter, Rung, ladders

PACKAGE = Path(robot_models.__file__).resolve().parent
RUNNABLE_TOP_LEVEL = 3   # the round's stop line: physics runs for M0-M3; M4-M7 are declared
TWINNED_LIMIT = 1.5      # WHY sim2real's cards.schema.TWINNED_LIMIT: twinned within 1.5x the rig's own floor
SLOW_BELOW = 0.5         # rad/s, robotparts' measured_by: "slow plateaus below 0.5 rad/s"


def _plateaus(log) -> np.ndarray:
    """Every sample's plateau: the log's `plateau` id (one per held speed and direction), else the speed repeating."""
    return np.asarray(log["plateau"]) if "plateau" in log else np.round(np.asarray(log["v"], float), 6)


def _both_ways(log, at_least: int, below: float = np.inf) -> str | None:
    speeds = set(np.unique(np.round(np.asarray(log["v"], float), 6)).tolist())
    held = [s for s in speeds if 0 < s < below and -s in speeds]
    return None if len(held) >= at_least else f"{len(held)} plateaus held both ways below {below} rad/s, needs {at_least}"


# WHY a table, not parsed text: what counts as "this motion" is the one thing the kit adds over model.json. Only plateau
# logs are read here; every other motion says so instead of pretending.
COVERAGE: dict[str, Callable] = {
    # WHY 5, not robotparts' "6-8 speeds": that is what the rig runs; held-out choice (amendment 1) fits on every other
    # plateau of a 10-speed run, so the half a fit sees holds 5.
    "velocity_plateaus": lambda log: _both_ways(log, at_least=5),
    "slow_plateaus": lambda log: _both_ways(log, at_least=3, below=SLOW_BELOW),
}


def experiment_id(motion: str) -> str:
    """'slow plateaus below 0.5 rad/s' -> 'slow_plateaus': the motion's own words up to its first qualifier."""
    head = re.sub(r"\b(below|with|then|or)\b.*$", "", re.split(r"[,(]", motion)[0])
    return "_".join(w for w in re.split(r"[\s-]+", head.lower()) if w and w not in ("a", "the"))


@dataclass(frozen=True)
class Experiment:
    """A rig motion that identifies parameters: robotparts' measured_by text, named by its own words."""

    id: str
    motion: str                                     # model.json's measured_by, verbatim
    identifies: tuple[tuple[str, str], ...] = ()    # (model key, canonical parameter) pairs

    @property
    def reads_datasheet(self) -> bool:              # "reading the datasheet", "chirps (or the datasheet)"
        return "datasheet" in self.motion

    def covers(self, log) -> str | None:
        """None when the log is this motion; otherwise why it is not."""
        if log is None or self.id not in COVERAGE:
            return "no log of it" if log is None else "not a plateau log (the only kind this prototype reads)"
        return COVERAGE[self.id](log)

    def __str__(self) -> str:
        unlocks = ", ".join(dict.fromkeys(key for key, _ in self.identifies))
        return f"{self.id}: \"{self.motion}\"\n  identifies {', '.join(dict.fromkeys(n for _, n in self.identifies))}" \
               f"\n  on {unlocks}"


class Experiments(dict):
    def covered_by(self, log) -> tuple[str, ...]:
        """The ids of the motions this log is."""
        return tuple(key for key, experiment in self.items() if experiment.covers(log) is None)


@dataclass(frozen=True, repr=False)
class Model:
    """One model of a type: what it needs and the experiment identifying each, its equation, references and fallback,
    and (M0-M3) its physics. The same class is the declaration and, with values, the identified model that predicts."""

    rung: Rung                                      # today's v1 record: level, title, equation, references, code
    needs: tuple[Parameter, ...]                    # required, inherited ones first (Ladder.parameters)
    reads: tuple[Parameter, ...]                    # its own optional parameters: read when stated
    identified_by: Mapping[str, Experiment]         # canonical parameter -> the experiment identifying it here
    base: Model | None = None
    physics: Callable | None = None
    why_not: str = ""                               # why it does not run, when it does not
    defaults: Mapping[str, float] = field(default_factory=dict)  # model.json "default" (stribeck_exponent: 2)
    terms: Mapping[str, str] = field(default_factory=dict)       # model.py's alias term -> canonical (sigma2)
    values: Mapping[str, float] = field(default_factory=dict)    # SI, by canonical name
    sources: Mapping[str, str] = field(default_factory=dict)     # parameter -> "datasheet" or an experiment id

    key = property(lambda self: self.rung.key)
    level = property(lambda self: self.rung.level)

    @property
    def fallback(self) -> tuple[Model, ...]:
        return (self, *(self.base.fallback if self.base else ()))

    @property
    def adds(self) -> tuple[Parameter, ...]:
        inherited = {p.name for p in self.base.needs} if self.base else set()
        return tuple(p for p in self.needs if p.name not in inherited) + self.reads

    @property
    def experiments(self) -> tuple[Experiment, ...]:
        """The rig motions that identify what this model adds, as its own model.json names them."""
        own = {p.name for p in self.adds}
        return tuple({e.id: e for n, e in self.identified_by.items() if (self.key, n) in e.identifies
                      and n in own}.values())

    def with_values(self, sources: Mapping[str, str] | None = None, **values: float) -> Model:
        return replace(self, values={**self.values, **values}, sources={**self.sources, **(sources or {})})

    def predict(self, v, **inputs):
        """The resisting torque tau_r [N·m] at output speed v (a number or an array), from the rung's own model.py."""
        if self.physics is None:
            raise NotImplementedError(f"{self.key} does not run: {self.why_not}")
        values = {**self.defaults, **self.values}
        terms = {**values, **{term: values[name] for term, name in self.terms.items() if name in values}}
        speeds = np.asarray(v, float)
        try:
            out = np.array([self.physics(terms, {"v": float(s), **inputs})[0] for s in speeds.ravel()])
        except KeyError as error:
            name = self.terms.get(error.args[0], error.args[0])
            how = self.identified_by.get(name)
            raise KeyError(f"{self.key} needs {name}: run {how.id if how else '?'} or pass it to with_values()") from None
        return float(out[0]) if speeds.ndim == 0 else out.reshape(speeds.shape)

    def __repr__(self) -> str:
        return f"<Model {self.key}{f' with {len(self.values)} values' if self.values else ''}>"

    def __str__(self) -> str:
        grouped: dict[str, list[str]] = {}
        for p in (*self.needs, *self.reads):
            how = self.identified_by.get(p.name)
            optional = f" (optional, {self.defaults[p.name]:g} when absent)" if p.name in self.defaults else ""
            grouped.setdefault(how.id if how else "(none named)", []).append(f"{p.name} [{p.unit}]{optional}")
        cited = (re.match(r".*?\(\d{4}\)", r.title) for r in self.rung.references)   # "Authors (year)"
        return "\n".join([f"{self.key}  {self.rung.title}", "  identified by",
                          *(f"    {how:<20}{', '.join(names)}" for how, names in grouped.items()),
                          f"  equation    {self.rung.equation}",
                          f"  references  {'; '.join(m.group(0) for m in cited if m)}",
                          f"  fallback    {' -> '.join(m.key for m in self.fallback)}",
                          f"  physics     {self.rung.code + '/model.py (runs)' if self.physics else self.why_not}"])


class Models(dict):
    """A type's models by key, in ladder order; prints as the ladder's tree, each with the motions that identify it."""

    def __str__(self) -> str:
        children: dict[str | None, list[Model]] = {}
        for model in self.values():
            children.setdefault(model.base.key if model.base else None, []).append(model)
        lines: list[str] = []

        def walk(model: Model, lead: str, indent: str) -> None:
            status = "runs" if model.physics else model.why_not.split(":")[0]
            motions = ", ".join(e.id for e in model.experiments) or "-"
            lines.append(f"{lead + model.key:<38}{status:<16}{motions}")
            kids = children.get(model.key, [])
            for i, kid in enumerate(kids):
                last = i == len(kids) - 1
                walk(kid, indent + ("└─ " if last else "├─ "), indent + ("   " if last else "│  "))

        for root in children.get(None, []):
            walk(root, "", "")
        return "\n".join(lines)


@dataclass(frozen=True)
class Fidelity:
    key: str
    error: float      # RMS of the model's output against the log
    floor: float      # the scatter within each plateau: what a perfect model still misses
    ratio = property(lambda self: self.error / self.floor)
    twinned = property(lambda self: self.ratio <= TWINNED_LIMIT)


@dataclass(frozen=True)
class FidelityMetric:
    """sim2real's FidelityMetric shape: what is compared, the floor it is divided by, and how it is evaluated."""

    id: str
    compares: str
    unit: str
    floor: str = "the scatter within each held plateau (the rig's own repeatability)"

    def evaluate(self, model: Model, log) -> Fidelity:
        tau, (_, index) = np.asarray(log["tau"], float), np.unique(_plateaus(log), return_inverse=True)
        means = np.bincount(index, tau) / np.bincount(index)
        floor = float(np.sqrt(np.sum((tau - means[index]) ** 2) / (tau.size - means.size)))
        return Fidelity(model.key, float(np.sqrt(np.mean((tau - model.predict(log["v"])) ** 2))), floor)


class Identified(dict):
    """{key: Model with values} that a log and a datasheet identify; `missing` says what each other model lacks, and
    `log` is what the values were fitted on (so the kit can refuse to score them on it)."""

    def __init__(self, found: dict, missing: dict, log=None):
        super().__init__(found)
        self.missing, self.log = missing, log

    def __str__(self) -> str:
        lines = []
        for key, model in self.items():
            fitted = {n: v for n, v in model.values.items() if model.sources.get(n) != "datasheet"}
            lines.append(f"  {key:<20}" + (", ".join(f"{n}={v:.4g}" for n, v in fitted.items())
                                           or f"the datasheet's {len(model.values)} stated values"))
        by_reason: dict[str, list[str]] = {}
        for key, why in self.missing.items():
            by_reason.setdefault(why, []).append(key)
        return "\n".join(lines + [f"  not yet: {', '.join(keys)}: {why}" for why, keys in by_reason.items()])


@dataclass(frozen=True)
class Comparison:
    rows: tuple[Fidelity, ...]          # on the held-out plateaus: what the choice is made on
    chosen: Model | None
    on_fit: tuple[Fidelity, ...] = ()   # the same metric on the fit plateaus: shown, never chosen on

    def __str__(self) -> str:
        fit = {r.key: f"{r.ratio:.2f}" for r in self.on_fit}
        return "\n".join([f"  {'model':<20}{'fit x floor':>12}{'held-out rms N·m':>18}{'floor':>8}{'x floor':>9}"
                          f"  twinned (<= {TWINNED_LIMIT}x)",
                          *(f"  {r.key:<20}{fit.get(r.key, '-'):>12}{r.error:>18.4f}{r.floor:>8.4f}{r.ratio:>9.2f}"
                            f"  {'yes' if r.twinned else 'no'}" for r in self.rows),
                          f"  chosen: {self.chosen.key}, the lowest rung the rig cannot tell from the part on plateaus "
                          f"the fit never saw" if self.chosen else
                          f"  chosen: none within {TWINNED_LIMIT}x on held-out plateaus: climb (found.missing)"])


def fit_plateaus(model: Model, log, names: list[str], known: Mapping[str, float]) -> dict[str, float]:
    """Least squares on a plateau log. WHY separable: friction levels (N·m) and slopes enter the friction rungs linearly,
    so lstsq solves them; a speed scale (rad/s) sets where the curve bends, so it is scanned over the speeds the log
    held, then refined by golden section. The columns come from the rung's own model.py, never a retyped equation."""
    v, tau = np.asarray(log["v"], float), np.asarray(log["tau"], float)
    units = {p.name: p.unit for p in model.needs}
    scales = [n for n in names if units[n] == "rad/s"]
    linear = [n for n in names if n not in scales]
    if len(scales) > 1:
        raise NotImplementedError(f"{model.key}: this fitter scans one speed scale, not {scales}")

    def solve(scale: list[float]) -> tuple[float, dict[str, float], np.ndarray]:
        base = model.with_values(**known, **dict(zip(scales, scale)), **dict.fromkeys(linear, 0.0))
        offset = base.predict(v)
        columns = np.column_stack([base.with_values(**{n: 1.0}).predict(v) - offset for n in linear])
        coefficients = np.linalg.lstsq(columns, tau - offset, rcond=None)[0]
        fitted = columns @ coefficients + offset
        return float(np.sum((fitted - tau) ** 2)), {**dict(zip(scales, scale)), **dict(zip(linear, coefficients))}, fitted

    if not scales:
        _, values, fitted = solve([])
    else:
        grid = np.geomspace(np.abs(v[v != 0]).min() / 3, np.abs(v).max() * 3, 120)
        best = int(np.argmin([solve([g])[0] for g in grid]))
        low, high = np.log(grid[max(best - 1, 0)]), np.log(grid[min(best + 1, grid.size - 1)])
        for _ in range(40):
            a, b = low + 0.382 * (high - low), low + 0.618 * (high - low)
            low, high = (low, b) if solve([np.exp(a)])[0] < solve([np.exp(b)])[0] else (a, high)
        _, values, fitted = solve([float(np.exp((low + high) / 2))])
    # WHY check superposition at the answer: the columns assume the rung is linear in `linear`; its own physics says.
    if not np.allclose(model.with_values(**known, **values).predict(v), fitted, rtol=1e-9, atol=1e-12):
        raise ValueError(f"{model.key} is not linear in {linear}: this fitter covers levels and slopes only")
    return {n: float(values[n]) for n in names}


def _physics(code: str) -> Callable:
    # WHY exec the source, never import it (PEP 0004 §8.1): an import leaves __pycache__/ in a vendored rung folder,
    # whose file set robotparts hashes byte for byte.
    path = PACKAGE / code / "model.py"
    namespace: dict = {"__name__": f"rung_{Path(code).name}", "__file__": str(path)}
    exec(compile(path.read_text(), str(path), "exec"), namespace)
    return namespace["torque"]


@dataclass(frozen=True, repr=False)
class Kit:
    """Everything a type has: its models (the ladder), its datasheet class, its experiments and its fidelity metric."""

    type: str
    ladder: Ladder                      # today's v1 object, kept whole: the kit is a layer over it
    models: Models
    experiments: Experiments
    fidelity: FidelityMetric | None

    datasheet = property(lambda self: self.ladder.datasheet)

    def canonical(self, name: str) -> str:
        return self.ladder.canonical(name)

    def stated(self, datasheet) -> dict[str, float]:
        """The datasheet's numeric typed fields in SI, by canonical name."""
        return {name: s.si for name in type(datasheet).typed_fields()
                if (s := getattr(datasheet, name)) is not None and s.is_number}

    def from_datasheet(self, datasheet, key: str = "M1.datasheet") -> Model:
        stated = self.stated(datasheet)
        return self.models[key].with_values(dict.fromkeys(stated, "datasheet"), **stated)

    def identify(self, log=None, datasheet=None) -> Identified:
        """Every runnable model that the log's experiments and the datasheet identify, with values."""
        stated = self.stated(datasheet) if datasheet is not None else {}
        found, missing = {}, {}
        for model in self.models.values():
            fit, known, lacking = [], {}, [model.why_not] if model.physics is None else []
            for p in model.needs:
                how = model.identified_by.get(p.name)
                if how is not None and how.covers(log) is None:
                    fit.append(p.name)
                elif (how is None or how.reads_datasheet) and p.name in stated:
                    known[p.name] = stated[p.name]
                elif how is None or how.motion.startswith("reading"):   # no motion to run: a source must state it
                    lacking.append(f"state {p.name}")
                else:
                    lacking.append(f"run {how.id}" + (f" or state {p.name}" if how.reads_datasheet else ""))
            if lacking:
                missing[model.key] = lacking[0] if model.physics is None else ", ".join(dict.fromkeys(lacking))
                continue
            # WHY optional terms are read, never fitted (robotparts' model.json required flags): an optional term keeps
            # its stated value or its default, so the fit carries only what the model needs.
            known |= {p.name: stated[p.name] for p in model.reads if p.name in stated}
            values = fit_plateaus(model, log, fit, known) if fit else {}
            sources = {**dict.fromkeys(known, "datasheet"), **{n: model.identified_by[n].id for n in fit}}
            found[model.key] = model.with_values(sources, **known, **values)
        return Identified(found, missing, log)

    def compare(self, found: Mapping[str, Model], held_out) -> Comparison:
        """The type's fidelity metric on every identified model, on plateaus the fit never saw. Chosen (sim2real's
        cards.schema.select): the lowest rung within TWINNED_LIMIT of the rig's own floor, same-level siblings by ratio;
        None when none is. WHY held out (Zach, 2026-11-17: "if doing a more complex model doesn't actually improve test
        time performance you are just overfitting"): a rung that nests another always fits its own plateaus as well."""
        fit = getattr(found, "log", None)
        if fit is not None and (seen := set(_plateaus(fit).tolist()) & set(_plateaus(held_out).tolist())):
            raise ValueError(f"the fit saw {len(seen)} of these plateaus: choose on plateaus it never saw")
        rows = tuple(self.fidelity.evaluate(model, held_out) for model in found.values())
        twinned = [r for r in rows if r.twinned]
        pick = min(twinned, key=lambda r: (found[r.key].level, r.ratio)) if twinned else None
        on_fit = tuple(self.fidelity.evaluate(model, fit) for model in found.values()) if fit is not None else ()
        return Comparison(rows, found[pick.key] if pick else None, on_fit)

    def __repr__(self) -> str:
        status = ["runs" if m.physics else m.why_not.split(":")[0] for m in self.models.values()]
        counts = ", ".join(f"{status.count(s)} {s}" for s in dict.fromkeys(status))
        sheet = f"{self.datasheet.__name__}, {len(self.datasheet.typed_fields())} typed fields" if self.datasheet else "-"
        return "\n".join([f"<{self.type} kit> {self.ladder.title}",
                          f"  models       {len(self.models)} on the ladder: {counts}", f"  datasheet    {sheet}",
                          f"  experiments  {', '.join(self.experiments) or '-'}",
                          f"  fidelity     {self.fidelity.id if self.fidelity else '-'}"
                          f"{' on held-out plateaus over the rig floor' if self.fidelity else ''}"])


def build_kit(ladder: Ladder) -> Kit:
    """A type's kit from its v1 Ladder and its rung folders: every model, every experiment, the fidelity metric."""
    cards = {r.key: json.loads((PACKAGE / r.code / "model.json").read_text())
             if r.code and (PACKAGE / r.code / "model.json").exists() else {} for r in ladder.rungs}
    own, terms, defaults = ({r.key: {} for r in ladder.rungs} for _ in range(3))
    for key, card in cards.items():                # own: {parameter: measured_by}; terms: aliases; defaults
        for term, spec in card.get("terms", {}).items():
            try:
                names = ladder.resolve(term)       # canonical, alias or packed: one vocabulary per type
            except KeyError:
                continue                           # a deployment-only name (MuJoCo's controller, saturation)
            terms[key] |= {term: names[0]} if ladder.alias(term) else {}             # LuGre's sigma2
            defaults[key] |= {names[0]: spec["default"]} if "default" in spec and len(names) == 1 else {}
            own[key] |= dict.fromkeys(names, spec["measured_by"])
    pairs: dict[str, list] = {}
    for key, found in own.items():
        for name, motion in found.items():
            pairs.setdefault(motion, []).append((key, name))
    experiments = Experiments({experiment_id(m): Experiment(experiment_id(m), m, tuple(p)) for m, p in pairs.items()})
    models = Models()

    def build(key: str) -> Model:
        if key not in models:
            rung, chain = ladder.rung(key), ladder.fallback(key)
            identified = {name: experiments[experiment_id(own[step][name])]
                          for name in (*ladder.parameters(key), *rung.optional)
                          for step in [next((s for s in chain if name in own[s]), None)] if step}
            physics, why = None, "structure only: no published physics (the type holds its terms)"
            if rung.level > RUNNABLE_TOP_LEVEL:
                why = f"declared: this round runs M0-M{RUNNABLE_TOP_LEVEL} only"
            elif rung.code:
                physics, why = _physics(rung.code), ""
            models[key] = Model(rung, tuple(map(ladder.parameter, ladder.parameters(key))),
                                tuple(map(ladder.parameter, rung.optional)), identified,
                                build(rung.base) if rung.base else None, physics, why, defaults[key], terms[key])
        return models[key]

    models = Models((key, build(key)) for key in ladder.keys)
    root = cards[ladder.keys[0]].get("output")
    fidelity = FidelityMetric(f"{root['name']}_rms", root["definition"], root["unit"]) if root else None
    return Kit(ladder.type, ladder, models, experiments, fidelity)


class Types(dict):
    """Every type's kit by name, as attributes too; `of(datasheet)` is Hugging Face AutoModel's move: the datasheet's
    class names its type the way a config's model_type names its architecture."""

    def __getattr__(self, name: str) -> Kit:
        try:
            return self[name]
        except KeyError:
            raise AttributeError(f"no type {name!r}; the types: {', '.join(self)}") from None

    def of(self, datasheet) -> Kit:
        """The kit whose Datasheet class this is, exactly (an IntegratedServo is no bldc_motor)."""
        return next(kit for kit in self.values() if kit.datasheet is type(datasheet))


types = Types((name, build_kit(ladder)) for name, ladder in sorted(ladders().items()))
