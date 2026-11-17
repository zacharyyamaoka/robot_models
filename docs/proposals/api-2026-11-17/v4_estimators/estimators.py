"""v4 · Estimators that climb: a Model is an estimator (sklearn's verbs); a Family is model selection over its ladder.

    m3 = Model.from_datasheet(datasheet, "M3.stribeck").fit(fit_log)   # then predict(v), rms(log), score(log), get_params()
    print(families()["integrated_servo"].climb(datasheet, fit_log, test_log))   # up the ladder until held out hits the floor

A rung's names, units, equation, references, aliases and fallback are read from robot_models.ladder; every torque
comes from actuators/rungs/<word>/model.py. numpy + stdlib + robot_models only.
"""

from __future__ import annotations

import functools
import importlib.util
import itertools
import json
from collections.abc import Mapping
from pathlib import Path

import numpy as np

import robot_models
from robot_models.datasheet.units import to_si
from robot_models.ladder import Ladder, Parameter, ladders

PACKAGE = Path(robot_models.__file__).resolve().parent
TOP_RUNNABLE_LEVEL = 3     # WHY: the round's stop line; M4–M7 are declared and say so when called
FIRST_FITTED_LEVEL = 2     # WHY: M0 and M1 state the part and fit nothing (the servo ladder's docstring)
TARGET = "tau"             # WHY: the rig log's measured column, the torque the motor added: every rung's tau_r
PLATEAU = "plateau"        # the rig log's id per held speed and direction: what makes its noise floor observable
# WHY 1.5: sim2real's selection rule (cards/schema.py select(), TWINNED_LIMIT): a rung within 1.5× the rig's own
# repeatability on plateaus it never saw has nothing left to explain, so a richer rung would only fit noise.
FLOOR_LIMIT = 1.5


class NotRunnable(RuntimeError):
    """The rung is on the ladder, but there is no physics here to run it, or the log lacks its inputs."""


@functools.cache
def _physics(code: str):
    """A rung folder's model.py (loaded by path: the folders are data, not a package) and its model.json."""
    folder = PACKAGE / code
    spec = importlib.util.spec_from_file_location(f"robot_models_rung_{folder.name}", folder / "model.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, json.loads((folder / "model.json").read_text())


class Model:
    """One rung of a type's ladder as an estimator: it says what it needs and means, and it computes and fits."""

    def __init__(self, family: Family, key: str, **values: float):
        self.family, self.rung = family, family.ladder.rung(key)
        self.values: dict[str, float] = {}       # {canonical name: SI value}: stated, set or fitted
        self.fitted_: dict[str, float] = {}      # sklearn's convention: what fit() learned ends in _
        self.rms_, self.n_samples_ = None, None
        self.set_params(**values)

    # --- what it is: read from the ladder, never retyped ------------------------------------------------------------

    key = property(lambda self: self.rung.key)
    needs = property(lambda self: self.family.ladder.parameters(self.key))     # required, inherited ones first
    holds = property(lambda self: self.family.ladder.holds(self.key))          # ... plus what it reads when stated
    fallback = property(lambda self: self.family.ladder.fallback(self.key))

    @property
    def free(self) -> tuple[str, ...]:
        """What fit() estimates: what the rung needs from M2 up. WHY not the optional names: they keep their
        default (Stribeck's exponent is 2), so a rung is never charged for a parameter it does not need."""
        ladder = self.family.ladder
        return tuple(name for step in reversed(self.fallback) if ladder.rung(step).level >= FIRST_FITTED_LEVEL
                     for name in ladder.rung(step).adds)

    inputs = property(lambda self: () if self._declared() else tuple(_physics(self.rung.code)[1]["inputs"]))

    def parameter(self, name: str) -> Parameter:
        return self.family.ladder.parameter(name)

    def _declared(self) -> str | None:
        if self.rung.level > TOP_RUNNABLE_LEVEL:
            return "declared, above the M0–M3 stop line"
        if not self.rung.code or not (PACKAGE / self.rung.code / "model.py").exists():
            return "declared, no physics shipped"
        return None

    def why_not(self, log: Mapping | None = None) -> str | None:
        """Why this model cannot run (or cannot fit `log`); None when it can."""
        missing = [name for name in (*self.inputs, TARGET) if log is not None and name not in log]
        return self._declared() or (f"the log has no {', '.join(missing)}" if missing else None)

    def describe(self) -> str:
        def listed(names):
            return ", ".join(f"{name} [{self.parameter(name).unit or '1'}]" for name in names) or "none"
        below = len(self.holds) - len(self.needs) - len(self.rung.optional)
        return "\n".join((
            f"{self.key}  {self.rung.title}",
            f"  falls back  {' → '.join(self.fallback)}   (siblings: {', '.join(self.family.siblings(self.key))})",
            f"  needs       {listed(self.needs)}",
            f"  fits        {', '.join(self.free) or 'nothing: it states the part'}",
            f"  optional    {listed(self.rung.optional)} (+{below} datasheet fields below it, read when stated)",
            f"  equation    {self.rung.equation}",
            *(f"  reference   {reference.title}" for reference in self.rung.references),
            f"  physics     {self._declared() or f'{self.rung.code}/model.py: tau_r({", ".join(self.inputs)})'}",
        ))

    # --- the estimator protocol (scikit-learn's verbs) --------------------------------------------------------------

    @classmethod
    def from_datasheet(cls, datasheet, key: str = "M1.datasheet") -> Model:
        """The rung holding every value the datasheet states for it, in SI; what the datasheet lacks stays to fit.
        WHY found by the datasheet's class (Ladder.datasheet): the caller never names the family twice."""
        family = next((f for f in families().values() if f.ladder.datasheet is type(datasheet)), None)
        if family is None:
            raise TypeError(f"no family's datasheet class is {type(datasheet).__name__}")
        model = cls(family, key)
        stated = {name: getattr(datasheet, name, None) for name in model.holds}
        return model.set_params(**{name: s.si for name, s in stated.items() if s is not None and s.is_number})

    def get_params(self, deep: bool = True) -> dict[str, float]:
        """Every value held, {canonical name: SI value}, stated and fitted alike (`deep` is sklearn's signature).
        WHY not sklearn's hyperparameters-vs-coef_ split: these values ARE what MuJoCo or sim2real is handed."""
        return {name: self.values[name] for name in self.holds if name in self.values}

    def set_params(self, **values: float) -> Model:
        """By any name the type knows: an alias lands on its canonical name, in SI when the alias carries a unit."""
        for name, value in values.items():
            canonical, alias = self.family.ladder.canonical(name), self.family.ladder.alias(name)
            if canonical not in self.holds:
                raise KeyError(f"{self.key} does not hold {canonical!r}; it holds {', '.join(self.holds)}")
            self.values[canonical] = to_si(float(value), alias.unit) if alias and alias.unit else float(value)
        return self

    def predict(self, X) -> np.ndarray | float:
        """tau_r [N·m] per sample. X is a log ({input: column}) or, for a one-input rung, the speeds v [rad/s]."""
        if self._declared():
            raise NotRunnable(f"{self.key}: {self._declared()}")
        missing = [name for name in self.needs if name not in self.values]
        if missing:
            raise ValueError(f"{self.key} has no value for {', '.join(missing)}: fit it on a log, or set_params(...)")
        return self._torque(self.values, X if isinstance(X, Mapping) else {self.inputs[0]: X})

    def fit(self, log: Mapping[str, np.ndarray]) -> Model:
        """Least squares on the log for `free`, from the values held; returns self, as sklearn's fit does."""
        if self.why_not(log):
            raise NotRunnable(f"{self.key} cannot fit this log: {self.why_not(log)}")
        tau, free = np.asarray(log[TARGET], float), self.free
        if free:
            def residuals(x):
                return self._torque({**self.values, **dict(zip(free, x))}, log) - tau
            tries = [found for start in self._starts(log) if (found := _least_squares(residuals, start))]
            if not tries:
                raise RuntimeError(f"{self.key}: no starting point converged on this log")
            best = min(tries, key=lambda found: found[1])[0]
            # WHY: Stribeck's speed enters squared, so the log cannot tell ±stribeck_velocity apart; a value whose
            # sign changes nothing is reported positive, so two fits of one log (or a warm-started climb) agree.
            for i in np.flatnonzero(best < 0):
                flipped = best.copy()
                flipped[i] *= -1
                if np.allclose(residuals(flipped), residuals(best)):
                    best = flipped
            self.values.update(zip(free, map(float, best)))
        self.fitted_, self.n_samples_ = {name: self.values[name] for name in free}, len(tau)
        self.rms_ = self.rms(log)
        return self

    def rms(self, log: Mapping[str, np.ndarray]) -> float:
        """Root-mean-square residual [N·m] on any log: on held-out plateaus, the number model selection reads;
        `rms_` is the same on the log it was fitted on."""
        return float(np.sqrt(np.mean((self.predict(log) - np.asarray(log[TARGET], float)) ** 2)))

    def score(self, log: Mapping[str, np.ndarray]) -> float:
        """R², as sklearn's regressors score: 1 is exact, 0 no better than the log's mean."""
        tau = np.asarray(log[TARGET], float)
        return float(1.0 - np.sum((self.predict(log) - tau) ** 2) / np.sum((tau - tau.mean()) ** 2))

    # --- inside -----------------------------------------------------------------------------------------------------

    def _torque(self, values: Mapping[str, float], columns: Mapping) -> np.ndarray | float:
        module, definition = _physics(self.rung.code)
        canonical = self.family.ladder.canonical
        # WHY by the definition's own term names: model.json may spell a term by an alias (LuGre's sigma2).
        terms = {term: values[canonical(term)] for term in definition["terms"] if canonical(term) in values}
        arrays = np.broadcast_arrays(*(np.asarray(columns[name], float) for name in self.inputs))
        state, out = None, []
        # WHY one sample at a time: model.py is the stdlib reference backend, scalar by contract; carrying its state
        # to the next sample lets a dynamic rung (LuGre) run on a time series too.
        for sample in zip(*(array.ravel() for array in arrays)):
            inputs = dict(zip(self.inputs, map(float, sample)))
            torque, state = module.torque(terms, inputs, state, inputs.get("dt"))
            out.append(torque)
        result = np.asarray(out, float).reshape(arrays[0].shape)
        return float(result) if result.ndim == 0 else result

    def _starts(self, log):
        """A held value starts where it is; a missing one by its unit: a torque at the log's torque scale, a speed
        scanned across the log's speeds (WHY: the Stribeck speed is the nonlinear term, and one guess can miss its
        basin), anything else across six decades."""
        speeds = np.abs(np.asarray(log["v"], float))
        speeds = speeds[speeds > 0]
        scans = {"N·m": (float(np.median(np.abs(log[TARGET]))),),
                 "rad/s": tuple(np.geomspace(speeds.min(), speeds.max(), 7))}
        return itertools.product(*((self.values[name],) if name in self.values
                                    else scans.get(self.parameter(name).unit, tuple(np.geomspace(1e-3, 1e3, 7)))
                                    for name in self.free))

    def __repr__(self) -> str:
        fitted = f"{len(self.fitted_)} fitted on {self.n_samples_} samples" if self.n_samples_ else "not fitted"
        return f"<Model {self.key}: {len(self.get_params())} values, {fitted}>"

    def __str__(self) -> str:                    # a display at 4 digits; get_params() holds the full values
        def shown(name):
            return f"{name} {self.values[name]:.4g} {self.parameter(name).unit}".rstrip()
        head = (f"{self.key}: fitted on {self.n_samples_} samples, rms {self.rms_:.3g} N·m" if self.n_samples_
                else f"{self.key}: not fitted")
        stated = [shown(name) for name in self.get_params() if name not in self.fitted_]
        missing = [name for name in self.needs if name not in self.values]
        return "\n".join([head, *(f"  fitted   {shown(name)}" for name in self.fitted_),
                          *([f"  stated   {', '.join(stated)}"] if stated else []),
                          *([f"  missing  {', '.join(missing)}"] if missing else [])])


def _least_squares(residuals, start, iterations: int = 200):
    """Levenberg–Marquardt (Marquardt 1963) with a forward-difference Jacobian, numpy only: (x, cost) or None.
    A step whose physics fails or goes non-finite (a Stribeck speed at 0) is rejected like one that costs more."""
    def attempt(x):
        try:
            r = residuals(x)
        except (ZeroDivisionError, OverflowError, ValueError, TypeError):
            return None
        return r if np.all(np.isfinite(r)) else None

    x, r = np.asarray(start, float), attempt(np.asarray(start, float))
    if r is None:
        return None
    cost, damping = float(r @ r), 1e-3
    for _ in range(iterations):
        steps = 1e-7 * np.maximum(np.abs(x), 1e-3)
        nudged = (attempt(x + np.eye(len(x))[i] * steps[i]) for i in range(len(x)))
        jacobian = np.column_stack([np.zeros_like(r) if column is None else (column - r) / step
                                    for column, step in zip(nudged, steps)])
        normal, gradient = jacobian.T @ jacobian, jacobian.T @ r
        while damping < 1e12:
            trial = x - np.linalg.solve(normal + damping * np.diag(np.diag(normal) + 1e-12), gradient)
            r_trial = attempt(trial)
            if r_trial is not None and float(r_trial @ r_trial) < cost:
                break
            damping *= 10
        else:
            return x, cost                       # no step lowers the cost: a minimum
        gain, x, r = cost - float(r_trial @ r_trial), trial, r_trial
        cost, damping = float(r @ r), damping / 10
        if gain <= 1e-12 * cost:
            break
    return x, cost


class Family:
    """A type's models, organized as its ladder: browsable as a tree, and the search order its climb walks."""

    def __init__(self, ladder: Ladder):
        self.ladder = ladder

    type = property(lambda self: self.ladder.type)
    keys = property(lambda self: self.ladder.keys)

    def model(self, key: str, **values: float) -> Model:
        return Model(self, key, **values)

    def models(self) -> dict[str, Model]:
        return {key: Model(self, key) for key in self.keys}

    canonical = property(lambda self: self.ladder.canonical)        # an alias resolves to the type's own name

    def children(self, key: str | None) -> tuple[str, ...]:
        return tuple(rung.key for rung in self.ladder.rungs if key is not None and rung.base == key)

    def siblings(self, key: str) -> tuple[str, ...]:
        return tuple(other for other in self.children(self.ladder.rung(key).base) if other != key)

    def __str__(self) -> str:
        def walk(key, connector, indent):
            run = [key]                          # a straight run of single children reads on one line
            while len(self.children(run[-1])) == 1:
                run.append(self.children(run[-1])[0])
            lines.append(connector + " → ".join(k + ("°" if self.model(k).why_not() else "") for k in run))
            children = self.children(run[-1])
            for i, child in enumerate(children):
                last = i == len(children) - 1
                walk(child, indent + ("└── " if last else "├── "), indent + ("    " if last else "│   "))

        lines = [f"{self.type}: {len(self.keys)} models (a → b: b extends a; ° declared, not run here)"]
        for root in (rung.key for rung in self.ladder.rungs if rung.base is None):
            walk(root, "", "")
        return "\n".join(lines)

    def climb(self, datasheet, fit: Mapping[str, np.ndarray], test: Mapping[str, np.ndarray],
              start: str = "M1.datasheet") -> Climb:
        """Model selection up the ladder, on held-out data: every rung is fitted on `fit` and judged on `test`.
        Stop at the first rung within FLOOR_LIMIT× the rig's noise floor (the lowest rung that reaches it); short of
        it, climb to the child with the lowest held-out rms, and only if that child beats the current rung there.
        WHY held out (Zach, 2026-11-17): "if doing a more complex model doesn't actually improve test time
        performance you are just overfitting"; a rung that nests the one below always fits its own training data at
        least as well, so a training score can never say no."""
        report = Climb(self.type, fit, test)
        current = report.chosen = Model.from_datasheet(datasheet, start).fit(fit)
        report.add(current, "start: the datasheet, nothing fitted")
        while True:
            reached = report.held_out[current.key] <= FLOOR_LIMIT * report.floor
            candidates = [Model.from_datasheet(datasheet, key) for key in self.children(current.key)]
            tried, held_out = [(candidate, candidate.why_not(fit)) for candidate in candidates], {}
            for candidate, reason in tried:
                if reason is None:
                    # WHY warm-start from the rung below: its fit is the nearest model, so each step stays local
                    candidate.set_params(**{n: v for n, v in current.values.items() if n in candidate.holds}).fit(fit)
                    held_out[candidate.key] = candidate.rms(test)
            best = min(held_out, key=held_out.get, default=None)
            climbs = not reached and best is not None and held_out[best] < report.held_out[current.key]
            for candidate, reason in tried:
                verdict = (f"skipped: {reason}" if reason
                           else "lowest held-out → climb" if candidate.key == best and climbs
                           else f"not needed: {current.key} reaches the floor" if reached
                           else f"no better held-out than {current.key}" if candidate.key == best
                           else f"{best} is lower held-out")
                report.add(None if reason else candidate, verdict, key=candidate.key)
            if not climbs:
                report.stop = (f"the lowest rung within {FLOOR_LIMIT:g}× the noise floor on held-out plateaus" if reached
                               else f"short of the floor, and {best} does not beat it held out" if best
                               else "short of the floor, and nothing above it fits this log")
                return report
            current = report.chosen = report.fits[best]


def noise_floor(log: Mapping[str, np.ndarray]) -> float:
    """What the rig repeats to [N·m]: the pooled within-plateau standard deviation, i.e. the held-out rms of the one
    estimator no rung can beat, the one that knows each plateau's own mean (charged one value per plateau)."""
    if PLATEAU not in log:
        raise ValueError(f"the log has no {PLATEAU!r} ids: the noise floor is the spread within each held speed")
    tau = np.asarray(log[TARGET], float)
    ids, which = np.unique(np.asarray(log[PLATEAU]), return_inverse=True)
    means = np.bincount(which, tau) / np.bincount(which)
    return float(np.sqrt(np.sum((tau - means[which]) ** 2) / (len(tau) - len(ids))))


class Climb:
    """The climb's table: one row per rung it looked at, in the order it looked; the rung it chose and why it stopped."""

    def __init__(self, type_name: str, fit: Mapping[str, np.ndarray], test: Mapping[str, np.ndarray]):
        self.type, self.test, self.floor = type_name, test, noise_floor(test)
        self.n_fit, self.n_test = len(fit[TARGET]), len(test[TARGET])
        self.rows, self.fits, self.held_out, self.chosen, self.stop = [], {}, {}, None, ""   # fits: {key: Model}

    def add(self, model, verdict, key=None):
        if model is None:
            self.rows.append((key, "·", "·", "·", "·", verdict))
            return
        self.fits[model.key], self.held_out[model.key] = model, model.rms(self.test)
        self.rows.append((model.key, len(model.fitted_), f"{model.rms_:.5f}", f"{self.held_out[model.key]:.5f}",
                          f"{self.held_out[model.key] / self.floor:.2f}", verdict))

    def __str__(self) -> str:
        table = [("rung", "k", "fit rms", "held out", "÷ floor", "verdict"), *self.rows]
        return "\n".join([f"climb {self.type}: fit on {self.n_fit} samples, judged on {self.n_test} held out; rms in N·m, "
                          f"noise floor {self.floor:.5f}; climb while held out > {FLOOR_LIMIT:g}× floor",
                          *(f"  {r[0]:<21} {r[1]!s:>2} {r[2]:>8} {r[3]:>8} {r[4]:>7}  {r[5]}" for r in table),
                          f"chosen {self.chosen.key}: {self.stop}"])


@functools.cache
def families() -> dict[str, Family]:
    """{type: Family} for every type robot_models has a ladder for."""
    return {name: Family(ladder) for name, ladder in ladders().items()}


__all__ = ["Climb", "Family", "Model", "NotRunnable", "families", "noise_floor"]
