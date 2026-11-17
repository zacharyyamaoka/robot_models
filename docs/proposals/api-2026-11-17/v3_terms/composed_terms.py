"""Composed terms (v3): a Model is a sum of term models, one per slot; a ladder rung is a named preset of slot choices.

    m3 = servo["M2.coulomb_viscous"].redeclare(friction="stribeck")   # climbing = swapping one slot -> M3.stribeck
    m3_fit = m3.fit(fit_log, start=servo["M1.datasheet"].from_datasheet(sheet))
    compare(m2_fit, m3_fit, on=test_log).chosen      # does the swap pay on plateaus neither fit saw?

Borrowed: Modelica's replaceable components and `redeclare`, lmfit's composite models (`model_a + model_b`, one
parameter set, `fit` on the composite, a result that predicts), Drake's Diagram of LeafSystems (each term runs alone;
the sum is what you simulate). Every fact a term holds is read from today's v1 ladder and rung folders; a type adds
one table, which slot each of its rungs fills (servo_slots.py). Runtime: numpy + stdlib + robot_models.
"""

from __future__ import annotations

import functools
import importlib.util
import itertools
import json
import math
from pathlib import Path

import numpy as np

import robot_models
from robot_models.ladder import Ladder, Parameter

PACKAGE = Path(robot_models.__file__).resolve().parent
# WHY a level cap and not "has a model.py": the round's stop line runs M0-M3; M4.ripple has code but stays declared.
RUNNABLE_UP_TO = 3
GRID_POINTS, ZOOMS = 41, 4
EMPTY = "·"
# WHY 1.5 and this rule: sim2real's select() (cards/schema.py), so the rung chosen here is the one the twin would take
REACHES_FLOOR = 1.5


def _ordered(names) -> tuple[str, ...]:
    return tuple(dict.fromkeys(names))


@functools.cache
def _physics(path: Path):
    """A rung's own reference backend (robotparts' byte-pinned model.py), loaded by path: never retyped here."""
    spec = importlib.util.spec_from_file_location(f"rung_{path.parent.name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Term:
    """One physics law filling one slot: a small model of its own (an lmfit Model, a Drake LeafSystem)."""

    def __init__(self, family: Slots, ladder: Ladder, rung, slot: str, same_slot: list):
        self.family, self.slot, self.word, self.level = family, slot, rung.name, rung.level
        self.rank, self.title, self.equation = family.slots.index(slot), rung.title, rung.equation
        self.references, self.composes, self.wears = rung.references, rung.composes, rung.wears
        # WHY the same-slot chain: a term owns what its own slot's rungs add (stribeck's friction includes
        # coulomb_viscous's two), never another slot's (the inertia), so the presets add up to v1's rungs.
        self.needs = _ordered(name for step in reversed(same_slot) for name in step.adds)
        self.reads = tuple(name for name in _ordered(n for step in reversed(same_slot) for n in step.optional)
                           if name not in self.needs)
        self.falls_back_to = same_slot[1].name if len(same_slot) > 1 else None
        self.vocabulary = {name: ladder.parameter(name) for name in (*self.needs, *self.reads)}
        self.aliases = tuple(alias for alias in ladder.aliases if alias.canonical in self.vocabulary)
        folder = PACKAGE / rung.code if rung.code else None
        card = json.loads((folder / "model.json").read_text()) if folder and (folder / "model.json").exists() else {}
        self.code = folder / "model.py" if folder and (folder / "model.py").exists() else None
        # WHY dt is no input: it is the state update's step; predict is the steady-state map (dt=None).
        self.inputs = {name: spec["unit"] for name, spec in card.get("inputs", {}).items() if name != "dt"}
        # WHY spellings: model.py reads robotparts' term names, and one may be an alias (LuGre's sigma2).
        self.spellings = {name: name for name in self.vocabulary}
        for name in card.get("terms", {}):
            try:
                canonical = ladder.canonical(name)
            except KeyError:
                continue  # a packed MuJoCo array or a deployment setting: no single quantity of this term
            if canonical in self.vocabulary:
                self.spellings.setdefault(name, canonical)

    @property
    def runnable(self) -> bool:
        return self.code is not None and self.level <= RUNNABLE_UP_TO

    def __repr__(self) -> str:
        return f"{self.slot}:{self.word}"

    def __add__(self, other) -> Model:
        return Model((self,), self.family) + other

    def require_runnable(self) -> None:
        if not self.runnable:
            why = (f"its model.py exists, but this prototype runs M0-M{RUNNABLE_UP_TO} only" if self.code else
                   "robot_models holds its structure only, no runnable code")
            raise NotImplementedError(f"{self!r} (M{self.level}) is declared, not runnable: {why}")

    def torque(self, values: dict, inputs: dict) -> np.ndarray:
        """This term's resisting torque over the inputs (arrays broadcast), from the rung's own model.py."""
        self.require_runnable()
        own = {spelling: values[name] for spelling, name in self.spellings.items() if name in values}
        columns = np.broadcast_arrays(*(np.asarray(inputs[name], float) for name in self.inputs))
        # WHY tolist: the reference backends are stdlib; numpy scalars break their `(v > 0) - (v < 0)`
        samples = zip(*(column.ravel().tolist() for column in columns))
        physics = _physics(self.code)
        out = [physics.torque(own, dict(zip(self.inputs, sample)))[0] for sample in samples]
        return np.array(out, float).reshape(columns[0].shape)


class Model:
    """A sum of terms, at most one per slot: its parameters are the union of theirs, its torque the sum of theirs."""

    def __init__(self, terms, family: Slots):
        self.terms, self.family = tuple(sorted(terms, key=lambda term: term.rank)), family
        filled = [term.slot for term in self.terms]
        for slot in filled:
            if filled.count(slot) > 1:
                raise ValueError(f"two terms fill {slot}: a slot holds one option; swap it with .redeclare({slot}=...)")

    @property
    def words(self) -> tuple[tuple[str, str], ...]:
        return tuple((term.slot, term.word) for term in self.terms)

    @property
    def key(self) -> str | None:
        """The preset this composition is, if any: a ladder key names a composition; it does not define one."""
        return self.family.key_of(self)

    @property
    def label(self) -> str:
        return self.key or repr(self)

    def __repr__(self) -> str:
        return f"{self.key or 'off the ladder'} = {' + '.join(map(repr, self.terms)) or 'nothing'}"

    def __eq__(self, other) -> bool:
        return isinstance(other, Model) and self.words == other.words

    def __add__(self, other) -> Model:
        return Model((*self.terms, *(other.terms if isinstance(other, Model) else (other,))), self.family)

    def redeclare(self, **choices) -> Model:
        """Swap slots by name (Modelica's redeclare): friction="stribeck"; ripple=None empties a slot."""
        terms = {term.slot: term for term in self.terms}
        for slot, choice in choices.items():
            term = self.family.term(choice) if isinstance(choice, str) else choice
            if slot not in self.family.slots or (term is not None and term.slot != slot):
                raise ValueError(f"{choice} does not fill {slot!r}; the slots: {', '.join(self.family.slots)}")
            terms[slot] = term
        return Model([term for term in terms.values() if term is not None], self.family)

    def down(self) -> Model | None:
        """The preset one slot-step below: one term stepped back within its own slot, or its slot emptied."""
        landed = [m for m in (self.redeclare(**{t.slot: t.falls_back_to}) for t in self.terms) if m.key]
        if len(landed) > 1:
            raise ValueError(f"{self!r} steps down to {' or '.join(m.key for m in landed)}: name the one you mean")
        return landed[0] if landed else None

    def fallback(self) -> tuple[Model, ...]:
        """Itself, then each preset one slot-step down to the root: the ladder's fallback, derived from the slots."""
        chain = [self]
        while (lower := chain[-1].down()) is not None:
            chain.append(lower)
        return tuple(chain)

    def changes(self, other: Model) -> str:
        """What swapping from `other` to this one changes, slot by slot: "friction: coulomb_viscous → stribeck"."""
        before, after = dict(other.words), dict(self.words)
        return ", ".join(f"{slot}: {before.get(slot, EMPTY)} → {after.get(slot, EMPTY)}"
                         for slot in self.family.slots if before.get(slot) != after.get(slot))

    @property
    def parameters(self) -> tuple[str, ...]:
        """What it needs: the union of its terms' needs, slot by slot."""
        return _ordered(name for term in self.terms for name in term.needs)

    @property
    def optional(self) -> tuple[str, ...]:
        """What it reads when stated, less what it needs."""
        needed = set(self.parameters)
        return tuple(n for n in _ordered(n for term in self.terms for n in term.reads) if n not in needed)

    @property
    def inputs(self) -> dict[str, str]:
        return {name: unit for term in self.terms for name, unit in term.inputs.items()}

    @property
    def runnable(self) -> bool:
        return bool(self.terms) and all(term.runnable for term in self.terms)

    def canonical(self, name: str) -> str:
        """The type's own name for `name`, when this model holds it (one vocabulary per type; aliases resolve)."""
        for term in self.terms:
            if name in term.vocabulary:
                return name
            for alias in term.aliases:
                if alias.name == name:
                    return alias.canonical
        raise KeyError(f"{self.label} holds no {name!r} under any name")

    def parameter(self, name: str) -> Parameter:
        canonical = self.canonical(name)
        return next(term.vocabulary[canonical] for term in self.terms if canonical in term.vocabulary)

    def describe(self) -> str:
        """Each term's equation, what the model needs (unit, meaning), what it reads when stated, its references."""
        width = max(len(repr(term)) for term in self.terms)
        lines = [repr(self), *(f"  {term!r:<{width}}  {term.equation}" for term in self.terms)]
        names = max(map(len, self.parameters), default=0)
        units = max((len(self.parameter(name).unit) for name in self.parameters), default=0)
        for index, name in enumerate(self.parameters):
            unit, meaning = self.parameter(name).unit or "—", self.parameter(name).definition
            lines.append(f"  {'needs' if index == 0 else '':<5} {name:<{names}}  {unit:<{units}}  {meaning}")
        if self.optional:
            more = f", …, {self.optional[-1]}" if len(self.optional) > 3 else ""
            lines.append(f"  reads {len(self.optional)} more when stated (.optional): "
                         f"{', '.join(self.optional[:3])}{more}")
        references = _ordered(ref.title.split(". ")[0] for term in self.terms for ref in term.references)
        return "\n".join([*lines, "  refs  " + " · ".join(references)])

    def torque(self, values: dict, **inputs) -> np.ndarray:
        """The resisting torque at the output [N·m]: the sum of the terms' torques."""
        for term in self.terms:
            term.require_runnable()
        missing = [n for n in self.parameters if n not in values] + [n for n in self.inputs if n not in inputs]
        if missing:
            raise ValueError(f"{self.label} needs {', '.join(missing)}: state them, fit them or pass them")
        total = np.zeros(np.broadcast_shapes(*(np.shape(inputs[name]) for name in self.inputs)))
        for term in self.terms:
            total = total + term.torque(values, inputs)
        return total

    def from_datasheet(self, datasheet) -> Bound:
        """The model with what the datasheet states, in SI (Stated.si), under each canonical name it holds."""
        values = {name: stated.si for name in (*self.parameters, *self.optional)
                  if (stated := getattr(datasheet, name, None)) is not None and stated.is_number}
        missing = [name for name in self.parameters if name not in values]
        if missing:
            raise ValueError(f"{self.label}: the datasheet states no {', '.join(missing)}; fit them: .fit(log, ...)")
        return Bound(self, values, "the datasheet")

    def fit(self, log: dict, start=None) -> Bound:
        """Fit the composition to a log ({"v": array, ..., "tau": array}), numpy only.

        WHY on the Model, not a term (lmfit's CompositeModel.fit): the parameters are the union, so the composition
        meets the data. A need the log cannot move (the inertia, on velocity plateaus) is carried from `start`. The
        parameters the torque is linear in are solved by least squares at each point of a grid over the rest, and
        the grid zooms in on the best point (variable projection, Golub & Pereyra 1973).
        """
        base = dict(start.values if isinstance(start, Bound) else start or {})
        inputs, tau = {name: np.asarray(log[name], float) for name in self.inputs}, np.asarray(log["tau"], float)
        probe = {**base, **{name: base.get(name, 1.0) for name in self.parameters}}
        at_probe = self.torque(probe, **inputs)

        def nudged(name, sign=1.0):  # WHY ±half of itself: never through zero, where a law may divide by it
            value = probe[name] * (1 + 0.5 * sign) if probe[name] else 0.5 * sign
            return self.torque({**probe, name: value}, **inputs)

        free = [name for name in self.parameters if not np.array_equal(nudged(name), at_probe)]
        tolerance = 1e-9 * (1.0 + np.abs(at_probe).max())
        linear = [n for n in free if np.abs(nudged(n) + nudged(n, -1.0) - 2 * at_probe).max() <= tolerance]
        nonlinear = [name for name in free if name not in linear]
        if len(nonlinear) > 2:
            raise NotImplementedError(f"{self.label}: a grid over {', '.join(nonlinear)} is too big; fit fewer")

        def project(point):  # the linear parameters by least squares, the nonlinear ones held at `point`
            held = dict(zip(nonlinear, point))
            at = {**probe, **held, **dict.fromkeys(linear, 0.0)}
            offset, design = self.torque(at, **inputs), np.zeros((tau.size, len(linear)))
            for column, name in enumerate(linear):
                design[:, column] = self.torque({**at, name: 1.0}, **inputs) - offset
            solution = np.linalg.lstsq(design, tau - offset, rcond=None)[0]
            rss = float(np.sum((design @ solution + offset - tau) ** 2))
            return rss, {**held, **dict(zip(linear, solution.tolist()))}

        grids = [self._grid(name, probe[name], inputs) for name in nonlinear]
        for _ in range(ZOOMS if nonlinear else 1):  # WHY zoom: re-grid one old step either side of the best point
            rss, best = min((project(point) for point in itertools.product(*grids)), key=lambda pair: pair[0])
            grids = [np.geomspace(best[n] * g[0] / g[1], best[n] * g[1] / g[0], GRID_POINTS).tolist()
                     for n, g in zip(nonlinear, grids)]
        fitted = tuple(name for name in self.parameters if name in best)
        return Bound(self, {**base, **best}, f"a fit to {tau.size} samples", fitted, math.sqrt(rss / tau.size), tau)

    def _grid(self, name: str, guess: float, inputs: dict) -> list[float]:
        # WHY the log's own range: a parameter in an input's unit (stribeck_velocity, rad/s) lives where the log ran
        unit = self.parameter(name).unit
        spans = [np.abs(inputs[i][inputs[i] != 0]) for i, u in self.inputs.items() if u == unit]
        low, high = (min(s.min() for s in spans), max(s.max() for s in spans)) if spans else (guess / 100, guess * 100)
        return np.geomspace(low, high, GRID_POINTS).tolist()


class Bound:
    """A Model with values (lmfit's ModelResult): it predicts, hands its values on, and scores itself on a log."""

    def __init__(self, model: Model, values: dict, source: str, fitted=(), rms=None, trained_on=()):
        self.model, self.values, self.source = model, values, source
        self.fitted, self.rms, self.trained_on = tuple(fitted), rms, np.asarray(trained_on, float)
        self.samples = self.trained_on.size

    def predict(self, v, **inputs):
        out = self.model.torque(self.values, v=v, **inputs)
        return float(out) if out.ndim == 0 else out

    def score(self, log: dict) -> float:
        """The rms error [N·m] on a log. Pass one the fit never saw: a law nesting another fits its own log better."""
        inputs = {name: np.asarray(log[name], float) for name in self.model.inputs}
        return float(np.sqrt(np.mean((self.model.torque(self.values, **inputs) - np.asarray(log["tau"])) ** 2)))

    @property
    def params(self) -> dict[str, float]:
        """What the data set: the fitted values, SI, by canonical name (ready for MuJoCo or sim2real)."""
        return {name: self.values[name] for name in self.fitted}

    def __repr__(self) -> str:
        def show(names):
            return " · ".join(f"{n} {self.values[n]:.4g} {self.model.parameter(n).unit}".rstrip() for n in names)
        carried = [n for n in self.model.parameters if n in self.values and n not in self.fitted]
        stated = [n for n in self.model.optional if n in self.values]
        lines = [f"{self.model.label} from {self.source}" + (f" (fit rms {self.rms:.4g} N·m)" if self.fitted else "")]
        lines += [f"  fitted   {show(self.fitted)}"] if self.fitted else []
        lines += [f"  {'carried' if self.fitted else 'needs':<8} {show(carried)}"] if carried else []
        return "\n".join(lines + ([f"  stated   {show(stated)}"] if stated else []))


def noise_floor(log: dict) -> float:
    """What the rig repeats to [N·m]: the pooled within-plateau std of tau, observable from the log itself."""
    tau, plateau = np.asarray(log["tau"], float), np.asarray(log["plateau"])
    ids = np.unique(plateau)
    spread = sum(float(np.sum((tau[plateau == p] - tau[plateau == p].mean()) ** 2)) for p in ids)
    return math.sqrt(spread / (tau.size - ids.size))


class Comparison:
    """Fits of one log, judged on a held-out log, lowest first; each row names the slot it swapped.

    The chosen one is the lowest whose held-out rms ÷ the rig's noise floor ≤ REACHES_FLOOR: climbing swaps one slot,
    and a swap pays only when the rung below it misses the floor. WHY held out (Zach, 2026-11-17): "if doing a more
    complex model doesn't actually improve test time performance you are just overfitting"; a friction option that
    nests the one it replaces (stribeck holds coulomb_viscous) always wins on its own fit log.
    """

    def __init__(self, fits, on: dict):
        if len({fit.samples for fit in fits}) != 1 or any(fit.rms is None for fit in fits):
            raise ValueError("compare fits of one log: each a .fit(log) over the same samples")
        if any(np.isin(np.asarray(on["tau"], float), fit.trained_on).any() for fit in fits):
            raise ValueError("on= shares samples with the fits' own log: judge on plateaus the fits never saw")
        ladder = {key: index for index, key in enumerate(fits[0].model.family.keys())}
        # WHY fewest fitted parameters first, then ladder order: "lowest" must also rank a sum that no rung names
        self.fits = tuple(sorted(fits, key=lambda fit: (len(fit.fitted), ladder.get(fit.model.key, len(ladder)))))
        self.floor, self.samples, self.plateaus = noise_floor(on), len(on["tau"]), np.unique(on["plateau"]).size
        self.held_out = tuple(fit.score(on) for fit in self.fits)
        reached = [fit for fit, rms in zip(self.fits, self.held_out) if rms <= REACHES_FLOOR * self.floor]
        self.chosen = reached[0].model.label if reached else None

    def __str__(self) -> str:
        rows = [f"held out: {self.samples} samples on {self.plateaus} plateaus the fits never saw; rig noise floor "
                f"{self.floor:.5f} N·m (pooled within-plateau std)",
                f"{'model':<20} {'k':>2}  {'fit rms':>8}  {'held-out rms':>12}  {'÷ floor':>7}  swapped"]
        for previous, fit, held_out in zip((None, *self.fits), self.fits, self.held_out):
            change = fit.model.changes(previous.model) if previous else ""
            row = (f"{fit.model.label:<20} {len(fit.fitted):>2}  {fit.rms:>8.5f}  {held_out:>12.5f}  "
                   f"{held_out / self.floor:>7.2f}  {change}")
            rows.append(row + ("   ← chosen" if fit.model.label == self.chosen else ""))
        rows.append(f"chosen: the lowest whose held-out rms ÷ floor ≤ {REACHES_FLOOR}" if self.chosen else
                    f"none reaches {REACHES_FLOOR} × the floor: swap or fill another slot and fit again")
        return "\n".join(row.rstrip() for row in rows)


def compare(*fits: Bound, on: dict) -> Comparison:
    """Judge fits of one log on `on`, a held-out log (v, tau, plateau) from the same rig that none of them saw."""
    return Comparison(fits, on)


class Slots:
    """A type's slots, each slot's options (Terms) and its presets (named Models): its ladder, as compositions."""

    def __init__(self, ladder: Ladder, slot_of: dict[str, str]):
        self.type, self.ladder, self.slots = ladder.type, ladder, _ordered(slot_of.values())
        if {rung.name for rung in ladder.rungs} != set(slot_of):
            raise ValueError(f"{ladder.type}: slot_of names a slot for each rung, and only for its rungs")
        self._terms, self._presets = {}, {}
        for rung in ladder.rungs:
            chain = map(ladder.rung, ladder.fallback(rung.key))
            same = [step for step in chain if slot_of[step.name] == slot_of[rung.name]]
            self._terms[rung.name] = Term(self, ladder, rung, slot_of[rung.name], same)
        for rung in ladder.rungs:
            nearest = {}  # WHY nearest per slot: a rung's own choice wins over its bases' for that slot
            for step in map(ladder.rung, ladder.fallback(rung.key)):
                nearest.setdefault(slot_of[step.name], self._terms[step.name])
            self._presets[rung.key] = Model(nearest.values(), self).words
        self._keys = {words: key for key, words in self._presets.items()}
        for key in ladder.keys:  # WHY: the composed view must BE today's ladder, or it is a fork of it
            model = self[key]
            got = (model.parameters, model.optional, tuple(m.key for m in model.fallback()))
            if got != (ladder.parameters(key), ladder.optional(key), ladder.fallback(key)):
                raise ValueError(f"{self.type} {key}: the composed preset {got} differs from v1's rung")

    def keys(self) -> tuple[str, ...]:
        return tuple(self._presets)

    def __getitem__(self, key: str) -> Model:
        if key not in self._presets:
            raise KeyError(f"{self.type} has no preset {key!r}; its presets: {', '.join(self._presets)}")
        return Model([self._terms[word] for _, word in self._presets[key]], self)

    def term(self, word: str) -> Term:
        if word not in self._terms:
            raise KeyError(f"{self.type} has no term {word!r}; its terms: {', '.join(self._terms)}")
        return self._terms[word]

    def key_of(self, model: Model) -> str | None:
        return self._keys.get(model.words)

    def __str__(self) -> str:
        rows = {key: dict(words) for key, words in self._presets.items()}
        width = {slot: max(len(slot), *(len(row.get(slot, EMPTY)) for row in rows.values())) for slot in self.slots}
        pad = max(map(len, rows)) + 1
        lines = [f"{self.type:<{pad + 1}} " + "  ".join(f"{slot:<{width[slot]}}" for slot in self.slots)]
        for key, row in rows.items():
            mark = "*" if self[key].runnable else " "
            lines.append(f"{key:<{pad}}{mark} " + "  ".join(f"{row.get(s, EMPTY):<{width[s]}}" for s in self.slots))
        return "\n".join(line.rstrip() for line in lines)
