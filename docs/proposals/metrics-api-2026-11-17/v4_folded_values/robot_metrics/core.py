"""The v4 core: a metric is a frozen config, its state a VALUE; live is a fold over chunks, offline the same fold once.

    rms = ErrorStat(statistic="rms", rows="moving")      # a frozen dataclass: identity = class + fields, hashable
    state = rms.empty()
    for target, measured in chunks:                      # live
        state = rms.update(state, target, measured)      # returns a NEW state; nothing hidden is mutated
    rms.compute(state)
    rms(target, measured)                                # offline: one update of the whole recording, from empty()
    rms.merge(a, b)                                      # two runs, two rungs, two stations: one state

torchmetrics' stateful face (``update`` / ``compute`` / ``reset`` on one object) is ``Stream(metric)``, ten lines over
this core. WHY the state as a value (clu, torcheval; the prior-art note §1): merging runs needs no extra code, a state
can be stored and resumed, and nothing differs between live and offline but where the fold's chunks come from. "Live
== offline" holds within 1e-9 (the contract's G4): a sum folded in another order may round differently.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any, ClassVar


@dataclass(frozen=True)
class Metric:
    """The base: what it is on the class, its config as fields. Subclasses define empty / update / merge / compute."""

    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    references: ClassVar[tuple[str, ...]] = ()
    value_type: ClassVar[str] = "number"
    unit: ClassVar[str] = "signal"        # the SI of the signal it is given; display units belong to the alias
    hint: ClassVar[str] = "kpi_trend"     # RobotDx's data-ui view hint
    Input: ClassVar[str] = "AlignedPair"  # what update() takes: (target, measured) on the same instants
    live: ClassVar[bool] = True           # a per-metric declaration: can it run on chunks as they arrive
    version: ClassVar[str] = "1"          # stored with every value it produces (the prior-art note's addition)
    align_tolerance_s: ClassVar[float] = 0.0   # the explicit alignment check: target and measured share instants

    def empty(self) -> Any:
        raise NotImplementedError

    def update(self, state: Any, target: Any, measured: Any) -> Any:
        raise NotImplementedError

    def merge(self, a: Any, b: Any) -> Any:
        raise NotImplementedError

    def compute(self, state: Any) -> Any:
        raise NotImplementedError

    def close(self, state: Any) -> Any:
        """A run boundary (a dwell never spans two runs); most states have nothing to close."""

        return state

    def items(self, state: Any) -> list[float]:
        """What an occurrence gate counts: one value per run by default (stops, rows for some metrics)."""

        value = self.compute(state)
        return [] if value is None else [float(value)]

    def __call__(self, target: Any, measured: Any) -> Any:
        return self.compute(self.close(self.update(self.empty(), target, measured)))

    def fold(self, pairs: Iterable[tuple[Any, Any]]) -> Any:
        state = self.empty()
        for target, measured in pairs:
            state = self.update(state, target, measured)
        return self.close(state)

    def card(self) -> dict[str, Any]:
        config = ", ".join(f"{f.name}={getattr(self, f.name)!r}" for f in dataclasses.fields(self))
        return {"base": type(self).__name__, "config": config, "description": self.description,
                "references": list(self.references), "value_type": self.value_type, "input": self.Input,
                "live": self.live, "version": self.version, "hint": self.hint}


def check_aligned(metric: Metric, target: Any, measured: Any) -> None:
    import numpy as np

    if len(target) != len(measured) or not np.allclose(target.t, measured.t, rtol=0.0, atol=metric.align_tolerance_s):
        raise ValueError(f"{type(metric).__name__}: target and measured are not on the same instants "
                         f"(tolerance {metric.align_tolerance_s} s); align them first (the data adapter's step)")


class Stream:
    """torchmetrics' stateful face over a value-state metric: update / compute / reset, and the state to store."""

    def __init__(self, metric: Metric) -> None:
        self.metric = metric
        self.state = metric.empty()

    def update(self, target: Any, measured: Any) -> None:
        self.state = self.metric.update(self.state, target, measured)

    def end_run(self) -> None:
        self.state = self.metric.close(self.state)

    def compute(self) -> Any:
        return self.metric.compute(self.metric.close(self.state))

    def reset(self) -> None:
        self.state = self.metric.empty()
