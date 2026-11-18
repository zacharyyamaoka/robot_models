"""``Metric``: torchmetrics' base class with numpy inside. A metric is its class + its frozen config; it holds state.

    metric = ErrorStat(statistic="rms", rows="moving")     # the constructor IS the config
    metric.update(target, measured)                        # a chunk of the current run (live), or the whole run
    metric.end_run()                                       # a run boundary: a dwell never spans two runs
    metric.compute()                                       # the value so far (SI); never mutates the state
    metric.reset()
    metric(target, measured)                               # torchmetrics' forward: this run's own value, and accumulate
    a.merge_state(b)                                       # pool two metrics' states (a rung from its runs, a station ...)

State is declared like torchmetrics' ``add_state(name, default, dist_reduce_fx)``: ``"sum"``, ``"max"``, ``"min"`` or
``"cat"``; ``merge_state`` reduces each state by its rule, so a station's metric is its rungs' metrics merged and every
pooled statistic (the median of ALL stops) is exact at every level. WHY sufficient statistics and not a buffer of
samples: a live servo runs for hours; RMS needs (sum of squares, count), peak a running max, and the stop rule a
tracker that carries the open dwell across chunk boundaries (stock.StopTracker). Only what a statistic cannot reduce
(a median, the lag's cross-correlation) keeps a "cat" list, and it says so in its ``memory`` attribute.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from collections.abc import Callable
from typing import Any, ClassVar

import numpy as np

REDUCE: dict[str, Callable[[Any, Any], Any]] = {
    "sum": lambda a, b: a + b,
    "max": max,
    "min": min,
    "cat": lambda a, b: [*a, *b],
}


class Metric:
    """The base: what a metric IS (name, description, references, value type) on the class; its config on the instance."""

    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    references: ClassVar[tuple[str, ...]] = ()
    value_type: ClassVar[str] = "number"          # Zach: "a metric can be a bool type or like a value type"
    higher_is_better: ClassVar[bool | None] = False
    unit: ClassVar[str] = "signal"                # SI of the signal it is given; display units belong to the alias
    hint: ClassVar[str] = "kpi_trend"             # the data-ui view (PEP 0010 §10) its result is shown in
    memory: ClassVar[str] = "O(1)"                # what the live state grows with
    Input: ClassVar[str] = "TrackedPair"          # what update() takes: (target, measured) of one recording's signal
    live: ClassVar[bool] = True                   # can run on chunks as they arrive (the prior-art note's live flag)
    Config: ClassVar[type]                        # a frozen dataclass: the constructor's arguments

    def __init__(self, **config: Any) -> None:
        self.config = self.Config(**config)       # WHY a frozen dataclass: the identity is (class, config), hashable
        self._defaults: dict[str, Any] = {}
        self._reduce: dict[str, str] = {}
        self.updates = 0                          # how many update() calls this state has taken (the dedup counter)
        self.setup()

    # ---- torchmetrics' contract ----------------------------------------------------------------------------------
    def setup(self) -> None:
        """Declare the states with add_state (torchmetrics does it in __init__)."""

    def add_state(self, name: str, default: Any, reduce: str) -> None:
        if reduce not in REDUCE:
            raise ValueError(f"{type(self).__name__}.{name}: reduce is one of {sorted(REDUCE)}")
        self._defaults[name], self._reduce[name] = default, reduce
        setattr(self, name, copy.deepcopy(default))

    def update(self, target: Any, measured: Any) -> None:
        self.updates += 1
        self._update(target, measured)

    def _update(self, target: Any, measured: Any) -> None:
        raise NotImplementedError

    def end_run(self) -> None:
        """A run boundary. Metrics with per-run transient state (an open dwell) flush it here."""

    def compute(self) -> float | bool | None:
        raise NotImplementedError

    def reset(self) -> None:
        for name, default in self._defaults.items():
            setattr(self, name, copy.deepcopy(default))
        self.updates = 0

    def forward(self, target: Any, measured: Any) -> float | bool | None:
        """This run's own value, and the run accumulated into the state (torchmetrics' forward)."""

        alone = self.clone()
        alone.update(target, measured)
        alone.end_run()
        self.update(target, measured)
        self.end_run()
        return alone.compute()

    __call__ = forward

    def merge_state(self, other: Metric) -> Metric:
        if other.identity != self.identity:
            raise ValueError(f"cannot merge {other.identity} into {self.identity}")
        other_open = copy.deepcopy(other)
        other_open.end_run()
        for name, rule in self._reduce.items():
            setattr(self, name, REDUCE[rule](getattr(self, name), getattr(other_open, name)))
        return self

    def clone(self) -> Metric:
        return type(self)(**dataclasses.asdict(self.config))

    # ---- identity and description --------------------------------------------------------------------------------
    @property
    def identity(self) -> tuple[str, tuple]:
        """What makes two metrics the same computation: the base class and its frozen config (an alias is a label)."""

        return type(self).__name__, dataclasses.astuple(self.config)

    def card(self) -> dict[str, Any]:
        config = ", ".join(f"{k}={v!r}" for k, v in dataclasses.asdict(self.config).items())
        return {"name": self.name, "base": type(self).__name__, "config": config, "description": self.description,
                "references": list(self.references), "value_type": self.value_type, "unit": self.unit,
                "hint": self.hint, "memory": self.memory}

    def __repr__(self) -> str:
        config = ", ".join(f"{k}={v!r}" for k, v in dataclasses.asdict(self.config).items())
        return f"{type(self).__name__}({config})"

    # ---- standard views --------------------------------------------------------------------------------------------
    def view(self, scale: float = 1.0, unit: str = "", title: str = "") -> Any:
        """The points behind the value, for plot() / W&B / Rerun (stock metrics override with their own breakdown)."""

        import _viz

        value = self.compute()
        return _viz.View(title=title or self.name, unit=unit, hint=self.hint, x=[0],
                         y=[float(value) * scale if value is not None else math.nan])

    def plot(self, ax: Any = None, *, scale: float = 1.0, unit: str = "", title: str = "", path: Any = None) -> tuple:
        """torchmetrics' Metric.plot: (fig, ax) on matplotlib (the optional 'viz' extra)."""

        import _viz

        return _viz.plot(self.view(scale, unit, title), ax=ax, path=path)


def finite(values: Any) -> np.ndarray:
    array = np.asarray(values, float)
    return array[np.isfinite(array)]
