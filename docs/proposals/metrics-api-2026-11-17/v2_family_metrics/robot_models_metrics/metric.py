"""``robot_models.metrics``: torchmetrics' shape where the cat buffer is the state and the functional twin is the math.

    metric = ErrorStat(statistic="rms", rows="moving", across_runs="median")   # the constructor is the config
    metric.update(target, measured)     # appends the chunk to the open run (live), or the whole run at once
    metric.end_run()                    # closes the run: the next update starts another
    metric.compute()                    # = ErrorStat.functional(config, runs): RobotDx's (config, runs) signature
    metric.reset()

WHY a buffer and the functional at compute (torchmetrics' "cat" states, as its AUROC keeps every score): the class can
never disagree with its functional twin, a stop that spans a chunk boundary is simply in the buffer, and a dataset is
one object fed run after run (the across_runs rule is the constructor's, RobotDx's ``TrackingErrorConfig.across_runs``).
The price is memory: O(rows of the runs it holds), about 30 kB per pendulum play, and compute() re-reads the buffer;
a monitor that runs for hours would need a window (not built here). WHY a memo on compute: two aliases of one identity (base + config) on the same data
are one computation (Zach's "prevent a lot of duplicate work"); the memo key is (identity, which data).
"""

from __future__ import annotations

import dataclasses
from collections.abc import Callable, Sequence
from typing import Any, ClassVar

from robot_msgs.trajectory import concatenate


class Memo:
    """compute() results by (identity, data key): the dedup counter the scenario prints."""

    def __init__(self) -> None:
        self.values: dict[tuple, Any] = {}
        self.hits = 0
        self.misses = 0

    def get(self, key: tuple, make: Callable[[], Any]) -> Any:
        if key in self.values:
            self.hits += 1
            return self.values[key]
        self.misses += 1
        self.values[key] = value = make()
        return value

    def clear(self) -> None:
        self.values.clear()
        self.hits = self.misses = 0


MEMO = Memo()


class Metric:
    name: ClassVar[str] = ""
    description: ClassVar[str] = ""
    references: ClassVar[tuple[str, ...]] = ()
    value_type: ClassVar[str] = "number"
    hint: ClassVar[str] = "kpi_trend"
    Input: ClassVar[str] = "TrackedPair"   # what update() takes: (target, measured) of one episode's signal
    live: ClassVar[bool] = True
    Config: ClassVar[type]

    def __init__(self, **config: Any) -> None:
        self.config = self.Config(**config)
        self.runs: list[tuple[str, list[tuple[Any, Any]]]] = []   # (data key, chunks) per run; the last may be open
        self._open = False

    # ---- the math: one pure function of (config, runs) ------------------------------------------------------------
    @classmethod
    def functional(cls, config: Any, runs: Sequence[tuple[Any, Any]]) -> Any:
        """Each run a (target, measured) pair of robot_msgs trajectories; the across-run rule is the config's."""

        raise NotImplementedError

    # ---- torchmetrics' contract ------------------------------------------------------------------------------------
    def update(self, target: Any, measured: Any, *, key: str = "") -> None:
        if not self._open:
            self.runs.append((key, []))
            self._open = True
        self.runs[-1][1].append((target, measured))

    def end_run(self) -> None:
        self._open = False

    def reset(self) -> None:
        self.runs, self._open = [], False

    def _joined(self) -> list[tuple[Any, Any]]:
        out = []
        for _, chunks in self.runs:
            if len(chunks) == 1:
                out.append(chunks[0])
            else:
                out.append((concatenate([c[0] for c in chunks]), concatenate([c[1] for c in chunks])))
        return out

    def data_key(self) -> tuple:
        """Which data: each run's key, rows and chunk count. WHY the chunk count: a live run streamed in chunks must be
        recomputed from its own buffer, never served the offline value from the memo."""

        return tuple((key, sum(len(c[1]) for c in chunks), len(chunks)) for key, chunks in self.runs)

    def compute(self) -> Any:
        if not self.runs:
            return None
        return MEMO.get((self.identity, self.data_key()), lambda: self.functional(self.config, self._joined()))

    def __call__(self, target: Any, measured: Any) -> Any:
        """One recording alone, offline (torchmetrics' functional call on a fresh state)."""

        return type(self).functional(self.config, [(target, measured)])

    def clone(self) -> Metric:
        return type(self)(**dataclasses.asdict(self.config))

    @property
    def identity(self) -> tuple[str, tuple]:
        return type(self).__name__, dataclasses.astuple(self.config)

    def card(self) -> dict[str, Any]:
        config = ", ".join(f"{k}={v!r}" for k, v in dataclasses.asdict(self.config).items())
        return {"base": type(self).__name__, "config": config, "description": self.description,
                "references": list(self.references), "value_type": self.value_type, "hint": self.hint}

    def __repr__(self) -> str:
        config = ", ".join(f"{k}={v!r}" for k, v in dataclasses.asdict(self.config).items())
        return f"{type(self).__name__}({config})"
