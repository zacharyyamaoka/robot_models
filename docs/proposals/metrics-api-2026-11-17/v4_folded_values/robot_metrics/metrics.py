"""The stock metrics of the v4 core: math only, each a frozen config whose state is a value.

    ErrorStat(statistic="rms", rows="moving")          state (sum e^2, n, max |e|)
    Dwell(of="error", statistic="median")             state DwellState(closed stops, the open dwell's tracker)
    TimeOffset()                                      state (per-run offsets, the open run's chunks)   live=False
    CodeShare(codes=(97, 98, 99, 102))                state (hits, rows)
    Repeatability(k=3)        Input Repeat            state (command, per-repeat stop errors)
    PerformanceGain(...)      Input HeldOutPair       state (nominal, fitted)

Formulas are robotdx.diagnostics.stock's (via robot_msgs.compare and the shared stop rule), with one change RobotDx
needs (the prior-art note): no ``limits`` field in any config, so a new target never changes a metric's identity.
"""

from __future__ import annotations

import copy
import math
import statistics
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from robot_msgs.compare import compare, estimate_time_offset, goal
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory, concatenate

import stock

from .core import Metric, check_aligned

ROBOTDX = "robotdx.diagnostics.stock (io-diagnostics main a7fc658)"


def _error(target: Any, measured: Any, field_name: str, joint: int, rows: str) -> np.ndarray:
    error = compare(target, measured).error
    values = (np.linalg.norm(error.positions, axis=1) if isinstance(error, PoseTrajectory)
              else np.asarray(getattr(error, field_name)[:, joint], float))
    if rows == "moving" and isinstance(target, JointTrajectory) and target.velocities is not None:
        values = values[stock.moving(target, joint)]
    return values[np.isfinite(values)]


@dataclass(frozen=True)
class ErrorStat(Metric):
    statistic: str = "rms"
    rows: str = "all"
    field: str = "positions"
    joint: int = 0
    name = "error statistic"
    description = "One statistic of target - measured over the rows it reads (rms, max_abs, mean_abs)."
    references = ("robot_msgs.compare.ErrorStats", ROBOTDX + " tracking_error", "evo APE")

    def empty(self) -> tuple[float, int, float, float]:
        return (0.0, 0, 0.0, 0.0)

    def update(self, state: tuple, target: Any, measured: Any) -> tuple:
        check_aligned(self, target, measured)
        error = _error(target, measured, self.field, self.joint, self.rows)
        return (state[0] + float(np.sum(error * error)), state[1] + int(error.size),
                max(state[2], float(np.max(np.abs(error)))) if error.size else state[2],
                state[3] + float(np.sum(np.abs(error))))

    def merge(self, a: tuple, b: tuple) -> tuple:
        return (a[0] + b[0], a[1] + b[1], max(a[2], b[2]), a[3] + b[3])

    def compute(self, state: tuple) -> float | None:
        if not state[1]:
            return None
        return {"rms": math.sqrt(state[0] / state[1]), "max_abs": state[2], "mean_abs": state[3] / state[1]}[self.statistic]


@dataclass(frozen=True)
class DwellState:
    closed: tuple = ()
    tracker: Any = None     # stock.StopTracker of the open run, copied on every update (the state stays a value)


@dataclass(frozen=True)
class Dwell(Metric):
    of: str = "error"            # error | settle | settled | overshoot | count
    statistic: str = "median"    # median | max | p90 | all | count
    band: float = math.inf
    include_last_dwell: bool = True
    joint: int = 0
    name = "dwell"
    description = "A quantity of every stop (dwell >= 0.25 s at zero commanded speed that a move led into), pooled."
    references = ("bam_rack_twin T4", ROBOTDX + " stop_error", "robot_msgs.compare.goal")
    hint = "distribution"

    @property
    def value_type(self) -> str:  # type: ignore[override]
        return "bool" if self.of == "settled" else "number"

    @property
    def rule(self) -> tuple:
        return ("Dwell", self.include_last_dwell, self.joint)

    def empty(self) -> DwellState:
        return DwellState()

    def update(self, state: DwellState, target: Any, measured: Any) -> DwellState:
        check_aligned(self, target, measured)
        tracker = copy.deepcopy(state.tracker) if state.tracker is not None else stock.StopTracker(
            joint=target.joint_names[self.joint], include_last_dwell=self.include_last_dwell)
        j = self.joint
        found = tracker.push_many(measured.t, measured.positions[:, j], target.positions[:, j], target.velocities[:, j])
        return DwellState(state.closed + found, tracker)

    def close(self, state: DwellState) -> DwellState:
        if state.tracker is None:
            return state
        return DwellState(state.closed + copy.deepcopy(state.tracker).finish(), None)

    def merge(self, a: DwellState, b: DwellState) -> DwellState:
        return DwellState(self.close(a).closed + self.close(b).closed, None)

    def items(self, state: DwellState) -> list[float]:
        found = self.close(state).closed
        if self.of == "error":
            return [abs(stop.error) for stop in found]
        if self.of == "settle":
            return [stock.settle_time(stop, self.band) for stop in found]
        if self.of == "settled":
            return [float(math.isfinite(stock.settle_time(stop, self.band))) for stop in found]
        if self.of == "overshoot":
            return [goal(*stop.move(), band=math.inf)[f"positions/{stop.joint}"].overshoot for stop in found]
        return [1.0 for _ in found]

    def compute(self, state: DwellState) -> Any:
        values = self.items(state)
        if self.statistic == "count":
            return float(len(values))
        if self.statistic == "all":
            return all(value == 1.0 for value in values)
        if not values:
            return 0.0 if self.of == "overshoot" else None
        if self.statistic == "median":
            middle = statistics.median(values)
            return None if math.isinf(middle) else float(middle)
        if self.statistic == "max":
            kept = [v for v in values if math.isfinite(v)]
            return max(kept) if kept else None
        return float(np.percentile(values, 90.0))


@dataclass(frozen=True)
class TimeOffset(Metric):
    max_offset_s: float = 0.25
    name = "time offset"
    description = "How late the response runs behind the command (estimate_time_offset), the median over runs."
    references = ("robot_msgs.compare.estimate_time_offset",)
    Input = "AlignedPair (the plan on its ticks, the state)"
    live = False
    align_tolerance_s = math.inf    # WHY: it is the step that finds the clock offset, so it cannot demand aligned input

    def empty(self) -> tuple:
        return ((), ())

    def update(self, state: tuple, target: Any, measured: Any) -> tuple:
        return (state[0], state[1] + ((target, measured),))

    def close(self, state: tuple) -> tuple:
        if not state[1]:
            return state
        target = concatenate([c[0] for c in state[1]]) if len(state[1]) > 1 else state[1][0][0]
        measured = concatenate([c[1] for c in state[1]]) if len(state[1]) > 1 else state[1][0][1]
        return (state[0] + (estimate_time_offset(target, measured, max_offset_s=self.max_offset_s).offset_s,), ())

    def merge(self, a: tuple, b: tuple) -> tuple:
        return (self.close(a)[0] + self.close(b)[0], ())

    def compute(self, state: tuple) -> float | None:
        offsets = self.close(state)[0]
        return float(statistics.median(offsets)) if offsets else None


@dataclass(frozen=True)
class CodeShare(Metric):
    codes: tuple[int, ...] = (97, 98, 99, 102)
    channel: str = "fault"
    name = "code share"
    description = "The share of rows whose code channel holds one of the codes."
    references = ("moteus fault and limiter codes", ROBOTDX + " fault_codes")
    Input = "AlignedPair (the state's code channel)"

    def empty(self) -> tuple[int, int]:
        return (0, 0)

    def update(self, state: tuple, target: Any, measured: Any) -> tuple:
        codes = np.asarray(measured.channels[self.channel]).astype(int)
        return (state[0] + int(np.isin(codes, self.codes).sum()), state[1] + int(codes.size))

    def merge(self, a: tuple, b: tuple) -> tuple:
        return (a[0] + b[0], a[1] + b[1])

    def compute(self, state: tuple) -> float | None:
        return state[0] / state[1] if state[1] else None


@dataclass(frozen=True)
class Repeat:
    command: tuple
    target: Any
    measured: Any


@dataclass(frozen=True)
class Repeatability(Metric):
    k: float = 3.0
    include_last_dwell: bool = True
    name = "repeatability"
    description = "ISO 9283 RP at each stop over same-input repeats: mean(l) + k S_l, the worst stop index."
    references = ("ISO 9283:1998 §7.2.2", ROBOTDX + " pose_accuracy")
    Input = "Repeat"
    live = False

    def empty(self) -> tuple:
        return (None, ())

    def update(self, state: tuple, repeat: Repeat) -> tuple:  # type: ignore[override]
        if state[0] is not None and repeat.command != state[0]:
            raise ValueError(f"{repeat.command} is not a repeat of {state[0]}")
        errors = tuple(stop.error for stop in stock.stops(repeat.target, repeat.measured,
                                                          include_last_dwell=self.include_last_dwell))
        return (repeat.command, state[1] + (errors,))

    def merge(self, a: tuple, b: tuple) -> tuple:
        if a[0] is not None and b[0] is not None and a[0] != b[0]:
            raise ValueError("repeats of two commands do not merge")
        return (a[0] or b[0], a[1] + b[1])

    def compute(self, state: tuple) -> float | None:
        errors = state[1]
        if len(errors) < 2 or len({len(e) for e in errors}) != 1:
            return None
        matrix = np.asarray(errors)
        distance = np.abs(matrix - matrix.mean(axis=0))
        return float(np.max(distance.mean(axis=0) + self.k * distance.std(axis=0, ddof=1)))


@dataclass(frozen=True)
class HeldOutPair:
    nominal: tuple
    fitted: tuple


@dataclass(frozen=True)
class PerformanceGain(Metric):
    """The alias's metric folded over held-out runs with nominal values and with fitted ones; nominal - fitted.
    Its identity carries the alias's metric, so a gain on an alias IS that alias's computation, twice."""

    metric: Metric = field(default_factory=lambda: ErrorStat())
    signal: tuple[str, str] = ("target", "state")
    name = "performance gain"
    description = "A metric on held-out runs with nominal and with fitted values; nominal minus fitted."
    references = ("hold-out validation", "robot_models select_best")
    Input = "HeldOutPair"
    live = False

    def empty(self) -> tuple:
        return (None, None)

    def update(self, state: tuple, pair: HeldOutPair) -> tuple:  # type: ignore[override]
        sides = []
        for runs in (pair.nominal, pair.fitted):
            folded = self.metric.empty()
            for run in runs:
                folded = self.metric.close(self.metric.update(folded, run.streams[self.signal[0]],
                                                              run.streams[self.signal[1]]))
            sides.append(self.metric.compute(folded))
        return tuple(sides)

    def merge(self, a: tuple, b: tuple) -> tuple:
        raise TypeError("a gain is one comparison; merge its sides' runs instead")

    def compute(self, state: tuple) -> float | None:
        return None if state[0] is None else state[0] - state[1]
