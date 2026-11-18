"""Diagnostics and gates: a metric's value judged against Limits, outside compute().

    Diagnostic("tip stop error", Limits.at_most(1.0), owner="Zach 2026-11-11")   # the alias's display unit (mm)
    Gate("tip stop", (Diagnostic(...), Diagnostic(...)))                          # every member passes
    gate.judge(values) -> Verdict("pass" | "fail" | "not_run", the failing members)

WHY Limits and judge are RobotDx's (robotdx/diagnostics/core.py, copied here with its rule unchanged: strictly beyond
a bound trips it; no value is "not_run", never green): Zach's "separate the calculation of the metric, and the
checking of thresholds", so a target changes without recomputing anything. A Diagnostic is metric + threshold; a gate
is a diagnostic someone owns as a target ("Zach 2026-11-11" for the 1 mm tip, "agent" for pendulum_goal's numbers).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

PASS, FAIL, NOT_RUN = "pass", "fail", "not_run"


@dataclass(frozen=True)
class Limits:
    low: float | None = None
    high: float | None = None

    @classmethod
    def at_most(cls, high: float) -> Limits:
        return cls(high=high)

    @classmethod
    def at_least(cls, low: float) -> Limits:
        return cls(low=low)

    @classmethod
    def is_true(cls) -> Limits:
        return cls(low=1.0, high=1.0)

    def __str__(self) -> str:
        if self.low == self.high == 1.0:
            return "is true"
        return " and ".join(part for part in (f">= {self.low:g}" if self.low is not None else "",
                                               f"<= {self.high:g}" if self.high is not None else "") if part)


def judge(value: float | bool | None, limits: Limits) -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return NOT_RUN
    number = float(value)
    if limits.high is not None and number > limits.high:
        return FAIL
    if limits.low is not None and number < limits.low:
        return FAIL
    return PASS


@dataclass(frozen=True)
class Diagnostic:
    alias: str
    limits: Limits
    owner: str = "proposed"

    def judge(self, values: Mapping[str, object]) -> str:
        return judge(values.get(self.alias), self.limits)  # type: ignore[arg-type]

    def __str__(self) -> str:
        return f"{self.alias} {self.limits} ({self.owner})"


@dataclass(frozen=True)
class Verdict:
    verdict: str
    failing: tuple[str, ...] = ()


@dataclass(frozen=True)
class Gate:
    name: str
    members: Sequence[Diagnostic]

    def judge(self, values: Mapping[str, object]) -> Verdict:
        verdicts = [(member, member.judge(values)) for member in self.members]
        failing = tuple(str(member) for member, verdict in verdicts if verdict != PASS)
        if any(verdict == FAIL for _, verdict in verdicts):
            return Verdict(FAIL, failing)
        if any(verdict == NOT_RUN for _, verdict in verdicts):
            return Verdict(NOT_RUN, failing)
        return Verdict(PASS)
