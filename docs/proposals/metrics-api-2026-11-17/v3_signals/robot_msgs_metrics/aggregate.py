"""Across runs, rliable-style: a point estimate AND how sure it is (Agarwal et al., "Deep RL at the Edge of the
Statistical Precipice", NeurIPS 2021: report interval estimates, use the IQM, bootstrap over runs).

    aggregate(results, alias, ci=0.95)   -> Estimate(point, low, high, n)
    iqm(values)                          -> the interquartile mean: robust like a median, efficient like a mean

WHY a CI on a rung of 7 plays: with N = 7 the median's uncertainty is about the size of the differences between rungs,
and a gate passed by a point estimate can fail on the next round. A gate can ask for the bound, not the point.
The bootstrap resamples runs (pooled items resample within each resampled run), seeded, numpy only.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .plan import Alias, RunResult, across_runs


@dataclass(frozen=True)
class Estimate:
    point: float | None
    low: float | None
    high: float | None
    n: int

    def __str__(self) -> str:
        if self.point is None:
            return "n/a"
        if self.low is None:
            return f"{self.point:.3g}"
        return f"{self.point:.3g} [{self.low:.3g}, {self.high:.3g}]"


def iqm(values: Sequence[float]) -> float:
    ordered = np.sort(np.asarray(values, float))
    cut = int(np.floor(0.25 * ordered.size))
    return float(np.mean(ordered[cut: ordered.size - cut])) if ordered.size else float("nan")


def aggregate(results: Sequence[RunResult], alias: Alias, *, ci: float = 0.95, reps: int = 1000,
              seed: int = 20261117) -> Estimate:
    point = across_runs(alias, results)
    if point is None or isinstance(point, bool) or len(results) < 3:
        return Estimate(point, None, None, len(results))
    rng = np.random.default_rng(seed)
    draws = []
    for _ in range(reps):
        sample = [results[i] for i in rng.integers(0, len(results), len(results))]
        value = across_runs(alias, sample)
        if value is not None:
            draws.append(value)
    low, high = np.quantile(draws, [(1 - ci) / 2, 1 - (1 - ci) / 2]) if draws else (None, None)
    return Estimate(point, float(low), float(high), len(results))
