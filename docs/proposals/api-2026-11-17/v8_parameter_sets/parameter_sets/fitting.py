"""Fitting and choosing for v8: numpy-only least squares, seeds read from units, the noise floor, the batch
result and the held-out selection. No model knowledge beyond `model.declared`, `Parameters` and `predict`."""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping
from typing import Any

import numpy as np

Log = Mapping[str, np.ndarray]  # a rig log: v [rad/s], tau [N·m] (the torque the motor added), plateau (a hold id)


class Fits(dict):
    """{key: Parameters} from a batch fit, plus .skipped {key: why}."""

    def __init__(self) -> None:
        super().__init__()
        self.skipped: dict[str, str] = {}

    def __str__(self) -> str:
        lines = [f"{'model':<20}{'fitted':>7}{'fit rms mN·m':>14}  values (SI)" ]
        for key, p in self.items():
            shown = ", ".join(f"{n}={v:.4g}" if isinstance(v, float) else f"{n}=({len(v)} weights)"
                              for n, v in p.needed().items() if n in p.info.fitted)
            lines.append(f"{key:<20}{p.info.count:>7}{1e3 * p.info.rms:>14.2f}  {shown or '(nothing the log moves)'}")
        return "\n".join(lines + [f"skipped: {', '.join(self.skipped)} (declared, no physics attached)"])


@dataclasses.dataclass(frozen=True)
class Selection:
    """Fits scored on held-out plateaus, lowest rung first. WHY held out (Zach 2026-11-17: "if doing a more complex
    model doesn't actually improve test time performance you are just overfitting")."""

    rows: tuple[tuple[Any, float], ...]   # (value set, held-out rms [N·m])
    floor: float                                    # N·m
    limit: float

    @property
    def best(self) -> Any | None:
        return next((p for p, rms in self.rows if rms / self.floor <= self.limit), None)

    @property
    def chosen(self) -> str | None:
        return self.best.model.key if self.best is not None else None

    def __str__(self) -> str:
        best = self.best
        lines = [f"held-out plateaus · floor {1e3 * self.floor:.2f} mN·m · lowest rung within {self.limit}× floor",
                 f"{'model':<20}{'params':>7}{'fit rms':>9}{'held-out':>10}{'÷ floor':>9}   [rms in mN·m]"]
        for p, rms in self.rows:
            mark = "  <- chosen" if p is best else ""
            lines.append(f"{p.model.key:<20}{p.model.param_count:>7}{1e3 * p.info.rms:>9.2f}{1e3 * rms:>10.2f}"
                         f"{rms / self.floor:>9.2f}{mark}")
        return "\n".join(lines + ([] if best else ["no rung reaches the floor: climb a rung, or measure more"]))



# --- helpers: the floor, seeds, numpy-only least squares, the reference ----------------------------------------------


def noise_floor(log: Log) -> float:
    """What the rig repeats to [N·m]: the pooled within-plateau standard deviation of tau, read from the log itself."""
    plateaus = np.unique(log["plateau"])
    spread = np.concatenate([log["tau"][log["plateau"] == p] - log["tau"][log["plateau"] == p].mean()
                             for p in plateaus])
    return float(np.sqrt(spread @ spread / (spread.size - plateaus.size)))


def _seeds(declared: Any, log: Log) -> list[Any]:
    if _is_weights(declared):
        return [(0.0,) * declared.count]
    speeds, torques = np.abs(log["v"][log["v"] != 0]), np.abs(log["tau"])
    return {"rad/s": list(np.geomspace(speeds.min(), speeds.max(), 6)), "N·m": list(np.quantile(torques, [0.5, 1.0])),
            "N·m·s/rad": [0.0]}.get(declared.unit, [1.0])


def _indices(declared: Any) -> list[int | None]:
    return list(range(declared.count)) if _is_weights(declared) else [None]


def _get(model: Any, values: dict, free: tuple[str, int | None]) -> float:
    name, index = free
    if index is None:
        return np.log(values[name]) if model.declared[name].positive else values[name]
    return values[name][index]


def _unpack(model: Any, values: dict, free: list, x: np.ndarray) -> dict:
    out = {n: (list(v) if isinstance(v, tuple) else v) for n, v in values.items()}
    for (name, index), value in zip(free, x.tolist()):
        if index is None:
            out[name] = float(np.exp(min(value, 700.0))) if model.declared[name].positive else value
        else:
            out[name][index] = value
    return {n: (tuple(v) if isinstance(v, list) else v) for n, v in out.items()}


def _moves(model: Any, values: dict, name: str, log: Log) -> bool:
    """Does the log's torque respond to `name`? Inertia and gear ratio never move a velocity-plateau sweep."""
    if _is_weights(model.declared[name]):
        return True
    bumped = {**values, name: values[name] * 1.01 + 1e-6}
    return not np.array_equal(model.predict(model.Parameters(**values), log["v"]),
                              model.predict(model.Parameters(**bumped), log["v"]))


def _least_squares(residual, x: np.ndarray, iterations: int = 100) -> tuple[np.ndarray, float]:
    """Levenberg-Marquardt with a forward-difference Jacobian: numpy only (the runtime has no scipy)."""
    r = residual(x)
    cost, damping = float(r @ r), 1e-3
    for _ in range(iterations if x.size else 0):
        steps = 1e-6 * np.maximum(np.abs(x), 1e-3)
        jacobian = np.column_stack([(residual(x + np.eye(x.size)[i] * steps[i]) - r) / steps[i] for i in range(x.size)])
        normal, gradient = jacobian.T @ jacobian, jacobian.T @ r
        while damping < 1e12:
            step = np.linalg.solve(normal + damping * np.diag(np.diag(normal) + 1e-12), -gradient)
            trial_r = residual(x + step)
            trial = float(trial_r @ trial_r)
            if np.isfinite(trial) and trial < cost:
                break
            damping *= 10
        else:
            break
        done = cost - trial < 1e-12 * cost
        x, r, cost, damping = x + step, trial_r, trial, max(damping / 10, 1e-9)
        if done:
            break
    return x, cost


def _is_weights(declared: Any) -> bool:
    return hasattr(declared, "count")  # a black box's Weights; a Parameter is one scalar
