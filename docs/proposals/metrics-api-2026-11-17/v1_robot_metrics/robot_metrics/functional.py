"""The functional twins: one call per metric on one recording's (target, measured), SI values.

WHY each twin is its class run once (update the whole recording, end the run, compute) and not a second formula:
torchmetrics keeps two implementations (functional core, class wrapper); here a twin cannot drift from its class.
"""

from __future__ import annotations

from typing import Any

from . import base


def _once(metric: base.Metric, target: Any, measured: Any) -> Any:
    metric.update(target, measured)
    metric.end_run()
    return metric.compute()


def error_stat(target: Any, measured: Any, **config: Any) -> float | None:
    return _once(base.ErrorStat(**config), target, measured)


def dwell_error(target: Any, measured: Any, **config: Any) -> float | None:
    return _once(base.DwellError(**config), target, measured)


def settle_time(target: Any, measured: Any, **config: Any) -> float | None:
    return _once(base.SettleTime(**config), target, measured)


def overshoot(target: Any, measured: Any, **config: Any) -> float | None:
    return _once(base.Overshoot(**config), target, measured)


def all_settled(target: Any, measured: Any, **config: Any) -> bool:
    return _once(base.AllSettled(**config), target, measured)


def time_offset(target: Any, measured: Any, **config: Any) -> float | None:
    return _once(base.TimeOffset(**config), target, measured)


def code_share(target: Any, measured: Any, **config: Any) -> float | None:
    return _once(base.CodeShare(**config), target, measured)
