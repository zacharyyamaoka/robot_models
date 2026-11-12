"""The homing bench's two seams (docs/peps/0002-homing-bench.md; the decision report's proposal C, grafted onto B).

* ``HomingPlant``      what a procedure runs on: ``reset(start)``, ``tick(command, dt)``, ``truth()``. A kinematic stop plant,
                       a MuJoCo rig and the real bus implement it, so the SAME procedure, the same sequence bytes, runs on every
                       rung; only the plant changes.
* ``HomingProcedure``  a black box: ``plan()`` (collect) emits a robot_msgs ``JointCommandSequence`` (or None when done),
                       run by the ``JointCommandSequenceController``; ``observe()`` (fit) turns what came back into a
                       ``HomingResult``. Nothing in the bench reads a procedure's internals.

Typing only: no runtime dependency beyond robot_msgs and this package.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from robot_msgs.control import JointCommandSequence, ServoCommand
from robot_msgs.sensor import JointState

from robot_models.homing.types import BacklashTable, HomingResult, StopMap


@runtime_checkable
class HomingPlant(Protocol):
    """A rig a homing sequence can run on, one controller tick at a time."""

    joint_names: tuple[str, ...]

    def reset(self, start: Any) -> JointState: ...

    def tick(self, command: ServoCommand, dt: float, defines: Mapping[str, float] | None = None) -> JointState: ...

    def truth(self) -> Mapping[str, Any]: ...


@dataclass(frozen=True)
class Prior:
    """What a procedure may know before it moves: the stop map (CAD or typed), the stored backlash table (None on a
    first boot) and the joints it homes, each with the side it seeks first."""

    stops: StopMap
    joints: tuple[str, ...]
    seek_side: Mapping[str, int]
    table: BacklashTable | None = None
    caps_nm: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class Phase:
    """One sequence as it ran: the sequence, the controller's step events and the per-tick log the plant measured."""

    sequence: JointCommandSequence
    status: str
    message: str
    events: tuple[Any, ...]
    log: Mapping[str, Any]


@runtime_checkable
class HomingProcedure(Protocol):
    """A homing procedure as a black box: collect (``plan``) then fit (``observe``)."""

    name: str

    def plan(self, prior: Prior, so_far: Sequence[Phase]) -> JointCommandSequence | None: ...

    def observe(self, prior: Prior, phases: Sequence[Phase]) -> HomingResult: ...


__all__ = ["HomingPlant", "HomingProcedure", "Phase", "Prior"]
