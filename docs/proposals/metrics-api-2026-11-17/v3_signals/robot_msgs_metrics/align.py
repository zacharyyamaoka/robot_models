"""Raw topics in, aligned pairs out: the recording as messages, and the aligner that pairs them live.

A real run is two topics on their own clocks: the command (the plan, one knot per 10 ms tick) and the state (each
reply, stamped when its command was sent, a millisecond or so after its tick). Comparing them needs the plan AT each
reply's instant, the cubic Hermite through the knots either side (the round-8 scorecard's ``align: measured``). Live,
the knot after a reply arrives one tick later, so the aligner HOLDS each reply until it can be paired: a fixed 1-tick
latency, stated, never an extrapolation. At the end of a run the last replies take the plan's last speed (as the
scorer does). WHY the aligner owns this and not each metric: time alignment is the one robot-signal step classification
metrics never needed, and every metric of a pair needs the same answer.
"""

from __future__ import annotations

import heapq
import math
from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from robot_msgs.trajectory import JointTrajectory, concatenate


@dataclass(frozen=True)
class Recording:
    """One run as its topics (MCAP-like): each a robot_msgs trajectory; ``messages()`` replays them in arrival order."""

    id: str
    topics: Mapping[str, Any]
    labels: Mapping[str, Any]
    rrd: str | None = None
    faulted: bool = False

    @classmethod
    def from_run(cls, run: Any) -> Recording:
        topics = {"/joint/command": run.plan, "/joint/state": run.measured}
        if run.tool_target is not None:
            topics = {"/joints/command": run.target, "/joints/state": run.measured,
                      "/tool/command": run.tool_target, "/tool/state": run.tool_measured}
        return cls(run.id, topics, {"station": run.station, "source": run.source, **run.labels}, faulted=run.faulted)

    def messages(self) -> Iterator[tuple[float, str, Any]]:
        """(time, topic, one-sample trajectory) by time; a command knot comes before a reply stamped at the same time."""

        order = {name: rank for rank, name in enumerate(sorted(self.topics, key=lambda n: "state" in n))}
        heads = [(float(stream.t[0]), order[name], name, 0) for name, stream in self.topics.items() if len(stream)]
        heapq.heapify(heads)
        while heads:
            time, rank, name, row = heapq.heappop(heads)
            stream = self.topics[name]
            yield time, name, stream[row:row + 1]
            if row + 1 < len(stream):
                heapq.heappush(heads, (float(stream.t[row + 1]), rank, name, row + 1))


@dataclass
class Aligner:
    """Pairs command knots and state replies of one joint signal. ``push`` returns the aligned pair it released.

    A message may hold one row (live) or a whole topic (offline): rows are held until a later knot exists."""

    command: str
    state: str
    rule: str = "hermite"      # hermite: the plan at each reply's instant | same_clock: rows already paired by time
    commands: list[Any] = field(default_factory=list)      # same_clock: command rows not yet paired
    knots_t: list[float] = field(default_factory=list)
    knots_q: list[float] = field(default_factory=list)
    knots_qd: list[float] = field(default_factory=list)
    held: list[Any] = field(default_factory=list)          # reply rows waiting for the knot after them
    max_held: int = 0                                      # the latency actually paid, in replies

    def push(self, topic: str, message: Any) -> tuple[Any, Any] | None:
        if self.rule == "same_clock":
            return self._same_clock(topic, message)
        if topic == self.command:
            self.knots_t += message.t.tolist()
            self.knots_q += message.positions[:, 0].tolist()
            self.knots_qd += message.velocities[:, 0].tolist()
        elif topic == self.state:
            self.held.append(message)
            self.max_held = max(self.max_held, sum(len(reply) for reply in self.held))
        else:
            return None
        if not self.held or not self.knots_t:
            return None
        waiting = self.held[0] if len(self.held) == 1 else concatenate(self.held)
        ready = np.asarray(waiting.t) < self.knots_t[-1]
        if not ready.any():
            self.held = [waiting]
            return None
        self.held = [waiting[np.flatnonzero(~ready)]] if not ready.all() else []
        return self._pair(waiting[np.flatnonzero(ready)], final=False)

    def _same_clock(self, topic: str, message: Any) -> tuple[Any, Any] | None:
        (self.commands if topic == self.command else self.held).append(message)
        if not self.commands or not self.held:
            return None
        command = self.commands[0] if len(self.commands) == 1 else concatenate(self.commands)
        state = self.held[0] if len(self.held) == 1 else concatenate(self.held)
        count = min(len(command), len(state))
        if not np.array_equal(command.t[:count], state.t[:count]):
            raise ValueError("same_clock streams disagree on their sample times")
        self.commands = [command[count:]] if len(command) > count else []
        self.held = [state[count:]] if len(state) > count else []
        return command[:count], state[:count]

    def finish(self) -> tuple[JointTrajectory, JointTrajectory] | None:
        if self.rule == "same_clock":
            self.commands, self.held = [], []
            return None
        if not self.held:
            return None
        waiting = self.held[0] if len(self.held) == 1 else concatenate(self.held)
        self.held = []
        return self._pair(waiting, final=True)

    def _pair(self, replies: Any, final: bool) -> tuple[JointTrajectory, JointTrajectory]:
        t = np.asarray(replies.t, float)
        q_target, qd_target = _hermite(t, np.asarray(self.knots_t), np.asarray(self.knots_q), np.asarray(self.knots_qd),
                                       final)
        target = JointTrajectory(t=t, joint_names=replies.joint_names, positions=q_target[:, None],
                                 velocities=qd_target[:, None], header=replies.header)
        return target, replies


def _hermite(t: np.ndarray, knots_t: np.ndarray, knots_q: np.ndarray, knots_qd: np.ndarray,
             final: bool) -> tuple[np.ndarray, np.ndarray]:
    """The plan at each t (pendulum_score.measured_target's rule) and its knot's commanded speed (the row's own tick)."""

    count = knots_t.size
    own = np.clip(np.searchsorted(knots_t, t, side="right") - 1, 0, count - 1)
    if count == 1:
        return knots_q[0] + knots_qd[0] * (t - knots_t[0]), knots_qd[own]
    j = np.clip(np.searchsorted(knots_t, t, side="right") - 1, 0, count - 2)
    dt = knots_t[j + 1] - knots_t[j]
    s = (t - knots_t[j]) / dt
    p0, p1, m0, m1 = knots_q[j], knots_q[j + 1], knots_qd[j], knots_qd[j + 1]
    inside = (s >= 0.0) & (s <= 1.0)
    h01 = s * s * (3.0 - 2.0 * s)
    hermite = np.clip(p0 + h01 * (p1 - p0) + dt * (s * (1.0 - s) ** 2 * m0 + s * s * (s - 1.0) * m1),
                      np.minimum(p0, p1), np.maximum(p0, p1))
    before = knots_q[0] + knots_qd[0] * (t - knots_t[0])
    after = knots_q[-1] + knots_qd[-1] * (t - knots_t[-1])
    if not final and np.any(t > knots_t[-1]):
        raise AssertionError("a reply was released before the knot after it arrived")
    return np.where(inside, hermite, np.where(t < knots_t[0], before, after)), knots_qd[own]


assert math.isfinite(_hermite(np.array([0.5]), np.array([0.0, 1.0]), np.array([0.0, 1.0]), np.array([0.0, 0.0]), True)[0][0])
