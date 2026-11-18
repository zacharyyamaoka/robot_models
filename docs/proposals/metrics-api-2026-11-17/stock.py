"""The two kernels the generic rows need that robot_msgs.compare does not have yet: which rows move, and where the
stops are. Shared by every proposal, so the five differ in API, never in arithmetic.

    moving(target)                              -> bool mask: rows whose commanded speed is not zero
    StopTracker(scale=, band=).push(...)        -> the stops, one sample (or chunk) at a time: the LIVE form
    stops(target, measured, scale=, band=)      -> tuple[Stop, ...]: the same tracker folded over a whole recording
    stop.move()                                 -> (reference, measured) of the move into it, for robot_msgs.compare.goal

WHY one per-sample rule for live and offline (Zach, 2026-11-17: "Able to compute live and offline ... it doesn't matter
if the sensor stream is online or offline"): ``stops()`` is literally the tracker run over the recording, the pattern
robot_msgs.compare already uses for ``check`` (offline) and ``ToleranceMonitor`` (live), so the two can never disagree.

WHY here and not in a variant: the vault note (2026-11-17, "Where it could live") names exactly two rows that live
outside robot_msgs.compare today, the stop rule (sim2real metrics.actuator.stops, pendulum_score.stops) and the speed
ladder. robot_models may import only numpy and robot_msgs, so a per-type list there cannot call sim2real. The proposed
home is robot_msgs.compare (``robot_msgs.compare.stops``), beside ``goal``; every variant README says so.

The rule is pendulum_score.stops (bam_ws branch claude/1dof-rack-sim-to-real-435fc0 @ cdeb39271) on robot_msgs
streams: a stop is a dwell of at least 0.25 s at zero commanded speed on one constant target that a move led into (a
dwell the episode starts in is no stop); its error is the mean of measured - target over the dwell's last 50 ms (T4's
settled tail); its settle time runs from the dwell's start (the move's end) to the first row after which every row
stays inside ``band`` (0 inside from the start, inf never inside at its end). ``scale`` turns the joint's radians into
the band's unit (1e3 x tip length for millimetres at a pendulum's tip; 1.0 for radians).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from robot_msgs.trajectory import JointTrajectory

STILL_RAD_S = 1e-9      # sim2real.metrics.actuator.stops: a row at rest has |commanded speed| below this
DWELL_MIN_S = 0.25      # stop_accuracy.DWELL_MIN_S (T4): a stop lasts at least this long on one constant target
SETTLE_WINDOW_S = 0.05  # stop_accuracy.SETTLE_WINDOW_S (T4): a stop's error is the mean over its last 50 ms
SAME_TARGET_RAD = 1e-9  # a dwell is one constant target


@dataclass(frozen=True, eq=False)
class Stop:
    """One settled stop. ``error`` is signed radians (measured - target); ``settle_s`` seconds (inf: never settled).
    ``move_t / move_q / move_target`` hold the move into it (from the last still row before it to the dwell's end)."""

    t_s: float
    target: float
    error: float
    settle_s: float
    move_t: np.ndarray
    move_q: np.ndarray
    move_target: np.ndarray
    move_end: int          # the dwell's first row within move_*: the move ends there
    joint: str = "joint"

    def move(self) -> tuple[JointTrajectory, JointTrajectory]:
        """The move as robot_msgs.compare.goal wants it: the reference up to the move's end (its last point is the
        stop's target), the measured run through the whole dwell (goal's settle phase)."""

        names = (self.joint,)
        reference = JointTrajectory(t=self.move_t[: self.move_end + 1], joint_names=names,
                                    positions=self.move_target[: self.move_end + 1, None])
        measured = JointTrajectory(t=self.move_t, joint_names=names, positions=self.move_q[:, None])
        return reference, measured


def moving(target: JointTrajectory, joint: int = 0) -> np.ndarray:
    """Rows whose commanded speed is not zero (the moving part of a move): tracking RMS is read on these."""

    return np.abs(np.asarray(target.velocities[:, joint], float)) >= STILL_RAD_S


class StopTracker:
    """The stop rule one sample at a time. ``push`` returns the stops it closed; ``finish`` closes the last dwell."""

    def __init__(self, *, scale: float = 1.0, band: float = math.inf, joint: str = "joint",
                 include_last_dwell: bool = True) -> None:
        # WHY a flag, default True: the pendulum's rule counts the episode's last dwell (a ladder play is A -> B -> A and
        # its return to A is a commanded stop); sim2real.metrics.actuator.stops and RobotDx's T4 drop it (a tape's last
        # dwell is parking). One constructor holds both rules instead of two functions drifting apart.
        self.scale, self.band, self.joint = float(scale), float(band), joint
        self.include_last_dwell = include_last_dwell
        self.rows = 0                      # samples seen
        self.t0: float | None = None
        self.since: list[tuple[float, float, float]] = []   # (t, q, target) from the last still row before this move
        self.dwell_start: int | None = None                 # index into `since` of the open dwell's first row
        self.dwell_target = 0.0
        self.dwell_first_row = 0

    def push(self, t: float, q: float, target: float, target_speed: float) -> tuple[Stop, ...]:
        if self.t0 is None:
            self.t0 = t
        still = abs(target_speed) < STILL_RAD_S
        closed: tuple[Stop, ...] = ()
        if self.dwell_start is not None and not (still and abs(target - self.dwell_target) < SAME_TARGET_RAD):
            closed = self._close()
        self.since.append((t, q, target))
        if still and self.dwell_start is None:
            self.dwell_start, self.dwell_target, self.dwell_first_row = len(self.since) - 1, target, self.rows
        self.rows += 1
        return closed

    def push_many(self, t: np.ndarray, q: np.ndarray, target: np.ndarray, target_speed: np.ndarray) -> tuple[Stop, ...]:
        found: list[Stop] = []
        for row in range(len(t)):
            found.extend(self.push(float(t[row]), float(q[row]), float(target[row]), float(target_speed[row])))
        return tuple(found)

    def finish(self) -> tuple[Stop, ...]:
        if self.dwell_start is None:
            return ()
        closed = self._close()
        return closed if self.include_last_dwell else ()

    def _close(self) -> tuple[Stop, ...]:
        assert self.dwell_start is not None
        rows = np.asarray(self.since, float)
        start = self.dwell_start
        dwell = rows[start:]
        out: tuple[Stop, ...] = ()
        if self.dwell_first_row > 0 and dwell[-1, 0] - dwell[0, 0] >= DWELL_MIN_S:
            tail = dwell[dwell[:, 0] >= dwell[-1, 0] - SETTLE_WINDOW_S]
            error = float(np.mean(tail[:, 1] - tail[:, 2]))
            outside = np.flatnonzero(self.scale * np.abs(dwell[:, 1] - dwell[:, 2]) > self.band)
            if not outside.size:
                settle = 0.0
            elif outside[-1] == len(dwell) - 1:
                settle = math.inf
            else:
                settle = float(dwell[outside[-1] + 1, 0] - dwell[0, 0])
            out = (Stop(float(dwell[0, 0] - self.t0), float(dwell[0, 2]), error, settle, rows[:, 0].copy(),
                        rows[:, 1].copy(), rows[:, 2].copy(), start, self.joint),)
        # the dwell's last row is the "last still row" the next move starts from
        self.since = [tuple(rows[-1])]
        self.dwell_start = None
        return out


def stops(target: JointTrajectory, measured: JointTrajectory, *, scale: float = 1.0, band: float = math.inf,
          joint: int = 0, include_last_dwell: bool = True) -> tuple[Stop, ...]:
    """Every stop of a whole recording: the StopTracker folded over it (one rule, live and offline)."""

    tracker = StopTracker(scale=scale, band=band, joint=target.joint_names[joint], include_last_dwell=include_last_dwell)
    found = tracker.push_many(np.asarray(measured.t, float), np.asarray(measured.positions[:, joint], float),
                              np.asarray(target.positions[:, joint], float),
                              np.asarray(target.velocities[:, joint], float))
    return found + tracker.finish()


def settle_time(stop: Stop, band: float, scale: float = 1.0) -> float:
    """A stop's settle time against any band, read from the dwell rows it carries (the tracker's own rule): from the
    move's end to the first row after which every row stays inside ``band``; 0 inside from the start; inf never.
    WHY a function of the stop and not of the tracker: the band is the caller's config (a tip's 1 mm, a joint's share),
    so one pass over the data serves every band."""

    t = stop.move_t[stop.move_end:]
    error = scale * np.abs(stop.move_q[stop.move_end:] - stop.move_target[stop.move_end:])
    outside = np.flatnonzero(error > band)
    if not outside.size:
        return 0.0
    if outside[-1] == t.size - 1:
        return math.inf
    return float(t[outside[-1] + 1] - t[0])
