"""The one dataset every metrics-API proposal runs on: real runs as robot_msgs streams, in and out.

    pendulum_runs("real")   -> tuple[Run, ...]   the 1-DOF rack's round 8: 252 plays, 6 stations x 6 rungs x 7 plays
    pendulum_runs("sim")    -> tuple[Run, ...]   the twin's answer to each of those plays (target vs sim)
    chain_runs("real")      -> tuple[Run, ...]   the five-bar's real speed ladder (bam_fb, 2026-11-10): 5 rungs
    chain_runs("sim")       -> tuple[Run, ...]   its sim twin's rungs

A Run is one episode (Zach, 3000-ft view, 2026-11-17: "runs are the atomic unit"): what was commanded and what came
out, both robot_msgs trajectories, plus the labels a roll-up groups by (deployment, station, rung, kind, source). The
fixture prepares data only; it computes no metric. Read-only on /archive and the sim memo.

What the fixture decides, each the round-8 scorecard's own rule (so a metric computed on these streams can match it):

- **The episode is the tape.** The runner's prelude (its move to A from wherever the joint rested) is dropped: rows on a
  plan tick at or after ``prelude_s`` (pendulum_score.load_run).
- **The target is the plan at each reply's own instant** (the scorecard's ``align: measured``): the cubic Hermite
  through the logged plan ticks (q_target, qd_target), sampled at each row's send time ``t``. Its velocities are the
  logged ``qd_target`` per row, so "is this row moving" reads the plan, not a derivative. The plan on its ticks rides
  along as ``Run.plan``. Sims answer on their exact ticks, so a sim's target is its logged ``q_target``.
- **Paired runs** (one file, two boards, ``<field>_can<id>`` columns) are one play per board, each read as its own
  single-board run (pendulum_score.board_view): its rows are the ones that board answered, its tape starts at
  ``tape_start_s``.
- **Output frame:** board = output on every pendulum station since 2026-10-24 (the station record's factor is 1.0).
- **Rung and kind** come from the round's own plan index (``plans/index.json``), matched by station and run name.
- **Which twin answered** is the one label the fixture reads from the scorecard (``twins_used``); no number is read
  from it. A sim answer is the memo file whose station, twin, controller hash and plan step match the play.

WHY header stamps of 0 with the wall clock in ``header.meta``: robot_msgs aligns on absolute times (stamp + t), and at
today's epoch float64 resolves 0.24 us, which moves an interpolated target by up to ~1e-6 rad on a fast rung. The
streams carry run-relative time so that comparing a target and a reply on the same instants is exact.
"""

from __future__ import annotations

import functools
import hashlib
import json
import math
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from robot_msgs.std.header import Header
from robot_msgs.trajectory import JointTrajectory, PoseTrajectory

ROUND = Path("/archive/datasets/bam_rig/onedof_2026-11-15/round8")
SCORECARD = Path("/archive/datasets/bam_rig/onedof_2026-11-15/scorecards/latest_real.json")
SIM_MEMO = Path(os.environ.get("BAM_PENDULUM_SIM_MEMO") or Path.home() / ".cache/bam_rack_twin/pendulum_sim_memo")
CHAIN_LADDER = Path("/archive/datasets/bam_rig/rack_2026-11-10/fivebar_real/ladder_real_124755/ladder")
DEPLOYMENT = "one-dof-test-rack"
CHAIN_DEPLOYMENT = "bam_fb five-bar (rack 2026-11-10)"
CHAIN_JOINTS = ("j2", "c1")

TICK_S = 0.01          # the runner's plan tick unless a run's meta says another (pendulum_score.tick_s_of)
NO_TICK = -1           # a row that is no plan tick's command
MODE_POSITION = 10     # moteus position mode
STEP_IDENTITY_IGNORES = ("label", "schedule")   # repeats differ in label and stamp, never in their input
SAMPLES_PER_BOARD = ("ff_coulomb_nm", "ff_gravity_nm", "ff_gravity_bottom_rad", "ff_drive_torque_nm", "kp_scale_cmd",
                     "kd_scale_cmd", "trajectory_limits", "ff_inertia_kg_m2", "ff_viscous_nm_per_rad_s", "velocity_mode")


@dataclass(frozen=True)
class Run:
    """One episode: the command and the response as robot_msgs streams, and the labels a roll-up groups by.

    ``target`` and ``measured`` share one clock and one row count (the measured run's rows; ``target`` is the command
    at each of them). ``plan`` is the command as sent, on its own ticks (None for the chain, whose target is its plan).
    A chain run also carries its tool: ``tool_target`` / ``tool_measured`` (FK of the command and of the encoders).
    ``tip_length_m`` is the pendulum's lever: the tip moves tip_length_m x the joint angle (None for the chain).
    """

    id: str
    deployment: str
    station: str
    part_type: str
    source: str                         # "real" (target vs real) or "sim" (target vs the twin's answer)
    labels: Mapping[str, Any]           # rung, kind, ... : what a roll-up groups by
    target: JointTrajectory
    measured: JointTrajectory
    plan: JointTrajectory | None = None
    tool_target: PoseTrajectory | None = None
    tool_measured: PoseTrajectory | None = None
    tip_length_m: float | None = None
    faulted: bool = False
    meta: Mapping[str, Any] = field(default_factory=dict)

    @property
    def rung(self) -> Any:
        return self.labels.get("rung")

    def __repr__(self) -> str:
        return f"Run({self.source} {self.station} rung {self.rung} {self.labels.get('kind', '')} · {len(self.measured)} rows)"


# --------------------------------------------------------------------------- reading the rack's run files


def _hermite_at(t_rows: np.ndarray, tick: np.ndarray, q_target: np.ndarray, qd_target: np.ndarray,
                tick_s: float) -> np.ndarray:
    """The plan at each row's own time: the cubic Hermite through the commanded ticks (pendulum_score.measured_target)."""

    out = q_target.copy()
    commanded = (tick != NO_TICK) & np.isfinite(q_target) & np.isfinite(qd_target) & np.isfinite(t_rows)
    rows = np.flatnonzero(commanded)
    if rows.size == 0:
        return out
    order = rows[np.argsort(tick[rows], kind="stable")]
    knots_t, keep = np.unique(tick[order] * tick_s, return_index=True)
    knots_q, knots_qd = q_target[order][keep], qd_target[order][keep]
    at = t_rows[rows]
    if knots_t.size == 1:
        out[rows] = knots_q[0] + knots_qd[0] * (at - knots_t[0])
        return out
    j = np.clip(np.searchsorted(knots_t, at, side="right") - 1, 0, knots_t.size - 2)
    dt = knots_t[j + 1] - knots_t[j]
    s = (at - knots_t[j]) / dt
    p0, p1, m0, m1 = knots_q[j], knots_q[j + 1], knots_qd[j], knots_qd[j + 1]
    inside = (s >= 0.0) & (s <= 1.0)
    h01 = s * s * (3.0 - 2.0 * s)
    hermite = p0 + h01 * (p1 - p0) + dt * (s * (1.0 - s) ** 2 * m0 + s * s * (s - 1.0) * m1)
    hermite = np.clip(hermite, np.minimum(p0, p1), np.maximum(p0, p1))
    before = knots_q[0] + knots_qd[0] * (at - knots_t[0])
    after = knots_q[-1] + knots_qd[-1] * (at - knots_t[-1])
    out[rows] = np.where(inside, hermite, np.where(at < knots_t[0], before, after))
    return out


def _meta_of(npz: Path) -> dict[str, Any]:
    path = npz.with_suffix(".json")
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _board_columns(npz: Path, board: int | None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """A run file's columns in the single-board layout, and its tape start (pendulum_score.read_run / board_view)."""

    meta = _meta_of(npz)
    with np.load(npz) as archive:
        data = {name: np.asarray(archive[name]) for name in archive.files}
    tick_s = _finite(meta.get("tick_s")) or TICK_S
    if board is None:
        return data, {"prelude_s": float(meta.get("prelude_s") or 0.0), "tick_s": tick_s, "reason": meta.get("reason")}
    suffix = f"_can{board}"
    own = {name[: -len(suffix)]: values for name, values in data.items() if name.endswith(suffix)}
    rows = np.isfinite(np.asarray(own["mode"], float))
    columns = {name: np.asarray(data[name])[rows] for name in ("t", "wall", "tick") if name in data}
    columns.update({name: values[rows] for name, values in own.items()})
    tape_start = _finite(meta.get("tape_start_s"))
    if tape_start is None:
        tape_start = (math.floor((_finite(meta.get("approach_s")) or 0.0) / tick_s + 1e-9) + 1) * tick_s
    outcome = str((meta.get("boards") or {}).get(str(board), {}).get("outcome") or meta.get("reason") or "")
    return columns, {"prelude_s": tape_start, "tick_s": tick_s, "reason": outcome}


def _header(frame: str, **meta: Any) -> Header:
    return Header(stamp=0.0, frame_id=frame, meta={key: str(value) for key, value in meta.items() if value is not None})


def _real_streams(npz: Path, board: int | None, station: str) -> tuple[JointTrajectory, JointTrajectory, JointTrajectory, int]:
    data, info = _board_columns(npz, board)
    t = np.asarray(data["t"], float)
    tick = np.asarray(data["tick"]).astype(np.int64)
    tick_s = info["tick_s"]
    q_target = np.asarray(data["q_target"], float)
    qd_target = np.asarray(data["qd_target"], float)
    tape = (tick != NO_TICK) & (tick * tick_s >= info["prelude_s"] - 1e-9)
    target = _hermite_at(t, tick, q_target, qd_target, tick_s)
    mode = np.asarray(data.get("mode", np.full(t.size, MODE_POSITION)), float)
    uncontrolled = int(np.count_nonzero(tape & (mode != MODE_POSITION)))
    # WHY drop uncontrolled rows: the scorecard judges position-mode rows only; round 8 has none outside it (asserted
    # by check_scenario.py through Run.meta["uncontrolled_rows"]), so the cut changes nothing it scores.
    keep = tape & (mode == MODE_POSITION) & np.isfinite(np.asarray(data["q"], float))
    wall0 = float(data["wall"][0]) if "wall" in data and np.size(data["wall"]) else None
    header = _header("output", wall0=wall0, source=str(npz), board=board)
    names = (station,)
    target_stream = JointTrajectory(t=t[keep], joint_names=names, positions=target[keep][:, None],
                                    velocities=qd_target[keep][:, None], header=header)
    measured = JointTrajectory(
        t=t[keep], joint_names=names, positions=np.asarray(data["q"], float)[keep][:, None],
        velocities=np.asarray(data["qd"], float)[keep][:, None], effort=np.asarray(data["tau"], float)[keep][:, None],
        channels={"fault": np.asarray(data.get("fault", np.zeros(t.size)), float)[keep],
                  **({"ff_nm": np.asarray(data["ff_model_nm"], float)[keep]} if "ff_model_nm" in data else {})},
        header=header)
    ticked = keep & (tick != NO_TICK)
    order = np.flatnonzero(ticked)
    plan_t, first = np.unique(tick[order] * tick_s, return_index=True)
    plan = JointTrajectory(t=plan_t, joint_names=names, positions=q_target[order][first][:, None],
                           velocities=qd_target[order][first][:, None], header=header)
    return target_stream, measured, plan, uncontrolled


def _identity(step: Mapping[str, Any]) -> str:
    return json.dumps({key: value for key, value in step.items() if key not in STEP_IDENTITY_IGNORES}, sort_keys=True,
                      default=str)


def _per_board(value: Any, can_id: int) -> Any:
    if isinstance(value, Mapping):
        return value[str(can_id)] if str(can_id) in value else value.get(can_id)
    return value


def _board_step(step: Mapping[str, Any], can_id: int) -> dict[str, Any]:
    """One board's part of a paired plan step, as the session built it (pendulum_score.board_step)."""

    samples = _per_board(step["samples_q"], can_id)
    samples = [float(value) for value in (samples.split(",") if isinstance(samples, str) else samples)]
    part: dict[str, Any] = {"kind": "samples", "can_id": int(can_id), "cap_nm": _per_board(step["cap_nm"], can_id),
                            "origin_rad": samples[0], "samples_q": [value - samples[0] for value in samples]}
    for key in SAMPLES_PER_BOARD:
        if key in step and _per_board(step[key], can_id) is not None:
            part[key] = _per_board(step[key], can_id)
    if _per_board(step.get("ff_band_rad_s"), can_id) is not None:
        part["ff_band_rad_s"] = _per_board(step["ff_band_rad_s"], can_id)
    if float(_per_board(step.get("ff_hold_nm"), can_id) or 0.0):
        part.update({key: _per_board(step[key], can_id) for key in ("ff_hold_nm", "ff_hold_band_rad", "ff_hold_after_s")
                     if key in step})
    if float(_per_board(step.get("kp_scale_hold"), can_id) or 1.0) != 1.0:
        part["kp_scale_hold"] = _per_board(step["kp_scale_hold"], can_id)
    rate = _finite(step.get("rate_hz"))
    if rate is not None and abs(rate * TICK_S - 1.0) > 1e-9:
        part["rate_hz"] = rate
    for key in STEP_IDENTITY_IGNORES:
        if key in step:
            part[key] = step[key]
    return part


@dataclass(frozen=True)
class _Play:
    station: str
    npz: Path
    board: int | None
    step: Mapping[str, Any]
    status: str
    fraction: float
    kind: str
    id: str


@functools.cache
def _index() -> dict[str, Any]:
    return json.loads((ROUND / "plans" / "index.json").read_text(encoding="utf-8"))


@functools.cache
def _plays() -> tuple[_Play, ...]:
    """Every play of round 8, in session order: one per board of each session step, its rung and kind from the index."""

    by_run = {(entry["station"], entry["run_name"]): entry for entry in _index()["runs"]}
    plays = []
    for session_dir in sorted(p for p in (ROUND / "real").iterdir() if (p / "session.json").is_file()):
        session = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
        plan_steps = (session.get("plan") or {}).get("steps") or []
        for entry in session.get("steps") or []:
            boards = entry.get("can_ids") if entry.get("can_id") is None else None
            if entry.get("kind") == "pause" or (entry.get("can_id") is None and not boards):
                continue
            step = plan_steps[int(entry["index"]) - 1]
            npz = session_dir / next(f for f in entry["files"] if f.endswith(".npz"))
            for can_id in boards or [int(entry["can_id"])]:
                station = f"can{can_id}"
                status = str((entry.get("boards") or {}).get(str(can_id)) or entry.get("status")) if boards else str(
                    entry.get("status"))
                found = by_run.get((station, npz.stem))
                if found is None:  # a step the plan index never named (a park, a recovery): scored nowhere, as the scorer does
                    continue
                own = _board_step(step, can_id) if boards else step
                play_id = str(npz) if not boards else f"{npz}#can{can_id}"
                plays.append(_Play(station, npz, int(can_id) if boards else None, own, status,
                                   float(found["fraction"]), str(found["kind"]), play_id))
    return tuple(plays)


def stations() -> dict[str, dict[str, float]]:
    """Each station's facts from the round's plan index: tip length (m) and top speed (rad/s)."""

    return {name: {"tip_length_m": float(facts["tip_length_m"]), "omega_max_rad_s": float(facts["omega_max_rad_s"])}
            for name, facts in _index()["stations"].items()}


@functools.cache
def _real_runs() -> tuple[Run, ...]:
    facts = stations()
    runs = []
    for play in _plays():
        target, measured, plan, uncontrolled = _real_streams(play.npz, play.board, play.station)
        runs.append(Run(id=play.id, deployment=DEPLOYMENT, station=play.station, part_type="integrated_servo",
                        source="real", labels={"rung": play.fraction, "kind": play.kind}, target=target,
                        measured=measured, plan=plan, tip_length_m=facts[play.station]["tip_length_m"],
                        faulted=play.status != "complete",
                        meta={"omega_max_rad_s": facts[play.station]["omega_max_rad_s"],
                              "uncontrolled_rows": uncontrolled}))
    return tuple(runs)


# --------------------------------------------------------------------------- the twin's answers (sim memo)


@functools.cache
def _memo_index() -> dict[tuple[str, str, str, str], Path]:
    """(station, twin, controller sha256, step identity) -> the memo's npz; the newest file wins a duplicate key."""

    found: dict[tuple[str, str, str, str], tuple[str, Path]] = {}
    for path in SIM_MEMO.glob("*.json"):
        entry = json.loads(path.read_text(encoding="utf-8"))
        key = (entry.get("station"), entry.get("twin"), entry.get("controller"), entry.get("step"))
        if None in key:
            continue
        written = str(entry.get("written_utc") or "")
        if key not in found or written > found[key][0]:
            found[key] = (written, path.with_suffix(".npz"))
    return {key: value[1] for key, value in found.items()}


@functools.cache
def _twins_used() -> dict[str, str]:
    """WHY the one label read from the scorecard: which twin version answered each station is the scoring's own
    metadata ("latest" resolved once per station); the fixture reads no number from it."""

    return dict(json.loads(SCORECARD.read_text(encoding="utf-8"))["twins_used"])


def _controller_sha() -> str:
    controller = json.loads((ROUND / "controller.json").read_text(encoding="utf-8"))
    return hashlib.sha256(json.dumps(controller, sort_keys=True, default=str).encode()).hexdigest()


@functools.cache
def _sim_runs() -> tuple[Run, ...]:
    facts, twins, controller, memo = stations(), _twins_used(), _controller_sha(), _memo_index()
    runs = []
    for play in _plays():
        path = memo.get((play.station, twins[play.station], controller, _identity(play.step)))
        if path is None or not path.is_file():
            continue
        with np.load(path) as archive:
            answer = {name: np.asarray(archive[name]) for name in archive.files}
        t = np.asarray(answer["t"], float)
        q_target = np.asarray(answer["q_target"], float)
        qd_target = (np.asarray(answer["qd_target"], float) if "qd_target" in answer
                     else np.gradient(q_target, t))
        prelude = float(answer["prelude_s"]) if "prelude_s" in answer else 0.0
        tape = t >= t[0] + prelude - 1e-9
        mode = np.asarray(answer["mode"], float) if "mode" in answer else np.full(t.size, MODE_POSITION)
        keep = tape & (mode == MODE_POSITION) & np.isfinite(np.asarray(answer["q"], float))
        header = _header("output", source=str(path), twin=twins[play.station])
        names = (play.station,)
        target = JointTrajectory(t=t[keep], joint_names=names, positions=q_target[keep][:, None],
                                 velocities=qd_target[keep][:, None], header=header)
        measured = JointTrajectory(t=t[keep], joint_names=names, positions=np.asarray(answer["q"], float)[keep][:, None],
                                   velocities=np.asarray(answer["qd"], float)[keep][:, None],
                                   effort=np.asarray(answer["tau"], float)[keep][:, None],
                                   channels={"fault": np.asarray(answer.get("fault", np.zeros(t.size)), float)[keep]},
                                   header=header)
        runs.append(Run(id=play.id, deployment=DEPLOYMENT, station=play.station, part_type="integrated_servo",
                        source="sim", labels={"rung": play.fraction, "kind": play.kind, "twin": twins[play.station]},
                        target=target, measured=measured, plan=target, tip_length_m=facts[play.station]["tip_length_m"],
                        meta={"omega_max_rad_s": facts[play.station]["omega_max_rad_s"],
                              "uncontrolled_rows": int(np.count_nonzero(tape & (mode != MODE_POSITION)))}))
    return tuple(runs)


def pendulum_runs(source: str = "real") -> tuple[Run, ...]:
    """Round 8's plays (``source="real"``), or the twin's answer to each (``"sim"``; a play without one is left out)."""

    if source not in ("real", "sim"):
        raise ValueError(f"source is 'real' or 'sim', not {source!r}")
    return _real_runs() if source == "real" else _sim_runs()


# --------------------------------------------------------------------------- the rigid chain: the five-bar ladder


def _tool(t: np.ndarray, xy: np.ndarray) -> PoseTrajectory:
    matrices = np.repeat(np.eye(4)[None], len(xy), axis=0)
    matrices[:, 0, 3], matrices[:, 1, 3] = xy[:, 0], xy[:, 1]
    return PoseTrajectory.from_matrices(t, matrices, child_frame_id="L2 tip")


@functools.cache
def _chain(source: str) -> tuple[Run, ...]:
    """bam_fivebar.speed_ladder's steps (summary.json): the tape's samples as the ladder's own step_comparisons cut it."""

    summary = json.loads((CHAIN_LADDER / "summary.json").read_text(encoding="utf-8"))
    runs = []
    for step in summary["steps"]:
        relative = step.get(source)
        if step.get("kind") != "rung" or not relative:
            continue
        with np.load(CHAIN_LADDER / relative, allow_pickle=False) as archive:
            arrays = {name: np.asarray(archive[name]) for name in archive.files}
        t = np.asarray(arrays["t"], float)
        t = t - t[0]
        count = min(len(arrays["tape_t"]), len(t)) if "tape_t" in arrays else len(t)
        header = _header("five-bar plane", source=str(CHAIN_LADDER / relative))
        target = JointTrajectory(t=t[:count], joint_names=CHAIN_JOINTS,
                                 positions=np.asarray(arrays["target_motors"], float)[:count], header=header)
        measured = JointTrajectory(t=t[:count], joint_names=CHAIN_JOINTS,
                                   positions=np.asarray(arrays["measured_motors"], float)[:count], header=header)
        runs.append(Run(id=f"{source}:{step['name']}", deployment=CHAIN_DEPLOYMENT, station="bam_fb five-bar",
                        part_type="rigid_body_chain", source=source,
                        labels={"rung": step["name"], "speed_m_s": step["speed_m_s"], "ff_mode": step["ff_mode"],
                                "gravity_mode": step["gravity_mode"]},
                        target=target, measured=measured,
                        tool_target=_tool(t[:count], np.asarray(arrays["target_tool"], float)[:count]),
                        tool_measured=_tool(t[:count], np.asarray(arrays["measured_tool"], float)[:count]),
                        faulted=step.get("result") != "ran"))
    return tuple(runs)


def chain_runs(source: str = "real") -> tuple[Run, ...]:
    """The five-bar's real speed-ladder rungs (``"real"``) or its sim twin's (``"sim"``)."""

    if source not in ("real", "sim"):
        raise ValueError(f"source is 'real' or 'sim', not {source!r}")
    return _chain(source)
