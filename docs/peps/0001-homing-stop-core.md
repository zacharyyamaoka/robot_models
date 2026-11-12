# PEP 0001: The homing stop core lives in robot_models

- **Status:** accepted, 2026-11-12
- **Decision report:** `/home/bam/bam_ws/reports/homing-subassembly-proposals-2026-11-12.html` (builder `bam_ws/docs/homing/build_proposals.py`). It recommends B, "two verbs, one stop core": `home()` on every boot with a stored backlash table, `identify_backlash()` at calibration, both on one pure stop core, built on C's homing bench, with A's one-routine procedure as a rival leaderboard row.
- **Layering:** the Robot Models Layering decision (vault, `Projects/robot_msgs/Robot Models Layering — Decision (2026-11-12).md`): `robot_msgs <- robot_models <- robot_description <- robotparts`, with sim2real as the outer API.

## Decision

1. **Home.** The pure stop core goes in a new local repo, `/home/bam/robot_models` (package `robot_models`, PyPI name `robot-models`). On 2026-11-12 no peer owned it: the path did not exist, no sim2real or bam_ws worktree or branch named it, and nothing imported it. It is local only, with no remote. The first public push is Zach's call.
2. **Scope.** Only `robot_models.homing` lives here so far. The decision note's move plan (metrics, fits, type cards) is separate and still gated on its step 0.
3. **Dependencies.** The core is `robot-msgs` + numpy. The `contact` extra adds coal 3.0.2 (the version bam_pick_worlds already locks through pin 3.8.0) and trimesh. `tests/test_import_direction.py` enforces this with an AST scan, three planted violations held red, and a runtime check that importing the core loads no contact library. `uv run --isolated` without the extra imports the core.

## What is here

| Name | What it is |
|---|---|
| `StopSide`, `StopMap` | One joint side's stop angle (`None` = never stops, never a silent 0), with basis `cad`, `typed` or `measured` |
| `BacklashModel` | The ladder. B0: none. B1: `gear_backlash` dead zone (`method="deadzone"`, `zero="center"`) in its rigid limit. B2: B1 plus a lumped series compliance (mesh + stop + structure; only the sum is identifiable without an output sensor) |
| `JointBacklash`, `BacklashTable` | Per-joint model plus observable flags; `home()` reads it on boot, `identify_backlash()` writes it |
| `StallEvidence` | One settled stall: joint, side, torque magnitude, raw motor-side encoder |
| `JointZero`, `HomingResult` | Zero per joint (`q = raw + offset`) with its repeat spread; `corrected(table)` re-fits the stored stalls without moving |
| `fit()` | Pure, numpy: stalls in, zero + backlash out. Compliance comes from two torque levels, the gap from both sides minus the CAD span (sim only: on real a CAD span error aliases into the gap) |
| `contact.first_contact()` | Extra: the exporter's algorithm 2 (`cad2robot/hardstops.py`, python-fcl) ported to coal. Conservative advancement capped at 1°, then bisection to 0.05°/8. It returns the last angle verified clear |
| `cad_bundle` | Extra: reads a cad2robot bundle's `hardstops.json` and places the named visual meshes from the exporter URDF |

**Sign convention** (as `gear_backlash`): `twist = q_link - q_motor`. Pressing into the `side` stop with torque `T` leaves the motor past the link by `side * (gap/2 + compliance*T)`. So `raw = q_stop + side*(gap/2 + c*T) - offset`.

**Touching criterion.** A stop whose `hardstops.json` entry has a `distance_mm` uses the exporter's proximity `contact_mm` (0.05 mm). The loop-joint sweeps (revolute_6 and revolute_7, `kind` `own_frame` / `interference`, `distance_mm: null`) used overlap. A 0.05 mm proximity there lands early by exactly 0.05 mm / lever: 0.073° on revolute_6 (lever 39 mm), and 0.037° to 0.043° on revolute_7 (lever 79 mm). That is how the prototype found it.

## The ruler (committed before the code it judges)

`tests/test_stop_ruler.py` and `tests/test_fit_ruler.py`, marked `ruler`:

- coal `first_contact` on the visual meshes `hardstops.json` names reproduces the v11 five-bar bundle 8957130: revolute_3 ±89.975°, revolute_6 −90.03° / +125.00°, revolute_7 −112.69° / +197.77°, revolute_8 and revolute_34 null. It also reproduces the wrist bundle 583ee39 (revolute_11 ±169.85°, revolute_14 and revolute_17 ±169.849°). The tolerance is 0.05°.
- A strict xfail at 0.01° on the exporter's `stop_mesh` channel: it flips once the exporter emits `stops.json` and the stop meshes (exporter change request in the report).
- Mutations that must go red: delete a stop mesh, shift a stop by 0.1°, double the hidden gap.
- `hardstops.json` sha256 is pinned, so a re-export cannot quietly move the ruler.

## Not here (later steps)

The kinematic `HomingPlant`, procedures as `plan()`/`observe()` black boxes emitting a robot_msgs `JointCommandSequence` (run by the `JointCommandSequenceController`), the bench, the leaderboard, the MuJoCo rung and the real rung.
