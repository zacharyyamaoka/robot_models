# PEP 0004: Model ladders by type — one vocabulary per type, rungs as a tree of bases

- **Status:** accepted as the v1 contract, 2026-11-16; amended the same night after three adversarial reviews
  (fidelity, breakage, physics: §13). Built: the core (`robot_models.ladder`), the citable works
  (`robot_models.references`), the five group packages, `.gitattributes`, and their contract tests. To build: the 12
  type ladders and the rung move, by the implementation lanes, judged by `tests/test_ladders_acceptance.py`
  (committed red before they start; off-limits to them).
- **Amended:** 2026-11-16, amendment 1 (§14): the physics errors the v1 check agents found in the frozen spec, fixed
  in the ruler first (`87db875`) and then in the ladders, this PEP and the generated docs. 83 → 84 rungs.
  2026-11-17, amendment 2 (§15): a type is one object, `class IntegratedServo(PartType)`, holding its models as
  classes; the datasheet class becomes `IntegratedServoDatasheet`. Built for `integrated_servo` only.
- **Ask:** Zach, 2026-11-16: "please start an ultracode workflow to implement robot_models as discussed. It can be a
  public repo." "As discussed" means the vault note `Projects/robot_msgs/Robot Models — Thinking to Date (2026-11-16).md`
  (its open questions, each with its default) and `Robot Models Layering — Decision (2026-11-12).md`.
- **Follows:** PEP 0003 §6 ("a ladder per type in one file beside the type ... a rung gets its own module only when it
  has code"), and sim2real's `docs/decision-rung-keys-2026-11-05.md` (rungs numbered by robotparts' layers).
- **Layering:** `robot_msgs ← robot_models ← robot_parts ← robot_descriptions`, with sim2real on top. robot_models
  imports the stdlib, numpy and robot_msgs only; the ladder core imports the stdlib only.
- **Settled before this PEP** (the dispatcher's decisions, not reopened here):
  1. One vocabulary per type. Canonical names are the typed-field names, robotparts' registry keys and robotparts'
     rung terms, never renamed. Every other name is an alias, with a note when it is not the same quantity one for one.
  2. Folders by type, mirroring robotparts' `user/parts`. One `ladder.py` per type folder. The eight robotparts rungs
     move byte for byte; robotparts keeps a vendored copy.
  3. Rungs numbered by robotparts' layer, M6 residual and M7 learned, `.wear` variants; M1 is the Datasheet's typed
     fields. A rung carries its level and name explicitly. The ladder is a tree of explicit bases.
  4. Type data only. Config and assembly parameters belong to a deployment, and the driver's parameters to
     `foc_controller`.
  5. Plain frozen dataclasses, stdlib only.
  6. sim2real gains one additive parity test, nothing else.
  7. Public repo, MIT.
  8. `parameters(rung)` is cumulative through the bases, for the coverage consumer.

## 0. Two flags for the dispatcher

1. **Decisions 3 and 8 together gave the coverage consumer a wrong answer, so the contract now separates what a rung
   needs from what it states.** With "M1 = every typed field" and "M2 → M1", `parameters("M2.coulomb_viscous")` was
   35 names on the servo, `flange_size` and `max_radial_load` among them, while robotparts' coverage ruler
   (`tools/part_coverage.py`, "only 100% coverage and accuracy of the specific terms we are looking for") asks 9 up to
   M2. Both decisions are kept literally:
   - a rung's `adds` are what its model **needs**, its `optional` what it **reads when a source states it**;
   - `M1.datasheet` holds every typed field (decision 3) as optional, which is what robotparts' own `M1.datasheet`
     says of its terms (`required: false` on all seven);
   - `parameters(key)` is the needed names, cumulative through the bases (decision 8); `holds(key)` adds the optional
     ones; a name stated at M1 becomes needed where a rung needs it (`coulomb_friction` at M2).

   Undo: drop `Rung.optional` and move the names back into `adds`; the reviewers' alternative (M1 adds only robotparts'
   seven terms, the other typed fields vocabulary-only) is one more undo away.
2. **12 types, not the brief's 13.** `arm` is folded into `rigid_body_chain`: Zach, Nov 6, `PROJECT - Sim2Real`
   `^s2r-chain-type`: "the 6 DOF robot arm is not a type.. the type here is more generic as I think the same model
   works for different types of chans", and sim2real's own v3 already maps both `arm_6dof` and `arm_subassembly` to
   `rigid_body_chain`. All 13 sim2real cards still map onto a type and a rung (§8.2). Undo: split `arm/` back out of
   `rigid_body_chain` with the arm's branch (`M2.geometric`, `M3.compliance`, `M3.wear`, `M6.residual`).

## 1. The answer in nine lines

1. **Each type folder holds one ladder, and each ladder holds one vocabulary.** `<type folder>/ladder.py` defines
   `LADDER = Ladder(...)`: the type's parameters (canonical name, SI unit, shape, definition), its rungs, its aliases,
   its packed names, and the names that belong to a deployment rather than the type. `robot_models.ladder.ladders()`
   finds the ladders by walking the package; there is no registry for parallel lanes to edit.
2. **A rung extends one base, and needs everything its base needs.** `ladder.parameters("M3.lugre")` is what the
   rung needs, inherited names included (the coverage KPI's question); `ladder.holds(...)` adds what it reads when
   stated; `ladder.fallback(...)` is the chain it falls back down; `needs(ladders, type, key)` adds the composed
   types' names (a servo's driver delay). A key is `M{level}.{name}`, composed from two fields and never parsed.
3. **Each rung shows its parameters, its equation and its references** (Zach, Nov 5; settled point 3 of the
   Thinking-to-Date note). `Rung.equation` and `Rung.references` are on every rung of the ruler's spec, citing
   `robot_models.references`: 55 works copied from sim2real's Crossref- and web-checked list, less a maker's manual,
   a maker's page and one rig's code (decision 4).
4. **12 types, 84 rungs** (§5), each in a folder that mirrors `user/parts`.
5. **A type with a Datasheet class states its typed fields at M1.** On the three servo types `M1.datasheet` holds
   exactly the typed fields, as optional names, and needs nothing beyond M0; the core refuses a ladder where it does not.
6. **Every other name is one of 117 aliases or 9 packed names** (§6). 43 aliases are not the same quantity one for
   one and carry a note; 24 carry the unit their source writes (`od_mm` is mm, `camera_latency_ms` is ms). A packed name
   (MuJoCo's `thermal`, the rack twin's `thermal_r_c`) is never read as one parameter: `canonical()` refuses it and
   `unpack()` names what it carries.
7. **robotparts' eight actuator rungs move here byte for byte**, to `actuators/rungs/<word>/`, as the integrated
   servo's code; `.gitattributes` keeps a Windows checkout from rewriting their bytes. robotparts keeps a
   byte-identical vendored copy behind a sync check (§8.1).
8. **sim2real stays as it is apart from one additive parity test, and the parity rule now checks rungs, not just
   names** (§8.2): same level; the card rung's base in the mapped rung's fallback; every card parameter held by the
   mapped rung, a composed type or the deployment list; every same-named parameter in its canonical unit or in a
   stated unit table. A scratch run against sim2real main @ `f4d84c8`: 13 cards, 76 rungs, no problems. The same run
   with the first draft's servo map (M5 → `M5.dcmotor`) finds 10.
9. **The ruler is red today for the right reasons: 114 failing, 6 passing.** Every failure is a missing type
   `ladder.py` or rung folder. A reference implementation generated from the spec, in a scratch copy, turns all 161
   tests green (contract, ruler and import direction); five planted mistakes (an extra file in a rung folder, a wrong
   base, a maker in a title, a missing reference, an empty equation) each turn exactly one test red.

## 2. What the brief assumed, and what holds

- *"The robotparts rung terms are already registry keys."* This holds for the key terms of M0 to M4. It does not hold
  for `stribeck_exponent`, `sigma0`, `sigma1`, `sigma2` and `harmonics`, terms each rung declares itself: the single
  quantities stay canonical; `sigma2` is an alias (robotparts' own definition calls it the other kinds'
  `viscous_friction`). `M5.dcmotor`'s nine terms are MuJoCo's packed arrays (`unit: mujoco`), the fork in §3.
- *"The eight rung folders hold model.json, README.md, model.py and model.js."* Seven do. `dcmotor` holds only
  `model.json` and `README.md`, because only MuJoCo implements it (its README says so). The ruler expects each
  folder's own file set.
- *"13 sim2real types."* sim2real has 13 cards but 12 v3 types, and Zach said the arm is not a type (§0, flag 2).
- *"M1 is the typed fields" with "parameters() for the coverage KPI".* Both hold once a rung's names split into
  needed and stated (§0, flag 1).
- *"Rung keys are never parsed."* This holds. A Rung has `level` and `name`, and `key` is composed from them.

## 3. The one open fork: the MuJoCo arrays in `M5.dcmotor` (re-scored after review)

`dcmotor`'s terms are how MuJoCo packs quantities the vocabulary already names one at a time: `motorconst` (2
numbers), `resistance` (1), `nominal` (3), `saturation` (3), `inductance` (2), `cogging` (3), `controller` (6),
`thermal` (6), `lugre` (5).

**Criteria and weights:**

| # | Criterion | Weight |
|---|---|---:|
| C1 | One quantity, one name in the vocabulary | 40 |
| C2 | The moved folder stays byte-identical, and every term in it still resolves | 25 |
| C3 | No silent conflation: a pair or an array never lands in a scalar's row | 20 |
| C4 | Small: no new machinery | 15 |

Each criterion is scored 1–5, and Total = Σ weight × score / 5.

| Option | C1 | C2 | C3 | C4 | Total |
|---|---|---|---|---|---:|
| A. The arrays are canonical: nine more parameters on M5 | 1 | 5 | 1 | 4 | 49 |
| B. Each array is an alias of one scalar it packs (the first draft) | 5 | 5 | 1 | 4 | 81 |
| **C. A `Packed` record (name → the canonical quantities it carries); `canonical()` refuses it** | 5 | 5 | 5 | 3 | **94** |

- **Why B lost on review.** The draft scored B's C3 a 4. The fidelity review showed `canonical("thermal_r_c")`
  returning `thermal_resistance`, so a parameter × source table would sort a (K/W, s) pair into the scalar's row; and
  `lugre → sigma0` is a mapping no source states. C3 is a 1.
- **C as built:** `Packed(name, holds, note, ordered)`. `ordered=True` only where the entry order is sourced (sim2real's
  `thermal_r_c`, `thermal` and `board_thermal` are "K/W, s"); MuJoCo's arrays are `ordered=False` and name only what a
  source says they carry. `resistance` (one number, Ω) stays a plain alias; `controller` and `saturation` are
  deployment-only (the drive's settings).
- **Not verified:** MuJoCo 3.16's own entry layout for each array (only 3.11 is installed here). Every MuJoCo note
  says so.
- **Undo:** turn a `Packed` back into an `Alias` of its first entry.

## 4. The core: `robot_models.ladder` (stdlib only)

| Record | Field | Meaning |
|---|---|---|
| `Parameter` | `name` | the canonical name: a registry key, a typed-field name or a robotparts rung term, else sim2real's name without its unit; never a unit suffix |
| | `unit` | the SI unit (`""` when dimensionless); a wear rate is per its usage unit (N·m/h, N/km) |
| | `definition`, `shape`, `note` | one line; scalar, integer, text, pair, list, map, pose, per joint or per link |
| `Rung` | `level` | 0–7. Physics levels follow robotparts' layers: 0 ideal, 1 datasheet, 2 Coulomb + viscous, 3 state-dependent friction, 4 drivetrain, 5 electrics and heat. 6 is the residual, 7 the learned net |
| | `name`, `key` | a word unique among the rungs of its level; `key` = `M{level}.{name}`, a property: composed, never parsed |
| | `title`, `base` | one line; the key of the rung it extends and falls back on (`None` only for the root M0 and a learned net) |
| | `adds` | what the rung's model **needs** that its base does not (new, or stated below and needed from here) |
| | `optional` | what it **reads when a source states it** (a default or another route covers it otherwise) |
| | `kind`, `code` | physics (0–5), residual (6), learned (7); a folder under `robot_models` with runnable code, else `""` |
| | `wears` | `Wear` records: which held parameter grows with which usage, by which law, at which rate the rung adds |
| | `composes` | `(type, rung key)` pairs: the other types' rungs it needs, such as a servo's driver |
| | `equation`, `references`, `note` | the model, in canonical names; `Reference`s from `robot_models.references` |
| `Wear` | `parameter`, `usage`, `usage_unit`, `law`, `rate` | `law` is one of linear, sqrt, archard |
| `Reference` | `title`, `url`, `kind`, `doi`, `note` | sim2real's shape; a title is required |
| `Alias` | `name`, `canonical`, `note`, `unit` | a note whenever not the same quantity one for one; `unit` is the unit its source writes (`""` = the canonical SI unit) |
| `Packed` | `name`, `holds`, `note`, `ordered` | an array of several canonical quantities; `ordered` when `holds` is its entry order |
| `DeploymentOnly` | `name`, `role`, `why` | `role` is config or assembly; a driver's parameter belongs to `foc_controller` instead |
| `Ladder` | `type`, `title`, `vocabulary`, `rungs`, `aliases`, `packed`, `deployment`, `datasheet`, `note` | `datasheet` is the type's Datasheet class, or `None` |

**What a Ladder answers:** `keys`, `names`, `rung(key)`; `fallback(key)`; `parameters(key)` (needed, cumulative, root
first); `optional(key)`; `holds(key)`; `composed(key)` (`{type: rung}`, the nearest rung wins); `canonical(name)`,
`unpack(name)`, `resolve(name)` (a tuple of canonical names for any name the type knows), `parameter(name)`,
`alias(name)`, `packing(name)`. Across ladders: `needs(ladders, type, key)` → `{type: parameters}` with the composed
types read at their composed rungs.

**What it refuses**, each with a message that says how to fix it:
- **Keys and bases:** a duplicate key; an unknown base (the message names the nearest key); a base on a higher layer;
  zero roots, or two; bases that loop.
- **Names on a rung:** a name outside the vocabulary (the message names the nearest); a needed name added again down
  its own chain; an optional name already held below; a name both needed and optional; a wear whose rate the rung does
  not add, or whose parameter it does not hold; a vocabulary entry no rung adds or lists as optional.
- **Aliases, packed and deployment names:** an alias that is canonical, points outside the vocabulary, or repeats; a
  packed name that is canonical or an alias, or carries a name outside the vocabulary; a deployment-only name that is
  the type's own; a unit inside a canonical name; `canonical()` of a packed name.
- **Levels and kinds:** a residual anywhere but M6; a learned net anywhere but M7, or with a base; a physics rung
  above M5; named parameters on a residual or a learned net; a reference that is not a `Reference`.
- **With a `datasheet`:** a typed field missing from the vocabulary or in another SI unit; an `M1.datasheet` that does
  not hold exactly the typed fields.

**Discovery:** `discover()` (and `ladders()`) walks the package for `ladder.py` files, skipping the core's own,
imports `<package>.<folder>.ladder` and takes its `LADDER`. It refuses two ladders of one type and a composition that
names a missing type or rung. It clears the import system's folder cache first, so a folder made seconds ago is found
(this was a real flake in the first draft's discovery test). `Found.folder` is the join key a consumer such as
robotparts' coverage tool uses (its `rgbd_camera` is this `depth_camera`, both at `sensors/rgbd`).

**How a ladder reads** (the encoder, as the reference implementation writes it; keyword arguments, so a lane copies a
readable form):

```python
"""An angle sensor on a shaft, such as a drive's rotor encoder: its ladder of models (PEP 0004)."""

from __future__ import annotations

from robot_models.ladder import Alias, Ladder, Parameter, Rung, Wear
from robot_models.references import (
    JUNKER_1969,
    MERRY_2010,
    WHITMORE_1995,
    WIDROW_2008,
    ZHAO_2024,
)

VOCABULARY = (
    Parameter("counts_per_rev", "1/rev",
              "Distinct readings per revolution of the shaft it reads: its quantisation.", shape="integer"),
    Parameter("noise_sigma", "rad",
              "Standard deviation of the reading's noise at standstill."),
    Parameter("latency", "s",
              "Delay from the shaft's angle to the reading that reports it."),
    Parameter("eccentricity_harmonics", "rad",
              ("Angle error periodic in the shaft angle (an eccentric magnet or disc), a list of (harmonic,"
               " amplitude, phase)."), shape="list"),
    Parameter("eccentricity_growth", "rad/√h",
              "How fast the eccentricity grows with the square root of vibration hours (a loosening mount)."),
)

RUNGS = (
    Rung(
        level=0,
        name="quantiser",
        title="An ideal quantiser",
        base=None,
        adds=("counts_per_rev",),
        equation="q_reported = round(q/lsb)·lsb, lsb = 2π/counts_per_rev",
        references=(WIDROW_2008,),
    ),
    Rung(
        level=1,
        name="noise_latency",
        title="Noise and latency",
        base="M0.quantiser",
        adds=("noise_sigma", "latency"),
        equation="q_reported(t) = round((q(t − latency) + n)/lsb)·lsb, n ~ N(0, noise_sigma²)",
        references=(MERRY_2010,),
    ),
    Rung(
        level=2,
        name="eccentricity",
        title="Eccentricity harmonics",
        base="M1.noise_latency",
        adds=("eccentricity_harmonics",),
        equation="q_reported = round((q + e(q) + n)/lsb)·lsb, delayed; e(q) the eccentricity harmonics",
        references=(ZHAO_2024,),
    ),
    Rung(
        level=2,
        name="wear",
        title="A mount that loosens over vibration hours",
        base="M2.eccentricity",
        adds=("eccentricity_growth",),
        wears=(Wear("eccentricity_harmonics", "vibration hours", "h", "sqrt", "eccentricity_growth"),),
        equation="e(h) = e + eccentricity_growth·√h",
        references=(JUNKER_1969, WHITMORE_1995),
    ),
)

LADDER = Ladder(
    type="encoder",
    title="An angle sensor on a shaft, such as a drive's rotor encoder.",
    vocabulary=VOCABULARY,
    rungs=RUNGS,
    aliases=(
        Alias("latency_s", "latency"),
        Alias("eccentricity_growth_mrad_per_sqrt_h", "eccentricity_growth", unit="mrad/√h"),
        Alias("encoder_counts_per_rev", "counts_per_rev"),
        Alias("encoder_counts", "counts_per_rev"),
        Alias("encoder_lines", "counts_per_rev", "Lines: a quadrature encoder counts 4 × lines per revolution."),
        Alias("encoder_bits", "counts_per_rev", "Bits: 2^bits counts per revolution."),
    ),
)
```

## 5. The type table

Source of every rung: sim2real `src/sim2real/cards/ladders.py`, read live from `registry.CARDS` on main @ `f4d84c8`
(rungs, parameters, units, equations, references); robotparts `models/actuator/*/model.json` @ `d16f6a8` (terms,
required flags, equations), unchanged at `e742ba0`; the rack twin's term list (robotparts `lab/`, structure only).
Each parameter is translated to its canonical name. The tables below are generated from the ruler's spec (`TYPES` in
`tests/test_ladders_acceptance.py`), so this PEP and the ruler cannot disagree. The ruler asks each rung for a
non-empty equation and at least the references named here; the equation shown is the spec's form.

### 5.1 The twelve types

| type | folder (under `src/robot_models/`) | Datasheet class | sim2real card(s) | robotparts category | rungs |
|---|---|---|---|---|---:|
| `integrated_servo` | `actuators/servo/` | `IntegratedServo` | `servo` | `actuator` | 14 |
| `bldc_motor` | `actuators/servo/bldc/` | `BldcMotor` | `motor` | `actuator` | 9 |
| `gearbox` | `actuators/servo/gearbox/` | `Gearbox` | `gearbox` | `gearbox` | 8 |
| `foc_controller` | `actuators/servo/foc_controller/` | — | `driver` | `driver` | 6 |
| `encoder` | `sensors/encoder/` | — | `encoder` | `sensor` | 4 |
| `camera` | `sensors/camera/` | — | `camera_rgb` | `camera` | 6 |
| `depth_camera` | `sensors/rgbd/` | — | `camera_rgbd` | `camera` | 5 |
| `hyperspectral_camera` | `sensors/hyperspectral_camera/` | — | `camera_hsi` | `camera` | 5 |
| `rigid_body_chain` | `arms/rigid_body_chain/` | — | `arm_6dof`, `arm_subassembly` | `arm` | 10 |
| `gripper` | `end_effectors/gripper/` | — | `arm_6dof_gripper` | `gripper` | 6 |
| `conveyor` | `conveyors/conveyor/` | — | `conveyor` | — | 7 |
| `aruco_marker` | `markers/aruco_marker/` | — | `aruco_marker` | — | 4 |
| | | | | **total** | **84** |

### 5.2 `integrated_servo`: `actuators/servo/`

A joint in one housing: a motor, its drive, a reducer or none, and an encoder. · M1 states every `IntegratedServo` field (35) · sim2real `servo`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.ideal`: The ideal actuator: the stated inertia and ratio, no losses | — | physics | `output_inertia`, `gear_ratio` | — | `actuators/rungs/ideal/` | — | tau_r = 0; armature = output_inertia | SICILIANO_2009, MUJOCO_ARMATURE |
| `M1.datasheet`: The datasheet: the stated values, nothing fitted. *states every IntegratedServo field and needs none beyond M0 (robotparts' M1: required false).* | `M0.ideal` | physics | — | every other typed field | `actuators/rungs/datasheet/` | — | tau_r = coulomb_friction·sign(v) + viscous_friction·v when stated; armature = output_inertia, or gear_ratio²·(rotor_inertia + gearbox_input_inertia) | ARMSTRONG_1994, MUJOCO_ARMATURE |
| `M2.coulomb_viscous`: Coulomb and viscous friction, fitted | `M1.datasheet` | physics | `coulomb_friction`, `viscous_friction` | — | `actuators/rungs/coulomb_viscous/` | — | tau_r = coulomb_friction·sign(v) + viscous_friction·v | ARMSTRONG_1994, OLSSON_1998 |
| `M3.stribeck`: Stribeck friction: breakaway falling to Coulomb with speed | `M2.coulomb_viscous` | physics | `breakaway_torque`, `stribeck_velocity` | `stribeck_exponent` | `actuators/rungs/stribeck/` | — | tau_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(\|v\|/stribeck_velocity)^stribeck_exponent))·sign(v) + viscous_friction·v | ARMSTRONG_1994, OLSSON_1998 |
| `M3.load`: Friction that grows with the transmitted load | `M2.coulomb_viscous` | physics | `friction_load_coefficient` | — | `actuators/rungs/load/` | — | tau_r = (coulomb_friction + friction_load_coefficient·\|tau_load\|)·sign(v) + viscous_friction·v | KIRCANSKI_1997, OLSSON_1998 |
| `M3.lugre`: LuGre dynamic friction | `M3.stribeck` | physics | `sigma0`, `sigma1` | — | `actuators/rungs/lugre/` | — | dz/dt = v − sigma0·\|v\|·z/g(v), g(v) the Stribeck level of M3.stribeck; tau_r = sigma0·z + sigma1·dz/dt + viscous_friction·v (LuGre's sigma2) | CANUDAS_1995 |
| `M3.weakening_capture`: Stribeck with weakening and a capture level at rest. *sim2real's servo M3, the rack twin's friction: its structure only, the twin is never published.* | `M3.stribeck` | physics | `friction_weakening`, `friction_weakening_limit`, `friction_capture`, `friction_capture_speed`, `friction_capture_dwell` | — | — | — | level(v) = breakaway_torque − friction_weakening·min(\|v\|, friction_weakening_limit) − (breakaway_torque − coulomb_friction)·(1 − exp(−(\|v\|/stribeck_velocity)²)); friction_capture holds below friction_capture_speed until friction_capture_dwell has passed | ARMSTRONG_1994, CANUDAS_1995 |
| `M4.ripple`: Rotor ripple on Coulomb and viscous friction | `M2.coulomb_viscous` | physics | `harmonics` | — | `actuators/rungs/ripple/` | — | tau_r = Σ_k amplitude_k·sin(rotor_harmonic_k·gear_ratio·θ + phase_k) + coulomb_friction·sign(v) + viscous_friction·v | JAHNS_1996 |
| `M4.drivetrain`: The drivetrain: ripple, backlash and compliance on the full friction model. *sim2real's servo M4, the rack twin (today's model), less its heat: that is M5.envelope's.* | `M3.weakening_capture` | physics | `harmonics`, `backlash`, `torsional_stiffness` | — | — | `foc_controller` `M1.delay_loop` | output_inertia·q̈ = tau_drive − tau_r; tau_r = level(v)·sign(v) + viscous_friction·v + the harmonics; the output across a backlash gap and torsional_stiffness; tau_drive from foc_controller M1.delay_loop | JAHNS_1996, NORDIN_2002, TUTTLE_1996, KRISHNAN_2010 |
| `M5.dcmotor`: MuJoCo's DC motor: electrics, heat, cogging and LuGre friction. *robotparts' rung: its terms are MuJoCo's arrays (the packed names); motorconst and resistance required, the rest optional, as its model.json marks them.* | `M3.lugre` | physics | `kt`, `resistance_line_to_line` | `harmonics`, `thermal_resistance`, `thermal_time_constant` | `actuators/rungs/dcmotor/` | — | MuJoCo's own: mjDYN_DCMOTOR (electrical dynamics), mjGAIN_DCMOTOR (K or K/R), mjBIAS_DCMOTOR (back-EMF, cogging, LuGre friction) | KRAUSE_2013, MUJOCO_DCMOTOR |
| `M5.envelope`: The drive's ceiling, the motor's electrics and heat on the drivetrain. *sim2real's servo M5 (the envelope ladder): the rack twin plus the drive's ceiling and the motor's electrics; the heat the twin carries at its M4 sits here, robotparts' layer 5.* | `M4.drivetrain` | physics | `kt`, `resistance_line_to_line`, `thermal_resistance`, `thermal_time_constant` | — | — | `foc_controller` `M2.ceiling_heat` | kt_out = kt·gear_ratio, R = resistance_line_to_line/2 per phase: motoring torque ≤ kt_out·(voltage_available − \|v\|·kt_out/1.5)/R; any torque ≤ kt_out·I_max (the deployment's current cap); winding rise ΔT over ambient: thermal_time_constant·dΔT/dt = thermal_resistance·1.5·i²R − ΔT (the three phases' copper loss, i the phase-current amplitude) | KRAUSE_2013, KRISHNAN_2010, MUJOCO_DCMOTOR |
| `M5.wear`: Wear: friction grows with hours, backlash opens with load. wears `breakaway_torque` by linear in deployment hours (h) at `friction_growth`; wears `coulomb_friction` by linear in deployment hours (h) at `friction_growth`; wears `backlash` by archard in torque-weighted output revolutions (N·m·rev) at `backlash_wear` | `M5.envelope` | physics | `friction_growth`, `backlash_wear` | — | — | — | backlash(u) = backlash + backlash_wear·Σ\|τ\|·Δrev; breakaway_torque and coulomb_friction both grow by friction_growth·h (the Stribeck drop stays) | ARCHARD_1953, FLODIN_1997, LU_MEEKER_1993 |
| `M6.residual`: Grey-box residual on M5.wear | `M5.wear` | residual | — | — | — | — | M5.wear + an MLP residual torque from 3 samples of position error, velocity and torque command | NGUYEN_TUONG_2010, ZENG_2020, KOVALEV_2026 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 3 samples of position error and velocity → output torque | HWANGBO_2019, HWANGBO_2019_ARXIV, ZHU_2025 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `breakaway_torque` | N·m | scalar | `M3.stribeck` | Static friction at the output: the torque that starts it moving from rest, the top of the Stribeck curve. |
| `stribeck_velocity` | rad/s | scalar | `M3.stribeck` | Output speed over which friction falls from breakaway_torque to coulomb_friction. |
| `stribeck_exponent` | — | scalar | `M3.stribeck` | Shape exponent of the Stribeck curve, how sharply friction falls with speed; 2 when absent. |
| `friction_load_coefficient` | — | scalar | `M3.load` | Extra output friction per N·m of load torque the part transmits: Fc + μ·abs(τ_load). |
| `sigma0` | N·m/rad | scalar | `M3.lugre` | LuGre bristle stiffness: output torque per radian of bristle deflection. |
| `sigma1` | N·m·s/rad | scalar | `M3.lugre` | LuGre bristle damping: output torque per rad/s of bristle deflection rate. |
| `friction_weakening` | N·m·s/rad | scalar | `M3.weakening_capture` | How fast the static friction level falls with output speed, up to friction_weakening_limit. |
| `friction_weakening_limit` | rad/s | scalar | `M3.weakening_capture` | The output speed up to which the friction level keeps weakening. |
| `friction_capture` | N·m | scalar | `M3.weakening_capture` | The friction level that holds a slowed output until it has stayed slow for friction_capture_dwell. |
| `friction_capture_speed` | rad/s | scalar | `M3.weakening_capture` | Below this output speed a joint counts as slow for capture. |
| `friction_capture_dwell` | s | scalar | `M3.weakening_capture` | How long a joint must stay slow before the static level returns. |
| `harmonics` | N·m | list | `M4.ripple` | Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, amplitude, phase): cogging and commutation ripple. |
| `torsional_stiffness` | N·m/rad | scalar | `M4.drivetrain` | Drivetrain stiffness at the output with the rotor held: output torque per radian of twist. |
| `thermal_resistance` | K/W | scalar | `M5.dcmotor` | Winding-to-ambient thermal resistance: winding temperature rise per watt of copper loss. |
| `thermal_time_constant` | s | scalar | `M5.dcmotor` | Time constant of the winding temperature's first-order rise. |
| `friction_growth` | N·m/h | scalar | `M5.wear` | How fast the friction level grows per deployment hour (seals, grease, bearings); on a Stribeck curve breakaway and Coulomb grow together. |
| `backlash_wear` | rad/(N·m·rev) | scalar | `M5.wear` | How fast backlash opens per torque-weighted output revolution (Archard). |

### 5.3 `bldc_motor`: `actuators/servo/bldc/`

A brushless motor alone, at its shaft. · M1 states every `BldcMotor` field (28) · sim2real `motor`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.ideal`: An ideal torque source: kt·i on the rotor inertia. *robotparts' M0.ideal read as a 1:1 actuator, whose output is the rotor.* | — | physics | `kt`, `rotor_inertia` | — | — | — | τ = kt·i; rotor_inertia·ω̇ = τ − τ_load | SICILIANO_2009, KRAUSE_2013 |
| `M1.datasheet`: The datasheet: the stated values, nothing fitted | `M0.ideal` | physics | — | every other typed field | — | — | τ = kt·i on rotor_inertia, less coulomb_friction·sign(ω) + viscous_friction·ω when stated | KRAUSE_2013, ARMSTRONG_1994 |
| `M2.coulomb_viscous`: Coulomb and viscous bearing friction. *the actuator's M2 on a bare motor: robotparts' actuator rungs cover its category, and its coverage ruler asks a BLDC motor's M2 for both.* | `M1.datasheet` | physics | `coulomb_friction`, `viscous_friction` | — | — | — | τ_r = coulomb_friction·sign(ω) + viscous_friction·ω | ARMSTRONG_1994, OLSSON_1998 |
| `M3.stribeck`: Stribeck friction: breakaway falling to Coulomb with shaft speed. *the actuator's M3.stribeck on a bare motor, whose output is the shaft: robotparts' coverage ruler asks a BLDC motor's M3 for both.* | `M2.coulomb_viscous` | physics | `breakaway_torque`, `stribeck_velocity` | `stribeck_exponent` | — | — | τ_r = (coulomb_friction + (breakaway_torque − coulomb_friction)·exp(−(\|ω\|/stribeck_velocity)^stribeck_exponent))·sign(ω) + viscous_friction·ω | ARMSTRONG_1994, OLSSON_1998 |
| `M4.ripple`: Cogging: rotor-periodic ripple. *sim2real's motor M4; a datasheet's torque_ripple states one amplitude of it.* | `M3.stribeck` | physics | `harmonics` | — | — | — | τ = kt·i − Σ_k amplitude_k·sin(k·θ + phase_k) | JAHNS_1996 |
| `M5.dcmotor`: DC motor electrics and heat. *sim2real's motor M5 (today's model): the actuator's M5 on a bare motor, without LuGre friction.* | `M4.ripple` | physics | `resistance_line_to_line`, `inductance_line_to_line`, `thermal_resistance`, `thermal_time_constant` | — | — | — | τ = kt·i, i the phase-current amplitude; per phase on a wye winding: V = (resistance_line_to_line/2)·i + (inductance_line_to_line/2)·di/dt + (kt/1.5)·ω, the per-phase EMF amplitude; thermal_time_constant·dΔT/dt = thermal_resistance·(3/4)·resistance_line_to_line·i² − ΔT, ΔT the winding's rise over ambient (the three phases' copper loss) | KRAUSE_2013, KRISHNAN_2010, MUJOCO_DCMOTOR |
| `M5.wear`: Bearing friction that grows with hours. wears `breakaway_torque` by linear in deployment hours (h) at `friction_growth`; wears `coulomb_friction` by linear in deployment hours (h) at `friction_growth` | `M5.dcmotor` | physics | `friction_growth` | — | — | — | breakaway_torque and coulomb_friction both grow by friction_growth·h (the Stribeck drop stays) | LU_MEEKER_1993, ARMSTRONG_1994 |
| `M6.residual`: Grey-box residual on M5.wear | `M5.wear` | residual | — | — | — | — | M5.wear + an MLP residual shaft torque from 3 samples of q-axis current, rotor speed and winding temperature | NGUYEN_TUONG_2010, ZENG_2020 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 3 samples of q-axis current command and rotor speed → shaft torque | HWANGBO_2019, HWANGBO_2019_ARXIV |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `breakaway_torque` | N·m | scalar | `M3.stribeck` | Static friction at the output: the torque that starts it moving from rest, the top of the Stribeck curve. |
| `stribeck_velocity` | rad/s | scalar | `M3.stribeck` | Output speed over which friction falls from breakaway_torque to coulomb_friction. |
| `stribeck_exponent` | — | scalar | `M3.stribeck` | Shape exponent of the Stribeck curve, how sharply friction falls with speed; 2 when absent. |
| `harmonics` | N·m | list | `M4.ripple` | Rotor-periodic torque harmonics at the output, a list of (rotor_harmonic, amplitude, phase): cogging and commutation ripple. |
| `thermal_resistance` | K/W | scalar | `M5.dcmotor` | Winding-to-ambient thermal resistance: winding temperature rise per watt of copper loss. |
| `thermal_time_constant` | s | scalar | `M5.dcmotor` | Time constant of the winding temperature's first-order rise. |
| `friction_growth` | N·m/h | scalar | `M5.wear` | How fast the friction level grows per deployment hour (seals, grease, bearings); on a Stribeck curve breakaway and Coulomb grow together. |

### 5.4 `gearbox`: `actuators/servo/gearbox/`

A gear stage between a motor and its load. · M1 states every `Gearbox` field (11) · sim2real `gearbox`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.ideal`: An ideal ratio and its reflected inertia. *the actuator's M0.ideal for a reducer.* | — | physics | `gear_ratio`, `gearbox_input_inertia` | — | — | — | q_out = q_in/gear_ratio; reflected inertia = gear_ratio²·gearbox_input_inertia | SICILIANO_2009, MUJOCO_ARMATURE |
| `M1.datasheet`: The datasheet: the stated values, nothing fitted | `M0.ideal` | physics | — | every other typed field | — | — | the ideal ratio, with τ_out = efficiency·gear_ratio·τ_in when efficiency is stated | SICILIANO_2009 |
| `M2.coulomb_viscous`: Coulomb and viscous friction at the output. *the actuator's M2 on a reducer, its no-load running torque: robotparts' load rung requires both, and its coverage ruler asks a gearbox's M2 for coulomb_friction.* | `M1.datasheet` | physics | `coulomb_friction`, `viscous_friction` | — | — | — | τ_r = coulomb_friction·sign(v) + viscous_friction·v at the output | ARMSTRONG_1994, OLSSON_1998 |
| `M3.load`: Mesh friction that grows with the transmitted load. *sim2real's gearbox M3. efficiency (η) is needed; friction_load_coefficient (μ) is read only when a source states μ, and then the two agree: driving forward μ = (1 − η)/η.* | `M2.coulomb_viscous` | physics | `efficiency`, `output_friction_fraction` | `friction_load_coefficient` | — | — | τ_out = efficiency·gear_ratio·τ_in; output_friction_fraction of the friction acts on the output side | KIRCANSKI_1997, OLSSON_1998 |
| `M4.drivetrain`: Backlash and torsional stiffness | `M3.load` | physics | `backlash`, `torsional_stiffness` | — | — | — | q_out = q_in/gear_ratio + lash(τ, backlash) + τ/torsional_stiffness | KIRCANSKI_1997, NORDIN_2002, TUTTLE_1996 |
| `M4.wear`: Backlash that opens with use (Archard). wears `backlash` by archard in torque-weighted output revolutions (N·m·rev) at `backlash_wear` | `M4.drivetrain` | physics | `backlash_wear` | — | — | — | backlash(u) = backlash + backlash_wear·Σ\|τ\|·Δrev | ARCHARD_1953, FLODIN_1997 |
| `M6.residual`: Grey-box residual on M4.wear | `M4.wear` | residual | — | — | — | — | M4.wear + an MLP residual transmission error from 3 samples of input angle, output torque and input speed | NGUYEN_TUONG_2010, ZENG_2020 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 5 samples of input angle, input speed and output torque → output angle error | HWANGBO_2019, ZHU_2025 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `coulomb_friction` | N·m | scalar | `M2.coulomb_viscous` | Coulomb friction torque at the output: the constant torque that opposes motion. |
| `viscous_friction` | N·m·s/rad | scalar | `M2.coulomb_viscous` | Viscous friction coefficient at the output. |
| `friction_load_coefficient` | — | scalar | `M3.load` | Extra output friction per N·m of load torque the part transmits: Fc + μ·abs(τ_load). |
| `output_friction_fraction` | — | scalar | `M3.load` | The share of the reducer's friction that acts on its output side. |
| `torsional_stiffness` | N·m/rad | scalar | `M4.drivetrain` | Drivetrain stiffness at the output with the rotor held: output torque per radian of twist. |
| `backlash_wear` | rad/(N·m·rev) | scalar | `M4.wear` | How fast backlash opens per torque-weighted output revolution (Archard). |

### 5.5 `foc_controller`: `actuators/servo/foc_controller/`

A field-oriented-control motor drive board. · sim2real `driver`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.ideal`: An ideal drive: the commanded current | — | physics | — | — | — | — | i_q = i_cmd (a deployment's torque cap clamps it) | KRISHNAN_2010 |
| `M1.delay_loop`: Transport delay, a first-order current loop and its torque scale | `M0.ideal` | physics | `command_latency`, `current_loop_bandwidth`, `torque_scale` | — | — | — | i_q = torque_scale·LPF(current_loop_bandwidth)[i_cmd(t − command_latency)] | KRISHNAN_2010 |
| `M2.ceiling_heat`: Voltage ceiling and board heating | `M1.delay_loop` | physics | `voltage_available`, `board_thermal_resistance`, `board_thermal_time_constant` | — | — | — | M1.delay_loop clamped to the voltage ceiling (voltage_available − ω·kt/1.5)/R of the motor it drives; board_thermal_time_constant·dΔT/dt = board_thermal_resistance·P_loss − ΔT, ΔT the board's rise over ambient | KRAUSE_2013, KRISHNAN_2010 |
| `M2.wear`: Board cooling that ages with hours. wears `board_thermal_resistance` by linear in deployment hours (h) at `board_thermal_growth` | `M2.ceiling_heat` | physics | `board_thermal_growth` | — | — | — | board_thermal_resistance(h) = board_thermal_resistance + board_thermal_growth·h | LU_MEEKER_1993, MEEKER_1998 |
| `M6.residual`: Grey-box residual on M2.wear | `M2.wear` | residual | — | — | — | — | M2.wear + an MLP residual q-current from 3 samples of current command, rotor speed and bus voltage | NGUYEN_TUONG_2010 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 3 samples of current command, rotor speed, bus voltage and board temperature → q-current | HWANGBO_2019 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `command_latency` | s | scalar | `M1.delay_loop` | Time from a command reaching the drive to its first response: the transport delay before the current loop. |
| `current_loop_bandwidth` | Hz | scalar | `M1.delay_loop` | The drive's closed-loop current (torque) bandwidth: the corner of its first-order current loop. |
| `torque_scale` | — | scalar | `M1.delay_loop` | Torque the drive delivers over the torque it was commanded: its torque-constant calibration error. |
| `voltage_available` | V | scalar | `M2.ceiling_heat` | The largest phase voltage the drive can apply: its bus voltage times its modulation limit. |
| `board_thermal_resistance` | K/W | scalar | `M2.ceiling_heat` | The board's temperature rise per watt it dissipates. |
| `board_thermal_time_constant` | s | scalar | `M2.ceiling_heat` | Time constant of the board temperature's first-order rise. |
| `board_thermal_growth` | K/W/h | scalar | `M2.wear` | How fast board_thermal_resistance grows per deployment hour (its cooling ageing). |

`voltage_available` is the type's own limit, which a deployment's `max_voltage_factor` (§7) can only lower.

### 5.6 `encoder`: `sensors/encoder/`

An angle sensor on a shaft, such as a drive's rotor encoder. · sim2real `encoder`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.quantiser`: An ideal quantiser | — | physics | `counts_per_rev` | — | — | — | q_reported = round(q/lsb)·lsb, lsb = 2π/counts_per_rev | WIDROW_2008 |
| `M1.noise_latency`: Noise and latency | `M0.quantiser` | physics | `noise_sigma`, `latency` | — | — | — | q_reported(t) = round((q(t − latency) + n)/lsb)·lsb, n ~ N(0, noise_sigma²) | MERRY_2010 |
| `M2.eccentricity`: Eccentricity harmonics | `M1.noise_latency` | physics | `eccentricity_harmonics` | — | — | — | q_reported = round((q + e(q) + n)/lsb)·lsb, delayed; e(q) the eccentricity harmonics | ZHAO_2024 |
| `M2.wear`: A mount that loosens over vibration hours. wears `eccentricity_harmonics` by sqrt in vibration hours (h) at `eccentricity_growth` | `M2.eccentricity` | physics | `eccentricity_growth` | — | — | — | amplitude_k(h) = amplitude_k + eccentricity_growth·√h for each eccentricity harmonic (its harmonic and phase stay) | JUNKER_1969, WHITMORE_1995 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `counts_per_rev` | 1/rev | integer | `M0.quantiser` | Distinct readings per revolution of the shaft it reads: its quantisation. |
| `noise_sigma` | rad | scalar | `M1.noise_latency` | Standard deviation of the reading's noise at standstill. |
| `latency` | s | scalar | `M1.noise_latency` | Delay from the shaft's angle to the reading that reports it. |
| `eccentricity_harmonics` | rad | list | `M2.eccentricity` | Angle error periodic in the shaft angle (an eccentric magnet or disc), a list of (harmonic, amplitude, phase). |
| `eccentricity_growth` | rad/√h | scalar | `M2.wear` | How fast the eccentricity grows with the square root of vibration hours (a loosening mount). |

### 5.7 `camera`: `sensors/camera/`

A frame camera's colour image: projection, distortion, exposure. · sim2real `camera_rgb`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.pinhole`: Pinhole projection | — | physics | `fx_fy`, `cx_cy`, `extrinsics` | — | — | — | p = K [R \| t] X, K from fx_fy and cx_cy, [R \| t] the extrinsics | ZHANG_2000 |
| `M1.distortion`: Brown-Conrady distortion | `M0.pinhole` | physics | `distortion` | — | — | — | radial (k1, k2, k3) and tangential (p1, p2) distortion of the projected point | ZHANG_2000, BROWN_1966, HEIKKILA_1997 |
| `M2.exposure_latency`: Exposure response and latency | `M1.distortion` | physics | `exposure_response`, `latency` | — | — | — | DN = exposure_response·illuminance·exposure time; the frame arrives latency after exposure | ZHANG_2000 |
| `M3.rolling_shutter`: Rolling shutter: a line delay | `M2.exposure_latency` | physics | `line_delay` | — | — | — | row v exposed at t0 + v·line_delay | OTH_2013, MEINGAST_2005 |
| `M3.wear`: A mount that loosens over vibration hours. wears `extrinsics` by sqrt in vibration hours (h) at `extrinsic_drift` | `M3.rolling_shutter` | physics | `extrinsic_drift` | — | — | — | translation offset(h) = e + extrinsic_drift·√h | JUNKER_1969, WHITMORE_1995 |
| `M6.residual`: Grey-box residual on M3.wear | `M3.wear` | residual | — | — | — | — | M3.wear + a B-spline grid of ray offsets over the pixel (u, v) | SCHOPS_2020 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `fx_fy` | px | pair | `M0.pinhole` | Focal lengths in pixels, (fx, fy). |
| `cx_cy` | px | pair | `M0.pinhole` | Principal point in pixels, (cx, cy). |
| `extrinsics` | m, rad | pose | `M0.pinhole` | The camera's pose in the frame it is calibrated against. |
| `distortion` | — | list | `M1.distortion` | Brown-Conrady lens distortion coefficients, radial and tangential. |
| `exposure_response` | DN/(lx·s) | scalar | `M2.exposure_latency` | Pixel value per unit of light exposure: the sensor's response. |
| `latency` | s | scalar | `M2.exposure_latency` | Delay from exposure to the frame's arrival. |
| `line_delay` | s | scalar | `M3.rolling_shutter` | Rolling shutter: the time between the exposures of successive rows. |
| `extrinsic_drift` | m/√h | scalar | `M3.wear` | How fast the extrinsics' translation drifts with the square root of vibration hours (a loosening mount); no source states a rotation rate. |

### 5.8 `depth_camera`: `sensors/rgbd/`

A stereo depth camera: depth on top of a colour stream. · sim2real `camera_rgbd`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.stereo`: Ideal stereo | — | physics | `stereo_focal_length`, `baseline`, `depth_scale`, `depth_to_color` | — | — | — | z = stereo_focal_length·baseline/d, d the disparity, scaled by depth_scale | KHOSHELHAM_2012 |
| `M1.disparity_noise`: Disparity noise | `M0.stereo` | physics | `subpixel_sigma` | — | — | — | σ_z = z²·subpixel_sigma/(stereo_focal_length·baseline) | KHOSHELHAM_2012 |
| `M2.depth_bias`: Depth bias | `M1.disparity_noise` | physics | `depth_bias_coeffs` | — | — | — | z_true = z + b(u, v, z), b from depth_bias_coeffs | HERRERA_2012 |
| `M2.wear`: A depth-to-colour mount that loosens over vibration hours. wears `depth_to_color` by sqrt in vibration hours (h) at `depth_to_color_drift` | `M2.depth_bias` | physics | `depth_to_color_drift` | — | — | — | translation offset(h) = e + depth_to_color_drift·√h | JUNKER_1969, WHITMORE_1995 |
| `M6.residual`: Grey-box residual on M2.wear | `M2.wear` | residual | — | — | — | — | M2.wear + a B-spline grid of depth corrections over (u, v, measured depth) | SCHOPS_2020, HERRERA_2012 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `stereo_focal_length` | px | scalar | `M0.stereo` | The stereo imagers' rectified focal length (on a typical unit the IR pair): the f of triangulation, not the colour camera's. |
| `baseline` | m | scalar | `M0.stereo` | Distance between the stereo pair's optical centres. |
| `depth_scale` | m | scalar | `M0.stereo` | Metres per unit of the depth image. |
| `depth_to_color` | m, rad | pose | `M0.stereo` | The depth frame's pose in the colour camera's frame. |
| `subpixel_sigma` | px | scalar | `M1.disparity_noise` | Standard deviation of the disparity: the stereo matcher's subpixel noise. |
| `depth_bias_coeffs` | m | list | `M2.depth_bias` | Coefficients of a depth bias that grows with depth and varies across the image. |
| `depth_to_color_drift` | m/√h | scalar | `M2.wear` | How fast depth_to_color's translation drifts with the square root of vibration hours; no source states a rotation rate. |

### 5.9 `hyperspectral_camera`: `sensors/hyperspectral_camera/`

A line-scan spectral camera over a belt. · sim2real `camera_hsi`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.bands`: Ideal spectral bands | — | physics | `band_centres`, `dark_reference`, `white_reference` | — | — | — | R(λ) = (DN − dark_reference)/(white_reference − dark_reference) | BURGER_2011 |
| `M1.band_widths`: Band widths | `M0.bands` | physics | `band_fwhm` | — | — | — | each band a Gaussian of its band_fwhm about its band_centres | BURGER_2011, KRUSE_1993 |
| `M2.smile_keystone`: Smile and keystone | `M1.band_widths` | physics | `smile_keystone_map` | — | — | — | the cube resampled through smile_keystone_map, then R(λ) per band | MOUROULIS_2000 |
| `M3.pushbroom`: Pushbroom line geometry | `M2.smile_keystone` | physics | `line_geometry` | — | — | — | a line camera swept by the belt: line k sees the belt at k·(belt distance per line) | GUPTA_1997 |
| `M3.wear`: A white reference that drifts with hours. wears `white_reference` by linear in deployment hours (h) at `reference_drift` | `M3.pushbroom` | physics | `reference_drift` | — | — | — | white_reference(h) = white_reference + reference_drift·h; dark_reference stays | LU_MEEKER_1993 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `band_centres` | m | list | `M0.bands` | Centre wavelength of each spectral band. |
| `dark_reference` | DN | map | `M0.bands` | The dark reference frame (shutter closed): each pixel's and band's counts at zero light. |
| `white_reference` | DN | map | `M0.bands` | The white reference frame: each pixel's and band's counts over a white tile of known reflectance. |
| `band_fwhm` | m | list | `M1.band_widths` | Full width at half maximum of each band's spectral response. |
| `smile_keystone_map` | px | map | `M2.smile_keystone` | The spectral (smile) and spatial (keystone) distortion map the cube is resampled with. |
| `line_geometry` | m, rad | pair | `M3.pushbroom` | The scan line's pose, and the belt distance per line. |
| `reference_drift` | DN/h | scalar | `M3.wear` | How fast the white reference's DN drifts per deployment hour (lamp ageing). The dark reference drifts the other way (dark current) and is not modelled by this rate. |

### 5.10 `rigid_body_chain`: `arms/rigid_body_chain/`

A chain of rigid links and joints, open or closed: a whole arm, a sub-assembly, a wrist, a five-bar. · sim2real `arm_6dof`, `arm_subassembly`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.nominal`: Nominal kinematics | — | physics | `link_geometry` | — | — | — | tool = FK(q; link_geometry) | SICILIANO_2009, DH_1955 |
| `M1.zero_offsets`: Joint zero offsets | `M0.nominal` | physics | `joint_zero_offsets` | — | — | — | tool = FK(q + joint_zero_offsets; link_geometry) | HOLLERBACH_WAMPLER_1996 |
| `M2.geometric`: Geometric (DH) errors. *sim2real's arm M2: the kinematic branch of an open chain.* | `M1.zero_offsets` | physics | `dh_errors` | — | — | — | tool = FK(q + joint_zero_offsets; link_geometry + dh_errors) | HAYATI_1983, OKAMURA_1996, HOLLERBACH_2016 |
| `M2.inertial`: Link inertials and loop closure. *sim2real's arm sub-assembly M2: the dynamic branch, which needs no DH errors.* | `M1.zero_offsets` | physics | `link_inertials`, `closure_tolerance` | — | — | — | tool = FK(q + joint_zero_offsets; link_geometry); dynamics from link_inertials with each joint's own type; a closed loop may stay open by closure_tolerance | ATKESON_1986, GAUTIER_1990, SWEVERS_1997 |
| `M2.wear`: Joint zeros that drift over vibration hours. wears `joint_zero_offsets` by sqrt in vibration hours (h) at `joint_zero_drift` | `M2.inertial` | physics | `joint_zero_drift` | — | — | — | joint_zero_offsets(h) = joint_zero_offsets + joint_zero_drift·√h | JUNKER_1969, WHITMORE_1995 |
| `M3.compliance`: Joint compliance and link inertials | `M2.geometric` | physics | `joint_compliance`, `link_inertials` | — | — | — | tool = FK(q + joint_zero_offsets + joint_compliance·τ; link_geometry + dh_errors); τ from link_inertials, each joint its own type | SPONG_1987, ATKESON_1986, GAUTIER_1990, SWEVERS_1997 |
| `M3.wear`: Joint compliance that grows with use (Archard). wears `joint_compliance` by archard in torque-weighted joint revolutions (N·m·rev) at `compliance_wear` | `M3.compliance` | physics | `compliance_wear` | — | — | — | joint_compliance(u) = joint_compliance + compliance_wear·Σ\|τ\|·Δrev | ARCHARD_1953, FLODIN_1997 |
| `M6.residual`: Grey-box residual on M3.wear | `M3.wear` | residual | — | — | — | — | M3.wear + an MLP residual of joint torques from 3 samples of each joint's error, speed and torque, added to M3.compliance's τ, so it moves the tool through the compliance | NGUYEN_TUONG_2010, ZENG_2020 |
| `M6.inertial_residual`: Grey-box residual on M2.wear. *the residual of the dynamic branch (sim2real's arm sub-assembly M6).* | `M2.wear` | residual | — | — | — | — | M2.wear + an MLP residual of joint torques from 3 samples of each joint's error, speed and torque | NGUYEN_TUONG_2010, ZENG_2020 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 3 samples of each joint's error and speed → joint torques | HWANGBO_2019 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `link_geometry` | m, rad | per joint | `M0.nominal` | The chain's nominal joint axes and link frames, as a URDF, a DH table or a CAD export states them. |
| `joint_zero_offsets` | rad | per joint | `M1.zero_offsets` | Each joint's offset between its encoder zero and its kinematic zero. |
| `dh_errors` | m, rad | per joint | `M2.geometric` | Errors of the chain's DH parameters from their nominal values. |
| `link_inertials` | kg, m, kg·m² | per link | `M2.inertial` | Each link's mass, centre of mass and inertia tensor. |
| `closure_tolerance` | m | scalar | `M2.inertial` | How far a closed chain's loop may stay open at its closure joint. |
| `joint_zero_drift` | rad/√h | scalar | `M2.wear` | How fast the joint zero offsets drift with the square root of vibration hours. |
| `joint_compliance` | rad/(N·m) | per joint | `M3.compliance` | Each joint's deflection per N·m it carries. |
| `compliance_wear` | rad/(N·m)/(N·m·rev) | scalar | `M3.wear` | How fast joint_compliance grows per torque-weighted joint revolution (Archard). |

### 5.11 `gripper`: `end_effectors/gripper/`

An end effector that grasps, on a chain's flange. · sim2real `arm_6dof_gripper`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.tcp`: A rigid gripper at the tool centre point | — | physics | `tcp_offset` | — | — | — | tool = flange · tcp_offset | SICILIANO_2009 |
| `M1.width_map`: Finger width map | `M0.tcp` | physics | `finger_width_map` | — | — | — | width = finger_width_map(command) | PRATTICHIZZO_2016 |
| `M2.contact`: Grip force and contact friction | `M1.width_map` | physics | `grip_force_map`, `contact_friction` | — | — | — | force = grip_force_map(command, width), each pad's normal force; the object holds while, at each contact, \|tangential\| ≤ contact_friction·force (two pads hold about twice one pad's) | PRATTICHIZZO_2016, MUJOCO_CONTACT |
| `M2.wear`: Finger pads that glaze with grasps. wears `contact_friction` by linear in grasp cycles (cycle) at `contact_friction_change` | `M2.contact` | physics | `contact_friction_change` | — | — | — | contact_friction(n) = contact_friction + contact_friction_change·n | ARCHARD_1953, LU_MEEKER_1993 |
| `M6.residual`: Grey-box residual on M2.wear | `M2.wear` | residual | — | — | — | — | M2.wear + an MLP residual width from 3 samples of width command, measured width and finger current | NGUYEN_TUONG_2010 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 5 samples of width command, width and finger current → finger force | HWANGBO_2019 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `tcp_offset` | m | pose | `M0.tcp` | The tool centre point's offset from the flange the gripper mounts on. |
| `finger_width_map` | m | map | `M1.width_map` | Finger opening as a function of the width command. |
| `grip_force_map` | N | map | `M2.contact` | The normal force each finger pad presses the object with, as a function of the command and the width: what a gauge squeezed between two jaws reads. A datasheet's gripping force summed over the jaws is twice it on a two-jaw gripper. |
| `contact_friction` | — | scalar | `M2.contact` | Coefficient of friction between the finger pads and the object. |
| `contact_friction_change` | 1/cycle | scalar | `M2.wear` | How much contact_friction changes per grasp cycle (pad wear and glazing). |

### 5.12 `conveyor`: `conveyors/conveyor/`

A belt driven through a drum. · sim2real `conveyor`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.ideal`: An ideal belt: kinematics and a rigid mass. *numbered by the actuator's layers (sim2real's decision); no M1: no Datasheet class yet.* | — | physics | `drum_radius`, `gear_ratio`, `moving_mass` | — | — | — | x = θ_motor·drum_radius/gear_ratio; (moving_mass + load)·ẍ = F | ZHANG_XIA_2011 |
| `M2.coulomb_viscous`: Coulomb and viscous friction on the belt | `M0.ideal` | physics | `coulomb_friction`, `viscous_friction` | — | — | — | F_f = coulomb_friction·sign(ẋ) + viscous_friction·ẋ | ARMSTRONG_1994, ZHANG_XIA_2011 |
| `M3.stribeck`: Static friction (Stribeck). *the actuator's M3.stribeck as forces on the belt.* | `M2.coulomb_viscous` | physics | `breakaway_force`, `stribeck_velocity` | — | — | — | F_f = (coulomb_friction + (breakaway_force − coulomb_friction)·exp(−(\|ẋ\|/stribeck_velocity)²))·sign(ẋ) + viscous_friction·ẋ | ARMSTRONG_1994 |
| `M4.stretch`: Belt stretch: drum and load as two masses. *M4 splits moving_mass: drum_mass on the drive side of the stretch, the rest moves with the load.* | `M3.stribeck` | physics | `stretch_stiffness`, `stretch_damping`, `drum_mass` | — | — | — | drum: drum_mass·ẍ_d = gear_ratio·τ/drum_radius − stretch_stiffness·(x_d − x_l) − stretch_damping·(ẋ_d − ẋ_l); load: (moving_mass − drum_mass + load)·ẍ_l = the same stretch force − F_f | SPONG_1987, HE_2017 |
| `M4.wear`: Friction that grows with belt distance. wears `breakaway_force` by linear in belt distance (km) at `friction_growth`; wears `coulomb_friction` by linear in belt distance (km) at `friction_growth` | `M4.stretch` | physics | `friction_growth` | — | — | — | breakaway_force and coulomb_friction both grow by friction_growth·s (the Stribeck drop stays) | LU_MEEKER_1993, ZHANG_XIA_2011 |
| `M6.residual`: Grey-box residual on M4.wear | `M4.wear` | residual | — | — | — | — | M4.wear + an MLP residual drive force from 3 samples of belt speed error, belt speed and motor current | NGUYEN_TUONG_2010, ZENG_2020 |
| `M7.learned`: Black-box network | — | learned | — | — | — | — | an MLP over 3 samples of position error and belt speed → drive force | HWANGBO_2019, HWANGBO_2019_ARXIV |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `drum_radius` | m | scalar | `M0.ideal` | Radius of the drive drum the belt wraps. |
| `gear_ratio` | — | scalar | `M0.ideal` | Motor turns per drum turn. |
| `moving_mass` | kg | scalar | `M0.ideal` | The moving mass referred to the belt: belt, drum and reflected rotor. |
| `coulomb_friction` | N | scalar | `M2.coulomb_viscous` | Coulomb friction force on the belt: the constant force that opposes its motion. |
| `viscous_friction` | N·s/m | scalar | `M2.coulomb_viscous` | Viscous friction coefficient on the belt. |
| `breakaway_force` | N | scalar | `M3.stribeck` | Static friction on the belt: the force that starts it moving from rest. |
| `stribeck_velocity` | m/s | scalar | `M3.stribeck` | Belt speed over which friction falls from breakaway_force to coulomb_friction. |
| `stretch_stiffness` | N/m | scalar | `M4.stretch` | The belt's stiffness along its length, between drum and load. |
| `stretch_damping` | N·s/m | scalar | `M4.stretch` | Damping of the belt's stretch between drum and load. |
| `drum_mass` | kg | scalar | `M4.stretch` | The drive side's mass referred to the belt, read from M4: the drum and the reflected rotor; moving_mass less drum_mass moves with the load. |
| `friction_growth` | N/km | scalar | `M4.wear` | How fast the friction level grows per kilometre of belt travel (idlers and slider bed wearing); breakaway_force and coulomb_friction grow together. |

### 5.13 `aruco_marker`: `markers/aruco_marker/`

A printed sheet carrying one or more ArUco markers rigidly. · sim2real `aruco_marker`

| rung | base | kind | adds (needed) | optional (read when stated) | code | composes | equation | references |
|---|---|---|---|---|---|---|---|---|
| `M0.nominal`: The sheet as designed, where CAD put it | — | physics | `dictionary`, `marker_ids`, `marker_length`, `layout`, `pose_on_link` | — | — | — | X_corner = FK(q) · pose_on_link · layout_corner | ZHANG_2000, GARRIDO_2014 |
| `M1.as_built`: How the sheet was actually stuck down | `M0.nominal` | physics | `as_built_offset` | — | — | — | X_corner = FK(q) · pose_on_link · as_built_offset · layout_corner | TRIGGS_2000 |
| `M2.print_scale`: Print scale | `M1.as_built` | physics | `print_scale` | — | — | — | X_corner = FK(q) · pose_on_link · as_built_offset · (print_scale · layout_corner) | STROBL_2011 |
| `M3.curl`: A sheet that is not flat | `M2.print_scale` | physics | `flatness` | — | — | — | layout_corner z = flatness·(2x/L)² (a bow across the sheet's length L) | STROBL_2011 |

| parameter | SI unit | shape | first on | definition |
|---|---|---|---|---|
| `dictionary` | — | text | `M0.nominal` | The ArUco dictionary the sheet is printed in. |
| `marker_ids` | — | list | `M0.nominal` | The marker ids the sheet carries. |
| `marker_length` | m | scalar | `M0.nominal` | Side of each marker's black square. |
| `layout` | m | map | `M0.nominal` | Each marker's corners in the sheet's own frame. |
| `pose_on_link` | m, rad | pose | `M0.nominal` | The sheet's frame on the link it is stuck to. |
| `as_built_offset` | m, rad | pose | `M1.as_built` | How the sheet was actually stuck down: its offset from pose_on_link. |
| `print_scale` | — | scalar | `M2.print_scale` | Printed size over designed size. |
| `flatness` | m | scalar | `M3.curl` | The sheet's out-of-plane bow at its far end. |

## 6. The alias and packed tables

An alias is another name for one canonical parameter of the same type; its note says how it differs whenever it is not
the same quantity one for one, and its unit is the one its source writes. A packed name carries several canonical
quantities.

### 6.1 `integrated_servo`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `torque_constant` | `kt` | (SI) | Old user-part key: a geared part's source may state it at the output (kt × gear_ratio). |
| `torque_coefficient` | `kt` | (SI) | Catalogue part key: check RMS against peak amperes (kt is per peak ampere). |
| `phase_resistance` | `resistance_line_to_line` | (SI) | Old user-part key: a 'phase to phase' value is line to line; a per-phase value is half of it on a wye winding. |
| `phase_resistance_ohm` | `resistance_line_to_line` | Ω | Per phase (sim2real's servo, Cad2Robot's motor): line to line is twice it on a wye winding. |
| `phase_inductance` | `inductance_line_to_line` | (SI) | Old user-part key: per phase is half of line to line on a wye winding. |
| `phase_inductance_h` | `inductance_line_to_line` | H | Per phase (Cad2Robot's motor): half of line to line on a wye winding. |
| `noload_speed` | `max_output_speed` | (SI) | Old user-part key: its unit is the part file's; check it before converting. |
| `max_speed` | `max_output_speed` | (SI) | Old user-part key: may be a mechanical limit rather than the no-load speed. |
| `rated_torque_nm` | `rated_torque` | (SI) |  |
| `peak_torque_nm` | `peak_torque` | (SI) |  |
| `noload_speed_rpm` | `max_output_speed` | rpm |  |
| `rated_speed_rpm` | `rated_speed` | rpm |  |
| `rated_power_w` | `rated_power` | (SI) |  |
| `voltage_v` | `nominal_voltage` | (SI) |  |
| `mass_kg` | `mass` | (SI) |  |
| `od_mm` | `outer_diameter` | mm |  |
| `length_mm` | `axial_length` | mm |  |
| `rotor_inertia_kg_m2` | `rotor_inertia` | (SI) |  |
| `peak_current_a` | `max_phase_current` | (SI) |  |
| `rated_current_a` | `rated_phase_current` | (SI) |  |
| `encoder_counts` | `encoder_lines` | (SI) | Counts per turn: 4 × lines on a quadrature encoder. |
| `peak_phase_current` | `max_phase_current` | (SI) | The registry's driver key: what the integrated drive can deliver; the motor alone may take more. |
| `backlash_arcmin` | `backlash` | arcmin |  |
| `max_effort_nm` | `peak_torque` | (SI) | Cad2Robot's servo field: the effort limit it exports. |
| `max_velocity_rad_s` | `max_output_speed` | (SI) | Cad2Robot's servo field: the velocity limit it exports. |
| `armature_kg_m2` | `output_inertia` | (SI) | As mounted: the whole axis' inertia at the output, the part's own plus what it drives; output_inertia is the part alone. |
| `frictionloss_nm` | `coulomb_friction` | (SI) | The rack twin's frictionloss is the static level: with no Stribeck term it is coulomb_friction; with one it is breakaway_torque, and coulomb_friction = frictionloss − friction_stribeck_drop. |
| `damping_nm_s_rad` | `viscous_friction` | (SI) |  |
| `friction_weakening_nm_s_rad` | `friction_weakening` | (SI) |  |
| `friction_weakening_limit_rad_s` | `friction_weakening_limit` | (SI) |  |
| `friction_stribeck_drop_nm` | `breakaway_torque` | (SI) | A difference, not a level: breakaway_torque − coulomb_friction. |
| `friction_stribeck_speed_rad_s` | `stribeck_velocity` | (SI) | The twin's Stribeck exponent is fixed at 2. |
| `friction_capture_nm` | `friction_capture` | (SI) |  |
| `friction_capture_speed_rad_s` | `friction_capture_speed` | (SI) |  |
| `friction_capture_dwell_s` | `friction_capture_dwell` | (SI) |  |
| `ripple_harmonics` | `harmonics` | (SI) |  |
| `ripple_rotor_per_output` | `gear_ratio` | (SI) | The ripple's rotor turns per output turn: the gear ratio as the twin's ripple reads it. |
| `backlash_rad` | `backlash` | (SI) |  |
| `gear_stiffness_nm_rad` | `torsional_stiffness` | (SI) | Fitted on a mounted unit: it includes the mount's compliance. |
| `kt_out_nm_a` | `kt` | (SI) | Output side: kt_out = kt × gear_ratio. |
| `backlash_wear_rad_per_nm_rev` | `backlash_wear` | (SI) |  |
| `friction_growth_nm_per_h` | `friction_growth` | (SI) | It grows the rack twin's frictionloss, the static level: breakaway_torque and coulomb_friction together. |
| `sigma2` | `viscous_friction` | (SI) | LuGre's own symbol (M3.lugre) for the same viscous coefficient. |
| `resistance` | `resistance_line_to_line` | Ω | M5.dcmotor's winding resistance as MuJoCo takes it, per phase as sim2real reads it (phase_resistance_ohm): line to line is twice it on a wye winding. |

| packed name | carries | ordered | note |
|---|---|---|---|
| `thermal_r_c` | `thermal_resistance`, `thermal_time_constant` | yes | The rack twin's (K/W, s) pair. Its thermal model fits a Foster network of one or two poles; with two, thermal_resistance = ΣR_k and no single time constant holds. |
| `motorconst` | `kt` | at least these | M5.dcmotor's MuJoCo array (unit mujoco), 2 numbers: sim2real reads the torque constant from it at the OUTPUT (kt × gear_ratio, unless the actuator's gear carries the ratio); the other entry is MuJoCo's, unverified here. |
| `nominal` | `nominal_voltage` | at least these | M5.dcmotor's MuJoCo array (unit mujoco), 3 numbers of nominal ratings; the other entries are MuJoCo's, unverified here. |
| `inductance` | `inductance_line_to_line` | at least these | M5.dcmotor's MuJoCo array (unit mujoco), 2 numbers; per phase or line to line is MuJoCo's convention, unverified here. |
| `cogging` | `harmonics` | at least these | M5.dcmotor's MuJoCo array (unit mujoco), 3 numbers: one cogging harmonic (harmonic, amplitude, phase) in MuJoCo's order, unverified here. |
| `thermal` | `thermal_resistance`, `thermal_time_constant` | at least these | M5.dcmotor's MuJoCo array (unit mujoco), 6 numbers; the rest and the order are MuJoCo's, unverified here. |
| `lugre` | `sigma0`, `sigma1` | at least these | M5.dcmotor's MuJoCo array (unit mujoco), 5 numbers: the M3.lugre terms in MuJoCo's order, unverified here. |

### 6.2 `bldc_motor`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `torque_constant` | `kt` | (SI) | Old user-part key: a geared part's source may state it at the output (kt × gear_ratio). |
| `torque_coefficient` | `kt` | (SI) | Catalogue part key: check RMS against peak amperes (kt is per peak ampere). |
| `phase_resistance` | `resistance_line_to_line` | (SI) | Old user-part key: a 'phase to phase' value is line to line; a per-phase value is half of it on a wye winding. |
| `phase_resistance_ohm` | `resistance_line_to_line` | Ω | Per phase (sim2real's servo, Cad2Robot's motor): line to line is twice it on a wye winding. |
| `phase_inductance` | `inductance_line_to_line` | (SI) | Old user-part key: per phase is half of line to line on a wye winding. |
| `phase_inductance_h` | `inductance_line_to_line` | H | Per phase (Cad2Robot's motor): half of line to line on a wye winding. |
| `noload_speed` | `max_output_speed` | (SI) | Old user-part key: its unit is the part file's; check it before converting. |
| `max_speed` | `max_output_speed` | (SI) | Old user-part key: may be a mechanical limit rather than the no-load speed. |
| `rated_torque_nm` | `rated_torque` | (SI) |  |
| `peak_torque_nm` | `peak_torque` | (SI) |  |
| `noload_speed_rpm` | `max_output_speed` | rpm |  |
| `rated_speed_rpm` | `rated_speed` | rpm |  |
| `rated_power_w` | `rated_power` | (SI) |  |
| `voltage_v` | `nominal_voltage` | (SI) |  |
| `mass_kg` | `mass` | (SI) |  |
| `od_mm` | `outer_diameter` | mm |  |
| `length_mm` | `axial_length` | mm |  |
| `rotor_inertia_kg_m2` | `rotor_inertia` | (SI) |  |
| `peak_current_a` | `max_phase_current` | (SI) |  |
| `rated_current_a` | `rated_phase_current` | (SI) |  |
| `encoder_counts` | `encoder_lines` | (SI) | Counts per turn: 4 × lines on a quadrature encoder. |
| `motorconst` | `kt` | (SI) | sim2real's motor card: the scalar kt, rotor side (robotparts' dcmotor packs an array under this name). |
| `resistance` | `resistance_line_to_line` | Ω | sim2real's motor card, after robotparts' dcmotor: per phase or line to line is not stated; check it before converting. |
| `inductance` | `inductance_line_to_line` | H | sim2real's motor card: the convention as for resistance. |
| `cogging` | `torque_ripple` | (SI) | sim2real's motor card: one cogging amplitude; its harmonic is not stated. |
| `bearing_friction_nm` | `coulomb_friction` | (SI) | A bare motor's Coulomb friction is its bearings'. |
| `bearing_friction_growth_nm_per_h` | `friction_growth` | (SI) |  |

| packed name | carries | ordered | note |
|---|---|---|---|
| `thermal` | `thermal_resistance`, `thermal_time_constant` | yes | sim2real's motor card: (K/W, s). |

### 6.3 `gearbox`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `mesh_efficiency_eta` | `efficiency` | (SI) | The twin's constant η in τ_out = η·N·τ_in; a datasheet's efficiency is at its stated operating point; μ = (1 − η)/η driving forward. |
| `backlash_rad` | `backlash` | (SI) |  |
| `backlash_arcmin` | `backlash` | arcmin |  |
| `gear_stiffness_nm_rad` | `torsional_stiffness` | (SI) | Fitted on a mounted unit: it includes the mount's compliance. |
| `backlash_wear_rad_per_nm_rev` | `backlash_wear` | (SI) |  |
| `rated_torque_nm` | `rated_torque` | (SI) |  |
| `peak_torque_nm` | `peak_torque` | (SI) |  |
| `mass_kg` | `mass` | (SI) |  |
| `od_mm` | `outer_diameter` | mm |  |
| `length_mm` | `axial_length` | mm |  |

### 6.4 `foc_controller`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `command_delay_s` | `command_latency` | (SI) | The twin's transport delay; the registry's command_latency is measured host to first response over a named bus. |
| `current_loop_hz` | `current_loop_bandwidth` | (SI) | The twin's first-order loop corner, which is its −3 dB bandwidth. |
| `voltage_available_v` | `voltage_available` | (SI) |  |
| `board_thermal_growth_per_h` | `board_thermal_growth` | (SI) |  |

| packed name | carries | ordered | note |
|---|---|---|---|
| `board_thermal` | `board_thermal_resistance`, `board_thermal_time_constant` | yes | sim2real's driver card: (K/W, s). |

### 6.5 `encoder`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `latency_s` | `latency` | (SI) |  |
| `eccentricity_growth_mrad_per_sqrt_h` | `eccentricity_growth` | mrad/√h |  |
| `encoder_counts_per_rev` | `counts_per_rev` | (SI) |  |
| `encoder_counts` | `counts_per_rev` | (SI) |  |
| `encoder_lines` | `counts_per_rev` | (SI) | Lines: a quadrature encoder counts 4 × lines per revolution. |
| `encoder_bits` | `counts_per_rev` | (SI) | Bits: 2^bits counts per revolution. |

### 6.6 `camera`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `latency_s` | `latency` | (SI) |  |
| `camera_latency_s` | `latency` | (SI) |  |
| `camera_latency_ms` | `latency` | ms |  |
| `line_delay_s` | `line_delay` | (SI) |  |
| `extrinsic_drift_mm_per_sqrt_h` | `extrinsic_drift` | mm/√h |  |

### 6.7 `depth_camera`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `baseline_m` | `baseline` | (SI) |  |
| `depth_to_color_drift_mm_per_sqrt_h` | `depth_to_color_drift` | mm/√h |  |

### 6.8 `hyperspectral_camera`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `reference_drift_per_h` | `reference_drift` | (SI) |  |

| packed name | carries | ordered | note |
|---|---|---|---|
| `dark_white_refs` | `dark_reference`, `white_reference` | yes | sim2real's camera_hsi card: (dark, white) as its name reads, both in DN. |

### 6.9 `rigid_body_chain`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `link_lengths` | `link_geometry` | (SI) | Lengths only: the geometry also carries the joint axes. |
| `joint_offsets` | `joint_zero_offsets` | (SI) |  |
| `joint_offset_drift_rad_per_sqrt_h` | `joint_zero_drift` | (SI) |  |
| `joint_zero_drift_rad_per_sqrt_h` | `joint_zero_drift` | (SI) |  |
| `compliance_wear_per_nm_rev` | `compliance_wear` | (SI) |  |

### 6.10 `gripper`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `contact_friction_change_per_cycle` | `contact_friction_change` | (SI) |  |

### 6.11 `conveyor`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `drum_radius_m` | `drum_radius` | (SI) |  |
| `drum_radius_mm` | `drum_radius` | mm |  |
| `moving_mass_kg` | `moving_mass` | (SI) |  |
| `coulomb_n` | `coulomb_friction` | (SI) |  |
| `viscous_n_s_m` | `viscous_friction` | (SI) |  |
| `static_n` | `breakaway_force` | (SI) |  |
| `stretch_stiffness_n_m` | `stretch_stiffness` | (SI) |  |
| `stretch_damping_n_s_m` | `stretch_damping` | (SI) |  |
| `coulomb_growth_n_per_km` | `friction_growth` | (SI) |  |

### 6.12 `aruco_marker`

| alias | canonical | unit as written | note |
|---|---|---|---|
| `marker_length_m` | `marker_length` | (SI) |  |
| `layout_m` | `layout` | (SI) |  |
| `flatness_m` | `flatness` | (SI) |  |

## 7. Deployment-only names (not on any ladder)

| type | name | role | where it belongs |
|---|---|---|---|
| `integrated_servo` | `gravity_nm` | assembly | The station's gravity torque on the joint: the description's. |
| `integrated_servo` | `kp_nm_rad` | config | The drive's position gain: the deployment's config. |
| `integrated_servo` | `kd_nm_s_rad` | config | The drive's velocity gain: the deployment's config. |
| `integrated_servo` | `max_current_a` | config | The drive's current limit: the deployment's config. |
| `integrated_servo` | `controller` | config | M5.dcmotor's MuJoCo controller array: the drive's control settings. |
| `integrated_servo` | `saturation` | config | M5.dcmotor's MuJoCo saturation array: the drive's ceilings. |
| `foc_controller` | `torque_cap_nm` | config | The runner's torque cap: the deployment's config. |
| `foc_controller` | `telemetry_rate_hz` | config | The host loop's rate: the deployment's config. |
| `foc_controller` | `max_voltage_factor` | config | The firmware's modulation-limit setting: the deployment's config. |
| `rigid_body_chain` | `base_frame` | assembly | Where the chain stands in its cell: the description's. |
| `rigid_body_chain` | `tool_frame` | assembly | The tool mounted on its flange: the description's. |
| `conveyor` | `load_mass_kg` | assembly | The parcels on the belt: the deployment's. |
| `conveyor` | `encoder_counts_per_m` | config | Derived from the encoder, the ratio and the drum radius. |

The servo card's driver-role names (`torque_scale`, `current_loop_hz`, `command_delay_s`, `voltage_available_v`) are
not deployment-only: they resolve through `foc_controller`, which `integrated_servo`'s `M4.drivetrain` and
`M5.envelope` compose.

## 8. The vendoring rule and the parity rule

### 8.1 robotparts vendors the rungs (the sync check is built next phase)

- **The canonical copy:** robot_models' `src/robot_models/actuators/rungs/<word>/`, exactly robotparts' file set
  (`model.json` and `README.md`, plus `model.py` and `model.js` where robotparts has them; `dcmotor` has neither), byte
  for byte robotparts @ `d16f6a8`. The ruler hard-codes every file's sha256 (`RUNG_FILES`) and each folder's terms
  with their required flags (`RUNG_TERMS`), and never reads robotparts.
- **Bytes pinned:** `.gitattributes` holds `src/robot_models/actuators/rungs/** -text`, so `core.autocrlf=true` (the
  Windows box) cannot rewrite them; the ruler checks the line.
- **The vendored copy:** robotparts keeps `models/actuator/<word>/` byte-identical. Not a symlink (tagged-release
  rebuilds extract with `filter="data"`, which rejects links out of the tree); no extra file (robotparts' build hashes
  and publishes every non-dot file); no edit (one changed byte breaks the published fit's `definition_sha256`).
- **The sync check:** a robotparts test (with a `--check` tool) compares each vendored file's sha256 with robot_models'
  copy. It copies **files**, never folders: importing a rung's `model.py` in-process leaves `__pycache__/` beside it
  (the ruler's import test runs with `PYTHONDONTWRITEBYTECODE=1` for that reason). A change lands in robot_models first
  and is then re-vendored; robotparts' rule that "a definition a fit binds is never edited away" still applies there.
- **Nothing else goes inside a rung folder:** no `__init__.py`, `ladder.py` or tests.
- **The lab twin does not move** (robotparts: "never published: one rack's system identification"). Its structure is
  here as names only: `M3.weakening_capture`, `M4.drivetrain`, `M5.envelope`.

### 8.2 sim2real parity (one additive test in sim2real, next phase)

The mapping lives in the sim2real test, as sim2real's own table: each card to a robot_models type, each
`(card, rung key)` to a robot_models rung key. robotparts and robot_models never read sim2real. For every card rung:

1. its mapped rung exists, at the same level;
2. its base (the previous physics rung, or a residual's named base) maps into the mapped rung's `fallback()`;
3. every card parameter is held: `ladder.resolve(name) ⊆ ladder.holds(mapped)`, or it resolves in a composed type and
   is held at the composed rung, or it is a deployment name. The one documented exception:

   - `servo` M4's `thermal_r_c`: heat is robotparts' layer 5: M5.envelope holds it.

4. a card parameter whose name is canonical carries the canonical SI unit (spelling aside: `1` = dimensionless,
   `rad/N·m` = `rad/(N·m)`, sim2real's `kg, kg·m²` for `link_inertials` omits the centre of mass's m) or sits in this
   table with its factor:

| card | name | sim2real's unit | × to SI |
|---|---|---|---:|
| `encoder` | `noise_sigma` | mrad | 0.001 |
| `encoder` | `eccentricity_harmonics` | mrad | 0.001 |
| `arm_subassembly` | `closure_tolerance` | mm | 0.001 |
| `camera_rgbd` | `depth_bias_coeffs` | mm | 0.001 |
| `camera_hsi` | `band_centres` | nm | 1e-9 |
| `camera_hsi` | `band_fwhm` | nm | 1e-9 |
| `camera_rgb` | `exposure_response` | DN / (lux·ms) | 1000 |

**Status:** the rule as written, run in scratch against sim2real main @ `f4d84c8` and the ruler's spec: 13 cards, 76
rungs, no problems; all seven unit rows used. Run with the first draft's servo map (M5 → `M5.dcmotor`, no exception)
it reports 10 problems, among them `M5.dcmotor`'s fallback missing M4 and `friction_weakening_nm_s_rad`,
`friction_capture_nm` and `gear_stiffness_nm_rad` not held: the mismatch the reviews found.

| card | type | rung map (sim2real → robot_models) |
|---|---|---|
| `servo` | `integrated_servo` | M0→`M0.ideal`, M2→`M2.coulomb_viscous`, M3→`M3.weakening_capture`, M4→`M4.drivetrain`, M5→`M5.envelope`, M5.wear→`M5.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `motor` | `bldc_motor` | M0→`M0.ideal`, M4→`M4.ripple`, M5→`M5.dcmotor`, M5.wear→`M5.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `gearbox` | `gearbox` | M0→`M0.ideal`, M3→`M3.load`, M4→`M4.drivetrain`, M4.wear→`M4.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `driver` | `foc_controller` | M0→`M0.ideal`, M1→`M1.delay_loop`, M2→`M2.ceiling_heat`, M2.wear→`M2.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `encoder` | `encoder` | M0→`M0.quantiser`, M1→`M1.noise_latency`, M2→`M2.eccentricity`, M2.wear→`M2.wear` |
| `arm_subassembly` | `rigid_body_chain` | M0→`M0.nominal`, M1→`M1.zero_offsets`, M2→`M2.inertial`, M2.wear→`M2.wear`, M6→`M6.inertial_residual`, M7→`M7.learned` |
| `arm_6dof` | `rigid_body_chain` | M0→`M0.nominal`, M1→`M1.zero_offsets`, M2→`M2.geometric`, M3→`M3.compliance`, M3.wear→`M3.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `arm_6dof_gripper` | `gripper` | M0→`M0.tcp`, M1→`M1.width_map`, M2→`M2.contact`, M2.wear→`M2.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `conveyor` | `conveyor` | M0→`M0.ideal`, M2→`M2.coulomb_viscous`, M3→`M3.stribeck`, M4→`M4.stretch`, M4.wear→`M4.wear`, M6→`M6.residual`, M7→`M7.learned` |
| `camera_rgb` | `camera` | M0→`M0.pinhole`, M1→`M1.distortion`, M2→`M2.exposure_latency`, M3→`M3.rolling_shutter`, M3.wear→`M3.wear`, M6→`M6.residual` |
| `camera_rgbd` | `depth_camera` | M0→`M0.stereo`, M1→`M1.disparity_noise`, M2→`M2.depth_bias`, M2.wear→`M2.wear`, M6→`M6.residual` |
| `camera_hsi` | `hyperspectral_camera` | M0→`M0.bands`, M1→`M1.band_widths`, M2→`M2.smile_keystone`, M3→`M3.pushbroom`, M3.wear→`M3.wear` |
| `aruco_marker` | `aruco_marker` | M0→`M0.nominal`, M1→`M1.as_built`, M2→`M2.print_scale`, M3→`M3.curl` |

## 9. The rulers

- **`tests/test_ladder.py`: the core, 36 tests, green.** Held red: a loop of bases; an unknown base; a name outside
  the vocabulary; a needed name added twice down one chain; an optional name held again; a name both needed and
  optional; a duplicate key, or a second root; a unit inside a name; an orphan; a bad wear; a datasheet mismatch; a
  packed name read as one parameter, or holding a name outside the vocabulary; a string where a `Reference` belongs;
  discovery of a module with no `LADDER`, two ladders of one type, or a composition of a missing type or rung.
- **`tests/test_ladders_acceptance.py`: the lanes' ruler, 120 tests, marked `ruler`.** Red until the lanes land.
  - **Per type (12 × 8), each importing the type's own `ladder.py`**, so one lane's state never decides another's:
    the folder; rung keys, bases, needed and optional names, code, wears, composition; equation and references; the
    vocabulary with units and shapes; aliases with notes and units; packed names; deployment names; no maker, path or
    value in any `.py` of the type folder (numbers judged in `ladder.py`; makers in every module, docstrings
    included).
  - **Across the repo:** the twelve types discovered at their folders; every composition landed; the eight rung
    folders' file sets and sha256; each folder's `model.json` terms against the servo rung that names it (a required
    term needed there, an optional one held); `.gitattributes`; `references.py` clean of makers and paths; the scan's
    own planted reds (a DOI or URL is a citation, never a value); every module imports in a fresh interpreter.
  - **Green now (6):** the spec is a valid ladder; the spec holds every robotparts term; `.gitattributes`; the
    references; the scan's planted reds; the import test.
- **The command:**

```bash
env -u VIRTUAL_ENV uv run --project /home/bam/robot_models --extra test pytest -q /home/bam/robot_models/tests
```

## 10. Lanes

- **Write sets:** each type lane writes `<its folder>/ladder.py` (and its folder's `__init__.py`); the rung lane
  writes `actuators/rungs/<word>/` only (copied files, never folders). The group packages (`sensors/`, `arms/`,
  `end_effectors/`, `conveyors/`, `markers/`) exist already, so no two lanes add the same file.
- **No barrier between type lanes:** each type's tests import its own module. `integrated_servo` composes
  `foc_controller`, so `test_every_composition_names_a_landed_type_and_rung` goes green only when both have landed.
- **Frozen for the lanes:** `ladder.py`, `references.py`, this PEP and both test files. A lane that needs a field or a
  reference asks the dispatcher; that is a barrier change, landed first and alone.

## 11. Calls made beyond the settled decisions (each one is reversible)

1. **Needed vs optional names on a rung** (`Rung.optional`, `Ladder.holds`), and **M1 as optional typed fields** (§0).
2. **`arm` folded into `rigid_body_chain`** (§0). Its two branches off `M1.zero_offsets`: `M2.geometric` (DH errors,
   then `M3.compliance`, `M3.wear`, `M6.residual`) and `M2.inertial` (inertials and loop closure, then `M2.wear`,
   `M6.inertial_residual`). Zach's `M0.kinematics` / `M0.dynamics` families ("if you want") are expressible later as
   branch names; the core keeps one root.
3. **`M5.envelope` added on the servo** (all three reviews): sim2real's servo M5 builds on its M4 (the rack twin), so
   `M5.envelope` stands on `M4.drivetrain`, composes `foc_controller` `M2.ceiling_heat` and holds the heat; `M5.wear`
   now builds on it. `M5.dcmotor` stays robotparts' rung on `M3.lugre` with its own folder. Heat is layer 5
   (robotparts' definition), although the rack twin carries `thermal_r_c` at its M4: the parity rule lists that one
   move.
4. **`M5.dcmotor` composes nothing:** its drive settings are MuJoCo's own `controller` and `saturation`, deployment-only.
5. **Type folders** (decision 2's rule): `actuators/servo/…` and `sensors/rgbd/` mirror `user/parts`; `arms/`,
   `end_effectors/`, `conveyors/`, `markers/` and `sensors/encoder/` are new groups; the hyperspectral camera takes its
   v3 id because the RGB camera already has category `camera`.
6. **The eight code folders belong to `integrated_servo` only.** The motor, gearbox and conveyor number their rungs by
   the same layers, and each rung's note says which actuator rung it specialises. A bare motor and a reducer keep an
   `M2.coulomb_viscous` (robotparts' actuator rungs cover their category, and its coverage ruler asks both for it); the
   motor's `M3.stribeck` builds on it and its M4 on that (amendment 1, §14).
7. **The rack twin's friction is a type rung with all its terms:** `M3.weakening_capture` adds the weakening limit and
   the capture speed and dwell its equation reads, from the twin's own term list.
8. **The servo's friction wear moves both levels:** `M5.wear` wears `breakaway_torque` and `coulomb_friction` at one
   rate, as the twin grows its static level with the Stribeck drop fixed.
9. **The gearbox's load loss runs on efficiency; μ is read only when stated** (`M3.load` needs `efficiency`, as
   sim2real's gearbox does; `friction_load_coefficient` is optional, and when a source states it the two agree:
   μ = (1 − η)/η driving forward). Amendment 1 (§14) replaced "η or μ, never both", which the needed η contradicted.
10. **A board's thermal pair is two parameters** (`board_thermal_resistance`, `board_thermal_time_constant`), so its
    wear grows the resistance alone; sim2real's `board_thermal` is a packed name.
11. **References are a shared module written now** (`robot_models.references`), not ported per lane; maker documents
    and one rig's code are left out (decision 4), and the foc_controller's M0 cites a drives textbook instead.
12. **`peak_phase_current` is no alias on the bare motor:** the registry defines it as what a driver can deliver. On
    the integrated servo it stays, noted as the integrated drive's limit.
13. **Units and definitions tightened:** `link_inertials` is kg, m, kg·m²; `line_geometry` is a pair (pose, belt
    distance per line); `extrinsic_drift` and `depth_to_color_drift` say translation only (no source states a rotation
    rate); the conveyor's Stribeck rung gains `stribeck_velocity` (m/s), which robotparts' stribeck rung, the one
    sim2real's conveyor M3 points at, requires.
14. **Two type docstrings no longer name makers** (`integrated_servo.py`, `gearbox.py`), and the scan covers every
    module in a type folder.
15. **`M7.learned` has no base**, a quantity none of the three name sets names takes sim2real's name without its unit
    (`friction_weakening`, `drum_radius`, `latency`), and a linear quantity a linear word (`breakaway_force`), as
    before.

## 12. What v1 deliberately does not do

- **The Nov 12 move plan's steps 2–7** (metrics, fits, cards and coverage into robot_models, with shims in sim2real).
  They wait for that plan's step-0 gate; `claude/one-click-fivebar` is still unmerged on sim2real.
- **The lab twin.** robotparts' rule (2026-10-28) is that it is never published, so there is no `twin` folder here.
- **robot-msgs 0.3.0 and a PyPI release.** `robot_models.homing.protocols` needs `robot_msgs.control`, which 0.2.0
  lacks. Release robot-msgs first, then raise the floor.
- **Flipping the repo public.** Decided (MIT, as robot_msgs: decision 7); it happens once the lanes land, with a
  LICENSE file, the PyPI metadata and a CI that skips the rulers that need the CAD bundles. `datasheet/`'s docstring
  examples and PEP 0003 still name a maker's product page as a sample source; they are outside the type folders the
  scan covers.
- **Datasheet classes for the other nine types**, with their registry names as vocabulary (PEP 0003 §9); the coverage
  tool's `foc_controller` and `rgbd_camera` keys (`voltage_min`, `fov_h`, …) arrive with them.
- **Reproducing robotparts' coverage table.** Its `RULER` is robotparts' own frozen contract (its M1 is a catalogue
  minimum, its M4 asks `pole_pairs`); aligning it to `parameters()` is robotparts' next phase.
- **Alternatives** ("`output_inertia` or `gear_ratio`² × (`rotor_inertia` + `gearbox_input_inertia`)"): the equation
  says so; the core has no field for it yet.
- **MuJoCo's array layouts** (§3) and **v3's two families** as separate roots.

## 13. The reviews (2026-11-16), and what changed

- **Applied:** equations and references on every rung, with the scan exempting DOIs and URLs (fidelity 1); `arm` into
  `rigid_body_chain` (fidelity 2, breakage 6); needed vs optional names (fidelity 3, breakage 1); `M5.envelope` and the
  strengthened parity rule (fidelity 4, breakage 3, physics 1); `Packed` (fidelity 5); the motor's and gearbox's M2
  notes (fidelity 6, as notes); maker docstrings and the wider scan (fidelity 7); a keyword-argument example (fidelity,
  readability); per-type tests that import the type's own module (breakage 2); `needs()` (breakage 4); `Found.folder`
  as the join key (breakage 5); the counts (breakage 7); `.gitattributes` and files-only sync (breakage 8); the unit
  table and `Alias.unit` (physics 2); the output-side and per-phase notes (physics 3); `cogging → torque_ripple` on the
  motor (physics 4); `peak_phase_current` (physics 5); η or μ (physics 6); the second wear (physics 7); the twin's
  missing terms and aliases, and the conveyor's Stribeck speed (physics 8); the Foster note (physics 9); units and
  definitions (physics 10).
- **Rejected:**
  - dropping the bare motor's `M2.coulomb_viscous` (fidelity 6): robotparts' actuator rungs cover the motor's category
    and its coverage ruler asks a BLDC motor's M2 for both frictions, so the rung is sourced; it now needs them;
  - asserting that `required()` reproduces robotparts' coverage `RULER` (breakage 1): that table is robotparts' own
    contract with its own rung meanings (§12), and robot_models' public ruler must not read or mirror it;
  - making `foc_controller` a barrier lane (breakage 2): the per-type tests no longer need one;
  - a rotation drift rate for the cameras (physics 10): no source states one; the definitions say translation only.

## Undo

`git revert` the commit that adds this PEP, `src/robot_models/ladder.py`, `src/robot_models/references.py`, the five
group packages, `.gitattributes`, `tests/test_ladder.py` and `tests/test_ladders_acceptance.py`, and restores the two
docstrings; nothing imports them yet. After the lanes land, reverting their commits removes the type folders and the
rung copies. robotparts and sim2real are not touched in v1.

## 14. Amendment 1 (2026-11-16)

The v1 check agents, run over the landed ladders, found physics errors in the frozen spec itself. The ruler moved
first, at `87db875` ("PEP 0004 amendment 1, the ruler first"); a review of the amendment then moved the ruler again
(the motor's EMF from kt, the board's rise ΔT, the gripper's per-pad force, the hyperspectral references split); the
ladders, the tables of §5 (regenerated from the ruler's spec) and the generated type docs followed. *Structural* means
the ruler asserts it (keys, bases, needed and optional names, wears, the vocabulary); *text* means the ruler records
the equation or note as the spec's form and asks only for a non-empty one.

- **`bldc_motor` gains `M3.stribeck`** (structural). Wrong: robotparts' coverage ruler asks a BLDC motor's M3 for
  `breakaway_torque` and `stribeck_velocity`, and the bare motor had no M3 and neither name. Fix: `M3.stribeck` on
  `M2.coulomb_viscous` needs both, reads `stribeck_exponent` when stated, has no code folder (the eight are the
  servo's, §11 call 6); `M4.ripple` now sits on it; the three names join the vocabulary with the servo's definitions,
  word for word, at the output, which on a bare motor is its shaft (the datasheet's `coulomb_friction` and
  `viscous_friction` read there too, so the motor's whole friction curve reads at one point). 83 → 84 rungs.
- **`bldc_motor` `M5.wear` grows breakaway with Coulomb** (structural). Wrong: with a Stribeck curve below it,
  growing `coulomb_friction` alone past `breakaway_torque` makes the Stribeck term negative. Fix: both wear linearly
  in deployment hours at `friction_growth`, as the servo's `M5.wear` already did; the Stribeck drop stays.
- **`conveyor` `M4.stretch` adds `drum_mass`** (structural). Wrong: its equation used a drum mass `m_d` no name held,
  and counted the drum twice (`moving_mass` already holds the drum and the reflected rotor, and the load side used
  `moving_mass + load`). Fix: `drum_mass` (kg) is the drive side, `moving_mass − drum_mass + load` the load side.
- **`conveyor` `M4.wear` grows `breakaway_force` with `coulomb_friction`** (structural). Wrong: growing Coulomb alone
  past breakaway made M3's Stribeck term negative (friction rising with speed, no static peak). Fix: both grow at
  `friction_growth`, whose definition now says the friction level, both together.
- **`depth_camera` `M0.stereo` adds `stereo_focal_length`** (structural). Wrong: z = f·baseline/d used an f no name
  held, and that f is the stereo imagers' rectified focal length (on a typical unit the IR pair), not the colour
  camera's `fx_fy`. Fix: `stereo_focal_length` (px, scalar) on M0; the M0 and M1 equations use it.
- **Three-phase copper loss, the rise over ambient, and per-phase EMF** (text). Wrong: the servo's `M5.envelope`
  heated the winding by thermal_resistance·i²R with R one phase's resistance (one phase's loss) and called the rise a
  temperature T, and `foc_controller`'s `M2.ceiling_heat` called the board's rise T the same way (read as a
  temperature, both drop ambient); the motor's `M5.dcmotor` wrote V = R·i + L·di/dt + back_emf_constant·ω per phase
  with a line-to-line EMF constant. Fix: 1.5·i²R (= 3/4·resistance_line_to_line·i², i the phase-current amplitude),
  ΔT the rise over ambient (the winding's and the board's), and the per-phase EMF amplitude (kt/1.5)·ω. The EMF is
  written from `kt`, a name the rung needs, as the servo's `M5.envelope` and the drive's `M2.ceiling_heat` already
  write it; it equals back_emf_constant/√3 only for a line-to-line peak constant, while `back_emf_constant` is read at
  M1 alone, when stated, and does not say rms or peak (a datasheet Ke in V_rms/krpm is off by √2). The v1 motor ladder
  already had the copper loss; its text now matches the spec.
- **`output_inertia` for J** (text): the servo's `M4.drivetrain` equation named a J no name held.
- **The gearbox's η is needed, μ read only when stated** (text): `M3.load`'s note said a fit sets η or μ, never
  both, while the rung needs `efficiency`; now μ is read only when a source states it, and then the two agree
  (§11 call 9). The ladder's equation is the spec's, η alone: it no longer adds coulomb_friction + μ·abs(τ_load) on
  top of η, which charged the same mesh loss twice.
- **The encoder's amplitudes grow per harmonic** (text): `M2.wear` grew a list of (harmonic, amplitude, phase) as one
  scalar e; now each amplitude grows with √h and its harmonic and phase stay.
- **The hyperspectral references are two frames, and only the white one drifts** (structural). Wrong:
  `reference_drift` said the dark and white references drift at one rate, but dark current moves the dark frame the
  other way; and `M3.wear` wore the whole `dark_white_refs` pair, so a consumer that applied the Wear grew the dark
  frame too. Fix: `dark_reference` and `white_reference` (DN, map) replace the pair, `M0.bands` needs both,
  `M3.wear` ("A white reference that drifts with hours") wears `white_reference` alone at `reference_drift` (lamp
  ageing), and sim2real's `dark_white_refs` is a packed name holding the two (§6.8).
- **`rigid_body_chain` on `link_geometry`, one output** (text): `M2.geometric` and `M3.compliance` wrote "DH +
  dh_errors" for geometry the type holds as `link_geometry` (a URDF, a DH table or a CAD export), and `M2.inertial`
  named its output "tip" where every other rung says "tool"; `M3.compliance` now says τ comes from `link_inertials`,
  and `M6.residual` says its torques enter at that τ, so they move the tool through the compliance.
- **The gripper's hold is per contact** (text): `M2.contact`'s friction cone applies at each contact, so two pads
  hold about twice one pad's. That is right only when `force` is the normal force at each pad, so `grip_force_map` is
  now defined as that force (what a gauge squeezed between two jaws reads), and a datasheet's gripping force summed
  over the jaws is twice it on a two-jaw gripper; read as one pad's, the summed figure would hold twice too much.
- **Ladder-side only, no contract change:** the type ladders' docstrings, which the generated READMEs quote, read as
  public text (no working provenance); the servo's wear usages are named constants; the `foc_controller` M0 comment
  names the deployment's `torque_cap_nm`.

**Raised, not taken** (each needs a contract change or another type's data first):

- **`noload_speed`'s unit:** the alias's source unit is the part file's own and unknown, and `Alias.unit` can only
  say a unit or "the SI unit"; saying "the source's own, unknown" needs a contract sentinel. Its note says to check.
- **`encoder_lines` / `encoder_bits` → `counts_per_rev`:** each needs a conversion (4 × lines, 2^bits), and `Alias`
  has no conversion field; that is a contract question. Their notes say how.
- **`voltage_available` against the deployment's `max_voltage_factor`:** settled as text only (one clause in §5.5:
  the type's own limit, which a deployment can only lower); no new name or rung.
- **robotparts' coverage M1 keys for `foc_controller` and `depth_camera`** (`voltage_min`, `fov_h`, …): they wait on
  Datasheet classes for those types (§12), not on the ladders.

Undo: `git revert` the commits after `87db875` that apply this amendment, then `87db875` itself; the ruler, the
ladders and the docs go back together.

## 15. Amendment 2 (2026-11-17): the type is one object; its models are classes; no "ModelFamily"

**Ask:** Zach, round 2 of the API proposals (`docs/proposals/api-2026-11-17/ROUND2.md`): "I 100% like organizing
everything around a type", "a model should 100% be a class", "we only ever want vocabulary to be additive"; then, in
his daily note of 2026-11-17 (*Types vs ModelFamilys*): "BLDC motor is a TYPE and that TYPE has a model family? or is
the TYPE just the name of the model family?" and "`servo.models` is how you access the modelfamily". The answer, in
the callout under that heading: the type is the one object, `servo.models` is its group of models, and the word
"ModelFamily" goes. Round 2's pick is v6 "Fitted instances" (values on Model instances). This amendment is the
default build of both; it is on branch `api-v6-parttype` until Zach picks.

**The names**

| name | what it is | spelled |
|---|---|---|
| **PartType** | a generic kind of part, no maker: its models, its Datasheet class, its aliases, packed and deployment names | `class IntegratedServo(PartType, id="integrated_servo")` |
| **Models** | a group of Model classes, in ladder order, with the batch calls; every level is one too | `IntegratedServo.models`, `.models.m3`, `.models["M3"]` |
| **Model** | one rung: a class of Parameters; an instance holds its values | `IntegratedServo.models.m3.stribeck`, `.models["M3.stribeck"]` |
| **Parameter** | unit, definition, shape, note, required; its value on an instance | `Stribeck.breakaway_torque` (class), `m3.breakaway_torque` (value) |
| **Datasheet** | the typed record of what the sources state about one SKU (PEP 0003) | `IntegratedServo.Datasheet(...)`, the class `IntegratedServoDatasheet` |
| **DatasheetModel** | the M1 rung's class: the SI numbers M1 runs on, not the Stated values | key still `M1.datasheet` |

- Every group has `fit(log, start=)`, `select_best(held_out, fits)`, `ladder(top=)` and `tree()`; iterating a group
  gives its Model classes. `IntegratedServo.fit`, `.select_best`, `.ladder`, `.tree` and `for m in IntegratedServo`
  forward to `IntegratedServo.models`. `IntegratedServo.models.m3.fit(log)` fits only the M3s;
  `.models.m2.select_best(held_out, fits)` chooses among the M2s of a whole-type fit.
- **Why not "ModelFamily":** "family" already means a maker's product line (robotparts' `RobotPart.family`), a kind of
  physics (robotparts' `model.json`) and sim2real's slices of one type's model (`FAMILIES`); a rigid body chain is one
  type with two families. The type also owns more than its models. **Why not a separate family object behind
  `.models`:** that is a fourth noun for a group, and every level is the same kind of group. **Why not "ModelType":**
  "model type" already means physics, residual or learned (`Rung.kind`). `PartType` is what ladder.py already says.
- **Calling the type refuses** (`IntegratedServo(...)` raises, naming `.Datasheet(...)` and `.models...`): a call
  means nothing yet, and a later pick (v7's unit) can give it a meaning without a second one to retire.

**What is built (code: `src/robot_models/part_type.py`, `src/robot_models/actuators/servo/integrated_servo_type.py`)**

- v6's prototype, moved into the package: the Model and Parameter classes, additive vocabulary enforced at class
  creation, a generic unit-seeded fit, numpy-vectorized `predict` running the rung folders' own `model.py`
  (loaded from their text, so no `__pycache__/` lands beside the bytes robotparts vendors), `select_best` by sim2real's
  1.5× noise-floor rule, a numpy black box (`M7.learned`) with the same outside.
- `from robot_models import IntegratedServo` is the type, loaded lazily (PEP 562): `import robot_models.ladder` and
  the datasheets still import no numpy (tests/test_part_type.py holds it).
- **One source, checked:** `IntegratedServo.v1_ladder()` derives today's `Ladder` record from the classes and equals
  `ladders()["integrated_servo"]` on the whole record (tests/test_part_type.py; held red by editing one rung's title).
  `ladder.py`'s hand-written record stays for now: it is what the equality is checked against, and what `ladders()`,
  the ruler, sim2real's parity test and robotparts read.
- **The datasheet's messages name the type** (`Datasheet.type_name()`: `IntegratedServoDatasheet` → "IntegratedServo"),
  so `IntegratedServo.rated_torque: unit 'N' ...` reads as before.
- The other 11 types keep their v1 API unchanged; each gets a PartType when it needs one.

**The back-compat alias, and why the bare name is split for now**

`robot_models.actuators.servo.IntegratedServo` (and `...servo.integrated_servo.IntegratedServo`) stays the datasheet
class, an alias of `IntegratedServoDatasheet`; `robot_models.IntegratedServo` is the type. The frozen rulers require
it: `tests/test_ladders_acceptance.py` asserts `ladder.datasheet is IntegratedServo` with that import, and
`tests/test_datasheet.py` builds `IntegratedServo(rated_torque=...)` and asserts `issubclass(IntegratedServo,
Datasheet)`. robotparts imports the same name as the datasheet in 8 live files (its GIM part file, its six
`sources/*.py`, `user/test_datasheets.py`) and in its archived source-file proposals; sim2real does not import it.

**To finish the rename** (each a later, separate step): robotparts writes `IntegratedServo.Datasheet(...)` (or
imports `IntegratedServoDatasheet`); the two rulers move to the new names (a ruler edit, Zach's call); then the alias
goes and `robot_models.actuators.servo.IntegratedServo` becomes the type; then `ladder.py` shrinks to
`LADDER = IntegratedServo.v1_ladder()`, with the equality kept as a snapshot.

**Undo:** delete branch `api-v6-parttype` before merging; after merging, `git revert` its commit. Nothing outside
robot_models changed.

