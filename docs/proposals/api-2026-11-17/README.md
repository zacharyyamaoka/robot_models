# Five API proposals: Model as the base object (2026-11-17)

Zach, [[Daily Note - Nov 17 2026]] line 20: "I don't totally love what you have done right now. I feel it should be
like Model as the base object and then each 'Type' or Family of models can be organized into the ladders." And line 13:
"assume I have accurate datasheet and the rigs, and I have selected the models.. what now?"

Today (v1, PEP 0004) the object you hold is a `Ladder`; a rung is a record keyed `M3.stribeck`, and the runnable
physics is a free function in a folder the rung names by string (`actuators/rungs/stribeck/model.py`).

This folder is a babble round: five isolated prototypes, one shared scenario, one frozen check. Nothing in `src/`
changes until Zach picks.

## The contract (frozen before any variant was built)

**Outcome all five serve:** Model is the base object; a type's models are organized into its ladder; and the "what
now" flow (datasheet + rig log → models with values → predict → fit → compare rungs) reads naturally.

**Invariants:** the integrated_servo ladder's 14 keys in today's order; one vocabulary per type with aliases
(`frictionloss_nm` → `coulomb_friction`); the physics in `actuators/rungs/*/model.py` is reused, not retyped;
robot_models ships no instance values.

**Scenario** ([fixture.py](fixture.py)): one illustrative servo's datasheet in catalogue units, and a 1-DOF rig
velocity-plateau sweep simulated from a hidden Stribeck truth plus noise. Every variant's `scenario.py` does:

1. browse the integrated_servo models as a ladder;
2. pick M3.stribeck: its parameters, units, equation, references, fallback; resolve an alias;
3. build the datasheet model (M1) from the datasheet and predict the resisting torque at 1 rad/s;
4. fit M2.coulomb_viscous and M3.stribeck to the rig log;
5. compare them and choose the rung that pays.

**Stop line:** runnable physics for M0–M3 only; M4–M7 declared, not run. Fitting is numpy only. No change to `src/`,
the ruler, robotparts or sim2real; each README states its migration path instead.

**Check:** [check_scenario.py](check_scenario.py) (`/home/bam/robot_models/.venv/bin/python check_scenario.py`):
the ladder keys, the fallback, the alias, M1 = the stated values in SI, M3 beats M2, and M3 recovers the hidden truth
(5 % on the friction levels, 10 % on stribeck_velocity and viscous_friction). Shown to fail on a wrong fit before
any variant was built.

### Weighted criteria (sum 100)

| id | criterion | weight | 1 | 3 | 5 |
|---|---|---|---|---|---|
| fr1 | **The Model is the thing you hold** | 25 | metadata and runnable code are separate objects joined by a string path (today) | one object, but it needs its family or a registry to run or describe itself | one object says what it needs, what it means, where it comes from, and computes |
| fr2 | **"What now" is short** | 25 | the user writes unit conversion, fitting and comparison glue | predict is direct; fit or compare needs hand glue | datasheet → model, fit on a log, compare rungs: one call each; the scenario reads as the workflow |
| fr3 | **The ladder stays browsable and honest** | 15 | the tree, fallback or same-level alternatives are lost | derivable with effort | the family prints as a tree; fallback, siblings and aliases are one call each |
| fr4 | **Migration cost** | 15 | the contract is rewritten and consumers break | the ruler is re-expressed; robotparts' vendoring or sim2real's parity test changes | an additive layer: the v1 ruler, rung bytes and consumers untouched |
| fr5 | **Adding a model is small and local** | 10 | a new rung edits several files or a registry | one folder plus one registration | one file or class in the type's folder, found automatically |
| fr6 | **Plays with the stack** | 10 | custom idioms; arrays need loops; nothing exports | some standard idioms | numpy-vectorized, type checkers see the parameters, values hand to MuJoCo/sim2real/scipy without glue |

The weights are the AI's reading of what Zach's message stresses (fr1, fr2), with migration weighted because three
repos consume v1 today. Revise them and the ranking moves.

### Hard gates (pass/fail, never averaged)

- **G1 Type data only:** robot_models holds no measured value and no maker; values come from the caller.
- **G2 Light runtime core:** numpy + robot_msgs + stdlib at runtime; anything heavier is an extra.
- **G3 Recovers the hidden truth:** `check_scenario.py` passes, and the variant never reads `_truth.py`.
- **G4 One vocabulary per type:** canonical names and aliases resolve as today.

## The five directions (assigned before building)

| | name | what a Model is | what organizes the ladder | prior art |
|---|---|---|---|---|
| v1 | Subclass ladder | a frozen dataclass whose fields are its parameters | Python inheritance: a rung subclasses its base, fallback = MRO | Modelica `extends`, PyTorch `nn.Module` |
| v2 | Records in a Family | a frozen record with a `bind(values)` → callable | a `Family` holding the type's vocabulary and model tree | lmfit `Model`, PyBaMM model vs `ParameterValues` |
| v3 | Composed terms | a sum of term models (inertia + friction + ripple ...) | a type's slots and the options per slot; rungs are named presets | Modelica `replaceable`, lmfit composite models, Drake diagrams |
| v4 | Estimators that climb | an estimator: `from_datasheet`, `fit`, `predict`, `score` | model selection: `family.climb(datasheet, log)` keeps the rungs that pay | scikit-learn estimators, AIC/BIC selection, SysIdentPy |
| v5 | Type kit | a model that names the experiment identifying it | the Type: its models, datasheet class, experiments and fidelity metrics in one kit | sim2real `models.camera.kits_for`, Hugging Face `AutoModel` |

Considered, not built: one symbolic declaration per rung that generates model.py, model.js and model.json (today
three hand-kept copies). It is a splice candidate on any of the five.
