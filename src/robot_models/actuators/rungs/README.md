# Actuator rung definitions

These eight folders (`ideal`, `datasheet`, `coulomb_viscous`, `stribeck`, `load`, `lugre`, `ripple`, `dcmotor`) are the
canonical rung definitions of the `integrated_servo` ladder (PEP 0004 section 8.1). robotparts vendors them
byte-identical under `models/actuator/<word>/` because its published rung bytes are pinned by sha256.

## Paths inside the copied files

The files were written in [robotparts](https://github.com/zacharyyamaoka/robotparts) and are kept exactly as it wrote
them, so some paths inside them point at robotparts, not at this repo. These live in robotparts and do not exist here:
`SCHEMA2.md` (section 4.3.2), `schemas/model.schema.json`, `robotparts.models.si_terms`, the `robotparts.mujoco` plugin,
and `tests/acceptance_v2/test_amendment_2_5.py` (the Python/JS parity test that holds `model.py` and `model.js` to 1e-12).
robot_models does not check that parity; robotparts does.

## Changing a rung

Edit it here, then run robotparts' `tools/sync_actuator_rungs.py` (it copies the files over; `--check` lists every
difference without writing). Keep each folder to its regular files: no `__init__.py`, no `__pycache__`, no dot-files.
`.gitattributes` marks this tree `-text` so no checkout rewrites a line ending.

WHY every byte is pinned in three places, so a change moves all three together: the `RUNG_FILES` sha256 table in
`tests/test_ladders_acceptance.py`; each `model.json`'s `implementation_sha256` for `model.py` and `model.js`; and
robotparts' published fits' `definition_sha256`. PEP 0004 section 8.1 also says a definition a fit binds is never
edited away, so a change to a published rung is a new definition (a new rung or version), not an in-place edit.
