# PEP 0003: Typed datasheets, resolved from one Python file per raw source

- **Status:** accepted and built, 2026-11-16 (tracer on two parts; the other 13 user parts are listed under §9).
  **Amended 2026-11-17** (amendment 1, at the end): a source file calls the type with keywords; a raw file sits in
  `sources/` named like its `.py`; the header says what the file is (`role=`).
- **Ask:** Zach, daily note 2026-11-16, "Ok what snext?" → "Do you think I should be taking a different architecture"
  (vault `Sources/Daily Notes/Daily Note - Nov 16 2026.md`). In short: give each RobotPart a type with known typed
  fields plus a dict for custom ones; robot_models holds the types in the same folder tree, each with a ladder of
  models; one parsed `.py` per raw source inside the part folder, cached, never re-parsed at run time; the part's
  datasheet imports from its sources ("source.nominal_voltage", with an `as_written` for the source's own label);
  several sources per field, reconciled somehow; and a parameter × source table to see what is given and what is
  missing. "Its very very important … the Source should almost feel like if I wrote it down my self."
- **Layering:** `robot_msgs ← robot_models ← robot_parts ← robot_descriptions` (vault, "Finishing the 1-DOF Pendulum —
  Parts Are the Atom (2026-11-16)"). robotparts imports robot_models; robot_models never imports robotparts.

## 1. The answer in five lines

1. **Keep his architecture.** Per-part `sources/<name>.py`, typed fields by type folder in robot_models, the part file
   choosing among sources, and the table. It scored 88 against 72 for the best alternative (§3).
2. **Four changes to it:** the typed names ARE robotparts' registry keys (one vocabulary, tested); a source file pins
   the sha256 of its raw file (a cache needs a key); the source's own label doubles as the anchor back into the raw
   file (tested on 69 labels); and the reconcile rule is *prefer a listed order, pick one field by name, show the rest*.
3. **Parsing from Python works both ways.** robotparts.dev already parses `part.py` statically (an AST walk, never
   run). The new source files are kept literal-only (a lint test), so a static reader can read them too. The table
   here imports them, because they are Zach's own files.
4. **Day one, the typed fields caught three real errors in the old GIM file** (§8): "Phase Resistance" is SteadyWin's
   *Phase to Phase* (line-to-line); Kv and Kt are stated after the 10:1 but were filed under rotor-side keys; and
   "Power 52 W" is electrical input (24 V × 2.18 A), not the registry's `rated_power` (mechanical output, 34 W).
5. **Built:** `robot_models.datasheet` (stdlib only) and `robot_models.actuators.servo.{IntegratedServo, BldcMotor,
   Gearbox}`. In robotparts' user tree: the GIM4305-10 (6 sources) and the JMC 60ASM400 (3 sources) are sourced;
   `python3 -m user.table <id>` prints the table, and `python3 -m user.page <id> --serve` draws it in data-ui.

## 2. What the question assumed, and what holds

- *"The specs are no longer linked."* Half true. Each `Spec` carried a `source_url`, but not *where in the source*
  (0 of the catalogue's 1,966 Specs carry a `quote`), not the source's own words, and not *which bytes* were read.
  So "linked" here means three things: a file, a place in it, and the bytes it was parsed from.
- *"Parsing from Python is the risk."* It is not. Python as the cache is fine, and robotparts has done the hard part
  (a no-run reader). The real risk is a cache with no key: a re-captured page that silently disagrees with its parse.
  The sha256 closes that.
- *"Multiple sources: TBD how to reconcile."* That is not an unknown. robotparts already decided it (SCHEMA2 2.6:
  "the left-most source with a value is Nominal"), and data-ui draws it (its CONTRACT §36). What was missing is a
  per-part, per-field override written in the part file. That is `pick`.
- *"A rung per file?"* Not yet. robot_models has no actuator ladder: the rungs live in sim2real's type cards and move
  here at the layering plan's step 7. §6 says when a rung gets its own file.
- **Affirmed.** Typed fields plus an `extra` dict; the common accessor (the type's name) over each source's own
  names; one `.py` per raw source inside the part folder; and Python, not JSON, as the cache. Each is the right call,
  and each is built.

## 3. Criteria, then the options

| # | Criterion | Weight |
|---|---|---:|
| C1 | **Every value links back to its raw source:** the file, the place in it, the bytes it was parsed from | 25 |
| C2 | **Reads like Zach wrote it:** a source file is plain lines; a part file is a short list of choices | 20 |
| C3 | **One vocabulary:** field names are registry keys, or names the catalogue already uses | 15 |
| C4 | **Disagreement is shown, never resolved silently** | 15 |
| C5 | **Reuses what exists and stays small:** registry, data-ui, the user loader | 10 |
| C6 | **A path upward:** descriptions and sim2real import typed values; the catalogue can still read it statically | 10 |
| C7 | **Ships today, and one revert undoes it** | 5 |

Scores are 1–5 per criterion; total = Σ weight × score / 5, out of 100.

| Option | C1 | C2 | C3 | C4 | C5 | C6 | C7 | Total |
|---|---|---|---|---|---|---|---|---:|
| **A. His proposal:** per-part `sources/<name>.py`, typed fields from robot_models by type folder, the part file picks, a parameter × source table | 5 | 5 | 4 | 4 | 3 | 4 | 5 | **88** |
| **B. Keep robotparts' machinery** (`specs={}` + `citations={}` of `Spec`, registry keys, the build's AST reader, data-ui's Sources view) and change only the authoring surface (group Specs by source, point the build at user/) | 3 | 2 | 5 | 4 | 5 | 4 | 4 | 72 |
| **C. A JSON/CSV cache per source** (datasheet-extractor's `published/<product>/<source>.json`), referenced from Python | 4 | 2 | 3 | 3 | 4 | 3 | 4 | 64 |
| **D. One wide table per part** (rows = fields, columns = sources) as the storage | 3 | 3 | 4 | 5 | 3 | 3 | 4 | 70 |

- **A wins.** It costs some new code (C5 3): a loader, a value type and a table. Its C3 is a 4 only because the
  proposal leaves the vocabulary open; taking registry keys closes that.
- **B is the strong alternative**, and A takes its assets: registry names, `Spec` as the generated catalogue
  projection, the literal-only rule, and data-ui. B loses because a `Spec` per value repeats its source on every
  line, which is exactly what Zach rejected. It also has no place for the source's own label or a variant column.
- **C** was rejected by Zach ("I like python as ground truth"). **D** makes every new source an edit to every row,
  and spreads one source's provenance across the file.

## 4. The settled questions

**Where the typed fields live.** In robot_models, by type folder, mirroring `user/parts/`:

- `robot_models/actuators/servo/integrated_servo.py` → `IntegratedServo`
- `robot_models/actuators/servo/bldc/bldc_motor.py` → `BldcMotor`
- `robot_models/actuators/servo/gearbox/gearbox.py` → `Gearbox`

One frozen dataclass per file. Each field is `Field(units, definition, group=…)`; `KEY_SPECS` is registry
`key_specs[category]`. IntegratedServo extends BldcMotor with the reducer's fields.

**The vocabulary.** Every typed field is either a registry key, with the same units and factors, or a name the
catalogue's generated `Actuator`/`Gearbox` classes already carry (`rated_phase_current`, `max_axial_load`,
`back_emf_constant`, …). `user/test_datasheets.py` checks both, so a third vocabulary fails a test. sim2real's type
cards name *model* parameters (`armature_kg_m2`, `frictionloss_nm`), not datasheet fields. Mapping those onto these
names is part of the move at layering step 7; nothing is renamed today.

**Typed fields plus `extra`.** A source writes a typed value as `name = Stated(...)` and a custom one in
`extra = {"noise": Stated(...)}`. A module-level name that is not a typed field refuses, with the nearest field
name ("did you mean nominal_voltage"), so a typo can never land quietly among the custom values.

**What a source file looks like.**

```python
source = Source("SteadyWin product page", kind="manufacturer", url="https://www.steadywin.cn/en/h-pd-9.html",
                file=("product_page.html", "datasheet(spec_table).html"), sha256="d04c01…",
                read_on="2026-11-16", columns=("GDZ34", "GDS34"))

nominal_voltage = Stated(24, "V", "额定电压(Nominal Voltage)")
peak_torque = Stated({"GDZ34": 3.82, "GDS34": 3.47}, "N·m", "堵转扭矩(Stall Torque)")
kv = Stated({"GDZ34": 16.75, "GDS34": 15.25}, "rpm/V", "转速常数(Speed Constant)", at="output")
```

- **`Stated(value, unit, as_written)` rather than a bare `nominal_voltage = 24  # V`.** The unit has to be data:
  it is what catches the 100× class of error (kg·m² against kg·cm²), and a comment cannot be checked. The label
  has to be data too, because it is the anchor back into the raw file.
- **Why a new class, not `Spec` or `Value`.** A `Spec` repeats its source on every line. dataclass_values' `Value`
  IS the number, for configs, and has no label or per-column value. `Stated` is the transcription; the catalogue's
  `Spec` is generated from it (`user.get(id).specs`), never typed twice.
- **The variant column** is the header's `columns=` plus a `{column: value}` dict on the values that differ; a value
  the same in every column is written once. Reading one without a column refuses (`steadywin["GDS34"].rated_torque`).
- **The location** is the label, as a text quote, when the raw file has text. It is `where=` (a source_msgs compact
  selector: `page=1&xywh=percent:18.3,84.1,6.3,2.6`) when it has none: a vector drawing, a picture. All of them parse
  as source_msgs selectors (tested), so source-highlighter can draw them.
- **`at="output"`** marks a rotor-side quantity (Kv, Kt, rotor inertia) that the source states after the reducer.

**How the datasheet imports from its sources.** The part file:

```python
sources = Sources(__file__)
datasheet = IntegratedServo.from_sources(
    sources,
    prefer=[steadywin["GDZ34"], sources.aifitlab_listing["GDZ34"], sources.smc_powers_drawing, sources.estimates],
    pick={"kt": steadywin["GDS34"], "kv": steadywin["GDS34"], ...},   # why, beside it
)
```

`datasheet.peak_torque` is the chosen `Stated` (3.82 N·m). It still reaches `.as_written` ("堵转扭矩(Stall Torque)")
and `.source` (title, url, column, file); `sources.steadywin_page["GDS34"].peak_torque` is any one statement.
A value stated at the output is carried to the rotor through `gear_ratio` and marked derived, with its basis:
the GIM's `kv` reads 152.5 rpm/V, "15.25 rpm/V at the output × gear_ratio 10".

**Disagreeing sources.** Every statement stays readable (`datasheet.statements(name)`), and `disagreements()`
lists each field whose sources differ from the chosen value. Values are compared in SI, so 140 g against 0.15 kg is
compared as a number, not as two different units. A source written for another model (`applies_to`, a sibling's
drawing) is shown beside the values but is never preferred and never counted as a disagreement. On the JMC, that
leaves exactly the three conflicts the catalogue's notes.md lists by hand: peak torque, rated current and rotor
inertia. That is the outside check that the rule is right.

## 5. Parsing: what robotparts.dev does today, and what this does

- **Today:** the build reads each `parts/<id>/part.py` with an AST walk (`build/robotparts2/partpy.py`). It evaluates
  only literals, names assigned above and the imported robotparts constructors, and never runs the file, because
  part files arrive in pull requests. It takes `specs` (the nominal), the part's own `citations` and the atlas
  overlays, and groups them by registry key into the sections (`build/site_pages.py` `SPEC_GROUPS`). Each source
  is a column keyed `source_kind:host`, and data-ui draws the columns and the "+1 agrees / 1 differs" pill.
  The values themselves were typed by hand or by the ingestor. Nothing parses the raw file at build time.
- **What a static parse buys:** it is safe on untrusted files, has no import side effects, and needs no package.
  **What it costs:** literals only, so no imports beyond robotparts, no arithmetic, and no functions.
- **Here:** user/ files are Zach's own, so the loader imports them, which gives real typed objects, unit checks and
  conversions. The source files are still kept to literals (`robot_models.datasheet.literal`, tested on all nine),
  so the catalogue's reader could learn to read them unchanged. The part file imports robot_models and is not
  static-readable; promotion to `parts/` writes out its generated Specs instead.

## 6. A rung per file?

No: a ladder per type in one file beside the type (`actuators/servo/ladder.py`), each rung an entry that names its
parameters. A rung gets its own module only when it has code, an equation that runs, as robotparts' `models/actuator/
<rung>/model.py` does today. The typed datasheet is rung 1 of that ladder ("the datasheet: the stated values, nothing
fitted", registry `model_rungs.actuator`). Nothing was invented here: the fitted rungs move from sim2real's cards at
step 7 of the layering plan.

## 7. Files

- **robot_models:**
  - `src/robot_models/datasheet/`: `stated.py`, `source.py` (`Source`, `Sources`, `load_source`), `datasheet.py`
    (`Field`, `Datasheet`, `from_sources`, `disagreements`), `units.py` (the registry's spellings and SI factors),
    `literal.py` (the no-run check). Stdlib only.
  - `src/robot_models/actuators/servo/`: the three types.
  - `tests/test_datasheet.py`: 21 tests, including 5 planted bad values held red. A mutation that stubs the value
    check turns them red.
- **robotparts, `user/`:**
  - The part folders: `sources/*.py` (6 for the GIM, 3 for the JMC) and the two rewritten part files.
  - `user/__init__.py` loads both formats and adds `datasheet()` and `sourced()`.
  - `datasheets.py` (the generated Specs, the table rows), `table.py` (CLI) and `page.py` (data-ui page).
  - `test_datasheets.py`: 19 tests, including 3 planted bad values and a stale raw file.

## 8. What the typed fields caught in the old GIM file

| Old key, value | Now | Why |
|---|---|---|
| `phase_resistance` 1.15 Ω, "whether phase-to-phase or per phase is not stated" | `resistance_line_to_line` | SteadyWin's own page says 相间电阻(**Phase to Phase** Resistance); "Phase Resistance" was aifitlab's copy |
| `kv` 15.25 rpm/V, `torque_constant` 0.62 N·m/A | `kv` 152.5 rpm/V, `kt` 0.062 N·m/A (derived) | The page's figures are after the 10:1 (366 rpm / 24 V = 15.25); registry `kv`/`kt` are rotor-side |
| (nothing) | `extra["power"]` 52 W | 24 V × 2.18 A is electrical input; registry `rated_power` is mechanical (1.05 N·m × 312 rpm = 34 W) |
| `noload_speed` 402 rpm | `max_output_speed` | Registry name |
| (nothing) | `extra["length_with_driver"]`: 32 mm (SteadyWin) ≠ 26 mm (aifitlab) | A copy error at the reseller, now visible |

## 9. Not done, and what would change the decision

- **Not migrated (13 parts, still on `specs={}`):**
  - jmc-60asm200-5-1024c
  - steadywin-gim-unidentified
  - mjbots-moteus-c1, mjbots-moteus-r4-11
  - stepperonline-ad60-g10-d14
  - mjbots-fdcanusb
  - jst-ph3-can-junction, mjbots-xt30-junction
  - acaproto-cnctopbaos-36v-600w-adj, odrive-regen-clamp
  - realsense-d405, realsense-d435
  - bam-robotics-1-dof-arm-plate

  It is not mechanical. Each needs its raw files read for labels and places, and most need a type that does not
  exist yet (driver board, CAN adapter, connector, power supply, camera, plate). The loader reads both formats.
- **No ladder file, no sim2real change.** That is step 7 of the layering plan.
- **What would change it:**
  - If the catalogue site must render user parts directly, teach `partpy.py` to read `sources/*.py` (they are
    literal-only), rather than generating Specs.
  - If datasheet-extractor's Copy as Spec is to feed this, point it at `Stated(...)` lines. That is the obvious next
    step, and nothing here blocks it.
  - If Zach prefers bare `nominal_voltage = 24  # V`, the cost is losing the unit check and the anchor. Not
    recommended.

## Undo

`git revert` the robot_models commit and the robotparts commit (the handoff names both). The old part files come
back, and the raw files are untouched (git-ignored).

## Amendment 1 (2026-11-17): the typed call, the raw file beside its .py, and `role`

**Ask:** Zach, daily note 2026-11-16: "To get type completion you should be filling in these values from the models
though ... Please make 5 proposals for what the source file here can look like", and "move the raw sources files into
the folder as well ... the file names should be the exact same for the sources so its just like the .pdf or .py".
The five proposals, side by side with a VS Code completion screenshot each: robotparts
`reports/source-file-proposals-2026-11-16.html`. Proposal 1 is applied.

- **A source file calls the part type with one keyword per value** (`values = IntegratedServo(peak_torque=Stated(...),
  ..., extra={...})`). The editor completes the field names from the dataclass's own signature and marks an unknown
  one; the type stays a frozen dataclass; nothing is generated. Replaces §4's one top-level line per value, which no
  editor can complete (the names were free module globals). The loader refuses the old form with the new spelling.
- **A per-column value is checked column by column** (`check_value`), so the type accepts a source's
  `{"GDZ34": 3.82, "GDS34": 3.47}`; a resolved datasheet still holds one value per field (`from_sources` reads through
  one column).
- **The source's type must be one the part type extends** (`check_source`): a `BldcMotor(...)` statement can feed an
  `IntegratedServo` part, a `Gearbox(...)` one cannot.
- **`Source.role`** (`ROLES`: datasheet, 2d_drawing, cad, product_page, manual, seller_chat, photo; one or a tuple) says
  what the raw document is. It moved out of the file name because the name now pairs the raw file with its `.py`
  (`sources/steadywin_page.html` + `sources/steadywin_page.py`). `kind` stays who says it.
- **`file` is relative to `sources/`**, and **a header with no `values` is allowed**: a raw file catalogued (its role,
  origin and bytes) but not transcribed yet. `Datasheet.stating()` / `columns()` leave such sources out of the table.
- **`literal.read(path)`** reads a source file without running it (header, type name, values, extra), refusing any file
  `literal.problems` flags; `values` must call a name imported from robot_models. robotparts' coverage table reads
  headers this way, so it stays true over a `git archive`.

Undo: `git revert` this amendment's robot_models commit together with robotparts' (the two move together: robotparts'
source files use the new form).
