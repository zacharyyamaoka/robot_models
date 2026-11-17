"""Write the browsable docs of the type ladders (PEP 0004): a README.md in each type folder, and docs/TYPES.md.

    python tools/make_type_docs.py            # write every generated file whose text changed
    python tools/make_type_docs.py --check    # name each stale or missing file and exit 1; writes nothing

Each type folder's README.md is generated from that folder's `ladder.py` (its `LADDER`) and the works in
`robot_models.references`: the rung tree as a mermaid flowchart, the rungs (base, needed and optional names, code
folder, equation, references), the wear, the vocabulary, the aliases, the packed and deployment-only names, and the
reference list. docs/TYPES.md indexes the types. Edit a ladder, never a generated file, then run this from the repo
root; tests/test_type_docs.py fails while any generated file is stale.

WHY one README per type folder, with an anchored row per rung (the v1 browsing choice of 2026-11-16: "the folder
tree in Obsidian and VS Code, with a README per rung", as robotparts' rungs already have): a rung that runs code has a
folder of its own (actuators/rungs/<word>/), and its README.md came with that code; the rung's row links it. Every
other rung is data in its type's ladder.py with no folder to hold a README, so its README is its row in the type
folder's README, one link away (#rung-m3-stribeck). A folder per rung that holds no code would be a deliberate
deviation from mirroring the parts tree; an anchored row keeps one link per rung without one.

WHY generated: a README written by hand drifts from its ladder the first time a lane adds an alias.

Stdlib only. The output is deterministic, with no timestamps, so --check compares text: authored order is kept where
it means something (rungs bottom up, the vocabulary by the rung that first reads it), names are sorted elsewhere.
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import posixpath
import re
import sys
import textwrap
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[1]
# WHY this checkout's src first: a worktree documents its own ladders, never an editable install of another checkout.
sys.path.insert(0, str(ROOT / "src"))

from robot_models import references as references_module
from robot_models.ladder import (
    DEPLOYMENT_ROLES,
    LEARNED_LEVEL,
    RESIDUAL_LEVEL,
    TOP_PHYSICS_LEVEL,
    WEAR_LAWS,
    Alias,
    Found,
    Ladder,
    Reference,
    Rung,
    discover,
)

GENERATOR = PurePosixPath("tools/make_type_docs.py")
REGENERATE = "python tools/make_type_docs.py"
PACKAGE = Path(inspect.getsourcefile(discover)).resolve().parent  # src/robot_models, as imported above
PACKAGE_OUT = PurePosixPath("src/robot_models")                     # the same folder, relative to the repo root
INDEX = PurePosixPath("docs/TYPES.md")
PEP = PurePosixPath("docs/peps/0004-model-ladders-by-type.md")
# WHY written out here: the stack line is the package docstring's and the repo README's (PEP 0004 "Layering"); the
# index repeats it so a reader who lands on docs/TYPES.md sees where the types sit.
STACK = "robot_msgs ← robot_models ← robot_parts ← robot_descriptions,   with sim2real on top"
NONE = "—"

# WHY explicit text colours on every class: GitHub draws mermaid on a dark page in dark mode, and a light fill with
# the dark theme's light text would be unreadable; each class sets its own fill, stroke and text colour.
MERMAID_CLASSES = {
    "physics": "fill:#e8f1fb,stroke:#2f6aa3,color:#10243a",
    "wear": "fill:#e8f1fb,stroke:#2f6aa3,color:#10243a,stroke-dasharray:6 3",
    "residual": "fill:#fdf0dc,stroke:#b06a00,color:#3a2400",
    "learned": "fill:#f1e6fb,stroke:#7a3db3,color:#2a1040",
    "other": "fill:#f2f2f2,stroke:#8a8a8a,color:#222222",
}
LABEL_WIDTH = 24  # characters per line of a rung's title inside its mermaid box
# WHY spelled out here (PEP 0004's Rung table, the `level` row, and Rung.level in ladder.py): a public reader of the
# index meets "M3" before any ladder, and robotparts' layers mean nothing without them.
ACTUATOR_LAYERS = ("the ideal model", "the datasheet", "Coulomb and viscous friction", "state-dependent friction",
                   "the drivetrain", "electrics and heat")


@dataclass(frozen=True)
class Project:
    """A project or tool the ladders' notes name without saying what it is: one line under docs/TYPES.md."""

    name: str
    url: str
    mentioned: str  # a regular expression, case-insensitive: the README text that names it
    what: str

    @property
    def html_id(self) -> str:
        return "name-" + re.sub(r"[^a-z0-9]+", "-", self.name.lower()).strip("-")


# WHY a fixed list, by name: the ladders' notes cite where a rung or an alias came from ("sim2real's servo M4", "the
# rack twin", "robotparts' M1"), and a public reader has no other page that says what those are. Each README links
# the entries its own text names.
PROJECTS = (
    Project(name="Cad2Robot", url="https://cad2robot.com", mentioned=r"cad2robot",
            what=("a pipeline from a robot's CAD to a MuJoCo simulation. Its exporter's motor and servo fields are "
                  "aliases here.")),
    Project(name="MuJoCo", url="https://mujoco.org", mentioned=r"mujoco",
            what=("the physics engine. Some rungs use its models as written (the servo's M5.dcmotor is its DC motor), "
                  "and most packed names are that model's parameter arrays.")),
    Project(name="robotparts", url="https://robotparts.dev", mentioned=r"robotparts|the registry|user-part",
            what=(f"an open catalogue of robot parts. Its actuator models' layers are the physics levels "
                  f"M0 to M{TOP_PHYSICS_LEVEL} here, its eight actuator rungs moved here as the servo's code "
                  "(actuators/rungs/), and its registry keys (\"the registry\") and old user-part keys are "
                  "canonical names or aliases.")),
    Project(name="sim2real", url="https://github.com/zacharyyamaoka/sim2real", mentioned=r"sim2real",
            what=("the outer API on top of this stack: twins and their calibration. Its type cards were the first "
                  "source of most ladders here, and its twin names are aliases.")),
    Project(name="the rack twin", url="", mentioned=r"rack twin",
            what=("a fitted model of one lab test rack, kept with robotparts' lab tools and never published. Only "
                  "its structure is used here: its terms and equations, never its fitted values.")),
)
PROJECTS_BY_NAME = {project.name: project for project in PROJECTS}


# --- markdown text ---------------------------------------------------------------------------------------------------

# WHY escape rather than strip (truthful rendering): a stored character shows as itself, never as markdown syntax.
_PROSE_ESCAPES = str.maketrans({
    "\\": "\\\\", "|": "\\|", "*": "\\*", "`": "\\`", "[": "\\[", "]": "\\]", "~": "\\~", "$": "\\$",
    "&": "&amp;", "<": "&lt;",
})
# WHY only an underscore at a word's edge: GitHub never reads one inside a word (coulomb_friction) as emphasis.
_EDGE_UNDERSCORE = re.compile(r"(?<!\w)_|_(?!\w)")


def one_line(text: str) -> str:
    return text.replace("\r\n", " ").replace("\n", " ")


def prose(text: str) -> str:
    """Stored text for a line or a table cell, shown as written: markdown's own characters escaped, on one line."""
    return _EDGE_UNDERSCORE.sub(r"\\_", one_line(text).translate(_PROSE_ESCAPES))


def code(text: str, table: bool = True) -> str:
    """Text as an inline code span; inside a table a pipe is written \\| (GFM splits cells before it reads code)."""
    text = one_line(text)
    if table:
        text = text.replace("|", "\\|")
    longest = max((len(run) for run in re.findall(r"`+", text)), default=0)
    fence = "`" * (longest + 1)
    pad = " " if text.startswith("`") or text.endswith("`") else ""
    return f"{fence}{pad}{text}{pad}{fence}"


def names(items: Iterable[str]) -> str:
    listed = [code(item) for item in items]
    return ", ".join(listed) if listed else NONE


def table(header: tuple[str, ...], rows: list[tuple[str, ...]], right: tuple[int, ...] = ()) -> list[str]:
    align = ["---:" if index in right else "---" for index in range(len(header))]
    lines = ["| " + " | ".join(header) + " |", "| " + " | ".join(align) + " |"]
    lines += ["| " + " | ".join(cell or " " for cell in row) + " |" for row in rows]
    return lines


def link(text: str, target: str) -> str:
    # WHY percent-encode parentheses: a DOI such as 10.1016/S0947-3580(98)70113-X would otherwise end the link early
    # in a renderer that does not balance them.
    return f"[{text}]({target.replace('(', '%28').replace(')', '%29').replace(' ', '%20')})"


def relative(source: PurePosixPath, target: PurePosixPath) -> str:
    """A link from the generated file `source` to `target`, both relative to the repo root."""
    return posixpath.relpath(target.as_posix(), source.parent.as_posix())


def anchor(html_id: str) -> str:
    return f'<a id="{html_id}"></a>'


def rung_anchor(rung: Rung) -> str:
    # WHY from the fields, never the key's text (sim2real CONTRACT "never parse a rung key"): level and name.
    return f"rung-m{rung.level}-{rung.name}"


def parameter_anchor(name: str) -> str:
    return f"param-{name}"


# --- references ------------------------------------------------------------------------------------------------------


def _module_labels() -> dict[Reference, str]:
    """{work: its name in robot_models.references}: the label a ladder.py cites it by (ARMSTRONG_1994)."""
    labels: dict[Reference, str] = {}
    for name in sorted(vars(references_module)):
        value = getattr(references_module, name)
        if isinstance(value, Reference):
            labels.setdefault(value, name)
    return labels


MODULE_LABELS = _module_labels()


def _fallback_label(reference: Reference) -> str:
    """A label for a work a ladder builds inline, not from robot_models.references: FIRSTAUTHOR_YEAR."""
    authors = reference.title.split("(")[0].strip()
    first = re.split(r"[,\s]", authors)[0] if authors else "WORK"
    year = re.search(r"\((\d{4})\)", reference.title)
    return re.sub(r"\W+", "_", f"{first}_{year.group(1) if year else ''}").strip("_").upper() or "WORK"


@dataclass(frozen=True)
class Cited:
    label: str
    reference: Reference
    by: tuple[Rung, ...]

    @property
    def html_id(self) -> str:
        return "ref-" + re.sub(r"[^a-z0-9_-]+", "-", self.label.lower())


def cited_works(rungs: tuple[Rung, ...]) -> dict[Reference, Cited]:
    """Every work the rungs cite, labelled as robot_models.references names it, with the rungs that cite it."""
    citing: dict[Reference, list[Rung]] = {}
    for rung in rungs:
        for reference in rung.references:
            citing.setdefault(reference, [])
            if rung not in citing[reference]:
                citing[reference].append(rung)
    labels: dict[Reference, str] = {}
    used: set[str] = set()
    for reference in citing:
        if reference in MODULE_LABELS:
            labels[reference] = MODULE_LABELS[reference]
            used.add(labels[reference])
    for reference in sorted((work for work in citing if work not in labels), key=lambda work: work.title):
        label, count = _fallback_label(reference), 1
        while label in used or label in MODULE_LABELS.values():
            count += 1
            label = f"{_fallback_label(reference)}_{count}"
        labels[reference] = label
        used.add(label)
    return {reference: Cited(label=labels[reference], reference=reference, by=tuple(by))
            for reference, by in citing.items()}


def reference_line(cited: Cited) -> str:
    reference = cited.reference
    parts = [f"{anchor(cited.html_id)}**{cited.label}**: {prose(reference.title)}"]
    if reference.doi:
        parts.append(link(f"doi:{prose(reference.doi)}", reference.url or f"https://doi.org/{reference.doi}"))
    elif reference.url:
        parts.append(link(prose(reference.url), reference.url))
    else:
        parts.append("no stable URL or DOI")
    if reference.kind != "paper":
        parts.append(f"({prose(reference.kind)})")
    line = " ".join(parts)
    if reference.note:
        line += f". What the rung uses: {prose(reference.note)}"
    by = ", ".join(link(code(rung.key, table=False), f"#{rung_anchor(rung)}") for rung in cited.by)
    return f"- {line}. Cited by {by}."


# --- one ladder ------------------------------------------------------------------------------------------------------


def bottom_up(ladder: Ladder) -> tuple[Rung, ...]:
    """The rungs from M0 up; within a level, in the order the ladder lists them (its branches stay together)."""
    order = {rung.key: index for index, rung in enumerate(ladder.rungs)}
    return tuple(sorted(ladder.rungs, key=lambda rung: (rung.level, order[rung.key])))


def node_id(rung: Rung, other_type: str = "") -> str:
    # WHY: a mermaid node id cannot hold a dot, so M3.stribeck is drawn as M3_stribeck; another type's rung is
    # T_<type>__M1_delay_loop, which can never collide with this ladder's own ids (they all start with M and a digit),
    # and the leading capital keeps an id clear of mermaid's "end" and its o/x edge endings.
    own = f"M{rung.level}_{rung.name}"
    return f"T_{other_type}__{own}" if other_type else own


def mermaid_text(text: str) -> str:
    """Text for inside a quoted mermaid label: mermaid's entity codes for the characters its parser reads."""
    return text.replace("#", "#35;").replace('"', "#quot;").replace("<", "#lt;").replace(">", "#gt;")


def node_label(rung: Rung) -> str:
    lines = [rung.key, *textwrap.wrap(rung.title, width=LABEL_WIDTH, break_long_words=False,
                                      break_on_hyphens=False)]
    return "<br/>".join(mermaid_text(line) for line in lines)


def node(rung: Rung, other_type: str = "") -> str:
    label = node_label(rung)
    identifier = node_id(rung, other_type)
    if other_type or rung.kind == "physics":
        return f'{identifier}["{label}"]'
    if rung.kind == "residual":
        return f'{identifier}{{{{"{label}"}}}}'
    return f'{identifier}(["{label}"])'


def node_class(rung: Rung) -> str:
    if rung.kind == "physics":
        return "wear" if rung.wears else "physics"
    return rung.kind


def mermaid(ladder: Ladder, found: dict[str, Found]) -> tuple[list[str], list[str]]:
    """The rung tree as a mermaid flowchart (M0 at the bottom), and the legend lines for what it draws."""
    rungs = bottom_up(ladder)
    lines = ["```mermaid", "flowchart BT"]
    lines += [f"  {node(rung)}" for rung in rungs]
    lines += [f"  {node_id(ladder.rung(rung.base))} --> {node_id(rung)}" for rung in rungs if rung.base is not None]
    classes: dict[str, list[str]] = {}
    for rung in rungs:
        classes.setdefault(node_class(rung), []).append(node_id(rung))
    composed: dict[str, list[Rung]] = {}
    edges: list[str] = []
    for rung in rungs:
        for other_type, key in rung.composes:
            other = found[other_type].ladder.rung(key)
            if other not in composed.setdefault(other_type, []):
                composed[other_type].append(other)
            edges.append(f"  {node_id(other, other_type)} -.->|composed into| {node_id(rung)}")
    for other_type in sorted(composed):
        lines.append(f'  subgraph S_{other_type} ["{mermaid_text(other_type)} (another type)"]')
        lines += [f"    {node(other, other_type)}" for other in bottom_up_of(composed[other_type])]
        lines.append("  end")
        classes.setdefault("other", []).extend(node_id(other, other_type)
                                               for other in bottom_up_of(composed[other_type]))
    lines += edges
    learned = [rung for rung in rungs if rung.kind == "learned"]
    below = [rung for rung in rungs if rung.kind != "learned"]
    for rung in learned:
        # WHY an invisible link: a learned net extends no model (base=None), so mermaid would rank it beside M0; the
        # link draws nothing and places it above the rungs it would replace.
        lines.append(f"  {node_id(below[-1])} ~~~ {node_id(rung)}")
    for name, style in MERMAID_CLASSES.items():
        if name in classes:
            lines.append(f"  classDef {name} {style}")
    for name in MERMAID_CLASSES:
        if name in classes:
            lines.append(f"  class {','.join(classes[name])} {name}")
    lines.append("```")
    legend = [("Read it bottom up: each solid arrow runs from a rung's base to the rung that extends it, and a rung "
               "needs everything below it on its own chain (its fallback).")]
    shapes = ["Blue box: a physics rung"]
    if "wear" in classes:
        shapes.append("dashed blue box: a wear rung (a parameter that grows with use)")
    if "residual" in classes:
        shapes.append(f"amber hexagon: the grey-box residual (M{RESIDUAL_LEVEL}) on a physics rung")
    if "learned" in classes:
        shapes.append(f"purple rounded box: the learned net (M{LEARNED_LEVEL}), which extends no model")
    if "other" in classes:
        shapes.append("grey box in a frame: another type's rung, its dashed arrow into the rung that composes it")
    legend.append("; ".join(shapes) + ".")
    return lines, legend


def bottom_up_of(rungs: list[Rung]) -> list[Rung]:
    return sorted(rungs, key=lambda rung: (rung.level, rung.name))


def datasheet_optional(ladder: Ladder, rung: Rung) -> bool:
    """True when the rung's optional names are exactly the typed fields its base does not hold (M1.datasheet)."""
    if ladder.datasheet is None or not rung.optional:
        return False
    held_below = set(ladder.holds(rung.base)) if rung.base else set()
    return set(rung.optional) == set(ladder.datasheet.typed_fields()) - held_below


def rung_rows(ladder: Ladder, found: dict[str, Found], out: PurePosixPath, readme_of: dict[str, PurePosixPath],
              cited: dict[Reference, Cited]) -> list[tuple[str, ...]]:
    rows = []
    for rung in bottom_up(ladder):
        title = prose(rung.title)
        if rung.note:
            title += f"<br>*{prose(rung.note)}*"
        if rung.wears:
            worn = ", ".join(code(wear.parameter) for wear in rung.wears)
            title += f"<br>wears {worn}: see {link('Wear', '#wear')}"
        for other_type, key in rung.composes:
            other = found[other_type].ladder.rung(key)
            target = relative(out, readme_of[other_type]) + f"#{rung_anchor(other)}"
            title += f"<br>composes {link(f'{code(other_type)} {code(key)}', target)}"
        base = link(code(rung.base), f"#{rung_anchor(ladder.rung(rung.base))}") if rung.base else NONE
        if datasheet_optional(ladder, rung):
            # WHY one phrase, not 33 names: the full list is one click away, each name's row under Vocabulary says
            # it is first read here (truthful rendering: shorten with a pointer, never drop silently).
            optional = (f"every {code(ladder.datasheet.__name__)} typed field its base does not hold: "
                        f"{len(rung.optional)} names, each listed under {link('Vocabulary', '#vocabulary')}")
        else:
            optional = names(rung.optional)
        code_cell = NONE
        if rung.code:
            folder = PACKAGE / rung.code
            target = PACKAGE_OUT / rung.code
            if (folder / "README.md").is_file():
                code_cell = link(code(rung.code), relative(out, target / "README.md"))
            elif folder.is_dir():
                code_cell = link(code(rung.code), relative(out, target))
            else:
                code_cell = code(rung.code)
        works = ", ".join(link(cited[work].label, f"#{cited[work].html_id}") for work in rung.references) or NONE
        rows.append((anchor(rung_anchor(rung)) + code(rung.key), title, base, names(rung.adds), optional, code_cell,
                     code(rung.equation) if rung.equation.strip() else NONE, works))
    return rows


def first_readers(ladder: Ladder) -> dict[str, tuple[Rung, bool, Rung | None]]:
    """{name: (the lowest rung that reads it, whether it only reads it when stated there, the lowest that needs it)}."""
    found: dict[str, tuple[Rung, bool, Rung | None]] = {}
    for rung in bottom_up(ladder):
        for name in rung.adds:
            first, stated, needed = found.get(name, (rung, False, None))
            found[name] = (first, stated, needed or rung)
        for name in rung.optional:
            if name not in found:
                found[name] = (rung, True, None)
    return found


def vocabulary_rows(ladder: Ladder) -> list[tuple[str, ...]]:
    readers = first_readers(ladder)
    position = {rung.key: index for index, rung in enumerate(bottom_up(ladder))}
    authored = {parameter.name: index for index, parameter in enumerate(ladder.vocabulary)}
    rows = []
    for parameter in sorted(ladder.vocabulary, key=lambda item: (
            position[readers[item.name][0].key] if item.name in readers else len(position), authored[item.name])):
        definition = prose(parameter.definition)
        if parameter.note:
            definition += f"<br>*{prose(parameter.note)}*"
        first_read = NONE
        if parameter.name in readers:
            first, stated, needed = readers[parameter.name]
            first_read = link(code(first.key), f"#{rung_anchor(first)}")
            if stated:
                first_read += " (when stated)"
                if needed is not None:
                    first_read += f"; needed from {link(code(needed.key), '#' + rung_anchor(needed))}"
        rows.append((anchor(parameter_anchor(parameter.name)) + code(parameter.name),
                     prose(parameter.unit) if parameter.unit else NONE, prose(parameter.shape), definition,
                     first_read))
    return rows


def canonical_link(name: str) -> str:
    return link(code(name), f"#{parameter_anchor(name)}")


def alias_unit(ladder: Ladder, alias: Alias) -> str:
    """The unit an alias's values are in: its own, else the canonical parameter's, spelled out and marked."""
    if alias.unit:
        return prose(alias.unit)
    # WHY spelled out and never the em dash: under Vocabulary the em dash means dimensionless, and a reader skimming
    # `mass_kg | —` takes it for dimensionless too. Alias.unit "" is the contract's "the canonical parameter's own SI
    # unit", so the cell shows that unit with "(canonical)": what is stored, with nothing lost.
    canonical_unit = ladder.parameter(alias.canonical).unit
    return f"{prose(canonical_unit) if canonical_unit else 'dimensionless'} *(canonical)*"


def by_name(items, key=lambda item: item.name):
    return sorted(items, key=lambda item: (key(item).casefold(), key(item)))


def readme(item: Found, found: dict[str, Found], readme_of: dict[str, PurePosixPath]) -> str:
    ladder = item.ladder
    out = readme_of[ladder.type]
    module = importlib.import_module(item.module)
    cited = cited_works(bottom_up(ladder))
    lines = [
        (f"> **Generated file.** Written by {link(code(GENERATOR.as_posix(), False), relative(out, GENERATOR))} "
         f"from this folder's {link(code('ladder.py', False), 'ladder.py')}: edit the ladder, not this file, then "
         f"run {code(REGENERATE, False)} from the repository root."),
        "",
        f"# {prose(ladder.type)}",
        "",
        prose(ladder.title),
        "",
    ]
    facts = [f"**{len(ladder.rungs)} rungs**", f"{len(ladder.vocabulary)} parameters", f"{len(ladder.aliases)} aliases",
             f"{len(ladder.packed)} packed names", f"{len(ladder.deployment)} deployment-only names",
             f"folder {code(item.folder + '/', False)}"]
    if ladder.datasheet is not None:
        source = Path(inspect.getsourcefile(ladder.datasheet)).resolve()
        name = code(ladder.datasheet.__name__, False)
        if source.is_relative_to(PACKAGE):
            name = link(name, relative(out, PACKAGE_OUT / source.relative_to(PACKAGE).as_posix()))
        facts.append(f"datasheet class {name}, whose typed fields rung {code('M1.datasheet', False)} states")
    lines += [" · ".join(facts) + ".", ""]
    lookup = code('ladders()["' + ladder.type + '"]', False)
    lines.append(f"In code: {lookup} from "
                 f"{link(code('robot_models.ladder', False), relative(out, PACKAGE_OUT / 'ladder.py'))}. "
                 f"Every type: {link('docs/TYPES.md', relative(out, INDEX))}. "
                 f"Spec: {link('PEP 0004', relative(out, PEP))}.")
    inside = [other for other in sorted(found.values(), key=lambda other: other.folder)
              if other.folder.startswith(item.folder + "/")]
    if inside:
        listed = ", ".join(link(code(other.ladder.type, False), relative(out, readme_of[other.ladder.type]))
                           for other in inside)
        lines.append(f"Types in the folders below this one: {listed}.")
    names_line_at = len(lines)
    lines.append("")
    if ladder.note:
        lines += [prose(ladder.note), ""]
    about, left_out = _about(module.__doc__ or "")
    if about:
        source = f"From the docstring of {link(code('ladder.py', False), 'ladder.py')}"
        if left_out:
            source += (f", less {left_out} on the order that file reads in (this page shows the ladder, then the "
                       "rungs, then the names)")
        lines += ["## About", "", source + ":", "", about, ""]
    diagram, legend = mermaid(ladder, found)
    lines += ["## The ladder", "", *diagram, "", " ".join(legend), ""]
    lines += ["## Rungs", "",
              ("Bottom up. *Needs* is what the rung's model cannot run without, new over its base; *reads when "
               "stated* is what it uses when a source states it. A rung needs everything its base needs."), ""]
    lines += table(("rung", "title", "base", "needs (adds)", "reads when stated (optional)", "code", "equation",
                    "references"), rung_rows(ladder, found, out, readme_of, cited))
    lines.append("")
    wear_rows = []
    for rung in bottom_up(ladder):
        for wear in rung.wears:
            wear_rows.append((link(code(rung.key), f"#{rung_anchor(rung)}"), canonical_link(wear.parameter),
                              f"{prose(wear.usage)} ({prose(wear.usage_unit)})",
                              f"{prose(wear.law)}: {code(WEAR_LAWS[wear.law])}", canonical_link(wear.rate)))
    if wear_rows:
        lines += ["## Wear", "", "Which parameter grows with use, with what, by which law, at which rate.", ""]
        lines += table(("rung", "parameter that wears", "grows with", "law", "rate"), wear_rows)
        lines.append("")
    lines += ["## Vocabulary", "",
              (f"Every parameter the ladder holds, by the rung that first reads it. {NONE} as a unit: dimensionless "
               "(a ratio or a count)."), ""]
    lines += table(("parameter", "SI unit", "shape", "definition", "first read on"), vocabulary_rows(ladder))
    lines.append("")
    lines += ["## Aliases", "",
              ("Other names a source uses for one of these parameters. A note says how it differs when it is not the "
               "same quantity one for one. The unit is the one the source writes its values in; *(canonical)* after "
               "it marks an alias stored with no unit of its own, which robot_models reads as the canonical "
               "parameter's SI unit, shown here spelled out. A note can still qualify that unit."), ""]
    if ladder.aliases:
        rows = [(code(alias.name), canonical_link(alias.canonical), alias_unit(ladder, alias), prose(alias.note))
                for alias in by_name(ladder.aliases)]
        lines += table(("alias", "canonical", "unit", "note"), rows)
    else:
        lines.append("None.")
    lines.append("")
    lines += ["## Packed names", "",
              "Names for one array that carries several of these parameters; never read as one parameter.", ""]
    if ladder.packed:
        rows = [(code(packed.name), ", ".join(canonical_link(name) for name in packed.holds),
                 "its full entry order" if packed.ordered else "at least these; entry order not stated",
                 prose(packed.note)) for packed in by_name(ladder.packed)]
        lines += table(("packed name", "carries", "order", "note"), rows)
    else:
        lines.append("None.")
    lines.append("")
    lines += ["## Deployment-only names", "",
              "Names some model reads that are not the type's own: a deployment sets them, or the mount gives them.",
              ""]
    if ladder.deployment:
        for role in sorted({other.role for other in ladder.deployment}):
            lines.append(f"- *{prose(role)}*: {prose(DEPLOYMENT_ROLES[role])}.")
        lines.append("")
        rows = [(code(other.name), prose(other.role), prose(other.why)) for other in by_name(ladder.deployment)]
        lines += table(("name", "role", "where it belongs"), rows)
    else:
        lines.append("None.")
    lines.append("")
    references_link = link(code("robot_models.references", False), relative(out, PACKAGE_OUT / "references.py"))
    lines += ["## References", "", f"The works the rungs cite, by their names in {references_link}.", ""]
    for work in sorted(cited.values(), key=lambda work: work.label):
        lines.append(reference_line(work))
    if not cited:
        lines.append("None.")
    body = "\n".join(lines[names_line_at:])
    named = [project for project in PROJECTS if re.search(project.mentioned, body, re.IGNORECASE)]
    if named:
        listed = ", ".join(link(prose(project.name), relative(out, INDEX) + f"#{project.html_id}") for project in named)
        lines.insert(names_line_at, f"Projects the notes below name, and what they are: {listed}.")
    return "\n".join(lines).rstrip("\n") + "\n"


# WHY drop a reading-order paragraph ("Read top to bottom: the vocabulary, ... then the rungs"): it describes the .py
# file's layout, and quoted under a README laid out differently it contradicts the page beneath it.
_READING_ORDER = re.compile(r"Read\b[^.:]*\b(top|bottom)\b")


def _about(docstring: str) -> tuple[str, str]:
    """The ladder module's docstring for the README, and what was left out of it ("" when nothing was).

    Left out: the first paragraph (the one line the README's title already shows) and any paragraph that tells a
    reader of ladder.py what order that file is in. Everything else is quoted as written.
    """
    paragraphs = re.split(r"\n\s*\n", inspect.cleandoc(docstring).strip("\n"))[1:]
    kept = [paragraph for paragraph in paragraphs if not _READING_ORDER.match(paragraph)]
    dropped = len(paragraphs) - len(kept)
    left_out = "" if not dropped else "a paragraph" if dropped == 1 else f"{dropped} paragraphs"
    return "\n\n".join(paragraph.strip("\n") for paragraph in kept), left_out


# --- the index -------------------------------------------------------------------------------------------------------


def top_physics(ladder: Ladder) -> tuple[Rung, ...]:
    """The physics rungs on the highest physics level the ladder reaches."""
    physics = [rung for rung in bottom_up(ladder) if rung.kind == "physics"]
    top = max(rung.level for rung in physics)
    return tuple(rung for rung in physics if rung.level == top)


def index(found: dict[str, Found], readme_of: dict[str, PurePosixPath]) -> str:
    out = INDEX
    items = sorted(found.values(), key=lambda item: item.folder)
    total = sum(len(item.ladder.rungs) for item in items)
    lines = [
        (f"> **Generated file.** Written by {link(code(GENERATOR.as_posix(), False), relative(out, GENERATOR))} "
         f"from each type folder's {code('ladder.py', False)}: edit the ladders, not this file, then run "
         f"{code(REGENERATE, False)} from the repository root."),
        "",
        f"# The {len(items)} types",
        "",
        (f"Every part type robot_models models, {total} rungs in all. Each type is a ladder of models in a folder of "
         f"its own under {link(code(PACKAGE_OUT.as_posix() + '/', False), relative(out, PACKAGE_OUT))}, and each "
         "folder's README.md shows its rung tree, its rungs with their equations and references, its vocabulary, "
         "aliases and the names that belong to a deployment instead."),
        "",
        "```text",
        STACK,
        "```",
        "",
    ]
    rows = []
    for item in items:
        ladder = item.ladder
        target = readme_of[ladder.type]
        residual_and_learned_rungs = [rung for rung in bottom_up(ladder) if rung.kind != "physics"]
        datasheet = NONE
        if ladder.datasheet is not None:
            source = Path(inspect.getsourcefile(ladder.datasheet)).resolve()
            datasheet = code(ladder.datasheet.__name__)
            if source.is_relative_to(PACKAGE):
                datasheet = link(datasheet, relative(out, PACKAGE_OUT / source.relative_to(PACKAGE).as_posix()))
        rows.append((
            link(code(ladder.type), relative(out, target)),
            prose(ladder.title),
            link(code(item.folder + "/"), relative(out, PACKAGE_OUT / item.folder)),
            str(len(ladder.rungs)),
            ", ".join(link(code(rung.key), relative(out, target) + f"#{rung_anchor(rung)}")
                      for rung in top_physics(ladder)),
            ", ".join(link(code(rung.key), relative(out, target) + f"#{rung_anchor(rung)}")
                      for rung in residual_and_learned_rungs) or NONE,
            datasheet,
        ))
    rows.append((f"**{len(items)} types**", "", "", f"**{total}**", "", "", ""))
    lines += table(("type", "what it is", "folder", "rungs", "top physics rung(s)", "residual, learned",
                    "datasheet class"), rows, right=(3,))
    lines += ["", "## The folder tree", ""]
    type_of = {item.folder: item.ladder.type for item in items}
    folders = sorted({PurePosixPath(*PurePosixPath(folder).parts[:depth])
                      for folder in type_of for depth in range(1, len(PurePosixPath(folder).parts) + 1)},
                     key=lambda folder: folder.parts)
    for folder in folders:
        indent = "  " * (len(folder.parts) - 1)
        entry = code(folder.name + "/", False)
        if folder.as_posix() in type_of:
            type_name = type_of[folder.as_posix()]
            entry = f"{link(entry, relative(out, readme_of[type_name]))}: {code(type_name, False)}"
        lines.append(f"{indent}- {entry}")
    compositions = []
    for item in items:
        for rung in bottom_up(item.ladder):
            for other_type, key in rung.composes:
                other = found[other_type].ladder.rung(key)
                here = relative(out, readme_of[item.ladder.type]) + f"#{rung_anchor(rung)}"
                there = relative(out, readme_of[other_type]) + f"#{rung_anchor(other)}"
                compositions.append(f"- {link(code(item.ladder.type, False) + ' ' + code(rung.key, False), here)} "
                                    f"composes {link(code(other_type, False) + ' ' + code(key, False), there)}")
    lines += ["", "## Across types", ""]
    lines += compositions or ["No rung composes another type's."]
    lines += [
        "",
        "## Reading a rung key",
        "",
        (f"A rung key is {code('M{level}.{name}', False)}. The physics levels M0 to M{TOP_PHYSICS_LEVEL} are the "
         f"actuator layers of {link('robotparts', '#' + PROJECTS_BY_NAME['robotparts'].html_id)}, where the "
         "numbering started: "
         + ", ".join(f"M{level} {layer}" for level, layer in enumerate(ACTUATOR_LAYERS))
         + ". Every other type climbs its own physics over the same levels, from its simplest model at M0 (the "
         f"ideal or nominal one). M{RESIDUAL_LEVEL} is always the grey-box residual and M{LEARNED_LEVEL} always the "
         f"learned net; a rung named {code('wear', False)} grows a parameter with use. Each rung extends one base "
         f"and needs everything its base needs. Spec: {link('PEP 0004', relative(out, PEP))}."),
        "",
        "## Projects the notes name",
        "",
        "The ladders' notes say where a rung or a name came from. These are the projects they name.",
        "",
    ]
    for project in PROJECTS:
        name = f"**{prose(project.name)}**"
        if project.url:
            name += f" ({link(prose(project.url.removeprefix('https://')), project.url)})"
        lines.append(f"- {anchor(project.html_id)}{name}: {prose(project.what)}")
    return "\n".join(lines).rstrip("\n") + "\n"


# --- the files -------------------------------------------------------------------------------------------------------

def generated() -> dict[PurePosixPath, str]:
    """{path relative to the repo root: its text} for every generated file."""
    found = discover()
    readme_of = {name: PACKAGE_OUT / item.folder / "README.md" for name, item in found.items()}
    files = {readme_of[name]: readme(item, found, readme_of) for name, item in sorted(found.items())}
    files[INDEX] = index(found, readme_of)
    return files


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true",
                        help="name each stale or missing generated file and exit 1; write nothing")
    parser.add_argument("--out", type=Path, default=ROOT,
                        help="the tree the generated files live in (default: this repository)")
    args = parser.parse_args(argv)
    stale = []
    for path, text in generated().items():
        target = args.out / path
        current = target.read_text(encoding="utf-8") if target.is_file() else None
        if current == text:
            continue
        if args.check:
            stale.append(f"{'stale' if current is not None else 'missing'}: {path.as_posix()}")
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {path.as_posix()}")
    if stale:
        print("\n".join(stale))
        print(f"regenerate: {REGENERATE}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
