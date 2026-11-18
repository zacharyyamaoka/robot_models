"""Build round 2's metrics-API gallery: the five glances side by side, the scoreboard, then each variant's session.

    uv run --offline --project /home/bam/robot_models --with markdown-it-py python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/build_gallery.py

What it reads, per slot (baseline, r2-1 ... r2-4; the folder is ``<id>`` or ``<id>_*`` in this folder, the way
check_standalone.py finds it):
- ``session.json``: the glance (``kind: "glance"``) and the session cells, recorded by round 1's ``_session.py``;
- ``scores.json``: the lane's self-score, ``{"scores": {"lg": {"score", "evidence"}, "fr1": ...}, "lg_unexplained":
  [...], "total": n}``;
- ``README.md``: the direction, the prior art and the live/offline note (matched by their ``##`` headings).
Plus ``dispatcher_lg.json`` in this folder when it exists (``{variant_id: {"score": n, "evidence": "..."}}``), and from
this folder's README (the frozen contract) the criteria, their weights, the coined words and the directions, so
nothing in the contract is copied here.

What it writes: ``gallery-spec.json`` here (what check_standalone.py reads: each variant's ``preview`` is its glance and
its one ``media`` item its session, both the exact html the page draws) and
``/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-18.html``. A slot with nothing yet shows
"not built yet", and a half-built one shows what exists, so the page builds at any time.

``--stand-in-round1`` puts round 1's v1-v5 sessions in the five slots, labelled, to prove the layout before round 2's
variants land. It never writes gallery-spec.json: the standalone check must only ever see round 2's own variants.

Round 1's builder is imported for its Pygments highlighting, its escaping, its statement grouping and (for the
stand-in) its variant names and scores. WHY the glances lead (Zach, 2026-11-18): "I am not seeing what is going on at
a glance. I don't know what you mean by fold etc."
"""

from __future__ import annotations

import argparse
import html
import importlib.util
import json
import os
import pathlib
import re
import sys
import tempfile

try:
    from markdown_it import MarkdownIt
except ImportError:  # WHY: a lane may run this with the venv's python just to refresh gallery-spec.json for G2
    MarkdownIt = None

HERE = pathlib.Path(__file__).resolve().parent
ROUND1 = HERE.parent / "metrics-api-2026-11-17"
CONTRACT = HERE / "README.md"
SPEC = HERE / "gallery-spec.json"
DISPATCHER_LG = HERE / "dispatcher_lg.json"
OUT = pathlib.Path("/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-18.html")
REBUILD = ("uv run --offline --project /home/bam/robot_models --with markdown-it-py python "
           "/home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/build_gallery.py")

SLOTS = ("baseline", "r2-1", "r2-2", "r2-3", "r2-4")
STAND_IN = dict(zip(SLOTS, ("v1", "v2", "v3", "v4", "v5")))
ACCENTS = dict(zip(SLOTS, ("#57606a", "#3f7be0", "#2e9e6b", "#c9822b", "#b5487a")))
MAX_LINE = 90            # Zach's ~90-character lines (readable-Python rule)
MAX_PAGE_BYTES = 1_200_000   # the desktop preview's ceiling for a text page
README_PARTS = (("Direction", r"\bdirection|\bthesis\b|\bthe face\b|\bapproach\b|\bidea\b"),
                ("Prior art", r"\bprior[ -]?art"),
                ("Live and offline", r"\blive\b|\boffline\b|\bstream"))   # \b: "Where it lives" is not live


def load_round1():
    """Round 1's build_gallery.py under its own module name (this file is also called build_gallery)."""

    spec = importlib.util.spec_from_file_location("round1_build_gallery", ROUND1 / "build_gallery.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ROUND1_GALLERY = load_round1()
esc = ROUND1_GALLERY.esc


# ---------------------------------------------------------------------------------------------------------- markdown

class Markdown:
    """README and contract text to html. Relative links are rewritten to work from reports/."""

    def __init__(self) -> None:
        self.md = MarkdownIt("commonmark", {"html": False}).enable("table") if MarkdownIt else None
        if self.md:
            self.md.add_render_rule("link_open", _link_open)

    def block(self, text: str, base: str = "") -> str:
        if not self.md:
            return f'<pre class="md-raw">{esc(text)}</pre>'
        return self.md.render(text, {"base": base})

    def inline(self, text: str, base: str = "") -> str:
        if not self.md:
            return re.sub(r"`([^`]+)`", r"<code>\1</code>", esc(text))
        return self.md.renderInline(text, {"base": base})


def _link_open(renderer, tokens, index, options, env):
    href = tokens[index].attrGet("href") or ""
    if href and env.get("base") and not re.match(r"[a-zA-Z][\w+.-]*:|#|/", href):
        tokens[index].attrSet("href", env["base"] + href)
    return renderer.renderToken(tokens, index, options, env)


MARKDOWN = Markdown()


def plain(text: str) -> str:
    """Contract cell text without its markdown marks, for title attributes."""

    return re.sub(r"[*`]", "", text)


# ---------------------------------------------------------------------------------------------------------- contract

def table_rows(text: str, first_cell: str) -> list[list[str]]:
    rows = []
    for line in text.splitlines():
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if re.fullmatch(first_cell, cells[0]):
            rows.append(cells)
    return rows


def read_contract() -> dict:
    """The criteria table (weights, anchors), the coined words of lg's 1-anchor, and the directions table."""

    text = CONTRACT.read_text(encoding="utf-8")
    criteria = []
    for cells in table_rows(text, r"lg|fr\d"):
        criterion_id, name, weight, anchor_1, anchor_3, anchor_5 = cells[:6]
        criteria.append({"id": criterion_id, "name": name.strip("*"), "weight": float(weight),
                         "anchors": {"1": anchor_1, "3": anchor_3, "5": anchor_5}})
    weights = sum(criterion["weight"] for criterion in criteria)
    if [c["id"] for c in criteria] != ["lg", *(f"fr{n}" for n in range(1, 8))] or abs(weights - 100) > 1e-9:
        raise SystemExit(f"{CONTRACT}: expected lg, fr1-fr7 summing to 100, read {criteria}")
    coined = re.search(r"\(([^)]*)\)", criteria[0]["anchors"]["1"])
    directions = {cells[0]: {"name": cells[1], "face": cells[2], "live": cells[3], "many_runs": cells[4],
                             "gate": cells[5], "prior_art": cells[6]}
                  for cells in table_rows(text, r"baseline|r2-\d")}
    return {"criteria": criteria, "directions": directions,
            "coined": [word.strip() for word in coined.group(1).split(",")] if coined else []}


# ---------------------------------------------------------------------------------------------------------- reading

def read_json(path: pathlib.Path) -> tuple[object, str | None]:
    """(data, problem): a missing file is (None, None); a half-written or bad one is (None, "why")."""

    if not path.exists():
        return None, None
    try:
        return json.loads(path.read_text(encoding="utf-8")), None
    except (OSError, ValueError) as error:
        return None, f"{path.name} unreadable: {error}"


def find_folder(slot: str) -> tuple[pathlib.Path | None, str | None]:
    matches = sorted(path for path in HERE.iterdir()
                     if path.is_dir() and (path.name == slot or path.name.startswith(slot + "_")))
    if len(matches) > 1:
        return matches[0], f"several folders for {slot}: {', '.join(m.name for m in matches)}; showing {matches[0].name}"
    return (matches[0] if matches else None), None


def score_entry(entry: object) -> dict:
    if isinstance(entry, dict):
        return {"score": entry.get("score"), "evidence": str(entry.get("evidence") or "")}
    if isinstance(entry, (int, float)) and not isinstance(entry, bool):
        return {"score": entry, "evidence": ""}
    return {"score": None, "evidence": ""}


def readme_parts(text: str) -> tuple[str, dict[str, list[tuple[str, str]]], str]:
    """(title, {part: [(heading, markdown)]}, intro). Each ``##`` section goes to the first part its heading names;
    with no direction section, the text between the title and the first ``##`` is the direction."""

    title, intro, sections, current, in_fence = "", [], [], None, False
    for line in text.splitlines():
        if line.lstrip().startswith(("```", "~~~")):
            in_fence = not in_fence
        if not in_fence and line.startswith("# ") and not title and current is None:
            title = line[2:].strip()
            continue
        if not in_fence and line.startswith("## "):
            current = (line[3:].strip(), [])
            sections.append(current)
            continue
        (current[1] if current else intro).append(line)
    parts: dict[str, list[tuple[str, str]]] = {name: [] for name, _ in README_PARTS}
    for heading, body in sections:
        for name, pattern in README_PARTS:
            if re.search(pattern, heading, re.I):
                parts[name].append((heading, "\n".join(body).strip()))
                break
    intro_text = "\n".join(intro).strip()
    if not parts["Direction"] and intro_text:
        parts["Direction"].append(("", intro_text))
    return title, parts, intro_text


def load_slot(slot: str, contract: dict, stand_in: bool, dispatcher: dict) -> dict:
    direction = contract["directions"].get(slot, {})
    record = {"id": slot, "accent": ACCENTS[slot], "direction": direction, "name": direction.get("name", slot),
              "standIn": None, "folder": None, "folderPath": None, "cells": None, "scores": None,
              "lgUnexplained": None, "selfTotal": None, "readme": None, "problems": [],
              "dispatcher": None if stand_in else dispatcher.get(slot)}
    if stand_in:
        round1_id = STAND_IN[slot]
        round1_variant = ROUND1_GALLERY.VARIANTS[round1_id]
        folder = ROUND1 / ROUND1_GALLERY.FOLDERS[round1_id]
        record["standIn"] = {"id": round1_id, "name": round1_variant["name"]}
        record["scores"] = {fr: {"score": score, "evidence": f"{evidence} (round 1's score, confidence {confidence})"}
                            for fr, (score, evidence, confidence) in round1_variant["scores"].items()}
    else:
        folder, problem = find_folder(slot)
        if problem:
            record["problems"].append(problem)
    if folder is None:
        return record
    record["folderPath"] = folder
    record["folder"] = os.path.relpath(folder, HERE)
    cells, problem = read_json(folder / "session.json")
    if problem:
        record["problems"].append(problem)
    elif cells is not None:
        if isinstance(cells, list) and all(isinstance(cell, dict) and "statements" in cell for cell in cells):
            record["cells"] = cells
        else:
            record["problems"].append("session.json is not a list of recorded cells")
    if not stand_in:
        scores, problem = read_json(folder / "scores.json")
        if problem:
            record["problems"].append(problem)
        elif isinstance(scores, dict):
            record["scores"] = {key: score_entry(value) for key, value in (scores.get("scores") or {}).items()}
            unexplained = scores.get("lg_unexplained")
            record["lgUnexplained"] = [str(item) for item in unexplained] if isinstance(unexplained, list) else None
            record["selfTotal"] = scores.get("total") if isinstance(scores.get("total"), (int, float)) else None
        elif scores is not None:
            record["problems"].append("scores.json is not an object")
    if (folder / "README.md").exists():
        record["readme"] = (folder / "README.md").read_text(encoding="utf-8")
    return record


def status_of(record: dict) -> str:
    if record["standIn"]:
        return "stand-in"
    if record["cells"] and record["scores"]:
        return "built"
    if record["cells"] or record["scores"]:
        return "in progress"
    return "not built yet"


# ---------------------------------------------------------------------------------------------------------- scoring

def weighted_total(scores: dict, criteria: list[dict], lg_override: object = None) -> tuple[float, list[str]]:
    """Sum of weight x score / 5 (out of 100), and the criteria with no score."""

    total, missing = 0.0, []
    for criterion in criteria:
        value = (scores or {}).get(criterion["id"], {}).get("score")
        if criterion["id"] == "lg" and lg_override is not None:
            value = lg_override
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            total += criterion["weight"] * value / 5
        else:
            missing.append(criterion["id"])
    return total, missing


def number(value: object) -> str:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return "—"
    return f"{value:.1f}".rstrip("0").rstrip(".") if isinstance(value, float) else str(value)


def word_parts(token: str) -> list[str]:
    spaced = re.sub(r"([a-z0-9])([A-Z])", r"\1 \2", token)
    return [part.lower() for part in re.split(r"[\s_]+", spaced) if part]


def coined_words_on(statements: list[dict], coined: list[str]) -> list[str]:
    """Tokens on the glance (code, comments and outputs) built from the contract's coined words (fold, state, ...).
    A word scan, not a judgment: it lists candidates for the lg evidence, it scores nothing. Stdlib module names
    (``collections``) are skipped; a Python user knows them."""

    stems = {word.lower() for word in coined}
    suffixes = ("", "s", "es", "d", "ed", "ing")
    found: list[str] = []
    for statement in statements:
        for token in re.findall(r"[A-Za-z_][A-Za-z0-9_]*", statement["code"] + "\n" + statement["output"]):
            if token in sys.stdlib_module_names or token.lower() in (seen.lower() for seen in found):
                continue
            if any(part == stem + suffix for part in word_parts(token) for stem in stems for suffix in suffixes):
                found.append(token)
    return found


# ---------------------------------------------------------------------------------------------------------- drawing

LEADING_SPACES = re.compile(r'^(?:<span class="w">( +)</span>|( +))')


def mark_indent(line_html: str) -> str:
    """A line's leading spaces in their own span, taken out of the flow; the line's padding and text-indent draw
    the indent instead (capped at 10 characters in a narrow column). Every space stays in the text, so the check
    and a copy see the real code. WHY: in-flow spaces are a place to break, so a long first word moved to the
    next line and left the indent alone on an empty one; an argument aligned 37 columns in filled a whole line."""

    match = LEADING_SPACES.match(line_html)
    if not match:
        return line_html
    return f'<span class="ind">{match.group(1) or match.group(2)}</span>' + line_html[match.end():]


def code_block(code: str) -> str:
    """Round 1's Pygments highlighting, one block per source line with a hanging indent: in a narrow column a long
    line wraps under its own indent + 3 beside a thin bar (only a wrapped line has one, so it never reads as a
    block's indent), and a wrapped line still reads as one line. Every line keeps its newline,
    so check_standalone.py extracts exactly the code that ran."""

    lines = ROUND1_GALLERY.code_html(code).split("\n")
    sources = code.split("\n")
    if len(lines) != len(sources):   # WHY: never drop a line the check must see; unhighlighted beats missing
        lines = [esc(source) for source in sources]
    drawn = []
    for line_html, source in zip(lines, sources):
        indent = len(source) - len(source.lstrip(" "))
        style = f' style="--h:{indent}ch"' if indent else ""
        drawn.append(f'<span class="ln"{style}>{mark_indent(line_html)}\n</span>')
    return "".join(drawn)


def flow_html(statements: list[dict]) -> str:
    """Round 1's flow: consecutive statements share one code block and each output breaks it right under the
    statement that printed it (the shape check_standalone.py parses: div.stmt > pre.code + pre.out)."""

    chunks: list[dict] = []
    for statement in statements:
        if not chunks or statement["gap"] or chunks[-1]["output"]:
            chunks.append({"gap": statement["gap"] and bool(chunks), "code": [], "output": ""})
        chunks[-1]["code"].append(statement["code"])
        chunks[-1]["output"] = (statement.get("output") or "").rstrip()
    parts = []
    for chunk in chunks:
        output = chunk["output"]
        if len(output) > ROUND1_GALLERY.OUTPUT_LIMIT:
            output = output[:ROUND1_GALLERY.OUTPUT_LIMIT] + "\n... (full output in scenario.out.txt)"
        drawn_output = f'<pre class="out">{esc(output)}</pre>' if output else ""
        gap = " gap" if chunk["gap"] else ""
        parts.append(f'<div class="stmt{gap}"><pre class="code">{code_block(chr(10).join(chunk["code"]))}</pre>'
                     f'{drawn_output}</div>')
    return "".join(parts)


def glance_of(cells: list[dict] | None) -> dict | None:
    return next((cell for cell in cells or [] if cell.get("kind") == "glance"), None)


def session_cells(cells: list[dict] | None) -> list[dict]:
    return [cell for cell in cells or [] if cell.get("kind") != "glance"]


def lines_of_code(cell: dict) -> int:
    return sum(1 for statement in cell["statements"] for line in statement["code"].splitlines()
               if line.strip() and not line.lstrip().startswith("#"))


def long_lines(cells: list[dict]) -> list[str]:
    return [line for cell in cells for statement in cell["statements"] for line in statement["code"].splitlines()
            if len(line) > MAX_LINE]


def session_html(cells: list[dict]) -> str:
    """The session as one notebook (check_standalone.py runs a media item's cells as one script, in page order)."""

    parts = []
    for number_, cell in enumerate(cells, start=1):
        caption = f'<p class="caption">{esc(cell.get("caption", ""))}</p>' if cell.get("caption") else ""
        parts.append(f'<div class="cell-head"><b>{number_} · {esc(cell.get("label", ""))}</b></div>{caption}'
                     f'{flow_html(cell["statements"])}')
    return f'<div class="session">{"".join(parts)}</div>'


def heading_html(record: dict) -> str:
    """A slot's name: the contract's direction, or for the stand-in round 1's variant with the slot beside it."""

    if record["standIn"]:
        return (f'<span class="standin">stand-in, round 1 {esc(record["standIn"]["id"])}</span>'
                f'<span class="vname">{esc(record["standIn"]["name"])}</span>')
    return f'<span class="vname">{MARKDOWN.inline(record["name"])}</span>'


def terms_html(record: dict, scan: list[str]) -> str:
    """Under each glance: the lane's lg_unexplained list (the contract's 'Evidence for lg'), the dispatcher's
    re-score when there is one, and any coined word the scan found that the list leaves out."""

    head = '<h4>Terms a newcomer would need explained</h4>'
    status = status_of(record)
    if record["standIn"]:
        listed = "".join(f"<li><code>{esc(token)}</code></li>" for token in scan)
        body = (f'<p class="note">Stand-in: round 1 has no scores.json. A word scan for the contract\'s coined words '
                f'finds:</p>' + (f"<ul>{listed}</ul>" if scan else '<p class="none">none</p>'))
    elif status == "not built yet":
        body = '<p class="none">—</p>'
    elif record["lgUnexplained"] is None:
        body = '<p class="none">No list yet (no scores.json).</p>'
    elif not record["lgUnexplained"]:
        body = '<p class="none">None: the lane lists no term a newcomer would need explained.</p>'
    else:
        body = "<ul>" + "".join(f"<li>{MARKDOWN.inline(item)}</li>" for item in record["lgUnexplained"]) + "</ul>"
    if not record["standIn"] and record["lgUnexplained"] is not None:
        listed_text = " ".join(record["lgUnexplained"]).lower()
        missed = [token for token in scan if token.lower() not in listed_text]
        if missed:
            words = ", ".join(f"<code>{esc(token)}</code>" for token in missed)
            body += f'<p class="note">A word scan for the contract\'s coined words also finds: {words}</p>'
    if record["dispatcher"]:
        dispatcher = record["dispatcher"]
        body += (f'<p class="dispatcher"><b>Dispatcher\'s lg: {esc(number(dispatcher.get("score")))}</b> · '
                 f'{MARKDOWN.inline(str(dispatcher.get("evidence") or ""))}</p>')
    return f'<section class="terms">{head}{body}</section>'


def glance_column(record: dict, groups: list[list[dict]] | None, rows: int, aligned: bool, scan: list[str]) -> str:
    """One glance as a column of the top grid: header row, its groups (one grid row each when every built glance
    has the same number, so the same story step lines up across columns), and the terms row."""

    status = status_of(record)
    glance = glance_of(record["cells"])
    lg_self = (record["scores"] or {}).get("lg", {}).get("score")
    meta = []
    if glance:
        meta.append(f"{lines_of_code(glance)} lines of code")
    if record["standIn"]:
        meta.append("lg not scored")
    elif lg_self is not None:
        meta.append(f"lg {esc(number(lg_self))} self")
    if record["dispatcher"]:
        meta.append(f"{esc(number(record['dispatcher'].get('score')))} dispatcher")
    if status == "in progress":
        meta.append("in progress")
    header = (f'<header class="ghead"><a href="#v-{record["id"]}"><span class="vid">{record["id"]}</span>'
              f'{heading_html(record)}</a><span class="meta">{" · ".join(meta)}</span></header>')
    if groups:
        drawn = [f'<div class="group">{flow_html(group)}</div>' for group in groups]
        body = "".join(drawn) if aligned else f'<div class="gbody">{"".join(drawn)}</div>'
    else:
        why = "; ".join(record["problems"]) or (
            f"no session.json in {record['folder']}/" if record["folder"] else f"no {record['id']}_* folder yet")
        if record["cells"] is not None and not glance:
            why = "session.json has no glance cell (kind: \"glance\")"
        body = (f'<div class="placeholder" style="grid-row:span {rows}"><b>not built yet</b>'
                f'<span>{esc(why)}</span></div>')
    return (f'<article class="gcol" data-slot="{record["id"]}" style="--accent:{record["accent"]}">'
            f'{header}{body}{terms_html(record, scan)}</article>')


def scoreboard_html(records: list[dict], criteria: list[dict], stand_in: bool) -> str:
    head = ['<th class="name">variant</th>']
    for criterion in criteria:
        label = criterion["id"] + (" self" if criterion["id"] == "lg" else "")
        head.append(f'<th title="{esc(criterion["name"])}">{label}<small>{number(criterion["weight"])}</small></th>')
        if criterion["id"] == "lg":
            head.append('<th title="the dispatcher\'s re-score from the glance alone">lg<small>dispatcher</small></th>')
    head.append('<th>total<small>self</small></th><th>total<small>with dispatcher lg</small></th>')
    rows, totals = [], []
    for record in records:
        status = status_of(record)
        name = (f'<td class="name"><a href="#v-{record["id"]}"><span class="vid" style="color:{record["accent"]}">'
                f'{record["id"]}</span> {heading_html(record)}</a></td>')
        if not record["scores"]:
            span = len(criteria) + 3
            rows.append(f'<tr>{name}<td class="pending" colspan="{span}">{esc(status)}: no scores.json yet</td></tr>')
            continue
        cells = []
        for criterion in criteria:
            entry = record["scores"].get(criterion["id"], {})
            cells.append(f'<td class="s s{esc(number(entry.get("score")))}" title="{esc(entry.get("evidence", ""))}">'
                         f'{esc(number(entry.get("score")))}</td>')
            if criterion["id"] == "lg":
                dispatcher = record["dispatcher"] or {}
                cells.append(f'<td class="s disp" title="{esc(str(dispatcher.get("evidence") or ""))}">'
                             f'{esc(number(dispatcher.get("score")))}</td>')
        computed, missing = weighted_total(record["scores"], criteria)
        shown = record["selfTotal"] if record["selfTotal"] is not None else computed
        total_text = number(round(shown, 1))
        if missing:
            total_text += f'<small>{" ".join(missing)} unscored</small>'
        elif record["selfTotal"] is not None and abs(record["selfTotal"] - computed) > 0.05:
            total_text += f'<small>recomputed {computed:.1f}</small>'
        dispatcher_score = (record["dispatcher"] or {}).get("score")
        with_dispatcher = "—"
        if dispatcher_score is not None:
            value, _ = weighted_total(record["scores"], criteria, lg_override=dispatcher_score)
            with_dispatcher = f"{value:.1f}"
            totals.append(value)
        rows.append(f'<tr>{name}{"".join(cells)}<td class="total">{total_text}</td>'
                    f'<td class="total">{with_dispatcher}</td></tr>')
    legend = " · ".join(f'<b>{c["id"]}</b> {esc(c["name"])} ({number(c["weight"])})' for c in criteria)
    if stand_in:
        note = ("<b>Stand-in.</b> These rows are round 1's own fr1-fr7 scores for its v1-v5 (the same anchors), with no "
                "lg: round 1 was never scored on it. Round 2's rows replace them when the lanes land.")
    else:
        note = ("<b>Self-scores.</b> Each lane scored its own variant against the contract's 1 / 3 / 5 anchors. The "
                "dispatcher re-scores lg from the glance alone: that column reads <code>dispatcher_lg.json</code> and "
                "shows — until it exists.")
    return (f'<div class="scorewrap"><table class="scores"><thead><tr>{"".join(head)}</tr></thead>'
            f'<tbody>{"".join(rows)}</tbody></table></div>'
            f'<p class="note">{note} Total = Σ weight × score / 5, out of 100. Hover a score for its evidence; each '
            f'variant\'s section below lists it.</p><p class="legend">{legend}</p>')


def readme_html(record: dict) -> str:
    if not record["readme"]:
        return '<p class="none">No README.md yet.</p>'
    base = os.path.relpath(record["folderPath"], OUT.parent) + "/"
    title, parts, _ = readme_parts(record["readme"])
    drawn = [f'<p class="readme-title">README: {MARKDOWN.inline(title, base)}</p>' if title else ""]
    for name, _ in README_PARTS:
        drawn.append(f"<h3>{name}</h3>")
        if not parts[name]:
            drawn.append(f'<p class="none">The README has no section whose heading names this.</p>')
        for heading, body in parts[name]:
            if heading and heading.lower() != name.lower():
                drawn.append(f'<h4>{MARKDOWN.inline(heading, base)}</h4>')
            drawn.append(f'<div class="readme">{MARKDOWN.block(body, base)}</div>')
    drawn.append(f'<details><summary>The whole README</summary><div class="readme">'
                 f'{MARKDOWN.block(record["readme"], base)}</div></details>')
    return "".join(drawn)


def evidence_html(record: dict, criteria: list[dict]) -> str:
    if not record["scores"]:
        return ""
    rows = []
    for criterion in criteria:
        entry = record["scores"].get(criterion["id"], {})
        rows.append(f'<tr><td><b>{criterion["id"]}</b> {esc(criterion["name"])}</td>'
                    f'<td class="s">{esc(number(entry.get("score")))}</td>'
                    f'<td>{MARKDOWN.inline(entry.get("evidence", ""))}</td></tr>')
    who = "round 1's scores" if record["standIn"] else "the lane's self-score"
    return (f'<details><summary>Score evidence ({who})</summary><table class="evidence">{"".join(rows)}</table>'
            f'</details>')


def variant_section(record: dict, criteria: list[dict]) -> str:
    direction = record["direction"]
    status = status_of(record)
    contract_line = ""
    if direction:
        prefix = "This slot's round-2 direction" if record["standIn"] else "The contract's direction"
        contract_line = (f'<p class="face"><b>{prefix}:</b> {MARKDOWN.inline(direction["face"])} '
                         f'<span class="dim">· prior art: {MARKDOWN.inline(direction["prior_art"])}</span></p>')
    where = f"<code>{esc(record['folder'])}/</code>" if record["folder"] else "no folder yet"
    problems = "".join(f'<p class="problem">{esc(problem)}</p>' for problem in record["problems"])
    cells = session_cells(record["cells"])
    if cells:
        session = (f"<h3>The session that ran</h3><p class=\"note\">Every statement with its own output, top to "
                   f"bottom; the first cell is the setup (imports, then the real data).</p>{session_html(cells)}")
    else:
        session = f'<h3>The session that ran</h3><p class="none">{esc(status)}: no session cells yet.</p>'
    readme = readme_html(record) if record["folder"] else ""
    return (f'<section class="variant" id="v-{record["id"]}" style="--accent:{record["accent"]}">'
            f'<header><h2><span class="vid">{record["id"]}</span> {heading_html(record)}</h2>{contract_line}'
            f'<p class="where">{where} · {esc(status)}</p>{problems}</header>'
            f'{session}{readme}{evidence_html(record, criteria)}</section>')


STYLE = """
:root{color-scheme:light;--ink:#1d232b;--muted:#57606a;--faint:#6e7781;--line:#e1e5ea;--code-bg:#f6f8fa;
  --panel:#f9fafb;--bg:#fff}
*{box-sizing:border-box}
html{-webkit-text-size-adjust:100%}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",
  Roboto,sans-serif}
a{color:inherit}
code{font:12.5px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;background:#eff2f5;border-radius:3px;
  padding:0 3px;overflow-wrap:anywhere}
code.nb{white-space:nowrap}
.page{max-width:1880px;margin:0 auto;padding:20px 16px 56px}
.prose{max-width:1100px}
.kicker{font-size:12px;color:var(--muted);letter-spacing:.04em;text-transform:uppercase}
h1{font-size:24px;line-height:1.25;margin:4px 0 8px}
h2{font-size:19px;margin:0 0 6px}
.lede{color:var(--muted);margin:0 0 6px;max-width:110ch}
.lede q{color:var(--ink)}
.banner{border:1px solid #e3c766;background:#fff8dc;border-radius:8px;padding:8px 12px;margin:10px 0;max-width:110ch}
.status{font-size:13px;color:var(--muted);margin:6px 0 0}
.toolbar{display:flex;flex-wrap:wrap;align-items:center;gap:8px 16px;margin:16px 0 10px}
.toolbar h2{margin:0 8px 0 0}
.seg{display:inline-flex;border:1px solid #c9d1d9;border-radius:7px;overflow:hidden}
.seg button{font:inherit;font-size:13px;padding:5px 12px;border:0;background:#fff;color:var(--ink);cursor:pointer}
.seg button+button{border-left:1px solid #c9d1d9}
.seg button[aria-pressed=true]{background:var(--ink);color:#fff}
.picks{display:inline-flex;flex-wrap:wrap;gap:6px 12px;font-size:13px;color:var(--muted)}
.picks select{font:inherit;font-size:13px;padding:3px 4px;max-width:46vw}
.story{font-size:12.5px;color:var(--muted);margin:0 0 10px;max-width:120ch}
.glances{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));grid-auto-rows:auto;gap:0 12px}
.gcol{grid-row:span var(--rows);display:grid;grid-template-rows:subgrid;min-width:0;background:#fff;
  border:1px solid var(--line);border-top:3px solid var(--accent);border-radius:8px;margin-bottom:14px}
.gcol>*{min-width:0}
.ghead{padding:8px 10px 7px;border-bottom:1px solid var(--line)}
.ghead a{text-decoration:none;display:block}
.vid{font:600 12.5px ui-monospace,SFMono-Regular,Menlo,monospace;color:var(--accent);margin-right:6px}
.vname{font-weight:600;font-size:14px}
.ghead .vname{display:block;font-size:13.5px;line-height:1.3}
.standin{font-size:11px;color:#8a6d00;background:#fff4c2;border-radius:4px;padding:0 5px;margin-right:4px}
.ghead .meta{display:block;font-size:11.5px;color:var(--faint);margin-top:2px}
.group,.gbody{padding:6px 10px 4px}
.gbody .group{padding:0}
.gbody .group+.group{margin-top:12px}
.group+.group{border-top:1px dashed #eef0f2}
.placeholder{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:4px;min-height:160px;
  margin:10px;border:1px dashed #c9d1d9;border-radius:6px;color:var(--faint);text-align:center;padding:12px;
  font-size:12.5px}
.placeholder b{font-size:14px;color:var(--muted)}
.terms{border-top:1px solid var(--line);background:var(--panel);border-radius:0 0 8px 8px;padding:8px 10px 10px;
  font-size:12.5px}
.terms h4{margin:0 0 4px;font-size:11px;letter-spacing:.04em;text-transform:uppercase;color:var(--muted)}
.terms ul{margin:0;padding-left:18px}
.terms li+li{margin-top:2px}
.none{color:var(--faint);margin:0}
.note{color:var(--muted);font-size:12.5px;margin:4px 0}
.dispatcher{margin:6px 0 0;padding-top:6px;border-top:1px solid var(--line)}
.stmt{display:grid;grid-template-columns:minmax(0,1fr);gap:2px;margin-top:4px}
.stmt.gap{margin-top:14px}
pre{margin:0;min-width:0;max-width:100%}
pre.code{font:12.5px/1.5 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;color:#1f2328;background:var(--code-bg);
  border:1px solid var(--line);border-radius:6px;padding:4px 8px;white-space:pre-wrap;overflow-wrap:anywhere}
pre.code{--cap:48ch;position:relative}
pre.code .ln{display:block;padding-left:calc(min(var(--h,0ch),var(--cap)) + 3ch);text-indent:-3ch;
  background:linear-gradient(#c9d1d9,#c9d1d9) no-repeat calc(min(var(--h,0ch),var(--cap)) + 1ch) 1.5em/2px
  calc(100% - 1.5em)}
pre.code .ind{position:absolute;width:0;height:0;overflow:hidden}
pre.out{font:12px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;color:#59636e;
  border-left:2px solid #d0d7de;margin:2px 0 4px 10px;padding:1px 8px;white-space:pre;overflow-x:auto}
.glances[data-mode=all] pre.code{font-size:11.5px;padding:3px 6px;--cap:10ch}
.glances[data-mode=all] pre.out{font-size:11px;margin-left:6px;padding:1px 6px}
.glances[data-mode=split]{grid-template-columns:repeat(2,minmax(0,1fr))}
.glances[data-mode=split] .gcol:not(.pick-a):not(.pick-b){display:none}
.glances[data-mode=split] .gcol.pick-b{order:1}
section.block{margin-top:30px}
.scorewrap{overflow-x:auto;border:1px solid var(--line);border-radius:8px}
table.scores{border-collapse:collapse;width:100%;min-width:860px;font-size:13px}
.scores th,.scores td{padding:6px 8px;border-bottom:1px solid var(--line);text-align:center;vertical-align:middle}
.scores th{font-weight:600;background:var(--panel);white-space:nowrap}
.scores th small,.scores td small{display:block;font-weight:400;font-size:11px;color:var(--faint)}
.scores .name{text-align:left;min-width:220px}
.scores .name a{text-decoration:none}
.scores .name .vname{font-size:13px}
.scores .name .standin{display:inline-block}
.scores td.s{font:600 13px ui-monospace,SFMono-Regular,Menlo,monospace}
.scores td.s1{color:#8a5a00}.scores td.s5{color:#1f6f43}
.scores td.disp{background:#f3f6fb}
.scores td.total{font-weight:700}
.scores td.pending{color:var(--faint);text-align:left}
.legend{font-size:12px;color:var(--muted);margin:4px 0 0}
.variant{margin-top:34px;padding-top:14px;border-top:3px solid var(--accent);max-width:1100px}
.variant h3{font-size:14px;margin:18px 0 6px;text-transform:uppercase;letter-spacing:.04em;color:var(--muted)}
.variant h4{font-size:13.5px;margin:10px 0 4px}
.face{margin:2px 0;color:var(--ink)}
.dim{color:var(--muted)}
.where{font-size:12.5px;color:var(--faint);margin:2px 0}
.problem{color:#9a3412;font-size:12.5px;margin:2px 0}
.session{border:1px solid var(--line);border-radius:8px;padding:6px 14px 12px;background:#fff}
.cell-head{margin:16px 0 2px;padding-top:10px;border-top:1px solid var(--line);color:var(--accent);font-size:14px}
.cell-head:first-child{margin-top:4px;border-top:0;padding-top:0}
.caption{margin:0 0 6px;color:var(--muted);font-size:12.5px}
.readme-title{font-size:12.5px;color:var(--faint);margin:14px 0 0}
.readme{font-size:13.5px}
.readme p{margin:4px 0 8px}
.readme ul,.readme ol{margin:4px 0 8px;padding-left:22px}
.readme pre,.md-raw{background:var(--code-bg);border:1px solid var(--line);border-radius:6px;padding:6px 10px;
  overflow-x:auto;font:12px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace}
.md-raw{white-space:pre-wrap}
.readme pre code{background:none;padding:0}
.readme table{border-collapse:collapse;display:block;overflow-x:auto;font-size:12.5px;margin:6px 0}
.readme th,.readme td{border:1px solid var(--line);padding:3px 8px;text-align:left;vertical-align:top}
.readme h1,.readme h2,.readme h3,.readme h4{font-size:14px;margin:12px 0 4px;text-transform:none;letter-spacing:0;
  color:var(--ink)}
details{margin-top:12px}
summary{cursor:pointer;color:var(--muted);font-size:13px}
table.evidence{border-collapse:collapse;font-size:12.5px;margin-top:6px}
.evidence td{border-bottom:1px solid var(--line);padding:4px 8px;vertical-align:top}
.evidence td.s{font:600 13px ui-monospace,monospace;text-align:center}
footer{margin-top:40px;font-size:12.5px;color:var(--muted);max-width:1100px}
footer code{font-size:11.5px}
@media (max-width:1279px){.glances[data-mode=all]{grid-template-columns:repeat(3,minmax(0,1fr))}}
@media (max-width:999px){.glances[data-mode=all]{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media (max-width:639px){
  .glances,.glances[data-mode=all],.glances[data-mode=split]{display:block}
  .gcol{display:block}
  .glances[data-mode=all] pre.code{font-size:12px}
  .glances[data-mode=all] pre.out{font-size:11.5px}
  h1{font-size:21px}
}
"""

SCRIPT = """
(() => {
  const grid = document.getElementById('glances');
  const pickA = document.getElementById('pick-a');
  const pickB = document.getElementById('pick-b');
  const buttons = [...document.querySelectorAll('.seg button')];
  const key = 'metrics-api-r2-glances';
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem(key) || '{}') || {}; } catch (error) { saved = {}; }
  const known = value => [...pickA.options].some(option => option.value === value);
  if (known(saved.a)) pickA.value = saved.a;
  if (known(saved.b)) pickB.value = saved.b;
  function apply(mode) {
    grid.dataset.mode = mode;
    buttons.forEach(button => button.setAttribute('aria-pressed', String(button.dataset.mode === mode)));
    grid.querySelectorAll('.gcol').forEach(column => {
      column.classList.toggle('pick-a', column.dataset.slot === pickA.value);
      column.classList.toggle('pick-b', column.dataset.slot === pickB.value && pickB.value !== pickA.value);
    });
    try { localStorage.setItem(key, JSON.stringify({mode, a: pickA.value, b: pickB.value})); } catch (error) {}
  }
  buttons.forEach(button => button.addEventListener('click', () => apply(button.dataset.mode)));
  [pickA, pickB].forEach(select => select.addEventListener('change', () => apply('split')));
  apply(saved.mode === 'split' ? 'split' : 'all');
})();
"""


def page_html(records: list[dict], contract: dict, stand_in: bool) -> tuple[str, dict]:
    criteria = contract["criteria"]
    glances = {record["id"]: glance_of(record["cells"]) for record in records}
    groups = {slot: ROUND1_GALLERY.concept_groups(cell) if cell else None for slot, cell in glances.items()}
    counts = {len(found) for found in groups.values() if found}
    aligned = len(counts) == 1
    body_rows = counts.pop() if aligned else 1
    scans = {slot: coined_words_on(cell["statements"], contract["coined"]) if cell else []
             for slot, cell in glances.items()}
    columns = "".join(glance_column(record, groups[record["id"]], body_rows, aligned, scans[record["id"]])
                      for record in records)
    options = "".join(f'<option value="{record["id"]}">{record["id"]} · '
                      f'{esc(record["standIn"]["name"] if record["standIn"] else plain(record["name"]))}</option>'
                      for record in records)
    options_b = options.replace(f'value="{SLOTS[1]}"', f'value="{SLOTS[1]}" selected', 1)
    built = [record["id"] for record in records if status_of(record) in ("built", "stand-in")]
    waiting = [record["id"] for record in records if status_of(record) not in ("built", "stand-in")]
    status_line = (f"{len(built)} of {len(records)} slots filled by round 1's sessions" if stand_in
                   else f"{len(built)} of {len(records)} built")
    if waiting:
        status_line += " · waiting on " + ", ".join(f"{slot} ({status_of(r)})" for slot in waiting
                                                    for r in records if r["id"] == slot)
    banner = ""
    if stand_in:
        banner = ('<p class="banner"><b>Stand-in, round 1.</b> Round 2\'s variants are still being built, so the five '
                  'slots hold round 1\'s v1-v5 sessions to show the layout. Their glances do not follow round 2\'s '
                  'eight steps and do not run as shown (round 1: 0 of 10 blocks). Rebuild without '
                  '<code class="nb">--stand-in-round1</code> when the lanes land.</p>')
    alignment = ("Each story step sits on the same row across the columns." if aligned else
                 "The glances have different numbers of steps, so their rows are not lined up.")
    html_page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Metrics APIs round 2</title>
<style>{STYLE}{ROUND1_GALLERY.PYGMENTS.get_style_defs("pre.code")}</style></head>
<body><main class="page">
<div class="prose">
<div class="kicker">robot_models · metrics API · babble round 2 · 2026-11-18</div>
<h1>Five metrics APIs, legible at a glance</h1>
<p class="lede">Zach, on robot_metrics 0.1.0: <q>I am not seeing what is going on at a glance. I don't know what you
mean by fold etc.</q> So this round leads with the glances: the baseline and four new directions, each telling the
same story on the same real recording, each highlighted with one statement and then its output.</p>
{banner}<p class="status">{esc(status_line)}</p>
</div>
<section id="glance-section">
<div class="toolbar"><h2>The glances</h2>
<div class="seg" role="group" aria-label="layout"><button type="button" data-mode="all" aria-pressed="true">All five
</button><button type="button" data-mode="split" aria-pressed="false">Split</button></div>
<span class="picks"><label>left <select id="pick-a" aria-label="left glance">{options}</select></label>
<label>right <select id="pick-b" aria-label="right glance">{options_b}</select></label></span></div>
<p class="story">The contract's story, in order: load round 8's real runs · the pendulum arm's metrics with units ·
one recording offline (<code>pendulum_runs("real")[32]</code>: CAN 11, rung 0.9) · the same recording live, 7 rows at
a time · live equals offline · many runs into a table by station and rung · Zach's 1 mm tip-stop gate, outside the
metric · its verdict, median and worst stop. {alignment} Split compares any two at reading width.</p>
<div class="glances" id="glances" data-mode="all" style="--rows:{body_rows + 2}">{columns}</div>
</section>
<section class="block" id="scoreboard"><h2>Scoreboard</h2>
{scoreboard_html(records, criteria, stand_in)}</section>
{"".join(variant_section(record, criteria) for record in records)}
<footer><p>Built by <code>docs/proposals/metrics-api-2026-11-18/build_gallery.py</code> from each variant's
session.json, scores.json and README.md; the weights and the coined words come from the round's frozen README. The
code shown is the code that ran; check_standalone.py reruns every block of the real build in a fresh
<code>python -I</code>. Rebuild: <code>{esc(REBUILD)}</code></p></footer>
</main><script>{SCRIPT}</script></body></html>
"""
    spec = {
        "schemaVersion": 1, "round": HERE.name, "standIn": stand_in,
        "title": "Five metrics APIs, legible at a glance",
        "criteria": criteria, "coinedWords": contract["coined"], "glanceRowsAligned": aligned,
        "variants": [{
            "id": record["id"], "folder": record["folder"], "name": plain(record["name"]),
            "status": status_of(record),
            "previewLabel": "API at a glance",
            "preview": ('<div class="glance">' + "".join(f'<div class="group">{flow_html(group)}</div>'
                                                          for group in groups[record["id"]]) + "</div>")
            if groups[record["id"]] else "",
            "media": [{"label": "The session that ran", "html": session_html(session_cells(record["cells"]))}]
            if session_cells(record["cells"]) else [],
            "scores": record["scores"], "lgUnexplained": record["lgUnexplained"], "total": record["selfTotal"],
            "dispatcherLg": record["dispatcher"], "coinedWordScan": scans[record["id"]],
            "glanceLinesOfCode": lines_of_code(glances[record["id"]]) if glances[record["id"]] else None,
        } for record in records],
    }
    return html_page, spec


def write_atomic(path: pathlib.Path, text: str) -> None:
    """Write next to the target, then rename: a lane reading gallery-spec.json mid-build never sees half a file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.",
                                     suffix=".tmp", delete=False) as handle:
        handle.write(text)
    os.chmod(handle.name, 0o664)
    os.replace(handle.name, path)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--stand-in-round1", action="store_true",
                        help="fill the five slots with round 1's v1-v5 sessions, labelled (never writes the spec)")
    parser.add_argument("--output", default=str(OUT), help=f"the page (default {OUT})")
    args = parser.parse_args(argv)
    contract = read_contract()
    dispatcher, problem = read_json(DISPATCHER_LG)
    if problem:
        print(f"warning: {problem}")
    dispatcher = dispatcher if isinstance(dispatcher, dict) else {}
    records = [load_slot(slot, contract, args.stand_in_round1, dispatcher) for slot in SLOTS]
    page, spec = page_html(records, contract, args.stand_in_round1)
    output = pathlib.Path(args.output)
    write_atomic(output, page)
    if args.stand_in_round1:
        print(f"stand-in: {SPEC.name} left as it was (the standalone check only reads round 2's own variants)")
    else:
        write_atomic(SPEC, json.dumps(spec, indent=1, ensure_ascii=False))
        print(f"wrote {SPEC}")
    for record, variant in zip(records, spec["variants"]):
        line = f"{record['id']:<9} {variant['status']:<14}"
        if variant["glanceLinesOfCode"] is not None:
            line += f" glance {variant['glanceLinesOfCode']} lines of code"
        if record["scores"]:
            line += f" · total {number(variant['total'])}" if variant["total"] is not None else ""
        print(line)
        for problem_text in record["problems"]:
            print(f"          problem: {problem_text}")
        too_long = long_lines(record["cells"] or [])
        if too_long:
            print(f"          {len(too_long)} code line(s) over {MAX_LINE} characters, e.g. {too_long[0][:60]!r}...")
        cells = session_cells(record["cells"])
        if cells and not re.search(r"setup", cells[0].get("label", ""), re.I) and not record["standIn"]:
            print(f"          first session cell is {cells[0].get('label')!r}, not a setup cell")
    if MarkdownIt is None:
        print("note: markdown-it-py not importable, README text drawn raw (run with --with markdown-it-py)")
    size = output.stat().st_size
    print(f"{output} {size / 1e6:.2f} MB" + (" (over the 1.2 MB preview ceiling)" if size > MAX_PAGE_BYTES else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
