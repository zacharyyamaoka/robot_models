"""Every code block a babble round's gallery displays must run exactly as shown, as its own script.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/check_standalone.py [ROUND_FOLDER] [--gallery HTML] [--path DIR ...] [--python CMD] [--only ID ...] [--jobs 2]

What it reads: the round's displayed blocks, from the built gallery (``--gallery``: the ``babble-data`` JSON the page
embeds) or else from ``ROUND_FOLDER/gallery-spec.json`` (what the round's build_gallery.py feeds the page).
ROUND_FOLDER defaults to this file's folder, so round 1 runs with no argument and round 2 reuses the file unchanged by
naming its own folder.

A displayed block is a variant's ``preview`` (the API at a glance) or any ``media`` item whose html holds
``<pre class="code">``. Inside one, each ``<div class="stmt">`` is code (``pre.code``) and the output drawn under it
(``pre.out``): the shape _session.py records and build_gallery.py draws.

What it asserts, per block, in a FRESH subprocess (``CMD -I``: no PYTHONPATH, no user site, no script folder):
- sys.path gets only what a reader would add: the variant's folder, the round folder, then each ``--path``;
- the block's code, extracted exactly as displayed, runs top to bottom in an empty ``__main__`` namespace and exits 0
  (a name the page never imports or defines is a NameError here);
- each statement's output equals the output the page draws under it: a bare expression echoes as the REPL does
  (``pprint.pformat(value, width=100, sort_dicts=False, compact=True)``), print() text as printed, and a statement
  the page shows no output for prints nothing;
- the glance also runs plainly, ``runpy.run_path(..., run_name="__main__")``, and exits 0.
A media item's statements run as one notebook (its cells in page order): a later cell may use a name an earlier cell
on the same page defined, never one from hidden setup. Outputs must be deterministic (no timings echoed).

The variant's folder is ``variant["folder"]`` when the spec names one, else the round folder's one directory named
``<id>`` or ``<id>_*`` (round 1: v1 -> v1_robot_metrics).

WHY (Zach, /rate bad 2026-11-18, on round 1's API at a glance): "you use this pendulum_arm but I have no idea what that
is? where did you import it from etc.. You need to make sure these examples are like valid python code unless it's
incredibly obvious what you are doing". Round 1's harness ran every cell in scenario.py's own globals, so a glance
could use a name from hidden setup and still "run"; only a fresh process with an empty namespace can fail on that.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import html.parser
import json
import pathlib
import re
import shlex
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
PYTHON = "/home/bam/robot_models/.venv/bin/python"
TIMEOUT_S = 900
TRUNCATED = re.compile(r"\n\.\.\. \(full output in [^)]*\)$")   # build_gallery.py's OUTPUT_LIMIT marker

# Runs inside the fresh interpreter. Its own imports live in this frame, never in the script's namespace.
RUNNER = r'''
import ast, builtins, contextlib, io, json, pprint, sys, traceback
script, results_path, *paths = sys.argv[1:]
sys.path[:0] = paths
source = open(script, encoding="utf-8").read()
namespace = {"__name__": "__main__", "__file__": script, "__builtins__": builtins}
results, error = [], None
for node in ast.parse(source, script).body:
    first = min([node.lineno, *(d.lineno for d in getattr(node, "decorator_list", []))])
    buffer = io.StringIO()
    try:
        with contextlib.redirect_stdout(buffer):
            if isinstance(node, ast.Expr):
                value = eval(compile(ast.Expression(node.value), script, "eval"), namespace)
                if value is not None:
                    print(pprint.pformat(value, width=100, sort_dicts=False, compact=True))
            else:
                exec(compile(ast.Module([node], type_ignores=[]), script, "exec"), namespace)
    except BaseException as raised:
        traceback.print_exc()
        error = {"line": first, "error": f"{type(raised).__name__}: {raised}"}
    results.append({"line": first, "end": node.end_lineno, "output": buffer.getvalue().rstrip()})
    if error:
        break
json.dump({"statements": results, "error": error}, open(results_path, "w", encoding="utf-8"))
sys.exit(1 if error else 0)
'''

PLAIN = "import runpy, sys; sys.path[:0] = sys.argv[2:]; runpy.run_path(sys.argv[1], run_name='__main__')"


class Blocks(html.parser.HTMLParser):
    """The statements of one displayed html fragment: [{"code": str, "output": str}], in page order."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.statements: list[dict[str, str]] = []
        self.capturing: str | None = None
        self.text: list[str] = []
        self.in_stmt = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        classes = (dict(attrs).get("class") or "").split()
        if tag == "div" and "stmt" in classes:
            self.statements.append({"code": "", "output": ""})
            self.in_stmt = True
        elif tag == "pre" and ("code" in classes or "out" in classes):
            if "code" in classes and (not self.in_stmt or self.statements[-1]["code"]):
                self.statements.append({"code": "", "output": ""})   # a code block outside a stmt div stands alone
            self.capturing, self.text = ("code" if "code" in classes else "output"), []

    def handle_endtag(self, tag: str) -> None:
        if tag == "pre" and self.capturing:
            if self.statements:
                self.statements[-1][self.capturing] = "".join(self.text)
            self.capturing = None

    def handle_data(self, data: str) -> None:
        if self.capturing:
            self.text.append(data)


def statements_of(fragment: str) -> list[dict[str, str]]:
    parser = Blocks()
    parser.feed(fragment)
    parser.close()
    return [statement for statement in parser.statements if statement["code"].strip()]


def load_spec(round_folder: pathlib.Path, gallery: pathlib.Path | None) -> dict:
    if gallery is None:
        return json.loads((round_folder / "gallery-spec.json").read_text(encoding="utf-8"))
    page = gallery.read_text(encoding="utf-8")
    match = re.search(r'<script type="application/json" id="babble-data">(.*?)</script>', page, re.S)
    if not match:
        raise SystemExit(f"{gallery}: no babble-data JSON in the page")
    return json.loads(match.group(1))


def variant_folder(round_folder: pathlib.Path, variant: dict) -> pathlib.Path | None:
    if variant.get("folder"):
        return (round_folder / variant["folder"]).resolve()
    matches = [path for path in round_folder.iterdir()
               if path.is_dir() and (path.name == variant["id"] or path.name.startswith(variant["id"] + "_"))]
    return matches[0] if len(matches) == 1 else None


def blocks_of(spec: dict, only: set[str]) -> list[dict]:
    blocks = []
    for variant in spec["variants"]:
        if only and variant["id"] not in only:
            continue
        exhibits = [(variant.get("previewLabel") or "preview", variant.get("preview") or "", True)]
        exhibits += [(media.get("label", "media"), media.get("html") or "", False) for media in variant.get("media", [])]
        for label, fragment, is_glance in exhibits:
            statements = statements_of(fragment)
            if statements:
                blocks.append({"variant": variant, "label": label, "statements": statements, "glance": is_glance})
    return blocks


def script_of(statements: list[dict[str, str]]) -> tuple[str, list[tuple[int, int]]]:
    """The displayed code as one file, and each displayed statement's (first, last) line in it."""

    lines: list[str] = []
    spans = []
    for statement in statements:
        if lines:
            lines.append("")
        code = statement["code"].rstrip("\n").splitlines()
        spans.append((len(lines) + 1, len(lines) + len(code)))
        lines += code
    return "\n".join(lines) + "\n", spans


def output_matches(shown: str, ran: str) -> bool:
    shown = shown.rstrip()
    truncated = TRUNCATED.search(shown)
    if truncated:
        return ran.startswith(shown[:truncated.start()])
    return ran.rstrip() == shown


def first_code_line(code: str) -> str:
    lines = [line for line in code.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    return (lines or [code.strip()])[0].strip()[:100]


def check_block(block: dict, round_folder: pathlib.Path, extra: list[str], python: list[str],
                workdir: pathlib.Path) -> tuple[bool, list[str]]:
    variant, statements = block["variant"], block["statements"]
    folder = variant_folder(round_folder, variant)
    if folder is None:
        return False, [f"no folder for variant {variant['id']!r} in {round_folder}"]
    paths = [str(folder), str(round_folder), *extra]
    slug = re.sub(r"[^a-z0-9]+", "-", block["label"].lower()).strip("-")
    script = workdir / f"{variant['id']}-{slug}.py"
    results_path = script.with_suffix(".json")
    source, spans = script_of(statements)
    script.write_text(source, encoding="utf-8")
    try:
        ran = subprocess.run([*python, "-I", "-c", RUNNER, str(script), str(results_path), *paths],
                             capture_output=True, text=True, timeout=TIMEOUT_S, cwd=workdir)
    except subprocess.TimeoutExpired:
        return False, [f"timed out after {TIMEOUT_S} s"]
    try:
        result = json.loads(results_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        tail = (ran.stderr.strip().splitlines() or ["no output"])[-1]
        return False, [f"exit {ran.returncode} before any statement ran: {tail}"]

    def shown_index(line: int) -> int:
        return next(index for index, (first, last) in enumerate(spans) if first <= line <= last)

    problems = []
    error = result["error"]
    failed_at = shown_index(error["line"]) if error else None
    if error:
        problems += [error["error"], "at: " + first_code_line(statements[failed_at]["code"])]
    ran_by_shown: dict[int, list[dict]] = {}
    for statement in result["statements"]:
        ran_by_shown.setdefault(shown_index(statement["line"]), []).append(statement)
    for index, shown in enumerate(statements):
        if index == failed_at:
            break
        ran_here = ran_by_shown.get(index, [])
        if not ran_here:
            problems.append(f"no statement ran for: {first_code_line(shown['code'])}")
            break
        for early in ran_here[:-1]:
            if early["output"]:
                problems.append(f"line {early['line']} printed output the page does not show: {early['output'][:80]!r}")
        if not output_matches(shown["output"], ran_here[-1]["output"]):
            problems.append(f"output differs under: {first_code_line(shown['code'])}\n"
                            f"            shown {shown['output'][:160]!r}\n"
                            f"            ran   {ran_here[-1]['output'][:160]!r}")
    if ran.returncode != 0 and not error:
        problems.append(f"exit {ran.returncode}")
    if block["glance"] and not problems:
        plain = subprocess.run([*python, "-I", "-c", PLAIN, str(script), *paths], capture_output=True, text=True,
                               timeout=TIMEOUT_S, cwd=workdir)
        if plain.returncode != 0:
            problems.append("as a plain script: " + (plain.stderr.strip().splitlines() or ["exit != 0"])[-1])
    return not problems, problems or [f"{len(result['statements'])} statements ran, every output as shown"]


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("round_folder", nargs="?", default=str(HERE))
    parser.add_argument("--gallery", help="read the blocks from this built gallery page instead of gallery-spec.json")
    parser.add_argument("--path", action="append", default=[], help="one more folder a reader puts on sys.path")
    parser.add_argument("--python", default=PYTHON, help="the interpreter command (default: robot_models' venv)")
    parser.add_argument("--only", action="append", default=[], help="variant id(s) to check")
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args(argv)
    round_folder = pathlib.Path(args.round_folder).resolve()
    gallery = pathlib.Path(args.gallery).resolve() if args.gallery else None
    blocks = blocks_of(load_spec(round_folder, gallery), set(args.only))
    if not blocks:
        print(f"no displayed code blocks in {round_folder}")
        return 1
    python = shlex.split(args.python)
    extra = [str(pathlib.Path(path).resolve()) for path in args.path]
    print(f"round {round_folder.name} · {len(blocks)} displayed blocks · {' '.join(python)} -I · "
          f"source {gallery.name if gallery else 'gallery-spec.json'}")
    with tempfile.TemporaryDirectory(prefix="check-standalone-") as temp:
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            verdicts = list(pool.map(lambda block: check_block(block, round_folder, extra, python,
                                                               pathlib.Path(temp)), blocks))
    passed = 0
    for block, (ok, lines) in zip(blocks, verdicts):
        passed += ok
        print(f"{'PASS' if ok else 'FAIL'}  {block['variant']['id']:<6} {block['label']}")
        for line in lines[:6]:
            print(f"      {'·' if ok else '-'} {line}")
    print(f"standalone: {passed} of {len(blocks)} blocks PASS")
    return 0 if passed == len(blocks) else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
