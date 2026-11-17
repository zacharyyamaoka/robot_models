"""The browsable docs: each type folder's README.md and docs/TYPES.md, generated from the ladders, must be current.

Regenerate after any change to a ladder, a rung's references or robot_models.references:

    python tools/make_type_docs.py

WHY a test: the v1 browsing choice of 2026-11-16 is "the folder tree in Obsidian and VS Code, with a README per rung".
A rung with code keeps the README in its own folder; every other rung's README is its anchored row in the type
folder's generated README (the tool's docstring says why). A README edited by hand, or left behind when a ladder
changes, is wrong the moment someone reads it; CI goes red instead. `--check` is itself held red on a stale and a missing file, so a check that passes everything cannot pass
here; and every link the generator writes must land on a file or an anchor that exists.
"""

from __future__ import annotations

import importlib.util
import os
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote

from robot_models.ladder import discover

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "make_type_docs.py"
INDEX = "docs/TYPES.md"
SAFE_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*")


def run_tool(*args: str) -> subprocess.CompletedProcess:
    # WHY no bytecode: the tool imports every type's ladder, and robotparts vendors the rung folders byte for byte.
    return subprocess.run([sys.executable, str(TOOL), *args], capture_output=True, text=True, check=False, cwd=ROOT,
                          env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})


def generated_paths() -> list[str]:
    return sorted([f"src/robot_models/{item.folder}/README.md" for item in discover().values()] + [INDEX])


def test_the_generated_docs_are_current():
    result = run_tool("--check")
    assert result.returncode == 0, f"run python tools/make_type_docs.py\n{result.stdout}{result.stderr}"


def test_the_check_names_a_stale_and_a_missing_file_and_writes_nothing(tmp_path):
    assert run_tool("--out", str(tmp_path)).returncode == 0
    written = sorted(path.relative_to(tmp_path).as_posix() for path in tmp_path.rglob("*") if path.is_file())
    assert written == generated_paths(), "one README per type folder, and the index"
    assert run_tool("--check", "--out", str(tmp_path)).returncode == 0, "two runs must write the same text"

    stale = tmp_path / "src/robot_models/sensors/encoder/README.md"
    stale.write_text(stale.read_text(encoding="utf-8") + "\nan edit by hand\n", encoding="utf-8")
    edited = stale.read_bytes()
    (tmp_path / INDEX).unlink()
    result = run_tool("--check", "--out", str(tmp_path))
    assert result.returncode == 1
    named = [line for line in result.stdout.splitlines() if line.startswith(("stale: ", "missing: "))]
    assert named == ["stale: src/robot_models/sensors/encoder/README.md", f"missing: {INDEX}"], result.stdout
    assert stale.read_bytes() == edited and not (tmp_path / INDEX).exists(), "--check never writes"


def _anchors(markdown: str) -> set[str]:
    """The ids a link can jump to: explicit <a id="..."> anchors and GitHub's slugs of the headings."""
    ids = set(re.findall(r'<a id="([^"]+)"></a>', markdown))
    for heading in re.findall(r"^#{1,6} (.+)$", markdown, re.MULTILINE):
        ids.add(re.sub(r"[^\w\- ]", "", heading.strip().lower()).replace(" ", "-"))
    return ids


def test_every_link_lands_on_a_file_or_an_anchor_that_exists():
    for relative in generated_paths():
        path = ROOT / relative
        text = path.read_text(encoding="utf-8")
        prose = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
        prose = re.sub(r"(`+).*?\1", "", prose)
        for target in re.findall(r"\]\(([^)\s]+)\)", prose):
            if target.startswith(("https://", "http://")):
                continue
            file_part, _, fragment = target.partition("#")
            landing = (path.parent / unquote(file_part)).resolve() if file_part else path
            assert landing.exists(), f"{relative}: {target} lands nowhere"
            if fragment:
                assert fragment in _anchors(landing.read_text(encoding="utf-8")), f"{relative}: no anchor {target}"


def test_each_readme_draws_one_mermaid_tree_with_safe_ids_and_quoted_labels():
    """Mermaid reads a dot in a node id as syntax, and an unquoted label breaks on punctuation."""
    for relative in generated_paths():
        if relative == INDEX:
            continue
        blocks = re.findall(r"```mermaid\n(.*?)```", (ROOT / relative).read_text(encoding="utf-8"), re.DOTALL)
        assert len(blocks) == 1, relative
        defined: set[str] = set()
        for line in blocks[0].splitlines()[1:]:
            line = line.strip()
            if not line or line == "end" or line.startswith(("classDef ", "class ", "subgraph ")):
                continue
            edge = re.fullmatch(r"(\S+) (?:-->|-\.->\|[^|]+\||~~~) (\S+)", line)
            if edge:
                assert {edge.group(1), edge.group(2)} <= defined, f"{relative}: an edge to an undrawn node: {line}"
                continue
            box = re.fullmatch(r'(\S+?)(\["|\(\["|\{\{")(.*)("\]|"\]\)|"\}\})', line)
            assert box, f"{relative}: not a node with a quoted label: {line}"
            assert SAFE_ID.fullmatch(box.group(1)), f"{relative}: unsafe node id {box.group(1)}"
            assert '"' not in box.group(3), f"{relative}: a raw quote inside a label: {line}"
            defined.add(box.group(1))
        assert defined, relative


def test_the_docs_name_no_maker_and_no_path():
    """robot_models' src is type data only (PEP 0004 decision 4), and the READMEs ship inside it."""
    # WHY the ruler's own list, loaded by path: one list of forbidden tokens, read whatever pytest's import mode is
    # (registered first, because its dataclasses look their module up while they are built).
    spec = importlib.util.spec_from_file_location("_type_docs_ruler", ROOT / "tests" / "test_ladders_acceptance.py")
    ruler = sys.modules.setdefault(spec.name, importlib.util.module_from_spec(spec))
    spec.loader.exec_module(ruler)
    for relative in generated_paths():
        text = (ROOT / relative).read_text(encoding="utf-8")
        found = [pattern for pattern in ruler.FORBIDDEN if re.search(pattern, text, re.IGNORECASE)]
        assert found == [], f"{relative}: {found}"
