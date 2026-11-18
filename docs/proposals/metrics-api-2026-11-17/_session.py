"""The transcript harness every scenario uses: run code cells in one namespace, one statement at a time.

    session = Session(globals_dict)
    session.cell("a part's metrics", "what to notice", "arm = pendulum_arm(0.339)\\narm.metrics[0]")
    session.write(folder)       # scenario.out.txt (the REPL transcript) and session.json (cells for the gallery)

Each top-level statement runs on its own and keeps its own output, the way a REPL or a notebook shows it: a bare
expression echoes its value (pretty-printed, as IPython does), a compound statement (``for`` / ``with`` / ``try`` /
``def``) is one unit. Comment lines above a statement belong to it; a blank line before it marks a new concept
(``gap``), which the gallery draws as space. The code shown is exactly the code that ran.

WHY per statement (Zach, /rate bad 2026-11-18): "make it more like line by line so I can see the output of each line"
- one combined output under a block of seven statements hid which line printed what. Harness only: it holds no
metric, so it is shared without making the five variants alike.
"""

from __future__ import annotations

import ast
import contextlib
import io
import json
import pathlib
import pprint
import time
from typing import Any

ECHO_WIDTH = 100     # WHY 100: a nested config (an alias's metric) stays on one line; code is <= 90


def echo(value: Any) -> str:
    """What the REPL prints for a bare expression, wrapped at ECHO_WIDTH like IPython's pretty printer (a short list
    packs several items per line, a long dataclass or dict puts one field per line)."""

    return pprint.pformat(value, width=ECHO_WIDTH, sort_dicts=False, compact=True)


def split_statements(code: str) -> list[dict[str, Any]]:
    """The cell's top-level statements with their own source lines (leading comments included) and a gap flag."""

    lines = code.splitlines()
    tree = ast.parse(code)
    statements, previous_end = [], 0
    for node in tree.body:
        first = min([node.lineno, *(d.lineno for d in getattr(node, "decorator_list", []))]) - 1
        if first < previous_end:
            raise ValueError(f"line {node.lineno}: one statement per line (no ';'), so each keeps its own output")
        leading = lines[previous_end:first]
        gap = bool(statements) and any(not line.strip() for line in leading)
        source = [line for line in leading if line.strip()] + lines[first:node.end_lineno]
        statements.append({"node": node, "code": "\n".join(source), "gap": gap})
        previous_end = node.end_lineno
    trailing = [line for line in lines[previous_end:] if line.strip()]
    if trailing and statements:
        statements[-1]["code"] += "\n" + "\n".join(trailing)
    return statements


class Session:
    def __init__(self, namespace: dict[str, Any], *, echo: bool = True) -> None:
        self.namespace = namespace
        self.cells: list[dict[str, Any]] = []
        self.echo = echo

    def cell(self, label: str, caption: str, code: str, *, kind: str = "session") -> None:
        """Run ``code`` statement by statement. ``kind="glance"`` marks the short API-at-a-glance exhibit."""

        code = code.strip("\n")
        filename = f"<cell {label}>"
        recorded, started = [], time.perf_counter()
        for statement in split_statements(code):
            node = statement["node"]
            buffer = io.StringIO()
            with contextlib.redirect_stdout(buffer):
                if isinstance(node, ast.Expr):
                    value = eval(compile(ast.Expression(node.value), filename, "eval"), self.namespace)
                    if value is not None:
                        print(echo(value))
                else:
                    exec(compile(ast.Module([node], type_ignores=[]), filename, "exec"), self.namespace)
            recorded.append({"code": statement["code"], "output": buffer.getvalue().rstrip(),
                             "gap": statement["gap"]})
        seconds = time.perf_counter() - started
        cell = {"label": label, "caption": caption, "kind": kind, "code": code,
                "output": "\n".join(s["output"] for s in recorded if s["output"]),
                "seconds": f"{seconds:.2f}", "statements": recorded}
        self.cells.append(cell)
        if self.echo:
            print(f"\n# {len(self.cells)} · {label}")
            print("\n".join(transcript(cell)))

    def write(self, folder: pathlib.Path) -> None:
        lines = []
        for number, cell in enumerate(self.cells, start=1):
            lines.append(f"# {number} · {cell['label']}   ({cell['seconds']} s)")
            lines += transcript(cell)
            lines.append("")
        (folder / "scenario.out.txt").write_text("\n".join(lines), encoding="utf-8")
        (folder / "session.json").write_text(json.dumps(self.cells, indent=1, ensure_ascii=False), encoding="utf-8")


def transcript(cell: dict[str, Any]) -> list[str]:
    """A cell as the REPL shows it: ``>>>`` on a statement's first line, ``...`` on its continuation lines."""

    lines = []
    for statement in cell["statements"]:
        if statement["gap"]:
            lines.append("")
        started = False
        for line in statement["code"].splitlines():
            lines.append(("... " if started else ">>> ") + line)
            started = started or not line.lstrip().startswith("#")
        if statement["output"]:
            lines.append(statement["output"])
    return lines
