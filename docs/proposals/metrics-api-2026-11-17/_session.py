"""The transcript harness every scenario uses: run code cells in one namespace, keep what each printed.

    session = Session(globals_dict)
    session.cell("a part's metrics", "what to notice", "print(IntegratedServo.metrics)")
    session.write(folder)       # scenario.out.txt (the REPL transcript) and session.json (cells for the gallery)

A cell's last expression is echoed like the REPL does. The code shown is exactly the code that ran. Harness only: it
holds no metric, so it is shared without making the five variants alike.
"""

from __future__ import annotations

import ast
import contextlib
import io
import json
import pathlib
import time
from typing import Any


class Session:
    def __init__(self, namespace: dict[str, Any], *, echo: bool = True) -> None:
        self.namespace = namespace
        self.cells: list[dict[str, str]] = []
        self.echo = echo

    def cell(self, label: str, caption: str, code: str) -> None:
        code = code.strip("\n")
        tree = ast.parse(code)
        last = tree.body.pop() if tree.body and isinstance(tree.body[-1], ast.Expr) else None
        buffer = io.StringIO()
        started = time.perf_counter()
        with contextlib.redirect_stdout(buffer):
            exec(compile(tree, f"<cell {label}>", "exec"), self.namespace)
            if last is not None:
                value = eval(compile(ast.Expression(last.value), f"<cell {label}>", "eval"), self.namespace)
                if value is not None:
                    print(repr(value))
        seconds = time.perf_counter() - started
        output = buffer.getvalue().rstrip()
        self.cells.append({"label": label, "caption": caption, "code": code, "output": output,
                           "seconds": f"{seconds:.2f}"})
        if self.echo:
            print(f"\n# {len(self.cells)} · {label}")
            for line in code.splitlines():
                print(("... " if line.startswith((" ", ")", "]", "}")) else ">>> ") + line)
            if output:
                print(output)

    def write(self, folder: pathlib.Path) -> None:
        lines = []
        for number, cell in enumerate(self.cells, start=1):
            lines.append(f"# {number} · {cell['label']}   ({cell['seconds']} s)")
            lines += [("... " if line.startswith((" ", ")", "]", "}")) else ">>> ") + line for line in cell["code"].splitlines()]
            if cell["output"]:
                lines.append(cell["output"])
            lines.append("")
        (folder / "scenario.out.txt").write_text("\n".join(lines), encoding="utf-8")
        (folder / "session.json").write_text(json.dumps(self.cells, indent=1, ensure_ascii=False), encoding="utf-8")
