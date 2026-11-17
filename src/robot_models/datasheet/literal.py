"""Is a `sources/<name>.py` literal-only, so a reader can parse it without running it?

WHY (PEP 0003 §6): robotparts' catalogue build reads `part.py` files with an AST walk and never runs them (they arrive
in pull requests). A source file kept to literals can be read the same way later, and stays a transcription: a
conversion or a sum is a derivation, and belongs in the part file or the type, never in what a source states.

Allowed at the top level: a docstring; `from robot_models.datasheet import ...` (and `from __future__`); and
`name = Source(...)` / `name = Stated(...)` / `extra = {"key": Stated(...), ...}`, whose arguments are literals
(numbers, text, True/False/None, tuples, lists and dicts of literals, a leading minus).
"""

from __future__ import annotations

import ast
import pathlib

CONSTRUCTORS = ("Source", "Stated")


def problems(path: str | pathlib.Path) -> list[str]:
    """What keeps the file from being read without running it; empty when it is literal-only."""
    path = pathlib.Path(path)
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: list[str] = []
    for index, node in enumerate(tree.body):
        if index == 0 and isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            continue
        if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module in ("robot_models.datasheet", "__future__"):
            continue
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            target = node.targets[0].id
            if target == "extra" and isinstance(node.value, ast.Dict):
                for key, value in zip(node.value.keys, node.value.values):
                    if not (isinstance(key, ast.Constant) and isinstance(key.value, str)):
                        found.append(f"line {node.lineno}: extra's keys are plain names in quotes")
                    found.extend(_call(value))
                continue
            found.extend(_call(node.value))
            continue
        found.append(f"line {node.lineno}: only a docstring, the import and `name = Stated(...)` lines belong here "
                     f"({type(node).__name__})")
    return found


def _call(node: ast.AST) -> list[str]:
    if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in CONSTRUCTORS):
        return [f"line {node.lineno}: a value is Stated(...) (or the header Source(...)), not "
                f"{ast.unparse(node)[:60]}"]
    out = []
    for argument in [*node.args, *(keyword.value for keyword in node.keywords)]:
        if not _literal(argument):
            out.append(f"line {argument.lineno}: {ast.unparse(argument)[:60]} is not a literal (no arithmetic, "
                       f"names or calls in a source file)")
    if any(keyword.arg is None for keyword in node.keywords) or any(isinstance(a, ast.Starred) for a in node.args):
        out.append(f"line {node.lineno}: no * or ** in a source file")
    return out


def _literal(node: ast.AST) -> bool:
    if isinstance(node, ast.Constant):
        return True
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub) and isinstance(node.operand, ast.Constant):
        return isinstance(node.operand.value, (int, float)) and not isinstance(node.operand.value, bool)
    if isinstance(node, (ast.Tuple, ast.List)):
        return all(_literal(element) for element in node.elts)
    if isinstance(node, ast.Dict):
        return all(key is not None and _literal(key) for key in node.keys) and all(_literal(v) for v in node.values)
    return False
