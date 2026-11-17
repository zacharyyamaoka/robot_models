"""Is a `sources/<name>.py` literal-only, so a reader can parse it without running it? And `read`, that reader.

WHY (PEP 0003 §6): robotparts' catalogue build reads `part.py` files with an AST walk and never runs them (they arrive
in pull requests). A source file kept to literals can be read the same way, and stays a transcription: a conversion
or a sum is a derivation, and belongs in the part file or the type, never in what a source states.

Allowed at the top level: a docstring; `from robot_models... import ...` (and `from __future__`); `source = Source(...)`;
and `values = <Type>(field=Stated(...), ..., extra={"key": Stated(...), ...})`, whose arguments are literals (numbers,
text, True/False/None, tuples, lists and dicts of literals, a leading minus). The type's name is read as a name: the
static reader does not need to import it to know which field each keyword fills.

    read("sources/steadywin_page.py")["values"]["peak_torque"]["value"]   # {"GDZ34": 3.82, "GDS34": 3.47}, never run
"""

from __future__ import annotations

import ast
import pathlib
from typing import Any

STATED = "Stated"
SOURCE = "Source"
# The positional order of Stated(value, unit, as_written) and Source(title)
STATED_POSITIONAL = ("value", "unit", "as_written")
SOURCE_POSITIONAL = ("title",)


def problems(path: str | pathlib.Path) -> list[str]:
    """What keeps the file from being read without running it; empty when it is literal-only."""
    path = pathlib.Path(path)
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: list[str] = []
    # the names imported from robot_models: a type called for `values` must be one of them, so a reader knows it is
    # a type's constructor and not some function that computes the values
    imported = {alias.asname or alias.name for node in tree.body if isinstance(node, ast.ImportFrom)
                and node.module and node.module.split(".")[0] == "robot_models" for alias in node.names}
    for index, node in enumerate(tree.body):
        if index == 0 and isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
            continue
        if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module and (
                node.module == "__future__" or node.module.split(".")[0] == "robot_models"):
            continue
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            target = node.targets[0].id
            if target == "source":
                found.extend(_call(node.value, (SOURCE,)))
                continue
            if target == "values":
                found.extend(_typed_call(node.value, imported - {SOURCE, STATED}))
                continue
            found.append(f"line {node.lineno}: only `source = Source(...)` and `values = <Type>(...)` belong here, "
                         f"not `{target} = ...`")
            continue
        found.append(f"line {node.lineno}: only a docstring, the imports, `source = Source(...)` and "
                     f"`values = <Type>(...)` belong here ({type(node).__name__})")
    return found


def _typed_call(node: ast.AST, types: set[str]) -> list[str]:
    if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in types):
        return [f"line {node.lineno}: values is a part type imported from robot_models, called with its fields "
                f"(`IntegratedServo(...)`), not {ast.unparse(node)[:60]}"]
    out = []
    if node.args:
        out.append(f"line {node.lineno}: every value is a keyword, `peak_torque=Stated(...)`")
    for keyword in node.keywords:
        if keyword.arg is None:
            out.append(f"line {node.lineno}: no ** in a source file")
        elif keyword.arg == "extra":
            if not isinstance(keyword.value, ast.Dict):
                out.append(f"line {keyword.value.lineno}: extra is a {{\"name\": Stated(...)}} dict")
                continue
            for key, value in zip(keyword.value.keys, keyword.value.values):
                if not (isinstance(key, ast.Constant) and isinstance(key.value, str)):
                    out.append(f"line {keyword.value.lineno}: extra's keys are plain names in quotes")
                out.extend(_call(value, (STATED,)))
        else:
            out.extend(_call(keyword.value, (STATED,)))
    return out


def _call(node: ast.AST, constructors: tuple[str, ...]) -> list[str]:
    if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in constructors):
        return [f"line {node.lineno}: a value is {' or '.join(c + '(...)' for c in constructors)}, not "
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


# --- the static reader -------------------------------------------------------------------------------------------

def read(path: str | pathlib.Path) -> dict[str, Any]:
    """A source file read without running it: {"source": {title, kind, role, file, ...}, "type": "IntegratedServo"
    or None, "values": {field: {value, unit, as_written, ...}}, "extra": {name: {...}}}.

    Refuses a file that is not literal-only (`problems`). Units, kinds and field names are NOT checked here: that is
    the loader's job, with the type in hand. This is what a reader that must never run a file (a catalogue build, a
    status table over `git archive`) uses.
    """
    path = pathlib.Path(path)
    found = problems(path)
    if found:
        raise ValueError(f"{path.name} is not literal-only: " + "; ".join(found))
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    out: dict[str, Any] = {"source": None, "type": None, "values": {}, "extra": {}}
    for node in tree.body:
        if not (isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)):
            continue
        target = node.targets[0].id
        if target == "source":
            out["source"] = _arguments(node.value, SOURCE_POSITIONAL)
        elif target == "values":
            out["type"] = node.value.func.id
            for keyword in node.value.keywords:
                if keyword.arg == "extra":
                    out["extra"] = {key.value: _arguments(value, STATED_POSITIONAL)
                                    for key, value in zip(keyword.value.keys, keyword.value.values)}
                else:
                    out["values"][keyword.arg] = _arguments(keyword.value, STATED_POSITIONAL)
    return out


def _arguments(call: ast.Call, positional: tuple[str, ...]) -> dict[str, Any]:
    arguments = {name: ast.literal_eval(argument) for name, argument in zip(positional, call.args)}
    arguments.update({keyword.arg: ast.literal_eval(keyword.value) for keyword in call.keywords})
    return arguments
