"""robot_models imports only {stdlib, numpy, robot_msgs, robot_models}; the contact extra adds coal + trimesh.

Held red: planted violations must fail the same checker, so a checker that passes everything
cannot pass this file.
"""

from __future__ import annotations

import ast
import subprocess
import sys
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "src" / "robot_models"
CORE_ROOTS = {"numpy", "robot_msgs", "robot_models"}
# WHY only these files: exact mesh contact is an optional extra; the core must import without it.
EXTRA_MODULES = {"homing/contact.py", "homing/cad_bundle.py"}
EXTRA_ROOTS = {"coal", "trimesh"}


def _roots(source: str, package_parts: list[str]) -> set[str]:
    roots = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:  # relative: resolves inside this package
                roots.add(package_parts[0])
            elif node.module:
                roots.add(node.module.split(".")[0])
    return roots


def violations(src: Path = SRC) -> list[str]:
    bad = []
    for path in sorted(src.rglob("*.py")):
        rel = path.relative_to(src).as_posix()
        allowed = CORE_ROOTS | (EXTRA_ROOTS if rel in EXTRA_MODULES else set())
        for root in _roots(path.read_text(encoding="utf-8"), ["robot_models"]):
            if root in sys.stdlib_module_names or root == "__future__" or root in allowed:
                continue
            bad.append(f"{rel}: imports {root}")
    return bad


def test_package_imports_only_its_layer():
    assert violations() == []


def _planted(tmp_path: Path, rel: str, text: str) -> Path:
    root = tmp_path / "robot_models"
    target = root / rel
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return root


def test_checker_goes_red_on_sim2real(tmp_path):
    assert violations(_planted(tmp_path, "homing/types.py", "import sim2real\n")) == ["homing/types.py: imports sim2real"]


def test_checker_goes_red_on_pidtuna(tmp_path):
    assert violations(_planted(tmp_path, "homing/fit.py", "from pidtuna import x\n")) == ["homing/fit.py: imports pidtuna"]


def test_checker_goes_red_on_coal_in_the_core(tmp_path):
    assert violations(_planted(tmp_path, "homing/fit.py", "import coal\n")) == ["homing/fit.py: imports coal"]


def test_core_import_pulls_in_no_contact_library():
    code = (
        "import sys, robot_models, robot_models.homing, robot_models.homing.fit, robot_models.homing.types\n"
        "bad = sorted(m for m in ('coal', 'trimesh', 'pinocchio', 'mujoco', 'scipy', 'sim2real', 'pidtuna')"
        " if m in sys.modules)\n"
        "assert not bad, bad\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
