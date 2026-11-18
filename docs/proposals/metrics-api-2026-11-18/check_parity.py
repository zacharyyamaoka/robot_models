"""Round 2's parity gate (G1): round 1's frozen check_scenario.py, unedited, pointed at this round's variant folders.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/check_parity.py [variant_folder ...]

Same oracle (round 8's scorecard and the five-bar ladder's summary), same tolerances, same structure rows (Z1 metric
cards, Z2 live = offline, G6-G8, A4 other inputs), same ``run() -> dict`` contract per ``scenario.py``. The folders
are this folder's ``baseline`` and ``r2-*`` directories, or the ones named.

WHY a wrapper and not a copy: the frozen file stays byte-identical (its md5 is printed beside every verdict), and
check_scenario.main() already accepts absolute folders; only its default looks in round 1's folder. A forbidden read
is named here first, because check_scenario words that problem as a path relative to round 1's folder, which raises for
a folder outside it.
"""

from __future__ import annotations

import hashlib
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROUND1 = HERE.parent / "metrics-api-2026-11-17"
FROZEN = ROUND1 / "check_scenario.py"
FROZEN_MD5 = "7d9ced1478d59e72fe4513e4466fbb59"   # check_scenario.py as round 1 left it (cc5edec)

sys.path.insert(0, str(ROUND1))

import check_scenario  # noqa: E402


def variant_folders(names: list[str]) -> list[pathlib.Path]:
    if names:
        return [(HERE / name).resolve() for name in names]
    return sorted(path for path in HERE.iterdir()
                  if path.is_dir() and (path.name == "baseline" or path.name.startswith("r2-")))


def main(names: list[str]) -> int:
    digest = hashlib.md5(FROZEN.read_bytes()).hexdigest()
    print(f"check_scenario.py md5 {digest} ({'frozen' if digest == FROZEN_MD5 else 'EDITED - not the frozen ruler'})")
    if digest != FROZEN_MD5:
        return 1
    folders = variant_folders(names)
    if not folders:
        print(f"no round-2 variant folders in {HERE} yet")
        return 1
    clean = []
    for folder in folders:
        offending = next((path for path in folder.rglob("*.py") if forbidden(path.read_text(encoding="utf-8"))), None)
        if offending:
            print(f"FAIL  {folder.name}\n      - {offending}: reads an oracle file or imports a heavy package (G1/G5)")
        else:
            clean.append(folder)
    verdict = check_scenario.main([str(folder) for folder in clean]) if clean else 0
    return 1 if len(clean) < len(folders) or verdict else 0


def forbidden(text: str) -> bool:
    return bool(check_scenario.FORBIDDEN_READS.search(text) or check_scenario.FORBIDDEN_IMPORTS.search(text))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
