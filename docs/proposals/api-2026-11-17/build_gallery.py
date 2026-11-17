"""Build the babble gallery for the five API proposals from round.json (the builders' and judges' returns).

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/build_gallery.py

Writes /home/bam/robot_models/reports/robot-models-api-proposals-2026-11-17.html via the babble skill's gallery.py.
WHY a script: the gallery is regenerated whenever a variant or a score changes, never hand-edited.
"""

from __future__ import annotations

import html
import re
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
GALLERY = pathlib.Path.home() / ".claude/skills/babble/scripts/gallery.py"
OUT = pathlib.Path("/home/bam/robot_models/reports/robot-models-api-proposals-2026-11-17.html")
# WHY 5: the gallery's walkthrough takes two to five steps; a session's later cells (the held-out choice, servo B, the
# hand-on) fold into the fifth, so every cell still drives a real step.
STORY_STEPS = 5


def story_state(index: int) -> str:
    return f"s{min(index, STORY_STEPS)}"


ACCENTS = {"v1": "#3f7be0", "v2": "#2e9e6b", "v3": "#c9822b", "v4": "#b5487a", "v5": "#7a5bd1"}
CONFIDENCE = ("low", "medium", "high")

REQUIREMENTS = [
    ("fr1", "The Model is the thing you hold", 25,
     "Zach's ask: Model as the base object. Today the rung record and its physics are joined by a string path.",
     "One object says what it needs, what it means and where it comes from, and computes.",
     ("Metadata and runnable code are separate objects joined by a string path (today).",
      "One object, but it needs its family or a registry to run or describe itself.",
      "One object says what it needs, what it means, where it comes from, and computes.")),
    ("fr2", '"What now" is short', 25,
     'Zach: "I have accurate datasheet and the rigs, and I have selected the models.. what now?"',
     "Datasheet to model, fit on a log, compare rungs: one call each.",
     ("The user writes unit conversion, fitting and comparison glue.",
      "Predict is direct; fit or compare needs hand glue.",
      "Datasheet → model, fit on a log, compare rungs: one call each; the scenario reads as the workflow.")),
    ("fr3", "The ladder stays browsable and honest", 15,
     "The ladder (tree, fallback, same-level alternatives, one vocabulary) is what PEP 0004 exists for.",
     "The family prints as a tree; fallback, siblings and aliases are one call each.",
     ("The tree, fallback or same-level alternatives are lost.", "Derivable with effort.",
      "The family prints as a tree; fallback, siblings and aliases are one call each.")),
    ("fr4", "Migration cost", 15,
     "robotparts vendors the rung bytes, sim2real has a parity test, and the v1 ruler holds 201 tests.",
     "The v1 ruler, rung bytes and consumers keep working.",
     ("The contract is rewritten and consumers break.",
      "The ruler is re-expressed; robotparts' vendoring or sim2real's parity test changes.",
      "An additive layer: the v1 ruler, rung bytes and consumers untouched.")),
    ("fr5", "Adding a model is small and local", 10,
     "Twelve types and growing; agents add rungs concurrently.",
     "One file or class in the type's folder, found automatically.",
     ("A new rung edits several files or a registry.", "One folder plus one registration.",
      "One file or class in the type's folder, found automatically.")),
    ("fr6", "Plays with the stack", 10,
     "Values flow to MuJoCo, sim2real and fitting tools; Zach reads the code in an IDE.",
     "Numpy-vectorized, typed parameters, values hand to MuJoCo/sim2real/scipy without glue.",
     ("Custom idioms; arrays need loops; nothing exports.", "Some standard idioms.",
      "Numpy-vectorized, type checkers see the parameters, values hand to MuJoCo/sim2real/scipy without glue.")),
]
GATES = [
    ("g1", "Type data only", "robot_models holds no measured value and no maker; values come from the caller."),
    ("g2", "Light runtime core", "numpy + robot_msgs + stdlib at runtime; anything heavier is an extra."),
    ("g3", "Recovers the hidden truth", "check_scenario.py passes and the variant never reads _truth.py."),
    ("g4", "One vocabulary per type", "Canonical names and aliases resolve as today."),
]


def esc(text: str) -> str:
    return html.escape(text or "", quote=True)


def prose(text: str) -> str:
    """The builders' README-style notes: **bold** and `code` rendered, everything else escaped."""
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", esc(text))
    return re.sub(r"`([^`]+)`", r"<code>\1</code>", text)


STYLE = """<style>
.api-repl{font:13px/1.45 ui-sans-serif,system-ui,sans-serif;color:var(--ink,#1d232b);padding:14px;display:grid;grid-template-columns:minmax(0,1fr);gap:10px;width:0;min-width:100%;box-sizing:border-box}
.api-repl .repl-head{display:flex;justify-content:space-between;gap:8px;font-size:12px;opacity:.8}
.api-repl ol{list-style:none;margin:0;padding:0;display:grid;grid-template-columns:minmax(0,1fr);gap:6px}
.api-repl .cell{all:unset;box-sizing:border-box;display:grid;grid-template-columns:22px minmax(0,1fr);gap:8px;width:100%;cursor:pointer;
  border:1px solid color-mix(in srgb,currentColor 18%,transparent);border-radius:8px;padding:7px 9px;
  background:color-mix(in srgb,currentColor 4%,transparent)}
.api-repl .cell:hover{border-color:var(--accent,#3f7be0)}
.api-repl .cell .n{font-weight:700;color:var(--accent,#3f7be0)}
.api-repl pre{margin:0;white-space:pre-wrap;overflow-wrap:anywhere;font:12px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace}
.api-repl ol,.api-repl li{min-width:0}
.api-repl .out{display:none;white-space:pre;overflow-x:auto;margin:2px 0 4px 30px;padding:8px 10px;border-left:3px solid var(--accent,#3f7be0);
  background:color-mix(in srgb,var(--accent,#3f7be0) 8%,transparent);border-radius:0 6px 6px 0}
.api-repl .lbl{font-size:11px;text-transform:uppercase;letter-spacing:.04em;opacity:.65;grid-column:2}
.shape-tree{font:12px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;white-space:pre-wrap;overflow-wrap:anywhere;margin:0;padding:12px}
.shape-tree code{font:inherit;background:color-mix(in srgb,currentColor 8%,transparent);padding:0 2px;border-radius:3px}
</style>"""


def preview(variant: dict) -> str:
    states = []
    cells = []
    for index, step in enumerate(variant["steps"], start=1):
        state = story_state(index)
        if state not in states:
            states.append(state)
        cells.append(
            f'<li><button type="button" class="cell" data-story-to="{state}" title="Show this step\'s output">'
            f'<span class="n">{index}</span><pre>{esc(step["code"].strip())}</pre>'
            f'<span class="lbl">{esc(step["label"])}</span></button>'
            f'<pre class="out" data-out="{state}">{esc(step["output"].rstrip())}</pre></li>')
    rules = "".join(
        f'.prototype[data-story-state="{s}"] .api-repl .out[data-out="{s}"]{{display:block}}'
        f'.prototype[data-story-state="{s}"] .api-repl .cell[data-story-to="{s}"]{{border-color:var(--accent,#3f7be0);'
        f'box-shadow:0 0 0 2px color-mix(in srgb,var(--accent,#3f7be0) 25%,transparent)}}' for s in states)
    rules += '.prototype:not([data-story-state]) .api-repl .out[data-out="s1"]{display:block}'
    accent = ACCENTS[variant["id"]]
    return (f'{STYLE}<style>{rules}</style><div class="api-repl" style="--accent:{accent}">'
            f'<div class="repl-head"><b>integrated_servo · "what now?" session</b>'
            f'<span>{variant["core_lines"]} lines of API core</span></div><ol>{"".join(cells)}</ol></div>')


def judged(judge: dict, variant_id: str) -> dict:
    """A judge's entry for a variant: judges name it by its folder (v1_subclass), the builders by its id (v1)."""
    return next(v for v in judge["variants"] if v["id"] == variant_id or v["id"].startswith(variant_id + "_"))


def merged_scores(variant_id: str, judges: list[dict]) -> dict:
    scores = {}
    for fr_id, *_ in REQUIREMENTS:
        cells = [(j["lens"], judged(j, variant_id)["scores"][fr_id]) for j in judges]
        values = [c["score"] for _, c in cells]
        spread = max(values) - min(values)
        confidence = min((c["confidence"] for _, c in cells), key=CONFIDENCE.index)
        if spread >= 2:
            confidence = "low"
        evidence = " · ".join(f"{lens} lens ({c['score']}): {c['evidence']}" for lens, c in cells)
        scores[fr_id] = {"score": round(sum(values) / len(values), 2), "evidence": evidence, "confidence": confidence}
    return scores


def merged_gates(variant_id: str, judges: list[dict]) -> dict:
    gates = {}
    for gate_id, *_ in GATES:
        cells = [(j["lens"], judged(j, variant_id)["gates"][gate_id]) for j in judges]
        gates[gate_id] = {"pass": all(c["pass"] for _, c in cells),
                          "evidence": " · ".join(f"{lens} lens: {c['evidence']}" for lens, c in cells)}
    return gates


def total(scores: dict) -> float:
    return sum(weight * scores[fr_id]["score"] / 5 for fr_id, _, weight, *_ in REQUIREMENTS)


def main() -> int:
    round_data = json.loads((HERE / "round.json").read_text())
    built = sorted((b for b in round_data["built"] if b), key=lambda b: b["id"])
    judges = [j for j in round_data["judges"] if j]
    decision = json.loads((HERE / "decision.json").read_text())
    variants = []
    for b in built:
        scores = merged_scores(b["id"], judges)
        steps = []
        for i, s in enumerate(b["steps"], start=1):
            state = story_state(i)
            if steps and steps[-1]["state"] == state:
                steps[-1]["label"] += " · " + s["label"]
                steps[-1]["caption"] += " " + s["caption"]
            else:
                steps.append({"label": s["label"], "caption": s["caption"], "state": state,
                              "target": f'[data-story-to="{state}"]'})
        variants.append({
            "id": b["id"], "name": b["name"], "thesis": b["thesis"], "accent": ACCENTS[b["id"]],
            "bestWhen": b["best_when"], "losesWhen": b["loses_when"],
            "decisions": b["decisions"][:4], "keepParts": b["keep_parts"][:3],
            "proof": [b["check_output"].strip().splitlines()[0] if b["check_output"].strip() else "no check output",
                      f"Scenario run: docs/proposals/api-2026-11-17/{b['slug']}/scenario.out.txt",
                      f"Simulated: {b['simulated']}"],
            "scores": scores, "gateResults": merged_gates(b["id"], judges),
            "previewLabel": "live session · click a cell or step",
            "preview": preview(b),
            "story": {"title": "Walk the session", "steps": steps},
            "media": [
                {"label": "The object model", "caption": "What a Model is here, what holds it, and the calls you make.",
                 "html": f'<pre class="shape-tree">{esc(b["shape"])}</pre>'},
                {"label": "Adding a rung", "caption": "What a contributor writes to add a new M3 friction model.",
                 "html": f'<pre class="shape-tree">{prose(b["new_rung_cost"])}</pre>'},
                {"label": "Migration from v1", "caption": "What changes for the ruler, robotparts' vendoring and "
                 "sim2real's parity test.", "html": f'<pre class="shape-tree">{prose(b["migration"])}</pre>'},
            ],
        })
    ranked = sorted(((total(v["scores"]), v["id"]) for v in variants
                     if all(g["pass"] for g in v["gateResults"].values())), reverse=True)
    spec = {
        "schemaVersion": 3,
        "title": "robot_models: five Model-first APIs",
        "kicker": "Babble & Prune · /babble 5 · 2026-11-17",
        "brief": ("Make Model the base object and let each type organize its models into a ladder, so that with a "
                  "datasheet and a rig log in hand the next steps (build, predict, fit, compare rungs) read naturally."),
        "count": len(variants),
        "defaultId": ranked[0][1],
        "defaultWhy": decision["why"],
        "decisionHinge": decision["hinge"],
        "invariants": [
            "The integrated_servo ladder's 14 keys in today's order, and its fallback chains.",
            "One vocabulary per type; frictionloss_nm resolves to coulomb_friction.",
            "The physics in actuators/rungs/*/model.py is reused, not retyped.",
            "One shared scenario: an illustrative servo datasheet and a simulated rig log from a hidden truth.",
        ],
        "boundary": ("Each variant is a runnable prototype in docs/proposals/api-2026-11-17/ that passes the frozen "
                     "check: fits recover the hidden truth, and the rung is chosen on held-out plateaus, with a negative control "
                     "where the simpler rung must win. Runnable physics M0–M3 only; nothing in src/, the ruler, robotparts "
                     "or sim2real changed."),
        "axes": [
            {"name": "What a Model is", "values": ["dataclass class", "record + bound fn", "sum of terms",
                                                   "estimator", "kit member with its experiment"]},
            {"name": "What organizes the ladder", "values": ["inheritance", "Family container", "slots + presets",
                                                             "model selection", "the Type kit"]},
            {"name": "Where the 'what now' verbs live", "values": ["instances", "BoundModel", "composite",
                                                                   "Model + Family.climb", "Type.identify"]},
        ],
        "requirements": [
            {"id": fr_id, "name": name, "weight": weight, "why": why, "passCondition": passing,
             "anchors": {"1": a1, "3": a3, "5": a5}}
            for fr_id, name, weight, why, passing, (a1, a3, a5) in REQUIREMENTS],
        "hardGates": [{"id": g, "name": n, "why": w} for g, n, w in GATES],
        "variants": variants,
    }
    spec_path = HERE / "gallery-spec.json"
    spec_path.write_text(json.dumps(spec, indent=1, ensure_ascii=False))
    if OUT.exists():
        OUT.unlink()  # WHY: gallery.py refuses to overwrite; this script owns the output
    subprocess.run([sys.executable, str(GALLERY), "build", "--spec", str(spec_path), "--output", str(OUT)], check=True)
    subprocess.run([sys.executable, str(GALLERY), "check", "--input", str(OUT), "--strict"], check=True)
    for score, variant_id in ranked:
        print(f"{variant_id} {score:.1f}")
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
