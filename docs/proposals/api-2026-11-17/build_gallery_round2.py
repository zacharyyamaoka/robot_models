"""Build the round-2 babble gallery (ModelFamily -> Model -> Parameters) from round2.json and decision2.json.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/build_gallery_round2.py

Writes /home/bam/robot_models/reports/robot-models-api-proposals-round2-2026-11-17.html. Reuses round 1's renderer
(build_gallery.py) with round 2's criteria (ROUND2.md), accents and texts.
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys

import build_gallery as g

HERE = pathlib.Path(__file__).resolve().parent
OUT = pathlib.Path("/home/bam/robot_models/reports/robot-models-api-proposals-round2-2026-11-17.html")

g.ACCENTS = {"v6": "#3f7be0", "v7": "#2e9e6b", "v8": "#b5487a"}
g.REQUIREMENTS = [
    ("fr1", "His access shape", 25,
     "Zach's feedback names the calls: models[key], models.m3.stribeck, ladder()[0], tree().print()/show(), "
     "iteration, param_count.",
     "Every call in ROUND2.md works as written or one obvious spelling away.",
     ("Most calls missing or reshaped.", "The calls exist with glue or different spellings.",
      "Every call works as written, iteration and param_count included.")),
    ("fr2", "Per-model and batch operations", 20,
     "\"we can do operations on per model level or ... BLDC.fit() ... BLDC.select_best()\"",
     "model.fit, Family.fit and Family.select_best(held_out) are one call each; the batch result reads easily.",
     ("Fitting or choosing needs hand glue.", "One of the three needs glue.",
      "All three are one call each and the batch result is easy to read.")),
    ("fr3", "One source, additive vocabulary", 15,
     "Round 1's V1 wrote every rung twice and dropped a class-less rung silently; Zach: vocabulary only ever grows.",
     "The classes are the source; today's Ladder is derived and equal; a subclass cannot remove or rename.",
     ("Two sources, or vocabulary can shrink.", "Derived but partial, or enforcement has holes.",
      "Derived Ladder equals today's; removal or renaming raises; nothing drops silently.")),
    ("fr4", "Analytical and black box, one interface", 15,
     "\"eventually though models could even be like NeuralNetwork ... from the outside they are a bit the same\"",
     "A learned model fits, predicts, counts parameters and competes in select_best with the same calls.",
     ("The black box needs its own API.", "Same calls with special cases.",
      "Same calls, same tables, no special case.")),
    ("fr5", "Migration", 10,
     "The v1 ruler, robotparts' vendored rung bytes and sim2real's parity test consume today's API.",
     "They keep working.",
     ("Consumers break.", "Changes are needed in one consumer.", "All three keep working unchanged.")),
    ("fr6", "Plays with the stack", 15,
     "Values flow to MuJoCo, sim2real and fitting; Zach reads it in an IDE.",
     "Typed parameters, numpy-vectorized predict, values hand to MuJoCo/sim2real without glue.",
     ("Untyped, scalar loops, nothing exports.", "Some of it.", "All of it.")),
]


def main() -> int:
    round_data = json.loads((HERE / "round2.json").read_text())
    built = sorted(round_data["built"], key=lambda b: b["id"])
    judges = [j for j in round_data["judges"] if j]
    decision = json.loads((HERE / "decision2.json").read_text())
    variants = []
    for b in built:
        steps = []
        for i, s in enumerate(b["steps"], start=1):
            state = g.story_state(i)
            if steps and steps[-1]["state"] == state:
                steps[-1]["label"] += " · " + s["label"]
                steps[-1]["caption"] += " " + s["caption"]
            else:
                steps.append({"label": s["label"], "caption": s["caption"], "state": state,
                              "target": f'[data-story-to="{state}"]'})
        variants.append({
            "id": b["id"], "name": b["name"], "thesis": b["thesis"], "accent": g.ACCENTS[b["id"]],
            "bestWhen": b["best_when"], "losesWhen": b["loses_when"],
            "decisions": b["decisions"][:4], "keepParts": b["keep_parts"][:3],
            "proof": [b["check_output"].strip().splitlines()[0],
                      f"Scenario run: docs/proposals/api-2026-11-17/{b['slug']}/scenario.out.txt",
                      f"Simulated: {b['simulated']}"],
            "scores": g.merged_scores(b["id"], judges), "gateResults": g.merged_gates(b["id"], judges),
            "previewLabel": "live session · click a cell or step", "preview": g.preview(b),
            "story": {"title": "Walk the session", "steps": steps},
            "media": [
                {"label": "The object model", "caption": "ModelFamily, Model and Parameters here, and the calls.",
                 "html": f'<pre class="shape-tree">{g.esc(b["shape"])}</pre>'},
                {"label": "Adding a rung", "caption": "What a contributor writes to add a new model.",
                 "html": f'<pre class="shape-tree">{g.prose(b["new_rung_cost"])}</pre>'},
                {"label": "Migration from v1", "caption": "What changes for the ruler, robotparts and sim2real.",
                 "html": f'<pre class="shape-tree">{g.prose(b["migration"])}</pre>'},
            ],
        })
    ranked = sorted(((g.total(v["scores"]), v["id"]) for v in variants
                     if all(x["pass"] for x in v["gateResults"].values())), reverse=True)
    spec = {
        "schemaVersion": 3,
        "title": "robot_models round 2: ModelFamily, Model, Parameters",
        "kicker": "Babble & Prune · round 2 · 2026-11-17",
        "brief": ("Zach's feedback on round 1: organize everything around a type. A ModelFamily holds Models; a Model is "
                  "a class made of Parameters, and subclassing only adds vocabulary; per-model and batch fit, and "
                  "select_best on the data you have."),
        "count": len(variants),
        "defaultId": ranked[0][1],
        "defaultWhy": decision["why"],
        "decisionHinge": decision["hinge"],
        "invariants": [
            "ModelFamily → Model (a class) → Parameters; a rung subclasses its base.",
            "His access shape: models[key], models.m3.stribeck, ladder()[0], tree().print()/show(), iteration.",
            "Today's Ladder is derived from the classes and equals ladders()['integrated_servo'].",
            "The same scenario and check as round 1, with the held-out choice and the negative control.",
        ],
        "boundary": ("Runnable prototypes in docs/proposals/api-2026-11-17/v6–v8 that pass the frozen check. Physics "
                     "for M0–M3 plus a numpy black box at M7; nothing in src/, robotparts or sim2real changed."),
        "axes": [
            {"name": "Where the values live", "values": ["a Model instance", "a ModelFamily instance (one unit)",
                                                         "a Parameters object"]},
            {"name": "What fit returns", "values": ["fitted instances", "the unit, fitted in place", "value sets"]},
        ],
        "requirements": [
            {"id": fr_id, "name": name, "weight": weight, "why": why, "passCondition": passing,
             "anchors": {"1": a1, "3": a3, "5": a5}}
            for fr_id, name, weight, why, passing, (a1, a3, a5) in g.REQUIREMENTS],
        "hardGates": [{"id": gid, "name": n, "why": w} for gid, n, w in g.GATES],
        "variants": variants,
    }
    spec_path = HERE / "gallery-spec-round2.json"
    spec_path.write_text(json.dumps(spec, indent=1, ensure_ascii=False))
    if OUT.exists():
        OUT.unlink()  # WHY: gallery.py refuses to overwrite; this script owns the output
    subprocess.run([sys.executable, str(g.GALLERY), "build", "--spec", str(spec_path), "--output", str(OUT)], check=True)
    subprocess.run([sys.executable, str(g.GALLERY), "check", "--input", str(OUT), "--strict"], check=True)
    for score, variant_id in ranked:
        print(f"{variant_id} {score:.1f}")
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
