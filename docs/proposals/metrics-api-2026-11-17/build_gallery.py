"""Build the babble gallery for the five metrics-API proposals from their real session transcripts.

    /home/bam/robot_models/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/build_gallery.py

Reads each vN_*/session.json (the code that ran and what it printed), check.out.txt (the frozen check's verdicts) and
the view PNGs in reports/media/, and writes reports/robot-models-metrics-api-proposals-2026-11-17.html through the
babble skill's gallery.py (deck shell). WHY a script: the gallery is regenerated when a variant or a score changes,
never hand-edited. The scores and the pick are the AI's judgment, written here beside their evidence.
"""

from __future__ import annotations

import base64
import html
import json
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
GALLERY = pathlib.Path.home() / ".claude/skills/babble/scripts/gallery.py"
OUT = pathlib.Path("/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-17.html")
MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
STORY_STEPS = 5      # the deck's stepper; the session's sixth cell folds into the fifth
ACCENTS = {"v1": "#3f7be0", "v2": "#2e9e6b", "v3": "#c9822b", "v4": "#b5487a", "v5": "#7a5bd1"}
FOLDERS = {"v1": "v1_robot_metrics", "v2": "v2_family_metrics", "v3": "v3_signals", "v4": "v4_folded_values",
           "v5": "v5_part_tree"}

REQUIREMENTS = [
    ("fr1", "A part's metrics read like its models", 20,
     'Zach: "RobotPart.models -> ModelFamily, RobotPart.metrics -> metrics".',
     "Part.metrics beside .models, each a configured stock metric with name, description, references, unit.",
     ("The rows are functions scattered over repos.", "A list exists, away from the part type.",
      "Part.metrics beside .models, each a configured stock metric with name, description, references and unit.")),
    ("fr2", "Recording in, summary out", 20,
     'Zach: "pass in the Run Recording into the black box and then the metrics come out"; summaries across runs.',
     "One call per recording, one for runs -> rung -> station -> deployment.",
     ("Per-run and roll-up are hand glue.", "Per run is one call; roll-ups are hand-written.",
      "One call per recording, one for the roll-up, pooled vs per-run stated per metric.")),
    ("fr3", "Live = offline, honestly streaming", 15, 'Zach: "Able to compute live and offline."',
     "Bounded state, stops safe across chunks, alignment handled or stated.",
     ('"Live" re-runs the whole buffer, unstated.', "Exact, but the state grows with the run.",
      "Bounded state, stops safe across chunk boundaries, time alignment handled or stated.")),
    ("fr4", "Gates stay outside compute", 15,
     'Zach: "100% agreed keep the target outside of compute"; his old rule, separate metric and threshold.',
     "Metric + Limits / target = diagnostic / gate, owner on each, median and worst visible.",
     ("Thresholds inside the metric.", "Metric + threshold, but owner or worst-vs-median hidden.",
      "Metric + Limits or target outside compute, owner on each, median and worst both visible.")),
    ("fr5", "Adding a metric or a part type is small", 10, "12 types and growing; agents add metrics concurrently.",
     "One subclass or one constructor call in the part's list.",
     ("Several files and a registry.", "One class plus a registration.", "One subclass or one constructor call.")),
    ("fr6", "One formula in one place", 10,
     "The same formulas live in robot_msgs.compare, sim2real.metrics and robotdx.diagnostics.stock today.",
     "The three copies become one, and the variant says what happens to each.",
     ("A fourth copy.", "Says where the copies go, keeps two.", "The three copies become one, each fate stated.")),
    ("fr7", "Standard views", 10, 'Zach: "each metric can support standard visualization ... matplotlib, W&B, rerun".',
     "plot() to matplotlib, the same result to W&B and Rerun, the data-ui hint carried.",
     ("Numbers only.", "A plot.", "plot() to matplotlib, the same result to W&B and Rerun, the view hint carried.")),
]
GATES = [
    ("g1", "Parity with round 8", "check_scenario.py: the scorecard and the five-bar ladder at frozen tolerances."),
    ("g2", "torchmetrics-shaped", "Metric base, constructor config, update/compute/reset, a collection, a functional twin."),
    ("g3", "Zach's Metric fields", "Name, description, references; number or bool; target outside compute()."),
    ("g4", "Live = offline", "A recording streamed in chunks equals the offline value (1e-9)."),
    ("g5", "Light, black-box core", "numpy + robot_msgs at runtime; no metric reads a model; source is a label."),
    ("g6", "Math-only bases", "One base serves several aliases; the meaning comes from the data."),
    ("g7", "Aliases, identity, units", "Identity = base + frozen config; units on the alias."),
    ("g8", "Computed once", "Aliases of one identity on one recording compute once."),
]
PRIOR_ART = [
    ("P1", "torchmetrics shape without torch"), ("P2", "state as a mergeable value"),
    ("P3", "Limits outside the config"), ("P4", "occurrence gates, a tail per stratum, Wilson for bools"),
    ("P5", "alignment an explicit step with a tolerance"), ("P6", "context beside the data, provenance carried"),
    ("P7", "live flag, value versions, units at display"),
]

VARIANTS = {
    "v1": {
        "name": "robot_metrics (torchmetrics, ported)",
        "thesis": "A literal numpy port of torchmetrics: add_state states, update/compute/reset/merge_state, wrappers "
                  "and compute groups, in its own robot_metrics package; a part type lists aliases as .metrics.",
        "where": "robot_msgs <- robot_metrics <- robotparts (RobotPart.metrics)\nrobot_models untouched\n"
                 "run data: RunRecording (named robot_msgs streams + labels + .rrd pointer)",
        "fate": "robotdx.diagnostics.stock: compute half -> robot_metrics classes (scale/unit/limits leave the config); "
                "RobotDx keeps Limits, judge, Result.\nsim2real.metrics.actuator: deleted, callers use robot_metrics.functional.\n"
                "CalibrationOutcome: robot_metrics.records.",
        "best": "You want torchmetrics' muscle memory exactly, and live monitors that run for hours (bounded state).",
        "loses": "State hides in attributes: merge needs each reduce rule right, compute() mid-run deep-copies the open "
                 "tracker, and collections with different configs (stations' tip lengths) merge only through gates.",
        "decisions": ["add_state(sum|max|cat) states", "Dwells shared by compute groups", "Diagnostic(alias, Limits, owner)",
                      "merge_state + PerRun(median)"],
        "keep": ["Metric.plot() -> (fig, ax)", "compute groups over a shared Dwells state", "PerRun wrapper"],
        "prior": {"P1": "yes", "P2": "partial (merge_state, attributes)", "P3": "yes", "P4": "no (value thresholds)",
                  "P5": "no (the data adapter's)", "P6": "partial (provenance label)", "P7": "partial (live flag, units)"},
        "scores": {"fr1": (4, "PartType(.models=IntegratedServo, .metrics=aliases); the table shows each base, config, "
                                  "signal and unit; a stand-in for robotparts' RobotPart.", "high"),
                   "fr2": (4, "collection(recording) is one call; a rung is merge_state of its plays; the grouping loop "
                                  "is scenario code.", "medium"),
                   "fr3": (4, "O(1) sufficient statistics, the stop tracker carries the open dwell, a mid-run peek; "
                                  "alignment left to the data adapter; 1.4 ms per 7-row update.", "high"),
                   "fr4": (4, "Diagnostic(alias, Limits, owner) and Gate; median and worst both shown; thresholds only, "
                                  "no occurrence form.", "high"),
                   "fr5": (4, "Velocity error is one Alias; new math is a subclass with setup/_update/compute.", "high"),
                   "fr6": (4, "Replaces robotdx stock compute and sim2real.actuator; math stays in robot_msgs.compare.",
                           "medium"),
                   "fr7": (4, "Metric.plot() returns (fig, ax); the same View to W&B and a written .rrd; hint ClassVar.",
                           "high")},
    },
    "v2": {
        "name": "Family metrics (IntegratedServo.metrics)",
        "thesis": "Metrics declared on the part type's ModelFamily beside its models; a cat buffer for state and one "
                  "functional over (config, runs), RobotDx's signature; one object fed a rung's runs is the rung.",
        "where": "robot_msgs <- robot_models (models AND metrics) <- robotparts <- sim2real / RobotDx\n"
                 "IntegratedServo.metrics in the family body; rigid_body_chain by type id until it has a family\n"
                 "run data: Episode (named streams + labels + .rrd)",
        "fate": "robotdx.diagnostics.stock: functions move into robot_models.metrics with (config, runs) unchanged.\n"
                "sim2real.metrics.actuator: a re-export until callers move.\nCalibrationOutcome: robot_models.metrics.",
        "best": "Offline datasets are the main use, and a type's models and metrics should be read in one place.",
        "loses": "Live for long (the buffer grows, compute re-reads it: 18 ms per call), and metrics in the models "
                 "package go against Zach's 'all there really is is models' and his later robot_metrics package.",
        "decisions": ["cat buffer + functional twin", "memo on (identity, data)", "Gate(alias, robot_msgs Value)",
                      "across_runs in the constructor"],
        "keep": ["Dwell(quantity=...) as config", "the cross-consumer memo", "targets as robot_msgs Values with source"],
        "prior": {"P1": "yes", "P2": "no (a buffer)", "P3": "yes", "P4": "no (value gates)", "P5": "no",
                  "P6": "partial (source label)", "P7": "partial (live flag)"},
        "scores": {"fr1": (4, "IntegratedServo.metrics sits on the family beside its 14 models (the literal adjacency); "
                                  "the chain has no family yet, and it conflicts with the models-only naming ruling.",
                           "medium"),
                   "fr2": (5, "metrics(episode) per play; one Metrics fed a rung's 7 plays computes the rung by its "
                                  "constructor's across_runs.", "high"),
                   "fr3": (2, "Exact by construction but O(rows) memory and an 18 ms compute that re-reads the buffer.",
                           "high"),
                   "fr4": (4, "Deployment holds Gate(alias, Value with source); median and worst both; no occurrence form.",
                           "high"),
                   "fr5": (5, "A new stop quantity is config (Dwell(quantity=...)); FeedbackShare shows the subclass path.",
                           "high"),
                   "fr6": (3, "Ports RobotDx's functions 1:1, but puts the math in the models package.", "medium"),
                   "fr7": (3, "A view() helper feeds the adapter; no plot() on the metric itself.", "medium")},
    },
    "v3": {
        "name": "Signals and aliases (robot_msgs.metrics)",
        "thesis": "Pure signal metrics in robot_msgs beside compare; meaning bound by aliases in the part's catalog; raw "
                  "command and state topics aligned live, aliases compiled into a deduplicated Plan.",
        "where": "robot_msgs (+ .metrics) <- robotparts' catalog (RobotDx's catalog.py moved) <- sim2real / RobotDx\n"
                 "run data: Recording = topics (MCAP-like), replayed in arrival order",
        "fate": "robotdx.diagnostics.stock: math -> robot_msgs.metrics; catalog.py -> robotparts' alias catalog.\n"
                "sim2real.metrics.actuator: deleted.\nCalibrationOutcome: a robot_msgs message.",
        "best": "Live streams on their own clocks need explicit alignment, and the math should sit at the lowest layer.",
        "loses": "robot_msgs grows a metrics framework it was not meant to hold, and an IQM read as a gate trims "
                 "CAN 11's failing stops (so gates read point values).",
        "decisions": ["Aligner holds each reply 1 tick", "Plan compiles aliases once", "Check(alias, Limits, owner) rows",
                      "rliable-style intervals"],
        "keep": ["the live Aligner (hermite | same_clock)", "Plan's common-subexpression dedup", "bootstrap CI beside points"],
        "prior": {"P1": "yes", "P2": "no (attributes)", "P3": "yes", "P4": "no (IQM for display, value gates)",
                  "P5": "yes (the Aligner, 1-tick latency)", "P6": "partial (source label)", "P7": "partial (live flag)"},
        "scores": {"fr1": (4, "PartType.metrics is an alias catalog with Checks and owners beside .models.", "high"),
                   "fr2": (4, "plan.run(recording) per play; across_runs per alias; aggregate() with CIs; grouping by "
                                  "hand.", "medium"),
                   "fr3": (5, "Raw topics message by message; the aligner holds at most 1 reply; exact; 0.66 ms per "
                                  "message pair.", "medium"),
                   "fr4": (3, "Checks with owners on point values; the CI is beside, not a gate; no occurrence form.",
                           "medium"),
                   "fr5": (4, "One alias line; the plan dedups automatically; new math is a subclass.", "high"),
                   "fr6": (4, "Math at the lowest layer, RobotDx's catalog becomes the alias table; but the prior-art "
                                  "note keeps classes out of robot_msgs.", "medium"),
                   "fr7": (3, "Views through the adapter in scenario code; no plot() on the metric.", "medium")},
    },
    "v4": {
        "name": "Folded values (state as a value, occurrence gates)",
        "thesis": "Frozen, hashable metrics whose state is a value (empty/update/merge/close/compute): live a fold over "
                  "chunks, offline the same fold once, runs merge for free; OpenSLO occurrence gates judged into "
                  "RobotDx's own Result.",
        "where": "robot_msgs <- robot_metrics (published from io-diagnostics beside RobotDx) <- robotparts\n"
                 "run data: Run (the run-data MOC: id, streams, labels, provenance, outcome, links rrd/wandb)",
        "fate": "robotdx.diagnostics.stock: formulas -> these classes, limits removed from every config; RobotDx keeps "
                "Limits, judge, Result, records.\nsim2real.metrics.actuator: deleted.\nCalibrationOutcome: robot_msgs message.",
        "best": "States must be stored, resumed and merged across machines or rounds, and gates must state tails.",
        "loses": "A new base costs four methods (empty, update, merge, compute), and each update copies the open tracker.",
        "decisions": ["state is a value (clu / torcheval)", "Stream = the torchmetrics face",
                      "Objective(op, value, share, on)", "RobotDx Result as the verdict"],
        "keep": ["value states that merge", "occurrence objectives on items", "Wilson intervals for bool metrics"],
        "prior": {"P1": "yes (Stream)", "P2": "yes", "P3": "yes", "P4": "yes", "P5": "partial (an alignment check)",
                  "P6": "partial (provenance, links)", "P7": "yes (live, version, units at display)"},
        "scores": {"fr1": (4, "PartType.metrics with live and version per alias; frozen metrics are their own identity.",
                           "high"),
                   "fr2": (4, "collection.fold(run) per play; summarize(collection, runs, by=strata) for any labels.",
                           "high"),
                   "fr3": (5, "A fold of value states over chunks, merge for free, an explicit alignment check, lag "
                                  "declared not live; 0.97 ms per update.", "high"),
                   "fr4": (5, "Occurrence and value objectives with owners, the same alias carries the median and the "
                                  "every-stop gate, RobotDx Result, Wilson for bools, limits out of configs.", "high"),
                   "fr5": (4, "A new alias is one frozen config; new math is four methods.", "high"),
                   "fr6": (4, "Lifts RobotDx's formulas, keeps its judge and records, deletes sim2real.actuator.",
                           "medium"),
                   "fr7": (3, "The view hint is carried; views through the adapter; no plot() on the metric.", "medium")},
    },
    "v5": {
        "name": "Part tree (RobotPart.metrics composes)",
        "thesis": "The deployment is a tree of RobotParts; each node judges its own output, an assembly reads its "
                  "children's through context (the arm's lever), and the state is move and dwell events.",
        "where": "robot_msgs <- robotparts (RobotPart: .models, .metrics, .children) <- sim2real / RobotDx\n"
                 "run data: named streams routed to nodes by their signals; each node keeps a ledger",
        "fate": "robotdx.diagnostics.stock: catalog -> node declarations, formulas -> event metrics in robotparts "
                "(RobotDx would depend on robotparts).\nsim2real.metrics.actuator: deleted.\nCalibrationOutcome: robot_msgs message.",
        "best": "The deployment page should read as the robot does (rack -> arm -> servo), each red cell naming the "
                "part and the event to open.",
        "loses": "A metric that is not a move/dwell event (a camera's single-instant error), hand-written roll-up "
                 "tables over ledgers, and formulas high in the stack.",
        "decisions": ["segment events as state", "assemblies read children (composition = dedup)",
                      "levels OK/WARN/ERROR, worst child", "context beside the data"],
        "keep": ["the arm reads its servo child", "events as the drill-down", "worst-child status with the reason"],
        "prior": {"P1": "partial (update on the node)", "P2": "no (events in a ledger)", "P3": "yes",
                  "P4": "no (thresholds)", "P5": "no", "P6": "yes (context beside data)", "P7": "partial (live flag)"},
        "scores": {"fr1": (5, "Each node has .models, .metrics, .children; the arm's tip aliases name the servo child; "
                                  "the five-bar's children are its joints.", "medium"),
                   "fr2": (3, "node.observe(streams, labels) per run, but the plays and rungs tables are hand-built "
                                  "over the ledgers.", "medium"),
                   "fr3": (4, "Open move/dwell carried across chunks, stops attached by time; raw pairs kept for lag "
                                  "(O(rows)); 0.71 ms per update.", "medium"),
                   "fr4": (4, "Checks per node with owners, worst-child levels with the reason; thresholds only.",
                           "high"),
                   "fr5": (3, "An alias is one line, but a new part type is a node with signals, context and children.",
                           "medium"),
                   "fr6": (3, "One home, but high in the stack: RobotDx and sim2real reach up into robotparts.",
                           "medium"),
                   "fr7": (3, "Events give each view a t0/t1 for Rerun and a W&B row; views via the adapter.", "low")},
    },
}

WHY = ("Folded values (v4) leads at 84 of 100. It is the only variant that meets the prior-art note's central "
       "recommendation (the state is a value: live is a fold, runs merge with no extra code), and its gates say "
       "what the scorecard's median hides: one alias, 'tip stop error', carries the median gate and Zach's 1 mm on "
       "every stop, judged into RobotDx's own Result, so CAN 11 fails 3 rungs on the every-stop objective and 99 % of "
       "its 84 stops reads 79 of 84. Closest: robot_metrics (v1) at 80, the literal torchmetrics port Zach liked; it "
       "loses points only where its state hides in attributes and its gates are plain thresholds.")
HINGE = ("The lead rests on fr3 and fr4 (honest streaming and tail-aware gates). Weight 'reads like torchmetrics' "
         "instead (v1's add_state and Metric.plot()) or move 10 points to fr1 (the part's adjacency: v2 and v5 score "
         "5 there with medium confidence) and v1 ties v4. Either way the splice keeps v4's value state and gates under "
         "v1's torchmetrics face; the open question is whether Zach wants update(state, chunk) visible or hidden "
         "behind Stream.")


def esc(text: str) -> str:
    return html.escape(text or "", quote=True)


def story_state(index: int) -> str:
    return f"s{min(index, STORY_STEPS)}"


STYLE = """<style>
.api-repl{font:13px/1.45 ui-sans-serif,system-ui,sans-serif;color:var(--ink,#1d232b);padding:14px;display:grid;grid-template-columns:minmax(0,1fr);gap:10px;width:0;min-width:100%;box-sizing:border-box}
.api-repl .repl-head{display:flex;justify-content:space-between;gap:8px;font-size:12px;opacity:.8}
.api-repl ol{list-style:none;margin:0;padding:0;display:grid;grid-template-columns:minmax(0,1fr);gap:6px}
.api-repl .cell{all:unset;box-sizing:border-box;display:grid;grid-template-columns:22px minmax(0,1fr);gap:8px;width:100%;cursor:pointer;border:1px solid color-mix(in srgb,currentColor 18%,transparent);border-radius:8px;padding:7px 9px;background:color-mix(in srgb,currentColor 4%,transparent)}
.api-repl .cell:hover{border-color:var(--accent,#3f7be0)}
.api-repl .cell .n{font-weight:700;color:var(--accent,#3f7be0)}
.api-repl pre{margin:0;white-space:pre-wrap;overflow-wrap:anywhere;font:12px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace}
.api-repl ol,.api-repl li{min-width:0}
.api-repl .out{display:none;white-space:pre;overflow-x:auto;margin:2px 0 4px 30px;padding:8px 10px;border-left:3px solid var(--accent,#3f7be0);background:color-mix(in srgb,var(--accent,#3f7be0) 8%,transparent);border-radius:0 6px 6px 0}
.api-repl .lbl{font-size:11px;text-transform:uppercase;letter-spacing:.04em;opacity:.65;grid-column:2}
.shape-tree{font:12px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;white-space:pre-wrap;overflow-wrap:anywhere;margin:0;padding:12px}
.pa{border-collapse:collapse;font:13px/1.4 ui-sans-serif,system-ui,sans-serif;margin:12px}
.pa td,.pa th{border-bottom:1px solid color-mix(in srgb,currentColor 15%,transparent);padding:5px 10px;text-align:left;vertical-align:top}
.pa .yes{color:#2e7d4f;font-weight:600}.pa .no{color:#b03a2e;font-weight:600}.pa .partial{color:#a6762a;font-weight:600}
.shot{max-width:100%;display:block;margin:8px auto}
</style>"""


def preview(variant_id: str, cells: list[dict]) -> str:
    states, items = [], []
    for index, cell in enumerate(cells, start=1):
        state = story_state(index)
        if state not in states:
            states.append(state)
        output = cell["output"].rstrip()
        if len(output) > 6000:
            output = output[:6000] + "\n... (full output in scenario.out.txt)"
        items.append(f'<li><button type="button" class="cell" data-story-to="{state}" title="Show this step\'s output">'
                     f'<span class="n">{index}</span><pre>{esc(cell["code"].strip())}</pre>'
                     f'<span class="lbl">{esc(cell["label"])} · {cell["seconds"]} s</span></button>'
                     f'<pre class="out" data-out="{state}">{esc(output)}</pre></li>')
    rules = "".join(f'.prototype[data-story-state="{s}"] .api-repl .out[data-out="{s}"]{{display:block}}'
                    f'.prototype[data-story-state="{s}"] .api-repl .cell[data-story-to="{s}"]{{border-color:var(--accent);'
                    f'box-shadow:0 0 0 2px color-mix(in srgb,var(--accent) 25%,transparent)}}' for s in states)
    rules += '.prototype:not([data-story-state]) .api-repl .out[data-out="s1"]{display:block}'
    return (f'{STYLE}<style>{rules}</style><div class="api-repl" style="--accent:{ACCENTS[variant_id]}">'
            f'<div class="repl-head"><b>round 8 · real runs · {esc(VARIANTS[variant_id]["name"])}</b>'
            f'<span>session on 252 real plays, 252 twin answers, 10 five-bar rungs</span></div><ol>{"".join(items)}</ol></div>')


def image(path: pathlib.Path) -> str:
    data = base64.b64encode(path.read_bytes()).decode()
    return f'<img class="shot" alt="{esc(path.name)}" src="data:image/png;base64,{data}">'


def prior_art_table(variant: dict) -> str:
    rows = "".join(f'<tr><td><b>{key}</b> {esc(label)}</td><td class="{variant["prior"][key].split()[0]}">'
                   f'{esc(variant["prior"][key])}</td></tr>' for key, label in PRIOR_ART)
    return f'{STYLE}<table class="pa"><tr><th>prior-art finding (Metric Library APIs note)</th><th>meets?</th></tr>{rows}</table>'


def check_lines() -> dict[str, list[str]]:
    lines, current = {}, None
    for line in (HERE / "check.out.txt").read_text(encoding="utf-8").splitlines():
        match = re.match(r"(PASS|FAIL)\s+(v\d)_", line)
        if match:
            current = match.group(2)
            lines[current] = [line.strip()]
        elif current:
            lines[current].append(line.strip())
    return lines


def main() -> int:
    checks = check_lines()
    variants = []
    for variant_id, meta in VARIANTS.items():
        folder = HERE / FOLDERS[variant_id]
        cells = json.loads((folder / "session.json").read_text(encoding="utf-8"))
        steps = []
        for index, cell in enumerate(cells, start=1):
            state = story_state(index)
            if steps and steps[-1]["state"] == state:
                steps[-1]["label"] += " · " + cell["label"]
                steps[-1]["caption"] += " " + cell["caption"]
            else:
                steps.append({"label": cell["label"], "caption": cell["caption"], "state": state,
                              "target": f'[data-story-to="{state}"]'})
        passed = checks.get(variant_id, ["no check output"])[0].startswith("PASS")
        deviation = next((l for l in checks.get(variant_id, []) if "max |deviation|" in l), "")
        gate_results = {g: {"pass": passed, "evidence": "check_scenario.py: " + checks.get(variant_id, ["?"])[0]}
                        for g, *_ in GATES}
        gate_results["g2"]["evidence"] = {
            "v4": "Metric is a frozen config with empty/update/merge/compute; Stream gives update/compute/reset; "
                  "Collection; __call__ is the functional.",
            "v5": "Metric.update/value/reset when used alone; in the tree the node's Segmenter holds the shared state; "
                  "Part is the collection; functional() per metric.",
        }.get(variant_id, "Metric base, constructor config, update/compute/reset, a collection type, functional twins.")
        variants.append({
            "id": variant_id, "name": meta["name"], "thesis": meta["thesis"], "accent": ACCENTS[variant_id],
            "bestWhen": meta["best"], "losesWhen": meta["loses"], "decisions": meta["decisions"],
            "keepParts": meta["keep"],
            "proof": [checks.get(variant_id, ["?"])[0], deviation,
                      f"Scenario run: docs/proposals/metrics-api-2026-11-17/{FOLDERS[variant_id]}/scenario.out.txt",
                      "Simulated: the sealed friction ladder (amendment 4) only; every pendulum and five-bar number "
                      "is a real or twin run file"],
            "scores": {fr: {"score": s, "evidence": e, "confidence": c} for fr, (s, e, c) in meta["scores"].items()},
            "gateResults": gate_results,
            "previewLabel": "the session that ran · click a cell or step",
            "preview": preview(variant_id, cells),
            "story": {"title": "Walk the session", "steps": steps},
            "media": [
                {"label": "A standard view", "caption": "Metric.plot() / the shared adapter on every CAN 11 stop at the "
                 "tip, with Zach's 1 mm; the same points went to a W&B payload and an .rrd.",
                 "html": image(MEDIA / f"{variant_id}-tip-stops.png")},
                {"label": "Where it lives", "caption": "The dependency direction and the run-data type it follows.",
                 "html": f'{STYLE}<pre class="shape-tree">{esc(meta["where"])}</pre>'},
                {"label": "RobotDx and sim2real", "caption": "What happens to the three copies of the formulas today.",
                 "html": f'{STYLE}<pre class="shape-tree">{esc(meta["fate"])}</pre>'},
                {"label": "Prior-art check", "caption": "Which of the prior-art note's findings this variant meets "
                 "(amendment 5: a check, not a reweighting).", "html": prior_art_table(meta)},
            ],
        })
    def total(v):
        return sum(w * v["scores"][fr]["score"] / 5 for fr, _, w, *_ in REQUIREMENTS)
    ranked = sorted(((total(v), v["id"]) for v in variants if all(g["pass"] for g in v["gateResults"].values())),
                    reverse=True)
    spec = {
        "schemaVersion": 3,
        "title": "Five metrics APIs, on round 8's real runs",
        "kicker": "Babble & Prune · /babble 5 · robot_models · 2026-11-17",
        "brief": ("Five runnable APIs for the performance metrics per part type (Zach's note of 2026-11-17), each "
                  "torchmetrics-shaped with numpy inside, math-only base metrics named by aliases, gates outside "
                  "compute(), live = offline, and each one matching round 8's scorecard on 252 real plays."),
        "count": len(variants), "defaultId": ranked[0][1], "defaultWhy": WHY, "decisionHinge": HINGE,
        "invariants": ["robot_msgs streams in; SI inside, degrees and mm shown",
                       "the generic rows are robot_msgs.compare's; the stop rule one shared per-sample kernel",
                       "one dataset: round 8's 252 real plays, the twin's 252 answers, the five-bar's 10 rungs",
                       "every variant passes the frozen parity check before it is scored"],
        "boundary": ("Prototypes in docs/proposals/metrics-api-2026-11-17/ that import robot_msgs and robot_models; "
                     "nothing in src/, robot_msgs, sim2real, io-diagnostics, robotparts or bam_ws changed. W&B "
                     "payloads are built, not sent; Rerun .rrd files are written locally."),
        "axes": [
            {"name": "Where the metrics live",
             "values": ["robot_metrics package (V1)", "on the ModelFamily (V2)", "robot_msgs + part catalog (V3)",
                        "robot_metrics beside RobotDx (V4)", "robotparts' part tree (V5)"]},
            {"name": "Streaming state",
             "values": ["add_state attributes (V1)", "cat buffer + functional (V2)", "raw topics + live aligner (V3)",
                        "an immutable value, folded (V4)", "move / dwell events (V5)"]},
            {"name": "Gates", "values": ["Diagnostic(alias, Limits) (V1)", "Gate(alias, robot_msgs Value) (V2)",
                                         "catalog Check rows (V3)", "OpenSLO occurrence objectives (V4)",
                                         "per-node levels, worst child (V5)"]},
            {"name": "Across runs", "values": ["merge_state + PerRun (V1)", "one object fed the runs (V2)",
                                               "per-alias rule + bootstrap CI (V3)", "strata, tails, Wilson (V4)",
                                               "node ledgers grouped (V5)"]},
        ],
        "requirements": [{"id": fr, "name": name, "weight": weight, "why": why, "passCondition": passing,
                          "anchors": {"1": a1, "3": a3, "5": a5}}
                         for fr, name, weight, why, passing, (a1, a3, a5) in REQUIREMENTS],
        "hardGates": [{"id": g, "name": n, "why": w} for g, n, w in GATES],
        "variants": variants,
    }
    spec_path = HERE / "gallery-spec.json"
    spec_path.write_text(json.dumps(spec, indent=1, ensure_ascii=False), encoding="utf-8")
    if OUT.exists():
        OUT.unlink()  # WHY: gallery.py refuses to overwrite; this script owns the output
    subprocess.run([sys.executable, str(GALLERY), "build", "--spec", str(spec_path), "--output", str(OUT)], check=True)
    subprocess.run([sys.executable, str(GALLERY), "check", "--input", str(OUT), "--strict"], check=True)
    for score, variant_id in ranked:
        print(f"{variant_id} {score:.1f}")
    print(OUT, f"{OUT.stat().st_size / 1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
