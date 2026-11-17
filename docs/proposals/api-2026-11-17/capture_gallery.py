"""Render the API-proposals gallery headlessly, drive it, measure that it works, and save stills to look at.

    /home/bam/psu-atlas/.venv/bin/python /home/bam/robot_models/docs/proposals/api-2026-11-17/capture_gallery.py

(the psu-atlas venv has Playwright and its browser). Nothing opens on screen. Writes PNGs and checks.json into
/home/bam/robot_models/reports/media/robot-models-api-proposals-2026-11-17/ and exits 1 if a measured check fails.

WHY measured, not DOM-present (dispatch discipline): a walkthrough is proved by which output is actually visible
(its box has height) after Next, Back and a direct click on a cell, and the grid by its cards' real x positions.
"""

from __future__ import annotations

import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

PAGE = pathlib.Path("/home/bam/robot_models/reports/robot-models-api-proposals-2026-11-17.html")
MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-api-proposals-2026-11-17")

VISIBLE_STATE = """card => {
  const proto = card.querySelector('.prototype');
  const shown = [...card.querySelectorAll('.api-repl .out')].filter(o => o.getBoundingClientRect().height > 0)
                 .map(o => o.dataset.out);
  return {state: proto.dataset.storyState || null, shown: [...new Set(shown)],
          label: card.querySelector('[data-story-label]').textContent.trim()};
}"""


def main() -> int:
    MEDIA.mkdir(parents=True, exist_ok=True)
    checks, failures, console_errors = {}, [], []
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=1)
        page.on("console", lambda m: console_errors.append(m.text) if m.type == "error" else None)
        page.on("pageerror", lambda e: console_errors.append(str(e)))
        page.goto(PAGE.as_uri())
        page.wait_for_timeout(400)
        page.screenshot(path=str(MEDIA / "01-top.png"))

        lefts = page.eval_on_selector_all(".variant-card", "cs => cs.map(c => Math.round(c.getBoundingClientRect().left))")
        checks["grid_columns"] = len(set(lefts))
        if checks["grid_columns"] < 2:
            failures.append(f"grid shows {checks['grid_columns']} column(s)")
        page.locator(".variant-card").first.scroll_into_view_if_needed()
        page.screenshot(path=str(MEDIA / "02-grid.png"))

        per_variant = {}
        for vid in ("v1", "v2", "v3", "v4", "v5"):
            card = page.locator(f'.variant-card[data-variant="{vid}"]')
            card.locator("[data-grid-focus-target]").first.click()
            page.wait_for_timeout(250)
            card = page.locator(f'.variant-card[data-variant="{vid}"]')
            handle = card.element_handle()
            seen = [page.evaluate(VISIBLE_STATE, handle)]
            steps = card.locator("[data-story-index]").count()
            for _ in range(steps - 1):
                card.locator('[data-story-action="next"]').click()
                page.wait_for_timeout(120)
                seen.append(page.evaluate(VISIBLE_STATE, handle))
            card.locator('[data-story-action="back"]').click()
            page.wait_for_timeout(120)
            after_back = page.evaluate(VISIBLE_STATE, handle)
            card.locator('.api-repl .cell[data-story-to="s2"]').first.click()
            page.wait_for_timeout(120)
            after_cell = page.evaluate(VISIBLE_STATE, handle)
            per_variant[vid] = {"steps": steps, "seen": seen, "after_back": after_back, "after_cell_s2": after_cell}
            for index, state in enumerate(seen, start=1):
                if state["state"] != f"s{index}" or state["shown"] != [f"s{index}"]:
                    failures.append(f"{vid}: after {index - 1} Next, state {state['state']} shows {state['shown']}")
            if after_back["state"] != f"s{steps - 1}" or after_back["shown"] != [f"s{steps - 1}"]:
                failures.append(f"{vid}: Back left {after_back}")
            if after_cell["state"] != "s2" or after_cell["shown"] != ["s2"] or after_cell["label"] != seen[1]["label"]:
                failures.append(f"{vid}: clicking cell 2 left {after_cell}")
            if vid in ("v1", "v4", "v5"):
                card.locator('.api-repl .cell[data-story-to="s5"]').first.click()
                page.wait_for_timeout(150)
                card.screenshot(path=str(MEDIA / f"03-{vid}-focus-step5.png"))
            page.locator("[data-layout='grid'], [data-view='grid'], button:has-text('Grid')").first.click()
            page.wait_for_timeout(200)
        checks["walkthrough"] = per_variant

        prune = page.locator("#score-matrix")
        prune.scroll_into_view_if_needed()
        page.wait_for_timeout(200)
        page.screenshot(path=str(MEDIA / "04-prune.png"), full_page=False)
        page.screenshot(path=str(MEDIA / "05-full.png"), full_page=True)
        browser.close()
    checks["console_errors"] = console_errors
    checks["failures"] = failures
    if console_errors:
        failures.append(f"console errors: {console_errors[:3]}")
    (MEDIA / "checks.json").write_text(json.dumps(checks, indent=1, ensure_ascii=False))
    print(f"grid columns {checks['grid_columns']}; "
          + "; ".join(f"{v} {d['steps']} steps" for v, d in per_variant.items()))
    print("FAIL\n  " + "\n  ".join(failures) if failures else "PASS: every step, Back and a direct cell click show "
          "exactly their own output; no console errors")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
