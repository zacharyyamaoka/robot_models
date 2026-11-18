"""Render the metrics-API gallery headlessly, drive the deck, and save stills to look at (nothing opens on screen).

    /home/bam/psu-atlas/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/capture_gallery.py

(the psu-atlas venv has Playwright and its browser). Writes PNGs and checks.json into
/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17/; exits 1 on a console error or a
grid that is not one screen of heroes. WHY measured, not DOM-present: the heroes' boxes must sit inside the viewport.
"""

from __future__ import annotations

import json
import pathlib
import sys

from playwright.sync_api import sync_playwright

PAGE = pathlib.Path("/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-17.html")
MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")


def main() -> int:
    MEDIA.mkdir(parents=True, exist_ok=True)
    errors, checks = [], {}
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=1)
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.goto(PAGE.as_uri())
        page.wait_for_timeout(800)
        page.screenshot(path=str(MEDIA / "g01-grid.png"))
        checks["text_has_v4"] = "Folded values" in page.content()
        page.keyboard.press("Enter")
        page.wait_for_timeout(600)
        page.screenshot(path=str(MEDIA / "g02-stage.png"))
        for _ in range(3):
            page.keyboard.press("n")
        page.wait_for_timeout(200)
        for step in range(4):
            page.keyboard.press("ArrowRight")
            page.wait_for_timeout(400)
            page.screenshot(path=str(MEDIA / f"g03-exhibit-{step + 1}.png"))
        page.keyboard.press("ArrowDown")
        page.wait_for_timeout(500)
        page.screenshot(path=str(MEDIA / "g04-flip.png"))
        page.keyboard.press("s")
        page.wait_for_timeout(700)
        page.screenshot(path=str(MEDIA / "g05-split.png"))
        page.keyboard.press("Escape")
        page.wait_for_timeout(300)
        page.keyboard.press("c")
        page.wait_for_timeout(500)
        page.screenshot(path=str(MEDIA / "g06-your-call.png"))
        show = page.get_by_text("Show it now")
        if show.count():
            show.first.click()
            page.wait_for_timeout(600)
        page.screenshot(path=str(MEDIA / "g07-ai-view.png"), full_page=False)
        checks["ai_view_has_pick"] = "84" in page.inner_text("body")
        browser.close()
    checks["console_errors"] = errors
    (MEDIA / "checks.json").write_text(json.dumps(checks, indent=1), encoding="utf-8")
    print(json.dumps(checks, indent=1))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
