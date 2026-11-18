"""Render the metrics-API gallery headlessly, drive the deck, and save stills to look at (nothing opens on screen).

    /home/bam/psu-atlas/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-17/capture_gallery.py

(the psu-atlas venv has Playwright and its browser). Writes PNGs and checks.json into
/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17/; exits 1 on a console error, a
page wider than the viewport, or a code block wider than its box on the full-width stage. WHY measured, not
DOM-present: Zach reads the boxes, so the boxes' widths are what is checked.

Stills: g01 the grid (five glances), g02-glance-vN each variant's API at a glance on the stage (-full: the whole
exhibit), g03-session-v1 / -v4
the session exhibit (top, then scrolled to the live-vs-offline cell), g04 v1 and v4 side by side on the glance,
g05 the split's flip, g06 "Your call", g07 the AI view.
"""

from __future__ import annotations

import json
import pathlib

from playwright.sync_api import Page, sync_playwright

PAGE = pathlib.Path("/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-17.html")
MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/robot-models-metrics-api-proposals-2026-11-17")
VARIANTS = ("v1", "v2", "v3", "v4", "v5")

# Every rendered exhibit lives in a shadow root; measure the ones on the stage (or split panes), not the filmstrip's
# posters, which reflow to a thumbnail's width.
MEASURE = """() => {
  const roots = [];
  document.querySelectorAll('*').forEach(el => {
    if (el.shadowRoot && !el.closest('.film-inner') && !el.classList.contains('poster')) roots.push(el.shadowRoot);
  });
  const visible = el => { const box = el.getBoundingClientRect(); return box.width > 0 && box.height > 0; };
  const blocks = kind => roots.flatMap(root => [...root.querySelectorAll('.api-repl pre.' + kind)]).filter(visible);
  const overflowing = list => list.filter(el => el.scrollWidth > el.clientWidth + 1)
                                  .map(el => el.textContent.split('\\n').sort((a, b) => b.length - a.length)[0].slice(0, 120));
  const code = blocks('code'), out = blocks('out');
  return {page_scroll_width: document.documentElement.scrollWidth, viewport: innerWidth,
          code_blocks: code.length, code_overflowing: overflowing(code),
          output_blocks: out.length, output_scrolling: overflowing(out).length,
          highlighted_spans: roots.flatMap(root => [...root.querySelectorAll('.api-repl pre.code span')]).length};
}"""


SCROLL_STAGE_TO_TOP = """() => {
  document.querySelectorAll('#stage, #stage *').forEach(el => { if (el.scrollTop) el.scrollTop = 0; });
}"""
SCROLL_TO_LIVE_CELL = """() => {
  for (const el of document.querySelectorAll('#stage *')) {
    const head = el.shadowRoot && [...el.shadowRoot.querySelectorAll('.cell-head')]
      .find(h => h.textContent.includes('offline and live'));
    if (head) { head.scrollIntoView({block: 'start'}); return true; }
  }
  return false;
}"""


def show_variant(page: Page, variant_id: str) -> None:
    """Flip the stage to one variant by its filmstrip thumbnail (the deck shuffles the order per load)."""

    label = page.locator(".th-lab", has_text=variant_id.upper())
    page.locator(".th-cell", has=label).first.click()
    page.wait_for_timeout(500)


def main() -> int:
    MEDIA.mkdir(parents=True, exist_ok=True)
    errors, checks = [], {}
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=1)
        page.on("console", lambda message: errors.append(message.text) if message.type == "error" else None)
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto(PAGE.as_uri())
        page.wait_for_timeout(900)
        page.screenshot(path=str(MEDIA / "g01-grid.png"))
        checks["text_has_v4"] = "Folded values" in page.content()

        page.keyboard.press("Enter")
        page.wait_for_timeout(700)
        checks["glance"] = {}
        for variant_id in VARIANTS:
            show_variant(page, variant_id)
            page.screenshot(path=str(MEDIA / f"g02-glance-{variant_id}.png"))
            checks["glance"][variant_id] = page.evaluate(MEASURE)
            # WHY a taller viewport, not an element shot: the stage clips its exhibit to the window.
            page.set_viewport_size({"width": 1440, "height": 1800})
            page.wait_for_timeout(500)
            page.screenshot(path=str(MEDIA / f"g02-glance-{variant_id}-full.png"))
            page.set_viewport_size({"width": 1440, "height": 900})
            page.wait_for_timeout(400)

        checks["session"] = {}
        for variant_id in ("v1", "v4"):
            show_variant(page, variant_id)
            page.keyboard.press("ArrowRight")
            page.wait_for_timeout(600)
            page.evaluate(SCROLL_STAGE_TO_TOP)
            page.wait_for_timeout(300)
            page.screenshot(path=str(MEDIA / f"g03-session-{variant_id}.png"))
            checks["session"][variant_id] = page.evaluate(MEASURE)
            page.evaluate(SCROLL_TO_LIVE_CELL)
            page.wait_for_timeout(400)
            page.screenshot(path=str(MEDIA / f"g03-session-{variant_id}-scrolled.png"))
            page.keyboard.press("ArrowLeft")
            page.wait_for_timeout(500)

        show_variant(page, "v1")
        page.keyboard.press("s")
        page.wait_for_timeout(700)
        show_variant(page, "v4")
        page.wait_for_timeout(500)
        page.screenshot(path=str(MEDIA / "g04-split-v1-v4.png"))
        checks["split"] = page.evaluate(MEASURE)
        page.keyboard.press("ArrowDown")
        page.wait_for_timeout(500)
        page.screenshot(path=str(MEDIA / "g05-flip.png"))
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
    measured = [*checks["glance"].values(), *checks["session"].values()]
    checks["page_wider_than_viewport"] = any(m["page_scroll_width"] > m["viewport"] for m in measured)
    checks["code_wider_than_box"] = sorted({line for m in measured for line in m["code_overflowing"]})
    (MEDIA / "checks.json").write_text(json.dumps(checks, indent=1, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(checks, indent=1, ensure_ascii=False))
    return 1 if errors or checks["page_wider_than_viewport"] or checks["code_wider_than_box"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
