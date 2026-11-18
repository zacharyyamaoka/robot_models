"""Render round 2's metrics-API gallery headlessly, measure it, and save stills to look at (nothing opens on screen).

    /home/bam/psu-atlas/.venv/bin/python /home/bam/robot_models/docs/proposals/metrics-api-2026-11-18/capture_gallery.py [--page HTML] [--prefix NAME]

(the psu-atlas venv has Playwright and its Chromium). One browser, closed at the end; viewports 1440 x 900 and
390 x 844 at scale 1. Writes PNGs and checks.json into /home/bam/robot_models/reports/media/metrics-api-2026-11-18/:
- top-1440.png: the glances, five side by side (clipped to the glance section);
- top-390.png: the same top at phone width, stacked (from the page top to the end of the first glance);
- split-1440.png: Split, baseline vs r2-3; scoreboard-1440.png; variant-1440.png: the first variant's section top.
Exits 1 on a console error, a page wider than its viewport, or a code block wider than its box. WHY measured, not
DOM-present: Zach reads the boxes, so their widths and the rows' tops are what is checked.
"""

from __future__ import annotations

import argparse
import json
import pathlib

from playwright.sync_api import Page, sync_playwright

PAGE = pathlib.Path("/home/bam/robot_models/reports/robot-models-metrics-api-proposals-2026-11-18.html")
MEDIA = pathlib.Path("/home/bam/robot_models/reports/media/metrics-api-2026-11-18")
MAX_LINE = 90

MEASURE = """(maxLine) => {
  const visible = el => { const box = el.getBoundingClientRect(); return box.width > 0 && box.height > 0; };
  const code = [...document.querySelectorAll('pre.code')].filter(visible);
  const out = [...document.querySelectorAll('pre.out')].filter(visible);
  const longest = el => el.textContent.split('\\n').sort((a, b) => b.length - a.length)[0].slice(0, 120);
  const overflowing = list => list.filter(el => el.scrollWidth > el.clientWidth + 1);
  const columns = [...document.querySelectorAll('#glances .gcol')].filter(visible).map(column => {
    const box = column.getBoundingClientRect();
    const groups = [...column.querySelectorAll(':scope > .group')].map(g => Math.round(g.getBoundingClientRect().top));
    const terms = column.querySelector(':scope > .terms');
    return {slot: column.dataset.slot, x: Math.round(box.x), width: Math.round(box.width),
            height: Math.round(box.height), group_tops: groups,
            terms_top: terms ? Math.round(terms.getBoundingClientRect().top) : null};
  });
  const first = columns[0];
  const sameRow = columns.length > 1 && first.group_tops.length > 0 &&
    columns.every(c => c.group_tops.length === first.group_tops.length) &&
    first.group_tops.every((top, index) => columns.every(c => Math.abs(c.group_tops[index] - top) <= 1));
  const codeLines = code.flatMap(el => el.textContent.split('\\n'));
  return {viewport: innerWidth, page_scroll_width: document.documentElement.scrollWidth,
          code_blocks: code.length, code_overflowing: overflowing(code).map(longest),
          output_blocks: out.length, output_scrolling: overflowing(out).length,
          highlighted_spans: document.querySelectorAll('pre.code span[class]').length,
          code_lines_over_max: codeLines.filter(line => line.length > maxLine).length,
          glance_columns: columns, glance_rows_aligned: sameRow,
          not_built: document.querySelectorAll('.placeholder').length};
}"""

GLANCE_BOTTOM = "() => document.getElementById('glance-section').getBoundingClientRect().bottom + scrollY"
FIRST_GLANCE_BOTTOM = "() => document.querySelector('#glances .gcol').getBoundingClientRect().bottom + scrollY"


def shoot(page: Page, path: pathlib.Path, top: float, bottom: float) -> None:
    width = page.viewport_size["width"]
    page.screenshot(path=str(path), full_page=True, clip={"x": 0, "y": top, "width": width, "height": bottom - top})


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--page", default=str(PAGE))
    parser.add_argument("--prefix", default="", help="prefix for the PNG names (e.g. 'empty-')")
    args = parser.parse_args(argv)
    page_path = pathlib.Path(args.page).resolve()
    MEDIA.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    checks: dict = {"page": str(page_path), "bytes": page_path.stat().st_size}
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        try:
            for width, height in ((1440, 900), (390, 844)):
                context = browser.new_context(viewport={"width": width, "height": height}, device_scale_factor=1)
                page = context.new_page()
                page.on("console", lambda message: errors.append(message.text) if message.type == "error" else None)
                page.on("pageerror", lambda error: errors.append(str(error)))
                page.goto(page_path.as_uri())
                page.wait_for_timeout(400)
                checks[f"all_{width}"] = page.evaluate(MEASURE, MAX_LINE)
                bottom = page.evaluate(GLANCE_BOTTOM if width > 640 else FIRST_GLANCE_BOTTOM)
                shoot(page, MEDIA / f"{args.prefix}top-{width}.png", 0, min(bottom + 12, 4000))
                if width == 1440:
                    page.click('.seg button[data-mode="split"]')
                    page.select_option("#pick-a", "baseline")
                    page.select_option("#pick-b", "r2-3")
                    page.wait_for_timeout(200)
                    checks["split_1440"] = page.evaluate(MEASURE, MAX_LINE)
                    section_top = page.evaluate("() => document.getElementById('glance-section')"
                                                ".getBoundingClientRect().top + scrollY")
                    shoot(page, MEDIA / f"{args.prefix}split-1440.png", section_top - 8,
                          min(page.evaluate(GLANCE_BOTTOM) + 12, section_top + 3000))
                    page.click('.seg button[data-mode="all"]')
                    page.locator("#scoreboard").screenshot(path=str(MEDIA / f"{args.prefix}scoreboard-1440.png"))
                    variant_top = page.evaluate("() => document.querySelector('.variant').getBoundingClientRect()"
                                                ".top + scrollY")
                    shoot(page, MEDIA / f"{args.prefix}variant-1440.png", variant_top - 8, variant_top + 1400)
                context.close()
        finally:
            browser.close()
    checks["console_errors"] = errors
    measured = [checks["all_1440"], checks["all_390"], checks["split_1440"]]
    checks["page_wider_than_viewport"] = any(m["page_scroll_width"] > m["viewport"] for m in measured)
    checks["code_wider_than_box"] = sorted({line for m in measured for line in m["code_overflowing"]})
    (MEDIA / f"{args.prefix}checks.json").write_text(json.dumps(checks, indent=1, ensure_ascii=False),
                                                     encoding="utf-8")
    summary = {key: checks[key] for key in ("bytes", "console_errors", "page_wider_than_viewport",
                                            "code_wider_than_box")}
    for key in ("all_1440", "all_390", "split_1440"):
        measure = checks[key]
        summary[key] = {name: measure[name] for name in ("viewport", "page_scroll_width", "code_blocks",
                                                         "output_blocks", "output_scrolling", "highlighted_spans",
                                                         "code_lines_over_max", "glance_rows_aligned", "not_built")}
        summary[key]["columns"] = [(c["slot"], c["x"], c["width"]) for c in measure["glance_columns"]]
    print(json.dumps(summary, indent=1, ensure_ascii=False))
    failed = errors or checks["page_wider_than_viewport"] or checks["code_wider_than_box"]
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
