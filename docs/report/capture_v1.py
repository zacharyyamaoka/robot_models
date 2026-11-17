"""Capture the media for the robot_models v1 review report: the real public GitHub pages, recorded in headless Chromium.

    /home/bam/psu-atlas/.venv/bin/python docs/report/capture_v1.py <media dir> [--repo https://github.com/zacharyyamaoka/robot_models] [--branch main] [--ffmpeg /usr/bin/ffmpeg]

(the psu-atlas venv has Playwright and its Chromium; the defaults are the public repo, its main branch and the system
ffmpeg, with ffprobe beside it). Nothing opens on screen: the browser is headless.

What it writes into <media dir>:
  hero.mp4, hero.gif     the primary reading flow, recorded from github.com at 1280x800: the repo landing (README) →
                         docs/ → TYPES.md (the 12 types) → integrated_servo's README (its mermaid ladder, then the rung
                         table) → the M3.stribeck rung (its README, its folder, model.py). Every step is a real click or
                         wheel; the pointer is an injected dot.
  still-*.jpg            the servo and bldc_motor mermaid ladders, the conveyor rung table, TYPES.md's table, the stribeck
                         rung folder: light theme, each clipped to its element, each <= 200 KB. The two wide tables
                         are GitHub's horizontal scrollers shown unclipped (their overflow set to visible, same layout)
  checks.json            what was measured: the hero's kept intervals, duration and sizes; each still's page, element
                         box and bytes; each page's HTTP status and title; whether GitHub blocked anything
  --workdir DIR          keeps the raw webm there (for re-encoding); by default it goes to a temp dir and is deleted

The terminal captures beside these (term-*.txt) are not made here; docs/report/api_session.py makes term-d.

WHY recorded, not animated: the video is what a person sees reading the public repo, so it is captured from the actual
pages (Zach, 2026-11-16: a hero video of the primary flow, never fabricated animation). The only edit is cutting out
page-load waits, each at a navigation the viewer has just seen clicked; checks.json lists every kept interval.
WHY the script drives the dot, not a mousemove listener: GitHub draws mermaid in a cross-origin iframe that swallows
mousemove, so the script moves the real pointer and the dot together, step by step: the dot is where the pointer is.
WHY no local fallback here: when GitHub blocks or rate-limits the headless browser, this exits and says so rather than
quietly rendering something else under the same file names.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import shutil
import subprocess
import tempfile
import time

from playwright.sync_api import sync_playwright

VIEWPORT = {"width": 1280, "height": 800}
FPS = 25
MP4_FPS = 15
STILL_LIMIT = 200_000
BLOCKED = ("rate limit", "too many requests", "page not found", "access has been restricted", "is disabled")

# the dot lives on <html>, not <body>: GitHub's soft navigations can swap <body>; __dot re-attaches it when they do
CURSOR_DOT = """
if (window.top === window) {
  window.__dot = (x, y) => {
    let dot = document.getElementById('__cursor_dot');
    if (!dot || !dot.isConnected) {
      dot = document.createElement('div');
      dot.id = '__cursor_dot';
      dot.style.cssText = 'position:fixed;left:0;top:0;width:16px;height:16px;margin:-8px 0 0 -8px;border-radius:50%;' +
        'background:rgba(30,100,230,.85);border:2px solid #fff;box-shadow:0 0 0 1px rgba(0,0,0,.45),0 2px 6px rgba(0,0,0,.35);' +
        'pointer-events:none;z-index:2147483647';
      document.documentElement.appendChild(dot);
    }
    dot.style.left = x + 'px';
    dot.style.top = y + 'px';
  };
}
"""


class Recorder:
    """The page, the pointer and the clock: moves the pointer and its dot together, and keeps the intervals to show."""

    def __init__(self, page, created: float):
        self.page, self.created = page, created
        self.x, self.y = 640.0, 400.0
        self.kept: list[list[float]] = []
        self.pages: list[dict] = []

    def now(self) -> float:
        return time.monotonic() - self.created

    def keep(self) -> None:
        self.kept.append([round(self.now(), 2), None])

    def cut(self) -> None:
        self.kept[-1][1] = round(self.now(), 2)

    def dot(self) -> None:
        self.page.evaluate("([x, y]) => window.__dot && window.__dot(x, y)", [self.x, self.y])

    def pause(self, seconds: float) -> None:
        self.page.wait_for_timeout(int(seconds * 1000))

    def glide(self, x: float, y: float, seconds: float = 0.6) -> None:
        steps = max(2, int(seconds * 30))
        start_x, start_y, began = self.x, self.y, time.monotonic()
        for step in range(1, steps + 1):
            fraction = step / steps
            eased = fraction * fraction * (3 - 2 * fraction)
            self.x, self.y = start_x + (x - start_x) * eased, start_y + (y - start_y) * eased
            self.page.mouse.move(self.x, self.y)
            self.dot()
            lag = began + seconds * fraction - time.monotonic()
            if lag > 0:
                time.sleep(lag)

    def wheel(self, dy: float, seconds: float, dx: float = 0.0) -> None:
        steps = max(2, int(seconds * 30))
        began = time.monotonic()
        for step in range(1, steps + 1):
            self.page.mouse.wheel(dx / steps, dy / steps)
            lag = began + seconds * step / steps - time.monotonic()
            if lag > 0:
                time.sleep(lag)

    def to(self, locator, seconds: float = 0.6, dx: float = 0.5) -> None:
        box = locator.bounding_box()
        self.glide(box["x"] + box["width"] * dx, box["y"] + box["height"] / 2, seconds)

    def click(self, url_part: str, ready: str, mermaid: bool = False) -> None:
        """Click where the pointer is, then cut the load: the next interval starts once `ready` shows on the new page."""
        self.page.mouse.down()
        self.page.mouse.up()
        # WHY 0.1 s: a soft navigation can paint the new page at the old scroll offset ~0.2 s after the click; the cut
        # lands before that flash
        self.pause(0.1)
        self.cut()
        self.page.wait_for_url(f"**{url_part}", timeout=30000)
        self.settle(ready, mermaid)
        self.keep()

    def settle(self, ready: str, mermaid: bool = False) -> None:
        self.page.locator(ready).filter(visible=True).first.wait_for(timeout=30000)
        if mermaid:
            self.page.frame_locator("iframe.render-viewer").first.locator("svg").first.wait_for(timeout=30000)
        try:
            self.page.wait_for_load_state("networkidle", timeout=6000)
        except Exception:
            pass                        # GitHub may hold a connection open; the selector above is the real gate
        self.pause(0.3)
        title = self.page.title()
        self.pages.append({"url": self.page.url, "title": title, "at_s": round(self.now(), 2)})
        if any(word in title.lower() for word in BLOCKED):
            raise SystemExit(f"GitHub blocked the headless browser at {self.page.url}: {title!r}")
        self.dot()


def record_hero(browser, repo: str, branch: str, workdir: pathlib.Path) -> tuple[pathlib.Path, Recorder]:
    path = repo.removeprefix("https://github.com")
    blob, tree = f"{path}/blob/{branch}", f"{path}/tree/{branch}"
    context = browser.new_context(viewport=VIEWPORT, color_scheme="light", record_video_dir=str(workdir),
                                  record_video_size=VIEWPORT)
    context.add_init_script(CURSOR_DOT)
    created = time.monotonic()
    page = context.new_page()
    rec = Recorder(page, created)
    response = page.goto(repo, wait_until="domcontentloaded")
    if response is None or response.status != 200:
        raise SystemExit(f"GitHub answered {response and response.status} for {repo}")
    rec.settle("article.markdown-body")
    rec.keep()

    # 1. the landing: its file list, then down to the README and back up to docs/
    rec.pause(0.4)
    rec.glide(700, 470, 0.4)
    rec.wheel(700, 0.7)
    rec.pause(1.0)
    rec.wheel(-700, 0.5)
    rec.to(page.locator(f"a[href='{tree}/docs']").filter(visible=True).first, 0.5)
    rec.click(f"{tree}/docs", f"a[href='{blob}/docs/TYPES.md']")

    # 2. docs/ → TYPES.md
    rec.to(page.locator(f"a[href='{blob}/docs/TYPES.md']").filter(visible=True).first, 0.5)
    rec.click(f"{blob}/docs/TYPES.md", "article.markdown-body table")

    # 3. TYPES.md: down to its table, along the type column, into integrated_servo
    table = page.locator("article.markdown-body table").first
    rec.glide(1000, 500, 0.3)
    rec.wheel(table.bounding_box()["y"] - 140, 0.6)
    links = table.locator("tbody tr td:first-child a")
    for row in (5, 2):
        rec.to(links.nth(row), 0.35)
        rec.pause(0.15)
    servo = table.locator(f"a[href='{blob}/src/robot_models/actuators/servo/README.md']").first
    rec.to(servo, 0.4)
    rec.click(f"{blob}/src/robot_models/actuators/servo/README.md", "article.markdown-body", mermaid=True)

    # 4. the servo README: its mermaid ladder (scrolled through), then the rung table, across to the stribeck code link
    diagram = page.locator("section.js-render-needs-enrichment").first
    rec.glide(1255, 420, 0.3)
    rec.wheel(diagram.bounding_box()["y"] - 120, 0.7)
    svg = page.frame_locator("iframe.render-viewer").first.locator("svg").first
    box = svg.bounding_box()
    rec.glide(box["x"] + box["width"] / 2, 450, 0.4)
    rec.pause(0.3)
    rec.glide(1255, 450, 0.3)
    rec.wheel(box["height"] - 560, 1.0)
    rec.pause(0.3)
    rungs = page.locator("article.markdown-body table").first
    rec.wheel(rungs.bounding_box()["y"] - 150, 0.6)
    stribeck_row = rungs.locator("tbody tr").nth(3)
    rec.to(stribeck_row.locator("td").first, 0.4)
    rec.pause(0.3)
    code = rungs.locator("a[href$='rungs/stribeck/README.md']").first
    shift = code.bounding_box()["x"] - 900
    rec.wheel(0, 0.6, dx=shift)
    rec.to(code, 0.4)
    rec.click(f"{blob}/src/robot_models/actuators/rungs/stribeck/README.md", "article.markdown-body")

    # 5. the rung: its README, its folder, model.py
    rec.pause(0.6)
    crumb = page.locator(f"a[href='{tree}/src/robot_models/actuators/rungs/stribeck']").filter(visible=True).first
    rec.to(crumb, 0.5)
    model_py = f"{blob}/src/robot_models/actuators/rungs/stribeck/model.py"
    rec.click(f"{tree}/src/robot_models/actuators/rungs/stribeck", f"a[href='{model_py}']")
    rec.pause(0.5)
    rec.to(page.locator(f"a[href='{model_py}']").filter(visible=True).first, 0.5)
    rec.click(model_py, ".react-code-lines")
    rec.pause(0.8)
    rec.glide(1000, 500, 0.3)
    rec.wheel(420, 0.6)
    rec.pause(0.7)
    rec.cut()

    video = pathlib.Path(page.video.path())
    rec.closed_s = round(rec.now(), 2)
    context.close()                     # the video is finished on close
    return video, rec


def probe_seconds(path: pathlib.Path, ffmpeg: str) -> float:
    ffprobe = str(pathlib.Path(ffmpeg).with_name("ffprobe"))
    probe = subprocess.run([ffprobe, "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)],
                           capture_output=True, text=True, check=True)
    return float(probe.stdout)


def encode(webm: pathlib.Path, kept: list[list[float]], closed_s: float, media: pathlib.Path, ffmpeg: str) -> dict:
    """Join the kept intervals; H.264 mp4 at 960 px and a small GIF, each re-encoded smaller until under 900 KB.

    WHY the offset (measured 2026-11-17: the webm ran 24.48 s for 25.08 s of script clock): the recording starts with
    the page's first frame, ~0.6 s after the clock does, and ends at close; so video time = script time - (closed_s -
    the webm's duration). Unshifted, every kept interval ran 0.6 s late and opened on the next page's first paint.
    """
    mp4, gif = media / "hero.mp4", media / "hero.gif"
    offset = max(0.0, closed_s - probe_seconds(webm, ffmpeg))
    kept = [[round(start - offset, 2), round(end - offset, 2)] for start, end in kept]
    select = "+".join(f"between(t,{start:.2f},{end:.2f})" for start, end in kept)
    joined = f"fps={FPS},select='{select}',setpts=N/{FPS}/TB"
    run = lambda *args: subprocess.run([ffmpeg, "-y", "-loglevel", "error", *args], check=True)
    # WHY veryslow, a 20 s GOP and aq-mode 3 (measured on this recording, 2026-11-17): at the same size they hold text
    # two crf steps sharper than preset slow with -tune stillimage (crf 30 at 886 KB against crf 34 at 879 KB)
    for crf in (28, 30, 31, 32, 34, 36, 38):
        run("-i", str(webm), "-an", "-vf", f"{joined},fps={MP4_FPS},scale=960:-2:flags=lanczos,format=yuv420p", "-c:v", "libx264",
            "-preset", "veryslow", "-crf", str(crf), "-g", "300", "-x264-params", "aq-mode=3", "-movflags", "+faststart",
            str(mp4))
        if mp4.stat().st_size <= 900_000:
            break
    palette = media / "palette.png"
    for fps, width, colours in ((5, 560, 32), (5, 520, 24), (4, 520, 24), (4, 480, 16), (3, 420, 16)):
        filters = f"{joined},fps={fps},scale={width}:-1:flags=lanczos"
        run("-i", str(webm), "-vf", f"{filters},palettegen=max_colors={colours}:stats_mode=diff", str(palette))
        run("-i", str(webm), "-i", str(palette), "-lavfi", f"{filters}[x];[x][1:v]paletteuse=dither=none:diff_mode=rectangle",
            str(gif))
        if gif.stat().st_size <= 900_000:
            break
    palette.unlink()
    ffprobe = str(pathlib.Path(ffmpeg).with_name("ffprobe"))
    probe = subprocess.run([ffprobe, "-v", "error", "-show_entries", "format=duration:stream=width,height", "-of", "json",
                            str(mp4)], capture_output=True, text=True, check=True)
    info = json.loads(probe.stdout)
    return {"seconds": round(float(info["format"]["duration"]), 2), "mp4_bytes": mp4.stat().st_size, "crf": crf,
            "video_clock_offset_s": round(offset, 2), "kept_intervals_video_s": kept,
            "mp4_fps": MP4_FPS,
            "mp4_size": [info["streams"][0]["width"], info["streams"][0]["height"]], "gif_bytes": gif.stat().st_size,
            "gif": {"fps": fps, "width": width, "colours": colours}}


def still(browser, url: str, target: str, out: pathlib.Path, width: int = 1280, pad: int = 16,
          mermaid: bool = False, contains: str = "", until: str = "", unclip: bool = False) -> dict:
    """One element of one page, light theme, at scroll 0 in a viewport tall enough to hold it: no sticky header on top.

    mermaid: clip to the drawn diagram (the svg's root group) inside GitHub's iframe, not the iframe's white margins.
    unclip: GitHub draws a wide table as a horizontal scroller (overflow: auto) and hides its right-hand columns; this
    sets that one table's overflow to visible, so the same layout shows every column at once. checks.json says so.
    """
    context = browser.new_context(viewport={"width": width, "height": 900}, color_scheme="light")
    page = context.new_page()
    response = page.goto(url, wait_until="domcontentloaded")
    page.locator("article.markdown-body, [aria-labelledby='folders-and-files']").first.wait_for(timeout=30000)
    if mermaid:
        page.frame_locator("iframe.render-viewer").first.locator("svg").first.wait_for(timeout=30000)
    try:
        page.wait_for_load_state("networkidle", timeout=6000)
    except Exception:
        pass
    title = page.title()
    if response is None or response.status != 200 or any(word in title.lower() for word in BLOCKED):
        raise SystemExit(f"GitHub blocked or failed {url}: {response and response.status} {title!r}")
    locator = page.locator(target)
    if contains:
        locator = locator.filter(has_text=contains)
    element = locator.first
    frame = page.frame_locator("iframe.render-viewer").first
    if mermaid:
        element = frame.locator("svg g.root, svg > g").first
    scroll = None
    if unclip:
        scroll = element.evaluate("e => { const before = [e.scrollWidth, e.clientWidth]; e.style.overflow = 'visible';"
                                  " return before; }")
    sizing = page.locator("section.js-render-needs-enrichment").first if mermaid else element
    box = sizing.bounding_box()
    if until:                           # the clip runs from the element's top down to `until`'s top
        box["height"] = page.locator(until).first.bounding_box()["y"] - box["y"]
    page.set_viewport_size({"width": width, "height": int(box["y"] + box["height"] + pad + 40)})
    page.wait_for_timeout(1200)
    if mermaid:
        page.frame_locator("iframe.render-viewer").first.locator("svg").first.wait_for(timeout=30000)
    box = element.bounding_box()
    if mermaid:                         # a diagram nearly as wide as its panel: show the whole panel, controls and all
        panel = sizing.bounding_box()
        if box["width"] + 2 * pad + 80 >= panel["width"]:
            box, pad = panel, 0
    if until:
        box["height"] = page.locator(until).first.bounding_box()["y"] - box["y"] - pad
    if unclip:
        box["width"] = max(box["width"], scroll[0])
    clip = {"x": max(0, box["x"] - pad), "y": max(0, box["y"] - pad), "width": box["width"] + 2 * pad,
            "height": box["height"] + 2 * pad}
    for quality in (85, 78, 70, 62, 55, 48):
        page.screenshot(path=str(out), type="jpeg", quality=quality, clip=clip)
        if out.stat().st_size <= STILL_LIMIT:
            break
    element_text = target + (f" containing {contains!r}" if contains else "") + (f", down to {until}" if until else "")
    measured = {"url": url, "status": response.status, "title": title, "element": element_text,
                "clip": {key: round(value) for key, value in clip.items()}, "jpeg_quality": quality,
                "bytes": out.stat().st_size, "viewport_width": width}
    if mermaid:
        measured["mermaid_nodes"] = frame.locator("svg g.node").count()
        measured["mermaid_labels"] = frame.locator("svg g.node").evaluate_all(
            "nodes => nodes.map(node => (node.textContent.trim().match(/^M\\d\\.[a-z0-9_]+/) || [node.textContent.trim()])[0])")
    if unclip:
        measured["github_table_scroll_vs_visible_px"] = scroll
        measured["unclipped"] = ("GitHub shows this table as a horizontal scroller; the still sets its overflow to "
                                 "visible so every column shows in the same layout")
    context.close()
    return measured


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("media", type=pathlib.Path, help="where the media go, e.g. reports/media/robot-models-v1-2026-11-16")
    parser.add_argument("--repo", default="https://github.com/zacharyyamaoka/robot_models")
    parser.add_argument("--branch", default="main")
    parser.add_argument("--ffmpeg", default=shutil.which("ffmpeg") or "/usr/bin/ffmpeg")
    parser.add_argument("--workdir", type=pathlib.Path, help="keep the raw recording (webm) here; default a temp dir")
    args = parser.parse_args()
    args.media.mkdir(parents=True, exist_ok=True)
    repo, branch = args.repo.rstrip("/"), args.branch
    blob, tree = f"{repo}/blob/{branch}", f"{repo}/tree/{branch}"

    checks: dict = {"repo": repo, "branch": branch, "fallback": None, "theme": "light"}
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch()
        checks["stills"] = {
            "still-servo-ladder.jpg": still(browser, f"{blob}/src/robot_models/actuators/servo/README.md", "svg",
                                            args.media / "still-servo-ladder.jpg", mermaid=True),
            "still-bldc-ladder.jpg": still(browser, f"{blob}/src/robot_models/actuators/servo/bldc/README.md", "svg",
                                           args.media / "still-bldc-ladder.jpg", mermaid=True),
            "still-conveyor-rungs.jpg": still(browser, f"{blob}/src/robot_models/conveyors/conveyor/README.md",
                                              "article.markdown-body table", args.media / "still-conveyor-rungs.jpg",
                                              width=2400, contains="equation", unclip=True),
            "still-types-table.jpg": still(browser, f"{blob}/docs/TYPES.md", "article.markdown-body table",
                                           args.media / "still-types-table.jpg", width=2400, unclip=True),
            "still-stribeck-folder.jpg": still(browser, f"{tree}/src/robot_models/actuators/rungs/stribeck",
                                               "[aria-labelledby='folders-and-files']",
                                               args.media / "still-stribeck-folder.jpg",
                                               until="article.markdown-body h2:has-text('Inputs and state')"),
        }
        with tempfile.TemporaryDirectory() as scratch:
            workdir = args.workdir or pathlib.Path(scratch)
            workdir.mkdir(parents=True, exist_ok=True)
            webm, rec = record_hero(browser, repo, branch, workdir)
            checks["hero"] = {**encode(webm, rec.kept, rec.closed_s, args.media, args.ffmpeg), "viewport": VIEWPORT,
                              "kept_intervals_script_s": rec.kept, "pages": rec.pages,
                              "recorded_s": rec.closed_s}
        browser.close()
    (args.media / "checks.json").write_text(json.dumps(checks, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(checks, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
