"""Machinery behind docs/make_assets.py: GIF writing, page capture, the bridges SVG."""

from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import escape

GIF_WIDTH = 480  # px
GIF_FPS = 15
GIF_MAX_BYTES = 1_000_000  # RULES 6.9
GIF_COLORS = (96, 64, 48, 32)  # palettes tried in turn until the GIF fits the budget


def write_gif(frames, path: Path, *, fps: int = GIF_FPS, width: int = GIF_WIDTH):
    """Resize RGB frames to ``width`` and save a looping GIF under the size budget."""
    from PIL import Image

    resized = []
    for frame in frames:
        frame = frame.convert("RGB")
        height = int(round(frame.height * width / frame.width))
        resized.append(frame.resize((width, height), Image.LANCZOS))
    for colors in GIF_COLORS:
        quantized = [frame.quantize(colors=colors) for frame in resized]
        quantized[0].save(
            path,
            save_all=True,
            append_images=quantized[1:],
            duration=int(1000 / fps),
            loop=0,
            optimize=True,
        )
        size = path.stat().st_size
        if size <= GIF_MAX_BYTES:
            break
    print(f"{path.name}: {len(resized)} frames, {colors} colors, {size / 1e6:.2f} MB")
    if size > GIF_MAX_BYTES:
        print(f"  warning: above the {GIF_MAX_BYTES / 1e6:.0f} MB budget")


def shrink_gif(path: Path, *, width: int = GIF_WIDTH, fps: int = GIF_FPS) -> None:
    """Resize and resample a GIF written at export DPI down to README size."""
    from PIL import Image, ImageSequence

    with Image.open(path) as im:
        src_ms = im.info.get("duration", 1000 / 30)
        keep_every = max(1, int(round((1000 / fps) / src_ms)))
        frames = [
            frame.convert("RGB")
            for k, frame in enumerate(ImageSequence.Iterator(im))
            if k % keep_every == 0
        ]
    write_gif(frames, path, fps=fps, width=width)


def require_playwright():
    """The Playwright sync API, or an ImportError that says how to install it."""
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as e:
        raise ImportError(
            "The 3-D GIFs are screenshots of the exported page in a headless "
            "browser. Install it once with: pip install playwright && "
            "python -m playwright install chromium"
        ) from e
    return sync_playwright


# Software WebGL: headless Chromium has no GPU to fall back from.
_CHROMIUM_ARGS = (
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
    "--ignore-gpu-blocklist",
)
_HIDE_VIEWER_PANEL = ".dg { display: none !important; }"
_SCENE_NODE_COUNT = "() => { let n = 0; viewer.scene.traverse(() => n++); return n; }"
_NEXT_PAINT = (
    "() => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)))"
)


def html_to_gif(
    html_path: Path,
    gif_path: Path,
    *,
    fps: int = GIF_FPS,
    viewport: tuple[int, int] = (640, 480),
    scale: int = 2,
    timeout_ms: int = 120_000,
) -> None:
    """Screenshot an exported meshcat page frame by frame and write the GIF.

    The page's own clip is paused and sought to each frame time, so the GIF
    shows exactly what the page plays. ``scale`` supersamples the screenshots.
    """
    from io import BytesIO

    from PIL import Image

    sync_playwright = require_playwright()
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(args=list(_CHROMIUM_ARGS))
        page = browser.new_page(
            viewport={"width": viewport[0], "height": viewport[1]},
            device_scale_factor=scale,
        )
        page.goto(Path(html_path).resolve().as_uri())
        page.add_style_tag(content=_HIDE_VIEWER_PANEL)
        try:
            page.wait_for_function(
                "() => window.viewer && viewer.animator.duration > 0",
                timeout=timeout_ms,
            )
        except Exception as e:
            raise RuntimeError(f"no meshcat animation found in {html_path}") from e
        # the page loads its commands asynchronously: wait until the scene settles
        count = -1
        while count != (count := page.evaluate(_SCENE_NODE_COUNT)):
            page.wait_for_timeout(500)
        duration = page.evaluate(
            "() => { viewer.animator.pause(); return viewer.animator.duration; }"
        )
        frames = []
        for j in range(int(duration * fps) + 1):
            # a hair past the key, so a discrete track lands on the frame's page
            page.evaluate(
                f"() => viewer.animator.seek({min(j / fps + 1e-4, duration)})"
            )
            page.evaluate(_NEXT_PAINT)
            frames.append(Image.open(BytesIO(page.screenshot())).convert("RGB"))
        browser.close()
    write_gif(frames, Path(gif_path), fps=fps)


# The "one model, every tool" figure: ten tiles round the System hub.
_BRIDGES_HEAD = """\
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 400" width="1000" height="400" font-family="'Helvetica Neue', Helvetica, Arial, sans-serif">
  <defs>
    <marker id="hub-ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto">
      <path d="M0,0 L10,5 L0,10 z" fill="#a1a1aa"/>
    </marker>
  </defs>
  <style>
    .canvas { fill: #fafafa; stroke: #e5e7eb; }
    .tile { fill: #ffffff; stroke: #d4d4d8; stroke-width: 1.4; }
    .core { fill: #eff6ff; stroke: #2563eb; stroke-width: 2.6; }
    .t { fill: #18181b; font-size: 18px; font-weight: 700; }
    .s { fill: #71717a; font-size: 15px; }
    .h1 { fill: #1e3a8a; font-size: 30px; font-weight: 700; }
    .math { fill: #2563eb; font-size: 22px; font-style: italic; font-family: 'Times New Roman', Times, serif; }
    .spoke { stroke: #a1a1aa; stroke-width: 1.7; fill: none; marker-end: url(#hub-ah); }
  </style>
  <rect class="canvas" x="0.5" y="0.5" width="999" height="399" rx="18"/>
"""
_TILE_W, _TILE_H = 219, 76
_COLUMNS = (32, 271, 510, 749)  # tile x, left to right
_ROWS = (16, 162, 308)  # tile y: top, middle, bottom
# tile slots in reading order: the top row, the middle row's two ends, the bottom row
_SLOTS = (
    *((x, _ROWS[0]) for x in _COLUMNS),
    (_COLUMNS[0], _ROWS[1]),
    (_COLUMNS[-1], _ROWS[1]),
    *((x, _ROWS[2]) for x in _COLUMNS),
)
_SPOKE_START = (430, 470, 530, 570)  # on the hub's top and bottom edges
_SPOKE_BEND = (240, 400, 600, 760)
_SPOKE_END = (141, 380, 620, 858)  # on the tiles' facing edges
_HUB_SPOKES = """\
  <path class="spoke" d="M330,200 L251,200"/>
  <path class="spoke" d="M670,200 L749,200"/>
"""


def bridges_svg(tiles, core) -> str:
    """The hub-and-spoke figure: ``tiles`` are ten ``(title, subtitle)`` pairs.

    A title with a ``\\n`` takes two lines and no subtitle. ``core`` is the hub's
    ``(name, equations)``.
    """
    if len(tiles) != len(_SLOTS):
        raise ValueError(f"bridges_svg takes {len(_SLOTS)} tiles, got {len(tiles)}")
    blocks = [_BRIDGES_HEAD]
    for (x, y), (title, subtitle) in zip(_SLOTS, tiles):
        cx = x + _TILE_W // 2
        block = f'  <rect class="tile" x="{x}" y="{y}" width="{_TILE_W}" height="{_TILE_H}" rx="12"/>\n'
        lines = title.split("\n")
        if len(lines) == 2:
            for dy, line in zip((26, 48), lines):
                block += f'  <text class="t" x="{cx}" y="{y + dy}" text-anchor="middle">{escape(line)}</text>\n'
        else:
            block += f'  <text class="t" x="{cx}" y="{y + 32}" text-anchor="middle">{escape(title)}</text>\n'
            block += f'  <text class="s" x="{cx}" y="{y + 54}" text-anchor="middle">{escape(subtitle)}</text>\n'
        blocks.append(block)
    name, equations = core
    blocks.append(
        '  <rect class="core" x="330" y="150" width="340" height="100" rx="16"/>\n'
        f'  <text class="h1" x="500" y="194" text-anchor="middle">{escape(name)}</text>\n'
        f'  <text class="math" x="500" y="226" text-anchor="middle">{escape(equations)}</text>\n'
    )
    spokes = ""
    for start, bend, end in zip(_SPOKE_START, _SPOKE_BEND, _SPOKE_END):
        spokes += f'  <path class="spoke" d="M{start},150 Q{bend},120 {end},92"/>\n'
    spokes += _HUB_SPOKES
    for start, bend, end in zip(_SPOKE_START, _SPOKE_BEND, _SPOKE_END):
        spokes += f'  <path class="spoke" d="M{start},250 Q{bend},280 {end},308"/>\n'
    blocks.append(spokes + "</svg>\n")
    return "\n".join(blocks)


def write_text_asset(path: Path, text: str) -> None:
    """Write a text asset with LF line endings on every platform."""
    Path(path).write_text(text, encoding="utf-8", newline="\n")
    print(path.name)
