"""Draw the pipeline figures of Minilink as hand-laid SVG.

Run from the repository root: ``python docs/pipeline_figures.py``; it writes one
``docs/_static/pipeline_<name>.svg`` per figure in :data:`FIGURES`: the library map, shown by
``docs/pipelines.md``.

The map is also the README's first figure, as a PNG (a raw-URL SVG does not show on
PyPI): ``python docs/make_assets.py map`` rebuilds both.
"""

from pathlib import Path

from matplotlib import rcParams
from matplotlib.textpath import TextPath

rcParams["mathtext.fontset"] = "cm"  # The equations in Computer Modern, as in the notes

FONT = "Inter, 'Helvetica Neue', Helvetica, Arial, sans-serif"

# (fill, stroke, ink) per role
COLORS = {
    "obj": ("#e0f2fe", "#0284c7", "#0c4a6e"),
    "par": ("#fef3c7", "#d97706", "#78350f"),
    "rnd": ("#fae8ff", "#c026d3", "#701a75"),
    "step": ("#ffffff", "#94a3b8", "#0f172a"),
    "out": ("#dcfce7", "#16a34a", "#14532d"),
    "jax": ("#eef2ff", "#4f46e5", "#312e81"),
    "loop": ("#f0fdfa", "#0f766e", "#134e4a"),
}
INK, MUTED, LINE = "#0f172a", "#64748b", "#64748b"


class Figure:
    """An SVG canvas with the few shapes the figures use."""

    def __init__(self, width, height):
        self.width, self.height, self.parts = width, height, []

    def add(self, element):
        self.parts.append(element)

    def text(self, x, y, s, size=15, weight=400, color=INK, anchor="middle"):
        s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.add(
            f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" '
            f'fill="{color}" text-anchor="{anchor}">{s}</text>'
        )

    def arrow(self, d, color=LINE, dashed=False, marker="arr", width=1.6):
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        self.add(
            f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash} '
            f'marker-end="url(#{marker})"/>'
        )

    def math(self, cx, cy, tex, size=17, color=INK, max_width=None):
        """A LaTeX equation (matplotlib's mathtext) as filled SVG paths, centred on (cx, cy)."""
        text = TextPath((0, 0), tex, size=size)
        if max_width is not None and text.get_extents().width > max_width:
            size = size * max_width / text.get_extents().width
            text = TextPath((0, 0), tex, size=size)
        box = text.get_extents()
        d = []
        for vertices, code in text.iter_segments():
            points = " ".join(
                f"{vx:.2f},{vy:.2f}" for vx, vy in vertices.reshape(-1, 2)
            )
            d.append(
                {1: "M", 2: "L", 3: "Q", 4: "C"}.get(code, "Z")
                + (points if code != 79 else "")
            )
        tx = cx - box.x0 - box.width / 2
        ty = cy + box.y0 + box.height / 2
        self.add(
            f'<path d="{" ".join(d)}" fill="{color}" '
            f'transform="translate({tx:.2f},{ty:.2f}) scale(1,-1)"/>'
        )

    def svg(self):
        markers = "".join(
            f'<marker id="{mid}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
            f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" '
            f'fill="{color}"/></marker>'
            for mid, color in (
                ("arr", LINE),
                ("arrg", COLORS["out"][1]),
                ("arrp", COLORS["rnd"][1]),
                ("arrt", COLORS["loop"][1]),
            )
        )
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.width} {self.height}" '
            f'width="{self.width}" height="{self.height}" font-family="{FONT}">'
            f'<defs>{markers}</defs><rect x="1" y="1" width="{self.width - 2}" height="{self.height - 2}" '
            f'rx="18" fill="#ffffff" stroke="#e2e8f0" stroke-width="1.5"/>'
            + "".join(self.parts)
            + "</svg>"
        )


JAX_TILES = (
    ("jit", "compile once"),
    ("grad", "exact gradients"),
    ("vmap", "batch many runs"),
)


def jax_icon(f, name, ix, iy):
    """The picture of one JAX transform, in a 44 × 48 box at (ix, iy)."""
    _fill, stroke, ink = COLORS["jax"]
    if name == "jit":
        # A lightning bolt
        f.add(
            f'<path d="M{ix + 22},{iy} L{ix + 6},{iy + 26} L{ix + 18},{iy + 26} L{ix + 12},{iy + 48} '
            f'L{ix + 32},{iy + 18} L{ix + 20},{iy + 18} Z" fill="{stroke}"/>'
        )
    elif name == "grad":
        # A cost curve, a point on it, and its tangent there (t = 1/4: the slope 58/42)
        f.add(
            f'<path d="M{ix},{iy + 6} Q{ix + 20},{iy + 62} {ix + 44},{iy + 10}" fill="none" '
            f'stroke="{stroke}" stroke-width="2"/>'
        )
        f.add(
            f'<path d="M{ix - 3.75},{iy + 7.9} L{ix + 24.25},{iy + 46.6}" stroke="{ink}" '
            f'stroke-width="1.6"/>'
        )
        f.add(f'<circle cx="{ix + 10.25}" cy="{iy + 27.25}" r="3.2" fill="{ink}"/>')
    else:
        # A fan of trajectories, run side by side
        for n in range(5):
            dy = n * 7
            f.add(
                f'<path d="M{ix},{iy + 36} C{ix + 12},{iy + 4 + dy} {ix + 26},{iy + 30 - dy / 2} '
                f'{ix + 44},{iy + 6 + dy}" fill="none" stroke="{stroke}" stroke-width="1.6" '
                f'opacity="{0.45 + 0.11 * n:.2f}"/>'
            )


def jax_column(f, x, y, w, h):
    """JAX as a column: its name, then jit, grad and vmap, evenly spaced."""
    fill, stroke, ink = COLORS["jax"]
    f.add(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{fill}" '
        f'stroke="{stroke}" stroke-width="1.6"/>'
    )
    f.text(x + w / 2, y + 30, "JAX", 19, 750, ink)
    top, step = y + 50, (h - 60) / 3
    for k, (name, line) in enumerate(JAX_TILES):
        ty = top + k * step
        if k:
            f.add(
                f'<path d="M{x + 24},{ty} L{x + w - 24},{ty}" stroke="{stroke}" '
                f'stroke-width="1" opacity="0.25"/>'
            )
        mid = ty + step / 2
        jax_icon(f, name, x + w / 2 - 22, mid - 52)
        f.text(x + w / 2, mid + 22, name, 16, 700, ink)
        f.text(x + w / 2, mid + 42, line, 14, 500, ink)


def link(f, x0, y0, x1, y1, kind="jax", dash="3 3"):
    """A dotted link from an object to a panel, a dot on the object."""
    stroke = COLORS[kind][1]
    f.add(
        f'<path d="M{x0},{y0} L{x1},{y1}" stroke="{stroke}" stroke-width="1.6" '
        f'stroke-dasharray="{dash}" fill="none"/>'
    )
    f.add(f'<circle cx="{x0}" cy="{y0}" r="4" fill="{stroke}"/>')


PATHS = {
    "Simulate": r"$x(t) = x_0 + \int_0^t f\, d\tau$",
    "Analyze": r"$\dot{\tilde{x}} = A\tilde{x} + B\tilde{u}$",
    "Design": r"$\lambda(A - BK) = \{p_i\}$",
    "Plan": r"$\pi^* = \mathrm{arg\,min}_{\pi}\; J$",
    "Learn": r"$\theta \leftarrow \theta + \eta \nabla_{\theta} \mathbb{E}[G]$",
    "Evaluate": r"$J(\theta) = \mathbb{E}[J]$",
}
# (fill, stroke, ink) per tool: a sweep around the wheel, left to right
ACCENT = {
    # The System's tools, cool
    "Simulate": ("#ecfeff", "#0891b2", "#164e63"),
    "Play": ("#eff6ff", "#2563eb", "#1e3a8a"),
    "Analyze": ("#ecfdf5", "#059669", "#064e3b"),
    "Design": ("#f7fee7", "#65a30d", "#365314"),
    # The problem's tools, warm
    "Plan": ("#fff7ed", "#ea580c", "#7c2d12"),
    "Learn": ("#fff1f2", "#e11d48", "#881337"),
    "Evaluate": ("#fdf4ff", "#c026d3", "#701a75"),
}
RESULT = {
    "Simulate": "Trajectory",
    "Play": "Animation",
    "Analyze": "Poles · Bode",
    "Design": "Controller",
    "Plan": "PlanningSolution",
    "Learn": "Neural policy",
    "Evaluate": "Evaluation",
}
BLUE = COLORS["obj"]


GREY, GREY_INK = "#94a3b8", "#64748b"  # The composition inset, which recedes
MONO = "ui-monospace, 'SF Mono', Menlo, Consolas, monospace"


def markers(f):
    """One arrowhead per accent, and one for the loop back (System blue)."""
    items = [(f"m{k}", v[1]) for k, v in ACCENT.items()]
    items += [("mblue", BLUE[1]), ("mgrey", GREY)]
    f.add(
        "<defs>"
        + "".join(
            f'<marker id="{mid}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
            f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" '
            f'fill="{c}"/></marker>'
            for mid, c in items
        )
        + '<filter id="lift" x="-10%" y="-25%" width="120%" height="160%">'
        '<feDropShadow dx="0" dy="3" stdDeviation="6" flood-color="#0284c7" '
        'flood-opacity="0.3"/></filter>'
        + '<linearGradient id="gblue" x1="0" y1="0" x2="0" y2="1">'
        '<stop offset="0" stop-color="#f0f9ff"/><stop offset="1" stop-color="#dbeafe"/>'
        "</linearGradient></defs>"
    )


# --- Pictograms: each drawn in a box (x, y, w, h), in one colour -------------------------


def pic_trajectory(f, x, y, w, h, c):
    f.add(
        f'<path d="M{x},{y + h} L{x + w},{y + h}" stroke="{c}" stroke-width="1" opacity="0.4"/>'
    )
    f.add(
        f'<path d="M{x + 2},{y + h * 0.7} C{x + w * 0.2},{y - h * 0.1} {x + w * 0.35},{y + h * 1.0} '
        f'{x + w * 0.55},{y + h * 0.45} S{x + w * 0.85},{y + h * 0.2} {x + w - 2},{y + h * 0.3}" '
        f'fill="none" stroke="{c}" stroke-width="2.4" stroke-linecap="round"/>'
    )
    f.add(f'<circle cx="{x + 2}" cy="{y + h * 0.7}" r="3" fill="{c}"/>')


def pic_bode(f, x, y, w, h, c):
    f.add(
        f'<path d="M{x},{y} L{x},{y + h} L{x + w},{y + h}" fill="none" stroke="{c}" '
        f'stroke-width="1" opacity="0.4"/>'
    )
    f.add(
        f'<path d="M{x + 4},{y + h * 0.25} L{x + w * 0.4},{y + h * 0.25} '
        f'Q{x + w * 0.52},{y + h * 0.15} {x + w * 0.58},{y + h * 0.4} L{x + w - 4},{y + h * 0.95}" '
        f'fill="none" stroke="{c}" stroke-width="2.4" stroke-linecap="round"/>'
    )


def cross(f, cx, cy, r, c, width=2.2, opacity=1):
    f.add(
        f'<path d="M{cx - r},{cy - r} L{cx + r},{cy + r} M{cx - r},{cy + r} L{cx + r},{cy - r}" '
        f'stroke="{c}" stroke-width="{width}" stroke-linecap="round" opacity="{opacity}"/>'
    )


def pic_poles(f, x, y, w, h, c):
    """The complex plane: poles moved from the right half into the left."""
    ax = x + w * 0.62
    f.add(
        f'<path d="M{x},{y + h / 2} L{x + w},{y + h / 2} M{ax},{y} L{ax},{y + h}" '
        f'stroke="{c}" stroke-width="1" opacity="0.4"/>'
    )
    for dy in (-0.3, 0, 0.3):
        cross(f, ax + w * 0.2, y + h * (0.5 + dy), 3.5, MUTED, 1.6, 0.6)
        cross(f, x + w * 0.22, y + h * (0.5 + dy), 4, c)
    f.add(
        f'<path d="M{ax + w * 0.13},{y + h / 2} L{x + w * 0.32},{y + h / 2}" stroke="{c}" '
        f'stroke-width="1.4" stroke-dasharray="3 3" marker-end="url(#mDesign)"/>'
    )


def pic_path(f, x, y, w, h, c):
    """A start, an obstacle, a path around it and a goal."""
    f.add(
        f'<circle cx="{x + w * 0.5}" cy="{y + h * 0.55}" r="{h * 0.3}" fill="#e2e8f0"/>'
    )
    f.add(
        f'<path d="M{x + 4},{y + h * 0.85} C{x + w * 0.3},{y + h * 0.9} {x + w * 0.3},{y - h * 0.05} '
        f'{x + w * 0.55},{y + h * 0.08} S{x + w * 0.85},{y + h * 0.5} {x + w - 8},{y + h * 0.45}" '
        f'fill="none" stroke="{c}" stroke-width="2.4" stroke-linecap="round"/>'
    )
    f.add(f'<circle cx="{x + 4}" cy="{y + h * 0.85}" r="3.5" fill="{c}"/>')
    f.add(
        f'<circle cx="{x + w - 8}" cy="{y + h * 0.45}" r="6" fill="none" stroke="{c}" stroke-width="2"/>'
    )


def pic_network(f, x, y, w, h, c):
    layers = [3, 4, 2]
    pts = []
    for i, n in enumerate(layers):
        lx = x + w * (0.12 + 0.38 * i)
        pts.append([(lx, y + h * (j + 0.5) / n) for j in range(n)])
    for a, b in zip(pts, pts[1:]):
        for p in a:
            for q in b:
                f.add(
                    f'<path d="M{p[0]},{p[1]} L{q[0]},{q[1]}" stroke="{c}" stroke-width="0.9" opacity="0.45"/>'
                )
    for layer in pts:
        for px, py in layer:
            f.add(
                f'<circle cx="{px}" cy="{py}" r="4.2" fill="#ffffff" stroke="{c}" stroke-width="2"/>'
            )


def pic_histogram(f, x, y, w, h, c):
    heights = [0.2, 0.5, 0.9, 1.0, 0.65, 0.3, 0.12]
    bw = w / len(heights)
    for i, v in enumerate(heights):
        f.add(
            f'<rect x="{x + i * bw + 1.5}" y="{y + h * (1 - v)}" width="{bw - 3}" height="{h * v}" '
            f'rx="1.5" fill="{c}" opacity="{0.35 + 0.5 * v:.2f}"/>'
        )
    mx = x + bw * 3.2
    f.add(
        f'<path d="M{mx},{y - 3} L{mx},{y + h}" stroke="{ACCENT["Evaluate"][2]}" stroke-width="1.6" '
        f'stroke-dasharray="3 2"/>'
    )


def pic_screen(f, x, y, w, h, c):
    """A screen on its stand, a cart and its pendulum in it."""
    sw, sh = min(w, 1.6 * h), h - 8
    sx = x + (w - sw) / 2
    f.add(
        f'<rect x="{sx}" y="{y}" width="{sw}" height="{sh}" rx="4" fill="none" stroke="{c}" '
        f'stroke-width="2"/>'
    )
    mx = sx + sw / 2
    f.add(
        f'<path d="M{mx},{y + sh} L{mx},{y + h} M{mx - 10},{y + h} L{mx + 10},{y + h}" '
        f'stroke="{c}" stroke-width="2" stroke-linecap="round"/>'
    )
    gy = y + sh - 9
    f.add(
        f'<path d="M{sx + 8},{gy} L{sx + sw - 8},{gy}" stroke="{c}" stroke-width="1" opacity="0.4"/>'
    )
    cx = mx - 10
    f.add(f'<rect x="{cx - 9}" y="{gy - 8}" width="18" height="8" rx="2" fill="{c}"/>')
    f.add(
        f'<path d="M{cx},{gy - 8} L{cx + 16},{y + 9}" stroke="{c}" stroke-width="2" '
        f'stroke-linecap="round"/>'
    )
    f.add(f'<circle cx="{cx + 16}" cy="{y + 9}" r="4" fill="{c}"/>')


def pic_gamepad(f, x, y, w, h, c):
    """A gamepad: a body with two grips, a d-pad and two buttons, centred in the box."""
    cx, cy = x + w / 2, y + h / 2 - 4
    f.add(
        f'<path d="M{cx - 26},{cy - 13} L{cx + 26},{cy - 13} Q{cx + 39},{cy - 13} {cx + 41},{cy + 2} '
        f"L{cx + 44},{cy + 17} Q{cx + 45},{cy + 25} {cx + 36},{cy + 23} L{cx + 21},{cy + 10} "
        f"L{cx - 21},{cy + 10} L{cx - 36},{cy + 23} Q{cx - 45},{cy + 25} {cx - 44},{cy + 17} "
        f'L{cx - 41},{cy + 2} Q{cx - 39},{cy - 13} {cx - 26},{cy - 13} Z" fill="none" '
        f'stroke="{c}" stroke-width="2.2" stroke-linejoin="round"/>'
    )
    dx = cx - 22
    f.add(
        f'<path d="M{dx - 6},{cy - 1} L{dx + 6},{cy - 1} M{dx},{cy - 7} L{dx},{cy + 5}" '
        f'stroke="{c}" stroke-width="3" stroke-linecap="round"/>'
    )
    for bx, by in ((cx + 18, cy - 5), (cx + 26, cy + 2)):
        f.add(f'<circle cx="{bx}" cy="{by}" r="3.2" fill="{c}"/>')


PICS = {
    "Simulate": pic_trajectory,
    "Play": pic_screen,
    "Analyze": pic_bode,
    "Design": pic_poles,
    "Plan": pic_path,
    "Learn": pic_network,
    "Evaluate": pic_histogram,
}


def grey_block(f, x, y, w, h, label):
    f.add(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="5" fill="#ffffff" '
        f'stroke="{GREY}" stroke-width="1.8"/>'
    )
    f.text(x + w / 2, y + h / 2 + 4, label, 11, 650, GREY)


def grey_wire(f, d, width=1.5):
    f.add(
        f'<path d="{d}" fill="none" stroke="{GREY}" stroke-width="{width}" '
        f'stroke-linejoin="round" marker-end="url(#mgrey)"/>'
    )


def code(f, cx, y, s):
    s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    f.add(
        f'<text x="{cx}" y="{y}" font-size="12.5" font-weight="600" fill="{GREY_INK}" '
        f'text-anchor="middle" font-family="{MONO}">{s}</text>'
    )


def pic_problem(f, x, y, w, h, c):
    """The allowed set X, a start in it and the goal set Xf."""
    f.add(
        f'<path d="M{x + w * 0.1},{y + h * 0.5} C{x},{y} {x + w * 0.6},{y - h * 0.1} {x + w * 0.85},{y + h * 0.2} '
        f"S{x + w * 1.05},{y + h * 0.95} {x + w * 0.5},{y + h * 0.95} S{x + w * 0.18},{y + h * 0.85} "
        f'{x + w * 0.1},{y + h * 0.5} Z" fill="#ffffff" stroke="{c}" stroke-width="1.6" stroke-dasharray="4 3"/>'
    )
    sx, sy, gx, gy = x + w * 0.24, y + h * 0.62, x + w * 0.72, y + h * 0.4
    f.add(
        f'<path d="M{sx},{sy} Q{x + w * 0.45},{y + h * 0.2} {gx - 9},{gy}" fill="none" stroke="{c}" '
        f'stroke-width="2" marker-end="url(#mblue)"/>'
    )
    f.add(f'<circle cx="{sx}" cy="{sy}" r="3.5" fill="{c}"/>')
    f.add(
        f'<circle cx="{gx}" cy="{gy}" r="8" fill="{BLUE[0]}" stroke="{c}" stroke-width="2"/>'
    )


def pic_bowl(f, x, y, w, h, c):
    """Level sets of a cost and the iterates descending to its minimum."""
    cx, cy = x + w / 2, y + h / 2
    for k, s in enumerate((1.0, 0.72, 0.44, 0.2)):
        f.add(
            f'<ellipse cx="{cx}" cy="{cy}" rx="{w / 2 * s}" ry="{h / 2 * s}" fill="none" '
            f'stroke="{c}" stroke-width="1.4" opacity="{0.35 + 0.15 * k:.2f}"/>'
        )
    pts = [
        (x + w * 0.08, y + h * 0.4),
        (x + w * 0.28, y + h * 0.7),
        (x + w * 0.42, y + h * 0.45),
        (cx, cy),
    ]
    d = " L".join(f"{px},{py}" for px, py in pts)
    f.add(f'<path d="M{d}" fill="none" stroke="{c}" stroke-width="1.8"/>')
    for px, py in pts[:-1]:
        f.add(f'<circle cx="{px}" cy="{py}" r="2.6" fill="{c}"/>')
    f.add(f'<circle cx="{cx}" cy="{cy}" r="4" fill="{BLUE[2]}"/>')


def foundation(f, x, y, w, h, title, texs, pic, pic_w, size):
    """A foundation card: a soft gradient, its pictogram on the left, title and equations."""
    f.add(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="url(#gblue)" '
        f'stroke="{BLUE[1]}" stroke-width="1.8"/>'
    )
    pad = 22
    pic(f, x + pad, y + 18, pic_w, h - 36, BLUE[1])
    lx = x + pad + pic_w + 10
    cx = (lx + x + w) / 2
    f.text(cx, y + 28, title, 16, 700, BLUE[2])
    room = (y + h - 12) - (y + 40)
    for i, tex in enumerate(texs):
        f.math(
            cx,
            y + 40 + room * (i + 0.5) / len(texs),
            tex,
            size,
            BLUE[2],
            max_width=x + w - lx - 16,
        )


TOOL_TEX = {**PATHS, "Plan": r"$\pi^{\star} = \arg\min_{\pi}\; J$"}


def system_card(f, x, y, w, h):
    """The System: its definition, centred, the first thing the eye meets; returns the definition's box."""
    f.add(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="#f0f9ff" '
        f'stroke="{BLUE[1]}" stroke-width="1.4"/>'
    )
    bw, bh = 440, h - 20
    bx, by = x + (w - bw) / 2, y + 10
    f.add(
        f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="10" fill="#ffffff" '
        f'stroke="{BLUE[1]}" stroke-width="3" filter="url(#lift)"/>'
    )
    cx = bx + bw / 2
    f.text(cx, by + 28, "System", 19, 750, BLUE[2])
    f.math(cx, by + 58, r"$\dot{x} = f(x, u, t;\, p)$", 25, BLUE[2])
    f.math(cx, by + 92, r"$y = h(x, u, t;\, p)$", 25, BLUE[2])
    return bx, by, bw


def pic_class(f, x, y, w, h, c):
    """A class written by hand: its two lines of code, the equation it owns."""
    f.add(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="7" fill="#ffffff" '
        f'stroke="#cbd5e1" stroke-width="1.2"/>'
    )
    for k, line in enumerate(
        ("class Arm(System):", "  def f(self, x, u, t, params):", "    return dx")
    ):
        s = line.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        f.add(
            f'<text x="{x + 12}" y="{y + 22 + 18 * k}" font-size="12" font-weight="600" '
            f'fill="{GREY_INK}" font-family="{MONO}" xml:space="preserve">{s}</text>'
        )


def pic_catalog(f, x, y, w, h, c):
    """The catalog: a shelf of plants, a pendulum, a cart-pole and a car."""
    base = y + h - 6
    f.add(f'<path d="M{x},{base} L{x + w},{base}" stroke="{GREY}" stroke-width="1.5"/>')
    # A pendulum on its pivot
    px, py = x + w * 0.16, y + 8
    f.add(
        f'<path d="M{px - 12},{py} L{px + 12},{py}" stroke="{GREY}" stroke-width="2"/>'
    )
    f.add(
        f'<path d="M{px},{py} L{px + 16},{py + 32}" stroke="{c}" stroke-width="2.2"/>'
    )
    f.add(
        f'<circle cx="{px + 16}" cy="{py + 32}" r="6" fill="{BLUE[0]}" stroke="{c}" stroke-width="2"/>'
    )
    # A cart and its pole
    cx = x + w * 0.5
    f.add(
        f'<rect x="{cx - 20}" y="{base - 18}" width="40" height="14" rx="3" fill="{BLUE[0]}" '
        f'stroke="{c}" stroke-width="1.8"/>'
    )
    f.add(
        f'<path d="M{cx},{base - 18} L{cx + 10},{y + 6}" stroke="{c}" stroke-width="2.2"/>'
    )
    # A car seen from above
    vx = x + w * 0.84
    f.add(
        f'<rect x="{vx - 14}" y="{y + 8}" width="28" height="{base - y - 16}" rx="7" '
        f'fill="{BLUE[0]}" stroke="{c}" stroke-width="1.8"/>'
    )
    for wy in (y + 14, base - 16):
        for wx in (vx - 18, vx + 14):
            f.add(f'<rect x="{wx}" y="{wy}" width="4" height="9" rx="1.5" fill="{c}"/>')


def pic_blocks(f, x, y, w, h, c):
    """A composition: a controller closed on a plant, K @ P."""
    cx, yy, bw, bh = x + w / 2, y + 8, 34, 22
    grey_block(f, cx - bw - 10, yy, bw, bh, "K")
    grey_block(f, cx + 10, yy, bw, bh, "P")
    my, fy = yy + bh / 2, yy + bh + 16
    grey_wire(f, f"M{cx - 10},{my} L{cx + 8},{my}")
    grey_wire(
        f,
        f"M{cx + 10 + bw},{my} L{cx + bw + 24},{my} L{cx + bw + 24},{fy} "
        f"L{cx - bw - 24},{fy} L{cx - bw - 24},{my} L{cx - bw - 12},{my}",
    )


def origin_cards(f, x0, y, width, h, target):
    """How a System is made: three cards, a class, the catalog, a composition, each arrow meeting the System."""
    gap = 24
    w = (width - 2 * gap) / 3
    origins = (
        ("Your own class", pic_class, "f, h, tf written by hand"),
        ("The catalog", pic_catalog, "Pendulum(), CartPole(), …"),
        ("A composition of blocks", pic_blocks, "ctl @ plant · a >> b · DiagramSystem"),
    )
    tx, ty, tw = target
    for k, (title, pic, line) in enumerate(origins):
        x = x0 + k * (w + gap)
        f.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="url(#gblue)" '
            f'stroke="{BLUE[1]}" stroke-width="1.6"/>'
        )
        f.text(x + w / 2, y + 24, title, 15.5, 700, BLUE[2])
        pic(f, x + 40, y + 34, w - 80, h - 66, BLUE[1])
        code(f, x + w / 2, y + h - 12, line)

        # The arrow from the card's foot to the System's definition
        end_x = tx + tw * (k + 1) / 4
        f.arrow(
            f"M{x + w / 2},{y + h} C{x + w / 2},{y + h + 26} {end_x},{ty - 30} {end_x},{ty - 3}",
            BLUE[1],
            marker="mblue",
            width=2,
        )


def library_map():
    """The whole library on one page: where a System comes from, three foundations, seven tools as coloured lanes, JAX beside."""
    order = list(ACCENT)
    x0, w, gap = 58, 144, 14
    pair = 14  # Simulate and Play share a lane, the replay arrow between them
    cols = {
        name: x0 + i * (w + gap) + (pair if i else 0) for i, name in enumerate(order)
    }
    right = cols[order[-1]] + w
    jx, jw = right + 24, 134
    px0, px1 = cols["Plan"], right
    org_y, org_h = 30, 132
    sys_y, sys_h = org_y + org_h + 56, 134
    py, ph = sys_y + sys_h + 36, 120
    ry, rh = py + ph + 36, 92
    oy, oh = ry + rh + 30, 104
    bus_y = oy + oh + 28
    by, bh = bus_y + 44, 88
    f = Figure(jx + jw + 30, by + bh + 30)
    markers(f)

    target = system_card(f, x0, sys_y, right - x0, sys_h)
    origin_cards(f, x0, org_y, right - x0, org_h, target)
    foundation(
        f,
        px0,
        py,
        px1 - px0,
        ph,
        "PlanningProblem",
        [
            r"$J = \int_0^{t_f} g(x, u, t)\, dt \;+\; h(x(t_f))$",
            r"$x \in X \quad u \in U \quad x(0) = x_0 \quad x(t_f) \in X_f$",
        ],
        pic_problem,
        92,
        16,
    )
    f.arrow(
        f"M{(px0 + px1) / 2},{sys_y + sys_h} L{(px0 + px1) / 2},{py - 2}",
        BLUE[1],
        marker="mblue",
        width=2,
    )

    # The program, reached by Plan and Evaluate: straight down from what they give back
    plan_cx = cols["Plan"] + w / 2
    links = {"Plan": plan_cx + 30, "Evaluate": cols["Evaluate"] + w / 2}
    for name, lx in links.items():
        f.arrow(
            f"M{lx},{oy + oh} L{lx},{by - 2}",
            ACCENT[name][1],
            dashed=True,
            marker=f"m{name}",
            width=1.8,
        )

    # Seven lanes: the tool, then what it gives back, in one colour
    for name, x in cols.items():
        fill, stroke, ink = ACCENT[name]
        cx = x + w / 2
        if name != "Play":
            bw = cols["Play"] + w - x if name == "Simulate" else w
            f.add(
                f'<rect x="{x - 4}" y="{ry - 8}" width="{bw + 8}" height="{oy + oh - ry + 16}" '
                f'rx="14" fill="{fill}"/>'
            )
        top = (
            sys_y + sys_h
            if name in ("Simulate", "Play", "Analyze", "Design")
            else py + ph
        )
        f.arrow(f"M{cx},{top} L{cx},{ry - 10}", stroke, marker=f"m{name}", width=2)
        f.add(
            f'<rect x="{x}" y="{ry}" width="{w}" height="{rh}" rx="10" fill="#ffffff" '
            f'stroke="{stroke}" stroke-width="1.6"/>'
        )
        f.add(
            f'<path d="M{x + 10},{ry} L{x + w - 10},{ry}" stroke="{stroke}" stroke-width="5" '
            f'stroke-linecap="round"/>'
        )
        f.text(cx, ry + 28, name, 15.5, 700, ink)
        if name == "Play":
            pic_gamepad(f, x + 14, ry + 40, w - 28, rh - 44, stroke)
        else:
            f.math(cx, ry + 62, TOOL_TEX[name], 17, "#0f172a", max_width=w - 28)
        f.arrow(f"M{cx},{ry + rh} L{cx},{oy - 2}", stroke, marker=f"m{name}", width=2)
        f.add(
            f'<rect x="{x}" y="{oy}" width="{w}" height="{oh}" rx="10" fill="#ffffff" '
            f'stroke="{stroke}" stroke-width="1.6"/>'
        )
        f.text(cx, oy + 24, RESULT[name], 14.5, 650, ink)
        PICS[name](f, x + 24, oy + 40, w - 48, oh - 54, stroke)

    # A trajectory replays as an animation too
    ty = oy + oh / 2 + 8
    f.arrow(
        f"M{cols['Simulate'] + w},{ty} L{cols['Play'] - 2},{ty}",
        ACCENT["Play"][1],
        marker="mPlay",
        width=2,
    )

    # Every controller is a System again: one bold loop back to the top
    drops = [cols["Design"] + w / 2, plan_cx - 30, cols["Learn"] + w / 2]
    loop = BLUE[1]
    lx = 34
    path = f"M{max(drops)},{bus_y} L{lx},{bus_y} L{lx},{sys_y + sys_h / 2} L{x0 - 2},{sys_y + sys_h / 2}"
    f.add(f'<path d="{path}" fill="none" stroke="#ffffff" stroke-width="8"/>')
    for cx in drops:
        f.add(
            f'<path d="M{cx},{oy + oh + 8} L{cx},{bus_y}" stroke="{loop}" stroke-width="2.4"/>'
        )
        f.add(f'<circle cx="{cx}" cy="{oy + oh + 8}" r="3.5" fill="{loop}"/>')
    f.arrow(path, loop, marker="mblue", width=2.4)
    f.text(
        x0,
        bus_y + 24,
        "closed loop: policy @ plant is a System again",
        14.5,
        650,
        BLUE[2],
        anchor="start",
    )

    foundation(
        f,
        px0,
        by,
        px1 - px0,
        bh,
        "MathematicalProgram · Optimizer",
        [r"$\min_{z}\; J(z) \quad \mathrm{s.t.} \quad h(z) = 0,\;\; g(z) \geq 0$"],
        pic_bowl,
        92,
        17,
    )

    # JAX
    jax_column(f, jx, sys_y, jw, by + bh - sys_y)
    for y in (sys_y + sys_h / 2, py + ph / 2, ry + rh / 2, by + bh / 2):
        link(f, right if y < by else px1, y, jx, y)

    # A two-line key, in the free corner
    kx, ky = x0 + 4, by + bh / 2 - 6
    for k, name in enumerate(("Plan", "Evaluate")):
        c = ACCENT[name][1]
        f.add(
            f'<path d="M{kx + 18 * k},{ky} L{kx + 18 * k + 14},{ky}" stroke="{c}" stroke-width="2.2"/>'
        )
    f.add(
        f'<path d="M{kx},{ky + 28} L{kx + 32},{ky + 28}" stroke="{COLORS["jax"][1]}" '
        f'stroke-width="2.2" stroke-dasharray="3 3"/>'
    )
    f.text(kx + 46, ky + 5, "solved as a program", 14.5, 500, MUTED, anchor="start")
    f.text(kx + 46, ky + 33, "traces under JAX", 14.5, 500, MUTED, anchor="start")
    return f.svg()


FIGURES = {
    "map": library_map,
}


if __name__ == "__main__":
    static = Path(__file__).resolve().parent / "_static"
    for name, draw in FIGURES.items():
        (static / f"pipeline_{name}.svg").write_text(draw())
