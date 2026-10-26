"""Draw the pipeline figures of Minilink as hand-laid SVG.

Run from the repository root: ``python docs/pipeline_figures.py``; it writes one
``docs/_static/pipeline_<name>.svg`` per figure in :data:`FIGURES`. The evaluator's two
(deterministic, stochastic) are in DESIGN §6; the gallery is ``docs/pipelines.md``.

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
CARD_W, CARD_H = 150, 76


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

    def card(self, x, y, w, h, title, lines, kind="step", stack=0, lift=0):
        """A rounded card, its title and lines centred; ``stack`` cards drawn behind it."""
        fill, stroke, ink = COLORS[kind]
        for k in range(stack, 0, -1):
            self.add(
                f'<rect x="{x + 6 * k}" y="{y - 6 * k}" width="{w}" height="{h}" rx="8" '
                f'fill="{fill}" stroke="{stroke}" stroke-width="1.2" opacity="0.85"/>'
            )
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="1.5"/>'
        )
        y0 = y + h / 2 - len(lines) * 10 + 5 - lift
        self.text(x + w / 2, y0, title, 15, 650, ink)
        for i, s in enumerate(lines):
            self.text(x + w / 2, y0 + 21 * (i + 1), s, 14, 400, ink)

    def pill(self, cx, cy, s, kind="par", w=50, h=24, size=13.5, weight=600):
        fill, stroke, ink = COLORS[kind]
        self.add(
            f'<rect x="{cx - w / 2}" y="{cy - h / 2}" width="{w}" height="{h}" rx="{h / 2}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>'
        )
        self.text(cx, cy + 4.5, s, size, weight, ink)

    def arrow(self, d, color=LINE, dashed=False, marker="arr", width=1.6):
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        self.add(
            f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash} '
            f'marker-end="url(#{marker})"/>'
        )

    def dotted(self, d, color):
        self.add(
            f'<path d="{d}" fill="none" stroke="{color}" stroke-width="1.4" '
            f'stroke-dasharray="3 3"/>'
        )

    def frame(self, x, y, w, h, label, dashed=False, label_bottom=False):
        dash = ' stroke-dasharray="6 5"' if dashed else ""
        fill = "#ffffff" if dashed else "#f8fafc"
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{fill}" '
            f'stroke="#cbd5e1" stroke-width="1.2"{dash}/>'
        )
        label_y = y + h - 12 if label_bottom else y + 22
        self.text(x + 16, label_y, label, 13.5, 600, MUTED, anchor="start")

    def trajectory(self, x, y, w, h, title, sub, stack=0):
        """The trajectory object: a card with a small sketch of x(t)."""
        self.card(x, y, w, h, "", [], "obj", stack=stack)
        ink = COLORS["obj"][2]
        sx, sy = x + 18, y + 30
        self.add(
            f'<path d="M{sx},{sy} C{sx + 14},{sy - 22} {sx + 26},{sy + 16} {sx + 40},{sy - 4} '
            f'S{sx + 62},{sy - 10} {sx + w - 36},{sy - 8}" fill="none" stroke="{COLORS["obj"][1]}" '
            f'stroke-width="2"/>'
        )
        self.text(x + w / 2, y + h - 30, title, 15, 650, ink)
        self.text(x + w / 2, y + h - 11, sub, 13.5, 400, ink)

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

    def eq_card(self, x, y, w, h, title, tex, kind="step", size=17):
        """A card with its title and one or two equations under it."""
        fill, stroke, ink = COLORS[kind]
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="1.5"/>'
        )
        lines = [tex] if isinstance(tex, str) else tex
        top = y + 26
        self.text(x + w / 2, top, title, 15, 650, ink)
        room = (y + h - 10) - (top + 12)
        for i, line in enumerate(lines):
            cy = top + 12 + room * (i + 0.5) / len(lines)
            self.math(x + w / 2, cy, line, size, ink, max_width=w - 18)

    def grid_icon(self, x, y, n=5, step=9):
        """A small lattice of nodes: a discretized state space."""
        for i in range(n):
            for j in range(n):
                self.add(
                    f'<circle cx="{x + i * step}" cy="{y + j * step}" r="1.8" fill="{COLORS["obj"][1]}"/>'
                )

    def heat_icon(self, x, y, n=5, size=9):
        """A small table of shaded cells: a cost-to-go on the grid."""
        for i in range(n):
            for j in range(n):
                d = ((i - 2) ** 2 + (j - 2) ** 2) / 8
                alpha = 0.15 + 0.7 * min(d, 1)
                self.add(
                    f'<rect x="{x + i * size}" y="{y + j * size}" width="{size - 1}" height="{size - 1}" '
                    f'fill="{COLORS["obj"][1]}" opacity="{alpha:.2f}"/>'
                )

    def loop_back(self, x0, x1, y, label, color=LINE, below=0):
        """An arc over a row (or under it, ``below`` its height), from ``x1`` back to ``x0``."""
        if below:
            yb = y + below
            d = f"M{x1},{yb} C{x1},{yb + 46} {x0},{yb + 46} {x0},{yb + 2}"
            self.arrow(d, color, dashed=True)
            self.text((x0 + x1) / 2, yb + 50, label, 12.5, 500, MUTED)
            return
        d = f"M{x1},{y} C{x1},{y - 46} {x0},{y - 46} {x0},{y - 2}"
        self.arrow(d, color, dashed=True)
        self.text((x0 + x1) / 2, y - 40, label, 12.5, 500, MUTED)

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


def objects(f, cards, problem_label, y=92, band_y=244):
    """The controller, the problem's cards, the loop, the parameter tabs and the tree."""
    ctl_x, prob_x, prob_w = 100, 300, 780
    f.frame(prob_x, y - 50, prob_w, CARD_H + 84, problem_label)

    # The controller, outside the problem, closing the loop on the problem's plant
    f.card(ctl_x, y, CARD_W, CARD_H, "Controller", ["u = π(y)"], "obj", lift=6)
    plant_x = cards[0][0]
    mid = (ctl_x + CARD_W + plant_x) / 2
    f.arrow(f"M{ctl_x + CARD_W},{y + 22} L{plant_x - 2},{y + 22}")
    f.text(mid, y + 15, "u", 14, 500, MUTED)
    f.arrow(f"M{plant_x},{y + 50} L{ctl_x + CARD_W + 2},{y + 50}")
    f.text(mid, y + 67, "y", 14, 500, MUTED)

    leaves = [(ctl_x + CARD_W / 2, [("p", "par")], "controller")]
    for x, title, lines, kind, tabs, key in cards:
        f.card(x, y, CARD_W, CARD_H, title, lines, kind, lift=6)
        leaves.append((x + CARD_W / 2, tabs, key))

    # The tree: one band, the branch of every object under its key
    left, right = ctl_x - 70, prob_x + prob_w
    par_stroke, par_ink = COLORS["par"][1], COLORS["par"][2]
    f.add(
        f'<rect x="{left}" y="{band_y}" width="{right - left}" height="46" rx="23" '
        f'fill="#ffffff" stroke="{par_stroke}" stroke-width="1.6"/>'
    )
    f.text(left + 36, band_y + 25, "θ", 22, 700, par_ink)
    f.text(left + 36, band_y + 39, "pe.params", 10.5, 500, par_ink)
    for cx, tabs, key in leaves:
        f.dotted(f"M{cx},{y + CARD_H + 12} L{cx},{band_y}", par_stroke)
        for i, (s, kind) in enumerate(tabs):
            f.pill(cx + (i - (len(tabs) - 1) / 2) * 58, y + CARD_H, s, kind)
        f.text(cx, band_y + 28, key, 14, 500, par_ink)
    return band_y + 46


def evaluator_frame(f, y, h, label):
    """The problem evaluator's box: one row, its steps inside."""
    f.frame(30, y - 24, 1120, h + 74, label, label_bottom=True)


def optional_export(f, objective_cx, y_obj, y, h, program, optimizer, band_bottom):
    """The optional export: the program over chosen leaves, its optimizer, θ* back into θ."""
    label = "optional export · pe.program(wrt): tune the chosen leaves of θ"
    f.frame(30, y - 40, 1120, h + 62, label, dashed=True)
    f.arrow(f"M{objective_cx},{y_obj} L{objective_cx},{y - 2}", dashed=True)
    f.card(830, y, 300, h, "Program", program)
    f.arrow(f"M830,{y + h / 2} L772,{y + h / 2}")
    f.card(470, y, 300, h, "Optimizer", optimizer)
    f.arrow(f"M470,{y + h / 2} L412,{y + h / 2}")
    f.pill(378, y + h / 2, "θ*", "out", w=62, h=34, size=16, weight=700)
    f.arrow(
        f"M347,{y + h / 2} L40,{y + h / 2} C14,{y + h / 2} 14,{y + h / 2 - 26} 14,{y + h / 2 - 40} "
        f"L14,{band_bottom + 40} C14,{band_bottom + 16} 22,{band_bottom + 8} 40,{band_bottom + 4}",
        COLORS["out"][1],
        dashed=True,
        marker="arrg",
    )


def objective(f, cx, cy, value, w=118):
    """The evaluator's output: the objective, and the record it comes with."""
    f.pill(cx, cy, value, "out", w=w, h=40, size=15, weight=700)
    f.text(cx, cy + 40, "pe.evaluate(θ)", 12.5, 500, COLORS["out"][2])
    f.text(cx, cy + 56, "→ Evaluation", 12.5, 500, COLORS["out"][2])


def deterministic():
    f = Figure(1180, 712)
    band_bottom = objects(
        f,
        [
            (330, "Plant", ["ẋ = f(x, u)"], "obj", [("p", "par")], "plant"),
            (520, "Cost", ["∫ g dt + h"], "obj", [("p", "par")], "cost"),
            (710, "Sets", ["X · U · Xf"], "obj", [("p", "par")], "X · U · Xf"),
            (900, "Start", ["x(0)"], "obj", [("x₀", "par")], "x_start"),
        ],
        "PlanningProblem",
    )

    # The problem evaluator: simulate the closed loop, score its trajectory
    r1, h = 362, 96
    evaluator_frame(f, r1, h, "ProblemEvaluator · pe = problem.compile(controller, dt)")
    f.arrow(f"M190,{band_bottom} L190,{r1 - 2}")
    f.card(
        60,
        r1,
        260,
        h,
        "Simulation",
        ["ẋ = f(x, π(y)),  x(0) = x₀", "integrated forward on [0, tf]"],
    )
    f.arrow(f"M320,{r1 + h / 2} L358,{r1 + h / 2}")
    f.trajectory(360, r1 + 4, 140, h - 8, "Trajectory", "x(t), u(t)")
    f.arrow(f"M500,{r1 + h / 2} L538,{r1 + h / 2}")
    f.card(
        540,
        r1,
        400,
        h,
        "Score",
        [
            "J = ∫ g(x, u) dt + h(x(tf))",
            "m = margin to X, U, Xf  (m < 0: a set is left)",
        ],
    )
    f.arrow(f"M940,{r1 + h / 2} L1008,{r1 + h / 2}")
    objective(f, 1070, r1 + h / 2 - 12, "J(θ), m(θ)")

    r2 = r1 + h + 112
    optional_export(
        f,
        1070,
        r1 + h / 2 + 52,
        r2,
        h,
        ["min J(θ)", "s.t.  m(θ) ≥ 0"],
        ["θ ← θ − η ∇J(θ)", "gd · Adam · L-BFGS · SLSQP · Ipopt"],
        band_bottom,
    )
    return f.svg()


def stochastic():
    f = Figure(1180, 718)
    band_bottom = objects(
        f,
        [
            (
                330,
                "Plant",
                ["ẋ = f(x, u, w)"],
                "obj",
                [("p", "par"), ("seed", "rnd")],
                "plant · seed",
            ),
            (520, "Cost", ["∫ g dt + h"], "obj", [("p", "par")], "cost"),
            (710, "Sets", ["X · U · Xf"], "obj", [("p", "par")], "X · U · Xf"),
            (
                900,
                "Distributions",
                ["x₀ ~ D₀ · p ~ Dₚ"],
                "rnd",
                [("p", "par")],
                "distributions",
            ),
        ],
        "StochasticPlanningProblem",
    )

    # The problem evaluator: n draws, n simulations, n trajectories, n scores, one criterion
    r1, h = 368, 96
    evaluator_frame(
        f,
        r1,
        h,
        "ProblemEvaluator · pe = problem.compile(controller, dt) · pe.evaluate(θ, key, n)",
    )
    f.pill(72, r1 + h / 2, "key", "rnd", w=48, h=28)
    f.arrow(f"M96,{r1 + h / 2} L108,{r1 + h / 2}", COLORS["rnd"][1], marker="arrp")
    f.arrow(f"M170,{band_bottom} L170,{r1 - 2}")
    f.card(110, r1, 120, h, "Draws", ["ω₁ … ωₙ", "x₀ᵢ, pᵢ, seedᵢ"])
    f.arrow(f"M230,{r1 + h / 2} L252,{r1 + h / 2}")
    f.card(
        254,
        r1,
        200,
        h,
        "Simulation × n",
        ["ẋ = f(x, π(y), wᵢ)", "x(0) = x₀ᵢ, one vmap"],
        stack=2,
    )
    f.arrow(f"M466,{r1 + h / 2} L488,{r1 + h / 2}")
    f.trajectory(490, r1 + 4, 124, h - 8, "Trajectories", "xᵢ(t), uᵢ(t)", stack=2)
    f.arrow(f"M626,{r1 + h / 2} L648,{r1 + h / 2}")
    f.card(
        650, r1, 190, h, "Scores", ["Jᵢ = ∫ g dt + h", "mᵢ = margin to sets"], stack=2
    )
    f.arrow(f"M852,{r1 + h / 2} L874,{r1 + h / 2}")
    f.card(876, r1, 140, h, "Criterion C", ["mean, CVaR", "or worst case"])
    f.arrow(f"M1016,{r1 + h / 2} L1032,{r1 + h / 2}")
    objective(f, 1090, r1 + h / 2 - 12, "C(J₁ … Jₙ)", w=112)

    r2 = r1 + h + 112
    optional_export(
        f,
        1090,
        r1 + h / 2 + 52,
        r2,
        h,
        ["min C(J₁ … Jₙ)", "s.t.  mᵢ(θ) ≥ 0"],
        ["θ ← θ − η ∇C(θ)", "Adam · L-BFGS · SLSQP · Ipopt"],
        band_bottom,
    )
    return f.svg()


def chain(f, y, h, items, x0=60, gap=44, reverse=False, arrows=True):
    """Lay cards along a row, left to right (or right to left), with arrows between.

    Each item is ``(kind, w, title, lines)``; ``kind`` is a colour role, ``"traj"``
    for a trajectory card, or ``"pill"`` for a result. Returns the ``(x, w)`` of each.
    """
    xs, x = [], x0
    for _kind, w, _title, _lines in items:
        xs.append((x, w))
        x += w + gap
    if reverse:
        right = xs[-1][0] + xs[-1][1]
        xs = [(right - (xx - x0) - w, w) for xx, w in xs]
    for (kind, w, title, lines), (x, _) in zip(items, xs):
        if kind == "traj":
            f.trajectory(x, y + 4, w, h - 8, title, lines[0])
        elif kind == "pill":
            f.pill(x + w / 2, y + h / 2, title, "out", w=w, h=40, size=15, weight=700)
        else:
            f.card(x, y, w, h, title, lines, kind)
    if arrows:
        for (a, wa), (b, wb) in zip(xs, xs[1:]):
            if reverse:
                f.arrow(f"M{a},{y + h / 2} L{b + wb + 2},{y + h / 2}")
            else:
                f.arrow(f"M{a + wa},{y + h / 2} L{b - 2},{y + h / 2}")
    return xs


def problem_row(f, cards, label, y=70, h=76):
    """The problem's box: its object cards in one row."""
    f.frame(30, y - 44, 1120, h + 66, label)
    w = (1120 - 40 - 24 * (len(cards) - 1)) / len(cards)
    for i, (title, lines, kind) in enumerate(cards):
        f.card(50 + i * (w + 24), y, w, h, title, lines, kind)
    return y + h + 22


def trajectory_optimization():
    f = Figure(1180, 520)
    bottom = problem_row(
        f,
        [
            ("Plant", ["ẋ = f(x, u)"], "obj"),
            ("Cost", ["∫ g dt + h(x(tf))"], "obj"),
            ("Sets", ["x ∈ X,  u ∈ U"], "obj"),
            ("Goal", ["x(tf) ∈ Xf"], "obj"),
            ("Start", ["x(0) = x₀"], "obj"),
        ],
        "PlanningProblem",
    )

    # The transcription: the trajectory itself is the decision
    y, h = bottom + 58, 96
    label = (
        "Transcription · direct collocation (or single, multiple shooting): "
        "no simulation, the dynamics become constraints"
    )
    f.frame(30, y - 34, 1120, h + 60, label)
    f.arrow(f"M590,{bottom} L590,{y - 36}")
    chain(
        f,
        y,
        h,
        [
            (
                "step",
                220,
                "Decision vector",
                ["z = (x₀ … xₙ, u₀ … uₙ)", "the knots of the trajectory"],
            ),
            (
                "step",
                300,
                "Dynamics as defects",
                ["xₖ₊₁ − xₖ − dt/2 (fₖ + fₖ₊₁) = 0", "on every interval"],
            ),
            (
                "step",
                250,
                "Cost as a sum",
                ["Σ dt/2 (gₖ + gₖ₊₁) + h(xₙ)", "the trapezoidal rule"],
            ),
            ("step", 210, "Sets as rows", ["xₖ ∈ X, uₖ ∈ U", "xₙ ∈ Xf"]),
        ],
        x0=50,
        gap=24,
        arrows=False,
    )

    # The program, its solver, and the plan read back
    y2 = y + h + 74
    f.arrow(f"M190,{y + h + 26} L190,{y2 - 2}")
    xs = chain(
        f,
        y2,
        h,
        [
            (
                "step",
                260,
                "MathematicalProgram",
                ["min J(z)", "s.t. h(z) = 0, g(z) ≥ 0"],
            ),
            ("step", 200, "Optimizer", ["SLSQP · Ipopt", "from a first guess z₀"]),
            ("traj", 150, "Trajectory", ["x*(t), u*(t)"]),
            (
                "obj",
                290,
                "PlanningSolution",
                ["policy u = π(t), open loop", "or tracked; MPC re-solves"],
            ),
        ],
        x0=60,
        gap=60,
    )
    f.text((xs[1][0] + xs[1][1] + xs[2][0]) / 2, y2 + h / 2 - 8, "z*", 13, 600, MUTED)
    return f.svg()


def dynamic_programming():
    f = Figure(1180, 580)
    bottom = problem_row(
        f,
        [
            ("Plant", ["ẋ = f(x, u)"], "obj"),
            ("Cost", ["g(x, u) dt,  h(x)"], "obj"),
            ("Sets", ["X, U: bounded boxes"], "obj"),
        ],
        "PlanningProblem",
    )

    # The discretization: nodes, actions and their successors
    y, h = bottom + 58, 96
    f.frame(
        30, y - 34, 1120, h + 60, "StateSpaceGrid · the continuous problem on a lattice"
    )
    f.arrow(f"M590,{bottom} L590,{y - 36}")
    xs = chain(
        f,
        y,
        h,
        [
            ("step", 320, "Nodes", ["x-grid over X", "N states"]),
            ("step", 320, "Actions", ["u-grid over U", "A inputs"]),
            (
                "step",
                360,
                "Successors",
                ["x⁺ = x + f(x, u) dt", "between nodes: interpolated"],
            ),
        ],
        x0=50,
        gap=40,
        arrows=False,
    )
    f.grid_icon(xs[0][0] + 24, y + 26)

    # The Bellman sweep, backward, then the greedy policy
    y2 = y + h + 70
    f.arrow(f"M420,{y + h + 26} L420,{y2 - 2}")
    xs = chain(
        f,
        y2,
        h,
        [
            ("step", 170, "Terminal", ["J(x, tf) = h(x)"]),
            (
                "step",
                320,
                "Bellman backup",
                ["Jₖ(x) = minᵤ [ g(x, u) dt", "+ α Jₖ₊₁(x⁺) ]"],
            ),
            ("obj", 220, "Cost-to-go", ["J(x) on the grid"]),
            (
                "obj",
                260,
                "PlanningSolution",
                ["policy u = π(x) = argminᵤ", "a lookup table, and J"],
            ),
        ],
        x0=50,
        gap=40,
    )
    f.loop_back(
        xs[1][0] + 60,
        xs[1][0] + xs[1][1] - 60,
        y2,
        "backward in time, until J converges",
        below=h,
    )
    f.heat_icon(xs[2][0] + 12, y2 + 30, size=7)
    return f.svg()


def lqr():
    f = Figure(1180, 470)
    problem_row(
        f,
        [
            ("Plant", ["ẋ = f(x, u)"], "obj"),
            ("QuadraticCost", ["g = (x − x̄)ᵀQ(x − x̄) + (u − ū)ᵀR(u − ū)"], "obj"),
        ],
        "PlanningProblem",
    )

    # The plant feeds the linearization, the cost's weights the Riccati equation
    y, h = 226, 96
    xs = chain(
        f,
        y,
        h,
        [
            (
                "step",
                290,
                "Linearize at (x̄, ū)",
                ["A = ∂f/∂x,  B = ∂f/∂u", "the model near the goal"],
            ),
            (
                "step",
                380,
                "Riccati",
                ["AᵀP + PA − PBR⁻¹BᵀP + Q = 0", "(finite tf: the ODE backward from S)"],
            ),
            ("step", 290, "Gain", ["K = R⁻¹BᵀP", "(K(t) on a finite horizon)"]),
        ],
        x0=60,
        gap=60,
    )
    f.arrow(f"M{xs[0][0] + 145},168 L{xs[0][0] + 145},{y - 2}")
    f.arrow(f"M{xs[1][0] + 190},168 L{xs[1][0] + 190},{y - 2}")
    f.text(xs[1][0] + 200, 200, "Q, R", 13, 600, MUTED, anchor="start")

    # The solution from the gain; the cost-to-go and the poles beside it
    y2 = y + h + 46
    f.card(
        xs[1][0],
        y2,
        380,
        h,
        "Cost-to-go",
        ["J(x) = (x − x̄)ᵀP(x − x̄)", "of the linear model"],
        "obj",
    )
    f.arrow(f"M{xs[1][0] + 190},{y + h} L{xs[1][0] + 190},{y2 - 2}")
    f.card(
        xs[2][0],
        y2,
        290,
        h,
        "PlanningSolution",
        ["policy u = ū − K(x − x̄)", "poles eig(A − BK)"],
        "obj",
    )
    f.arrow(f"M{xs[2][0] + 145},{y + h} L{xs[2][0] + 145},{y2 - 2}")
    return f.svg()


def reinforcement_learning():
    f = Figure(1180, 576)
    bottom = problem_row(
        f,
        [
            ("Plant", ["ẋ = f(x, u, w)"], "obj"),
            ("Cost", ["reward r = −g dt"], "obj"),
            ("Sets", ["leave X: a failure"], "obj"),
            ("Distributions", ["x₀ ~ D₀,  p ~ Dₚ"], "rnd"),
        ],
        "StochasticPlanningProblem",
    )

    y, h = bottom + 58, 96
    f.frame(
        30, y - 34, 1120, h + 60, "RolloutEnvironment · the step model of the problem"
    )
    f.arrow(f"M590,{bottom} L590,{y - 36}")
    chain(
        f,
        y,
        h,
        [
            ("step", 330, "Step", ["xₖ₊₁ = Φ(xₖ, uₖ)", "the plant held over dt"]),
            (
                "step",
                330,
                "Reward and end",
                ["rₖ = −g(xₖ, uₖ) dt", "done when x ∉ X, or at the horizon"],
            ),
            ("rnd", 340, "Reset", ["x₀ drawn from D₀", "one seed per episode"]),
        ],
        x0=50,
        gap=40,
        arrows=False,
    )

    # The learning loop: collect with the current policy, then update it
    y2 = y + h + 70
    f.arrow(f"M410,{y + h + 26} L410,{y2 - 2}")
    xs = chain(
        f,
        y2,
        h,
        [
            ("step", 220, "Policy πθ", ["u ~ πθ(· | x)", "a neural network"]),
            ("step", 220, "Collect", ["episodes, one scan", "vmap over envs"]),
            ("step", 220, "Returns", ["Gₖ = Σⱼ γʲ rₖ₊ⱼ", "advantages, critic"]),
            ("step", 230, "Update θ", ["θ ← θ + η ∇θ E[G]", "PPO · SAC · REINFORCE"]),
        ],
        x0=50,
        gap=30,
    )
    f.loop_back(
        xs[0][0] + xs[0][1] / 2,
        xs[3][0] + xs[3][1] / 2,
        y2,
        "repeat: collect with the updated policy",
        below=h,
    )
    f.arrow(f"M{xs[3][0] + xs[3][1]},{y2 + h / 2} L1053,{y2 + h / 2}")
    f.pill(1094, y2 + h / 2, "π(x)", "out", w=78, h=40, size=15, weight=700)
    f.text(1094, y2 + h / 2 + 40, "PlanningSolution", 12.5, 500, COLORS["out"][2])
    return f.svg()


def modeling_and_simulation():
    f = Figure(1180, 540)
    # The model: Controller @ Plant = one closed-loop System
    y, h = 70, 86
    f.frame(
        30,
        y - 44,
        1120,
        h + 66,
        "Model · every block a System, and so is their diagram",
    )
    f.card(60, y, 230, h, "Controller", ["u = π(y; p)"], "obj")
    f.text(318, y + h / 2 + 10, "@", 28, 700, MUTED)
    f.card(346, y, 230, h, "Plant", ["ẋ = f(x, u, t; p)", "y = h(x, u, t; p)"], "obj")
    f.text(606, y + h / 2 + 10, "=", 30, 700, MUTED)
    lines = ["Ẋ = F(X, U, t; P)", "operators @ >> +, or connect"]
    f.card(636, y, 484, h, "Closed-loop diagram, a System", lines, "obj")

    # Compile once
    y2 = y + h + 92
    f.frame(30, y2 - 34, 1120, h + 60, "compile(backend): the diagram lowered once")
    f.arrow(f"M860,{y + h + 22} L860,{y2 - 36}")
    chain(
        f,
        y2,
        h,
        [
            (
                "step",
                330,
                "ExecutionPlan",
                ["block order, wires, state slices", "algebraic loops refused"],
            ),
            (
                "step",
                340,
                "Evaluator",
                ["f, h, rollout as arrays", "NumPy, or JAX: jit · grad · vmap"],
            ),
            (
                "step",
                330,
                "Parameters",
                ["params: one tree for all blocks", "swap any leaf, no recompile"],
            ),
        ],
        x0=50,
        gap=40,
        arrows=False,
    )

    # What uses it: the simulator and the analysis tools
    y3 = y2 + h + 80
    f.arrow(f"M190,{y2 + h + 26} L190,{y3 - 2}")
    f.arrow(f"M975,{y2 + h + 26} L975,{y3 - 2}")
    chain(
        f,
        y3,
        h,
        [
            (
                "step",
                260,
                "Simulator",
                ["integrate ẋ = f from x₀", "solve_ivp or fixed-step RK4"],
            ),
            ("traj", 150, "Trajectory", ["x(t), u(t), y(t)"]),
            ("out", 170, "Plots, animation", ["plot_trajectory", "animate"]),
        ],
        x0=60,
        gap=40,
    )
    f.card(
        830,
        y3,
        290,
        h,
        "Analysis",
        ["linearize: A, B, C, D", "poles · Bode · margins"],
        "step",
    )
    return f.svg()


def overview():
    f = Figure(1180, 450)
    y, h = 30, 70
    f.card(
        390,
        y,
        400,
        h,
        "PlanningProblem",
        ["plant · cost · sets · start · horizon"],
        "obj",
    )

    # Five planners, one problem, one kind of solution
    y2 = y + h + 64
    y3 = y2 + 92 + 76
    solution_cx = 260 + 125
    planners = [
        ("LQR", ["linearize + Riccati", "u = ū − K(x − x̄)"]),
        ("Dynamic programming", ["Bellman on a grid", "u = π(x), J(x)"]),
        ("Trajectory optimization", ["transcription + NLP", "u = π(t), x*(t)"]),
        ("Reinforcement learning", ["collect + update θ", "u = π(x; θ)"]),
        ("RRT", ["kinodynamic tree", "u = π(t)"]),
    ]
    w, gap = 200, 20
    for i, (title, lines) in enumerate(planners):
        x = 50 + i * (w + gap)
        cx = x + w / 2
        f.arrow(f"M590,{y + h} C590,{y + h + 36} {cx},{y2 - 36} {cx},{y2 - 2}")
        f.card(x, y2, w, 92, title, lines, "step")
        down = f"M{cx},{y2 + 92} C{cx},{y2 + 130} {solution_cx},{y3 - 40} {solution_cx},{y3 - 2}"
        f.arrow(down)

    chain(
        f,
        y3,
        96,
        [
            (
                "obj",
                250,
                "PlanningSolution",
                ["policy · trajectory", "and the solver's log"],
            ),
            (
                "obj",
                230,
                "Closed loop",
                ["solution.policy @ plant", "you connect it"],
            ),
            (
                "step",
                300,
                "ProblemEvaluator",
                ["simulate, score J and margins", "→ Evaluation, compare(…)"],
            ),
        ],
        x0=260,
        gap=50,
    )
    return f.svg()


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
    "Analyze": ("#ecfdf5", "#059669", "#064e3b"),
    "Design": ("#f7fee7", "#65a30d", "#365314"),
    # The problem's tools, warm
    "Plan": ("#fff7ed", "#ea580c", "#7c2d12"),
    "Learn": ("#fff1f2", "#e11d48", "#881337"),
    "Evaluate": ("#fdf4ff", "#c026d3", "#701a75"),
}
RESULT = {
    "Simulate": "Trajectory",
    "Analyze": "Poles · Bode",
    "Design": "Controller",
    "Plan": "PlanningSolution",
    "Learn": "Neural policy",
    "Evaluate": "Evaluation",
}
BLUE = COLORS["obj"]


def markers(f):
    """One arrowhead per accent, and one for the loop back (System blue)."""
    items = [(f"m{k}", v[1]) for k, v in ACCENT.items()] + [("mblue", BLUE[1])]
    f.add(
        "<defs>"
        + "".join(
            f'<marker id="{mid}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
            f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" '
            f'fill="{c}"/></marker>'
            for mid, c in items
        )
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


PICS = {
    "Simulate": pic_trajectory,
    "Analyze": pic_bode,
    "Design": pic_poles,
    "Plan": pic_path,
    "Learn": pic_network,
    "Evaluate": pic_histogram,
}


def pic_diagram(f, x, y, w, h, c):
    """A controller and a plant closed in a loop."""
    bw, bh = w * 0.26, h * 0.46
    b1x, b2x, by = x + w * 0.16, x + w * 0.6, y + h * 0.04
    for bx in (b1x, b2x):
        f.add(
            f'<rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="5" fill="#ffffff" '
            f'stroke="{c}" stroke-width="2"/>'
        )
    for bx, s_ in ((b1x, "π"), (b2x, "f")):
        f.text(bx + bw / 2, by + bh / 2 + 5, s_, 14, 600, c)
    my, fy = by + bh / 2, y + h * 0.88
    f.add(
        f'<path d="M{b1x + bw},{my} L{b2x - 2},{my}" stroke="{c}" stroke-width="2" '
        f'marker-end="url(#mblue)"/>'
    )
    f.add(
        f'<path d="M{b2x + bw},{my} L{x + w},{my} L{x + w},{fy} L{x},{fy} L{x},{my} '
        f'L{b1x - 2},{my}" fill="none" stroke="{c}" stroke-width="2" stroke-linejoin="round" '
        f'marker-end="url(#mblue)"/>'
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


def library_map():
    """The whole library on one page: three foundations, six tools as coloured lanes, JAX beside."""
    order = list(ACCENT)
    x0, w, gap = 58, 144, 14
    cols = {name: x0 + i * (w + gap) for i, name in enumerate(order)}
    right = x0 + 6 * w + 5 * gap
    px0, px1 = cols["Plan"], right
    sys_y, sys_h = 30, 80
    py, ph = sys_y + sys_h + 36, 120
    ry, rh = py + ph + 36, 92
    oy, oh = ry + rh + 30, 104
    bus_y = oy + oh + 28
    by, bh = bus_y + 44, 88
    f = Figure(1180, by + bh + 30)
    markers(f)

    foundation(
        f,
        x0,
        sys_y,
        right - x0,
        sys_h,
        "System",
        [r"$\dot{x} = f(x, u, t;\, p) \qquad y = h(x, u, t;\, p)$"],
        pic_diagram,
        150,
        21,
    )
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

    # Six lanes: the tool, then what it gives back, in one colour
    for name, x in cols.items():
        fill, stroke, ink = ACCENT[name]
        cx = x + w / 2
        f.add(
            f'<rect x="{x - 4}" y="{ry - 8}" width="{w + 8}" height="{oy + oh - ry + 16}" rx="14" '
            f'fill="{fill}"/>'
        )
        top = sys_y + sys_h if name in ("Simulate", "Analyze", "Design") else py + ph
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
        f.math(cx, ry + 62, TOOL_TEX[name], 17, "#0f172a", max_width=w - 28)
        f.arrow(f"M{cx},{ry + rh} L{cx},{oy - 2}", stroke, marker=f"m{name}", width=2)
        f.add(
            f'<rect x="{x}" y="{oy}" width="{w}" height="{oh}" rx="10" fill="#ffffff" '
            f'stroke="{stroke}" stroke-width="1.6"/>'
        )
        f.text(cx, oy + 24, RESULT[name], 14.5, 650, ink)
        PICS[name](f, x + 24, oy + 40, w - 48, oh - 54, stroke)

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
    jx = right + 24
    jax_column(f, jx, sys_y, 1150 - jx, by + bh - sys_y)
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
    "deterministic": deterministic,
    "stochastic": stochastic,
    "trajectory_optimization": trajectory_optimization,
    "dynamic_programming": dynamic_programming,
    "lqr": lqr,
    "reinforcement_learning": reinforcement_learning,
    "modeling_and_simulation": modeling_and_simulation,
    "overview": overview,
    "map": library_map,
}


if __name__ == "__main__":
    static = Path(__file__).resolve().parent / "_static"
    for name, draw in FIGURES.items():
        (static / f"pipeline_{name}.svg").write_text(draw())
