"""Draw the two evaluator-pipeline figures of DESIGN §6 as hand-laid SVG.

Run from the repository root: ``python docs/pipeline_figures.py``; it writes
``docs/_static/pipeline_deterministic.svg`` and ``pipeline_stochastic.svg``.
"""

from pathlib import Path

FONT = "Inter, 'Helvetica Neue', Helvetica, Arial, sans-serif"

# (fill, stroke, ink) per role
COLORS = {
    "obj": ("#e0f2fe", "#0284c7", "#0c4a6e"),
    "par": ("#fef3c7", "#d97706", "#78350f"),
    "rnd": ("#f3e8ff", "#9333ea", "#581c87"),
    "step": ("#ffffff", "#94a3b8", "#0f172a"),
    "out": ("#dcfce7", "#16a34a", "#14532d"),
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

    def frame(self, x, y, w, h, label):
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="#f8fafc" '
            f'stroke="#cbd5e1" stroke-width="1.2"/>'
        )
        self.text(x + 16, y + 22, label, 13.5, 600, MUTED, anchor="start")

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

    def svg(self):
        markers = "".join(
            f'<marker id="{mid}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
            f'markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" '
            f'fill="{color}"/></marker>'
            for mid, color in (
                ("arr", LINE),
                ("arrg", COLORS["out"][1]),
                ("arrp", COLORS["rnd"][1]),
            )
        )
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.width} {self.height}" '
            f'width="{self.width}" height="{self.height}" font-family="{FONT}">'
            f'<defs>{markers}</defs><rect width="100%" height="100%" fill="#ffffff"/>'
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

    leaves = [(ctl_x + CARD_W / 2, [("θc", "par")], "controller")]
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


def deterministic():
    f = Figure(1180, 640)
    band_bottom = objects(
        f,
        [
            (330, "Plant", ["ẋ = f(x, u)"], "obj", [("θp", "par")], "plant"),
            (520, "Cost", ["∫ g dt + h"], "obj", [("θg", "par")], "cost"),
            (710, "Sets", ["X · U · Xf"], "obj", [("θs", "par")], "X · U · Xf"),
            (900, "Start", ["x(0)"], "obj", [("x₀", "par")], "x_start"),
        ],
        "PlanningProblem",
    )

    # Forward: simulate the closed loop, then score its trajectory
    r1, h = 372, 96
    f.text(
        246,
        r1 - 30,
        "pe = problem.compile(controller, dt)",
        13.5,
        600,
        MUTED,
        anchor="start",
    )
    f.arrow(f"M{100 + 130},{band_bottom} L{100 + 130},{r1 - 2}")
    f.card(
        100,
        r1,
        260,
        h,
        "Simulation",
        ["ẋ = f(x, π(y)),  x(0) = x₀", "integrated forward on [0, tf]"],
    )
    f.arrow(f"M360,{r1 + h / 2} L408,{r1 + h / 2}")
    f.trajectory(410, r1 + 4, 150, h - 8, "Trajectory", "x(t), u(t)")
    f.arrow(f"M560,{r1 + h / 2} L608,{r1 + h / 2}")
    f.card(
        610,
        r1,
        470,
        h,
        "Score",
        [
            "J = ∫ g(x, u) dt + h(x(tf)),  over [0, tf]",
            "m = margin of x(t), u(t) to X, U, Xf  (m < 0: a set is left)",
        ],
    )

    # Back: the program, the optimizer, the tuned leaves into θ
    r2 = r1 + h + 44
    f.arrow(f"M{845},{r1 + h} L{845},{r2 - 2}")
    f.card(
        700,
        r2,
        290,
        h,
        "Program",
        ["min J(θ)  over chosen leaves of θ", "s.t.  m(θ) ≥ 0"],
    )
    f.arrow(f"M700,{r2 + h / 2} L632,{r2 + h / 2}")
    f.card(
        330,
        r2,
        300,
        h,
        "Optimizer",
        ["θ ← θ − η ∇J(θ)", "gd · Adam · L-BFGS · SLSQP · Ipopt"],
    )
    f.arrow(f"M330,{r2 + h / 2} L238,{r2 + h / 2}")
    f.pill(205, r2 + h / 2, "θ*", "out", w=62, h=34, size=16, weight=700)
    f.arrow(
        f"M174,{r2 + h / 2} C60,{r2 + h / 2} 52,{band_bottom + 60} 52,{band_bottom + 4}",
        COLORS["out"][1],
        dashed=True,
        marker="arrg",
    )
    f.text(66, r2 - 8, "tuned leaves", 12.5, 500, COLORS["out"][2], anchor="start")
    f.text(66, r2 + 8, "back in θ", 12.5, 500, COLORS["out"][2], anchor="start")
    return f.svg()


def stochastic():
    f = Figure(1180, 660)
    band_bottom = objects(
        f,
        [
            (
                330,
                "Plant",
                ["ẋ = f(x, u, w)"],
                "obj",
                [("θp", "par"), ("seed", "rnd")],
                "plant · seed",
            ),
            (520, "Cost", ["∫ g dt + h"], "obj", [("θg", "par")], "cost"),
            (710, "Sets", ["X · U · Xf"], "obj", [("θs", "par")], "X · U · Xf"),
            (900, "Laws", ["x₀ ~ X0 · θp ~ Θ"], "rnd", [("θℓ", "par")], "laws"),
        ],
        "StochasticPlanningProblem",
    )

    # Forward: n draws, n simulations, n trajectories, n scores
    r1, h = 382, 96
    f.text(
        220,
        r1 - 34,
        "pe = problem.compile(controller, dt) · pe.evaluate(θ, key, n)",
        13.5,
        600,
        MUTED,
        anchor="start",
    )
    f.pill(90, r1 + h / 2, "key", "rnd", w=52, h=30)
    f.arrow(f"M116,{r1 + h / 2} L128,{r1 + h / 2}", COLORS["rnd"][1], marker="arrp")
    f.arrow(f"M{130 + 75},{band_bottom} L{130 + 75},{r1 - 2}")
    f.card(130, r1, 150, h, "Draws", ["ω₁ … ωₙ", "x₀ᵢ, θpᵢ, seedᵢ"])
    f.arrow(f"M280,{r1 + h / 2} L304,{r1 + h / 2}")
    f.card(
        306,
        r1,
        290,
        h,
        "Simulation × n",
        ["ẋ = f(x, π(y), wᵢ),  x(0) = x₀ᵢ", "integrated forward, one vmap"],
        stack=2,
    )
    f.arrow(f"M608,{r1 + h / 2} L638,{r1 + h / 2}")
    f.trajectory(640, r1 + 4, 140, h - 8, "Trajectories", "xᵢ(t), uᵢ(t)", stack=2)
    f.arrow(f"M792,{r1 + h / 2} L818,{r1 + h / 2}")
    f.card(
        820,
        r1,
        270,
        h,
        "Scores",
        ["Jᵢ = ∫ g dt + h", "mᵢ = margin to X, U, Xf  (< 0: left)"],
        stack=2,
    )

    # Back: the criterion, the program, the optimizer, the tuned leaves into θ
    r2 = r1 + h + 50
    f.arrow(f"M{955},{r1 + h} L{955},{r2 - 2}")
    f.card(840, r2, 240, h, "Criterion C", ["mean · CVaR · worst", "over J₁ … Jₙ"])
    f.arrow(f"M840,{r2 + h / 2} L782,{r2 + h / 2}")
    f.card(560, r2, 220, h, "Program", ["min C(J₁ … Jₙ)", "s.t.  mᵢ(θ) ≥ 0"])
    f.arrow(f"M560,{r2 + h / 2} L502,{r2 + h / 2}")
    f.card(
        250,
        r2,
        250,
        h,
        "Optimizer",
        ["θ ← θ − η ∇C(θ)", "Adam · L-BFGS · SLSQP · Ipopt"],
    )
    f.arrow(f"M250,{r2 + h / 2} L208,{r2 + h / 2}")
    f.pill(175, r2 + h / 2, "θ*", "out", w=62, h=34, size=16, weight=700)
    f.arrow(
        f"M144,{r2 + h / 2} C12,{r2 + h / 2} 12,{band_bottom + 50} 40,{band_bottom + 4}",
        COLORS["out"][1],
        dashed=True,
        marker="arrg",
    )
    return f.svg()


if __name__ == "__main__":
    static = Path(__file__).resolve().parent / "_static"
    (static / "pipeline_deterministic.svg").write_text(deterministic())
    (static / "pipeline_stochastic.svg").write_text(stochastic())
