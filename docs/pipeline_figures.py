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
            (330, "Plant", ["ẋ = f(x, u)"], "obj", [("θp", "par")], "plant"),
            (520, "Cost", ["∫ g dt + h"], "obj", [("θg", "par")], "cost"),
            (710, "Sets", ["X · U · Xf"], "obj", [("θs", "par")], "X · U · Xf"),
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
                [("θp", "par"), ("seed", "rnd")],
                "plant · seed",
            ),
            (520, "Cost", ["∫ g dt + h"], "obj", [("θg", "par")], "cost"),
            (710, "Sets", ["X · U · Xf"], "obj", [("θs", "par")], "X · U · Xf"),
            (
                900,
                "Distributions",
                ["x₀ ~ X0 · θp ~ Θ"],
                "rnd",
                [("θd", "par")],
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
    f.card(110, r1, 120, h, "Draws", ["ω₁ … ωₙ", "x₀ᵢ, θpᵢ, seedᵢ"])
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


if __name__ == "__main__":
    static = Path(__file__).resolve().parent / "_static"
    (static / "pipeline_deterministic.svg").write_text(deterministic())
    (static / "pipeline_stochastic.svg").write_text(stochastic())
