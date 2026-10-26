"""Draw the two evaluator-pipeline figures of DESIGN §6 as hand-laid SVG.

Run from the repository root: ``python docs/pipeline_figures.py``; it writes
``docs/_static/pipeline_deterministic.svg`` and ``pipeline_stochastic.svg``.
"""

from pathlib import Path

FONT = "Inter, 'Helvetica Neue', Helvetica, Arial, sans-serif"
C = {
    "obj": ("#e0f2fe", "#0284c7", "#0c4a6e"),
    "par": ("#fef3c7", "#d97706", "#78350f"),
    "rnd": ("#f3e8ff", "#9333ea", "#581c87"),
    "step": ("#ffffff", "#94a3b8", "#0f172a"),
    "out": ("#dcfce7", "#16a34a", "#14532d"),
}
INK, MUTED, LINE = "#0f172a", "#64748b", "#64748b"


class Fig:
    def __init__(self, w, h):
        self.w, self.h, self.parts = w, h, []

    def add(self, s):
        self.parts.append(s)

    def text(
        self, x, y, s, size=15, weight=400, color=INK, anchor="middle", italic=False
    ):
        style = ' font-style="italic"' if italic else ""
        self.add(
            f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{color}" text-anchor="{anchor}"{style}>{s}</text>'
        )

    def card(self, x, y, w, h, title, lines, kind="obj", r=8, stack=0):
        fill, stroke, ink = C[kind]
        for k in range(stack, 0, -1):  # a stack of cards behind, for "× n"
            self.add(
                f'<rect x="{x + 5 * k}" y="{y - 5 * k}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="1.2" opacity="0.8"/>'
            )
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>'
        )
        n = 1 + len(lines)
        y0 = y + h / 2 - (n - 1) * 10 + 5 - (6 if kind in ("obj", "rnd") else 0)
        self.text(x + w / 2, y0, title, 15, 650, ink)
        for i, s in enumerate(lines):
            self.text(x + w / 2, y0 + 20 * (i + 1), s, 14, 400, ink)

    def pill(self, cx, cy, s, kind="par", w=None, h=24, size=13.5, weight=600):
        fill, stroke, ink = C[kind]
        w = w or max(36, 9 * len(s) + 18)
        self.add(
            f'<rect x="{cx - w / 2}" y="{cy - h / 2}" width="{w}" height="{h}" rx="{h / 2}" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>'
        )
        self.text(cx, cy + 4.5, s, size, weight, ink)

    def arrow(self, d, color=LINE, dashed=False, width=1.6, marker="arr"):
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        self.add(
            f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash} marker-end="url(#{marker})"/>'
        )

    def line(self, d, color, dashed=True, width=1.4):
        dash = ' stroke-dasharray="3 3"' if dashed else ""
        self.add(
            f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash}/>'
        )

    def frame(self, x, y, w, h, label):
        self.add(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="#f8fafc" stroke="#cbd5e1" stroke-width="1.2"/>'
        )
        self.text(x + 16, y + 22, label, 13.5, 600, MUTED, anchor="start")

    def svg(self):
        defs = "".join(
            f'<marker id="{mid}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{col}"/></marker>'
            for mid, col in (
                ("arr", LINE),
                ("arrg", C["out"][1]),
                ("arrp", C["rnd"][1]),
            )
        )
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" width="{self.w}" height="{self.h}" font-family="{FONT}">'
            f'<defs>{defs}</defs><rect width="100%" height="100%" fill="#ffffff"/>'
            + "".join(self.parts)
            + "</svg>"
        )


def objects(f, cards, ctl_x, prob_x, prob_w, y, band_y, prob_label):
    """The controller, the problem's cards, the loop, the tabs and the tree band."""
    cw, ch = 150, 76
    f.frame(prob_x, y - 50, prob_w, ch + 84, prob_label)
    # The controller, outside the problem, closing the loop on its plant
    f.card(ctl_x, y, cw, ch, "Controller", ["u = π(y)"])
    plant_x = cards[0][0]
    f.arrow(f"M{ctl_x + cw},{y + 22} L{plant_x - 2},{y + 22}")
    f.text((ctl_x + cw + plant_x) / 2, y + 15, "u", 14, 500, MUTED)
    f.arrow(f"M{plant_x},{y + 50} L{ctl_x + cw + 2},{y + 50}")
    f.text((ctl_x + cw + plant_x) / 2, y + 67, "y", 14, 500, MUTED)
    leaves = [(ctl_x + cw / 2, [("θc", "par")], "controller")]
    for x, title, lines, kind, tabs, key in cards:
        f.card(x, y, cw, ch, title, lines, kind)
        leaves.append((x + cw / 2, tabs, key))
    # The tree: one band, a leaf under every object
    left = ctl_x - 70
    right = prob_x + prob_w
    f.add(
        f'<rect x="{left}" y="{band_y}" width="{right - left}" height="46" rx="23" fill="#fffbeb" stroke="{C["par"][1]}" stroke-width="1.5"/>'
    )
    f.text(left + 36, band_y + 25, "θ", 22, 700, C["par"][2])
    f.text(left + 36, band_y + 39, "pe.params", 10.5, 500, C["par"][2])
    for cx, tabs, key in leaves:
        if not tabs:
            continue
        f.line(f"M{cx},{y + ch + 12} L{cx},{band_y}", C["par"][1])
        n = len(tabs)
        for i, (s, kind) in enumerate(tabs):
            f.pill(cx + (i - (n - 1) / 2) * 58, y + ch, s, kind, w=50)
        f.text(cx, band_y + 28, key, 14, 500, C["par"][2])
    return left, right


def deterministic():
    f = Fig(1180, 480)
    y, band_y = 92, 244
    cards = [
        (330, "Plant", ["ẋ = f(x, u)"], "obj", [("θp", "par")], "plant"),
        (520, "Cost", ["∫ g dt + h"], "obj", [("θg", "par")], "cost"),
        (710, "Sets", ["X · U · Xf"], "obj", [("θs", "par")], "X · U · Xf"),
        (900, "Start", ["x(0)"], "obj", [("x₀", "par")], "x_start"),
    ]
    left, right = objects(f, cards, 100, 300, 780, y, band_y, "PlanningProblem")
    # The pipeline
    py = 368
    f.text(
        190,
        py - 26,
        "pe = problem.compile(controller, dt)",
        13.5,
        600,
        MUTED,
        anchor="start",
    )
    f.arrow(f"M175,{band_y + 46} L175,{py - 2}")
    boxes = [
        (100, 150, "Rollout", ["x(t), u(t)"], "step"),
        (290, 150, "Score", ["J(θ), m(θ)"], "step"),
        (480, 190, "Program", ["min J(θ)", "s.t. m(θ) ≥ 0"], "step"),
        (710, 150, "Optimizer", ["over wrt ⊂ θ"], "step"),
    ]
    for i, (x, w, t, ls, k) in enumerate(boxes):
        f.card(x, py, w, 76, t, ls, k)
        if i:
            px, pw = boxes[i - 1][0], boxes[i - 1][1]
            f.arrow(f"M{px + pw},{py + 38} L{x - 2},{py + 38}")
    f.arrow(f"M860,{py + 38} L918,{py + 38}")
    f.pill(950, py + 38, "θ*", "out", w=60, h=34, size=16, weight=700)
    # θ* written back into the tree
    f.arrow(
        f"M980,{py + 38} C1040,{py + 38} 1040,{band_y + 70} 1040,{band_y + 48}",
        C["out"][1],
        dashed=True,
        marker="arrg",
    )
    f.text(1050, py - 30, "the tuned leaves", 12.5, 500, C["out"][2], anchor="start")
    f.text(1050, py - 14, "back in θ", 12.5, 500, C["out"][2], anchor="start")
    return f.svg()


def stochastic():
    f = Fig(1240, 500)
    y, band_y = 92, 244
    cards = [
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
    ]
    left, right = objects(
        f, cards, 100, 300, 780, y, band_y, "StochasticPlanningProblem"
    )
    py = 380
    f.text(
        180,
        py - 28,
        "pe = problem.compile(controller, dt) · pe.evaluate(θ, key, n)",
        13.5,
        600,
        MUTED,
        anchor="start",
    )
    f.pill(left + 30, py + 38, "key", "rnd", w=52, h=30)
    f.arrow(f"M{left + 56},{py + 38} L98,{py + 38}", C["rnd"][1], marker="arrp")
    f.arrow(f"M165,{band_y + 46} L165,{py - 2}")
    boxes = [
        (100, 130, "Realize", ["ω₁ … ωₙ"], 0),
        (262, 130, "Rollout × n", ["one vmap"], 2),
        (430, 120, "Scores", ["Jᵢ, mᵢ"], 0),
        (582, 160, "Criterion C", ["mean · CVaR · worst"], 0),
        (774, 180, "Program", ["min C(J₁ … Jₙ)", "s.t. mᵢ(θ) ≥ 0"], 0),
        (986, 110, "Optimizer", ["over wrt"], 0),
    ]
    for i, (x, w, t, ls, st) in enumerate(boxes):
        f.card(x, py, w, 76, t, ls, "step", stack=st)
        if i:
            px, pw = boxes[i - 1][0], boxes[i - 1][1]
            f.arrow(f"M{px + pw},{py + 38} L{x - 2},{py + 38}")
    f.arrow(f"M1096,{py + 38} L1132,{py + 38}")
    f.pill(1160, py + 38, "θ*", "out", w=52, h=34, size=16, weight=700)
    f.arrow(
        f"M1160,{py + 20} C1160,{band_y + 90} 1110,{band_y + 23} {right + 4},{band_y + 23}",
        C["out"][1],
        dashed=True,
        marker="arrg",
    )
    return f.svg()


if __name__ == "__main__":
    static = Path(__file__).resolve().parent / "_static"
    (static / "pipeline_deterministic.svg").write_text(deterministic())
    (static / "pipeline_stochastic.svg").write_text(stochastic())
