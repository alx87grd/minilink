"""Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the plans."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.graphical.figure import (
    TAB10,
    FigureSpec,
    Panel,
    RefLine,
    Trace,
    draw,
    side_by_side,
)
from minilink.graphical.signals.time_signals import TIME_ABSCISSA_LABEL, signal_figure
from minilink.planning.evaluation import Evaluation, evaluate_trajectory
from minilink.planning.evaluator import ProblemEvaluator
from minilink.planning.plotting import (
    controller_law_figure,
    cost_to_go_panel,
    sample_field,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import (
    PlanningSolution,
    controller_run_of,
    plain_plant_run_of,
    require_cost_to_go,
)
from minilink.planning.step_model import is_continuous

if TYPE_CHECKING:
    from minilink.graphical.common.plot_result import PlotResult

# The problem's marks in greys, which no solution takes: the input limits and the
# horizon in grey, the goal in dark grey; one colour per solution, the same on every row
LIMIT_COLOR = "#7f7f7f"
GOAL_COLOR = "#333333"
SOLUTION_COLORS = tuple(color for color in TAB10 if color != LIMIT_COLOR)

# An answer's runs drawn one per draw: thin and faint, so their spread reads
FAINT_WIDTH = 0.6
FAINT_ALPHA = 0.12


###############################################################################
# Public API
###############################################################################


def compare(
    *args,
    dt=None,
    n_trials=None,
    key=0,
    solver="rk4",
    substeps=None,
    backend="auto",
    episode_length=None,
    **entries,
) -> Comparison:
    """
    Name the solutions of one problem side by side, and score them with one evaluation.

    ``compare(VI=vi, LQR=lqr)`` (or ``compare({"VI": vi, "LQR": lqr})``) names
    solutions for the table and the overlays. With the problem first,
    ``compare(problem, PID=pid, VI=vi, dt=0.01)`` also scores every entry by
    :func:`~minilink.planning.evaluator.evaluate` on the same draws (one ``key``,
    one backend for all, so the trials are paired): an entry is a
    :class:`~minilink.planning.results.PlanningSolution`, a controller block, or
    an :class:`~minilink.planning.evaluation.Evaluation` already computed.
    ``dt`` follows :func:`~minilink.planning.evaluator.evaluate`: a sampled
    controller (``pid % 0.01``) runs on its own clock, ``dt`` omitted; a
    continuous one runs on ``dt``, the period its loop is integrated on, and
    with ``dt`` omitted on the evaluator's automatic step. Each evaluation
    keeps the judge's runs, one per trial (``evaluation.trajectories``), which
    the figure draws: memory grows with ``n_trials`` (a pendulum's loop runs
    with their recorded signals, 5 s at ``dt = 0.01``: 17 MB for 4 answers ×
    100 draws).
    """
    problem, named = split_entries(args, entries)

    # Names only: the solutions, read as they are
    if problem is None:
        require_no_scoring(dt, n_trials)
        require_solutions(named)
        return Comparison(named)

    # Every entry scored on the same draws, its runs kept, its solution kept for the
    # table and the overlays (each loop's period as evaluate reads it: a sampled
    # one's own clock, else dt)
    evaluations = score_entries(
        problem,
        named,
        dict(
            dt=dt,
            solver=solver,
            substeps=substeps,
            backend=backend,
            episode_length=episode_length,
        ),
        n_trials,
        key,
    )
    solutions = {
        name: solution_of(problem, entry)
        for name, entry in named.items()
        if not isinstance(entry, Evaluation)
    }

    return Comparison(solutions, evaluations, problem)


@dataclass(frozen=True)
class Comparison:
    """
    Named solutions of one problem, their scores, and the overlays that read them together.

    ``solutions`` maps a name to a :class:`~minilink.planning.results.PlanningSolution`
    (a controller entry is wrapped as one); ``evaluations`` maps every scored
    name to its :class:`~minilink.planning.evaluation.Evaluation` (``None`` when
    :func:`compare` was given no problem); ``problem`` is the judged problem,
    ``None`` for a comparison by names only: the plans are scored on it and
    the figure draws its goal and limits. Each evaluation :func:`compare`
    makes keeps the judge's runs, one per trial, in its ``trajectories``: the
    one home of the runs the figure draws; memory grows with ``n_trials``.
    The nominal behaviour of the answers is compared on a deterministic
    problem, one run each; their spread, on the same problem with its
    distributions, every run drawn.

    ``print`` is the table, each answer read twice, the same columns each
    (``J``, goal, limits, length): its plan as the planner wrote it, cut at
    the horizon ``t0 + T`` and scored on the problem without simulation (by
    names only, on its own problem), beside its run, the block it hands out
    simulated by the judge (on several draws, the criterion and the counts
    over them). A star marks the best run: the lowest criterion value among
    the answers whose every run reaches the goal within the limits. The
    planners' records follow, one line each. Each ``plot_*`` draws every
    solution on one figure with one scale, and :meth:`difference` reads the
    paired trials. The judged problem's system labels the axes, else the
    first solution's.
    """

    solutions: dict
    evaluations: dict | None = None
    problem: PlanningProblem | None = None

    # The named solutions, read as a mapping

    def __getitem__(self, name) -> PlanningSolution:
        return self.solutions[name]

    def __iter__(self):
        return iter(self.solutions)

    def __len__(self) -> int:
        return len(self.solutions)

    def items(self):
        return self.solutions.items()

    # Scores and overlays

    def difference(self, a, b):
        """The paired differences ``J_a − J_b``, one per trial (the same draws for both)."""
        require_scores(self.evaluations, (a, b))
        J_a = np.asarray(self.evaluations[a].J)
        J_b = np.asarray(self.evaluations[b].J)
        require_paired(J_a, J_b, (a, b))

        dJ = J_a - J_b

        return dJ

    def plot_control_law(
        self,
        *,
        x_axis=None,
        y_axis=None,
        u_axis=0,
        bounds=None,
        grid_shape=(101, 101),
        t=0.0,
        title=None,
        figsize=None,
        renderer="matplotlib",
        show=True,
        ax=None,
    ) -> PlotResult:
        """One panel per feedback law, side by side, one colour scale from the problem's inputs: a :class:`PlotResult`, ``title`` above the panels."""
        # One panel per law, each the controller's own, titled by its name
        laws = []
        for name, solution in self.items():
            solution.require_feedback_law()
            law = controller_law_figure(
                solution,
                x_axis=x_axis,
                y_axis=y_axis,
                u_axis=u_axis,
                bounds=bounds,
                grid_shape=grid_shape,
                t=t,
            )
            laws.append(replace(law.panels[0], title=name))
        spec = side_by_side(laws, title or "")

        return draw(spec, figsize=figsize, renderer=renderer, show=show, ax=ax)

    def plot_cost_to_go(
        self,
        *,
        x_axis=0,
        y_axis=1,
        anchor=None,
        bounds=None,
        grid_shape=(51, 51),
        t=0.0,
        jmax=None,
        title=None,
        figsize=None,
        renderer="matplotlib",
        show=True,
        ax=None,
    ) -> PlotResult:
        """One panel per cost-to-go field, side by side, one colour scale (``jmax`` or the largest finite value): a :class:`PlotResult`, ``title`` above the panels."""
        sweep = dict(
            x_axis=x_axis,
            y_axis=y_axis,
            anchor=anchor,
            bounds=bounds,
            grid_shape=grid_shape,
            t=t,
        )

        # The solutions that carry a cost-to-go field
        fields = {
            name: solution
            for name, solution in self.items()
            if solution.cost_to_go is not None
        }
        require_some_cost_to_go(fields)

        # Each field sampled on one mesh; one colour scale: jmax, else the largest value
        samples = {
            name: sample_cost_to_go(solution, **sweep)
            for name, solution in fields.items()
        }
        if jmax is None:
            jmax = max(float(np.nanmax(J)) for _, _, J in samples.values())

        # One panel per field, titled by its name
        heatmaps = [
            cost_to_go_panel(
                fields[name].problem, x_level, y_level, J, x_axis, y_axis, jmax, name
            )
            for name, (x_level, y_level, J) in samples.items()
        ]
        spec = side_by_side(heatmaps, title or "")

        return draw(spec, figsize=figsize, renderer=renderer, show=show, ax=ax)

    def plot_trajectory(
        self,
        *,
        signals=("x", "u"),
        title="plans (dashed) and runs (solid)",
        figsize=None,
        renderer="matplotlib",
        show=True,
        ax=None,
    ) -> PlotResult:
        """
        Each answer's runs solid and its plan dashed, one colour per answer, one row per signal component: a :class:`PlotResult`, ``title`` over the first row.

        The runs are the judge's, kept in the answer's evaluation (the plant's
        part of each): one run is a solid line, several runs (one per draw)
        thin faint lines; by names only, the controller run once from its
        problem's start. The plan is drawn on top, whole, past the horizon
        when it is longer. On a judged problem, each row of ``x`` marks the
        goal's extent at the deadline ``t0 + T``, each row of ``u`` the input
        limits, and every row the horizon. The legend holds the answers' names
        and the limits. An :class:`~minilink.planning.evaluation.Evaluation`
        entry is drawn when it keeps its runs, else left out.
        """
        runs = {name: drawn_runs_of(self, name) for name in self.evaluations or self}
        colors = {
            name: SOLUTION_COLORS[m % len(SOLUTION_COLORS)]
            for m, name in enumerate(name for name in runs if runs[name])
        }
        require_drawable(colors)
        problem = self.problem
        sys = (problem or next(iter(self.solutions.values())).problem).sys

        # Each answer's runs in its colour, one solid, several thin and faint; then
        # the plans dashed on top
        lines = []
        for name, color in colors.items():
            faint = len(runs[name]) > 1
            width, alpha = (FAINT_WIDTH, FAINT_ALPHA) if faint else (1.5, 1.0)
            for i, run in enumerate(runs[name]):
                label = name if i == 0 else ""
                lines.append((label, color, "solid", width, alpha, run))
        for name, color in colors.items():
            solution = self.solutions.get(name)
            if solution is not None and solution.trajectory is not None:
                plan = solution.trajectory
                plan = Trajectory(plan.t, plan.x, plan.u)
                lines.append(("", color, "dash", 1.5, 1.0, plan))
        span = time_span_of(trajectory for *_, trajectory in lines)

        # One row per component of each signal, its marks read from the signal: the
        # answers' names on the first row
        rows = []
        for signal in signals:
            specs = [
                signal_figure(sys, trajectory, signals=(signal,))
                for *_, trajectory in lines
            ]
            for k, row in enumerate(specs[0].panels):
                traces = tuple(
                    Trace(
                        spec.panels[k].layers[0].x,
                        spec.panels[k].layers[0].y,
                        color=color,
                        dash=dash,
                        width=width,
                        alpha=alpha,
                        label=name if not rows else "",
                    )
                    for (name, color, dash, width, alpha, _), spec in zip(lines, specs)
                )
                deadline = deadline_marks_of(problem, signal, k)
                limits = limit_marks_of(problem, signal, k, span)
                rows.append((row.y_label, deadline + traces + limits))

        n_rows = len(rows)
        panels = tuple(
            Panel(
                layers=layers,
                title=title if i == 0 else "",
                x_label=TIME_ABSCISSA_LABEL if i == n_rows - 1 else "",
                y_label=y_label.replace("\n", " "),
            )
            for i, (y_label, layers) in enumerate(rows)
        )
        height = max(2.6, 1.8 * n_rows)
        spec = FigureSpec(panels=panels, share_x=True, figsize=(9.0, height))

        return draw(spec, figsize=figsize, renderer=renderer, show=show, ax=ax)

    # Display

    def __str__(self) -> str:
        return comparison_text(self)


def sample_cost_to_go(
    solution,
    *,
    x_axis=0,
    y_axis=1,
    anchor=None,
    bounds=None,
    grid_shape=(51, 51),
    t=0.0,
):
    """``(x_level, y_level, J)``: the cost-to-go sampled on a mesh over the state axes ``x_axis`` and ``y_axis``."""
    field = require_cost_to_go(solution)

    return sample_field(
        solution.problem,
        field,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        bounds=bounds,
        grid_shape=grid_shape,
        t=t,
    )


###############################################################################
# Internal machinery
###############################################################################


@dataclass(frozen=True)
class GivenController:
    """The record of a controller compared as given: no planner solved it."""

    success: bool = True

    def __str__(self) -> str:
        return "given"


def split_entries(args, entries):
    """``(problem or None, named entries)`` from ``compare``'s positional and keyword arguments."""
    args = list(args)
    problem = args.pop(0) if args and isinstance(args[0], PlanningProblem) else None
    if len(args) > 1 or (args and not isinstance(args[0], dict)):
        raise TypeError(
            "compare takes named entries, after the problem when they are scored: "
            "compare(VI=vi, LQR=lqr), compare(problem, PID=pid, VI=vi, dt=0.01)"
        )

    # The dict passed positionally, then the keyword entries over it
    named = dict(args[0]) if args else {}
    named.update(entries)
    if not named:
        raise ValueError("compare needs at least one entry")

    return problem, named


def score_entries(problem, named, settings, n_trials, key):
    """Every entry's :class:`Evaluation` on one key and one backend (NumPy for all when one entry does not trace), its runs kept."""
    scored = {n: e for n, e in named.items() if not isinstance(e, Evaluation)}

    # The loops compiled, then on NumPy for all when one of them is
    evaluators = {
        n: ProblemEvaluator(problem, e, **settings) for n, e in scored.items()
    }
    if settings["backend"] == "auto":
        backends = {pe.backend for pe in evaluators.values()}
        if len(backends) > 1:
            numpy = {**settings, "backend": "numpy"}
            evaluators = {
                n: ProblemEvaluator(problem, e, **numpy) for n, e in scored.items()
            }

    evaluations = {}
    for name, entry in named.items():
        if isinstance(entry, Evaluation):
            evaluations[name] = entry
        else:
            evaluations[name] = evaluators[name].evaluate(
                key=key, n_trials=n_trials, trajectories=True
            )

    return evaluations


def solution_of(problem, entry):
    """A solution as given, or a controller wrapped as a solution of ``problem``."""
    if isinstance(entry, PlanningSolution):
        return entry

    return PlanningSolution(problem=problem, controller=entry, record=GivenController())


def drawn_runs_of(comparison, name) -> list:
    """The plant runs the figure draws for an answer: its evaluation's, else (by names only) its controller run once; none for an evaluation that keeps no run."""
    evaluation = (comparison.evaluations or {}).get(name)
    if evaluation is not None:
        return [plain_plant_run_of(run) for run in evaluation.trajectories or ()]

    return [controller_run_of(comparison.solutions[name])]


def time_span_of(trajectories):
    """``(t_first, t_last)`` over the trajectories' times."""
    times = [np.asarray(trajectory.t, dtype=float) for trajectory in trajectories]

    return min(t[0] for t in times), max(t[-1] for t in times)


def deadline_marks_of(problem, signal, k) -> tuple:
    """
    The judged problem's deadline on the row of component ``k`` of ``signal``.

    On every row the horizon ``t0 + T``, a dotted grey vertical; on a row of
    ``x`` also the goal's extent there, a dark grey segment
    (``Xf.bounding_box()``; none for a set without one). None without a
    problem or a finite horizon.
    """
    if problem is None or problem.horizon_kind() != "finite":
        return ()
    t_end = float(problem.t0) + float(problem.horizon)
    goal = None if problem.Xf is None else problem.Xf.bounding_box()

    marks = [RefLine("x", t_end, color=LIMIT_COLOR, dash="dot")]
    if signal == "x" and goal is not None:
        extent = [float(goal.lower[k]), float(goal.upper[k])]
        marks.append(Trace([t_end, t_end], extent, color=GOAL_COLOR, width=4.0))

    return tuple(marks)


def limit_marks_of(problem, signal, k, span) -> tuple:
    """The judged problem's input limits on the row of component ``k`` of ``u``: dotted grey lines over ``span`` (``U.bounding_box()``), each finite bound; none on another row."""
    if problem is None or problem.U is None or signal != "u":
        return ()
    limits = problem.U.bounding_box()
    if limits is None:
        return ()

    bounds = [float(limits.lower[k]), float(limits.upper[k])]
    marks = tuple(
        Trace(span, [bound, bound], color=LIMIT_COLOR, dash="dot", label="limits")
        for bound in bounds
        if np.isfinite(bound)
    )

    return marks


# The table: its two halves, titled, and the columns of each
PLAN_TITLE = "plan, as the planner wrote it"
RUN_TITLE = "run, simulated by the judge"
COLUMNS = ("goal", "limits", "length")


def comparison_text(comparison) -> str:
    """The print text: one line per answer, its plan beside its run, then the records."""
    scores = comparison.evaluations or {}
    solutions = comparison.solutions
    names = list(dict.fromkeys([*solutions, *scores]))
    runs = {
        name: scores.get(name, getattr(solutions.get(name), "evaluation", None))
        for name in names
    }

    # The numbers: the best run, each plan's score and each run's length
    best = best_run_of(runs)
    plans = {name: plan_score_of(comparison, name) for name in names}
    lengths = {name: run_length_of(runs[name]) for name in names}

    # Formatted last: each half's cells, or the words that span it
    label = criterion_label_of(runs.values())
    plan_halves = {
        name: plan if isinstance(plan, str) else cells(*plan)
        for name, plan in plans.items()
    }
    run_halves = {
        name: "not scored"
        if runs[name] is None
        else cells(runs[name], lengths[name], "*" if name == best else " ")
        for name in names
    }
    headers = (("J", "", *COLUMNS), (label, "", *COLUMNS))
    lines = table_lines(names, plan_halves, run_halves, headers)

    # The star's footnote, then the planners' own accounts, one line each
    if best is not None:
        which = "that" if runs[best].n_trials == 1 else "whose every run"
        lines += [
            "",
            f"* best run: the lowest {label} {which} reaches the goal within the limits",
        ]
    solved = [name for name in names if name in solutions]
    if solved:
        pad = max(len(name) for name in solved)
        lines += ["", "records"]
        lines += [f"  {name:{pad}}  {solutions[name].record}" for name in solved]

    return "\n".join(lines)


def best_run_of(runs):
    """The name of the best run: the lowest ``objective`` among the evaluations whose every run succeeds; ``None`` when none does."""
    qualified = {
        name: float(evaluation.objective)
        for name, evaluation in runs.items()
        if evaluation is not None and float(evaluation.success_rate) == 1.0
    }
    if not qualified:
        return None

    return min(qualified, key=qualified.get)


def plan_score_of(comparison, name):
    """``(evaluation, length)`` of an answer's plan on the judged problem (by names only, its own), or the words of its plan half."""
    solution = comparison.solutions.get(name)
    if solution is None:
        return "no plan"
    if solution.trajectory is None:
        return "no plan" if solution.open_loop else "no plan (a feedback law)"
    problem = solution.problem if comparison.problem is None else comparison.problem
    if not is_continuous(problem):
        return "not scored"

    # The plan as the plant's run on the horizon, scored without simulation
    plan = solution.trajectory
    evaluation = evaluate_trajectory(problem, plan_on_horizon(plan, problem))
    length = float(plan.t[-1] - plan.t[0])

    return evaluation, length


def plan_on_horizon(plan, problem) -> Trajectory:
    """The plan as the plant's run ``(t, x, u)``, cut at ``t0 + T``: the state there interpolated, the input held from the sample before."""
    t = np.asarray(plan.t, dtype=float)
    x = np.asarray(plan.x, dtype=float)
    u = np.asarray(plan.u, dtype=float)
    if problem.horizon_kind() != "finite":
        return Trajectory(t, x, u)
    t_end = float(problem.t0) + float(problem.horizon)
    if t[-1] <= t_end:
        return Trajectory(t, x, u)

    # The samples before t0 + T, then x(t0 + T) between its two neighbours
    kept = t < t_end
    x_end = np.array([np.interp(t_end, t, x_i) for x_i in x])
    u_end = u[:, kept][:, -1]
    cut = Trajectory(
        np.append(t[kept], t_end),
        np.column_stack([x[:, kept], x_end]),
        np.column_stack([u[:, kept], u_end]),
    )

    return cut


def run_length_of(evaluation):
    """The length of an evaluation's first run; ``None`` when it keeps none."""
    if evaluation is None or not evaluation.trajectories:
        return None
    run = evaluation.trajectories[0]

    return float(run.t[-1] - run.t[0])


def criterion_label_of(evaluations) -> str:
    """The run's ``J`` header: ``J`` for one run, else the criterion over the draws (``E[J]``, ``max J``, ``CVaR_α(J)``)."""
    several = [e for e in evaluations if e is not None and e.n_trials > 1]
    if not several:
        return "J"
    criterion = several[0].criterion
    if criterion == "expectation":
        return "E[J]"
    if criterion == "worst_case":
        return "max J"

    return f"CVaR_{float(criterion[1]):g}(J)"


def cells(evaluation, length, star=""):
    """One half's cells from its numbers: ``J``, the star, goal, limits, length; words for one run, counts for several."""
    n = evaluation.n_trials
    reached = np.asarray(evaluation.reached)
    failed = np.asarray(evaluation.failed)
    finite = np.isfinite(np.asarray(evaluation.raw_cost))
    J = f"{float(evaluation.objective):.4g}"

    # One run: in words, and its length; several: how many of them
    if n == 1:
        goal = "reached" if reached[0] else "missed"
        limits = "diverged" if not finite[0] else "violated" if failed[0] else "ok"
        span = "" if length is None else f"{length:.3g} s"
    else:
        goal = f"{int(reached.sum())}/{n}"
        limits = f"{int((~failed).sum())}/{n}"
        span = ""

    return J, star, goal, limits, span


def table_lines(names, plan_halves, run_halves, headers):
    """The table's lines: the halves' titles, the header, a rule, one line per answer; each column as wide as its widest cell."""
    halves = (plan_halves, run_halves)

    # Each column's width, then each half's: its columns or its widest words
    widths = [
        [
            max(
                len(row[k])
                for row in (header, *half.values())
                if not isinstance(row, str)
            )
            for k in range(len(header))
        ]
        for header, half in zip(headers, halves)
    ]
    texts = [
        {name: half_text(half[name], width, slot) for name in names}
        for half, width, slot in zip(halves, widths, (False, True))
    ]
    header_texts = [
        half_text(header, width, slot)
        for header, width, slot in zip(headers, widths, (False, True))
    ]
    w_name = max(len(name) for name in names)
    w_plan = max(len(PLAN_TITLE), len(header_texts[0]), *map(len, texts[0].values()))
    w_run = max(len(RUN_TITLE), len(header_texts[1]), *map(len, texts[1].values()))

    def line(name, plan, run):
        return f"{name:{w_name}} │ {plan:{w_plan}} │ {run}".rstrip()

    rule = "─" * (w_name + 1) + "┼" + "─" * (w_plan + 2) + "┼" + "─" * (w_run + 1)

    return [
        line("", PLAN_TITLE, RUN_TITLE),
        line("", *header_texts),
        rule,
        *(line(name, texts[0][name], texts[1][name]) for name in names),
    ]


def half_text(half, widths, slot) -> str:
    """One half of a line: its spanning words, or its cells aligned, ``J`` right with the star in its slot, the words left."""
    if isinstance(half, str):
        return half
    J, star, *words = half
    first = J.rjust(widths[0]) + (f" {star or ' '}" if slot else "")
    rest = [word.ljust(width) for word, width in zip(words, widths[2:])]

    return "  ".join([first, *rest])


def require_no_scoring(dt, n_trials):
    """Refuse scoring settings without a problem to score on."""
    if dt is not None or n_trials is not None:
        raise TypeError(
            "compare scores its entries only on a problem: compare(problem, ...)"
        )


def require_scores(evaluations, names):
    """Refuse a difference of entries that were not scored."""
    missing = [name for name in names if not evaluations or name not in evaluations]
    if missing:
        raise ValueError(
            f"{missing} carry no evaluation: compare(problem, ...) scores them"
        )


def require_paired(J_a, J_b, names):
    """Refuse a difference of two evaluations that do not hold the same trials."""
    if J_a.shape != J_b.shape:
        raise ValueError(
            f"{names[0]!r} holds {J_a.shape[0]} runs and {names[1]!r} {J_b.shape[0]}: "
            "a paired difference needs the same trials, as compare scores them"
        )


def require_solutions(named):
    """Refuse a named entry that is not a ``PlanningSolution``."""
    for name, solution in named.items():
        if not isinstance(solution, PlanningSolution):
            raise TypeError(
                f"{name!r} is not a PlanningSolution: to score a controller, pass the "
                "problem first, compare(problem, name=controller, ...)"
            )


def require_drawable(drawn):
    """Refuse a figure of a comparison with no run to draw: every entry an evaluation that keeps none."""
    if not drawn:
        raise ValueError(
            "no entry carries a run to draw: an Evaluation that keeps no trajectories "
            "is scored, not drawn; pass a solution, a controller, or "
            "evaluate(..., trajectories=True)"
        )


def require_some_cost_to_go(fields):
    """Refuse an overlay of cost-to-go fields when no solution carries one."""
    if not fields:
        raise ValueError("no solution carries a cost_to_go")


__all__ = [
    "compare",
    "Comparison",
]
