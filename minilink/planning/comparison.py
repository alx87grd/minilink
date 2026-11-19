"""Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the plans."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.graphical.figure import (
    TAB10,
    FigureSpec,
    Panel,
    Trace,
    draw,
    side_by_side,
)
from minilink.graphical.signals.time_signals import TIME_ABSCISSA_LABEL
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
    drawn_trajectory_of,
    require_cost_to_go,
)
from minilink.planning.step_model import is_continuous

if TYPE_CHECKING:
    from minilink.graphical.common.plot_result import PlotResult

# One colour per solution, the same on every row
SOLUTION_COLORS = TAB10


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
    with ``dt`` omitted on the evaluator's automatic step.
    """
    problem, named = split_entries(args, entries)

    # Names only: the solutions, read as they are
    if problem is None:
        require_no_scoring(dt, n_trials)
        require_solutions(named)
        return Comparison(named)

    # Every entry scored on the same draws, its solution and its evaluator kept for
    # the table and the overlays (each loop's period as evaluate reads it: a
    # sampled one's own clock, else dt)
    evaluators, evaluations = score_entries(
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

    return Comparison(solutions, evaluations, problem, evaluators)


@dataclass(frozen=True)
class Comparison:
    """
    Named solutions of one problem, their scores, and the overlays that read them together.

    ``solutions`` maps a name to a :class:`~minilink.planning.results.PlanningSolution`
    (a controller entry is wrapped as one); ``evaluations`` maps every scored
    name to its :class:`~minilink.planning.evaluation.Evaluation` (``None`` when
    :func:`compare` was given no problem); ``problem`` is the judged problem,
    ``None`` for a comparison by names only: the plans are scored on it and
    the figure draws its goal and limits; ``evaluators`` are the judge's
    compiled loops, one per scored solution, which the figure runs.

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
    evaluators: dict | None = field(default=None, repr=False, compare=False)

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
        title="plans and runs",
        figsize=None,
        renderer="matplotlib",
        show=True,
        ax=None,
    ) -> PlotResult:
        """The plans and the controllers' runs overlaid, one row per signal component, legend by name: a :class:`PlotResult`, ``title`` over the first row."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import signal_figure

        # One description per solution, on the first solution's system
        first = next(iter(self.solutions.values()))
        sys = first.problem.sys
        specs = {
            name: signal_figure(sys, drawn_trajectory_of(solution), signals=signals)
            for name, solution in self.items()
        }

        # One row per signal component, each solution in its colour on every row; the
        # rows labelled from the first spec, the names, the legend and the title on
        # the first
        rows = next(iter(specs.values())).panels
        n_rows = len(rows)
        panels = [
            Panel(
                layers=tuple(
                    Trace(
                        spec.panels[k].layers[0].x,
                        spec.panels[k].layers[0].y,
                        color=SOLUTION_COLORS[m % len(SOLUTION_COLORS)],
                        label=name if k == 0 else "",
                    )
                    for m, (name, spec) in enumerate(specs.items())
                ),
                title=title if k == 0 else "",
                x_label=TIME_ABSCISSA_LABEL if k == n_rows - 1 else "",
                y_label=row.y_label.replace("\n", " "),
                legend=True,
            )
            for k, row in enumerate(rows)
        ]
        spec = FigureSpec(
            panels=tuple(panels), share_x=True, figsize=(8.0, 1.8 * n_rows)
        )

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
    """``(evaluators, evaluations)``: each scored entry's compiled loop, and every entry's :class:`Evaluation` on one key and one backend (NumPy for all when one entry does not trace)."""
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
            evaluations[name] = evaluators[name].evaluate(key=key, n_trials=n_trials)

    return evaluators, evaluations


def solution_of(problem, entry):
    """A solution as given, or a controller wrapped as a solution of ``problem``."""
    if isinstance(entry, PlanningSolution):
        return entry

    return PlanningSolution(problem=problem, controller=entry, record=GivenController())


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
    lengths = {name: run_length_of(comparison, name, runs[name]) for name in names}

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


def run_length_of(comparison, name, evaluation):
    """The length of an answer's run: its evaluator's horizon, else its kept run's; ``None`` when no run is known."""
    evaluator = (comparison.evaluators or {}).get(name)
    if evaluator is not None:
        return float(evaluator.horizon)
    if evaluation is not None and evaluation.trajectories:
        run = evaluation.trajectories[0]
        return float(run.t[-1] - run.t[0])

    return None


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
    success = np.asarray(evaluation.reached) & ~np.asarray(evaluation.failed)
    failed = np.asarray(evaluation.failed)
    finite = np.isfinite(np.asarray(evaluation.raw_cost))
    J = f"{float(evaluation.objective):.4g}"

    # One run: in words, and its length; several: how many of them
    if n == 1:
        goal = "reached" if success[0] else "missed"
        limits = "diverged" if not finite[0] else "violated" if failed[0] else "ok"
        span = "" if length is None else f"{length:.3g} s"
    else:
        goal = f"{int(success.sum())}/{n}"
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
    """Refuse an empty comparison, or a named entry that is not a ``PlanningSolution``."""
    if not named:
        raise ValueError("compare needs at least one solution")
    for name, solution in named.items():
        if not isinstance(solution, PlanningSolution):
            raise TypeError(
                f"{name!r} is not a PlanningSolution: to score a controller, pass the "
                "problem first, compare(problem, name=controller, ...)"
            )


def require_some_cost_to_go(fields):
    """Refuse an overlay of cost-to-go fields when no solution carries one."""
    if not fields:
        raise ValueError("no solution carries a cost_to_go")


__all__ = [
    "compare",
    "Comparison",
]
