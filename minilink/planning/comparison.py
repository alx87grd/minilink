"""Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the trajectories."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.reporting import DISP_LINE_WIDTH
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
from minilink.planning.evaluation import (
    Evaluation,
    evaluate_trajectory,
    reduce_criterion,
)
from minilink.planning.evaluator import ProblemEvaluator
from minilink.planning.plotting import (
    controller_law_figure,
    cost_to_go_panel,
    sample_field,
)
from minilink.planning.problems import (
    PlanningProblem,
    criterion_text,
    planning_horizon_text,
)
from minilink.planning.results import (
    PlanningSolution,
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

# An answer's line: one solid; one per draw thin and faint, so their spread reads
SOLID_WIDTH = 1.5
FAINT_WIDTH = 0.6
FAINT_ALPHA = 0.12


###############################################################################
# Public API
###############################################################################


def compare(
    problem,
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
    """The :class:`Comparison` of :meth:`PlanningProblem.compare`: every entry scored on the same draws."""
    named = entries_of(args, entries)

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
    (a controller entry is wrapped as one); ``evaluations`` maps every name to
    its :class:`~minilink.planning.evaluation.Evaluation`; ``problem`` is the
    judged problem: the nominal trajectories are scored on it and the figures
    draw its goal and limits. Each evaluation :meth:`PlanningProblem.compare`
    makes keeps the judge's runs, one per trial, in its ``trajectories``: the
    one home of the simulations the figure draws. One run each on the
    deterministic problem reads the answers' behaviour; the same problem with
    its distributions reads their spread, every run drawn.

    ``print`` is the table, a framed panel titled by the judged system, its
    horizon and, on several draws, the criterion and their number: each
    answer read twice (``J``, goal, constraints), its nominal trajectory, the
    planner's own, cut at the horizon ``t0 + T`` and scored on the problem
    without simulation (``-`` for an answer without one), beside its
    simulation, the block it hands out simulated by the judge (on several draws, the judged criterion first when
    it is not the expectation, then ``E[J]``, ``σ(J)`` and ``max J``, and the
    counts). ``print(comparison[name])`` shows a planner's record. Each
    ``plot_*`` draws the solutions on one figure with one scale, and
    :meth:`difference` reads the paired trials. The judged problem's system
    labels the axes.
    """

    solutions: dict
    evaluations: dict
    problem: PlanningProblem

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

    def plot_nominal_trajectory(
        self,
        *,
        signals=("x", "u"),
        title="nominal trajectory",
        figsize=None,
        renderer="matplotlib",
        show=True,
        ax=None,
    ) -> PlotResult:
        """
        Each answer's nominal trajectory, one colour per answer, one row per signal component: a :class:`PlotResult`, ``title`` over the first row.

        The nominal trajectory is the planner's own, drawn whole, past the
        horizon when it is longer; the answers without one (a feedback law, an
        :class:`~minilink.planning.evaluation.Evaluation` entry) are left out.
        The colours and the marks of the judged problem are those of
        :meth:`plot_simulated_trajectory`.
        """
        colors = colors_of(self)

        # Each answer's nominal trajectory, the plant's (t, x, u) alone
        lines = []
        for name, color in colors.items():
            solution = self.solutions.get(name)
            if solution is not None and solution.trajectory is not None:
                nominal = solution.trajectory
                plant_part = Trajectory(nominal.t, nominal.x, nominal.u)
                lines.append((name, color, SOLID_WIDTH, 1.0, plant_part))
        require_some_nominal(lines)
        spec = trajectory_figure(self, lines, signals, title)

        return draw(spec, figsize=figsize, renderer=renderer, show=show, ax=ax)

    def plot_simulated_trajectory(
        self,
        *,
        signals=("x", "u"),
        title="simulation",
        figsize=None,
        renderer="matplotlib",
        show=True,
        ax=None,
    ) -> PlotResult:
        """
        Each answer's simulations, one colour per answer, one row per signal component: a :class:`PlotResult`, ``title`` over the first row.

        The simulations are the judge's, kept in the answer's evaluation (the
        plant's part of each): one simulation is a solid line, several (one
        per draw) thin faint lines. Each row of ``x`` marks
        the goal's extent at the deadline ``t0 + T``, each row of ``u`` the
        input limits, and every row the horizon. The legend holds the answers'
        names and the limits. An
        :class:`~minilink.planning.evaluation.Evaluation` entry is drawn when
        it keeps its runs, else left out.
        """
        colors = colors_of(self)

        # Each answer's simulations in its colour, one solid, several thin and faint
        lines = []
        for name, color in colors.items():
            runs = drawn_runs_of(self, name)
            faint = len(runs) > 1
            width, alpha = (FAINT_WIDTH, FAINT_ALPHA) if faint else (SOLID_WIDTH, 1.0)
            lines += [(name, color, width, alpha, run) for run in runs]
        require_some_simulation(lines)
        spec = trajectory_figure(self, lines, signals, title)

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


def entries_of(args, entries) -> dict:
    """The named entries of ``problem.compare``: a dict passed positionally, then the keyword entries over it."""
    if len(args) > 1 or (args and not isinstance(args[0], dict)):
        raise TypeError(
            "compare takes named entries: problem.compare(PID=pid, VI=vi, dt=0.01) "
            'or problem.compare({"PID": pid, "VI": vi}, dt=0.01)'
        )
    named = dict(args[0]) if args else {}
    named.update(entries)
    if not named:
        raise ValueError("compare needs at least one entry")

    return named


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


def colors_of(comparison) -> dict:
    """One colour per answer, in the order the entries were given: the same in both trajectory figures."""
    names = list(comparison.evaluations)
    colors = {
        name: SOLUTION_COLORS[m % len(SOLUTION_COLORS)] for m, name in enumerate(names)
    }

    return colors


def trajectory_figure(comparison, lines, signals, title) -> FigureSpec:
    """
    The figure of ``lines``, ``(name, color, width, alpha, trajectory)`` each: one row per component of each signal, the judged problem's marks read from the row's signal.

    The answers' names label the first row; the judged problem's system
    labels the axes.
    """
    problem = comparison.problem
    sys = problem.sys
    times = [np.asarray(trajectory.t, dtype=float) for *_, trajectory in lines]
    span = (min(t[0] for t in times), max(t[-1] for t in times))

    # One row per component of each signal, its marks read from the signal
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
                    width=width,
                    alpha=alpha,
                    label=label,
                )
                for (label, color, width, alpha, _), spec in zip(lines, specs)
            )
            deadline = deadline_marks_of(problem, signal, k)
            limits = limit_marks_of(problem, signal, k, span)
            rows.append((row.y_label, deadline + traces + limits, limits))

    n_rows = len(rows)
    panels = tuple(
        Panel(
            layers=layers,
            title=title if i == 0 else "",
            x_label=TIME_ABSCISSA_LABEL if i == n_rows - 1 else "",
            y_label=y_label.replace("\n", " "),
            legend=i == 0 or bool(limits),
        )
        for i, (y_label, layers, limits) in enumerate(rows)
    )
    height = max(2.6, 1.8 * n_rows)
    spec = FigureSpec(panels=panels, share_x=True, figsize=(9.0, height))

    return spec


def drawn_runs_of(comparison, name) -> list:
    """The plant runs the simulated figure draws for an answer: its evaluation's; none for an evaluation that keeps no run."""
    evaluation = comparison.evaluations[name]

    return [plain_plant_run_of(run) for run in evaluation.trajectories or ()]


def deadline_marks_of(problem, signal, k) -> tuple:
    """
    The judged problem's deadline on the row of component ``k`` of ``signal``.

    On every row the horizon ``t0 + T``, a dotted grey vertical; on a row of
    ``x`` also the goal's extent there, a dark grey segment
    (``Xf.bounding_box()``; none for a set without one). None without a
    finite horizon.
    """
    if problem.horizon_kind() != "finite":
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
    if problem.U is None or signal != "u":
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
NOMINAL_TITLE = "nominal trajectory"
SIMULATION_TITLE = "simulation"
COLUMNS = ("goal", "constraints")

# The J column of one run: its value, the criterion over a single draw
ONE_RUN = (("J", "objective"),)


def comparison_text(comparison) -> str:
    """The print text: a framed panel, its title, then one line per answer, its nominal trajectory beside its simulation."""
    runs = comparison.evaluations
    names = list(runs)

    # The numbers: each nominal trajectory's score; the draws, the judged criterion and
    # the statistics the simulations show
    nominals = {name: nominal_score_of(comparison, name) for name in names}
    several = [e for e in runs.values() if e.n_trials > 1]
    criterion = comparison.problem.criterion
    statistics = statistics_of(criterion, several)

    # Formatted last: each half's cells, or the words that span it
    nominal_halves = {
        name: nominal if isinstance(nominal, str) else cells(nominal, ONE_RUN)
        for name, nominal in nominals.items()
    }
    simulation_halves = {
        name: cells(runs[name], statistics, criterion) for name in names
    }
    headers = (
        (*(label for label, _ in ONE_RUN), *COLUMNS),
        (*(label for label, _ in statistics), *COLUMNS),
    )
    lines = table_lines(names, nominal_halves, simulation_halves, headers)

    # The frame: the title between two rules, a closing rule, as wide as the table
    title = title_of(comparison, criterion, several)
    width = max(DISP_LINE_WIDTH, len(title), *map(len, lines))
    rule = "=" * width
    panel = [rule, title, rule, *lines, rule]

    return "\n".join(panel)


def nominal_score_of(comparison, name):
    """The evaluation of an answer's nominal trajectory on the judged problem, or the words of its half."""
    solution = comparison.solutions.get(name)
    if solution is None or solution.trajectory is None:
        return "-"
    problem = comparison.problem
    if not is_continuous(problem):
        return "not scored"
    if starts_after_deadline(solution.trajectory, problem):
        return "starts after the deadline"

    # The nominal trajectory as the plant's run on the horizon, scored without simulation
    evaluation = evaluate_trajectory(
        problem, nominal_on_horizon(solution.trajectory, problem)
    )

    return evaluation


def starts_after_deadline(nominal, problem) -> bool:
    """``True`` when the nominal trajectory begins at or after the judged deadline ``t0 + T``: nothing of it is left on the horizon."""
    if problem.horizon_kind() != "finite":
        return False

    return float(nominal.t[0]) >= float(problem.t0) + float(problem.horizon)


def nominal_on_horizon(nominal, problem) -> Trajectory:
    """The nominal trajectory as the plant's run ``(t, x, u)``, cut at ``t0 + T``: the state there interpolated, the input held from the sample before."""
    t = np.asarray(nominal.t, dtype=float)
    x = np.asarray(nominal.x, dtype=float)
    u = np.asarray(nominal.u, dtype=float)
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


def statistics_of(criterion, several) -> tuple:
    """
    The simulation half's ``J`` columns, ``(label, statistic)`` each.

    One run: ``J``. Several draws: ``E[J]``, ``σ(J)`` and ``max J``, the
    judged criterion first when it is not the expectation (a CVaR its own
    column; the worst case is ``max J`` itself), each statistic once.
    """
    if not several:
        return ONE_RUN

    mean = ("E[J]", "mean")
    std = ("σ(J)", "std")
    worst = ("max J", "worst")
    if criterion == "expectation":
        return (mean, std, worst)
    if criterion == "worst_case":
        return (worst, mean, std)
    cvar = (f"CVaR_{float(criterion[1]):g}(J)", "criterion")

    return (cvar, mean, std, worst)


def cells(evaluation, statistics, criterion="expectation"):
    """
    One half's cells from its numbers: each statistic's value, then goal and constraints; words for one run, counts for several.

    The ``criterion`` statistic is the judged criterion over the evaluation's
    ``J``, whatever criterion the evaluation was made with. One run among
    several draws shows its ``J`` in the first column, ``-`` in the others: it
    has no spread.
    """
    n = evaluation.n_trials
    reached = np.asarray(evaluation.reached)
    failed = np.asarray(evaluation.failed)
    finite = np.isfinite(np.asarray(evaluation.raw_cost))
    values = [
        f"{float(statistic_value_of(evaluation, name, criterion)):.4g}"
        for _, name in statistics
    ]
    if n == 1 and len(values) > 1:
        values = [values[0], *("-" for _ in values[1:])]

    # One run: in words; several: how many of them
    if n == 1:
        goal = "reached" if reached[0] else "missed"
        constraints = "diverged" if not finite[0] else "violated" if failed[0] else "ok"
    else:
        goal = f"{int(reached.sum())}/{n}"
        constraints = f"{int((~failed).sum())}/{n}"

    return (*values, goal, constraints)


def statistic_value_of(evaluation, name, criterion):
    """One statistic of an evaluation's ``J``: the judged ``criterion`` over it, else the attribute ``name`` (``objective``, ``mean``, ``std``, ``worst``)."""
    if name == "criterion":
        return reduce_criterion(evaluation.J, criterion)

    return getattr(evaluation, name)


def table_lines(names, nominal_halves, simulation_halves, headers):
    """The table's lines: the halves' titles, the header, a rule, one line per answer; each column as wide as its widest cell."""
    # Each half's header and lines, each column as wide as its widest cell
    texts = []
    for header, half in zip(headers, (nominal_halves, simulation_halves)):
        rows = [header, *(half[name] for name in names)]
        cell_rows = [row for row in rows if not isinstance(row, str)]
        widths = [max(len(row[k]) for row in cell_rows) for k in range(len(header))]
        texts.append([half_text(row, widths) for row in rows])
    nominal_texts, simulation_texts = texts
    w_name = max(len(name) for name in names)
    w_nominal = max(len(NOMINAL_TITLE), *map(len, nominal_texts))
    w_simulation = max(len(SIMULATION_TITLE), *map(len, simulation_texts))

    def line(name, nominal, simulation):
        return f"{name:{w_name}} │ {nominal:{w_nominal}} │ {simulation}".rstrip()

    rule = (
        "─" * (w_name + 1)
        + "┼"
        + "─" * (w_nominal + 2)
        + "┼"
        + "─" * (w_simulation + 1)
    )
    header, *body = map(line, ["", *names], nominal_texts, simulation_texts)

    return [line("", NOMINAL_TITLE, SIMULATION_TITLE), header, rule, *body]


def half_text(half, widths) -> str:
    """One half of a line: its spanning words, or its cells aligned, the numbers right, the words left."""
    if isinstance(half, str):
        return half
    n_numbers = len(half) - len(COLUMNS)
    numbers = [cell.rjust(width) for cell, width in zip(half[:n_numbers], widths)]
    words = [
        cell.ljust(width) for cell, width in zip(half[n_numbers:], widths[n_numbers:])
    ]

    return "  ".join([*numbers, *words])


def title_of(comparison, criterion, several) -> str:
    """The panel's title: the judged system and its horizon, and on several draws the criterion and their number."""
    problem = comparison.problem
    title = (
        f"Comparison, {problem.sys.name}, "
        f"horizon={planning_horizon_text(problem.horizon)}"
    )

    # The criterion and the number of draws, a range when the entries differ
    if several:
        counts = sorted({e.n_trials for e in several})
        draws = f"{counts[0]}" if len(counts) == 1 else f"{counts[0]}–{counts[-1]}"
        title = f"{title}, criterion={criterion_text(criterion)}, {draws} draws"

    return title


def require_paired(J_a, J_b, names):
    """Refuse a difference of two evaluations that do not hold the same trials."""
    if J_a.shape != J_b.shape:
        raise ValueError(
            f"{names[0]!r} holds {J_a.shape[0]} runs and {names[1]!r} {J_b.shape[0]}: "
            "a paired difference needs the same trials, as compare scores them"
        )


def require_some_simulation(lines):
    """Refuse a figure of simulations when no answer carries one: every entry an evaluation that keeps no run."""
    if not lines:
        raise ValueError(
            "no entry carries a run to draw: an Evaluation that keeps no trajectories "
            "is scored, not drawn; pass a solution, a controller, or "
            "problem.evaluate(..., trajectories=True)"
        )


def require_some_nominal(lines):
    """Refuse a figure of nominal trajectories when no answer carries one."""
    if not lines:
        raise ValueError(
            "no answer carries a nominal trajectory: a feedback law has none; "
            "plot_simulated_trajectory draws every answer's simulation"
        )


def require_some_cost_to_go(fields):
    """Refuse an overlay of cost-to-go fields when no solution carries one."""
    if not fields:
        raise ValueError("no solution carries a cost_to_go")


__all__ = [
    "Comparison",
]
