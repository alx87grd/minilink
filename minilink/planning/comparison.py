"""Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the plans."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from minilink.graphical.figure import FigureSpec, Panel, Trace, draw
from minilink.planning.evaluation import Evaluation
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

# One colour per solution, the same on every row
SOLUTION_COLORS = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
)


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

    # Every entry scored on the same draws, its solution kept for the overlays (each
    # loop's period as evaluate reads it: a sampled one's own clock, else dt)
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

    return Comparison(solutions, evaluations)


@dataclass(frozen=True)
class Comparison:
    """
    Named solutions of one problem, their scores, and the overlays that read them together.

    ``solutions`` maps a name to a :class:`~minilink.planning.results.PlanningSolution`
    (a controller entry is wrapped as one); ``evaluations`` maps every scored
    name to its :class:`~minilink.planning.evaluation.Evaluation` (``None`` when
    :func:`compare` was given no problem). ``print`` is the table, each
    ``plot_*`` draws every solution on one figure with one scale, and
    :meth:`difference` reads the paired trials. The first solution's system
    labels the axes.
    """

    solutions: dict
    evaluations: dict | None = None

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
        title="",
        renderer="matplotlib",
        show=None,
        ax=None,
    ):
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
        spec = side_by_side(laws, title)

        return draw(spec, renderer=renderer, show=show, ax=ax)

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
        title="",
        renderer="matplotlib",
        show=None,
        ax=None,
    ):
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
        spec = side_by_side(heatmaps, title)

        return draw(spec, renderer=renderer, show=show, ax=ax)

    def plot_trajectory(
        self,
        *,
        signals=("x", "u"),
        title="plans and runs",
        renderer="matplotlib",
        show=None,
        ax=None,
    ):
        """The plans and the controllers' runs overlaid, one row per signal component, legend by name: a :class:`PlotResult`, ``title`` over the first row."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import build_signal_plot_spec

        # One plot spec per solution, on the first solution's system
        first = next(iter(self.solutions.values()))
        sys = first.problem.sys
        specs = {
            name: build_signal_plot_spec(
                sys, drawn_trajectory_of(solution), signals=signals
            )
            for name, solution in self.items()
        }

        # One row per signal component, each solution in its colour on every row; the
        # rows labelled from the first spec, the names, the legend and the title on
        # the first
        rows = next(iter(specs.values())).traces
        n_rows = len(rows)
        panels = [
            Panel(
                layers=tuple(
                    Trace(
                        spec.t,
                        spec.traces[k].values,
                        color=SOLUTION_COLORS[m % len(SOLUTION_COLORS)],
                        label=name if k == 0 else "",
                    )
                    for m, (name, spec) in enumerate(specs.items())
                ),
                title=title if k == 0 else "",
                x_label="Time [s]" if k == n_rows - 1 else "",
                y_label=row.label + (f" [{row.unit}]" if row.unit else ""),
                legend="upper right",
            )
            for k, row in enumerate(rows)
        ]
        spec = FigureSpec(
            panels=tuple(panels), share_x=True, figsize=(8.0, 1.8 * n_rows)
        )

        return draw(spec, renderer=renderer, show=show, ax=ax)

    # Display

    def __str__(self) -> str:
        # One row per entry: its name, success, record and evaluation
        scores = self.evaluations or {}
        names = list(dict.fromkeys([*self.solutions, *scores]))
        rows = []
        for name in names:
            solution = self.solutions.get(name)
            evaluation = scores.get(name, getattr(solution, "evaluation", None))
            rows.append(
                (
                    name,
                    "-" if solution is None else ("yes" if solution.success else "no"),
                    "-" if solution is None else str(solution.record),
                    "-" if evaluation is None else str(evaluation),
                )
            )
        header = ("", "success", "record", "evaluation")

        # Each column as wide as its widest cell
        widths = [
            max(len(row[k]) for row in (header, *rows)) for k in range(len(header))
        ]
        lines = [
            "  ".join(cell.ljust(width) for cell, width in zip(row, widths)).rstrip()
            for row in (header, *rows)
        ]

        return "\n".join(lines)


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
    """Every entry's :class:`Evaluation` on one key and one backend (NumPy for all when one entry does not trace)."""
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

    return evaluations


def solution_of(problem, entry):
    """A solution as given, or a controller wrapped as a solution of ``problem``."""
    if isinstance(entry, PlanningSolution):
        return entry

    return PlanningSolution(problem=problem, controller=entry, record=GivenController())


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


def side_by_side(panels, title):
    """One row of panels side by side, ``title`` above them, as a description."""
    n = len(panels)

    return FigureSpec(
        panels=tuple(panels), title=title, n_cols=n, figsize=(4.5 * n, 4.0)
    )


def require_some_cost_to_go(fields):
    """Refuse an overlay of cost-to-go fields when no solution carries one."""
    if not fields:
        raise ValueError("no solution carries a cost_to_go")


__all__ = [
    "compare",
    "Comparison",
]
