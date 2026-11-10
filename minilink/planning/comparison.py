"""Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the plans."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.fields import GridField
from minilink.core.sets import is_finite_box
from minilink.planning.evaluation import Evaluation
from minilink.planning.evaluator import ProblemEvaluator
from minilink.planning.planner import require_cost_to_go
from minilink.planning.plotting import maybe_show
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution, drawn_trajectory_of

# Matplotlib is imported inside each view, so the planning layer stays importable
# without a display backend until a plot is requested.


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
        show=True,
    ):
        """One panel per feedback law, side by side, one colour scale from the problem's inputs: a :class:`PlotResult` (``.figure``, ``.axes``)."""
        import matplotlib.pyplot as plt

        # One panel per law, each the controller's own, titled by its name
        fig, axes = panels(plt, len(self))
        for ax, (name, solution) in zip(axes, self.items()):
            solution.require_feedback_law()
            controller_law_figure(
                solution,
                x_axis=x_axis,
                y_axis=y_axis,
                u_axis=u_axis,
                bounds=bounds,
                grid_shape=grid_shape,
                t=t,
                ax=ax,
                show=False,
            )
            ax.set_title(name)

        maybe_show(plt, show)

        return plot_result_of((fig, axes))

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
        show=True,
    ):
        """One panel per cost-to-go field, side by side, one colour scale (``jmax`` or the largest finite value): a :class:`PlotResult`."""
        import matplotlib.pyplot as plt

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
        fig, axes = panels(plt, len(fields))
        for ax, (name, (x_level, y_level, J)) in zip(axes, samples.items()):
            problem = fields[name].problem
            cost_to_go_heatmap(
                problem, x_level, y_level, J, x_axis, y_axis, jmax, name, ax
            )

        maybe_show(plt, show)

        return plot_result_of((fig, axes))

    def plot_trajectory(self, *, signals=("x", "u"), show=True):
        """The plans and the controllers' runs overlaid, one row per signal component, legend by name: a :class:`PlotResult`."""
        import matplotlib.pyplot as plt

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

        # One row per signal component
        n_rows = len(next(iter(specs.values())).traces)
        fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(8.0, 1.8 * n_rows))
        axes = [axes] if n_rows == 1 else list(axes)

        # Every solution's traces, labelled by its name
        for name, spec in specs.items():
            for ax, trace in zip(axes, spec.traces):
                ax.plot(spec.t, trace.values, label=name)

        # The rows labelled from the first spec, then the legend and the title
        for ax, trace in zip(axes, next(iter(specs.values())).traces):
            unit = f" [{trace.unit}]" if trace.unit else ""
            ax.set_ylabel(trace.label + unit)
            ax.grid(True, alpha=0.3)
        axes[0].legend(loc="upper right", fontsize=8)
        axes[0].set_title("plans and runs")
        axes[-1].set_xlabel("Time [s]")

        maybe_show(plt, show)

        return plot_result_of((fig, axes))

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


def plot_cost_to_go(
    solution,
    *,
    x_axis=0,
    y_axis=1,
    anchor=None,
    bounds=None,
    grid_shape=(51, 51),
    t=0.0,
    jmax=None,
    is_3d=False,
    trajectory=None,
    title=None,
    ax=None,
    renderer="matplotlib",
    show=True,
):
    """A solution's ``cost_to_go`` over the state axes ``x_axis`` and ``y_axis``: :meth:`PlanningSolution.plot_cost_to_go`."""
    field = require_cost_to_go(solution)
    tables = getattr(solution.record, "tables", None)

    return cost_to_go_figure(
        solution.problem,
        field,
        grid=None if tables is None else tables.grid,
        family=solution.family,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        bounds=bounds,
        grid_shape=grid_shape,
        t=t,
        jmax=jmax,
        is_3d=is_3d,
        trajectory=trajectory,
        title=title,
        ax=ax,
        renderer=renderer,
        show=show,
    )


def cost_to_go_figure(
    problem,
    field,
    *,
    grid,
    family,
    x_axis,
    y_axis,
    anchor,
    bounds,
    grid_shape,
    t,
    jmax,
    is_3d,
    trajectory,
    title,
    ax,
    renderer,
    show,
):
    """
    The cost-to-go field ``J`` drawn over two state axes, the others pinned at ``anchor``: a :class:`PlotResult`.

    A grid planner's table is drawn on the grid's own nodes (``grid``, the
    field that grid's table) unless the call asks for another mesh
    (``bounds``, ``grid_shape``) or another time; any other field is sampled
    on a mesh over ``bounds``, the problem's state box by default, ``anchor``
    the goal, else the start. ``jmax`` tops the colour scale, by default the
    field's value past its grid (the failure cost) when it has one; an
    infinite ``jmax`` hands the scale to Matplotlib. ``is_3d`` adds the
    surface; ``trajectory`` overlays a run on the heatmap.
    """
    import matplotlib.pyplot as plt

    # Lazy import: the grid's drawing loads on first use, not with the planning layer
    from minilink.planning.dynamic_programming import plotting as grid_plotting

    require_matplotlib(renderer)
    grid_plotting.require_flat_overlay(is_3d, trajectory)
    jmax = colour_top(field, jmax)

    # On the grid's own nodes: the table itself
    if is_grid_table(field, grid) and on_the_nodes(bounds, grid_shape, (51, 51), t):
        figures = grid_plotting.plot_cost_to_go(
            grid,
            node_table(field),
            jmax=jmax,
            x_axis=x_axis,
            y_axis=y_axis,
            anchor=anchor,
            is_3d=is_3d,
            trajectory=trajectory,
            ax=ax,
            title="Cost-to-go" if title is None else title,
            show=show,
        )
        return plot_result_of(figures)

    # Else the field sampled on a mesh: the heatmap, then the surface and the overlay
    x_level, y_level, J = sample_field(
        problem,
        field,
        x_axis=x_axis,
        y_axis=y_axis,
        anchor=anchor,
        bounds=bounds,
        grid_shape=grid_shape,
        t=t,
    )
    title = f"cost-to-go ({family})" if title is None else title
    fig, ax = cost_to_go_heatmap(
        problem, x_level, y_level, J, x_axis, y_axis, jmax, title, ax
    )
    if is_3d:
        fig, ax = cost_to_go_surface(
            problem, x_level, y_level, J, x_axis, y_axis, jmax, title
        )
    if trajectory is not None:
        grid_plotting.overlay_trajectory(
            ax, trajectory, x_axis, y_axis, ("closed loop", "start", "end")
        )

    maybe_show(plt, show)

    return plot_result_of((fig, ax))


def plot_control_law(
    solution,
    *,
    x_axis=None,
    y_axis=None,
    u_axis=0,
    anchor=None,
    bounds=None,
    grid_shape=(101, 101),
    t=0.0,
    is_3d=False,
    trajectory=None,
    title=None,
    ax=None,
    renderer="matplotlib",
    show=True,
):
    """
    A solution's feedback law over two state axes: :meth:`PlanningSolution.plot_control_law`.

    A grid planner's law is drawn as its table on the grid's own nodes unless
    the call asks for another mesh (``bounds``, ``grid_shape``), another time
    or the surface, the other axes at ``anchor``; any other law is the
    controller's own ``plot_control_law``, ``bounds`` from the problem's box
    and the colour limits from its inputs, the other coordinates at their
    nominal values (``anchor`` refused). ``trajectory`` overlays a run on the
    heatmap.
    """
    import matplotlib.pyplot as plt

    # Lazy import: the grid's drawing loads on first use, not with the planning layer
    from minilink.planning.dynamic_programming import plotting as grid_plotting

    require_matplotlib(renderer)
    grid_plotting.require_flat_overlay(is_3d, trajectory)
    solution.require_feedback_law()
    tables = getattr(solution.record, "tables", None)
    i = 0 if x_axis is None else x_axis
    j = 1 if y_axis is None else y_axis

    # On the grid's own nodes: the law's table itself
    if (
        tables is not None
        and not is_3d
        and on_the_nodes(bounds, grid_shape, (101, 101), t)
    ):
        figures = grid_plotting.plot_control_law(
            tables,
            u_axis=u_axis,
            x_axis=i,
            y_axis=j,
            anchor=anchor,
            trajectory=trajectory,
            ax=ax,
            show=show and title is None,
        )
        result = plot_result_of(figures)
        if title is not None:
            for drawn in result if isinstance(result, list) else [result]:
                drawn.axes.set_title(title)
            maybe_show(plt, show)
        return result

    # Else the controller's own law over the problem's box, then the overlay
    require_no_anchor(anchor)
    result = controller_law_figure(
        solution,
        x_axis=x_axis,
        y_axis=y_axis,
        u_axis=u_axis,
        bounds=bounds,
        grid_shape=grid_shape,
        t=t,
        is_3d=is_3d,
        title=title,
        ax=ax,
        show=show and trajectory is None,
    )
    if trajectory is not None:
        grid_plotting.overlay_trajectory(
            result.axes, trajectory, i, j, ("closed loop", "start", "end")
        )
        maybe_show(plt, show)

    return result


def controller_law_figure(solution, *, bounds=None, **kwargs):
    """The solution's controller drawn by its own ``plot_control_law``, ``bounds`` and the colour limits filled from the problem."""
    given = dict(kwargs)
    if bounds is not None:
        given["bounds"] = bounds

    return solution.controller.plot_control_law(**control_law_kwargs(solution, given))


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


def sample_field(problem, field, *, x_axis, y_axis, anchor, bounds, grid_shape, t):
    """``(x_level, y_level, J)``: a field sampled on a mesh over the state axes ``x_axis`` and ``y_axis``, the others at ``anchor``."""
    box = state_box(problem)

    # The sweep's bounds: the state box over the two axes, unless given
    if bounds is None:
        require_bounded_state_box(box)
        i, j = x_axis, y_axis
        bounds = ((box.lower[i], box.upper[i]), (box.lower[j], box.upper[j]))

    # The other axes pinned at the anchor: the goal, else the start
    if anchor is None:
        anchor = problem.goal_point()
    anchor = np.asarray(anchor, dtype=float)

    # The mesh levels of the two axes
    x_level = np.linspace(bounds[0][0], bounds[0][1], int(grid_shape[0]))
    y_level = np.linspace(bounds[1][0], bounds[1][1], int(grid_shape[1]))

    # J(x) at every mesh point, J(x, t) away from t = 0
    J = np.empty((x_level.size, y_level.size))
    x = anchor.copy()
    for i, xi in enumerate(x_level):
        for j, yj in enumerate(y_level):
            x[x_axis], x[y_axis] = xi, yj
            J[i, j] = float(field(x) if t == 0.0 else field(x, t=t))

    return x_level, y_level, J


def plot_solution_trajectory(solution, traj, *, signals, renderer, show):
    """The solution's plan, or its controller's run ``traj``, on its system, the figure titled as plan or run."""
    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.signals.time_signals import plot_time_signals

    # The trajectory on the solution's system, shown once it is titled
    result = plot_time_signals(
        solution.problem.sys,
        traj,
        signals=signals,
        renderer=renderer,
        show=False,
    )

    # The title says which: the plan the planner computed, or the controller's run
    kind = "plan" if solution.trajectory is not None else "run"
    if renderer == "matplotlib":
        import matplotlib.pyplot as plt

        result.figure.suptitle(f"{kind} ({solution.family})")
        maybe_show(plt, show)

    return result


def control_law_kwargs(solution, kwargs) -> dict:
    """Fill ``bounds`` from the problem's box and the colour limits from its inputs, unless given."""
    kwargs = dict(kwargs)
    problem = solution.problem
    box = state_box(problem)
    n = int(problem.sys.n)

    # The sweep the policy's verb picks by default: x[0] against x[1], x[0] alone on
    # a scalar state
    x_axis = kwargs.get("x_axis")
    y_axis = kwargs.get("y_axis")
    x_axis = 0 if x_axis is None else x_axis
    y_axis = (1 if n >= 2 else None) if y_axis is None else y_axis

    # The sweep's bounds from the state box, unless given
    if "bounds" not in kwargs and box is not None:
        if y_axis is None:
            kwargs["bounds"] = (box.lower[x_axis], box.upper[x_axis])
        else:
            kwargs["bounds"] = (
                (box.lower[x_axis], box.upper[x_axis]),
                (box.lower[y_axis], box.upper[y_axis]),
            )

    # The colour limits from the input set's box when it is finite, unless given
    u_box = problem.U.bounding_box()
    u_axis = kwargs.get("u_axis", 0)
    if is_finite_box(u_box):
        kwargs.setdefault("vmin", float(u_box.lower[u_axis]))
        kwargs.setdefault("vmax", float(u_box.upper[u_axis]))

    return kwargs


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


def require_no_anchor(anchor):
    """Refuse an anchor for a law the controller draws: it pins the other coordinates at their nominal values."""
    if anchor is not None:
        raise ValueError(
            "anchor pins the other axes of a grid planner's law; a controller's own "
            "law is drawn with the other coordinates at their nominal values"
        )


def require_matplotlib(renderer):
    """Refuse a drawing library the planning plots do not draw with: Matplotlib only."""
    if renderer != "matplotlib":
        raise ValueError(
            f"the planning plots draw with renderer='matplotlib' only, got {renderer!r}"
        )


def colour_top(field, jmax):
    """The top of the cost-to-go colour scale: ``jmax``, else the field's value past its grid; an infinite one is no top (``None``)."""
    # Lazy import: the grid's drawing loads on first use, not with the planning layer
    from minilink.planning.dynamic_programming.plotting import cost_scale_limit

    if jmax is None:
        jmax = (field.params or {}).get("outside")

    return cost_scale_limit(jmax)


def is_grid_table(field, grid) -> bool:
    """Whether ``field`` is the table of ``grid``: a grid field on its levels, one value per node."""
    if grid is None or not isinstance(field, GridField):
        return False

    return node_table(field).size == grid.n_nodes and field.n == len(grid.x_levels)


def on_the_nodes(bounds, grid_shape, default_shape, t) -> bool:
    """Whether a call reads a table on its own nodes: no other mesh asked for (``bounds``, ``grid_shape``), at the first time."""
    return bounds is None and tuple(grid_shape) == default_shape and t == 0.0


def node_table(field) -> np.ndarray:
    """A grid field's values by node id ``(n_nodes,)``: its first table on a schedule (stage 0)."""
    values = np.asarray(field.params["values"], dtype=float)
    if field.timed:
        values = values[0]

    return values.reshape(-1)


def plot_result_of(figures):
    """``(fig, ax)`` as a :class:`PlotResult`, or a list of them for several slices."""
    # Lazy import: the graphics load on first use, not with the planning layer
    from minilink.graphical.common.plot_result import PlotResult

    if isinstance(figures, list):
        return [plot_result_of(item) for item in figures]
    fig, axes = figures

    return PlotResult(renderer="matplotlib", payload=(fig, axes), figure=fig, axes=axes)


def cost_to_go_heatmap(problem, x_level, y_level, J, x_axis, y_axis, jmax, title, ax):
    """The heatmap of a sampled field on the colour scale ``0 ≤ J ≤ jmax``, its colour bar, the state labels: ``(fig, ax)``."""
    import matplotlib.pyplot as plt

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure

    # The heatmap on the colour scale 0 ≤ J ≤ jmax, and its colour bar
    mesh = ax.pcolormesh(x_level, y_level, J.T, shading="gouraud", cmap="YlOrRd")
    mesh.set_clim(vmin=0.0, vmax=jmax)
    fig.colorbar(mesh, ax=ax)

    # The state labels and the title
    labels = state_labels(problem.sys)
    ax.set_xlabel(labels[x_axis])
    ax.set_ylabel(labels[y_axis])
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    return fig, ax


def cost_to_go_surface(problem, x_level, y_level, J, x_axis, y_axis, jmax, title):
    """The surface of a sampled field, clipped to ``0 ≤ J ≤ jmax``, on a figure of its own: ``(fig, ax)``."""
    import matplotlib.pyplot as plt

    # J clipped to the colour scale, over the mesh of the two levels
    J_plot = np.clip(J, 0.0, np.inf if jmax is None else jmax)
    mesh_x, mesh_y = np.meshgrid(x_level, y_level)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    surf = ax.plot_surface(mesh_x, mesh_y, J_plot.T, cmap="YlOrRd", linewidth=0)
    labels = state_labels(problem.sys)
    ax.set_xlabel(labels[x_axis])
    ax.set_ylabel(labels[y_axis])
    ax.set_zlabel("J")
    ax.set_title(title)
    fig.colorbar(surf, ax=ax, shrink=0.6)

    return fig, ax


def panels(plt, n):
    """One row of ``n`` panels side by side: the figure and the list of its axes."""
    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.0), squeeze=False)
    return fig, list(axes[0])


def require_some_cost_to_go(fields):
    """Refuse an overlay of cost-to-go fields when no solution carries one."""
    if not fields:
        raise ValueError("no solution carries a cost_to_go")


def state_labels(sys) -> list:
    """The state's axis labels with their units, ``x[i]`` where the state names none."""
    labels = getattr(sys.state, "labels", None)
    units = getattr(sys.state, "units", None)
    n = int(sys.n)
    return [
        (labels[i] if labels else f"x[{i}]")
        + (f" [{units[i]}]" if units and units[i] else "")
        for i in range(n)
    ]


def state_box(problem):
    """The finite box to sweep: the problem's constraint set, else the plant's state box, else ``None``."""
    box = problem.X.bounding_box()
    if is_finite_box(box):
        return box
    box = problem.sys.state.box
    return box if is_finite_box(box) else None


def require_bounded_state_box(box):
    """Refuse a sweep without ``bounds`` over an unbounded state box."""
    if box is None:
        raise ValueError(
            "the problem's state box is unbounded; pass bounds=((xlo, xhi), (ylo, yhi))"
        )


__all__ = [
    "compare",
    "Comparison",
    "plot_cost_to_go",
    "sample_cost_to_go",
]
