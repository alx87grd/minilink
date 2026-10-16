"""Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the plans."""

from __future__ import annotations

from copy import copy
from dataclasses import dataclass, replace

import numpy as np

from minilink.core.sets import is_finite_box
from minilink.planning.evaluation import Evaluation, MonteCarloEvaluator
from minilink.planning.results import PlanningSolution

# Matplotlib is imported inside each view, so the planning layer stays importable
# without a display backend until a plot is requested.


###############################################################################
# Public API
###############################################################################


def compare(*args, **solutions) -> Comparison:
    """
    Name solutions of one problem for a side-by-side reading.

    ``compare(VI=vi, LQR=lqr)`` or ``compare({"VI": vi, "LQR": lqr})``.
    """
    require_named(args)

    # The dict passed positionally, then the keyword solutions over it
    named = dict(args[0]) if args else {}
    named.update(solutions)
    require_solutions(named)

    return Comparison(named)


@dataclass(frozen=True)
class Comparison:
    """
    Named solutions of one problem and the overlays that read them together.

    ``solutions`` maps a name to a :class:`~minilink.planning.results.PlanningSolution`
    (``compare(VI=vi, LQR=lqr, PPO=ppo)`` builds one). ``print`` is the table of
    their records, and each ``plot_*`` draws every solution on one figure with
    one scale. The verbs read what the planners produced and nothing else:
    closing a loop and simulating it stays in the script, ``solution.policy @
    plant``. The first solution's system labels the axes.
    """

    solutions: dict

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

    def evaluate(self, evaluator) -> Comparison:
        """
        Score every policy with one evaluator, the same draws for all.

        Returns a new comparison whose solutions carry that ``evaluation``,
        so ``print(race.evaluate(evaluator))`` is the table with one yardstick.
        A Monte Carlo evaluator's ``backend="auto"`` is chosen once for the
        whole comparison: NumPy for every policy when one of them does not
        trace on JAX, JAX otherwise.
        """
        # One backend for every policy, resolved on a copy of the evaluator
        if isinstance(evaluator, MonteCarloEvaluator) and evaluator.backend == "auto":
            policies = [solution.policy for solution in self.solutions.values()]
            evaluator = copy(evaluator)
            evaluator.backend = evaluator.auto_backend(*policies)

        # Each solution with its evaluation under the one evaluator
        scored = {
            name: replace(solution, evaluation=evaluator.evaluate(solution))
            for name, solution in self.items()
        }

        return Comparison(scored)

    def plot_control_law(self, *, show=True, **kwargs):
        """One panel per feedback law, side by side, one colour scale from the problem's inputs."""
        import matplotlib.pyplot as plt

        # One panel per law, titled by its name
        fig, axes = panels(plt, len(self))
        for ax, (name, solution) in zip(axes, self.items()):
            solution.plot_control_law(ax=ax, show=False, **kwargs)
            ax.set_title(name)

        maybe_show(plt, show)

        return fig, axes

    def plot_cost_to_go(self, *, jmax=None, show=True, **kwargs):
        """One panel per cost-to-go field, side by side, one colour scale (``jmax`` or the largest finite value)."""
        import matplotlib.pyplot as plt

        # The solutions that carry a cost-to-go field
        fields = {
            name: solution
            for name, solution in self.items()
            if solution.cost_to_go is not None
        }
        require_some_cost_to_go(fields)

        # One colour scale: jmax, else the largest value sampled over every field
        if jmax is None:
            jmax = max(
                float(np.nanmax(sample_cost_to_go(solution, **kwargs)[2]))
                for solution in fields.values()
            )

        # One panel per field, titled by its name
        fig, axes = panels(plt, len(fields))
        for ax, (name, solution) in zip(axes, fields.items()):
            plot_cost_to_go(solution, ax=ax, show=False, jmax=jmax, **kwargs)
            ax.set_title(name)

        maybe_show(plt, show)

        return fig, axes

    def plot_trajectory(self, *, signals=("x", "u"), show=True):
        """The plans and nominal rollouts overlaid, one row per signal component, legend by name."""
        import matplotlib.pyplot as plt

        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import build_signal_plot_spec

        # One plot spec per solution, on the first solution's system
        first = next(iter(self.solutions.values()))
        sys = first.problem.sys
        specs = {
            name: build_signal_plot_spec(
                sys, solution.require_trajectory(), signals=signals
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
        axes[0].set_title("plans and nominal rollouts")
        axes[-1].set_xlabel("Time [s]")

        maybe_show(plt, show)

        return fig, axes

    # Display

    def __str__(self) -> str:
        # One row per solution: its name, success, solver record and evaluation
        rows = [
            (
                name,
                "yes" if solution.success else "no",
                str(solution.solver),
                "-" if solution.evaluation is None else str(solution.evaluation),
            )
            for name, solution in self.items()
        ]
        header = ("", "success", "solver", "evaluation")

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
    axes=(0, 1),
    anchor=None,
    bounds=None,
    grid_shape=(51, 51),
    jmax=None,
    t=0.0,
    cmap="YlOrRd",
    ax=None,
    title=None,
    show=True,
):
    """
    Heatmap of a solution's ``cost_to_go`` over two state axes, the others pinned at ``anchor``.

    ``bounds`` (``((xlo, xhi), (ylo, yhi))``) defaults to the problem's
    constraint box, else the plant's state box; ``anchor`` to the goal,
    else the start; ``jmax`` clips the colour scale.
    """
    import matplotlib.pyplot as plt

    # The field sampled on a mesh over the two axes
    x_level, y_level, J = sample_cost_to_go(
        solution, axes=axes, anchor=anchor, bounds=bounds, grid_shape=grid_shape, t=t
    )

    # A new figure, or the one that holds the given axes
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure

    # The heatmap on the colour scale 0 ≤ J ≤ jmax, and its colour bar
    mesh = ax.pcolormesh(x_level, y_level, J.T, shading="gouraud", cmap=cmap)
    mesh.set_clim(vmin=0.0, vmax=jmax)
    fig.colorbar(mesh, ax=ax)

    # The state labels and the title
    labels = state_labels(solution.problem.sys)
    ax.set_xlabel(labels[axes[0]])
    ax.set_ylabel(labels[axes[1]])
    ax.set_title(f"cost-to-go ({solution.method})" if title is None else title)
    ax.grid(True, alpha=0.3)

    maybe_show(plt, show)

    return fig, ax


def sample_cost_to_go(
    solution, *, axes=(0, 1), anchor=None, bounds=None, grid_shape=(51, 51), t=0.0
):
    """``(x_level, y_level, J)``: the cost-to-go sampled on a mesh over two state axes."""
    field = require_cost_to_go(solution)
    problem = solution.problem
    box = state_box(problem)

    # The sweep's bounds: the state box over the two axes, unless given
    if bounds is None:
        require_bounded_state_box(box)
        i, j = axes
        bounds = ((box.lower[i], box.upper[i]), (box.lower[j], box.upper[j]))

    # The other axes pinned at the anchor: the goal, else the start
    if anchor is None:
        anchor = problem.x_start if problem.x_goal is None else problem.x_goal
    anchor = np.asarray(anchor, dtype=float)

    # The mesh levels of the two axes
    x_level = np.linspace(bounds[0][0], bounds[0][1], int(grid_shape[0]))
    y_level = np.linspace(bounds[1][0], bounds[1][1], int(grid_shape[1]))

    # J(x) at every mesh point, J(x, t) away from t = 0
    J = np.empty((x_level.size, y_level.size))
    x = anchor.copy()
    for i, xi in enumerate(x_level):
        for j, yj in enumerate(y_level):
            x[axes[0]], x[axes[1]] = xi, yj
            J[i, j] = float(field(x) if t == 0.0 else field(x, t))

    return x_level, y_level, J


def plot_solution_trajectory(solution, *, signals, backend, show):
    """The solution's trajectory on its system, the figure titled as plan or nominal rollout."""
    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.signals.time_signals import plot_time_signals

    # The trajectory on the solution's system, shown once it is titled
    result = plot_time_signals(
        solution.problem.sys,
        solution.require_trajectory(),
        signals=signals,
        backend=backend,
        show=False,
    )

    # The title says which: the plan of an open-loop policy, or the nominal rollout
    kind = "plan" if solution.open_loop else "nominal rollout"
    if backend == "matplotlib":
        import matplotlib.pyplot as plt

        result.figure.suptitle(f"{kind} ({solution.method})")
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


def require_named(args):
    """Refuse positional solutions other than one dict of named ones."""
    if len(args) > 1 or (args and not isinstance(args[0], dict)):
        raise TypeError("compare takes named solutions: compare(VI=vi, LQR=lqr)")


def require_solutions(named):
    """Refuse an empty comparison, or a named entry that is not a ``PlanningSolution``."""
    if not named:
        raise ValueError("compare needs at least one solution")
    for name, solution in named.items():
        if not isinstance(solution, PlanningSolution):
            raise TypeError(f"{name!r} is not a PlanningSolution")


def panels(plt, n):
    """One row of ``n`` panels side by side: the figure and the list of its axes."""
    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.0), squeeze=False)
    return fig, list(axes[0])


def maybe_show(plt, show):
    """Show the figures, unless asked not to or on the non-interactive Agg backend."""
    if show and plt.get_backend().lower() != "agg":
        plt.show()


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


def require_cost_to_go(solution):
    """Refuse a solution that carries no cost-to-go field; return the field."""
    field = solution.cost_to_go
    if field is None:
        raise ValueError(f"this {solution.method} solution carries no cost_to_go")

    return field


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
    "Evaluation",
]
