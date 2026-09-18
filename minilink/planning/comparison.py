"""
Several solutions of one problem side by side: a table, the laws, the cost-to-go fields, the plans.

``compare(VI=vi, LQR=lqr, PPO=ppo)`` holds named
:class:`~minilink.planning.results.PlanningSolution` objects; ``print`` is
the table of their records, and each ``plot_*`` draws every solution on one
figure with one scale. The verbs read what the planners produced and
nothing else: closing a loop and simulating it stays in the script,
``solution.policy @ plant``. Matplotlib is imported lazily.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from minilink.core.sets import is_finite_box
from minilink.planning.evaluation import Evaluation
from minilink.planning.results import PlanningSolution

# Public API


def compare(*args, **solutions) -> "Comparison":
    """
    Name solutions of one problem for a side-by-side reading.

    ``compare(VI=vi, LQR=lqr)`` or ``compare({"VI": vi, "LQR": lqr})``.
    """
    if len(args) > 1 or (args and not isinstance(args[0], dict)):
        raise TypeError("compare takes named solutions: compare(VI=vi, LQR=lqr)")
    named = dict(args[0]) if args else {}
    named.update(solutions)
    if not named:
        raise ValueError("compare needs at least one solution")
    for name, solution in named.items():
        if not isinstance(solution, PlanningSolution):
            raise TypeError(f"{name!r} is not a PlanningSolution")
    return Comparison(named)


@dataclass(frozen=True)
class Comparison:
    """
    Named solutions of one problem and the overlays that read them together.

    ``solutions`` maps a name to a :class:`~minilink.planning.results.PlanningSolution`.
    The first solution's system labels the axes.
    """

    solutions: dict

    def __getitem__(self, name) -> PlanningSolution:
        return self.solutions[name]

    def __iter__(self):
        return iter(self.solutions)

    def __len__(self) -> int:
        return len(self.solutions)

    def items(self):
        return self.solutions.items()

    def __str__(self) -> str:
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
        widths = [
            max(len(row[k]) for row in (header, *rows)) for k in range(len(header))
        ]
        lines = [
            "  ".join(cell.ljust(width) for cell, width in zip(row, widths)).rstrip()
            for row in (header, *rows)
        ]
        return "\n".join(lines)

    def evaluate(self, evaluator) -> "Comparison":
        """
        Score every policy with one evaluator, the same draws for all.

        Returns a new comparison whose solutions carry that ``evaluation``,
        so ``print(race.evaluate(evaluator))`` is the table with one yardstick.
        """
        return Comparison(
            {
                name: replace(solution, evaluation=evaluator.evaluate(solution))
                for name, solution in self.items()
            }
        )

    def plot_control_law(self, *, show=True, **kwargs):
        """One panel per feedback law, side by side, one colour scale from the problem's inputs."""
        import matplotlib.pyplot as plt

        fig, axes = _panels(plt, len(self))
        for ax, (name, solution) in zip(axes, self.items()):
            solution.plot_control_law(ax=ax, show=False, **kwargs)
            ax.set_title(name)
        _maybe_show(plt, show)
        return fig, axes

    def plot_cost_to_go(self, *, jmax=None, show=True, **kwargs):
        """One panel per cost-to-go field, side by side, one colour scale (``jmax`` or the largest finite value)."""
        import matplotlib.pyplot as plt

        fields = {
            name: solution
            for name, solution in self.items()
            if solution.cost_to_go is not None
        }
        if not fields:
            raise ValueError("no solution carries a cost_to_go")
        if jmax is None:
            jmax = max(
                float(np.nanmax(sample_cost_to_go(solution, **kwargs)[2]))
                for solution in fields.values()
            )
        fig, axes = _panels(plt, len(fields))
        for ax, (name, solution) in zip(axes, fields.items()):
            plot_cost_to_go(solution, ax=ax, show=False, jmax=jmax, **kwargs)
            ax.set_title(name)
        _maybe_show(plt, show)
        return fig, axes

    def plot_trajectory(self, *, signals=("x", "u"), show=True):
        """The plans and nominal rollouts overlaid, one row per signal component, legend by name."""
        import matplotlib.pyplot as plt

        from minilink.graphical.signals.time_signals import build_signal_plot_spec

        first = next(iter(self.solutions.values()))
        sys = first.problem.sys
        specs = {
            name: build_signal_plot_spec(
                sys, solution.require_trajectory(), signals=signals
            )
            for name, solution in self.items()
        }
        n_rows = len(next(iter(specs.values())).traces)
        fig, axes = plt.subplots(n_rows, 1, sharex=True, figsize=(8.0, 1.8 * n_rows))
        axes = [axes] if n_rows == 1 else list(axes)
        for name, spec in specs.items():
            for ax, trace in zip(axes, spec.traces):
                ax.plot(spec.t, trace.values, label=name)
        for ax, trace in zip(axes, next(iter(specs.values())).traces):
            unit = f" [{trace.unit}]" if trace.unit else ""
            ax.set_ylabel(trace.label + unit)
            ax.grid(True, alpha=0.3)
        axes[0].legend(loc="upper right", fontsize=8)
        axes[0].set_title("plans and nominal rollouts")
        axes[-1].set_xlabel("Time [s]")
        _maybe_show(plt, show)
        return fig, axes


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

    x_level, y_level, J = sample_cost_to_go(
        solution, axes=axes, anchor=anchor, bounds=bounds, grid_shape=grid_shape, t=t
    )
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure
    mesh = ax.pcolormesh(x_level, y_level, J.T, shading="gouraud", cmap=cmap)
    mesh.set_clim(vmin=0.0, vmax=jmax)
    fig.colorbar(mesh, ax=ax)
    labels = state_labels(solution.problem.sys)
    ax.set_xlabel(labels[axes[0]])
    ax.set_ylabel(labels[axes[1]])
    ax.set_title(f"cost-to-go ({solution.method})" if title is None else title)
    ax.grid(True, alpha=0.3)
    _maybe_show(plt, show)
    return fig, ax


def sample_cost_to_go(
    solution, *, axes=(0, 1), anchor=None, bounds=None, grid_shape=(51, 51), t=0.0
):
    """``(x_level, y_level, J)``: the cost-to-go sampled on a mesh over two state axes."""
    field = solution.cost_to_go
    if field is None:
        raise ValueError(f"this {solution.method} solution carries no cost_to_go")
    problem = solution.problem
    box = state_box(problem)
    if bounds is None:
        if box is None:
            raise ValueError(
                "the problem's state box is unbounded; pass bounds=((xlo, xhi), (ylo, yhi))"
            )
        i, j = axes
        bounds = ((box.lower[i], box.upper[i]), (box.lower[j], box.upper[j]))
    if anchor is None:
        anchor = problem.x_start if problem.x_goal is None else problem.x_goal
    anchor = np.asarray(anchor, dtype=float)
    x_level = np.linspace(bounds[0][0], bounds[0][1], int(grid_shape[0]))
    y_level = np.linspace(bounds[1][0], bounds[1][1], int(grid_shape[1]))

    J = np.empty((x_level.size, y_level.size))
    x = anchor.copy()
    for i, xi in enumerate(x_level):
        for j, yj in enumerate(y_level):
            x[axes[0]], x[axes[1]] = xi, yj
            J[i, j] = float(field(x) if t == 0.0 else field(x, t))
    return x_level, y_level, J


def plot_solution_trajectory(solution, *, signals, backend, show):
    """The solution's trajectory on its system, the figure titled as plan or nominal rollout."""
    from minilink.graphical.signals.time_signals import plot_time_signals

    result = plot_time_signals(
        solution.problem.sys,
        solution.require_trajectory(),
        signals=signals,
        backend=backend,
        show=False,
    )
    kind = "plan" if solution.open_loop else "nominal rollout"
    if backend == "matplotlib":
        import matplotlib.pyplot as plt

        result.figure.suptitle(f"{kind} ({solution.method})")
        _maybe_show(plt, show)
    return result


def control_law_kwargs(solution, kwargs) -> dict:
    """Fill ``bounds`` from the problem's box and the colour limits from its inputs, unless given."""
    kwargs = dict(kwargs)
    problem = solution.problem
    box = state_box(problem)
    # the sweep the policy's verb picks by default: x[0] against x[1], x[0] alone on a scalar state
    n = int(problem.sys.n)
    x_axis = kwargs.get("x_axis")
    y_axis = kwargs.get("y_axis")
    x_axis = 0 if x_axis is None else x_axis
    y_axis = (1 if n >= 2 else None) if y_axis is None else y_axis
    if "bounds" not in kwargs and box is not None:
        if y_axis is None:
            kwargs["bounds"] = (box.lower[x_axis], box.upper[x_axis])
        else:
            kwargs["bounds"] = (
                (box.lower[x_axis], box.upper[x_axis]),
                (box.lower[y_axis], box.upper[y_axis]),
            )
    u_box = problem.U.bounding_box()
    u_axis = kwargs.get("u_axis", 0)
    if is_finite_box(u_box):
        kwargs.setdefault("vmin", float(u_box.lower[u_axis]))
        kwargs.setdefault("vmax", float(u_box.upper[u_axis]))
    return kwargs


# Internal machinery


def state_box(problem):
    """The finite box to sweep: the problem's constraint set, else the plant's state box, else ``None``."""
    box = problem.X.bounding_box()
    if is_finite_box(box):
        return box
    box = problem.sys.state.box
    return box if is_finite_box(box) else None


def state_labels(sys) -> list:
    labels = getattr(sys.state, "labels", None)
    units = getattr(sys.state, "units", None)
    n = int(sys.n)
    return [
        (labels[i] if labels else f"x[{i}]")
        + (f" [{units[i]}]" if units and units[i] else "")
        for i in range(n)
    ]


def _panels(plt, n):
    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.0), squeeze=False)
    return fig, list(axes[0])


def _maybe_show(plt, show):
    if show and plt.get_backend().lower() != "agg":
        plt.show()


__all__ = [
    "compare",
    "Comparison",
    "plot_cost_to_go",
    "sample_cost_to_go",
    "Evaluation",
]
