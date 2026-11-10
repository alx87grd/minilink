"""Lookup-table feedback: a dynamic-programming policy as a state-feedback block."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.fields import grid_value, levels_of
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class LookupPolicy(System):
    """
    State-feedback controller backed by a policy lookup table.

    Wraps a discrete policy ``pi`` (action ids over a
    :class:`~minilink.planning.dynamic_programming.discretizer.StateSpaceGrid`) as a
    state-feedback block ``u = pi(x)``. Its input port is named ``x``, so it
    composes with a plant through ``controller @ plant`` and simulates like any
    other block. Each input axis is interpolated on the grid (multilinear, the
    edge cells extended past the grid, or the nearest node), so the discrete
    table yields a smooth command, clipped to the grid's input box (the corners
    of a cell at the bound interpolate to the bound, not past it by rounding);
    the law traces under JAX. ``params`` is
    ``{"u_table", "levels"}``: the input at every node, ``(*grid_shape, m)``, and
    each axis's levels by position.

    A finite horizon's law is a schedule: ``pi`` stacks the stages' tables
    ``(N, n_nodes)`` on the stage times ``t`` ``(N + 1,)``, and the law holds
    stage ``i`` for ``t_i ≤ t < t_{i+1}`` (a tick a hair short of ``t_i``, by
    float rounding, counts in stage ``i``). Outside ``[t_0, t_N)`` it applies
    stage 0, the full-horizon law: past the horizon, the stationary law
    approximated without a second solve. ``params`` then also holds ``t``, and
    ``u_table`` is ``(N, *grid_shape, m)``.

    Parameters
    ----------
    grid : StateSpaceGrid
        Grid the policy is defined on.
    pi : array of int, shape (n_nodes,) or (N, n_nodes)
        Greedy policy as action ids by node id, or one per stage.
    t : array, shape (N + 1,), optional
        The stage times of a stacked ``pi``.
    interpolation : {"linear", "nearest"}, optional
        Per-axis input interpolation method.
    """

    INTERPOLATIONS = ("linear", "nearest")

    def __init__(self, grid, pi, t=None, *, interpolation: str = "linear") -> None:
        require_interpolation(interpolation, self.INTERPOLATIONS)
        require_stages(pi, t)

        super().__init__()

        # The grid and the policy's action ids; the rule, the schedule and the grid's
        # input box are structure
        self.grid = grid
        self.pi = np.asarray(pi, dtype=int)
        self.interpolation = interpolation
        self.timed = t is not None
        self.u_lb = np.asarray(grid.u_lb, dtype=float)
        self.u_ub = np.asarray(grid.u_ub, dtype=float)

        # Parameters: the input at every node (per stage on a schedule), the grid's
        # levels, and the stage times
        n = grid.n
        m = grid.m
        stages = self.pi.shape[:-1]
        u_table = grid.input_from_policy(self.pi).reshape(
            *stages, *grid.x_grid_shape, m
        )
        levels = {
            str(i): np.asarray(level, dtype=float)
            for i, level in enumerate(grid.x_levels)
        }
        self.params = {"u_table": u_table, "levels": levels}
        if self.timed:
            self.params["t"] = np.asarray(t, dtype=float).reshape(-1)

        # Ports: their bounds mirror the grid's domain (plot sweeps and color limits)
        X = grid.X
        U = grid.U
        self.add_input_port("x", dim=n, lower_bound=X.lower, upper_bound=X.upper)
        self.add_output_port(
            "u",
            dim=m,
            function=self.ctl,
            dependencies=("x",),
            lower_bound=U.box.lower,
            upper_bound=U.box.upper,
        )

    def action(self, x, params=None, *, t=0.0) -> np.ndarray:
        """Return the interpolated command ``u`` at a single state ``x`` (on a schedule, the stage of ``t``)."""
        params = self.params if params is None else params
        xp = array_module(x, params["u_table"])

        n = self.grid.n
        nearest = self.interpolation == "nearest"
        u_lb = self.u_lb
        u_ub = self.u_ub
        u_table = xp.asarray(params["u_table"])
        levels = levels_of(params, n, xp)

        x = xp.reshape(xp.asarray(x, dtype=float), (-1,))

        # On a schedule, the table of the stage of t
        if self.timed:
            u_table = u_table[stage_of(xp.asarray(params["t"]), t, xp)]

        # u read on the grid: multilinear in the cell holding x (or its nearest node),
        # the edge cell extended past the grid
        u = grid_value(levels, u_table, x, nearest=nearest)

        # The command inside the grid's input box: the corners at a bound sum to it
        u = xp.clip(u, u_lb, u_ub)

        return u

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u, params, t=t)


###############################################################################
# Internal machinery
###############################################################################


def stage_of(times, t, xp):
    """
    The stage ``i`` with ``t_i ≤ t < t_{i+1}`` of the stage times ``t_0 … t_N``; stage 0 outside ``[t_0, t_N)``.

    A tick a hair short of ``t_i`` (float rounding, ``1e-9`` of a period)
    counts in stage ``i``, so ``t0 + i dt`` computed any way reads stage
    ``i``. Traced under JAX.
    """
    N = times.shape[0] - 1
    tol = 1e-9 * (times[1] - times[0])

    # t_i ≤ t + tol < t_{i+1}
    i = xp.searchsorted(times, t + tol, side="right") - 1

    # Before t_0 or past t_N: stage 0, the full-horizon law
    i = xp.where((i >= 0) & (i < N), i, 0)

    return i


def require_stages(pi, t):
    """Refuse a stacked ``pi`` without its stage times, or times that do not bound its stages."""
    pi = np.asarray(pi)
    if t is None:
        if pi.ndim != 1:
            raise ValueError(
                f"a stacked policy {pi.shape} needs its stage times t (N + 1,)"
            )
        return
    t = np.asarray(t, dtype=float).reshape(-1)
    if pi.ndim != 2 or t.size != pi.shape[0] + 1 or np.any(np.diff(t) <= 0.0):
        raise ValueError(
            f"a schedule takes pi (N, n_nodes) and increasing stage times t (N + 1,), "
            f"got pi {pi.shape} and t {t.shape}"
        )


def require_interpolation(interpolation, supported):
    """Refuse an interpolation the lookup table does not have."""
    if interpolation not in supported:
        raise ValueError(
            f"interpolation must be one of {supported}, got {interpolation!r}"
        )
