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

    Parameters
    ----------
    grid : StateSpaceGrid
        Grid the policy is defined on.
    pi : array of int, shape (nodes_n,)
        Greedy policy as action ids by node id.
    interpolation : {"linear", "nearest"}, optional
        Per-axis input interpolation method.
    """

    INTERPOLATIONS = ("linear", "nearest")

    def __init__(self, grid, pi, *, interpolation: str = "linear") -> None:
        require_interpolation(interpolation, self.INTERPOLATIONS)

        super().__init__()

        # The grid and the policy's action ids; the rule and the grid's input box
        # are structure
        self.grid = grid
        self.pi = np.asarray(pi, dtype=int)
        self.interpolation = interpolation
        self.u_lb = np.asarray(grid.u_lb, dtype=float)
        self.u_ub = np.asarray(grid.u_ub, dtype=float)

        # Parameters: the input at every node and the grid's levels
        n = grid.n
        m = grid.m
        u_table = grid.input_from_policy(self.pi).reshape(*grid.x_grid_shape, m)
        levels = {
            str(i): np.asarray(level, dtype=float)
            for i, level in enumerate(grid.x_levels)
        }
        self.params = {"u_table": u_table, "levels": levels}

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

    def action(self, x, params=None) -> np.ndarray:
        """Return the interpolated command ``u`` at a single state ``x``."""
        params = self.params if params is None else params
        xp = array_module(x, params["u_table"])

        u_table = params["u_table"]
        n = self.grid.n
        nearest = self.interpolation == "nearest"
        u_lb = self.u_lb
        u_ub = self.u_ub

        levels = levels_of(params, n, xp)
        X = xp.reshape(xp.asarray(x, dtype=float), (1, -1))

        # u read on the grid: multilinear in the cell holding x (or its nearest node),
        # the edge cell extended past the grid
        u = grid_value(levels, u_table, X, nearest=nearest)[0]

        # The command inside the grid's input box: the corners at a bound sum to it
        u = xp.clip(u, u_lb, u_ub)

        return u

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u, params)


###############################################################################
# Internal machinery
###############################################################################


def require_interpolation(interpolation, supported):
    """Refuse an interpolation the lookup table does not have."""
    if interpolation not in supported:
        raise ValueError(
            f"interpolation must be one of {supported}, got {interpolation!r}"
        )
