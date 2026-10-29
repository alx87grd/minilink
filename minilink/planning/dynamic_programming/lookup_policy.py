"""Lookup-table feedback: a dynamic-programming policy as a state-feedback block."""

import itertools

import numpy as np

from minilink.core.backends import array_module
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
    table yields a smooth command; the law traces under JAX. ``params`` is
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

        # The grid and the policy's action ids; the rule is structure
        self.grid = grid
        self.pi = np.asarray(pi, dtype=int)
        self.interpolation = interpolation

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

        x = xp.reshape(xp.asarray(x, dtype=float), (-1,))

        # On each axis k the cell l_{i_k} ≤ x_k < l_{i_k + 1} (the edge cell past the
        # grid) and the position in it, y_k = (x_k − l_{i_k}) / (l_{i_k + 1} − l_{i_k})
        cells = []
        for k in range(n):
            level = xp.asarray(params["levels"][str(k)])
            i = xp.clip(xp.searchsorted(level, x[k]) - 1, 0, level.shape[0] - 2)
            y = (x[k] - level[i]) / (level[i + 1] - level[i])
            cells.append((i, y))

        # Nearest: the node of each axis's closer end, the lower one on a tie
        if nearest:
            index = tuple(xp.where(y <= 0.5, i, i + 1) for i, y in cells)
            u = u_table[index]

        # Linear: u = Σ_corners u_corner Πₖ (y_k or 1 − y_k) over the cell's 2ⁿ corners
        else:
            u = 0.0
            for corner in itertools.product((0, 1), repeat=n):
                index = tuple(i + upper for (i, _), upper in zip(cells, corner))
                term = u_table[index]
                for (_, y), upper in zip(cells, corner):
                    term = term * (y if upper else 1.0 - y)
                u = u + term

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
