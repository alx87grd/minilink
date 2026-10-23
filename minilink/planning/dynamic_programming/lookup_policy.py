"""Lookup-table feedback: a dynamic-programming policy as a state-feedback block."""

import numpy as np
from scipy.interpolate import RegularGridInterpolator

from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class LookupTableController(System):
    """
    State-feedback controller backed by a policy lookup table.

    Wraps a discrete policy ``pi`` (action ids over a
    :class:`~minilink.planning.dynamic_programming.discretizer.StateSpaceGrid`) as a
    state-feedback block ``u = pi(x)``. Its input port is named ``x``, so it
    composes with a plant through ``controller @ plant`` and simulates like any
    other block. Each input axis is interpolated independently, so the discrete
    table yields a smooth command.

    Parameters
    ----------
    grid : StateSpaceGrid
        Grid the policy is defined on.
    pi : array of int, shape (nodes_n,)
        Greedy policy as action ids by node id.
    interpolation : {"linear", "nearest"}, optional
        Per-axis input interpolation method.
    """

    def __init__(self, grid, pi, *, interpolation: str = "linear") -> None:
        super().__init__()
        self.name = "Lookup Table Controller"

        # Parameters: the grid and the policy's action ids
        self.grid = grid
        self.pi = np.asarray(pi, dtype=int)

        # Ports: their bounds mirror the grid's domain (plot sweeps and color limits)
        n = grid.n
        m = grid.m
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

        # One interpolator per input axis over the policy's input table
        u_table = grid.input_from_policy(self.pi)
        levels = tuple(grid.x_levels)
        self._interp = [
            RegularGridInterpolator(
                levels,
                u_table[:, k].reshape(grid.x_grid_shape),
                method=interpolation,
                bounds_error=False,
                fill_value=None,
            )
            for k in range(m)
        ]

    def action(self, x) -> np.ndarray:
        """Return the interpolated command ``u`` at a single state ``x``."""
        interpolators = self._interp

        x = np.asarray(x, dtype=float).reshape(1, -1)

        # Each input axis read from its own interpolated table
        u = np.array([float(interp(x)[0]) for interp in interpolators])

        return u

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u)
