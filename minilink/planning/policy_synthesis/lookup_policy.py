"""
Tabular feedback controller from a dynamic-programming policy.

:class:`LookupTableController` wraps a discrete policy ``pi`` (action ids over a
:class:`~minilink.planning.policy_synthesis.discretizer.StateSpaceGrid`) as a
state-feedback block ``u = pi(x)``. It is a
:class:`~minilink.core.feedback.Controller`, so it composes with a plant
through ``controller @ plant`` and simulates like any other block. Each input
axis is interpolated independently so the discrete table yields a smooth
command.
"""

import numpy as np
from scipy.interpolate import RegularGridInterpolator

from minilink.core.feedback import Controller

# Public API


class LookupTableController(Controller):
    """
    State-feedback controller backed by a policy lookup table.

    Parameters
    ----------
    grid : StateSpaceGrid
        Grid the policy is defined on.
    pi : array of int, shape (nodes_n,)
        Greedy policy as action ids by node id.
    interpolation : {"linear", "nearest"}, optional
        Per-axis input interpolation method.
    """

    # Explicit port roles: state measurement, no boundary reference.
    measurement_port = "x"
    ref_port = None
    control_port = "u"
    plot_space = "state"

    def __init__(self, grid, pi, *, interpolation: str = "linear") -> None:
        super().__init__()
        self.name = "Lookup Table Controller"

        self.grid = grid
        self.pi = np.asarray(pi, dtype=int)
        n, m = grid.n, grid.m

        # Port bounds mirror the grid domain (plot sweeps and color limits).
        self.add_input_port("x", dim=n, lower_bound=grid.x_lb, upper_bound=grid.x_ub)
        self.add_output_port(
            "u",
            dim=m,
            function=self.ctl,
            dependencies=("x",),
            lower_bound=grid.u_lb,
            upper_bound=grid.u_ub,
        )

        # One interpolator per input axis over the policy's input table.
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
        x = np.asarray(x, dtype=float).reshape(1, -1)
        return np.array([float(interp(x)[0]) for interp in self._interp])

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u)
