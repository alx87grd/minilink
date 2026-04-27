"""Explicit Euler integration on the simulation time grid."""

from __future__ import annotations

from typing import Any

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend


class EulerSolverBackend(SolverBackend):
    """Explicit Euler on the ``times`` grid (variable ``dt`` between knots)."""

    def __init__(self) -> None:
        self.last_debug = None

    def integrate(
        self,
        evaluator: Any,
        times: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:

        # Get trajectory dimensions
        n_pts = times.shape[0]
        n = evaluator.n

        # Initialize the state trajectory
        x_traj = np.zeros((n, n_pts), dtype=float)
        x_traj[:, 0] = x0

        # Integrate with explicit Euler
        for i in range(n_pts - 1):
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step_ivp(x_traj[:, i], times[i], dt)

        # Debug information
        self.last_debug = {
            "solver": "euler",
            "mode": "nominal",
            "nfev": max(n_pts - 1, 0),
            "njev": 0,
            "nlu": 0,
            "n_t": n_pts,
        }
        return x_traj

    def integrate_forced(
        self,
        evaluator: Any,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:

        # Get trajectory dimensions
        n_pts = times.shape[0]
        n = evaluator.n

        # Initialize the state trajectory
        x_traj = np.zeros((n, n_pts), dtype=float)
        x_traj[:, 0] = x0

        # Integrate with explicit Euler and forced inputs
        for i in range(n_pts - 1):
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step(x_traj[:, i], u[:, i], times[i], dt)

        # Debug information
        self.last_debug = {
            "solver": "euler",
            "mode": "forced",
            "nfev": max(n_pts - 1, 0),
            "njev": 0,
            "nlu": 0,
            "n_t": n_pts,
        }
        return x_traj
