"""Fixed-step fourth-order Runge–Kutta on the simulation time grid."""

from __future__ import annotations

from typing import Any

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend


class RK4SolverBackend(SolverBackend):
    """Fixed-step RK4 on the ``times`` grid."""

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

        # Fixed-step rollout through evaluator primitive
        t0 = times[0]
        dt = times[1] - times[0]
        n_steps = n_pts - 1
        x_seq = evaluator.rk4_rollout_ivp(x0, t0, dt, n_steps)  # (n_pts, n)
        x_traj = np.asarray(x_seq).T  # (n, n_pts)

        # Debug information
        self.last_debug = {
            "solver": "rk4",
            "mode": "nominal",
            "nfev": 4 * max(n_pts - 1, 0),
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
        raise NotImplementedError("RK4SolverBackend supports nominal mode only for now")
