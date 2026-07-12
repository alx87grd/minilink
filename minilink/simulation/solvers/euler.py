"""Variable-step explicit Euler on the simulation time grid."""

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend


class EulerSolverBackend(SolverBackend):
    """Explicit Euler with per-interval ``dt = times[i+1] - times[i]``."""

    def __init__(self) -> None:
        self.last_debug = None

    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
    ) -> np.ndarray:
        n_pts = times.shape[0]
        n = evaluator.n
        x_traj = np.zeros((n, n_pts), dtype=float)
        x_traj[:, 0] = x0

        for i in range(n_pts - 1):
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step_ivp(x_traj[:, i], times[i], dt)

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
        evaluator,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args=None,
    ) -> np.ndarray:
        n_pts = times.shape[0]
        n = evaluator.n
        x_traj = np.zeros((n, n_pts), dtype=float)
        x_traj[:, 0] = x0

        for i in range(n_pts - 1):
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step(x_traj[:, i], u[:, i], times[i], dt)

        self.last_debug = {
            "solver": "euler",
            "mode": "forced",
            "nfev": max(n_pts - 1, 0),
            "njev": 0,
            "nlu": 0,
            "n_t": n_pts,
        }
        return x_traj
