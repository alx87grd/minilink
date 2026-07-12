"""Explicit Euler integration on the simulation time grid."""

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend


class EulerSolverBackend(SolverBackend):
    """Explicit Euler on the ``times`` grid (fixed ``dt`` between knots)."""

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
        t0 = times[0]
        dt = times[1] - times[0]
        n_steps = n_pts - 1
        x_seq = evaluator.euler_integrate_ivp(x0, t0, dt, n_steps)
        x_traj = np.asarray(x_seq).T

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
        t0 = times[0]
        dt = times[1] - times[0]
        # Hold u[:, i] over each interval → ZOH sequence of length n_pts - 1
        u_sequence = np.asarray(u[:, : n_pts - 1].T)
        x_seq = evaluator.euler_integrate_zoh(x0, u_sequence, t0, dt)
        x_traj = np.asarray(x_seq).T

        self.last_debug = {
            "solver": "euler",
            "mode": "forced",
            "nfev": max(n_pts - 1, 0),
            "njev": 0,
            "nlu": 0,
            "n_t": n_pts,
        }
        return x_traj
