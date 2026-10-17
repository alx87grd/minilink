"""Variable-step explicit Euler on the simulation time grid."""

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend, fixed_step_debug

###############################################################################
# Public API
###############################################################################


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
            # x_{k+1} = x_k + Δt_k f(x_k, ū, t_k), with Δt_k = t_{k+1} − t_k
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step_ivp(x_traj[:, i], times[i], dt)

        # The run's record: one evaluation of f per step
        self.last_debug = fixed_step_debug("euler", "nominal", n_pts, 1)

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
            # x_{k+1} = x_k + Δt_k f(x_k, u_k, t_k), u_k held over the interval
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step(x_traj[:, i], u[:, i], times[i], dt)

        # The run's record: one evaluation of f per step
        self.last_debug = fixed_step_debug("euler", "forced", n_pts, 1)

        return x_traj
