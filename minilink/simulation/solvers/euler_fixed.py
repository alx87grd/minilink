"""Fixed-step explicit Euler on a uniform simulation time grid."""

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend, require_uniform_times

###############################################################################
# Public API
###############################################################################


class EulerFixedStepSolverBackend(SolverBackend):
    """Fixed-step Euler via compiled rollout primitives (uniform ``times`` only)."""

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
        dt = require_uniform_times(times, "euler_fixedsteps")
        n_steps = n_pts - 1

        # x_{k+1} = x_k + Δt f(x_k, ū, t_k), n_steps steps from x₀ in one compiled rollout
        x_seq = evaluator.euler_integrate_ivp(x0, t0, dt, n_steps)  # (n_pts, n)
        x_traj = np.asarray(x_seq).T  # (n, n_pts)

        # The run's record: one evaluation of f per step
        self.last_debug = {
            "solver": "euler_fixedsteps",
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
        dt = require_uniform_times(times, "euler_fixedsteps")

        # The samples that drive a step: the last one starts none
        u_sequence = np.asarray(u[:, : n_pts - 1].T)

        # x_{k+1} = x_k + Δt f(x_k, u_k, t_k), each u_k held, in one compiled rollout
        x_seq = evaluator.euler_integrate_zoh(x0, u_sequence, t0, dt)
        x_traj = np.asarray(x_seq).T

        # The run's record: one evaluation of f per step
        self.last_debug = {
            "solver": "euler_fixedsteps",
            "mode": "forced",
            "nfev": max(n_pts - 1, 0),
            "njev": 0,
            "nlu": 0,
            "n_t": n_pts,
        }

        return x_traj
