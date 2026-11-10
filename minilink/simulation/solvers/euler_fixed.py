"""Fixed-step explicit Euler on a uniform simulation time grid."""

import numpy as np

from minilink.simulation.solvers.solver import (
    SolverBackend,
    fixed_step_debug,
    require_uniform_times,
)

###############################################################################
# Public API
###############################################################################


class EulerFixedStepSolverBackend(SolverBackend):
    """Fixed-step Euler via compiled rollout primitives (uniform ``times`` only); a diagram's ``record`` read off each step's pass."""

    records_from_pass = True

    def __init__(self) -> None:
        self.last_debug = None

    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
        params=None,
        record=(),
    ) -> np.ndarray:
        n_pts = times.shape[0]
        t0 = times[0]
        dt = require_uniform_times(times, "euler")
        n_steps = n_pts - 1

        # x_{k+1} = x_k + Δt f(x_k, ū, t_k), n_steps steps from x₀ in one compiled rollout
        run = evaluator.rollout(
            x0, None, t0, dt, params, solver="euler", n_steps=n_steps, record=record
        )
        x_seq, records = run if record else (run, None)
        x_traj = np.asarray(x_seq).T  # (n, n_pts)

        # The run's record: one evaluation of f per step
        self.last_debug = fixed_step_debug("euler", "nominal", n_pts, 1)

        return x_traj if not record else (x_traj, records)

    def integrate_forced(
        self,
        evaluator,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args=None,
        params=None,
        record=(),
    ) -> np.ndarray:
        n_pts = times.shape[0]
        t0 = times[0]
        dt = require_uniform_times(times, "euler")

        # The samples that drive a step: the last one starts none
        u_sequence = np.asarray(u[:, : n_pts - 1].T)

        # x_{k+1} = x_k + Δt f(x_k, u_k, t_k), each u_k held, in one compiled rollout
        run = evaluator.rollout(
            x0, u_sequence, t0, dt, params, solver="euler", record=record
        )
        x_seq, records = run if record else (run, None)
        x_traj = np.asarray(x_seq).T

        # The run's record: one evaluation of f per step
        self.last_debug = fixed_step_debug("euler", "forced", n_pts, 1)

        return x_traj if not record else (x_traj, records)
