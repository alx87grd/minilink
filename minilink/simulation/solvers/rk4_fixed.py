"""Fixed-step fourth-order Runge–Kutta on the simulation time grid."""

import numpy as np

from minilink.core.compile.evaluators.stepping import step_body
from minilink.simulation.solvers.solver import (
    SolverBackend,
    fixed_step_debug,
    require_fixed_steps,
    rollout_on_grid,
)

###############################################################################
# Public API
###############################################################################


class RK4SolverBackend(SolverBackend):
    """Fixed-step RK4 on the ``times`` grid (its last step may be shorter, landing on ``tf``); a diagram's ``record`` read off each step's first stage."""

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
        steps = require_fixed_steps(times, "rk4")
        n_steps = n_pts - 1

        # x_{k+1} = x_k + Δt/6 (k₁ + 2k₂ + 2k₃ + k₄), n_steps steps from x₀ in one
        # compiled rollout, the nominal input held
        run = rollout_on_grid(evaluator, x0, None, t0, steps, n_steps, params, record)
        x_seq, records = run if record else (run, None)
        x_traj = np.asarray(x_seq).T  # (n, n_pts)

        # The run's record: four evaluations of f per step
        self.last_debug = fixed_step_debug("rk4", "nominal", n_pts, 4)

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
        steps = require_fixed_steps(times, "rk4")
        n_steps = n_pts - 1

        # The same RK4 step, each sample u_k held over [t_k, t_{k+1}]; one compiled rollout
        u_sequence = u[:, :n_steps].T
        run = rollout_on_grid(
            evaluator, x0, u_sequence, t0, steps, n_steps, params, record
        )
        x_seq, records = run if record else (run, None)
        x_traj = np.asarray(x_seq).T

        # The run's record: four evaluations of f per step
        self.last_debug = fixed_step_debug("rk4", "forced", n_pts, 4)

        return x_traj if not record else (x_traj, records)

    def integrate_input(self, evaluator, times, u_of_t, x0, args=None, params=None):
        """The RK4 step with the input ``u(t)`` read at each stage time, not held."""
        n_pts = times.shape[0]
        t0 = times[0]
        dt, dt_last = require_fixed_steps(times, "rk4")
        n_steps = n_pts - 1
        u_unused = np.zeros(evaluator.m)

        def f(x, u, t):
            return evaluator.f(x, u_of_t(t), t, params)

        # x_{k+1} = x_k + Δt_k/6 (k₁ + 2k₂ + 2k₃ + k₄), each kᵢ at u(t) of its stage;
        # Δt_k = Δt, the last one Δt_last
        x = np.asarray(x0, dtype=float)
        x_seq = [x]
        for k in range(n_steps):
            dt_k = dt_last if k == n_steps - 1 else dt
            x = step_body(f, x, u_unused, u_unused, t0 + k * dt, dt_k, "rk4", 1)
            x_seq.append(np.asarray(x))
        x_traj = np.array(x_seq).T

        # The run's record: four evaluations of f per step
        self.last_debug = fixed_step_debug("rk4", "forced", n_pts, 4)

        return x_traj
