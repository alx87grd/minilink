"""Fixed-step fourth-order Runge–Kutta on the simulation time grid."""

import numpy as np

from minilink.core.compile.evaluators.stepping import step_body
from minilink.simulation.solvers.solver import (
    SolverBackend,
    fixed_step_debug,
    require_uniform_times,
)

###############################################################################
# Public API
###############################################################################


class RK4SolverBackend(SolverBackend):
    """Fixed-step RK4 on the ``times`` grid; a diagram's ``record`` read off each step's first stage."""

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
        dt = require_uniform_times(times, "rk4_fixedsteps")
        n_steps = n_pts - 1

        # x_{k+1} = x_k + Δt/6 (k₁ + 2k₂ + 2k₃ + k₄), n_steps steps from x₀ in one
        # compiled rollout, the nominal input held
        run = evaluator.rollout(
            x0, None, t0, dt, params, n_steps=n_steps, record=record
        )
        x_seq, records = run if record else (run, None)
        x_traj = np.asarray(x_seq).T  # (n, n_pts)

        # The run's record: four evaluations of f per step
        self.last_debug = fixed_step_debug("rk4_fixedsteps", "nominal", n_pts, 4)

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
        dt = require_uniform_times(times, "rk4_fixedsteps")

        # The same RK4 step, each sample u_k held over [t_k, t_{k+1}]; one compiled rollout
        run = evaluator.rollout(x0, u[:, : n_pts - 1].T, t0, dt, params, record=record)
        x_seq, records = run if record else (run, None)
        x_traj = np.asarray(x_seq).T

        # The run's record: four evaluations of f per step
        self.last_debug = fixed_step_debug("rk4_fixedsteps", "forced", n_pts, 4)

        return x_traj if not record else (x_traj, records)

    def integrate_input(self, evaluator, times, u_of_t, x0, args=None, params=None):
        """The RK4 step with the input ``u(t)`` read at each stage time, not held."""
        n_pts = times.shape[0]
        t0 = times[0]
        dt = require_uniform_times(times, "rk4_fixedsteps")
        u_unused = np.zeros(evaluator.m)

        def f(x, u, t):
            return evaluator.f(x, u_of_t(t), t, params)

        # x_{k+1} = x_k + Δt/6 (k₁ + 2k₂ + 2k₃ + k₄), each kᵢ at u(t) of its stage
        x = np.asarray(x0, dtype=float)
        x_seq = [x]
        for k in range(n_pts - 1):
            x = step_body(f, x, u_unused, u_unused, t0 + k * dt, dt, "rk4", 1)
            x_seq.append(np.asarray(x))
        x_traj = np.array(x_seq).T

        # The run's record: four evaluations of f per step
        self.last_debug = fixed_step_debug("rk4_fixedsteps", "forced", n_pts, 4)

        return x_traj
