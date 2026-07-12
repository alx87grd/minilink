"""Fixed-step explicit Euler on a uniform simulation time grid."""

import numpy as np

from minilink.simulation.solvers.solver import SolverBackend


def _require_uniform_times(times: np.ndarray) -> float:
    """Return the common step size; raise when knot spacing is not uniform."""
    n_pts = times.shape[0]
    if n_pts < 2:
        return 0.0
    dt = float(times[1] - times[0])
    if not np.allclose(np.diff(times), dt):
        raise ValueError(
            "euler_fixedsteps requires a uniform time grid; use solver='euler' "
            "for variable dt between knots"
        )
    return dt


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
        dt = _require_uniform_times(times)
        n_steps = n_pts - 1
        x_seq = evaluator.euler_integrate_ivp(x0, t0, dt, n_steps)
        x_traj = np.asarray(x_seq).T

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
        dt = _require_uniform_times(times)
        u_sequence = np.asarray(u[:, : n_pts - 1].T)
        x_seq = evaluator.euler_integrate_zoh(x0, u_sequence, t0, dt)
        x_traj = np.asarray(x_seq).T

        self.last_debug = {
            "solver": "euler_fixedsteps",
            "mode": "forced",
            "nfev": max(n_pts - 1, 0),
            "njev": 0,
            "nlu": 0,
            "n_t": n_pts,
        }
        return x_traj
