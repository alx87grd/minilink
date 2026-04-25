"""Pluggable time integrators."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp

from minilink.simulation.input_interpolation import INPUT_INTERP_KEY, build_u_at_t


class SolverBackend(ABC):
    """
    Solve the ODE using an evaluator that implements the dynamics primitives: f(x, u, t), f_ivp(x, t)
    """

    @abstractmethod
    def integrate(
        self,
        evaluator: Any,
        times: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:
        """
        Nominal u: x(t) = x0 + int f_ivp(x(s), s) ds with f_ivp(x, t) = f(x, u_nom, t).

        evaluator — ``f_ivp(x, t)``
        times — 1D grid (n_pts,).
        x0 — initial state (n,).
        args — optional backend args.

        Returns x (n, n_pts).
        """
        ...

    @abstractmethod
    def integrate_forced(
        self,
        evaluator: Any,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:
        """
        Forced u: x(t) = x0 + int f(x(s), u(s), s) ds.

        evaluator — ``f(x, u, t)``
        times — 1D grid (n_pts,).
        u — (m, n_pts), column k at times[k].
        x0 — (n,).
        args — optional backend args.

        Returns x (n, n_pts).
        """
        ...


class SciPySolverBackend(SolverBackend):
    """SciPy solve_ivp backend."""

    def __init__(self) -> None:
        self.last_debug = None
        self.last_solve_ivp_solution = None

    def _finalize_solution(self, sol, *, mode, method, extra_debug=None):
        self.last_solve_ivp_solution = sol
        debug = {
            "solver": "scipy",
            "mode": mode,
            "method": method,
            "success": sol.success,
            "status": sol.status,
            "message": sol.message,
            "nfev": sol.nfev,
            "njev": sol.njev,
            "nlu": sol.nlu,
            "n_t": len(sol.t),
        }
        if extra_debug:
            debug.update(extra_debug)
        self.last_debug = debug
        if not sol.success:
            raise RuntimeError(
                f"SciPy solver failed in {mode} mode with method "
                f"{method}: {sol.message}"
            )
        return sol.y

    def integrate(
        self,
        evaluator: Any,
        times: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:

        # Get the time interval
        t0, tf = times[0], times[-1]

        # Build the right-hand side of the ODE
        rhs = evaluator.as_scipy_rhs()

        # Get the integration parameters
        kw = dict(args or {})
        use_jac = kw.pop("use_jac", False)

        # Build the Jacobian if needed
        if use_jac and getattr(evaluator, "backend", None) == "jax":
            kw["jac"] = evaluator.as_scipy_jac()
            jac_applied = True
        else:
            jac_applied = False

        # Integrate the system
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # Debug information
        method = kw.get("method", "RK45")
        return self._finalize_solution(
            sol,
            mode="nominal",
            method=method,
            extra_debug={"jac_applied": jac_applied},
        )

    def integrate_forced(
        self,
        evaluator: Any,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:

        # Get the integration parameters
        kw = dict(args or {})
        scheme = kw.pop(INPUT_INTERP_KEY, "linear")
        kw.pop("use_jac", None)

        # Debug information
        method = kw.get("method", "RK45")

        # Get the time interval
        t0, tf = times[0], times[-1]

        # Build the forced right-hand side of the ODE
        u_at_t = build_u_at_t(times, u, scheme)
        rhs = evaluator.as_scipy_rhs_forced(u_at_t)

        # Integrate the system
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # Debug information
        return self._finalize_solution(
            sol,
            mode="forced",
            method=method,
            extra_debug={"interp": scheme},
        )


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
        n = evaluator.n

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
