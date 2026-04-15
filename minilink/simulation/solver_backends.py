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

        self.last_debug = {
            "solver": "scipy",
            "mode": "nominal",
            "jac_applied": jac_applied,
            "method": method,
            "success": sol.success,
            "status": sol.status,
            "message": sol.message,
            "nfev": sol.nfev,
            "njev": sol.njev,
            "nlu": sol.nlu,
            "n_t": len(sol.t),
        }
        return sol.y

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
        self.last_debug = {
            "solver": "scipy",
            "mode": "forced",
            "method": method,
            "interp": scheme,
            "success": sol.success,
            "status": sol.status,
            "message": sol.message,
            "nfev": sol.nfev,
            "njev": sol.njev,
            "nlu": sol.nlu,
            "n_t": len(sol.t),
        }
        return sol.y


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

        # Degenerate case
        if n_pts <= 1:
            x_traj = np.zeros((n, n_pts), dtype=float)
            if n_pts == 1:
                x_traj[:, 0] = x0
            self.last_debug = {
                "solver": "rk4",
                "mode": "nominal",
                "nfev": 0,
                "njev": 0,
                "nlu": 0,
                "n_t": n_pts,
            }
            return x_traj

        # JAX fast path
        if getattr(evaluator, "backend", None) == "jax":
            jax = getattr(evaluator, "jax", None) or getattr(evaluator, "_jax", None)
            jnp = getattr(evaluator, "jnp", None) or getattr(evaluator, "_jnp", None)
            if jax is None or jnp is None:
                import jax
                import jax.numpy as jnp

            times_j = jnp.asarray(times)
            x0_j = jnp.asarray(x0)
            t_pairs = jnp.stack((times_j[:-1], times_j[1:]), axis=1)

            def body(x, pair):
                t = pair[0]
                t_next = pair[1]
                dt = t_next - t
                k1 = evaluator.f_ivp(x, t)
                k2 = evaluator.f_ivp(x + 0.5 * dt * k1, t + 0.5 * dt)
                k3 = evaluator.f_ivp(x + 0.5 * dt * k2, t + 0.5 * dt)
                k4 = evaluator.f_ivp(x + dt * k3, t + dt)
                x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return x_next, x_next

            rollout = jax.jit(lambda x0_, pairs_: jax.lax.scan(body, x0_, pairs_))
            _, xs = rollout(x0_j, t_pairs)
            x_traj = jnp.concatenate((x0_j[None, :], xs), axis=0).T
            x_traj = np.asarray(x_traj)
        else:
            # NumPy path
            x_traj = np.zeros((n, n_pts), dtype=float)
            x_traj[:, 0] = x0
            for i in range(n_pts - 1):
                t = times[i]
                dt = times[i + 1] - times[i]
                x = x_traj[:, i]
                k1 = evaluator.f_ivp(x, t)
                k2 = evaluator.f_ivp(x + 0.5 * dt * k1, t + 0.5 * dt)
                k3 = evaluator.f_ivp(x + 0.5 * dt * k2, t + 0.5 * dt)
                k4 = evaluator.f_ivp(x + dt * k3, t + dt)
                x_traj[:, i + 1] = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

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
