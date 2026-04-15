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
        pass

    def integrate(
        self,
        evaluator: Any,
        times: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:
        kw = dict(args or {})
        t0, tf = times[0], times[-1]
        rhs = evaluator.as_scipy_rhs()
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)
        return sol.y

    def integrate_forced(
        self,
        evaluator: Any,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:
        kw = dict(args or {})
        scheme = kw.pop(INPUT_INTERP_KEY, "linear")
        t0, tf = times[0], times[-1]
        u_at_t = build_u_at_t(times, u, scheme)
        rhs = evaluator.as_scipy_rhs_forced(u_at_t)
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)
        return sol.y


class EulerSolverBackend(SolverBackend):
    """Explicit Euler on the ``times`` grid (variable ``dt`` between knots)."""

    def integrate(
        self,
        evaluator: Any,
        times: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:
        n_pts = times.shape[0]
        n = evaluator.n
        x_traj = np.zeros((n, n_pts), dtype=float)
        x_traj[:, 0] = x0
        for i in range(n_pts - 1):
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step_ivp(x_traj[:, i], times[i], dt)
        return x_traj

    def integrate_forced(
        self,
        evaluator: Any,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args: dict[str, Any] | None = None,
    ) -> np.ndarray:
        n_pts = times.shape[0]
        n = evaluator.n
        x_traj = np.zeros((n, n_pts), dtype=float)
        x_traj[:, 0] = x0
        for i in range(n_pts - 1):
            dt = times[i + 1] - times[i]
            x_traj[:, i + 1] = evaluator.euler_step(x_traj[:, i], u[:, i], times[i], dt)
        return x_traj
