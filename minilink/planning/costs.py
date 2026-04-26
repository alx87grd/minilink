"""
Cost functions for deterministic planning.

The planning cost follows the textbook optimal-control form

``J = integral g(x, u, t) dt + h(x(tf), tf)``.

Costs live in :mod:`minilink.planning`, not on
:class:`~minilink.core.framework.System`, so the same model can be reused
across many planning problems.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

import numpy as np

from minilink.core.trajectory import Trajectory


class CostFunction(ABC):
    """
    Mother class for deterministic planning cost functions.

    Subclasses define a running cost ``g(x, u, t)`` and terminal cost
    ``h(x, t)``. Both methods accept optional parameters so planning
    problems can later support parameter sweeps without putting costs on
    the system object.
    """

    @abstractmethod
    def g(
        self,
        x: np.ndarray,
        u: np.ndarray,
        t: float = 0.0,
        params: Any | None = None,
    ) -> float:
        """Return the running cost density ``g(x, u, t)``."""
        ...

    @abstractmethod
    def h(
        self,
        x: np.ndarray,
        t: float = 0.0,
        params: Any | None = None,
    ) -> float:
        """Return the terminal cost ``h(x, t)``."""
        ...

    def evaluate_trajectory(
        self,
        traj: Trajectory,
        params: Any | None = None,
    ) -> Trajectory:
        """
        Return ``traj`` with sampled running and cumulative costs.

        The added signals are ``"cost_rate"`` and ``"cost"`` with shape
        ``(1, N)``. The cumulative integral uses the trapezoidal rule and
        excludes the terminal cost; callers can add ``h(x(tf), tf)`` when
        they need the full objective scalar.
        """
        dJ = np.zeros(traj.n_samples, dtype=float)
        for i, t in enumerate(traj.t):
            dJ[i] = float(self.g(traj.x[:, i], traj.u[:, i], float(t), params=params))

        J = np.zeros(traj.n_samples, dtype=float)
        if traj.n_samples > 1:
            dt = np.diff(traj.t)
            increments = 0.5 * (dJ[:-1] + dJ[1:]) * dt
            J[1:] = np.cumsum(increments)

        return traj.with_signals(
            {
                "cost_rate": dJ.reshape(1, -1),
                "cost": J.reshape(1, -1),
            }
        )

    def terminal_cost(
        self,
        traj: Trajectory,
        params: Any | None = None,
    ) -> float:
        """Return ``h`` evaluated at the final state of ``traj``."""
        return float(self.h(traj.x[:, -1], traj.tf, params=params))

    def total_cost(
        self,
        traj: Trajectory,
        params: Any | None = None,
    ) -> float:
        """Return the trapezoidal running cost plus terminal cost."""
        evaluated = self.evaluate_trajectory(traj, params=params)
        return float(
            evaluated.signals["cost"][0, -1] + self.terminal_cost(traj, params)
        )


@dataclass(frozen=True)
class QuadraticCost(CostFunction):
    """
    Quadratic running and terminal cost.

    The running cost is
    ``(x - xbar).T @ Q @ (x - xbar) + (u - ubar).T @ R @ (u - ubar)``.
    The terminal cost is ``(x - xbar).T @ S @ (x - xbar)``.
    """

    Q: np.ndarray
    R: np.ndarray
    S: np.ndarray
    xbar: np.ndarray
    ubar: np.ndarray

    def __post_init__(self) -> None:
        Q = np.asarray(self.Q, dtype=float).copy()
        R = np.asarray(self.R, dtype=float).copy()
        S = np.asarray(self.S, dtype=float).copy()
        xbar = np.asarray(self.xbar, dtype=float).reshape(-1).copy()
        ubar = np.asarray(self.ubar, dtype=float).reshape(-1).copy()

        n = xbar.size
        m = ubar.size
        if Q.shape != (n, n):
            raise ValueError("Q must have shape (n, n)")
        if S.shape != (n, n):
            raise ValueError("S must have shape (n, n)")
        if R.shape != (m, m):
            raise ValueError("R must have shape (m, m)")

        object.__setattr__(self, "Q", Q)
        object.__setattr__(self, "R", R)
        object.__setattr__(self, "S", S)
        object.__setattr__(self, "xbar", xbar)
        object.__setattr__(self, "ubar", ubar)

    @classmethod
    def from_system(
        cls,
        sys: Any,
        *,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
        S: np.ndarray | None = None,
        xbar: np.ndarray | None = None,
        ubar: np.ndarray | None = None,
    ) -> QuadraticCost:
        """
        Create a default quadratic cost from a Minilink system.

        Parameters not provided default to identity running weights, zero
        terminal weight, zero state target, and the system's nominal input.
        """
        n = int(sys.n)
        m = int(sys.m)
        if xbar is None:
            xbar = np.zeros(n)
        if ubar is None:
            ubar = sys.get_u_from_input_ports()
        return cls(
            Q=np.eye(n) if Q is None else Q,
            R=np.eye(m) if R is None else R,
            S=np.zeros((n, n)) if S is None else S,
            xbar=xbar,
            ubar=ubar,
        )

    def g(
        self,
        x: np.ndarray,
        u: np.ndarray,
        t: float = 0.0,
        params: Any | None = None,
    ) -> float:
        """Return the quadratic running cost."""
        x_arr = np.asarray(x, dtype=float).reshape(self.xbar.shape)
        u_arr = np.asarray(u, dtype=float).reshape(self.ubar.shape)
        dx = x_arr - self.xbar
        du = u_arr - self.ubar
        return float(dx.T @ self.Q @ dx + du.T @ self.R @ du)

    def h(
        self,
        x: np.ndarray,
        t: float = 0.0,
        params: Any | None = None,
    ) -> float:
        """Return the quadratic terminal cost."""
        x_arr = np.asarray(x, dtype=float).reshape(self.xbar.shape)
        dx = x_arr - self.xbar
        return float(dx.T @ self.S @ dx)
