"""Abstract ODE integrator (solver) backend for :class:`Simulator`."""

from abc import ABC, abstractmethod

import numpy as np


class SolverBackend(ABC):
    """
    Solve the ODE using an evaluator that implements the dynamics primitives: f(x, u, t), f_ivp(x, t)
    """

    @abstractmethod
    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
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
        evaluator,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args=None,
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
