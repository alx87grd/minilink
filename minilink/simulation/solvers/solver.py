"""ODE solver backends: the integrator contract a :class:`Simulator` steps through."""

from abc import ABC, abstractmethod

import numpy as np

###############################################################################
# Public API
###############################################################################


class SolverBackend(ABC):
    """
    Integrate ``ẋ = f(x, u, t)`` through an evaluator of the dynamics primitives.

    The evaluator provides ``f(x, u, t)`` for a forced input and
    ``f_ivp(x, t) = f(x, u_nom, t)`` for the nominal one.
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
        Nominal input: ``x(t) = x₀ + ∫ f_ivp(x(s), s) ds`` with ``f_ivp(x, t) = f(x, u_nom, t)``.

        Parameters
        ----------
        evaluator : evaluator
            Provides ``f_ivp(x, t)``.
        times : ndarray
            1D time grid, shape ``(n_pts,)``.
        x0 : ndarray
            Initial state, shape ``(n,)``.
        args : optional
            Backend options.

        Returns
        -------
        ndarray
            The states ``x``, shape ``(n, n_pts)``.
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
        Forced input: ``x(t) = x₀ + ∫ f(x(s), u(s), s) ds``.

        Parameters
        ----------
        evaluator : evaluator
            Provides ``f(x, u, t)``.
        times : ndarray
            1D time grid, shape ``(n_pts,)``.
        u : ndarray
            Sampled input, shape ``(m, n_pts)``, column ``k`` at ``times[k]``.
        x0 : ndarray
            Initial state, shape ``(n,)``.
        args : optional
            Backend options.

        Returns
        -------
        ndarray
            The states ``x``, shape ``(n, n_pts)``.
        """
        ...
