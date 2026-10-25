"""ODE solver backends: the integrator contract a :class:`Simulator` steps through."""

from abc import ABC, abstractmethod

import numpy as np

###############################################################################
# Public API
###############################################################################


class SolverBackend(ABC):
    """
    Integrate ``ẋ = f(x, u, t)`` through an evaluator of the dynamics primitives.

    The evaluator provides ``f(x, u, t)`` and its ``nominal_input``.
    """

    #: Whether a forced input given as a function of time is read at the solver's
    #: stage times (:meth:`integrate_input`) rather than sampled on the grid.
    reads_input_at_stages = False

    @abstractmethod
    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
    ) -> np.ndarray:
        """
        Nominal input: ``x(t) = x₀ + ∫ f(x(s), u_nom, s) ds``.

        Parameters
        ----------
        evaluator : evaluator
            Provides ``f(x, u, t)`` and ``nominal_input``.
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


###############################################################################
# Internal machinery
###############################################################################


def require_uniform_times(times: np.ndarray, solver: str) -> float:
    """Refuse knots that are not evenly spaced, to a relative tolerance; return the common step.

    ``solver`` names the fixed-step solver in the refusal.
    """
    n_pts = times.shape[0]
    if n_pts < 2:
        return 0.0
    dt = float(times[1] - times[0])
    if not np.allclose(np.diff(times), dt, atol=0.0):
        raise ValueError(
            f"{solver} requires a uniform time grid; use solver='euler' "
            "for variable dt between knots"
        )
    return dt


def fixed_step_debug(solver: str, mode: str, n_pts: int, f_per_step: int) -> dict:
    """The record of a fixed-step run: ``f_per_step`` evaluations of ``f`` per step, no Jacobian."""
    return {
        "solver": solver,
        "mode": mode,
        "nfev": f_per_step * max(n_pts - 1, 0),
        "njev": 0,
        "nlu": 0,
        "n_t": n_pts,
    }
