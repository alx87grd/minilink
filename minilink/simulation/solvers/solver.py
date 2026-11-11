"""ODE solver backends: the integrator contract a :class:`Simulator` steps through."""

from abc import ABC, abstractmethod

import numpy as np

###############################################################################
# Public API
###############################################################################


class SolverBackend(ABC):
    """
    Integrate ``ẋ = f(x, u, t)`` through an evaluator of the dynamics primitives.

    The evaluator provides ``f(x, u, t)`` and its ``nominal_input``. A backend
    whose ``records_from_pass`` is true takes ``record=`` on :meth:`integrate`
    and :meth:`integrate_forced` and returns ``(x, records)``, a diagram's named
    signals read off each step's own pass; the others record nothing (the
    simulator reads the signals at the saved samples after the run).
    """

    records_from_pass = False

    @abstractmethod
    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
        params=None,
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
        params=None,
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

    def integrate_input(self, evaluator, times, u_of_t, x0, args=None, params=None):
        """
        A function of time: ``x(t) = x₀ + ∫ f(x(s), u(s), s) ds``, ``u`` read where the method evaluates ``f``.

        A one-stage method evaluates ``f`` at the grid times only, so the input
        sampled there and held gives the same run: the default; a method with
        stages (RK4) or its own points (an adaptive solver) reads ``u(t)`` at
        each of them.
        """
        u = np.stack(
            [np.asarray(u_of_t(float(t)), dtype=float).reshape(-1) for t in times],
            axis=1,
        )

        return self.integrate_forced(evaluator, times, u, x0, args=args, params=params)


###############################################################################
# Internal machinery
###############################################################################


def fixed_steps_of(times: np.ndarray):
    """``(dt, dt_last)`` of a fixed-step grid, every step ``dt`` but a last one ``0 < dt_last ≤ dt``; ``None`` for other knots.

    The steps are compared to a relative tolerance; a last step equal to
    ``dt`` within it is ``dt`` itself, so a uniform grid has ``dt_last = dt``.
    A shorter last step lands on ``tf`` when ``dt`` does not divide the run
    (:func:`~minilink.simulation.time_grid.build_time_grid`).
    """
    n_pts = times.shape[0]
    if n_pts < 2:
        return 0.0, 0.0
    steps = np.diff(times)
    dt = float(steps[0])
    dt_last = float(steps[-1])
    if not np.allclose(steps[:-1], dt, atol=0.0):
        return None
    if np.isclose(dt_last, dt, atol=0.0):
        return dt, dt
    if not 0.0 < dt_last < dt:
        return None
    return dt, dt_last


def require_fixed_steps(times: np.ndarray, solver: str):
    """``(dt, dt_last)`` of a fixed-step grid (:func:`fixed_steps_of`); refuse other knots.

    ``solver`` names the fixed-step solver in the refusal.
    """
    steps = fixed_steps_of(times)
    if steps is None:
        raise ValueError(
            f"solver={solver!r} steps at a fixed dt and needs a uniform time grid; "
            "pass one (dt= or n_samples=; a last step shorter than dt lands on tf), "
            "or solver='scipy' for an adaptive run on these knots"
        )
    return steps


def rollout_on_grid(evaluator, x0, u_seq, t0, steps, n_steps, params, record, **step):
    """The compiled rollout over a fixed-step grid: ``n_steps`` steps of ``dt``, the last one of ``dt_last``.

    ``u_seq`` holds one row per step (``None``: the nominal input). On a
    uniform grid this is one rollout; a shorter last step is a second
    rollout of one step from where the first ends. Returns the states
    ``(n_steps + 1, n)``, with the records ``(n_steps, r)`` when ``record``.
    """
    dt, dt_last = steps

    # A uniform grid: one rollout of n steps of dt
    if dt_last == dt:
        if u_seq is None:
            return evaluator.rollout(
                x0, None, t0, dt, params, n_steps=n_steps, record=record, **step
            )
        return evaluator.rollout(x0, u_seq, t0, dt, params, record=record, **step)

    # n − 1 steps of dt, then the last of dt_last from t_{n−1} = t₀ + (n − 1) dt
    head_u = None if u_seq is None else u_seq[:-1]
    last_u = None if u_seq is None else u_seq[-1:]
    head = evaluator.rollout(
        x0, head_u, t0, dt, params, n_steps=n_steps - 1, record=record, **step
    )
    x_head, records_head = head if record else (head, None)
    x_head = np.asarray(x_head)
    t_last = t0 + (n_steps - 1) * dt
    last = evaluator.rollout(
        x_head[-1], last_u, t_last, dt_last, params, n_steps=1, record=record, **step
    )
    x_last, records_last = last if record else (last, None)

    # The two runs joined at x_{n−1}
    x_seq = np.concatenate([x_head, np.asarray(x_last)[1:]])
    if not record:
        return x_seq
    records = np.concatenate([np.asarray(records_head), np.asarray(records_last)])
    return x_seq, records


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
