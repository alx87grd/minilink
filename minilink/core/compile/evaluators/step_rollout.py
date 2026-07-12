"""
Step rollouts and diagram input gathering for compiled evaluators.

Shared by NumPy and JAX step evaluators and by :class:`~minilink.simulation.computer.Computer`.
"""

from __future__ import annotations

import numpy as np

from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
)
from minilink.core.step_rollout import StepRollout

# =============================================================================
# Public API — gather_u
# =============================================================================


def gather_u(
    gather_sources: tuple[tuple[int, object, int], ...],
    u_dim: int,
    signals: np.ndarray,
    u: np.ndarray,
) -> np.ndarray:
    """
    Assemble a subsystem local input vector from a compiled gather plan.

    Each entry in ``gather_sources`` is ``(source_type, source_value, dim)`` as
    stored on :class:`~minilink.core.compile.execution_plan.PortOperation` and
    :class:`~minilink.core.compile.execution_plan.StateOperation`. Source types
    are :data:`~minilink.core.compile.execution_plan.INTERNAL_SIGNAL`,
    :data:`~minilink.core.compile.execution_plan.NOMINAL`, and
    :data:`~minilink.core.compile.execution_plan.EXTERNAL_INPUT`. For
    ``INTERNAL_SIGNAL`` and ``EXTERNAL_INPUT``, ``source_value`` is a ``slice``
    into ``signals`` or the diagram boundary input ``u`` respectively.

    Parameters
    ----------
    gather_sources
        Flattened wiring plan for one subsystem input port bundle.
    u_dim
        Expected local input dimension (sum of ``dim`` over sources).
    signals
        Internal signal buffer (1-D) or port-signal workspace used for
        ``INTERNAL_SIGNAL`` lookups.
    u
        Diagram boundary input vector.

    Returns
    -------
    ndarray
        Local input ``u_local`` with shape ``(u_dim,)``.
    """
    if u_dim == 0:
        return np.array([])

    local_u = np.empty(u_dim)
    idx = 0
    for src_type, src_val, dim in gather_sources:
        if src_type == INTERNAL_SIGNAL:
            local_u[idx : idx + dim] = signals[src_val]
        elif src_type == NOMINAL:
            local_u[idx : idx + dim] = src_val
        elif src_type == EXTERNAL_INPUT:
            local_u[idx : idx + dim] = u[src_val]
        else:
            raise RuntimeError(f"Unknown source_type={src_type}")
        idx += dim
    return local_u


# =============================================================================
# Public API — StepRolloutMixin
# =============================================================================


class StepRolloutMixin:
    """
    Discrete-time rollouts for compiled :class:`~minilink.core.system.StepSystem` evaluators.

    Subclasses must implement ``step``, ``step_p``, ``outputs``, and ``outputs_p``
    and expose ``n``, ``m``, ``p``, and ``_u_nominal``.
    """

    n: int
    m: int
    p: int
    _u_nominal: np.ndarray

    def _coerce_rollout_inputs(self, n_steps, u):
        if n_steps < 0:
            raise ValueError("n_steps must be nonnegative")

        u_nominal = np.asarray(self._u_nominal, dtype=float).reshape(self.m)

        if u is None:
            u_steps = (
                np.tile(u_nominal, (n_steps, 1)) if n_steps else np.zeros((0, self.m))
            )
        elif callable(u):
            u_steps = np.asarray(
                [np.asarray(u(k), dtype=float).reshape(self.m) for k in range(n_steps)]
            )
        else:
            u_arr = np.asarray(u, dtype=float)
            if u_arr.ndim == 1:
                if u_arr.shape != (self.m,):
                    raise ValueError(f"Constant u must have shape ({self.m},)")
                u_steps = np.tile(u_arr, (n_steps, 1))
            elif u_arr.ndim == 2:
                if u_arr.shape != (n_steps, self.m):
                    raise ValueError(
                        f"u sequence must have shape ({n_steps}, {self.m})"
                    )
                u_steps = u_arr
            else:
                raise ValueError(
                    "u must be None, a constant vector, a sequence, or callable(k)"
                )

        n_samples = n_steps + 1
        u_samples = np.zeros((self.m, n_samples), dtype=float)
        if n_samples:
            if n_steps:
                u_samples[:, :n_steps] = u_steps.T
                u_samples[:, -1] = u_samples[:, n_steps - 1]
            elif self.m:
                u_samples[:, 0] = u_nominal

        return u_samples

    def rollout(self, x0, *, n_steps, u=None) -> StepRollout:
        """
        Roll out ``x_{k+1} = step(x_k, u_k, k)`` for ``n_steps`` transitions.

        Returns a state-only :class:`~minilink.core.step_rollout.StepRollout` with
        ``k.shape == (n_steps + 1,)``, ``x.shape == (n, n_steps + 1)``, and
        ``u.shape == (m, n_steps + 1)``. Signal histories are filled by
        :class:`~minilink.simulation.computer.Computer`,
        :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`, or
        :func:`~minilink.simulation.step_recording.record_boundary_outputs`
        when a synchronous reference rollout needs boundary samples.
        """
        x0 = np.asarray(x0, dtype=float).reshape(self.n)
        n_samples = n_steps + 1
        k = np.arange(n_samples, dtype=float)
        x_samples = np.zeros((self.n, n_samples), dtype=float)
        x_samples[:, 0] = x0

        u_samples = self._coerce_rollout_inputs(n_steps, u)

        for step_k in range(n_steps):
            u_k = u_samples[:, step_k]
            x_samples[:, step_k + 1] = self.step(x_samples[:, step_k], u_k, step_k)

        return StepRollout(k=k, x=x_samples, u=u_samples)

    def rollout_p(self, x0, *, n_steps, u=None, params) -> StepRollout:
        """Parametric twin of :meth:`rollout`."""
        x0 = np.asarray(x0, dtype=float).reshape(self.n)
        n_samples = n_steps + 1
        k = np.arange(n_samples, dtype=float)
        x_samples = np.zeros((self.n, n_samples), dtype=float)
        x_samples[:, 0] = x0

        u_samples = self._coerce_rollout_inputs(n_steps, u)

        for step_k in range(n_steps):
            u_k = u_samples[:, step_k]
            x_samples[:, step_k + 1] = self.step_p(
                x_samples[:, step_k], u_k, step_k, params
            )

        return StepRollout(k=k, x=x_samples, u=u_samples)
