"""
Compiled evaluator ABCs — output, dynamics, step, and static contracts.

Concrete backends live in :mod:`numpy_evaluators` and :mod:`jax_evaluators`.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

# =============================================================================
# Public API — helpers
# =============================================================================


def outputs_from_ports(system, x, u, t, params) -> dict:
    """Evaluate every output port on ``system``."""
    return {
        port_id: port.compute(x, u, t, params)
        for port_id, port in system.outputs.items()
    }


# =============================================================================
# Public API — OutputEvaluator
# =============================================================================


class OutputEvaluator(ABC):
    """
    Base class for compiled evaluators — output tier only.

    Boundary signals are returned as a dict keyed by output port id.
    """

    n: int
    m: int
    p: int
    backend: str
    _frozen_params: dict
    _u_nominal: np.ndarray

    @abstractmethod
    def outputs(self, x, u, t=0.0) -> dict:
        """All boundary output ports, frozen params."""
        ...

    @abstractmethod
    def outputs_p(self, x, u, t, params) -> dict:
        """All boundary output ports; caller-supplied params."""
        ...


# =============================================================================
# Public API — DynamicsEvaluator
# =============================================================================


class DynamicsEvaluator(OutputEvaluator):
    """
    Base class for compiled continuous-time evaluators.

    Subclasses implement ``f``, ``f_p``, ``outputs``, and ``outputs_p``.
    Integration helpers are mixed in by the NumPy / JAX backend modules.
    """

    @abstractmethod
    def f(self, x, u, t=0.0):
        """dx = f(x, u, t) with frozen params."""
        ...

    @abstractmethod
    def f_p(self, x, u, t, params):
        """dx = f(x, u, t, p) with caller-supplied params."""
        ...

    def f_ivp(self, x, t=0.0):
        """dx = f(x, u_nom, t) with nominal input fixed at compile."""
        return self.f(x, self._u_nominal, t)

    def f_ivp_p(self, x, t, params):
        """dx = f(x, u_nom, t, p) — nominal u, runtime params."""
        return self.f_p(x, self._u_nominal, t, params)

    def f_scipy(self, x, u, t=0.0):
        return np.asarray(self.f(x, u, t))

    def f_ivp_scipy(self, x, t=0.0):
        return np.asarray(self.f_ivp(x, t))

    def as_scipy_rhs(self):
        return lambda t, x: self.f_ivp_scipy(x, t)

    def as_scipy_rhs_forced(self, u_of_t):
        return lambda t, x: self.f_scipy(x, u_of_t(t), t)

    def as_scipy_jac(self):
        raise NotImplementedError(
            "Jacobian callable is not available for this evaluator"
        )

    def jacobian_f_params(self, x, u, t, params):
        raise NotImplementedError(
            "jacobian_f_params is not available for this evaluator"
        )


# =============================================================================
# Public API — StepEvaluator
# =============================================================================


class StepEvaluator(OutputEvaluator):
    """
    Base class for compiled discrete-time evaluators.

    Subclasses implement ``step``, ``step_p``, ``outputs``, and ``outputs_p``.
    Rollout helpers live on the NumPy / JAX backend mixins.
    """

    @abstractmethod
    def step(self, x, u, k=0):
        """x_{k+1} = step(x, u, k) with frozen params."""
        ...

    @abstractmethod
    def step_p(self, x, u, k, params):
        """x_{k+1} = step(x, u, k, p) with caller-supplied params."""
        ...


# =============================================================================
# Public API — StaticEvaluator
# =============================================================================


class StaticEvaluator(OutputEvaluator):
    """Compiled evaluator for stateless IO blocks — no evolution map."""
