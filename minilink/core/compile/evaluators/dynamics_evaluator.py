"""
DynamicsEvaluator — continuous-time evolution evaluators.

Notation:  dx = f(x, u, t)   boundary outputs via :meth:`outputs`
"""

from abc import abstractmethod

import numpy as np

from minilink.core.compile.evaluators.integration import IntegrationMixin
from minilink.core.compile.evaluators.output_evaluator import OutputEvaluator


class DynamicsEvaluator(OutputEvaluator, IntegrationMixin):
    """
    Base class for compiled continuous-time evaluators.

    Subclasses implement ``f``, ``f_p``, ``outputs``, and ``outputs_p``.
    Integration and SciPy bridge methods have default implementations.
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
        return self.f(x, self._u_nominal, t)

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
