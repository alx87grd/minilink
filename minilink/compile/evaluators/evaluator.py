"""
DynamicsEvaluator — Abstract base class for all compiled evaluators.

Notation:  ẋ = f(x, u, t)   and   y = h(x, u, t)

Three tiers:
  f(x, u, t)           — frozen params
  f_p(x, u, t, params) — caller-supplied params (JAX pytree)
  f_ivp(x, t)          — frozen u + frozen params
"""

from abc import ABC, abstractmethod

import numpy as np


class DynamicsEvaluator(ABC):
    """
    Base class for all compiled dynamics evaluators.

    Subclasses must implement the abstract methods (f, h, outputs, and their
    ``_p`` parametric variants).  All integration, differentiation, and
    convenience methods have default implementations that delegate to
    these core callables.

    Attributes set by subclass ``__init__``:
        n : int            — state dimension
        m : int            — input dimension (total across all ports)
        p : int            — primary output dimension (dim of 'y' port)
        backend : str      — "numpy" | "jax"
        _frozen_params     — deep copy of system params at compile time
        _u_nominal         — nominal input snapshot (from port values at t=0)
    """

    # ================================================================
    # Standard tier — frozen params
    # ================================================================

    @abstractmethod
    def f(self, x, u, t=0.0):
        """ẋ = f(x, u, t) with frozen params."""
        ...

    @abstractmethod
    def h(self, x, u, t=0.0):
        """y = h(x, u, t).  Primary 'y' output port, frozen params."""
        ...

    @abstractmethod
    def outputs(self, x, u, t=0.0) -> dict:
        """All **boundary** output ports as a flat dict, frozen params.

        * **Leaf** systems: keys are output port ids (e.g. ``\"y\"``).
        * **Diagram** systems: keys are diagram output port ids (same as
          :attr:`~minilink.core.diagram.DiagramSystem.outputs`).  Empty when no
          external outputs were added (e.g. no :meth:`~minilink.core.diagram.DiagramSystem.connect_new_output_port` calls).  For every subsystem port in the internal buffer, use
          :meth:`~minilink.compile.evaluators.numpy_evaluator.NumpyDiagramEvaluator.compute_internal_signals_dict` instead.
        """
        ...

    # ================================================================
    # Parametric tier — caller supplies params dict
    # ================================================================

    @abstractmethod
    def f_p(self, x, u, t, params):
        """ẋ = f(x, u, t, p) with caller-supplied params dict."""
        ...

    @abstractmethod
    def h_p(self, x, u, t, params):
        """y = h(x, u, t, p).  Primary 'y' port, caller-supplied params."""
        ...

    @abstractmethod
    def outputs_p(self, x, u, t, params) -> dict:
        """All output ports as a flat dict; same key convention as :meth:`outputs`."""
        ...

    # ================================================================
    # IVP tier — frozen u (from nominal port values) + frozen params
    # Snapshotted at compile time → JIT-safe on all backends
    # ================================================================

    def f_ivp(self, x, t=0.0):
        """ẋ = f(x, t) with frozen u and frozen params."""
        return self.f(x, self._u_nominal, t)

    def h_ivp(self, x, t=0.0):
        """y = h(x, t) with frozen u and frozen params."""
        return self.h(x, self._u_nominal, t)

    # ================================================================
    # Scipy bridge
    # ================================================================

    def f_scipy(self, x, u, t=0.0):
        """SciPy-friendly ``f`` with numpy in/out."""
        return np.asarray(self.f(x, u, t))

    def f_ivp_scipy(self, x, t=0.0):
        """SciPy-friendly ``f_ivp`` with numpy in/out."""
        return np.asarray(self.f_ivp(x, t))

    def as_scipy_rhs(self):
        """Returns ``(t, x) -> dx`` callable for ``scipy.integrate.solve_ivp``."""
        return lambda t, x: self.f_ivp_scipy(x, t)

    def as_scipy_rhs_forced(self, u_of_t):
        """Returns ``(t, x) -> dx`` callable for forced ``solve_ivp``."""
        return lambda t, x: self.f_scipy(x, u_of_t(t), t)

    def as_scipy_jac(self):
        """Returns ``(t, x) -> df/dx`` callable for nominal ``solve_ivp``."""
        raise NotImplementedError("Jacobian callable is not available for this evaluator")

    # ================================================================
    # Integration — standard tier (frozen params)
    # ================================================================

    def rk4_step(self, x, u, t, dt):
        """Single RK4 step: x_{k+1}."""
        k1 = self.f(x, u, t)
        k2 = self.f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = self.f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = self.f(x + dt * k3, u, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def euler_step(self, x, u, t, dt):
        """Explicit Euler: ``x + dt * f(x, u, t)``."""
        return x + dt * self.f(x, u, t)

    def rk4_rollout_forced(self, x0, u_knots, t0, dt):
        """Fixed-step RK4 rollout with sampled forced inputs.

        ``u_knots`` has shape ``(N, m)`` and the result has shape ``(N, n)``.
        Step ``k`` uses ``u[k]``, ``0.5 * (u[k] + u[k + 1])``, and
        ``u[k + 1]`` at the RK4 stages.
        """
        x = np.asarray(x0).reshape(self.n)
        u = np.asarray(u_knots)
        x_samples = np.zeros((u.shape[0], self.n))
        x_samples[0] = x
        t = t0
        for k in range(u.shape[0] - 1):
            u0 = u[k]
            u1 = u[k + 1]
            umid = 0.5 * (u0 + u1)
            k1 = self.f(x, u0, t)
            k2 = self.f(x + 0.5 * dt * k1, umid, t + 0.5 * dt)
            k3 = self.f(x + 0.5 * dt * k2, umid, t + 0.5 * dt)
            k4 = self.f(x + dt * k3, u1, t + dt)
            x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    def rollout(self, x0, u_sequence, t0, dt):
        """Forward integrate N steps with frozen params.

        u_sequence: (N, m) → returns (N+1, n) including x0.
        """
        raise NotImplementedError("TODO")

    # ================================================================
    # Integration — parametric tier (caller-supplied params)
    # ================================================================

    def rk4_step_p(self, x, u, t, dt, params):
        """Single RK4 step with caller-supplied params."""
        raise NotImplementedError("TODO")

    def euler_step_p(self, x, u, t, dt, params):
        """Single Euler step with caller-supplied params."""
        raise NotImplementedError("TODO")

    def rollout_p(self, x0, u_sequence, t0, dt, params):
        """Parametric rollout. (N, m) → (N+1, n)."""
        raise NotImplementedError("TODO")

    # ================================================================
    # Integration — IVP tier (frozen u + frozen params)
    # ================================================================

    def rk4_step_ivp(self, x, t, dt):
        """Single RK4 step with frozen u and params."""
        k1 = self.f_ivp(x, t)
        k2 = self.f_ivp(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = self.f_ivp(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = self.f_ivp(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def euler_step_ivp(self, x, t, dt):
        """Explicit Euler with nominal ``u`` (same as :meth:`f_ivp`)."""
        return self.euler_step(x, self._u_nominal, t, dt)

    def rk4_rollout_ivp(self, x0, t0, dt, n_steps):
        """IVP RK4 rollout. Returns (n_steps+1, n)."""
        x_seq = [x0]
        x = x0
        t = t0
        for _ in range(n_steps):
            x = self.rk4_step_ivp(x, t, dt)
            t = t + dt
            x_seq.append(x)
        return np.asarray(x_seq)

    # ================================================================
    # Differentiation (NotImplementedError by default)
    # Could be implemented with finite differences for numpy later
    # ================================================================

    def jacobian_f_x(self, x, u, t=0.0):
        """∂f/∂x → (n, n). JAX backends override."""
        raise NotImplementedError("TODO")

    def jacobian_f_u(self, x, u, t=0.0):
        """∂f/∂u → (n, m). JAX backends override."""
        raise NotImplementedError("TODO")

    def jacobian_h_x(self, x, u, t=0.0):
        """∂h/∂x → (p, n). JAX backends override."""
        raise NotImplementedError("TODO")

    def jacobian_h_u(self, x, u, t=0.0):
        """∂h/∂u → (p, m). JAX backends override."""
        raise NotImplementedError("TODO")

    def linearize(self, x_eq, u_eq, t=0.0):
        """(A, B, C, D) linearization at equilibrium.

        Returns
        -------
        A : (n, n) — ∂f/∂x
        B : (n, m) — ∂f/∂u
        C : (p, n) — ∂h/∂x
        D : (p, m) — ∂h/∂u
        """
        raise NotImplementedError("TODO")
