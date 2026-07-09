"""
Step evolution evaluators — discrete-time leaf compile path.

Notation:  x_{k+1} = step(x, u, k)   boundary outputs via :meth:`outputs`
"""

import copy
from abc import abstractmethod

import numpy as np

from minilink.core.compile.evaluators.output_evaluator import (
    OutputEvaluator,
    outputs_from_ports,
)
from minilink.core.compile.evaluators.step_rollout_mixin import StepRolloutMixin
from minilink.core.step_rollout import StepRollout
from minilink.core.system import StepSystem


class StepEvaluator(OutputEvaluator, StepRolloutMixin):
    """
    Base class for compiled discrete-time evaluators.

    Subclasses implement ``step``, ``step_p``, ``outputs``, and ``outputs_p``.
    Rollout helpers have default implementations on :class:`StepRolloutMixin`.
    """

    @abstractmethod
    def step(self, x, u, k=0):
        """x_{k+1} = step(x, u, k) with frozen params."""
        ...

    @abstractmethod
    def step_p(self, x, u, k, params):
        """x_{k+1} = step(x, u, k, p) with caller-supplied params."""
        ...


class NumpyStepEvaluator(StepEvaluator):
    """Compiled evaluator for a :class:`StepSystem` using NumPy."""

    def __init__(self, system: StepSystem):
        if not isinstance(system, StepSystem):
            raise TypeError("NumpyStepEvaluator requires StepSystem")
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "numpy"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = np.copy(system.get_u_from_input_ports())

    def step(self, x, u, k=0):
        return self._system.step(x, u, k, self._frozen_params)

    def step_p(self, x, u, k, params):
        return self._system.step(x, u, k, params)

    def outputs(self, x, u, k=0):
        return outputs_from_ports(self._system, x, u, k, self._frozen_params)

    def outputs_p(self, x, u, k, params):
        return outputs_from_ports(self._system, x, u, k, params)


class JaxStepEvaluator(StepEvaluator):
    """Compiled evaluator for a :class:`StepSystem` using JAX."""

    def __init__(self, system: StepSystem, verbose=False):
        if not isinstance(system, StepSystem):
            raise TypeError("JaxStepEvaluator requires StepSystem")

        import time

        import jax
        import jax.numpy as jnp

        from minilink.core.compile.evaluators.jax_utils import (
            build_jit_step_leaf,
            build_jit_step_rollout,
            check_jax_compatible,
        )

        self.jax = jax
        self.jnp = jnp
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        step_raw = system.step
        frozen_p = self._frozen_params
        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_k = 0

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 0: Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        check_jax_compatible(
            step_raw, system.name, dummy_x, dummy_u, dummy_k, frozen_p, jax
        )
        for port_id, port in system.outputs.items():
            check_jax_compatible(
                port.compute,
                f"{system.name}:{port_id}",
                dummy_x,
                dummy_u,
                dummy_k,
                frozen_p,
                jax,
            )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        jits = build_jit_step_leaf(jax, system, frozen_p)
        self._jit_step = jits["step"]
        self._jit_step_p = jits["step_p"]
        self._jit_outputs = jits["outputs"]
        self._jit_outputs_p = jits["outputs_p"]
        self._jit_rollout = build_jit_step_rollout(jax, jnp, self._jit_step)

    def step(self, x, u, k=0):
        return self._jit_step(self.jnp.asarray(x), self.jnp.asarray(u), k)

    def step_p(self, x, u, k, params):
        return self._jit_step_p(self.jnp.asarray(x), self.jnp.asarray(u), k, params)

    def outputs(self, x, u, k=0):
        return self._jit_outputs(self.jnp.asarray(x), self.jnp.asarray(u), k)

    def outputs_p(self, x, u, k, params):
        return self._jit_outputs_p(self.jnp.asarray(x), self.jnp.asarray(u), k, params)

    def rollout(self, x0, *, n_steps, u=None, record_outputs=True):
        if record_outputs and self.p:
            return super().rollout(
                x0, n_steps=n_steps, u=u, record_outputs=record_outputs
            )

        jnp = self.jnp
        x0 = jnp.asarray(x0, dtype=float).reshape(self.n)
        u_samples = self._coerce_rollout_inputs(n_steps, u)
        if n_steps == 0:
            x_path = x0.reshape(1, self.n)
        else:
            u_steps = jnp.asarray(u_samples[:, :n_steps].T)
            x_path = self._jit_rollout(x0, u_steps)
        x_samples = np.asarray(x_path, dtype=float).T
        k = np.arange(n_steps + 1, dtype=float)
        u_np = np.asarray(u_samples, dtype=float)
        return StepRollout(k=k, x=x_samples, u=u_np, signals={})
