"""Static (n=0) compiled evaluators — output tier only."""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.compile.evaluators.output_evaluator import (
    OutputEvaluator,
    outputs_from_ports,
)
from minilink.core.system import DynamicSystem, System


class StaticEvaluator(OutputEvaluator):
    """Compiled evaluator for stateless IO blocks — no evolution map."""


class NumpyStaticEvaluator(StaticEvaluator):
    """NumPy evaluator for a static ``System`` leaf (``n == 0``)."""

    def __init__(self, system: System):
        if isinstance(system, DynamicSystem):
            raise TypeError("NumpyStaticEvaluator requires a static System (n=0)")
        if system.n != 0:
            raise TypeError(f"static evaluator requires n=0, got n={system.n}")
        self.n = 0
        self.m = system.m
        self.p = system.p
        self.backend = "numpy"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = np.copy(system.get_u_from_input_ports())

    def outputs(self, x, u, t=0.0):
        return outputs_from_ports(self._system, x, u, t, self._frozen_params)

    def outputs_p(self, x, u, t, params):
        return outputs_from_ports(self._system, x, u, t, params)


class JaxStaticEvaluator(StaticEvaluator):
    """JAX evaluator for a static ``System`` leaf."""

    def __init__(self, system: System, verbose=False):
        if isinstance(system, DynamicSystem):
            raise TypeError("JaxStaticEvaluator requires a static System (n=0)")
        if system.n != 0:
            raise TypeError(f"static evaluator requires n=0, got n={system.n}")

        import jax
        import jax.numpy as jnp

        from minilink.core.compile.evaluators.jax_utils import (
            build_jit_static_outputs,
            check_jax_compatible,
        )

        self.jax = jax
        self.jnp = jnp
        self.n = 0
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        frozen_p = self._frozen_params
        dummy_x = jnp.zeros(0)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 0: Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        for port_id, port in system.outputs.items():
            check_jax_compatible(
                port.compute,
                f"{system.name}:{port_id}",
                dummy_x,
                dummy_u,
                dummy_t,
                frozen_p,
                jax,
            )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")
            t0 = time.perf_counter()
            print(
                f"[compile] Step 1: JIT-compiling static outputs on {jax.default_backend()}...",
                end="",
                flush=True,
            )

        self._jit_outputs, self._jit_outputs_p = build_jit_static_outputs(
            jax, system, frozen_p
        )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")
            t0 = time.perf_counter()
            print("[compile] Step 2: Warm-starting JIT cache...", end="", flush=True)

        try:
            self._jit_outputs(dummy_x, dummy_u, dummy_t)
            self._jit_outputs_p(dummy_x, dummy_u, dummy_t, frozen_p)
        except Exception as e:
            raise RuntimeError(
                f"\n\nBlock '{system.name}' failed during JAX warm-start.\n"
                f"Port compute functions are not fully JAX-traceable.\n"
                f"Original JAX error: {e}"
            ) from e

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    def outputs(self, x, u, t=0.0):
        return self._jit_outputs(x, u, t)

    def outputs_p(self, x, u, t, params):
        return self._jit_outputs_p(x, u, t, params)
