"""The evaluator of a :class:`~minilink.core.system.DynamicSystem`: its ``f``, its outputs and the step model."""

from __future__ import annotations

import copy
import time

from minilink.core.backends import BACKEND_JAX
from minilink.core.compile.evaluators.base import (
    ContinuousEvaluator,
    check_jax_compatible,
    outputs_from_ports,
)
from minilink.core.compile.evaluators.compiled import body_of, compiled
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


class DynamicEvaluator(ContinuousEvaluator):
    """A compiled :class:`DynamicSystem` on NumPy or JAX."""

    def __init__(self, system: DynamicSystem, backend="numpy", verbose=False):
        if not isinstance(system, DynamicSystem):
            raise TypeError("DynamicEvaluator requires a DynamicSystem")
        super().__init__(
            system.n, system.m, system.p, backend, system.get_u_from_input_ports()
        )
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self.params_reader = lambda: system.params

        if backend == BACKEND_JAX:
            check_leaf_traces(self, system, system.f, 0.0, verbose)

        self._jac_setup(
            system,
            "dynamic",
            state_fn=body_of(self, "f"),
            outputs_fn=body_of(self, "outputs"),
        )

    @compiled
    def f(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return self._system.f(x, u, t, params)

    @compiled
    def outputs(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, t, params)


###############################################################################
# Internal machinery
###############################################################################


def check_leaf_traces(evaluator, system, evolution, dummy_t, verbose):
    """Refuse a system whose evolution (``f`` or ``step``) or an output port does not trace."""
    jax, jnp = evaluator.jax, evaluator.jnp
    if verbose:
        t0 = time.perf_counter()
        print(
            f"[compile] Checking JAX compatibility of '{system.name}'...",
            end="",
            flush=True,
        )

    dummy_x = jnp.zeros(system.n)
    dummy_u = jnp.zeros(system.m)
    params = system.params
    if evolution is not None:
        check_jax_compatible(
            evolution, system.name, dummy_x, dummy_u, dummy_t, params, jax
        )
    for port_id, port in system.outputs.items():
        label = f"{system.name}:{port_id}"
        check_jax_compatible(
            port.compute, label, dummy_x, dummy_u, dummy_t, params, jax
        )

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")
