"""The evaluator of a static :class:`~minilink.core.system.System` (``n = 0``): its outputs."""

from __future__ import annotations

import copy

from minilink.core.backends import BACKEND_JAX
from minilink.core.compile.evaluators.base import Evaluator, outputs_from_ports
from minilink.core.compile.evaluators.compiled import body_of, compiled
from minilink.core.compile.evaluators.dynamic import check_leaf_traces
from minilink.core.system import DynamicSystem, System

###############################################################################
# Public API
###############################################################################


class StaticEvaluator(Evaluator):
    """A compiled static ``System`` (``n == 0``) on NumPy or JAX: ``outputs`` only."""

    def __init__(self, system: System, backend="numpy", verbose=False):
        if isinstance(system, DynamicSystem):
            raise TypeError("StaticEvaluator requires a static System (n=0)")
        if system.n != 0:
            raise TypeError(f"static evaluator requires n=0, got n={system.n}")
        super().__init__(
            0, system.m, system.p, backend, system.get_u_from_input_ports()
        )
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self.params_reader = lambda: system.params

        if backend == BACKEND_JAX:
            check_leaf_traces(self, system, None, 0.0, verbose)

        self._jac_setup(
            system, "static", state_fn=None, outputs_fn=body_of(self, "outputs")
        )

    @compiled
    def outputs(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, t, params)
