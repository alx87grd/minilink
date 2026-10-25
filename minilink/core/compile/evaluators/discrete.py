"""The evaluators of discrete-time systems, ``x_{k+1} = step(x_k, u_k, k; p)``: a leaf and a step diagram."""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.backends import BACKEND_JAX
from minilink.core.compile.evaluators.base import (
    Evaluator,
    added,
    assigned,
    check_jax_compatible,
    gather_u,
    op_params_of,
    outputs_from_ports,
)
from minilink.core.compile.evaluators.compiled import body_of, compiled
from minilink.core.compile.evaluators.dynamic import check_leaf_traces
from minilink.core.compile.evaluators.stepping import iterate
from minilink.core.compile.plan import StepExecutionPlan
from minilink.core.diagram import check_diagram_params
from minilink.core.discrete.step_rollout import StepRollout
from minilink.core.discrete.step_system import StepSystem

###############################################################################
# Public API
###############################################################################


class DiscreteEvaluator(Evaluator):
    """A compiled discrete-time system: its own ``step``, its ``outputs`` and the ``rollout`` of the steps."""

    def step(self, x, u, k=0, params=None):
        """``x_{k+1} = step(x, u, k; p)``; ``params=None`` reads the compiled parameters."""
        raise NotImplementedError

    def rollout(self, x0, *, n_steps, u=None, params=None) -> StepRollout:
        """
        Roll out ``x_{k+1} = step(x_k, u_k, k)`` for ``n_steps`` transitions.

        Returns a state-only :class:`~minilink.core.discrete.step_rollout.StepRollout` with
        ``k.shape == (n_steps + 1,)``, ``x.shape == (n, n_steps + 1)``, and
        ``u.shape == (m, n_steps + 1)``; ``params=None`` reads the compiled
        parameters. Signal histories are owned by
        :class:`~minilink.simulation.discrete.computer.Computer` and
        :class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimulator`.
        """
        xp = self.xp
        x0 = xp.asarray(x0, dtype=float).reshape(self.n)
        u_samples = self.rollout_inputs_of(n_steps, u)
        u_steps = xp.asarray(u_samples[:, :n_steps].T)

        x_path = self.scan_steps(x0, u_steps, params) if n_steps else x0[None, :]

        x_samples = np.asarray(x_path, dtype=float).T
        k = np.arange(n_steps + 1, dtype=float)

        return StepRollout(k=k, x=x_samples, u=u_samples)

    @compiled
    def scan_steps(self, x0, u_steps, params=None):
        """``[x_0, …, x_N]``: the step run over the rows of ``u_steps`` (the rollout's core)."""
        xp = self.xp
        step = self.step
        n = self.n

        # x_{k+1} = step(x_k, u_k, k), a bare number for n = 1 read as its one entry
        def transition(x, k, u_k):
            x_next = xp.reshape(step(x, u_k, k, params), (n,))
            return x_next

        x_path = iterate(
            self, transition, x0, (self.indices(u_steps.shape[0]), u_steps)
        )

        return x_path

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def rollout_inputs_of(self, n_steps, u):
        """The rollout's inputs ``(m, n_steps + 1)``: the nominal one, a constant, a sequence or ``u(k)``."""
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


class StepEvaluator(DiscreteEvaluator):
    """A compiled :class:`StepSystem` on NumPy or JAX."""

    def __init__(self, system: StepSystem, backend="numpy", verbose=False):
        if not isinstance(system, StepSystem):
            raise TypeError("StepEvaluator requires a StepSystem")
        super().__init__(
            system.n, system.m, system.p, backend, system.get_u_from_input_ports()
        )
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)

        if backend == BACKEND_JAX:
            check_leaf_traces(self, system, system.step, 0, verbose)

        self._jac_setup(
            system,
            "step",
            state_fn=body_of(self, "step"),
            outputs_fn=body_of(self, "outputs"),
        )

    @compiled
    def step(self, x, u, k=0, params=None):
        xp = self.xp
        params = self._frozen_params if params is None else params
        return self._system.step(xp.asarray(x), xp.asarray(u), k, params)

    @compiled
    def outputs(self, x, u, k=0, params=None):
        xp = self.xp
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, xp.asarray(x), xp.asarray(u), k, params)


class StepDiagramEvaluator(DiscreteEvaluator):
    """A compiled step diagram on NumPy or JAX: its frozen plan swept in order, stateless."""

    def __init__(
        self, plan: StepExecutionPlan, diagram, backend="numpy", verbose=False
    ):
        p = sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
        super().__init__(
            plan.state_dim, diagram.m, p, backend, diagram.get_u_from_input_ports()
        )
        self.plan = plan
        self.dtype = self.xp.float64
        self._frozen_params = None
        self._subsystem_ids = tuple(diagram.subsystems)

        if backend == BACKEND_JAX:
            check_plan_traces(self, plan, verbose)

        self._jac_setup(diagram, "step", state_fn=None, outputs_fn=None, plan=plan)

    @compiled
    def step(self, x, u, k=0, params=None):
        """``x_{k+1}`` of every block, stacked; ``params=None`` reads each block's compiled parameters."""
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        return self.next_from_signals(x, u, k, params, signals)

    @compiled
    def outputs(self, x, u, k=0, params=None):
        self.require_params(params)
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def compute_internal_signals_dict(self, x, u, k=0, params=None) -> dict:
        """Every block output at ``(x, u, k)``, keyed ``"block:port"``."""
        x, u = self.float_arrays(x, u)
        signals = self.port_signals(x, u, k, params)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_params(self, params):
        """Refuse a params tree naming a block the diagram does not hold."""
        if params is not None:
            check_diagram_params(params, self._subsystem_ids)

    def float_arrays(self, x, u):
        """``x`` and ``u`` as float arrays of the diagram's dtype."""
        xp = self.xp
        return xp.asarray(x, dtype=self.dtype), xp.asarray(u, dtype=self.dtype)

    def port_signals(self, x, u, k, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        xp = self.xp
        dtype = self.dtype
        signals = xp.zeros(self.plan.signal_dim, dtype=dtype)
        x_arr = xp.asarray(x, dtype=dtype).reshape(self.plan.state_dim)
        for op in self.plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            op_params = op_params_of(op, params)
            y_out = op.compute_func(local_x, local_u, k, op_params)
            signals = assigned(xp, signals, op.out_slice, y_out)
            if inject is not None and op.out_slice == inject[0]:
                signals = added(xp, signals, op.out_slice, inject[1])
        return signals

    def next_from_signals(self, x, u, k, params, signals):
        """Every block's ``x_{k+1}`` from the filled signal buffer."""
        xp = self.xp
        dtype = self.dtype
        x_new = xp.array(x, dtype=dtype).reshape(self.plan.state_dim)
        for op in self.plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, signals, u, xp, dtype)
            op_params = op_params_of(op, params)
            x_new = assigned(
                xp,
                x_new,
                op.local_x_slice,
                op.step_func(local_x, local_u, k, op_params),
            )
        return x_new

    def _jac_validate_params(self, params):
        check_diagram_params(params, self._subsystem_ids)

    def _jac_probe(self, target, wire=None):
        kind, key = target
        signals_of = self.port_signals

        if kind == "state":
            next_of = self.next_from_signals

            def probe(x, u, k, params, delta):
                signals = signals_of(x, u, k, params, inject=(wire, delta))
                return next_of(x, u, k, params, signals)

            return probe

        sl = self.plan.external_output_slices[key] if kind == "port" else key

        def probe(x, u, k, params, delta):
            return signals_of(x, u, k, params, inject=(wire, delta))[sl]

        return probe


###############################################################################
# Internal machinery
###############################################################################


def check_plan_traces(evaluator, plan, verbose):
    """Refuse a step diagram holding a block whose ``step`` or an output port does not trace."""
    jax, jnp = evaluator.jax, evaluator.jnp
    if verbose:
        t0 = time.perf_counter()
        print(
            "[compile] Checking JAX compatibility of step diagram...",
            end="",
            flush=True,
        )

    dummy_x = jnp.zeros(plan.state_dim)
    for op in plan.port_ops:
        check_jax_compatible(
            op.compute_func,
            op.label,
            dummy_x[op.local_x_slice],
            jnp.zeros(op.u_dim),
            0,
            op.bound_params,
            jax,
        )
    for op in plan.step_ops:
        check_jax_compatible(
            op.step_func,
            op.label,
            dummy_x[op.local_x_slice],
            jnp.zeros(op.u_dim),
            0,
            op.bound_params,
            jax,
        )

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")
