"""
JAX compiled evaluators — dynamics, diagrams, step, and static.

Every method is one body taking ``params=None``, compiled once per evaluator
(:func:`~minilink.core.compile.evaluators.compiled.compiled`): fast from
Python, and composable inside an outer ``jit``, ``grad``, ``vmap`` or ``scan``.
"""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.backends import ensure_jax_x64
from minilink.core.compile.evaluators.compiled import compiled
from minilink.core.compile.evaluators.evaluators import (
    DynamicsEvaluator,
    StaticEvaluator,
    StepEvaluator,
    outputs_from_ports,
)
from minilink.core.compile.evaluators.jacobian import JaxJacobianMixin
from minilink.core.compile.evaluators.step_rollout import StepRolloutMixin
from minilink.core.compile.evaluators.stepping import (
    held_inputs,
    require_step_knobs,
    step_body,
)
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)
from minilink.core.compile.step_execution_plan import StepExecutionPlan
from minilink.core.diagram import check_diagram_params
from minilink.core.step_rollout import StepRollout
from minilink.core.system import DynamicSystem, StepSystem, System

###############################################################################
# Public API
###############################################################################


class JaxIntegrationMixin:
    """The step model on :meth:`f`: ``step``, ``rollout`` and ``rollout_batch``."""

    @compiled(static=("solver", "substeps"))
    def step(self, x, u, t, params=None, *, dt, solver="rk4", substeps=1):
        """``x_{k+1}``: the plant one sample period ``dt`` later, ``u`` held over it.

        ``params=None`` reads the compiled parameters; ``solver`` (``"rk4"``,
        ``"euler"``) and ``substeps`` are static.
        """
        require_step_knobs(solver, substeps)
        f = self.rhs_of(params)

        x_next = step_body(f, x, u, u, t, dt, solver, substeps)

        return x_next

    @compiled(static=("solver", "substeps", "hold", "n_steps"))
    def rollout(
        self,
        x0,
        u_seq,
        t0,
        dt,
        params=None,
        *,
        solver="rk4",
        substeps=1,
        hold="zoh",
        n_steps=None,
    ):
        """``[x_0, …, x_N]``, shape ``(N + 1, n)``: the step model scanned over ``u_seq``.

        ``hold="zoh"`` holds each row of ``u_seq`` (shape ``(N, m)``) over its
        period; ``hold="foh"`` reads ``u_seq`` as ``N + 1`` knots, linear between
        them. ``u_seq=None`` holds the nominal input for ``n_steps`` periods.
        """
        jax = self.jax
        jnp = self.jnp
        require_step_knobs(solver, substeps, hold)

        f = self.rhs_of(params)
        x0 = jnp.asarray(x0, dtype=float)
        u_seq = self.input_sequence_of(u_seq, n_steps, hold)
        u_start, u_end = held_inputs(u_seq, hold)
        ks = jnp.arange(u_start.shape[0])

        # x_{k+1} = Φ(x_k, u_k, t_k), t_k = t₀ + k dt
        def period(x, inputs):
            k, ua, ub = inputs
            x_next = step_body(f, x, ua, ub, t0 + k * dt, dt, solver, substeps)
            return x_next, x_next

        _, xs = jax.lax.scan(period, x0, (ks, u_start, u_end))
        x_samples = jnp.concatenate((x0[None, :], xs), axis=0)

        return x_samples

    def rollout_batch(
        self, x0s, u_sequences=None, *, t0=0.0, dt, n_steps=None, params=None
    ):
        """Roll out a family of RK4 / ZOH trajectories in one ``jax.vmap`` call.

        Parameters
        ----------
        x0s : array, shape ``(B, n)``
            One initial state per family member.
        u_sequences : None, ``(N, m)``, or ``(B, N, m)``
            Held inputs per step: the compiled nominal input (``n_steps``
            required), one sequence shared by the batch, or one per member.
        t0, dt : float
            Start time and step, shared by the batch.
        params : None or pytree
            ``None`` uses the compiled values. Otherwise a params pytree whose
            leaves are either shared by the batch or carry a leading axis of
            size ``B`` — a leaf with one more dimension than the compiled value
            is a family (``{"l": jnp.linspace(0.5, 2.0, B)}`` sweeps a length).

        Returns
        -------
        jax.Array, shape ``(B, N + 1, n)``
            Sampled states, ``x0`` first, for every member of the family.
        """
        jax, jnp = self.jax, self.jnp
        x0s = jnp.asarray(x0s, dtype=float)
        if x0s.ndim != 2 or x0s.shape[1] != self.n:
            raise ValueError(f"x0s must have shape (B, {self.n}); got {x0s.shape}")
        batch = int(x0s.shape[0])

        if u_sequences is None:
            if n_steps is None:
                raise ValueError("n_steps is required when u_sequences is None")
            u_seq = jnp.broadcast_to(self._u_nominal, (int(n_steps), self.m))
            u_axis = None
        else:
            u_seq = jnp.asarray(u_sequences, dtype=float)
            if u_seq.ndim == 2:
                u_axis = None
            elif u_seq.ndim == 3 and u_seq.shape[0] == batch:
                u_axis = 0
            else:
                raise ValueError(
                    f"u_sequences must have shape (N, {self.m}) or "
                    f"({batch}, N, {self.m}); got {u_seq.shape}"
                )
            if n_steps is not None and u_seq.shape[-2] != int(n_steps):
                raise ValueError("n_steps disagrees with u_sequences")
        t0 = jnp.asarray(t0, dtype=float)
        dt = jnp.asarray(dt, dtype=float)

        # One jitted vmap per (input layout, family layout): repeated sweeps
        # with the same shapes reuse the compiled rollout instead of re-tracing.
        cache = self.compiled_forms.setdefault("rollout_batch", {})
        if params is None:
            key = (u_axis, None)
            fn = cache.get(key)
            if fn is None:
                fn = jax.jit(
                    jax.vmap(
                        lambda x0, u, t0, dt: self.rollout(x0, u, t0, dt),
                        in_axes=(0, u_axis, None, None),
                    )
                )
                cache[key] = fn
            return fn(x0s, u_seq, t0, dt)
        axes = self._params_batch_axes(params, batch)
        leaves, treedef = jax.tree_util.tree_flatten(axes, is_leaf=lambda a: a is None)
        key = (u_axis, str(treedef), tuple(leaves))
        fn = cache.get(key)
        if fn is None:
            fn = jax.jit(
                jax.vmap(
                    lambda x0, u, t0, dt, p: self.rollout(x0, u, t0, dt, p),
                    in_axes=(0, u_axis, None, None, axes),
                )
            )
            cache[key] = fn
        return fn(x0s, u_seq, t0, dt, params)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def rhs_of(self, params):
        """``f(x, u, t)`` on the compiled parameters, or on ``params``."""
        f = self.f

        return lambda x, u, t: f(x, u, t, params)

    def input_sequence_of(self, u_seq, n_steps, hold):
        """The input sequence as an array; the nominal input repeated when ``u_seq`` is None."""
        jnp = self.jnp
        if u_seq is not None:
            return jnp.asarray(u_seq)

        rows = int(n_steps) + (1 if hold == "foh" else 0)

        return jnp.tile(jnp.asarray(self._u_nominal), (rows, 1))

    def _params_batch_axes(self, params, batch):
        """``in_axes`` for ``params``: 0 on family leaves, ``None`` on shared ones.

        A leaf is a family when it has one more dimension than the compiled
        value at the same path; leaves without a compiled twin (partial nested
        params) count as a family when their leading axis has size ``batch``.
        """
        from jax.tree_util import keystr, tree_leaves_with_path, tree_map_with_path

        reference = {}
        if self._frozen_params is not None:
            for path, leaf in tree_leaves_with_path(self._frozen_params):
                reference[keystr(path)] = np.ndim(leaf)

        def axis(path, leaf):
            ndim = np.ndim(leaf)
            ref_ndim = reference.get(keystr(path))
            if ref_ndim is not None:
                return 0 if ndim == ref_ndim + 1 else None
            return 0 if (ndim >= 1 and np.shape(leaf)[0] == batch) else None

        return tree_map_with_path(axis, params)


class JaxDynamicEvaluator(JaxIntegrationMixin, JaxJacobianMixin, DynamicsEvaluator):
    """Compiled evaluator for a :class:`DynamicSystem` using JAX."""

    def __init__(self, system: DynamicSystem, verbose=False):
        if not isinstance(system, DynamicSystem):
            raise TypeError("JaxDynamicEvaluator requires DynamicSystem")

        import jax
        import jax.numpy as jnp

        ensure_jax_x64()

        self.jax = jax
        self.jnp = jnp
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self.compiled_forms = {}
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        check_leaf_compatible(system, system.f, jnp.zeros(self.n), self.m, 0.0, jax)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        self._jac_setup(system, "dynamic", state_fn=self.f, outputs_fn=self.outputs)

    @compiled
    def f(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return self._system.f(x, u, t, params)

    @compiled
    def outputs(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, t, params)


class JaxDiagramEvaluator(JaxIntegrationMixin, JaxJacobianMixin, DynamicsEvaluator):
    """JAX-compatible evaluator for a compiled diagram."""

    def __init__(self, plan: ExecutionPlan, diagram, verbose=False):
        try:
            import jax
            import jax.numpy as jnp

            ensure_jax_x64()
        except ImportError as e:
            raise ImportError(
                "JAX is required for the 'jax' backend. "
                "Install with: pip install jax jaxlib"
            ) from e

        self.plan = plan
        self.jax = jax
        self.jnp = jnp

        self.n = plan.state_dim
        self.m = diagram.m
        self.p = (
            sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
            if plan.external_output_slices
            else 0
        )
        self.backend = "jax"
        self.compiled_forms = {}
        self._frozen_params = None
        self._u_nominal = jnp.array(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)
        self._jac_setup(diagram, "dynamic", state_fn=None, outputs_fn=None, plan=plan)

        if verbose:
            t0 = time.perf_counter()
            n_blocks = len(diagram.subsystems)
            print(
                f"[compile] Checking JAX compatibility of {n_blocks} blocks...",
                end="",
                flush=True,
            )

        for sys_id, sys in diagram.subsystems.items():
            dummy_x = jnp.zeros(sys.n)
            dummy_u = jnp.zeros(sys.m)
            params = getattr(sys, "params", {})
            if isinstance(sys, DynamicSystem):
                label = f"{sys_id} ({sys.name})"
                check_jax_compatible(sys.f, label, dummy_x, dummy_u, 0.0, params, jax)
            for port_id, port in sys.outputs.items():
                label = f"{sys_id}:{port_id} ({sys.name})"
                check_jax_compatible(
                    port.compute, label, dummy_x, dummy_u, 0.0, params, jax
                )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    @compiled
    def f(self, x, u, t=0.0, params=None):
        """``dx`` of every block, stacked; ``params=None`` reads each block's compiled parameters."""
        self.require_params(params)
        dtype = self.dtype_of(x, u)
        signals = self.port_signals(x, u, t, dtype, params)
        return self.dx_from_signals(x, u, t, dtype, params, signals)

    @compiled
    def outputs(self, x, u, t=0.0, params=None):
        self.require_params(params)
        dtype = self.dtype_of(x, u)
        signals = self.port_signals(x, u, t, dtype, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    @compiled
    def signals(self, x, u, t=0.0, params=None):
        """The internal signal buffer at ``(x, u, t)``: every block output."""
        dtype = self.dtype_of(x, u)
        return self.port_signals(x, u, t, dtype, params)

    def compute_internal_signals_dict(self, x, u, t=0.0, params=None):
        """Every block output at ``(x, u, t)``, keyed ``"block:port"``."""
        signals = self.signals(x, u, t, params)
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

    def dtype_of(self, x, u):
        """The floating dtype of the buffers: ``x``'s (else ``u``'s), at least float."""
        jnp = self.jnp
        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        if dtype is None:
            return jnp.float32
        # An integer x0 still gets float dx; float inputs keep their width.
        return jnp.result_type(dtype, float)

    def port_signals(self, x, u, t, dtype, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        jnp = self.jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)
        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            y_out = op.compute_func(local_x, local_u, t, op_params)
            signals = signals.at[op.out_slice].set(y_out)
            if inject is not None and op.out_slice == inject[0]:
                signals = signals.at[op.out_slice].add(inject[1])
        return signals

    def dx_from_signals(self, x, u, t, dtype, params, signals):
        """Every block's ``dx`` from the filled signal buffer."""
        jnp = self.jnp
        dx = jnp.zeros(self.plan.state_dim, dtype=dtype)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            dx_piece = op.f_func(local_x, local_u, t, op_params)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

    def _jac_validate_params(self, params):
        check_diagram_params(params, self._subsystem_ids)

    def _jac_probe(self, target, wire=None):
        kind, key = target
        dtype_of = self.dtype_of
        signals_of = self.port_signals

        if kind == "state":
            dx_of = self.dx_from_signals

            def probe(x, u, t, params, delta):
                dtype = dtype_of(x, u)
                signals = signals_of(x, u, t, dtype, params, inject=(wire, delta))
                return dx_of(x, u, t, dtype, params, signals)

            return probe

        sl = self.plan.external_output_slices[key] if kind == "port" else key

        def probe(x, u, t, params, delta):
            dtype = dtype_of(x, u)
            return signals_of(x, u, t, dtype, params, inject=(wire, delta))[sl]

        return probe


class JaxStaticEvaluator(JaxJacobianMixin, StaticEvaluator):
    """JAX evaluator for a static ``System`` leaf."""

    def __init__(self, system: System, verbose=False):
        if isinstance(system, DynamicSystem):
            raise TypeError("JaxStaticEvaluator requires a static System (n=0)")
        if system.n != 0:
            raise TypeError(f"static evaluator requires n=0, got n={system.n}")

        import jax
        import jax.numpy as jnp

        ensure_jax_x64()

        self.jax = jax
        self.jnp = jnp
        self.n = 0
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self.compiled_forms = {}
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        check_leaf_compatible(system, None, jnp.zeros(0), self.m, 0.0, jax)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        self._jac_setup(system, "static", state_fn=None, outputs_fn=self.outputs)

    @compiled
    def outputs(self, x, u, t=0.0, params=None):
        params = self._frozen_params if params is None else params
        return outputs_from_ports(self._system, x, u, t, params)


class JaxStepEvaluator(JaxJacobianMixin, StepEvaluator, StepRolloutMixin):
    """Compiled evaluator for a :class:`StepSystem` using JAX."""

    def __init__(self, system: StepSystem, verbose=False):
        if not isinstance(system, StepSystem):
            raise TypeError("JaxStepEvaluator requires StepSystem")

        import jax
        import jax.numpy as jnp

        ensure_jax_x64()

        self.jax = jax
        self.jnp = jnp
        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self.compiled_forms = {}
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        check_leaf_compatible(system, system.step, jnp.zeros(self.n), self.m, 0, jax)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        self._jac_setup(system, "step", state_fn=self.step, outputs_fn=self.outputs)

    @compiled
    def step(self, x, u, k=0, params=None):
        jnp = self.jnp
        params = self._frozen_params if params is None else params
        return self._system.step(jnp.asarray(x), jnp.asarray(u), k, params)

    @compiled
    def outputs(self, x, u, k=0, params=None):
        jnp = self.jnp
        params = self._frozen_params if params is None else params
        return outputs_from_ports(
            self._system, jnp.asarray(x), jnp.asarray(u), k, params
        )

    def rollout(self, x0, *, n_steps, u=None, params=None):
        """``x_{k+1} = step(x_k, u_k, k)`` for ``n_steps`` transitions, scanned: a :class:`StepRollout`."""
        return scanned_rollout(self, x0, n_steps, u, params)

    @compiled
    def scan_steps(self, x0, u_steps, params=None):
        """``[x_0, …, x_N]``: the step scanned over the rows of ``u_steps`` (the rollout's core)."""
        return scan_steps(self, x0, u_steps, params)


class JaxStepDiagramEvaluator(JaxJacobianMixin, StepEvaluator, StepRolloutMixin):
    """JAX evaluator for a compiled step diagram."""

    def __init__(self, plan: StepExecutionPlan, diagram, verbose=False):
        import jax
        import jax.numpy as jnp

        ensure_jax_x64()

        self.jax = jax
        self.jnp = jnp
        self.plan = plan
        self.n = plan.state_dim
        self.m = diagram.m
        self.p = (
            sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
            if plan.external_output_slices
            else 0
        )
        self.backend = "jax"
        self.compiled_forms = {}
        self._frozen_params = None
        self._u_nominal = jnp.array(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)
        self._dtype = jnp.float64
        self._jac_setup(diagram, "step", state_fn=None, outputs_fn=None, plan=plan)

        dummy_x = jnp.zeros(self.n)
        dummy_k = 0

        if verbose:
            t0 = time.perf_counter()
            print(
                "[compile] Checking JAX compatibility of step diagram...",
                end="",
                flush=True,
            )

        for op in plan.port_ops:
            check_jax_compatible(
                op.compute_func,
                op.label,
                dummy_x[op.local_x_slice],
                jnp.zeros(op.u_dim),
                dummy_k,
                op.bound_params,
                jax,
            )
        for op in plan.step_ops:
            check_jax_compatible(
                op.step_func,
                op.label,
                dummy_x[op.local_x_slice],
                jnp.zeros(op.u_dim),
                dummy_k,
                op.bound_params,
                jax,
            )

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

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

    def rollout(self, x0, *, n_steps, u=None, params=None):
        """``x_{k+1} = step(x_k, u_k, k)`` for ``n_steps`` transitions, scanned: a :class:`StepRollout`."""
        return scanned_rollout(self, x0, n_steps, u, params)

    @compiled
    def scan_steps(self, x0, u_steps, params=None):
        """``[x_0, …, x_N]``: the step scanned over the rows of ``u_steps`` (the rollout's core)."""
        return scan_steps(self, x0, u_steps, params)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_params(self, params):
        """Refuse a params tree naming a block the diagram does not hold."""
        if params is not None:
            check_diagram_params(params, self._subsystem_ids)

    def float_arrays(self, x, u):
        """``x`` and ``u`` as float arrays of the diagram's dtype."""
        jnp = self.jnp
        return jnp.asarray(x, dtype=self._dtype), jnp.asarray(u, dtype=self._dtype)

    def port_signals(self, x, u, k, params, inject=None):
        """Fill the signal buffer; ``inject=(wire_slice, delta)`` adds ``delta`` to one wire."""
        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype
        signals = jnp.zeros(plan.signal_dim, dtype=dtype)
        x_arr = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
        for op in plan.port_ops:
            local_x = x_arr[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            signals = signals.at[op.out_slice].set(
                op.compute_func(local_x, local_u, k, op_params)
            )
            if inject is not None and op.out_slice == inject[0]:
                signals = signals.at[op.out_slice].add(inject[1])
        return signals

    def next_from_signals(self, x, u, k, params, signals):
        """Every block's ``x_{k+1}`` from the filled signal buffer."""
        plan = self.plan
        jnp = self.jnp
        dtype = self._dtype
        x_new = jnp.asarray(x, dtype=dtype).reshape(plan.state_dim)
        for op in plan.step_ops:
            local_x = x_new[op.local_x_slice]
            local_u = gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = op.bound_params if params is None else params.get(op.sys_id)
            x_new = x_new.at[op.local_x_slice].set(
                op.step_func(local_x, local_u, k, op_params)
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


def scanned_rollout(evaluator, x0, n_steps, u, params):
    """A step evaluator's :class:`StepRollout`: the inputs coerced, the steps scanned."""
    jnp = evaluator.jnp
    x0 = jnp.asarray(x0, dtype=float).reshape(evaluator.n)
    u_samples = evaluator._coerce_rollout_inputs(n_steps, u)
    if n_steps == 0:
        x_path = x0.reshape(1, evaluator.n)
    else:
        u_steps = jnp.asarray(u_samples[:, :n_steps].T)
        x_path = evaluator.scan_steps(x0, u_steps, params)
    x_samples = np.asarray(x_path, dtype=float).T
    k = np.arange(n_steps + 1, dtype=float)
    u_np = np.asarray(u_samples, dtype=float)

    return StepRollout(k=k, x=x_samples, u=u_np)


def scan_steps(evaluator, x0, u_steps, params):
    """``[x_0, …, x_N]`` of ``x_{k+1} = step(x_k, u_k, k)`` over the rows of ``u_steps``."""
    jax, jnp = evaluator.jax, evaluator.jnp
    step = evaluator.step
    ks = jnp.arange(u_steps.shape[0])

    def body(x, inputs):
        k, u_k = inputs
        x_next = step(x, u_k, k, params)
        return x_next, x_next

    _, xs = jax.lax.scan(body, x0, (ks, u_steps))

    return jnp.concatenate((x0[None, :], xs), axis=0)


def check_leaf_compatible(system, evolution, dummy_x, m, dummy_t, jax):
    """Refuse a leaf whose evolution (``f`` or ``step``) or an output port does not trace."""
    dummy_u = jax.numpy.zeros(m)
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


def check_jax_compatible(func, label, dummy_x, dummy_u, dummy_t, params, jax):
    """Test if *func* is JAX-traceable via ``jax.make_jaxpr``."""
    from jax.errors import ConcretizationTypeError

    try:
        jax.make_jaxpr(lambda x, u, t: func(x, u, t, params))(dummy_x, dummy_u, dummy_t)
    except (ConcretizationTypeError, TypeError, Exception) as e:
        raise RuntimeError(
            f"\n\nBlock '{label}' is not JAX-traceable.\n"
            f"Its f()/compute() likely performs in-place array mutation "
            f"(e.g., x[0] = ...)\n"
            f"or uses Python control flow on traced values "
            f"(e.g., if t < threshold).\n"
            f"Rewrite in purely functional style or use backend='numpy'.\n"
            f"Original JAX error: {e}"
        ) from e


def gather_u_jax(gather_sources, u_dim, signals, u, jnp, dtype):
    """Assemble the local input vector using JAX operations."""
    if u_dim == 0:
        return jnp.array([], dtype=dtype)

    pieces = []
    for src_type, src_val, dim in gather_sources:
        if src_type == INTERNAL_SIGNAL:
            pieces.append(signals[src_val])
        elif src_type == NOMINAL:
            pieces.append(jnp.asarray(src_val, dtype=dtype))
        elif src_type == EXTERNAL_INPUT:
            pieces.append(u[src_val])
        else:
            raise RuntimeError(f"Unknown source_type={src_type}")

    return jnp.concatenate(pieces, axis=0) if pieces else jnp.array([], dtype=dtype)
