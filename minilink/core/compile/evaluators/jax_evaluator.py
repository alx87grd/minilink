"""
JAX evaluation backends for compiled systems.

Provides two evaluators:

- :class:`JaxLeafEvaluator` — for a single (non-diagram) System.
- :class:`JaxDiagramEvaluator` — for a compiled DiagramSystem.

Both inherit from :class:`~minilink.core.compile.evaluators.evaluator.DynamicsEvaluator`.

**Limitations**:
- Subsystem ``f`` / ``h`` / port compute functions must be JAX-traceable
  (no in-place mutation, no Python-side branching on traced values).
- Lazy import: JAX is imported at class instantiation, not at module level.
"""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.core.compile.evaluators.evaluator import DynamicsEvaluator
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
)
from minilink.core.diagram import validate_diagram_params


# JAX compatibility checking
def _check_jax_compatible(func, label, dummy_x, dummy_u, dummy_t, params, jax):
    """Test if *func* is JAX-traceable via ``jax.make_jaxpr``.

    Raises :class:`RuntimeError` with a clear diagnostic if tracing fails.
    """
    from jax.errors import ConcretizationTypeError

    try:
        jax.make_jaxpr(lambda x, u, t: func(x, u, t, params))(dummy_x, dummy_u, dummy_t)
    except (ConcretizationTypeError, TypeError, Exception) as e:
        raise RuntimeError(
            f"\n\nBlock '{label}' is not JAX-traceable.\n"
            f"Its f()/h()/compute() likely performs in-place array mutation "
            f"(e.g., x[0] = ...)\n"
            f"or uses Python control flow on traced values "
            f"(e.g., if t < threshold).\n"
            f"Rewrite in purely functional style or use backend='numpy'.\n"
            f"Original JAX error: {e}"
        ) from e


def _build_jit_rk4_rollout_ivp(jax, jnp, f_ivp):
    def _rk4_rollout_ivp(x0_, t0_, dt_, n_steps_):
        def body(carry, _):
            x, t = carry
            k1 = f_ivp(x, t)
            k2 = f_ivp(x + 0.5 * dt_ * k1, t + 0.5 * dt_)
            k3 = f_ivp(x + 0.5 * dt_ * k2, t + 0.5 * dt_)
            k4 = f_ivp(x + dt_ * k3, t + dt_)
            x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(body, (x0_, t0_), jnp.arange(n_steps_))
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return jax.jit(_rk4_rollout_ivp, static_argnums=3)


def _build_jit_rk4_rollout_forced(jax, jnp, f):
    def _rk4_rollout_forced(x0_, u_knots_, t0_, dt_):
        def body(carry, u_pair):
            x, t = carry
            u0, u1 = u_pair
            umid = 0.5 * (u0 + u1)
            k1 = f(x, u0, t)
            k2 = f(x + 0.5 * dt_ * k1, umid, t + 0.5 * dt_)
            k3 = f(x + 0.5 * dt_ * k2, umid, t + 0.5 * dt_)
            k4 = f(x + dt_ * k3, u1, t + dt_)
            x_next = x + (dt_ / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            return (x_next, t + dt_), x_next

        (_, _), xs = jax.lax.scan(
            body,
            (x0_, t0_),
            (u_knots_[:-1], u_knots_[1:]),
        )
        return jnp.concatenate((x0_[None, :], xs), axis=0)

    return jax.jit(_rk4_rollout_forced)


# Leaf evaluator
class JaxLeafEvaluator(DynamicsEvaluator):
    """Compiled evaluator for a single System using JAX.

    Core callables (``f``, ``h``, ``outputs``, ``f_p``, ``h_p``, ``f_ivp``,
    ``outputs_p``) are JIT-compiled at construction and warm-started
    with dummy data so that the first real call incurs no compilation latency.

    Parameters
    ----------
    system : System
        The system to wrap.  Its ``params`` are deep-copied at construction
        (frozen), and nominal input port values are snapshotted.
    verbose : bool
        If ``True``, print timed compilation steps.
    """

    def __init__(self, system, verbose=False):
        import jax
        import jax.numpy as jnp

        self.jax = jax
        self.jnp = jnp

        self.n = system.n
        self.m = system.m
        self.p = system.p
        self.backend = "jax"
        self._system = system
        self._frozen_params = copy.deepcopy(system.params)
        self._u_nominal = jnp.array(system.get_u_from_input_ports())

        # Store references to the raw system functions
        f_raw = system.f
        h_raw = system.h
        frozen_p = self._frozen_params
        u_nom = self._u_nominal

        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        # Step 0: JAX compatibility check
        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 0: Checking JAX compatibility of '{system.name}'...",
                end="",
                flush=True,
            )

        _check_jax_compatible(
            f_raw, system.name, dummy_x, dummy_u, dummy_t, frozen_p, jax
        )
        _check_jax_compatible(
            h_raw, system.name, dummy_x, dummy_u, dummy_t, frozen_p, jax
        )
        for port_id, port in system.outputs.items():
            _check_jax_compatible(
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

        # Step 1: JIT-compile core callables
        # Frozen-params tier: params baked into closure, JAX never sees them
        # as an argument — no static_argnames needed.
        # Parametric tier: params passed as arg and expected to vary — also
        # no static_argnames (caller controls recompilation).
        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 1: JIT-compiling to XLA on {jax.default_backend()}...",
                end="",
                flush=True,
            )

        self._jit_f = jax.jit(lambda x, u, t: f_raw(x, u, t, frozen_p))
        self._jit_h = jax.jit(lambda x, u, t: h_raw(x, u, t, frozen_p))
        self._jit_f_p = jax.jit(lambda x, u, t, p: f_raw(x, u, t, p))
        self._jit_h_p = jax.jit(lambda x, u, t, p: h_raw(x, u, t, p))
        self._jit_jac_f_params = jax.jit(
            jax.jacfwd(lambda x, u, t, p: f_raw(x, u, t, p), argnums=3)
        )
        self._jit_f_ivp = jax.jit(lambda x, t: f_raw(x, u_nom, t, frozen_p))
        self._jit_jac_ivp = jax.jit(jax.jacfwd(self._jit_f_ivp, argnums=0))

        self._jit_rk4_rollout_ivp = _build_jit_rk4_rollout_ivp(
            jax, jnp, self._jit_f_ivp
        )
        self._jit_rk4_rollout_forced = _build_jit_rk4_rollout_forced(
            jax, jnp, self._jit_f
        )

        output_items = tuple(
            (pid, port.compute) for pid, port in system.outputs.items()
        )

        def _outputs_frozen(x, u, t):
            return {pid: fn(x, u, t, frozen_p) for pid, fn in output_items}

        def _outputs_param(x, u, t, p):
            return {pid: fn(x, u, t, p) for pid, fn in output_items}

        if output_items:
            self._jit_outputs = jax.jit(_outputs_frozen)
            self._jit_outputs_p = jax.jit(_outputs_param)
        else:
            self._jit_outputs = jax.jit(lambda x, u, t: {})
            self._jit_outputs_p = jax.jit(lambda x, u, t, p: {})

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        # Step 2: Warm-start with dummy data
        if verbose:
            t0 = time.perf_counter()
            print("[compile] Step 2: Warm-starting JIT cache...", end="", flush=True)

        try:
            self._jit_f(dummy_x, dummy_u, dummy_t)
            self._jit_h(dummy_x, dummy_u, dummy_t)
            self._jit_f_p(dummy_x, dummy_u, dummy_t, frozen_p)
            self._jit_h_p(dummy_x, dummy_u, dummy_t, frozen_p)
            self._jit_f_ivp(dummy_x, dummy_t)
            self._jit_outputs(dummy_x, dummy_u, dummy_t)
            self._jit_outputs_p(dummy_x, dummy_u, dummy_t, frozen_p)
        except Exception as e:
            raise RuntimeError(
                f"\n\nBlock '{system.name}' failed during JAX warm-start.\n"
                f"Its f()/h()/outputs likely performs in-place array mutation "
                f"or uses strict NumPy operations incompatible with JAX.\n"
                f"Rewrite in purely functional style or use backend='numpy'.\n"
                f"Original JAX error: {e}"
            ) from e

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    # Standard tier (frozen params)
    def f(self, x, u, t=0.0):
        return self._jit_f(x, u, t)

    def f_scipy(self, x, u, t=0.0):
        x = self.jnp.asarray(x)
        u = self.jnp.asarray(u)
        return np.asarray(self._jit_f(x, u, t))

    def h(self, x, u, t=0.0):
        return self._jit_h(x, u, t)

    def outputs(self, x, u, t=0.0):
        return self._jit_outputs(x, u, t)

    # Parametric tier (caller-supplied params)
    def f_p(self, x, u, t, params):
        return self._jit_f_p(x, u, t, params)

    def h_p(self, x, u, t, params):
        return self._jit_h_p(x, u, t, params)

    def outputs_p(self, x, u, t, params):
        return self._jit_outputs_p(x, u, t, params)

    def jacobian_f_params(self, x, u, t, params):
        """∂f/∂θ as a pytree matching ``params``; leaves have shape (n, *leaf_shape)."""
        if params is None:
            raise ValueError("jacobian_f_params requires an explicit params pytree")
        return self._jit_jac_f_params(x, u, t, params)

    def get_f_p_jit(self):
        """Return the JIT-compiled parametric ``f_p`` callable (for grad/vmap composition)."""
        return self._jit_f_p

    def get_outputs_jit(self):
        """Return the JIT-compiled ``outputs`` callable (same as :meth:`outputs`)."""
        return self._jit_outputs

    def get_outputs_p_jit(self):
        """Return the JIT-compiled parametric ``outputs_p`` callable."""
        return self._jit_outputs_p

    # IVP tier (override ABC defaults with JIT versions)
    def f_ivp(self, x, t=0.0):
        return self._jit_f_ivp(x, t)

    def f_ivp_scipy(self, x, t=0.0):
        x = self.jnp.asarray(x)
        return np.asarray(self._jit_f_ivp(x, t))

    def as_scipy_jac(self):
        return lambda t, x: np.asarray(self._jit_jac_ivp(self.jnp.asarray(x), t))

    def rk4_step_ivp(self, x, t, dt):
        f = self._jit_f_ivp
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_rollout_ivp(self, x0, t0, dt, n_steps):
        jnp = self.jnp

        x0 = jnp.asarray(x0)
        t0 = jnp.asarray(t0)
        dt = jnp.asarray(dt)
        return self._jit_rk4_rollout_ivp(x0, t0, dt, n_steps)

    def rk4_rollout_forced(self, x0, u_knots, t0, dt):
        jnp = self.jnp

        x0 = jnp.asarray(x0)
        u_knots = jnp.asarray(u_knots)
        t0 = jnp.asarray(t0)
        dt = jnp.asarray(dt)
        return self._jit_rk4_rollout_forced(x0, u_knots, t0, dt)


# JAX signal gathering
def _gather_u_jax(gather_sources, u_dim, signals, u, jnp, dtype):
    """Assemble the local input vector using JAX operations.

    Unlike the NumPy version, this uses ``jnp.concatenate`` on collected
    pieces (no in-place mutation) so the computation stays JAX-traceable.

    Parameters
    ----------
    gather_sources : tuple of (source_type, source_value, dim)
    u_dim : int
    signals : jax array
        Internal signal buffer.
    u : jax array
        Diagram external input vector.
    jnp : module
        ``jax.numpy`` (passed explicitly to avoid module-level import).
    dtype : jax dtype
        Output dtype for constant arrays.

    Returns
    -------
    jax array
    """
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


# Diagram evaluator
class JaxDiagramEvaluator(DynamicsEvaluator):
    """JAX-compatible evaluator for a compiled diagram.

    Inherits from :class:`DynamicsEvaluator`.  Implements ``f`` and JIT-compiled
    :meth:`outputs` for **diagram boundary** ports only (same semantics as
    :class:`JaxLeafEvaluator`).  :meth:`compute_internal_signals_dict` exposes
    all subsystem outputs in the buffer.  :meth:`h` applies when there is
    exactly one boundary output.  The parametric tier (:meth:`f_p`,
    :meth:`outputs_p`, :meth:`jacobian_f_params`) takes nested params
    ``{sys_id: {...}}`` as a JAX pytree, enabling differentiation with
    respect to parameters.

    All operations use functional array updates so they are traceable
    by JAX's transformation system (``jit``, ``grad``, ``vmap``, etc.).

    At construction the evaluator JIT-compiles ``f`` and ``outputs`` and
    warm-starts with dummy data, so the first real call incurs no compilation
    latency — matching the behaviour of :class:`JaxLeafEvaluator`.

    Parameters
    ----------
    plan : ExecutionPlan
        The immutable execution schedule produced by
        :func:`~minilink.core.compile.compiler.build_execution_plan`.
    diagram : DiagramSystem
        The source diagram (used only to snapshot ``m`` and ``_u_nominal``).
    verbose : bool
        If ``True``, print timed compilation steps.

    Examples
    --------
    >>> evaluator = compile_diagram(diagram, backend="jax")
    >>> dx = evaluator.f(x_jax, u_jax, t)
    """

    def __init__(self, plan: ExecutionPlan, diagram, verbose=False):
        try:
            import jax
            import jax.numpy as jnp
        except ImportError as e:
            raise ImportError(
                "JAX is required for the 'jax' backend. "
                "Install with: pip install jax jaxlib"
            ) from e

        self.plan = plan
        self._jax = jax
        self._jnp = jnp

        self.n = plan.state_dim
        self.m = diagram.m
        self.p = (
            sum(diagram.outputs[pid].dim for pid in plan.external_output_slices)
            if plan.external_output_slices
            else 0
        )
        self.backend = "jax"
        self._frozen_params = None  # per-op binding, not diagram-level
        self._u_nominal = jnp.array(diagram.get_u_from_input_ports())
        self._subsystem_ids = tuple(diagram.subsystems)

        # Step 0 (JAX): Check compatibility of all subsystem blocks
        if verbose:
            t0 = time.perf_counter()
            n_blocks = len(diagram.subsystems)
            print(
                f"[compile] Step 0: Checking JAX compatibility of {n_blocks} blocks...",
                end="",
                flush=True,
            )

        self._check_jax_compatibility(diagram, jax, jnp)

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        # Step 3 (JAX): JIT-compile to XLA
        if verbose:
            t0 = time.perf_counter()
            print(
                f"[compile] Step 3: JIT-compiling to XLA on {jax.default_backend()}...",
                end="",
                flush=True,
            )

        # Store reference to the eager (traceable) implementation, then JIT.
        # Params are captured at trace time: if bind_params=False, subsystem
        # f()/compute() reads self.params during tracing, effectively freezing
        # those values into the compiled XLA code.
        _f_eager = self._f_eager
        self._jit_f = jax.jit(_f_eager)

        u_nom = self._u_nominal
        self._jit_f_ivp = jax.jit(lambda x, t: _f_eager(x, u_nom, t))
        self._jit_jac_ivp = jax.jit(jax.jacfwd(self._jit_f_ivp, argnums=0))

        self._jit_rk4_rollout_ivp = _build_jit_rk4_rollout_ivp(
            jax, jnp, self._jit_f_ivp
        )
        self._jit_rk4_rollout_forced = _build_jit_rk4_rollout_forced(
            jax, jnp, self._jit_f
        )

        self._jit_outputs = jax.jit(
            lambda x, u, t: self._external_outputs_eager(x, u, t)
        )

        self._jit_internal_signals = jax.jit(
            lambda x, u, t: self._internal_signals_eager(x, u, t)
        )

        # Parametric tier: nested params {sys_id: {...}} enter as a pytree
        # argument, so leaf values vary without retracing; changing the dict
        # *structure* (which sys_ids / which keys) triggers a retrace.
        self._jit_f_p = jax.jit(self._f_p_eager)
        self._jit_outputs_p = jax.jit(self._outputs_p_eager)
        self._jit_jac_f_params = jax.jit(jax.jacfwd(self._f_p_eager, argnums=3))

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

        # Step 4 (JAX): Warm-start JIT cache
        if verbose:
            t0 = time.perf_counter()
            print("[compile] Step 4: Warm-starting JIT cache...", end="", flush=True)

        dummy_x = jnp.zeros(self.n)
        dummy_u = jnp.zeros(self.m)
        dummy_t = 0.0

        try:
            self._jit_f(dummy_x, dummy_u, dummy_t)
            self._jit_f_ivp(dummy_x, dummy_t)
            self._jit_outputs(dummy_x, dummy_u, dummy_t)
            self._jit_internal_signals(dummy_x, dummy_u, dummy_t)
        except Exception as e:
            raise RuntimeError(
                f"\n\nDiagram failed during JAX warm-start.\n"
                f"One or more subsystem functions are not fully "
                f"JAX-traceable.\n"
                f"Original JAX error: {e}"
            ) from e

        # Best-effort warm start of the parametric tier with the nominal
        # nested params. This may fail when some subsystem params are not
        # numeric pytrees; the tier still works for numeric (partial) params
        # dicts — subsystems left out of the caller dict have their live
        # params read at trace time as constants — so failure here only costs
        # first-call latency, not functionality.
        params_nominal = {
            sys_id: sub.params for sys_id, sub in diagram.subsystems.items()
        }
        try:
            self._jit_f_p(dummy_x, dummy_u, dummy_t, params_nominal)
            self._jit_outputs_p(dummy_x, dummy_u, dummy_t, params_nominal)
        except Exception:
            pass

        if verbose:
            print(f"  ({time.perf_counter() - t0:.3f}s)")

    # Diagram boundary outputs (JAX-traceable; same contract as leaf)
    def _external_outputs_eager(self, x, u, t):
        """Boundary outputs only — keys match :attr:`DiagramSystem.outputs`."""
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals(x, u, t, dtype)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def _internal_signals_eager(self, x, u, t):
        """Full internal signal buffer (same graph as :meth:`_compute_port_signals`)."""
        dtype = self._infer_dtype(x, u)
        return self._compute_port_signals(x, u, t, dtype)

    # JAX compatibility pre-flight
    def _check_jax_compatibility(self, diagram, jax, jnp):
        """Test each subsystem's f() and port compute() for JAX traceability."""
        for sys_id, sys in diagram.subsystems.items():
            dummy_x = jnp.zeros(sys.n)
            dummy_u = jnp.zeros(sys.m)
            dummy_t = 0.0
            params = getattr(sys, "params", {})

            # Check f() for dynamic systems
            if sys.n > 0:
                _check_jax_compatible(
                    sys.f,
                    f"{sys_id} ({sys.name})",
                    dummy_x,
                    dummy_u,
                    dummy_t,
                    params,
                    jax,
                )

            # Check each output port compute()
            for port_id, port in sys.outputs.items():
                _check_jax_compatible(
                    port.compute,
                    f"{sys_id}:{port_id} ({sys.name})",
                    dummy_x,
                    dummy_u,
                    dummy_t,
                    params,
                    jax,
                )

    # Dtype inference
    def _infer_dtype(self, x, u):
        """Best-effort dtype inference from input arrays."""
        jnp = self._jnp
        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        return dtype if dtype is not None else jnp.float32

    # Eager f implementation (JAX-traceable, used by jit)
    def f(self, x, u, t=0.0):
        """ẋ = f(x, u, t) — delegates to the JIT-compiled version."""
        return self._jit_f(x, u, t)

    def f_scipy(self, x, u, t=0.0):
        x = self._jnp.asarray(x)
        u = self._jnp.asarray(u)
        return np.asarray(self._jit_f(x, u, t))

    def _f_eager(self, x, u, t=0.0):
        """Compute the diagram's state derivative vector (JAX-traceable).

        This is the eager (un-JIT'd) implementation that JAX traces through.
        The public ``f`` delegates to the JIT-compiled wrapper of this method.

        Parameters
        ----------
        x : jax array, shape (state_dim,)
            Global state vector.
        u : jax array, shape (m,)
            External input vector.
        t : float or jax scalar
            Current time.

        Returns
        -------
        jax array, shape (state_dim,)
            State derivative vector ``dx/dt``.
        """
        jnp = self._jnp
        dtype = self._infer_dtype(x, u)

        signals = self._compute_port_signals(x, u, t, dtype)

        dx = jnp.zeros(self.plan.state_dim, dtype=dtype)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            dx_piece = op.f_func(local_x, local_u, t, op.bound_params)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

    def _f_p_eager(self, x, u, t, params):
        """Parametric ``f`` (JAX-traceable): nested params routed per op."""
        jnp = self._jnp
        dtype = self._infer_dtype(x, u)

        signals = self._compute_port_signals_p(x, u, t, dtype, params)

        dx = jnp.zeros(self.plan.state_dim, dtype=dtype)
        for op in self.plan.state_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = None if params is None else params.get(op.sys_id)
            dx_piece = op.f_func(local_x, local_u, t, op_params)
            dx = dx.at[op.local_x_slice].set(dx_piece)
        return dx

    def _outputs_p_eager(self, x, u, t, params):
        """Parametric boundary outputs (JAX-traceable)."""
        dtype = self._infer_dtype(x, u)
        signals = self._compute_port_signals_p(x, u, t, dtype, params)
        return {
            port_id: signals[sl]
            for port_id, sl in self.plan.external_output_slices.items()
        }

    def h(self, x, u, t=0.0):
        out = self.outputs(x, u, t)
        if len(out) == 1:
            return next(iter(out.values()))
        raise NotImplementedError(
            "Diagram h() is only defined when exactly one diagram output port "
            "exists; use outputs()."
        )

    def outputs(self, x, u, t=0.0):
        return self._jit_outputs(x, u, t)

    # ABC: Parametric tier
    def f_p(self, x, u, t, params):
        """Diagram ``f`` with caller-supplied nested params ``{sys_id: {...}}``.

        ``params`` must be a numeric pytree (dicts of arrays/scalars). Missing
        subsystem ids fall back to each block's live ``self.params`` (read at
        trace time); ``bound_params`` (frozen tier) is ignored here.
        """
        validate_diagram_params(params, self._subsystem_ids)
        return self._jit_f_p(x, u, t, params)

    def h_p(self, x, u, t, params):
        out = self.outputs_p(x, u, t, params)
        if len(out) == 1:
            return next(iter(out.values()))
        raise NotImplementedError(
            "Diagram h_p() is only defined when exactly one diagram output "
            "port exists; use outputs_p()."
        )

    def outputs_p(self, x, u, t, params):
        """Boundary outputs with caller-supplied nested params (see :meth:`f_p`)."""
        validate_diagram_params(params, self._subsystem_ids)
        return self._jit_outputs_p(x, u, t, params)

    def jacobian_f_params(self, x, u, t, params):
        """∂f/∂θ as a pytree matching ``params``; leaves have shape (n, *leaf_shape).

        ``params`` follows the same nested contract as :meth:`f_p` and must be
        supplied explicitly (the derivative is taken with respect to it).
        """
        if params is None:
            raise ValueError(
                "jacobian_f_params requires an explicit nested params pytree"
            )
        validate_diagram_params(params, self._subsystem_ids)
        return self._jit_jac_f_params(x, u, t, params)

    # IVP tier
    def f_ivp(self, x, t=0.0):
        return self._jit_f_ivp(x, t)

    def f_ivp_scipy(self, x, t=0.0):
        x = self._jnp.asarray(x)
        return np.asarray(self._jit_f_ivp(x, t))

    def as_scipy_jac(self):
        return lambda t, x: np.asarray(self._jit_jac_ivp(self._jnp.asarray(x), t))

    def rk4_step_ivp(self, x, t, dt):
        f = self._jit_f_ivp
        k1 = f(x, t)
        k2 = f(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def rk4_rollout_ivp(self, x0, t0, dt, n_steps):
        jnp = self._jnp

        x0 = jnp.asarray(x0)
        t0 = jnp.asarray(t0)
        dt = jnp.asarray(dt)
        return self._jit_rk4_rollout_ivp(x0, t0, dt, n_steps)

    def rk4_rollout_forced(self, x0, u_knots, t0, dt):
        jnp = self._jnp

        x0 = jnp.asarray(x0)
        u_knots = jnp.asarray(u_knots)
        t0 = jnp.asarray(t0)
        dt = jnp.asarray(dt)
        return self._jit_rk4_rollout_forced(x0, u_knots, t0, dt)

    # JIT convenience
    def get_f_jit(self):
        """Return the JIT-compiled ``f`` callable directly (skips method dispatch)."""
        return self._jit_f

    def get_f_p_jit(self):
        """Return the JIT-compiled parametric ``f_p`` callable (for grad/vmap composition).

        Note: unlike :meth:`f_p`, the raw callable does not validate the
        nested params keys.
        """
        return self._jit_f_p

    def get_outputs_jit(self):
        """Return the JIT-compiled ``outputs`` callable (same as :meth:`outputs`)."""
        return self._jit_outputs

    def get_internal_signals_jit(self):
        """Return the JIT-compiled ``compute_internal_signals`` callable."""
        return self._jit_internal_signals

    # Diagram-specific methods
    def compute_internal_signals(self, x, u, t=0.0):
        """Evaluate and return the full internal signal buffer (flat JAX array).

        JIT-compiled at evaluator construction (same schedule as ``f``'s port
        evaluation). Use :meth:`get_internal_signals_jit` for the raw callable.

        Parameters
        ----------
        x : jax array, shape (state_dim,)
        u : jax array, shape (m,)
        t : float or jax scalar

        Returns
        -------
        jax array, shape (signal_dim,)
        """
        return self._jit_internal_signals(x, u, t)

    def compute_internal_signals_dict(self, x, u, t=0.0):
        """Full internal buffer as ``\"sys_id:port_id\"`` → array (diagram-specific)."""
        signals = self.compute_internal_signals(x, u, t)
        return {
            f"{sys_id}:{port_id}": signals[sl]
            for (sys_id, port_id), sl in self.plan.output_slices.items()
        }

    # Private
    def _compute_port_signals(self, x, u, t, dtype):
        """Evaluate all port signals in topological order (frozen params)."""
        jnp = self._jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)

        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            y_out = op.compute_func(local_x, local_u, t, op.bound_params)
            signals = signals.at[op.out_slice].set(y_out)
        return signals

    def _compute_port_signals_p(self, x, u, t, dtype, params):
        """Port-signal buffer with caller-supplied nested params (JAX-traceable)."""
        jnp = self._jnp
        signals = jnp.zeros(self.plan.signal_dim, dtype=dtype)

        for op in self.plan.port_ops:
            local_x = x[op.local_x_slice]
            local_u = _gather_u_jax(op.gather_sources, op.u_dim, signals, u, jnp, dtype)
            op_params = None if params is None else params.get(op.sys_id)
            y_out = op.compute_func(local_x, local_u, t, op_params)
            signals = signals.at[op.out_slice].set(y_out)
        return signals
