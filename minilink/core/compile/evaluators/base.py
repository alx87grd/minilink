"""
The compiled evaluator's base: one class per system kind, the backend picked at construction.

Every method is one body taking ``params=None`` (``None`` reads the parameters
compiled in), written once on the array module; on JAX each is compiled once
(:func:`~minilink.core.compile.evaluators.compiled.compiled`). The step model
of a continuous evaluator (``step``, ``rollout``, ``rollout_batch``) lives here.
"""

from __future__ import annotations

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    ensure_jax_x64,
    require_jax,
    require_jax_numpy,
)
from minilink.core.compile.evaluators.compiled import compiled
from minilink.core.compile.evaluators.jacobian import JacobianMixin
from minilink.core.compile.evaluators.stepping import (
    held_inputs,
    iterate,
    require_step_knobs,
    step_body,
)
from minilink.core.compile.plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
)
from minilink.core.params import canonical_tree, flat_leaves, tree_digest

# Above this many numbers a frozen leaf is passed as an argument, not baked into the executable
LARGE_LEAF = 1_000


###############################################################################
# Public API
###############################################################################


class Evaluator(JacobianMixin):
    """
    A compiled system: its sizes, its backend, its nominal input and its boundary ``outputs``.

    ``backend`` is ``"numpy"`` or ``"jax"``; on JAX the constructor binds
    ``jax`` and ``jnp`` and starts the compiled forms empty, so JAX is imported
    only by a JAX evaluator.
    """

    def __init__(self, n, m, p, backend, u_nominal):
        self.n = int(n)
        self.m = int(m)
        self.p = int(p)
        self.backend = backend
        self.compiled_forms = {}

        # The array module: NumPy, or JAX imported here and only here, under the
        # float64 policy applied again for every evaluator
        if backend == BACKEND_JAX:
            ensure_jax_x64()
            self.jax = require_jax()
            self.jnp = require_jax_numpy()
            self.xp = self.jnp
        else:
            self.jax = None
            self.jnp = None
            self.xp = np
        self._u_nominal = self.xp.array(np.asarray(u_nominal, dtype=float))
        self._frozen_params = None
        self.params_reader = None
        self.on_device = None

    @property
    def params(self):
        """The parameters taken at compile: ``params=None`` reads these."""
        return self._frozen_params

    def live_params(self):
        """The parameters read from the system now: the tree a new compile would freeze."""
        return self.params_reader()

    def call_params(self):
        """``None`` when the system's parameters still equal the compiled ones (the frozen form), else the live tree."""
        live = self.live_params()
        if tree_digest(live) == tree_digest(self._frozen_params):
            return None

        return live

    def call_tree(self, params):
        """What a compiled method receives for ``params``: the frozen form, or a tree in its compiled types.

        ``None`` stays ``None`` (the compiled values constants of the trace), unless
        a leaf is large (network weights), where the frozen tree is passed as an
        argument instead, so the executable carries no copy. A tree is put on the
        device in its compiled dtypes and shapes, kept there while its values
        repeat: every value of every number type is then one trace.
        """
        if params is None and not has_large_leaf(self._frozen_params):
            return None
        tree = self._frozen_params if params is None else params

        # The tree in its compiled types, on the device: put there once and kept
        # while its values repeat (a host loop's calls transfer nothing)
        digest = tree_digest(tree)
        if self.on_device is None or self.on_device[0] != digest:
            canonical = canonical_tree(tree, self._frozen_params)
            self.on_device = (digest, self.jax.device_put(canonical))

        return self.on_device[1]

    @property
    def nominal_input(self):
        """The input held when none is given: every port at its nominal value."""
        return self._u_nominal

    def indices(self, n):
        """The step indices ``0 … n − 1``: Python integers on NumPy, an array on JAX."""
        return range(n) if self.xp is np else self.jnp.arange(n)

    def outputs(self, x, u, t=0.0, params=None) -> dict:
        """Every boundary output port at ``(x, u, t)``; ``params=None`` reads the compiled ones."""
        raise NotImplementedError

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def batched_rollout(self, run, x0s, u_sequences, n_steps, params, *shared):
        """``run(x0, u, p, *shared)`` over a family in one jitted ``jax.vmap`` (JAX).

        The family is the starts ``x0s`` ``(B, n)``, the inputs (the nominal one
        for ``n_steps``, one ``(N, m)`` sequence shared, or ``(B, N, m)``) and the
        parameter leaves with one more dimension than the compiled value; the
        ``shared`` arguments are the same for every member.
        """
        require_jax_evaluator(self, "rollout_batch")
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
        shared_axes = (None,) * len(shared)

        # One jitted vmap per (input layout, family layout): repeated sweeps
        # with the same shapes reuse the compiled rollout instead of re-tracing.
        cache = self.compiled_forms.setdefault("rollout_batch", {})
        if params is None:
            key = (u_axis, None)
            fn = cache.get(key)
            if fn is None:
                fn = jax.jit(
                    jax.vmap(
                        lambda x0, u, *rest: run(x0, u, None, *rest),
                        in_axes=(0, u_axis, *shared_axes),
                    )
                )
                cache[key] = fn
            return fn(x0s, u_seq, *shared)
        axes = self.params_batch_axes(params, batch)
        leaves, treedef = jax.tree_util.tree_flatten(axes, is_leaf=lambda a: a is None)
        key = (u_axis, str(treedef), tuple(leaves))
        fn = cache.get(key)
        if fn is None:
            fn = jax.jit(
                jax.vmap(
                    lambda x0, u, p, *rest: run(x0, u, p, *rest),
                    in_axes=(0, u_axis, axes, *shared_axes),
                )
            )
            cache[key] = fn
        return fn(x0s, u_seq, params, *shared)

    def params_batch_axes(self, params, batch):
        """``in_axes`` for ``params``: 0 on family leaves, ``None`` on shared ones.

        A leaf is a family when it has one more dimension than the compiled
        value at the same path.
        """
        tree_util = self.jax.tree_util
        reference = {
            tree_util.keystr(path): np.ndim(leaf)
            for path, leaf in tree_util.tree_leaves_with_path(self._frozen_params)
        }

        def axis(path, leaf):
            ref_ndim = reference.get(tree_util.keystr(path))
            if ref_ndim is None:
                return None
            return 0 if np.ndim(leaf) == ref_ndim + 1 else None

        return tree_util.tree_map_with_path(axis, params)


class ContinuousEvaluator(Evaluator):
    """A compiled continuous-time system: ``f``, ``outputs`` and the step model on ``f``."""

    def f(self, x, u, t=0.0, params=None):
        """``dx = f(x, u, t; p)``; ``params=None`` reads the compiled parameters."""
        raise NotImplementedError

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
        t_start,
        dt,
        params=None,
        *,
        solver="rk4",
        substeps=1,
        hold="zoh",
        n_steps=None,
    ):
        """``[x_0, …, x_N]``, shape ``(N + 1, n)``: the step model run over ``u_seq``.

        ``hold="zoh"`` holds each row of ``u_seq`` (shape ``(N, m)``) over its
        period; ``hold="foh"`` reads ``u_seq`` as ``N + 1`` knots, linear between
        them. ``u_seq=None`` holds the nominal input for ``n_steps`` periods.
        """
        xp = self.xp
        require_step_knobs(solver, substeps, hold)

        f = self.rhs_of(params)
        x0 = xp.asarray(x0, dtype=float).reshape(self.n)
        u_seq = self.input_sequence_of(u_seq, n_steps, hold)
        u_start, u_end = held_inputs(u_seq, hold)
        ks = self.indices(u_start.shape[0])

        # x_{k+1} = Φ(x_k, u_k, t_k), t_k = t_start + k dt
        def period(x, k, ua, ub):
            return step_body(f, x, ua, ub, t_start + k * dt, dt, solver, substeps)

        x_samples = iterate(self, period, x0, (ks, u_start, u_end))

        return x_samples

    def rollout_batch(
        self, x0s, u_sequences=None, *, t_start=0.0, dt, n_steps=None, params=None
    ):
        """Roll out a family of RK4 / ZOH trajectories in one ``jax.vmap`` call (JAX).

        Parameters
        ----------
        x0s : array, shape ``(B, n)``
            One initial state per family member.
        u_sequences : None, ``(N, m)``, or ``(B, N, m)``
            Held inputs per step: the compiled nominal input (``n_steps``
            required), one sequence shared by the batch, or one per member.
        t_start, dt : float
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
        t_start = self.jnp.asarray(t_start, dtype=float)
        dt = self.jnp.asarray(dt, dtype=float)

        return self.batched_rollout(
            lambda x0, u, p, t_start, dt: self.rollout(x0, u, t_start, dt, p),
            x0s,
            u_sequences,
            n_steps,
            params,
            t_start,
            dt,
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def rhs_of(self, params):
        """``f(x, u, t)`` on the compiled parameters, or on ``params``."""
        f = self.f

        return lambda x, u, t: f(x, u, t, params)

    def input_sequence_of(self, u_seq, n_steps, hold):
        """The input sequence as a ``(rows, m)`` array; the nominal input repeated when None."""
        xp = self.xp
        if u_seq is None:
            rows = int(n_steps) + (1 if hold == "foh" else 0)
            return xp.tile(xp.asarray(self._u_nominal, dtype=float), (rows, 1))

        u_seq = xp.asarray(u_seq, dtype=float)
        if u_seq.ndim != 2 or u_seq.shape[1] != self.m:
            raise ValueError(f"u_seq must have shape (N, {self.m}), got {u_seq.shape}")

        return u_seq


def blocks_params(diagram, frozen):
    """A diagram's parameters now, by the block ids its compiled tree holds."""
    return {sys_id: diagram.subsystems[sys_id].params for sys_id in frozen}


def has_large_leaf(tree) -> bool:
    """Whether a leaf of ``tree`` holds more than :data:`LARGE_LEAF` numbers (network weights)."""
    return any(
        np.size(leaf) > LARGE_LEAF for leaf in flat_leaves(tree) if leaf is not None
    )


def outputs_from_ports(system, x, u, t, params) -> dict:
    """Evaluate every output port on ``system``."""
    return {
        port_id: port.compute(x, u, t, params)
        for port_id, port in system.outputs.items()
    }


def gather_u(gather_sources, u_dim, signals, u, xp=np, dtype=float):
    """
    A block's local input from the compiled gather plan: its sources, concatenated.

    Each source is ``(kind, where, dim)``: an internal signal (``where`` a slice
    of the signal buffer), a nominal value, or the diagram's boundary input
    (``where`` a slice of ``u``).
    """
    if u_dim == 0:
        return xp.zeros(0, dtype=dtype)

    # NumPy: the pieces written into one buffer (the per-step hot path of a host loop)
    if xp is np:
        return gathered_on_host(gather_sources, u_dim, signals, u, dtype)

    pieces = []
    for kind, where, dim in gather_sources:
        if kind == INTERNAL_SIGNAL:
            pieces.append(signals[where])
        elif kind == NOMINAL:
            pieces.append(xp.asarray(where, dtype=dtype))
        elif kind == EXTERNAL_INPUT:
            pieces.append(xp.asarray(u[where], dtype=dtype))
        else:
            raise RuntimeError(f"Unknown source_type={kind}")

    return xp.concatenate(pieces, axis=0)


def gathered_on_host(gather_sources, u_dim, signals, u, dtype):
    """:func:`gather_u` on NumPy: each source written into its rows of one buffer."""
    local_u = np.empty(u_dim, dtype=dtype)
    start = 0
    for kind, where, dim in gather_sources:
        stop = start + dim
        if kind == INTERNAL_SIGNAL:
            local_u[start:stop] = signals[where]
        elif kind == NOMINAL:
            local_u[start:stop] = where
        elif kind == EXTERNAL_INPUT:
            local_u[start:stop] = u[where]
        else:
            raise RuntimeError(f"Unknown source_type={kind}")
        start = stop

    return local_u


def assigned(xp, buffer, where, value):
    """``buffer`` with ``buffer[where] = value``: in place on NumPy, a new array on JAX."""
    if xp is np:
        buffer[where] = value
        return buffer

    return buffer.at[where].set(value)


def added(xp, buffer, where, value):
    """``buffer`` with ``value`` added at ``where``: in place on NumPy, a new array on JAX."""
    if xp is np:
        buffer[where] += value
        return buffer

    return buffer.at[where].add(value)


def require_jax_evaluator(evaluator, verb):
    """Refuse a JAX-only verb on a NumPy evaluator."""
    if evaluator.backend != BACKEND_JAX:
        raise TypeError(f"{verb} needs a JAX evaluator: compile(backend='jax')")


def check_jax_compatible(func, label, dummy_x, dummy_u, dummy_t, params, jax):
    """Refuse a block function that does not trace (``jax.make_jaxpr``)."""
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
