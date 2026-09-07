"""
Jacobians of compiled evaluators — ``evaluator.jacobian(of, wrt)``.

``of`` names what is differentiated: ``"f"`` (continuous evolution),
``"step"`` (discrete evolution), a boundary output port id, or on a diagram
the wire ``"block:port"`` leaving a subsystem output port. ``wrt`` names the
variable: ``"x"`` (state), ``"u"`` (every input stacked), an input port id,
``"t"`` (time), ``"params"`` (the parameter dict), or a diagram wire, which
means an additive perturbation of that signal.

The mixin resolves the two names once and hands a *probe*
``(x, u, t, params, delta) -> vector`` to the backend, which returns the
Jacobian as a callable with the signature of the parametric tier:
``(x, u, t, params)`` on continuous and static evaluators, ``(x, u, k, params)``
on step evaluators. JAX evaluators use ``jax.jacfwd`` under ``jax.jit``; NumPy
evaluators use central finite differences.
"""

from __future__ import annotations

import numpy as np

# =============================================================================
# Public API — JacobianMixin
# =============================================================================


class JacobianMixin:
    """Name resolution shared by every evaluator; backends supply ``_jac_build``."""

    def jacobian(self, of, wrt, *, eps=1e-6):
        """Return ``d(of)/d(wrt)`` as a callable ``(x, u, t, params) -> J``.

        ``J`` has shape ``(dim(of), dim(wrt))``; ``wrt="t"`` gives ``(dim(of),)``
        and ``wrt="params"`` a dict shaped like ``params`` whose leaves are
        ``(dim(of), *leaf.shape)`` (integer and boolean leaves are skipped).
        ``eps`` is the central-difference step on NumPy evaluators.
        """
        target = self._jac_resolve_of(of)
        variable = self._jac_resolve_wrt(wrt)
        return self._jac_build(target, variable, float(eps))

    # --- layout registered by each evaluator constructor ---

    def _jac_setup(self, system, kind, *, state_fn, outputs_fn, plan=None):
        """Record the port layout the resolver needs.

        ``kind`` is ``"dynamic"``, ``"static"`` or ``"step"``; ``state_fn`` is
        the parametric evolution (``f_p`` / ``step_p`` or ``None``) and
        ``outputs_fn`` the parametric output map returning a dict by port id.
        Diagrams pass their execution ``plan`` so wires become addressable.
        """
        self._jac_kind = kind
        self._jac_state_fn = state_fn
        self._jac_outputs_fn = outputs_fn
        self._jac_input_slices = {
            port_id: system.get_input_port_slice(port_id) for port_id in system.inputs
        }
        if plan is None:
            self._jac_output_ids = tuple(system.outputs)
            self._jac_wires = {}
        else:
            self._jac_output_ids = tuple(plan.external_output_slices)
            self._jac_wires = {
                f"{sys_id}:{port_id}": sl
                for (sys_id, port_id), sl in plan.output_slices.items()
            }

    # --- resolution ---

    def _jac_resolve_of(self, of):
        kind = self._jac_kind
        evolution = {"dynamic": "f", "step": "step"}.get(kind)
        if of == evolution:
            if of in self._jac_output_ids:
                raise ValueError(
                    f"of={of!r} is ambiguous: an output port is also named {of!r}; "
                    "rename the port"
                )
            return ("state", None)
        if isinstance(of, str) and of in self._jac_output_ids:
            return ("port", of)
        if isinstance(of, str) and ":" in of:
            return ("wire", self._jac_wire_slice(of))
        choices = [repr(evolution)] if evolution else []
        choices += [repr(port_id) for port_id in self._jac_output_ids]
        if self._jac_wires:
            choices.append("a wire 'block:port'")
        raise ValueError(
            f"of must be {' or '.join(choices) or 'an output port (none declared)'}; "
            f"got {of!r}"
        )

    def _jac_resolve_wrt(self, wrt):
        inputs = self._jac_input_slices
        has_state = self.n > 0
        if wrt == "x":
            if has_state and "x" in inputs:
                raise ValueError(
                    "wrt='x' is ambiguous: an input port is also named 'x'; "
                    "rename the port"
                )
            if has_state:
                return ("x", None)
            if "x" in inputs:
                return ("u", inputs["x"])
            raise ValueError("wrt='x': this system has no state")
        if wrt == "u":
            if not inputs:
                raise ValueError("wrt='u': this system has no input port")
            return ("u", None)
        if wrt in ("t", "params"):
            if wrt in inputs:
                raise ValueError(
                    f"wrt={wrt!r} is ambiguous: an input port is also named {wrt!r}; "
                    "rename the port"
                )
            if wrt == "t" and self._jac_kind == "step":
                raise ValueError(
                    "wrt='t' is not defined on a step system (k is an integer)"
                )
            return (wrt, None)
        if isinstance(wrt, str) and wrt in inputs:
            return ("u", inputs[wrt])
        if isinstance(wrt, str) and ":" in wrt:
            return ("wire", self._jac_wire_slice(wrt))
        choices = ["'x'"] if has_state else []
        choices += ["'u'", "'t'" if self._jac_kind != "step" else None, "'params'"]
        choices += [repr(port_id) for port_id in inputs]
        if self._jac_wires:
            choices.append("a wire 'block:port'")
        raise ValueError(
            f"wrt must be {' or '.join(c for c in choices if c)}; got {wrt!r}"
        )

    def _jac_wire_slice(self, name):
        if not self._jac_wires:
            raise ValueError(
                f"{name!r} names a diagram wire but this system is not a diagram"
            )
        try:
            return self._jac_wires[name]
        except KeyError:
            available = ", ".join(repr(key) for key in self._jac_wires)
            raise ValueError(f"Unknown wire {name!r}; available: {available}") from None

    # --- probes: leaves; diagram evaluators override ---

    def _jac_probe(self, target, wire=None):
        """Return ``probe(x, u, t, params, delta) -> flat vector`` for ``target``."""
        kind, key = target
        if kind == "state":
            evolution = self._jac_state_fn

            def probe(x, u, t, params, delta):
                return _ravel(evolution(x, u, t, params))

            return probe
        outputs = self._jac_outputs_fn

        def probe(x, u, t, params, delta):
            return _ravel(outputs(x, u, t, params)[key])

        return probe

    def _jac_validate_params(self, params):
        return None


# =============================================================================
# Public API — backend builders
# =============================================================================


class NumpyJacobianMixin(JacobianMixin):
    """Central finite differences on the parametric tier."""

    def _jac_build(self, target, variable, eps):
        wire = variable[1] if variable[0] == "wire" else None
        probe = self._jac_probe(target, wire)
        vkind, vsel = variable
        wire_dim = _slice_dim(wire)
        validate = self._jac_validate_params

        def jac(x, u, t, params):
            validate(params)
            x = np.asarray(x, dtype=float).reshape(-1)
            u = np.asarray(u, dtype=float).reshape(-1)
            zero = np.zeros(wire_dim)
            if vkind == "x":
                return central_difference(
                    lambda z: probe(z, u, t, params, zero), x, eps
                )
            if vkind == "u" and vsel is None:
                return central_difference(
                    lambda z: probe(x, z, t, params, zero), u, eps
                )
            if vkind == "u":

                def embedded(z):
                    full = u.copy()
                    full[vsel] = z
                    return probe(x, full, t, params, zero)

                return central_difference(embedded, u[vsel], eps)
            if vkind == "wire":
                return central_difference(
                    lambda z: probe(x, u, t, params, z), zero, eps
                )
            if vkind == "t":
                t = float(t)
                plus = probe(x, u, t + eps, params, zero)
                minus = probe(x, u, t - eps, params, zero)
                return (plus - minus) / (2.0 * eps)
            # params: one column per element of every float leaf
            result = {}
            for path, leaf in _float_leaves(params):
                leaf = np.asarray(leaf, dtype=float)

                def perturbed(z, path=path, leaf=leaf):
                    value = z.reshape(leaf.shape) if leaf.shape else float(z[0])
                    return probe(x, u, t, replace_leaf(params, path, value), zero)

                columns = central_difference(perturbed, leaf.reshape(-1), eps)
                set_leaf(result, path, columns.reshape(columns.shape[0], *leaf.shape))
            return result

        return jac


class JaxJacobianMixin(JacobianMixin):
    """``jax.jacfwd`` on the trace tier, evaluated eagerly.

    The returned callable is not jitted: ``params`` stay concrete Python
    values, so blocks that build constant matrices from them (``np.array``
    in an ``A(t, params)``) differentiate exactly with respect to ``x``,
    ``u``, ``t`` and wires, and nothing recompiles when a parameter changes.
    Wrap the callable in ``jax.jit`` yourself for hot loops over a block
    written with ``xp = array_module(...)`` all the way through.
    """

    def _jac_build(self, target, variable, eps):
        jax = self.jax
        jnp = self.jnp
        wire = variable[1] if variable[0] == "wire" else None
        probe = self._jac_probe(target, wire)
        vkind, vsel = variable
        wire_dim = _slice_dim(wire)
        continuous = self._jac_kind != "step"
        validate = self._jac_validate_params

        def jac(x, u, t, params):
            x = jnp.asarray(x, dtype=float).reshape(-1)
            u = jnp.asarray(u, dtype=float).reshape(-1)
            if continuous:
                t = jnp.asarray(t, dtype=float)
            zero = jnp.zeros(wire_dim)
            if vkind == "x":
                return jax.jacfwd(lambda z: probe(z, u, t, params, zero))(x)
            if vkind == "u" and vsel is None:
                return jax.jacfwd(lambda z: probe(x, z, t, params, zero))(u)
            if vkind == "u":
                return jax.jacfwd(
                    lambda z: probe(x, u.at[vsel].set(z), t, params, zero)
                )(u[vsel])
            if vkind == "wire":
                return jax.jacfwd(lambda z: probe(x, u, t, params, z))(zero)
            if vkind == "t":
                return jax.jacfwd(lambda z: probe(x, u, z, params, zero))(t)
            # params: differentiate the float leaves as one list, keep the rest fixed
            leaves = _float_leaves(params)
            paths = [path for path, leaf in leaves]
            values = [jnp.asarray(leaf, dtype=float) for path, leaf in leaves]

            def with_leaves(new_values):
                tree = params
                for path, value in zip(paths, new_values):
                    tree = replace_leaf(tree, path, value)
                return probe(x, u, t, tree, zero)

            blocks = jax.jacfwd(with_leaves)(values)
            result = {}
            for path, block in zip(paths, blocks):
                set_leaf(result, path, block)
            return result

        def call(x, u, t, params):
            validate(params)
            return jac(x, u, t, params)

        return call


# =============================================================================
# Internal machinery
# =============================================================================


def central_difference(g, z, eps):
    """Central-difference Jacobian of ``g`` at ``z``: column ``i`` is ``dg/dz_i``."""
    z = np.asarray(z, dtype=float).reshape(-1)
    n = z.size
    p = np.asarray(g(z), dtype=float).reshape(-1).size
    J = np.zeros((p, n))
    for i in range(n):
        dz = np.zeros(n)
        dz[i] = eps
        plus = np.asarray(g(z + dz), dtype=float).reshape(-1)
        minus = np.asarray(g(z - dz), dtype=float).reshape(-1)
        J[:, i] = (plus - minus) / (2.0 * eps)
    return J


def flatten_params(params, prefix=()):
    """Yield ``(path, leaf)`` pairs of a nested parameter dict (depth-first)."""
    if params is None:
        return
    if not isinstance(params, dict):
        raise TypeError(
            f"params must be a dict (nested for diagrams), got {type(params).__name__}"
        )
    for key, value in params.items():
        if isinstance(value, dict):
            yield from flatten_params(value, prefix + (key,))
        else:
            yield prefix + (key,), value


def replace_leaf(params, path, value):
    """Copy of ``params`` with the leaf at ``path`` replaced (containers copied on the path)."""
    key, *rest = path
    tree = dict(params)
    tree[key] = value if not rest else replace_leaf(params[key], rest, value)
    return tree


def set_leaf(tree, path, value):
    """Insert ``value`` at ``path`` in ``tree``, creating nested dicts."""
    for key in path[:-1]:
        tree = tree.setdefault(key, {})
    tree[path[-1]] = value


def _float_leaves(params):
    """``(path, leaf)`` pairs of the float leaves; raise when there is nothing to differentiate."""
    leaves = [
        (path, leaf) for path, leaf in flatten_params(params) if _is_float_leaf(leaf)
    ]
    if not leaves:
        raise ValueError(
            f"wrt='params' needs a params dict with float leaves; got {params!r}"
        )
    return leaves


def _is_float_leaf(leaf):
    dtype = getattr(leaf, "dtype", None)
    if dtype is not None:
        return np.issubdtype(np.dtype(dtype), np.floating)
    if isinstance(leaf, bool):
        return False
    if isinstance(leaf, (int, float)):
        return isinstance(leaf, float)
    return np.issubdtype(np.asarray(leaf).dtype, np.floating)


def _ravel(value):
    """Flatten NumPy or JAX arrays alike (no host copy for tracers)."""
    if hasattr(value, "reshape"):
        return value.reshape(-1)
    return np.asarray(value, dtype=float).reshape(-1)


def _slice_dim(sl):
    return 0 if sl is None else sl.stop - sl.start
