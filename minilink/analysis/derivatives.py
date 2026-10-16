"""Jacobians of a :class:`~minilink.core.system.System` at an operating point."""

from __future__ import annotations

import numpy as np

###############################################################################
# Public API
###############################################################################


def jacobian(
    sys,
    of,
    wrt,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    method="auto",
    eps=1e-6,
):
    """Return ``d(of)/d(wrt)`` of ``sys`` at ``(x_bar, u_bar, t, params)``.

    ``jacobian(sys, "f", "x")`` is ∂f/∂x: the first argument names what is
    differentiated (``"f"``, ``"step"``, an output port, or a diagram wire
    ``"block:port"``), the second the variable (``"x"``, ``"u"``, an input port,
    ``"t"``, ``"params"``, or a wire). The point follows in the order of
    ``f(x, u, t, params)`` with the textbook defaults, and each call compiles its
    own evaluator (about a millisecond): exact under JAX when the system traces,
    central finite differences otherwise. Nothing is stored on the system; keep
    ``evaluator.jacobian(of, wrt)`` when a loop needs the callable.

    Parameters
    ----------
    sys : System
        Leaf, diagram, static block, or step system.
    of : str
        ``"f"`` (``"step"`` on step systems), an output port id, or a diagram
        wire ``"block:port"``.
    wrt : str
        ``"x"``, ``"u"`` (every input stacked), an input port id, ``"t"``,
        ``"params"``, or a diagram wire (additive perturbation of that signal).
    x_bar, u_bar : array-like, optional
        Operating point; default the nominal state and the nominal port values.
    t : float, optional
        Time (the step index ``k`` on step systems).
    params : dict, optional
        Parameter set; default the live ``sys.params``.
    method : {"auto", "fd", "jax"}, optional
        ``"auto"`` is exact under JAX when it is installed and the system
        traces, finite differences otherwise; ``"jax"`` raises instead of
        falling back.
    eps : float, optional
        Central-difference step.

    Returns
    -------
    ndarray or dict
        Shape ``(dim(of), dim(wrt))``; ``(dim(of),)`` for ``wrt="t"``; for
        ``wrt="params"`` a dict shaped like ``params`` with leaves
        ``(dim(of), *leaf.shape)`` (float leaves only). A block whose ``f``
        reads instance attributes instead of its ``params`` argument reports
        zero sensitivity, like every parametric tier.
    """
    # The operating point, the family defaults filled in
    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)

    # d(of)/d(wrt) at the point: exact under JAX, central differences otherwise
    J = jacobian_with_fd_fallback(sys, of, wrt, (x_bar, u_bar, t, params), method, eps)

    return as_numpy(J)


###############################################################################
# Internal machinery
###############################################################################


def operating_point(sys, x_bar=None, u_bar=None, params=None):
    """Fill the family defaults: the nominal state and inputs, ``sys.params`` with every random block at its mean.

    The operating point is the system's declared nominal, ``state.nominal_value``
    and the input ports' nominal values; ``x0`` is the start of a simulation
    and plays no part here. A point of the wrong size is refused by name, so
    it never surfaces as a broadcast error inside the evaluator.
    """
    nominal_u = sys.get_u_from_input_ports()

    # The nominal point and the mean parameters, where none is given
    x_bar = sys.state.nominal_value if x_bar is None else x_bar
    u_bar = nominal_u if u_bar is None else u_bar
    params = sys.realize(None) if params is None else params

    # Flat float vectors, of the system's sizes
    x_bar = np.asarray(x_bar, dtype=float).reshape(-1)
    u_bar = np.asarray(u_bar, dtype=float).reshape(-1)
    require_point_size(sys, x_bar, u_bar, nominal_u)

    return x_bar, u_bar, params


def require_point_size(sys, x_bar, u_bar, nominal_u):
    """Refuse a point whose state or inputs do not match the system's sizes, by name."""
    if x_bar.size != sys.n:
        raise ValueError(
            f"{sys.name!r}: x_bar has {x_bar.size} entries, the system has {sys.n} states"
        )
    if u_bar.size != nominal_u.size:
        raise ValueError(
            f"{sys.name!r}: u_bar has {u_bar.size} entries, the inputs stack to "
            f"{nominal_u.size}"
        )


def jacobian_with_fd_fallback(sys, of, wrt, point, method, eps):
    """The Jacobian at ``point`` from the evaluator ``method`` picks, or by finite differences.

    ``"auto"`` promised a derivative, not a backend: a block that traces with
    concrete parameters but not with traced ones (``np.array`` built from
    ``params``, ``wrt="params"``) falls back to finite differences.
    """
    evaluator = compiled(sys, method)
    try:
        J = evaluator.jacobian(of, wrt, eps=eps)(*point)
    except Exception as exc:
        if method != "auto" or evaluator.backend != "jax" or not is_tracing_error(exc):
            raise
        J = compiled(sys, "fd").jacobian(of, wrt, eps=eps)(*point)

    return J


def compiled(sys, method="auto"):
    """The evaluator behind one derivative call.

    ``"auto"`` prefers JAX when it is installed and the system traces,
    ``"fd"`` compiles with NumPy, ``"jax"`` compiles with JAX and raises when
    the system does not trace.
    """
    # Lazy import: the compiler loads on first use, not with the analysis verbs
    from minilink.core.compile import compiler

    key = str(method).strip().lower()
    if key == "auto":
        return compiler.compile_auto(sys)[1]
    if key in ("fd", "jax"):
        return compiler.compile(sys, backend="numpy" if key == "fd" else "jax")
    raise ValueError(f"method must be 'auto', 'fd' or 'jax'; got {method!r}")


def is_tracing_error(exc):
    """True for JAX's "concrete value needed" family of tracer errors."""
    # The exception type only: require_jax() would also apply the x64 policy
    try:
        from jax.errors import JAXTypeError
    except ImportError:  # pragma: no cover - JAX evaluators imply JAX
        return False
    return isinstance(exc, JAXTypeError)


def as_numpy(value):
    """NumPy float arrays all the way down (dict trees included)."""
    if isinstance(value, dict):
        return {key: as_numpy(leaf) for key, leaf in value.items()}
    return np.asarray(value, dtype=float)


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    pendulum = Pendulum()
    print("df/dx =\n", np.round(jacobian(pendulum, "f", "x", [0.3, 0.0]), 4))
    print("df/du =\n", np.round(jacobian(pendulum, "f", "u", [0.3, 0.0]), 4))
