"""Jacobians of a :class:`~minilink.core.system.System` at an operating point.

``jacobian(sys, "f", "x")`` is ∂f/∂x: the first argument names what is
differentiated (``"f"``, ``"step"``, an output port, or a diagram wire
``"block:port"``), the second the variable (``"x"``, ``"u"``, an input port,
``"t"``, ``"params"``, or a wire). The point follows in the order of
``f(x, u, t, params)`` with the textbook defaults, and the arithmetic runs on
the cached compiled evaluator: exact under JAX when the system traces, central
finite differences otherwise.
"""

from __future__ import annotations

import numpy as np


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
        Operating point; default ``sys.x0`` and the nominal port values.
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
    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)
    evaluator = sys.compiled_evaluator(method)
    try:
        J = evaluator.jacobian(of, wrt, eps=eps)(x_bar, u_bar, t, params)
    except Exception as exc:
        # "auto" promised a derivative, not a backend: a block that traces
        # with concrete parameters but not with traced ones (np.array built
        # from params, wrt="params") falls back to finite differences.
        if method != "auto" or evaluator.backend != "jax" or not _is_tracing_error(exc):
            raise
        J = sys.compiled_evaluator("fd").jacobian(of, wrt, eps=eps)(
            x_bar, u_bar, t, params
        )
    return as_numpy(J)


def _is_tracing_error(exc):
    """True for JAX's "concrete value needed" family of tracer errors."""
    try:
        from jax.errors import JAXTypeError
    except ImportError:  # pragma: no cover - JAX evaluators imply JAX
        return False
    return isinstance(exc, JAXTypeError)


def operating_point(sys, x_bar=None, u_bar=None, params=None):
    """Fill the family defaults: ``x0``, the nominal inputs, the live ``sys.params``."""
    x_bar = sys.x0 if x_bar is None else x_bar
    u_bar = sys.get_u_from_input_ports() if u_bar is None else u_bar
    params = sys.params if params is None else params
    return (
        np.asarray(x_bar, dtype=float).reshape(-1),
        np.asarray(u_bar, dtype=float).reshape(-1),
        params,
    )


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
