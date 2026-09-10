"""Equilibrium linearization of :class:`~minilink.core.system.DynamicSystem` models.

``linearize_matrices`` returns the first-order matrices

    dDelta x = A Delta x + B Delta u
    Delta y  = C Delta x + D Delta u

about an operating point ``(x_bar, u_bar)``; ``linearize`` wraps them in an
:class:`~minilink.dynamics.abstraction.state_space.LTISystem`. Both are
compositions of :func:`~minilink.analysis.derivatives.jacobian`, so the
``of`` / ``wrt`` selectors, the ``method`` and the ``eps`` keywords mean the
same thing everywhere in the analysis family.
"""

from __future__ import annotations

import numpy as np

from minilink.analysis.derivatives import jacobian
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.abstraction.state_space import LTISystem


def linearize_matrices(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    method="auto",
    eps=1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return ``A, B, C, D``.

    Parameters
    ----------
    sys : DynamicSystem
        Leaf or diagram whose ``f`` defines the dynamics.
    x_bar, u_bar : array-like, optional
        Operating point; default ``sys.x0`` and the nominal port values.
    t : float, optional
        Time at which the Jacobians are evaluated.
    params : dict, optional
        Parameter set; default the live ``sys.params``.
    of : selector or list of selectors, optional
        Rows of ``C`` and ``D``: an output port id, a diagram wire
        ``"block:port"``, or ``(selector, index)`` for one component. Default:
        every boundary output of a diagram, the ``y`` port of a leaf, or the
        full state (``C = I``) when there is no ``y`` port.
    wrt : selector or list of selectors, optional
        Columns of ``B`` and ``D``: an input port id, a diagram wire, or
        ``(selector, index)``. Default: every input port, stacked.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.
    """
    at = dict(method=method, eps=eps)
    n = sys.n
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    if n == 0 and outputs is None:
        raise ValueError(f"{sys.name!r} has neither a state nor an output port")

    if n > 0:
        A = jacobian(sys, "f", "x", x_bar, u_bar, t, params, **at)
        B = _stack_columns(
            [
                _columns(jacobian(sys, "f", name, x_bar, u_bar, t, params, **at), index)
                for name, index in inputs
            ],
            rows=n,
        )
        if outputs is None:  # no y port: the output is the state itself
            return A, B, np.eye(n), np.zeros((n, B.shape[1]))
        C = np.vstack(
            [
                _rows(jacobian(sys, name, "x", x_bar, u_bar, t, params, **at), index)
                for name, index in outputs
            ]
        )
    else:  # static block: no state, the channel is the feedthrough D
        A = np.zeros((0, 0))

    D = np.vstack(
        [
            _stack_columns(
                [
                    _columns(
                        _rows(
                            jacobian(
                                sys, name, wrt_name, x_bar, u_bar, t, params, **at
                            ),
                            index,
                        ),
                        wrt_index,
                    )
                    for wrt_name, wrt_index in inputs
                ],
                rows=_rows_of(sys, name, index),
            )
            for name, index in outputs
        ]
    )
    if n == 0:
        B = np.zeros((0, D.shape[1]))
        C = np.zeros((D.shape[0], 0))
    return A, B, C, D


def linearize(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    method="auto",
    eps=1e-6,
):
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return an ``LTISystem``.

    Same arguments as :func:`linearize_matrices`; ``x_bar=None`` defaults to
    ``sys.x0`` like every tool of the analysis family.
    """
    A, B, C, D = linearize_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )
    lti = LTISystem(A, B, C, D, name=f"Linearized {sys.name}")
    lti.state.labels = [f"Delta {label}" for label in sys.state.labels]
    return lti


# Selector helpers shared with the frequency tools


def input_selectors(sys, wrt):
    """Normalize ``wrt`` to a list of ``(name, index)``; default every input stacked."""
    if wrt is None:
        return [("u", None)] if sys.inputs else []
    return [_selector(item, "wrt") for item in _as_list(wrt)]


def output_selectors(sys, of):
    """Normalize ``of`` to a list of ``(name, index)``; ``None`` means the state.

    Default: every boundary output of a diagram, the ``y`` port of a leaf,
    the command ``u`` of a compensator, every output of a static block, or
    the state when nothing applies.
    """
    if of is None:
        if isinstance(sys, DiagramSystem) and sys.outputs:
            return [(port_id, None) for port_id in sys.outputs]
        if "y" in sys.outputs:
            return [("y", None)]
        if "u" in sys.outputs:
            return [("u", None)]
        if sys.n == 0 and sys.outputs:
            return [(port_id, None) for port_id in sys.outputs]
        return None
    return [_selector(item, "of") for item in _as_list(of)]


def _selector(item, name):
    if isinstance(item, str):
        return (item, None)
    if (
        isinstance(item, tuple)
        and len(item) == 2
        and isinstance(item[0], str)
        and isinstance(item[1], (int, np.integer))
        and not isinstance(item[1], bool)
    ):
        return (item[0], int(item[1]))
    raise TypeError(
        f"{name} selectors are port ids, diagram wires 'block:port', or "
        f"(selector, index) tuples with an integer index; got {item!r}"
    )


def _as_list(value):
    if isinstance(value, (str, tuple)):
        return [value]
    return list(value)


def _rows(J, index):
    return J if index is None else J[[index], :]


def _columns(J, index):
    return J if index is None else J[:, [index]]


def _stack_columns(blocks, *, rows):
    return np.hstack(blocks) if blocks else np.zeros((rows, 0))


def _rows_of(sys, name, index):
    """Number of rows one output selector contributes."""
    if index is not None:
        return 1
    if name in sys.outputs:
        return sys.outputs[name].dim
    block, port = name.split(":", 1)
    return sys.subsystems[block].outputs[port].dim


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    lti = linearize(Pendulum(), x_bar=[0.0, 0.0])
    print("A =\n", np.round(lti.A(), 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 4))
