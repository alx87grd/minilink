"""Linearization of a ``System`` about an operating point: the Jacobians ``(A, B, C, D)`` and an ``LTISystem``."""

from __future__ import annotations

import numpy as np

from minilink.analysis.derivatives import jacobian
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.graphical.control import style


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
    point = (x_bar, u_bar, t, params)
    n = sys.n
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    if n == 0 and outputs is None:
        raise ValueError(f"{sys.name!r} has neither a state nor an output port")

    # A = ∂f/∂x and B = ∂f/∂u at the operating point; a static block has no state
    A = jacobian(sys, "f", "x", *point, **at) if n > 0 else np.zeros((0, 0))
    B = jacobian_block(sys, [("f", None)], inputs, point, at) if n > 0 else None

    # C = ∂h/∂x and D = ∂h/∂u; with no output port the output is the state: C = I, D = 0
    if outputs is None:
        C = np.eye(n)
        D = np.zeros((n, B.shape[1]))
    else:
        C = jacobian_block(sys, outputs, [("x", None)], point, at) if n > 0 else None
        D = jacobian_block(sys, outputs, inputs, point, at)

    # A static block's channel is its feedthrough D alone
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


# =============================================================================
# Internal machinery
# =============================================================================


def input_selectors(sys, wrt):
    """Normalize ``wrt`` to a list of ``(name, index)``; default every input stacked."""
    if wrt is None:
        return [("u", None)] if sys.inputs else []
    return [as_selector(item, "wrt") for item in as_list(wrt)]


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
    return [as_selector(item, "of") for item in as_list(of)]


def siso_channel(sys, of, wrt):
    """Normalize the channel to ``((of_name, index), (wrt_name, index))``.

    ``of`` names the output and ``wrt`` the input: a port id means component
    0, ``(port, index)`` one component, a diagram wire ``"block:port"`` an
    internal signal. ``of_name`` is ``None`` when the output is the state
    itself (no ``y`` port): the row is then taken from ``C = I``.
    """
    if wrt is None:
        if not sys.inputs:
            raise ValueError("Frequency analysis requires at least one input port.")
        wrt = (next(iter(sys.inputs)), 0)
    if of is None:
        default = output_selectors(sys, None)
        of = (None, 0) if default is None else (default[0][0], 0)
    return siso_selector(of, "of"), siso_selector(wrt, "wrt")


def channel_label(sys, of, wrt):
    """``"y[1] / u[0]"`` for the selected channel."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return f"{'x' if of_name is None else of_name}[{i}] / {wrt_name}[{j}]"


def channel_subtitle(sys, of, wrt):
    """``"From: u[0]  To: y[1]"`` for the selected channel."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return style.channel_subtitle("x" if of_name is None else of_name, i, wrt_name, j)


def siso_matrices(sys, x_bar, u_bar, t, params, *, of, wrt, method, eps):
    """``A, b, c, d`` of the selected channel (``b`` a column, ``c`` a row)."""
    (of_name, i), channel_in = siso_channel(sys, of, wrt)
    A, B, C, D = linearize_matrices(
        sys,
        x_bar,
        u_bar,
        t,
        params,
        of=None if of_name is None else [(of_name, i)],
        wrt=[channel_in],
        method=method,
        eps=eps,
    )
    if of_name is None:  # state output: pick the component of C = I
        if i < 0 or i >= C.shape[0]:
            raise ValueError(
                f"of index must be in [0, {C.shape[0] - 1}] for the state."
            )
        C, D = C[[i], :], D[[i], :]
    return A, B, C, D


def siso_selector(selector, name):
    """One component ``(name, index)`` from a port id or a ``(port, index)`` pair."""
    if isinstance(selector, str):
        return (selector, 0)
    if (
        isinstance(selector, tuple)
        and len(selector) == 2
        and (selector[0] is None or isinstance(selector[0], str))
        and isinstance(selector[1], (int, np.integer))
        and not isinstance(selector[1], bool)
    ):
        return (selector[0], int(selector[1]))
    raise TypeError(
        f"{name} names one channel: a port id, a diagram wire 'block:port', or "
        f"(selector, index); got {selector!r}"
    )


def as_selector(item, name):
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


def as_list(value):
    if isinstance(value, (str, tuple)):
        return [value]
    return list(value)


def select_rows(J, index):
    return J if index is None else J[[index], :]


def select_columns(J, index):
    return J if index is None else J[:, [index]]


def stack_columns(blocks, *, rows):
    return np.hstack(blocks) if blocks else np.zeros((rows, 0))


def jacobian_block(sys, of, wrt, point, at):
    """The Jacobian of the ``of`` selectors with respect to the ``wrt`` selectors, stacked."""
    return np.vstack(
        [
            stack_columns(
                [
                    select_columns(
                        select_rows(jacobian(sys, name, wrt_name, *point, **at), index),
                        wrt_index,
                    )
                    for wrt_name, wrt_index in wrt
                ],
                rows=selector_rows(sys, name, index),
            )
            for name, index in of
        ]
    )


def selector_rows(sys, name, index):
    """Number of rows one output selector (or the state equation ``"f"``) contributes."""
    if index is not None:
        return 1
    if name == "f":
        return sys.n
    if name in sys.outputs:
        return sys.outputs[name].dim
    block, port = name.split(":", 1)
    return sys.subsystems[block].outputs[port].dim


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    lti = linearize(Pendulum(), x_bar=[0.0, 0.0])
    print("A =\n", np.round(lti.A(), 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 4))
