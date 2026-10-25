"""Linearization of a ``System`` about an operating point: the Jacobians ``(A, B, C, D)`` and an ``LTISystem``."""

from __future__ import annotations

import numpy as np

from minilink.analysis.derivatives import compiled, jacobian, operating_point
from minilink.core.diagram import DiagramSystem
from minilink.core.roles import command_port
from minilink.core.system import System
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.graphical.control import style

###############################################################################
# Public API
###############################################################################


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
        Operating point; default the nominal state and the nominal port values.
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
    require_own_point(sys, x_bar, u_bar)
    n = sys.n
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    require_state_or_output(sys, outputs)

    point = (x_bar, u_bar, t, params)
    at = dict(method=method, eps=eps)

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
    the nominal state like every tool of the analysis family. The model
    carries the operating point it was taken at (``xbar``, ``ubar`` for the
    selected inputs, ``ybar`` for the selected outputs), so a design made on
    it (``lqr``, ``kalman``) knows where it applies.
    """
    # The Jacobians at the operating point
    A, B, C, D = linearize_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # The point the model carries: the one given, else the nominal, on the
    # selected channels
    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    ubar = point_inputs(sys, u_bar, inputs)
    ybar = point_outputs(sys, outputs, x_bar, u_bar, t, params)

    # The model, its state the deviation from the point
    lti = LTISystem(
        A,
        B,
        C,
        D,
        name=f"Linearized {sys.name}",
        xbar=x_bar,
        ubar=ubar,
        ybar=ybar,
    )
    lti.state.labels = [f"Delta {label}" for label in sys.state.labels]

    return lti


def design_model(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    method="auto",
    eps=1e-6,
    measured=False,
):
    """The linear model a design verb reads: ``sys`` itself when it is an ``LTISystem``, else ``sys`` linearized about its command port.

    An ``LTISystem`` carries the operating point it was taken at, so a point
    given here is refused: linearize the plant at ``x_bar`` instead. Any other
    system is linearized at ``(x_bar, u_bar)``, the nominal state and inputs
    by default, with ``B`` over the command port alone (``u``, else the one
    input that is not a disturbance ``d`` or a noise ``n``; every other input
    is held at its nominal value) and ``C`` over the measured output ``y``
    alone, so a plant with noise ports gives a gain over its command and a box
    exposing its state is measured through ``y``. ``measured=True`` (the
    observer designs) refuses a plant without ``y`` rather than reading its
    state as the measurement.
    """
    require_design_system(sys)
    require_own_point(sys, x_bar, u_bar)

    # A linear model is read as is, with the operating point it carries
    if isinstance(sys, LTISystem):
        return sys

    # An observer design reads the measured output y, never the state
    if measured:
        require_measured_output(sys)

    # B over the command port: "u" alone names every input stacked, so a plant
    # with several inputs picks the components of the port named u
    command = command_port(sys)
    if command == "u" and len(sys.inputs) > 1:
        columns = sys.get_input_port_slice("u")
        command = [("u", i) for i in range(columns.start, columns.stop)]

    # C over the measured output alone, not every edge port of a box
    measurement = "y" if "y" in sys.outputs else None

    return linearize(
        sys,
        x_bar,
        u_bar,
        t,
        params,
        of=measurement,
        wrt=command,
        method=method,
        eps=eps,
    )


###############################################################################
# Internal machinery
###############################################################################


def require_own_point(sys, x_bar, u_bar):
    """Refuse a point given with a linear model: an ``LTISystem`` carries the one it was taken at."""
    if isinstance(sys, LTISystem) and (x_bar is not None or u_bar is not None):
        raise ValueError(
            f"{sys.name!r} is a linear model and carries its operating point "
            f"(xbar = {sys.xbar}); linearize the plant at x_bar instead of passing "
            "it here"
        )


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


def as_list(value):
    """A single selector as a one-item list, a sequence of selectors as a list."""
    if isinstance(value, (str, tuple)):
        return [value]
    return list(value)


def as_selector(item, name):
    """One selector ``(name, index)``: a port id or a wire stands for all its components."""
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


def require_state_or_output(sys, outputs):
    """Refuse a system with nothing to linearize: no state and no output port."""
    if sys.n == 0 and outputs is None:
        raise ValueError(f"{sys.name!r} has neither a state nor an output port")


def jacobian_block(sys, of, wrt, point, at):
    """The Jacobian of the ``of`` selectors with respect to the ``wrt`` selectors, stacked."""
    blocks = []
    for name, index in of:
        # The rows of one output selector, against every input selector side by side
        columns = []
        for wrt_name, wrt_index in wrt:
            J = jacobian(sys, name, wrt_name, *point, **at)
            columns.append(select_columns(select_rows(J, index), wrt_index))
        blocks.append(stack_columns(columns, rows=selector_rows(sys, name, index)))

    stacked = np.vstack(blocks)

    return stacked


def select_rows(J, index):
    """All the rows of ``J``, or the one row ``index``."""
    return J if index is None else J[[index], :]


def select_columns(J, index):
    """All the columns of ``J``, or the one column ``index``."""
    return J if index is None else J[:, [index]]


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


def stack_columns(blocks, *, rows):
    """The blocks side by side; with no block, an empty matrix of ``rows`` rows."""
    stacked = np.hstack(blocks) if blocks else np.zeros((rows, 0))

    return stacked


def point_inputs(sys, u_bar, inputs):
    """The components of ``u_bar`` the ``wrt`` selectors pick, in the order of the ``B`` columns.

    A diagram wire is an additive perturbation of a signal, so its nominal is zero.
    """
    pieces = []
    for name, index in inputs:
        if name == "u":
            values = u_bar
        elif name in sys.inputs:
            values = u_bar[sys.get_input_port_slice(name)]
        else:
            values = np.zeros(wire_dim(sys, name))
        pieces.append(values if index is None else values[[index]])

    ubar = np.concatenate(pieces) if pieces else np.zeros(0)

    return ubar


def wire_dim(sys, wire):
    """The dimension of the diagram wire ``"block:port"``."""
    block, port_id = wire.split(":", 1)
    return sys.subsystems[block].outputs[port_id].dim


def point_outputs(sys, outputs, x_bar, u_bar, t, params):
    """The outputs the ``of`` selectors pick, evaluated at the operating point (the state itself when ``None``)."""
    if outputs is None:
        return x_bar

    # The output ports, and the internal wires only when a selector names one
    evaluator = compiled(sys, "fd")
    ports = evaluator.outputs(x_bar, u_bar, t, params)
    wires = (
        evaluator.compute_internal_signals_dict(x_bar, u_bar, t)
        if any(name not in ports for name, _ in outputs)
        else {}
    )

    pieces = []
    for name, index in outputs:
        values = np.asarray(ports[name] if name in ports else wires[name], dtype=float)
        pieces.append(values if index is None else values[[index]])

    ybar = np.concatenate(pieces)

    return ybar


def require_design_system(sys):
    """Refuse a design on anything but a system, naming the gains that take arrays."""
    if not isinstance(sys, System):
        raise TypeError(
            "a design takes a system, a linear model LTISystem(A, B, C) or a plant, "
            f"not {type(sys).__name__}; on arrays the gains are lqr_gain(A, B, Q, R), "
            "place_gain(A, B, poles), kalman_gain(A, C, Q_n, R_n) and "
            "luenberger_gain(A, C, poles)"
        )


def require_measured_output(sys):
    """Refuse an observer design on a plant without a measured output ``y``."""
    if "y" not in sys.outputs:
        raise ValueError(
            f"{sys.name!r} has no measured output 'y' for an observer to read: make "
            "the sensor part of the plant box (plant >> Gain(...)), or design on "
            "linearize(plant, x_bar, of=...)"
        )


# One SISO channel, as the frequency and time responses read it


def siso_channel(sys, of, wrt):
    """Normalize the channel to ``((of_name, index), (wrt_name, index))``.

    ``of`` names the output and ``wrt`` the input: a port id means component
    0, ``(port, index)`` one component, a diagram wire ``"block:port"`` an
    internal signal. The default input is the command port (``u``, else the
    one input that is not ``d`` or ``n``), as the design verbs read it; the
    default output is the first component of ``y``. ``of_name`` is ``None``
    when the output is the state itself (no ``y`` port): the row is then
    taken from ``C = I``.
    """
    if wrt is None:
        wrt = (command_port(sys), 0)
    if of is None:
        default = output_selectors(sys, None)
        of = (None, 0) if default is None else (default[0][0], 0)
    return siso_selector(of, "of"), siso_selector(wrt, "wrt")


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


def channel_label(sys, of, wrt):
    """``"y[1] / u[0]"`` for the selected channel."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return f"{'x' if of_name is None else of_name}[{i}] / {wrt_name}[{j}]"


def channel_subtitle(sys, of, wrt):
    """``"From: u[0]  To: y[1]"`` for the selected channel, in declared labels where a port has them."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return style.channel_subtitle(
        component_label(sys, of_name, i, sys.outputs),
        component_label(sys, wrt_name, j, sys.inputs),
    )


def component_label(sys, name, index, ports):
    """A port's declared label for one component, else ``name[index]`` (``x`` for the state)."""
    # The signal the selector names: the state, a port, or a diagram wire
    signal = None
    if name is None:
        signal = sys.state
    elif name in ports:
        signal = ports[name]
    elif isinstance(sys, DiagramSystem) and ":" in name:
        block, port_id = name.split(":", 1)
        if block in sys.subsystems:
            signal = sys.subsystems[block].outputs.get(port_id)

    # Its declared label, when it has one other than the generic id[index]
    default = f"{'x' if name is None else name}[{index}]"
    if signal is None or not 0 <= index < len(signal.labels):
        return default
    declared = signal.labels[index] != f"{signal.id}[{index}]"
    return signal.labels[index] if declared else default


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

    # A state output: the row i of C = I, and of D
    if of_name is None:
        require_state_index(i, C.shape[0])
        C = C[[i], :]
        D = D[[i], :]

    return A, B, C, D


def require_state_index(i, n):
    """Refuse a state component outside ``[0, n - 1]``."""
    if i < 0 or i >= n:
        raise ValueError(f"of index must be in [0, {n - 1}] for the state.")


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    lti = linearize(Pendulum(), x_bar=[0.0, 0.0])
    print("A =\n", np.round(lti.A(), 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 4))
