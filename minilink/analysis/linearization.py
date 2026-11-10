"""Linearization of a ``System`` about an operating point: the Jacobians ``(A, B, C, D)`` and a ``LinearizedSystem``."""

from __future__ import annotations

import warnings

import numpy as np

from minilink.analysis.derivatives import compiled, jacobian, operating_point
from minilink.core.diagram import DiagramSystem
from minilink.core.discrete.step_system import StepSystem
from minilink.core.options import caller_stacklevel
from minilink.core.roles import command_port
from minilink.core.signals import InputPort
from minilink.core.system import System
from minilink.dynamics.abstraction.state_space import (
    DiscreteLTISystem,
    LinearizedStepSystem,
    LinearizedSystem,
    LTISystem,
)
from minilink.graphical.control import style

# The linear models a design reads as they are, and those of them taken about a point
LINEARIZATIONS = (LinearizedSystem, LinearizedStepSystem)
LINEAR_MODELS = (LTISystem, DiscreteLTISystem, *LINEARIZATIONS)

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

    On a continuous system ``A = ∂f/∂x``; on a step or sampled system (a tick
    law, ``block % dt``, a sampled loop) the difference equation's
    ``A_d = ∂step/∂x``, ``t`` its tick ``k``: the z-domain model.

    Parameters
    ----------
    sys : DynamicSystem or StepSystem
        Leaf or diagram whose ``f`` (``step``) defines the dynamics.
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
    n = sys.n
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    require_state_or_output(sys, outputs)

    point = (x_bar, u_bar, t, params)
    at = dict(method=method, eps=eps)

    # A linearization's drift is not in (A, B, C, D): one off an equilibrium is
    # announced. At its own point it is read as stored, exactly on every method
    if isinstance(sys, LINEARIZATIONS):
        warn_off_equilibrium(sys, x_bar, u_bar, t, params)
        stored = stored_matrices(sys, x_bar, u_bar, t, params, inputs, outputs)
        if stored is not None:
            return stored

    # A = ∂f/∂x and B = ∂f/∂u at the operating point (∂step/∂x and ∂step/∂u on a
    # step system); a static block has no state
    dynamics = "step" if isinstance(sys, StepSystem) else "f"
    A = jacobian(sys, dynamics, "x", *point, **at) if n > 0 else np.zeros((0, 0))
    B = jacobian_block(sys, [(dynamics, None)], inputs, point, at) if n > 0 else None

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
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return a ``LinearizedSystem``.

    Same arguments as :func:`linearize_matrices`; ``x_bar=None`` defaults to
    the nominal state like every tool of the analysis family. The model is
    ``ẋ = f(x̄, ū) + A (x − x̄) + B (u − ū)``, ``y = h(x̄, ū) + C (x − x̄) +
    D (u − ū)`` in the system's own coordinates: its state, inputs (the
    selected ones) and output (the selected rows) carry the system's labels
    and units, its nominal point is ``(x̄, ū)``, its ``x0`` the system's, so
    a design made on it (``lqr``, ``kalman``) regulates to ``x̄`` and a run of
    it is the linear prediction of the system's. A step or sampled system
    gives a :class:`~minilink.dynamics.abstraction.state_space.LinearizedStepSystem`,
    ``x_{k+1} = F(x̄, ū) + A_d (x_k − x̄) + B_d (u_k − ū)``, with the period it
    runs at (``None`` for a tick law): its poles sit in ``z``, stable inside
    the unit circle. ``model.deviation_model()`` is the pure linear model in
    the deviations from the point.
    """
    # The Jacobians at the operating point
    A, B, C, D = linearize_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # The point: the one given, else the nominal; the selected inputs and outputs there
    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    u_selected = point_inputs(sys, u_bar, inputs)
    y_bar = point_outputs(sys, outputs, x_bar, u_bar, t, params)

    # The model about the point: the drift f(x̄, ū) on a continuous system, the
    # next state F(x̄, ū) on a step system, with its period
    point = dict(x_bar=x_bar, u_bar=u_selected, y_bar=y_bar)
    name = f"Linearized {sys.name}"
    if isinstance(sys, StepSystem):
        x_next_bar = np.asarray(sys.step(x_bar, u_bar, t, params), dtype=float)
        model = LinearizedStepSystem(
            A, B, C, D, x_next_bar=x_next_bar, period=period_of(sys), name=name, **point
        )
    else:
        dx_bar = np.asarray(sys.f(x_bar, u_bar, t, params), dtype=float)
        model = LinearizedSystem(A, B, C, D, dx_bar=dx_bar, name=name, **point)

    # The system's own signals: its labels and units, its ports, its start
    copy_signals(model, sys, inputs, outputs, u_selected)

    return model


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
    """The linear model a design verb reads: ``sys`` itself when it is a linear model and no point is given, else ``sys`` linearized about its command port.

    A linear model (an ``LTISystem``, a ``LinearizedSystem`` and their
    discrete twins) is read as is, with its operating point (the origin of an
    ``LTISystem``, the ``x̄``, ``ū`` a linearization carries). Any other system,
    or a linear model given a point, is linearized at ``(x_bar, u_bar)``, the
    nominal state and inputs by default, with ``B`` over the command port
    alone (``u``, else the one input that is not a disturbance ``d`` or a
    noise ``n``; every other input is held at its nominal value) and ``C``
    over the measured output ``y`` alone, so a plant with noise ports gives a
    gain over its command and a box exposing its state is measured through
    ``y``. ``measured=True`` (the observer designs) refuses a plant without
    ``y`` rather than reading its state as the measurement.
    """
    require_design_system(sys)

    # A linear model is read as is, with the operating point it carries; the
    # drift of one taken off an equilibrium is announced
    if isinstance(sys, LINEAR_MODELS) and x_bar is None and u_bar is None:
        warn_off_equilibrium(sys, None, None, t, params)
        return sys

    # An observer design reads the measured output y, never the state
    if measured:
        require_measured_output(sys)

    # The linear model drops f(x̄, ū): a point the plant does not rest at is announced
    warn_off_equilibrium(sys, x_bar, u_bar, t, params)

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


def stored_matrices(sys, x_bar, u_bar, t, params, inputs, outputs):
    """A linearization's ``(A, B, C, D)`` on the selected rows and columns, read at its own point; ``None`` at another point or on a diagram wire."""
    if x_bar is not None or u_bar is not None:
        return None
    columns = selected_columns(sys, inputs)
    rows = selected_rows(sys, outputs)
    if columns is None or rows is None:
        return None
    n = sys.n

    A = np.asarray(sys.A(t, params), dtype=float)
    B = np.asarray(sys.B(t, params), dtype=float)[:, columns]

    # The rows of y = ȳ + C (x − x̄) + D (u − ū), and of the state output x (C = I, D = 0)
    C_y = np.asarray(sys.C(t, params), dtype=float)
    D_y = np.asarray(sys.D(t, params), dtype=float)[:, columns]
    C_x = np.eye(n)
    D_x = np.zeros((n, len(columns)))
    C = np.vstack([C_y[i] if port == "y" else C_x[i] for port, i in rows]).reshape(
        -1, n
    )
    D = np.vstack([D_y[i] if port == "y" else D_x[i] for port, i in rows]).reshape(
        -1, len(columns)
    )

    return A, B, C, D


def selected_columns(sys, inputs):
    """The entries of the stacked ``u`` the ``wrt`` selectors pick; ``None`` for a diagram wire."""
    columns = []
    for name, index in inputs:
        if name == "u":
            entries = range(sys.m)
        elif name in sys.inputs:
            entries = range(sys.m)[sys.get_input_port_slice(name)]
        else:
            return None
        columns += list(entries) if index is None else [entries[index]]
    return columns


def selected_rows(sys, outputs):
    """``(port, entry)`` of each row the ``of`` selectors pick, on the ports ``y`` and ``x``; ``None`` otherwise."""
    if outputs is None:
        outputs = [("x", None)]
    rows = []
    for name, index in outputs:
        if name not in ("x", "y") or name not in sys.outputs:
            return None
        entries = range(sys.outputs[name].dim)
        rows += [(name, i) for i in (entries if index is None else [entries[index]])]
    return rows


def copy_signals(model, sys, inputs, outputs, u_bar):
    """Give ``model`` the signals of ``sys``: the state's labels, units and ``x0``, the selected input ports at ``ū``, the selected outputs' labels and units on ``y``."""
    # The state, and its start
    model.state.labels = list(sys.state.labels)
    model.state.units = list(sys.state.units)
    model.x0 = np.asarray(sys.x0, dtype=float).reshape(-1).copy()
    if "x" in model.outputs:
        model.outputs["x"].labels = list(sys.state.labels)
        model.outputs["x"].units = list(sys.state.units)

    # The input ports the B columns select, each at its share of ū; a selection
    # that is not a set of whole ports keeps the one port u
    ports = selected_input_ports(sys, inputs, u_bar)
    if ports is not None:
        output = model.outputs["y"]
        feedthrough = bool(output.dependencies)
        model.inputs = {}
        for port in ports:
            model.inputs[port.id] = port
        output.dependencies = tuple(model.inputs) if feedthrough else ()

    # The output rows the C rows select
    labels, units = selected_output_signal(sys, outputs)
    model.outputs["y"].labels = labels
    model.outputs["y"].units = units


def selected_input_ports(sys, inputs, u_bar):
    """The input ports of ``sys`` the ``wrt`` selectors pick, in the order of the ``B`` columns, at ``ū``; ``None`` when a selector is not on a port."""
    # Each B column as (port id, component of that port)
    columns = []
    for name, index in inputs:
        if name == "u":
            stacked = range(sys.m) if index is None else [index]
            columns += [port_component_of(sys, i) for i in stacked]
        elif name in sys.inputs:
            components = range(sys.inputs[name].dim) if index is None else [index]
            columns += [(name, i) for i in components]
        else:
            return None

    # The consecutive columns of one port make one port, each port once
    groups = []
    for port_id, i in columns:
        if groups and groups[-1][0] == port_id:
            groups[-1][1].append(i)
        else:
            groups.append((port_id, [i]))
    ids = [port_id for port_id, _ in groups]
    if len(set(ids)) < len(ids) or set(ids) & {"x", "y"}:
        return None

    # Each port restricted to its components, at its share of ū
    ports = []
    start = 0
    for port_id, components in groups:
        source = sys.inputs[port_id]
        stop = start + len(components)
        ports.append(
            InputPort(
                port_id,
                dim=len(components),
                nominal_value=u_bar[start:stop],
                labels=[source.labels[i] for i in components],
                units=[source.units[i] for i in components],
                lower_bound=source.lower_bound[components],
                upper_bound=source.upper_bound[components],
            )
        )
        start = stop

    return ports


def port_component_of(sys, i):
    """``(port id, component)`` of the entry ``i`` of the stacked input ``u``."""
    for port_id in sys.inputs:
        columns = sys.get_input_port_slice(port_id)
        if columns.start <= i < columns.stop:
            return port_id, i - columns.start
    raise IndexError(f"{sys.name!r} has {sys.m} inputs; got the input {i}")


def selected_output_signal(sys, outputs):
    """The labels and units of the rows the ``of`` selectors pick (the state's when ``None``)."""
    if outputs is None:
        return list(sys.state.labels), list(sys.state.units)

    labels = []
    units = []
    for name, index in outputs:
        if name in sys.outputs:
            signal = sys.outputs[name]
        else:
            block, port_id = name.split(":", 1)
            signal = sys.subsystems[block].outputs[port_id]
        components = range(signal.dim) if index is None else [index]
        labels += [signal.labels[i] for i in components]
        units += [signal.units[i] for i in components]

    return labels, units


def discrete_period(sys):
    """``Δ`` of a step system's model (1 per tick for a tick law), ``None`` for a continuous one."""
    if not isinstance(sys, StepSystem):
        return None
    period = period_of(sys)

    return 1.0 if period is None else period


def period_of(sys):
    """The sample period ``Δ`` a step system runs at, in seconds; ``None`` for a tick law."""
    if isinstance(sys, (DiscreteLTISystem, LinearizedStepSystem)):
        return sys.period

    return getattr(sys, "sample_period", None)


def design_point(model):
    """The point a design on ``model`` holds: ``(x̄, ū, ȳ)`` of a linearization, the origin of a linear model."""
    if isinstance(model, LINEARIZATIONS):
        return model.x_bar, model.u_bar, model.y_bar
    return np.zeros(model.n), np.zeros(model.m), np.zeros(model.p)


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
    if name in ("f", "step"):
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

    u_selected = np.concatenate(pieces) if pieces else np.zeros(0)

    return u_selected


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

    y_bar = np.concatenate(pieces)

    return y_bar


def require_design_system(sys):
    """Refuse a design on anything but a system, naming the gains that take arrays."""
    if not isinstance(sys, System):
        raise TypeError(
            "a design takes a system, a linear model LTISystem(A, B, C) or a plant, "
            f"not {type(sys).__name__}; on arrays the gains are lqr_gain(A, B, Q, R), "
            "place_gain(A, B, poles), kalman_gain(A, C, Q_n, R_n) and "
            "luenberger_gain(A, C, poles)"
        )


# ‖f(x̄, ū)‖ above this, relative to 1 + ‖x̄‖, is not an equilibrium
EQUILIBRIUM_TOL = 1e-6


def warn_off_equilibrium(sys, x_bar, u_bar, t, params):
    """Warn when a continuous plant, or a linearization, does not rest at the design point ``(x̄, ū)``: the linear model drops its drift."""
    stepped = isinstance(sys, LinearizedStepSystem)
    if (isinstance(sys, StepSystem) and not stepped) or sys.n == 0:
        return
    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)

    # The drift ẋ = f(x̄, ū), zero at an equilibrium; x⁺ − x̄ = F(x̄, ū) − x̄ on a step model
    if stepped:
        drift = "F(x_bar, u_bar) - x_bar"
        dx = np.asarray(sys.step(x_bar, u_bar, t, params), dtype=float) - x_bar
    else:
        drift = "f(x_bar, u_bar)"
        dx = np.asarray(sys.f(x_bar, u_bar, t, params), dtype=float)
    rate = float(np.linalg.norm(dx))

    if rate > EQUILIBRIUM_TOL * (1.0 + float(np.linalg.norm(x_bar))):
        warnings.warn(
            f"{sys.name!r} does not rest at the design point: |{drift}| = {rate:.3g}. "
            "The linear model (A, B, C, D) drops that term: its poles and responses "
            "are those of the deviations, and a loop designed on it will not hold "
            "x_bar. Linearize where the plant rests: pass the input that holds it "
            "(u_bar=), or find the point with find_equilibrium(sys, x_guess)",
            stacklevel=caller_stacklevel(),
        )


def require_continuous_model(model, verb):
    """Refuse a Riccati design on a discrete-time model, naming the design that runs on a clock."""
    if isinstance(model, (StepSystem, DiscreteLTISystem)):
        raise ValueError(
            f"{verb} solves the continuous-time Riccati equation, and {model.name!r} is "
            "a discrete-time model: design on the continuous plant, and put the law on "
            "a clock (law % dt). place and luenberger read their poles in z on a "
            "discrete-time model"
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
