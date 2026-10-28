"""The closed loop: ``@``, ``closed_loop`` and ``feedback`` wrap a controller around its plant."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.assembly import (
    add_input_port_like,
    add_system_to_diagram,
    algebra_of,
    copy_diagram,
    default_output_port,
    inline_diagram,
    is_box,
    kind_of,
    propagate_animation_camera,
)
from minilink.core.diagram import DiagramSystem
from minilink.core.discrete.sampled_system import SampledSystem, sampled_substeps
from minilink.core.roles import (
    block_roles,
    command_port,
    require_command,
)
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


def feedback(sys, filter=1, *, of=None, sign=-1.0, validate=True) -> DiagramSystem:
    """Close ``sys`` on itself with an :class:`~minilink.blocks.routing.Error` block: ``r -> e = r - y -> sys -> y``, the textbook ``T = L / (1 + L H)``.

    Negative feedback (the default) inserts ``Error``, so the diagram shows
    ports ``+``, ``-``, and ``e``. Positive feedback still uses a signed
    :class:`~minilink.blocks.routing.Sum`. The junction drives the command
    input of ``sys`` (:func:`~minilink.core.roles.command_port`: ``u``,
    else its one free input) — a plant, a transfer function, a compensator,
    or a series diagram such as ``C >> G``; a block that reads its own
    measurement (``y`` or ``x``) is closed on its plant with ``@`` instead.
    What comes back is ``of``, else ``y``, else the only output. ``sys`` is
    not modified: the result is a new diagram sharing its blocks, with
    boundary input ``r`` and that output as boundary output ``y``. ``@``
    always means controller at plant; this is the one way to close an open
    loop on itself.

    Parameters
    ----------
    filter : 1, scalar, array, or System
        The return path ``H``: ``1`` is a direct wire, a scalar or matrix
        inserts a ``Gain`` block, a system (a sensor, a low-pass) is inserted
        as a block; the block is named ``filter`` either way.
    of : str or (str, int), optional
        Which output comes back: a boundary output port id, or
        ``(port, index)`` for one component. Default: the whole output when
        its dimension matches the input; a scalar input against a vector
        output takes component 0 through a visible ``Demux``; any other
        mismatch is an error.
    sign : float
        ``-1.0`` for negative feedback (default), ``+1.0`` for positive.
    validate : bool
        Run algebraic-loop detection before returning.
    """
    # The open loop: sys with its command entry free for the junction, and y out
    diagram, entries = open_loop(sys)

    # The junction on the entry: e = r − H y, a signed Sum for any other sign
    close_on(diagram, entries, filter, of=of, sign=sign)

    diagram.name = f"feedback({sys.name})"
    if validate:
        diagram.check_algebraic_loops()

    return diagram


def closed_loop(
    controller: System,
    plant: System,
    *,
    reference: bool | System = True,
    disturbance: bool | System = False,
    noise: bool | System = False,
    filter=None,
    observer: System | None = None,
    validate: bool = True,
) -> DiagramSystem:
    """Build a controller/plant feedback diagram — the ``controller @ plant`` operator.

    One rule, read from the port names (:func:`~minilink.core.roles.block_roles`):
    every controller input named ``y`` or ``x`` is wired to the plant output of
    the same name; a controller with neither (an error-driven block: a
    compensator, a transfer function, a plant, a series diagram) gets an
    :class:`~minilink.blocks.routing.Error` block ``e = r - y`` on its command
    entry, the junction of :func:`feedback`. The controller's command (``u``, else
    its one output) drives the plant's command input, and the plant's ``y``
    (else its one output) is the loop's output ``y``.

    The port names are the whole declaration: there is no port-name keyword.
    A mechanical plant's ``y`` is its state ``[q; dq]``, read straight; a block
    with other names is renamed (``rename_ports``) or wired by hand. A
    controller that reads ``x`` needs a plant that exposes it: a leaf built
    with ``expose_state=True``, or a box, whose own state is read.

    A box (a diagram) on either side is nested under its role name as one
    block; a leaf stays a leaf. ``@`` needs a plant on the right:
    :func:`feedback` closes an open loop on itself (``feedback(L)``,
    ``feedback(L, K)``).

    Neither operand is modified: the result is a new diagram sharing their
    blocks; a plant box whose state is read is copied, with its blocks shared.

    Parameters
    ----------
    controller : System
        Controller subsystem with reference, measurement, and control ports.
    plant : System
        Plant subsystem with control input and measurement output ports.
    reference, disturbance, noise : bool or System, optional
        The loop's inputs, on the ports ``r``, ``d`` and ``n``: the reference
        (default on), the load disturbance and the measurement noise (both
        off by default). ``True`` puts the port on the loop's edge, ``False``
        leaves it out (the reference then reads its nominal value), and a
        source block (``Step()``, ``WhiteNoise(...)``) drives it inside the
        loop, named ``reference``, ``disturbance`` or ``noise``. A plant that
        declares ``d`` or ``n`` owns it: the input drives the plant's own port.
        Otherwise a ``Sum`` adds the disturbance to the plant's command
        (``disturbance_sum``) and the noise to the measurement before it
        returns (``noise_sum``).
    filter : System, scalar or array, optional
        The return path: a sensor or filter block, or a gain (a ``Gain``
        block), named ``filter``, between the plant's ``y`` and what reads
        the measurement (the controller's ``y``, the ``Error`` block, or the
        observer). The plant's ``y`` stays the loop's output and the noise
        ``n`` enters before the filter. A controller that reads ``x`` without
        an observer is refused: a filter on the state is an observer.
    observer : System, optional
        The state estimator, named ``observer``, for a controller that reads
        ``x``: the plant's ``y`` and the controller's ``u`` go in, and its
        ``x`` (the estimate) goes to the controller's ``x``, so the plant's
        own state is never read. A ``kalman`` or ``luenberger`` observer with a
        state-feedback gain is the LQG loop.
    validate : bool, optional
        If ``True``, run algebraic-loop detection before returning.

    Returns
    -------
    DiagramSystem
        A diagram with the plant's ``y`` as output and the inputs left on
        its edge (``r``, ``d``, ``n``) as inputs.
    """
    # A plant on the right; an error-driven controller takes the junction, any
    # other reads a measurement; the loop's inputs are flags or source blocks
    require_plant_on_right(plant)
    roles = block_roles(controller)
    if not roles.error_driven:
        require_measurement(controller, roles, "controller")
    require_loop_inputs(reference, disturbance, noise)
    if observer is not None:
        check_observer(controller, roles, observer, plant)
    else:
        require_no_state_filter(controller, roles, filter)

    # The plant as the loop reads it: a box whose state the controller reads gives
    # its own x; under a sampled controller, the plant on its base clock
    reads_plant_state = "x" in roles.measurements and observer is None
    plant_box = plant_box_with_state(plant) if reads_plant_state else plant
    plant_box = on_the_controller_clock(controller, plant_box)

    # The controller and the plant, each under its role name, in their algebra
    diagram = algebra_of(controller, plant_box)()
    controller_id = add_system_to_diagram(diagram, controller, role="controller")
    plant_id = add_system_to_diagram(diagram, plant_box, role="plant")
    command = default_control_out(controller)
    entry = default_input_port(plant_box)

    # The measurement path
    if observer is not None:
        # The plant's y and the command u into the observer, its estimate x out
        observer_id = add_system_to_diagram(diagram, observer, role="observer")
        diagram.connect(plant_id, "y", observer_id, "y")
        diagram.connect(controller_id, command, observer_id, "u")
        diagram.connect(observer_id, "x", controller_id, "x")
    else:
        # The plant's y into the controller's y, the plant's x into its x
        for port in roles.measurements:
            check_measurement(controller, plant_box, port, controller_id, plant_id)
            diagram.connect(plant_id, port, controller_id, port)

    # The command into the plant, and the plant's y as the loop's output
    diagram.connect(controller_id, command, plant_id, entry)
    diagram.connect_new_output_port(plant_id, default_output_port(plant_box), "y")

    # The return path: e = r − y on the controller's entry, or the measurement
    # (through the filter) and the reference r read by the controller itself
    if roles.error_driven:
        measurement = close_on(diagram, [(controller_id, roles.entry)], filter)
    else:
        if observer is not None:
            reader = (observer_id, "y")
        else:
            reader = (controller_id, roles.measurement)
        measurement = filter_into(diagram, filter, reader)
        if "r" in controller.inputs:
            add_input_port_like(diagram, "r", controller.inputs["r"])
            diagram.connect("input", "r", controller_id, "r")

    # The reference: on the edge, at its nominal value (False), or from a source
    require_reference_reader(diagram, reference, controller)
    if reference is not True and "r" in diagram.inputs:
        (target,) = drop_input(diagram, "r")
        if reference is not False:
            drive(diagram, target, "r", reference, "reference")

    # The disturbance: on the plant's own d, else summed into its command
    if disturbance is not False:
        if "d" in plant_box.inputs:
            target = (plant_id, "d")
        else:
            target = sum_into(diagram, (plant_id, entry), "disturbance_sum")
        drive(diagram, target, "d", disturbance, "disturbance")

    # The noise: on the plant's own n, else summed into the measurement
    if noise is not False:
        if "n" in plant_box.inputs:
            target = (plant_id, "n")
        else:
            target = sum_into(diagram, measurement, "noise_sum")
        drive(diagram, target, "n", noise, "noise")

    # The loop's name and camera, then the algebraic-loop check
    diagram.name = f"{controller.name} @ {plant.name}"
    propagate_animation_camera(diagram, plant_box)
    if validate:
        diagram.check_algebraic_loops()

    return diagram


def resolve_standard_feedback(
    controller,
    plant,
    *,
    controller_id: str = "controller",
    plant_id: str = "plant",
) -> StandardFeedbackWiring:
    """Choose the feedback port names shared by ``closed_loop`` and the sampled loop.

    The controller's port names decide (:func:`minilink.core.roles.block_roles`):
    a block that reads ``y`` closes on the plant's ``y``, one that reads ``x`` on
    its ``x``, and one that reads both on the first the plant can serve. The
    command leaves on ``u`` (``u_ff`` for a solver, else the one output) and
    enters the plant on ``u`` (else its one input). Dimensions are a check,
    never a choice: a mechanical plant's ``y`` is its state ``[q; dq]``.
    """
    roles = block_roles(controller)
    require_measurement(controller, roles, controller_id)

    # The first measurement the plant serves
    measurement_in, plant_out = feedback_path(
        controller,
        plant,
        measurements=roles.measurements,
        controller_id=controller_id,
        plant_id=plant_id,
    )

    # The command out of the controller and into the plant
    control_out = default_control_out(controller)
    plant_in = default_input_port(plant)

    return StandardFeedbackWiring(control_out, measurement_in, plant_in, plant_out)


@dataclass(frozen=True)
class StandardFeedbackWiring:
    """Resolved port names for controller/computer ↔ plant feedback."""

    control_out: str
    measurement_in: str
    plant_in: str
    plant_out: str


def default_computer_boundary_ports(controller) -> tuple[str, str]:
    """Return ``(measurement_in, control_out)`` for step-diagram boundary expose."""
    roles = block_roles(controller)

    # The measurement y or x, else the error entry, else the default input
    measurement_in = roles.measurement or roles.entry or default_input_port(controller)
    control_out = default_control_out(controller)

    return measurement_in, control_out


###############################################################################
# Internal machinery
###############################################################################


# The loop


def open_loop(sys):
    """``sys`` as a diagram with its command entry open for the junction and ``y`` out.

    Returns the diagram and the inputs the entry fed: a box keeps the rest of
    its edge; a leaf is the block ``plant`` with its default output as ``y``.
    """
    entry = command_entry(sys)
    diagram = algebra_of(sys)()

    # A box laid out flat, its entry taken off the edge
    if is_box(sys):
        inline_diagram(diagram, sys, output_collision="replace")
        entries = drop_input(diagram, entry)
        return diagram, entries

    # A leaf as the block plant, its default output as y
    sys_id = add_system_to_diagram(diagram, sys, role="plant")
    diagram.connect_new_output_port(sys_id, default_output_port(sys), "y")
    entries = [(sys_id, entry)]

    return diagram, entries


def command_entry(sys) -> str:
    """The input the junction drives: ``u``, else the one free input, of a block reading no measurement.

    A block with a measurement port (``y`` or ``x``) is refused: a junction on
    its first input would leave the measurement open and feed ``e = r - y``
    into ``r``; it is closed on its plant with ``@``.
    """
    roles = block_roles(sys)
    if roles.measurements:
        raise ValueError(
            f"{sys.name} reads its measurement on {roles.measurement!r}"
            + (" (on its edge)" if is_box(sys) else "")
            + (f" and its reference on {roles.reference!r}" if roles.reference else "")
            + ": an Error block cannot drive it. Close it on the plant directly "
            "(controller @ plant), use its error layout (ports='error') if it has one, "
            "or wire the loop with DiagramSystem.connect."
        )

    try:
        return command_port(sys)
    except ValueError as exc:
        raise ValueError(
            f"feedback() closes {sys.name!r} on its command input: 'u', else its one "
            f"free input; it has {list(sys.inputs)}. Close a controller on its plant "
            "with controller @ plant, or wire the loop with DiagramSystem.connect."
        ) from exc


def drop_input(diagram, port_id: str) -> list[tuple[str, str]]:
    """Take boundary input ``port_id`` off the edge; returns the inputs it fed, now open."""
    fed = [
        (sys_id, in_port)
        for sys_id, sys in diagram.subsystems.items()
        for in_port in sys.inputs
        if diagram.connections[sys_id][in_port] == ("input", port_id)
    ]
    for sys_id, in_port in fed:
        diagram.connections[sys_id][in_port] = None
    del diagram.inputs[port_id]

    return fed


def close_on(diagram, entries, filter=1, *, of=None, sign=-1.0):
    """Close ``diagram`` with a junction: ``r`` in, ``y`` back through the filter, ``e`` on each entry.

    Returns where the measurement enters first: the filter's input when there
    is one, else the junction's minus input; a noise is summed in there.
    """
    # Lazy import: the blocks load on first use, not with the core
    from minilink.blocks.routing import Error, Sum

    entry_sys, entry_port = entries[0]
    entry_dim = diagram.subsystems[entry_sys].inputs[entry_port].dim

    # What comes back, through the filter
    signal, filter_in = insert_filter(
        diagram, filter, return_signal(diagram, of, entry_dim)
    )

    # The junction: an Error block e = r − y for negative feedback, else a signed Sum
    if float(sign) == -1.0:
        junction_id = add_system_to_diagram(diagram, Error(dim=entry_dim), role="error")
        plus_port = "+"
        minus_port = "-"
        out_port = "e"
    else:
        junction_id = add_system_to_diagram(
            diagram, Sum(signs=(1.0, float(sign)), dim=entry_dim), role="sum"
        )
        plus_port = "in0"
        minus_port = "in1"
        out_port = "y"

    # The reference r on the edge into the plus input, the return into the minus,
    # and e on each entry
    diagram.add_input_port("r", dim=entry_dim)
    diagram.connect("input", "r", junction_id, plus_port)
    diagram.connect(*signal, junction_id, minus_port)
    for entry_sys, entry_port in entries:
        diagram.connect(junction_id, out_port, entry_sys, entry_port)

    return filter_in or (junction_id, minus_port)


def return_signal(diagram, of, entry_dim):
    """Source ``(sys_id, port_id)`` of what comes back to the junction."""
    # Lazy import: the blocks load on first use, not with the core
    from minilink.blocks.routing import Demux

    # The output that comes back: of, else y, else the only output
    port_id, index = of if isinstance(of, tuple) else (of, None)
    if port_id is None:
        if "y" in diagram.outputs or len(diagram.outputs) == 1:
            port_id = "y" if "y" in diagram.outputs else next(iter(diagram.outputs))
        else:
            raise ValueError(
                "Cannot choose what comes back: no output 'y' among "
                f"{', '.join(diagram.outputs)}; pass of=(port, index)"
            )
    if port_id not in diagram.outputs:
        raise ValueError(
            f"Unknown output {port_id!r}; boundary outputs: {', '.join(diagram.outputs)}"
        )
    out_sys, out_port = diagram.connections["output"][port_id]
    dim = diagram.subsystems[out_sys].outputs[out_port].dim

    # The whole output when the sizes match; a scalar entry takes one component
    if index is None:
        if dim == entry_dim:
            return out_sys, out_port
        if entry_dim == 1 and dim > 1:
            index = 0  # The standard choice: the first component (the position)
        else:
            raise ValueError(
                f"Cannot close the loop: the input takes {entry_dim} values and "
                f"the output {port_id!r} has {dim}; pass of=({port_id!r}, index) "
                "or wire a Demux / Mux explicitly."
            )

    # One component, split off through a visible Demux
    index = int(index)
    if not 0 <= index < dim:
        raise ValueError(f"of index must be in [0, {dim - 1}] for {port_id!r}")
    if entry_dim != 1:
        raise ValueError("of=(port, index) selects one component; the input takes more")
    parts = [d for d in (index, 1, dim - index - 1) if d > 0]
    demux_id = add_system_to_diagram(
        diagram, Demux(dims=parts, port=out_port), role="demux"
    )
    diagram.connect(out_sys, out_port, demux_id, out_port)

    return demux_id, f"{out_port}[{index}]"


def insert_filter(diagram, filter, signal):
    """Put the return filter after ``signal``: a block, a ``Gain``, nothing for ``1`` or ``None``.

    Returns the filtered signal ``(block, port)`` and the filter's input
    ``(block, port)``, where a measurement noise enters, or ``None`` without a
    filter. The block is named ``filter`` either way.
    """
    # Lazy import: the blocks load on first use, not with the core
    from minilink.blocks.routing import Gain

    # A direct wire
    if filter is None or (np.isscalar(filter) and float(filter) == 1.0):
        return signal, None

    # A gain becomes a Gain block of the signal's size
    signal_sys, signal_port = signal
    if not isinstance(filter, System):
        filter = Gain(
            filter, dim=diagram.subsystems[signal_sys].outputs[signal_port].dim
        )

    # The filter block on the signal
    filter_id = add_system_to_diagram(diagram, filter, role="filter")
    filter_in = default_input_port(filter)
    filter_out = default_output_port(filter)
    diagram.connect(signal_sys, signal_port, filter_id, filter_in)

    return (filter_id, filter_out), (filter_id, filter_in)


def on_the_controller_clock(controller, plant):
    """A continuous plant under a sampled controller: the plant on the loop's base clock, else as is.

    ``sampled @ continuous`` is where the two algebras meet: the plant becomes
    ``discretize(plant, Δ, substeps=n)``, ``Δ`` the controller's base period and
    ``n`` from the automatic step rule.
    """
    if kind_of(controller) != "clocked" or kind_of(plant) != "continuous":
        return plant

    base = controller.sample_period
    sampled = SampledSystem(plant, base, substeps=sampled_substeps(plant, base))
    sampled.name = plant.name

    return sampled


def plant_box_with_state(plant):
    """``plant`` with its state on its edge: a box gets its own ``x``, a leaf is as built."""
    if not is_box(plant) or "x" in plant.outputs:
        return plant

    return copy_diagram(plant).expose("x")


def filter_into(diagram, filter, target):
    """The return filter on the wire into ``target``; returns where a noise then enters."""
    sys_id, port_id = target
    source = diagram.connections[sys_id][port_id]
    signal, filter_in = insert_filter(diagram, filter, source)
    diagram.connect(*signal, sys_id, port_id)

    return filter_in or target


def drive(diagram, target, port_id, value, name):
    """Drive the input ``target`` from a new edge input ``port_id`` (``value=True``), or from ``value``, a source block named ``name``."""
    sys_id, in_port = target

    # A source block, under its role name
    if isinstance(value, System):
        source_id = add_system_to_diagram(diagram, value, role=name)
        diagram.connect(source_id, default_output_port(value), sys_id, in_port)
        return

    # A new edge input: a plant's own d or n keeps its labels and units, a
    # junction's input takes its size alone
    port = diagram.subsystems[sys_id].inputs[in_port]
    if in_port == port_id:
        add_input_port_like(diagram, port_id, port)
    else:
        diagram.add_input_port(port_id, dim=port.dim)
    diagram.connect("input", port_id, sys_id, in_port)


def sum_into(diagram, target, name):
    """Rewire ``target`` to read its source plus a second signal, through a ``Sum`` named ``name``.

    Returns the ``Sum``'s open input ``(name, "in1")``, where the added signal enters.
    """
    # Lazy import: the blocks load on first use, not with the core
    from minilink.blocks.routing import Sum

    sys_id, in_port = target
    source = diagram.connections[sys_id][in_port]
    dim = diagram.subsystems[sys_id].inputs[in_port].dim

    # The Sum between the target and its source, its second input left open
    sum_id = add_system_to_diagram(diagram, Sum(signs=(1.0, 1.0), dim=dim), role=name)
    diagram.connect(*source, sum_id, "in0")
    diagram.connect(sum_id, "y", sys_id, in_port)

    return sum_id, "in1"


def feedback_path(
    controller, plant, *, measurements: tuple, controller_id: str, plant_id: str
) -> tuple[str, str]:
    """The (controller input, plant output) pair: the first measurement the plant serves.

    With several measurements the first wireable one wins, and the first
    one's mismatch is reported when none is.
    """
    first_error = None
    for measurement in measurements:
        try:
            check_measurement(controller, plant, measurement, controller_id, plant_id)
            return measurement, measurement
        except ValueError as exc:
            first_error = first_error or exc
    raise first_error


# The refusals, in the order the public verbs meet them


def require_plant_on_right(plant) -> None:
    """Refuse a right operand of ``@`` that is not a block: ``@`` needs a plant there."""
    if not isinstance(plant, System):
        raise TypeError(
            "@ needs a plant on the right; to close an open loop on itself use "
            "feedback(L) or feedback(L, K)"
        )


def require_measurement(controller, roles, controller_id: str) -> None:
    """Refuse a controller that reads neither ``y`` nor ``x``, naming its inputs."""
    if not roles.measurements:
        raise ValueError(
            f"Cannot wire closed-loop feedback: {controller_id} "
            f"({controller.name!r}) reads neither 'y' nor 'x'; its inputs are "
            f"{list(controller.inputs)}. Name its measurement port y or x "
            "(rename_ports), or wire the loop with DiagramSystem.connect."
        )


def require_loop_inputs(reference, disturbance, noise) -> None:
    """Refuse a loop input that is neither a flag nor a source block."""
    for word, value in (
        ("reference", reference),
        ("disturbance", disturbance),
        ("noise", noise),
    ):
        if not isinstance(value, (bool, System)):
            raise TypeError(
                f"{word}= takes True (an input on the loop's edge), False (none) or a "
                f"source block such as Step() or WhiteNoise(...); got {value!r}"
            )


def check_observer(controller, roles, observer, plant) -> None:
    """Refuse an observer the loop cannot wire: ``u``, ``y`` -> ``x``, for a controller that reads ``x`` alone."""
    if roles.measurements != ("x",):
        reads = (
            f"reads {' and '.join(repr(p) for p in roles.measurements)}"
            if roles.measurements
            else "is driven by the error"
        )
        raise ValueError(
            "observer= gives the state estimate 'x' to a controller that reads 'x' "
            f"alone; {controller.name!r} {reads}. Wire it with DiagramSystem.connect."
        )
    if not ({"u", "y"} <= set(observer.inputs) and "x" in observer.outputs):
        raise ValueError(
            f"observer {observer.name!r} must read 'u' and 'y' and give 'x' (a "
            f"kalman or luenberger observer); it reads {list(observer.inputs)} and "
            f"gives {list(observer.outputs)}"
        )
    if "y" not in plant.outputs:
        raise ValueError(
            f"the observer reads the plant's measurement 'y'; plant {plant.name!r} "
            f"gives {list(plant.outputs)}"
        )


def require_no_state_filter(controller, roles, filter) -> None:
    """Refuse a filter on the state ``x`` without an observer: a filter on the state is an observer."""
    if filter is not None and "x" in roles.measurements:
        raise ValueError(
            f"{controller.name!r} reads the state 'x': a filter on the state is an "
            "observer; pass it as observer="
        )


def check_measurement(controller, plant, measurement, controller_id, plant_id) -> None:
    """Refuse a measurement the plant cannot serve: the port of the same name, same size."""
    dim = controller.inputs[measurement].dim
    if measurement not in plant.outputs:
        fix = {
            "y": "declare it with output_dim=... (the standard port, y = x by default)",
            "x": "build the plant with expose_state=True",
        }[measurement]
        raise ValueError(
            f"Cannot wire closed-loop feedback: plant {plant.name!r} ({plant_id}) has "
            f"no {measurement!r} output port; {fix}, or wire the loop with "
            "DiagramSystem.connect"
        )
    if plant.outputs[measurement].dim != dim:
        raise ValueError(
            f"Cannot wire closed-loop feedback: {controller_id} declares measurement "
            f"{measurement!r} dim {dim}, but {plant_id} {measurement!r} output has "
            f"dim {plant.outputs[measurement].dim}"
        )


def require_reference_reader(diagram, reference, controller) -> None:
    """Refuse a reference source the loop has no ``r`` for: the controller reads no reference."""
    if isinstance(reference, System) and "r" not in diagram.inputs:
        raise ValueError(
            f"reference= has nothing to drive: {controller.name!r} reads no reference 'r'"
        )


# Default ports


def default_control_out(sys) -> str:
    """The command a block gives: ``u`` (``u_ff`` for a solver), else its default output."""
    return require_command(sys) or default_output_port(sys)


def default_input_port(sys) -> str:
    """The input a block reads by default: ``u``, else ``r``, else its only input."""
    for preferred in ("u", "r"):
        if preferred in sys.inputs:
            return preferred
    if len(sys.inputs) == 1:
        return next(iter(sys.inputs))

    ports = ", ".join(sys.inputs)
    raise ValueError(
        f"Cannot choose a default input port for {sys.name!r}; candidates: {ports}"
    )
