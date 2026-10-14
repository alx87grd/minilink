"""Composition shortcuts: the ``+``, ``>>`` and ``@`` operators that build a diagram in one line.

No operator modifies its operands: each returns a new diagram that shares the
operands' blocks. ``+`` and ``>>`` lay a diagram operand out flat, with its ids,
wiring, boundary ports, ``x0`` and camera settings, and keep the left operand's
name. ``a >> b`` connects one output of ``a`` to one input of ``b`` by a fixed
rule on their edges, with no memory of how either was built. Only ``@`` nests:
a box on either side sits under its role name as one block.
Explicit ``add_subsystem`` / ``connect`` remains the canonical way to build any topology.
"""

from __future__ import annotations

import re
import warnings
from dataclasses import dataclass

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.feedback import (
    block_roles,
    check_reserved_id,
    command_port,
    require_command,
)
from minilink.core.system import System

#: Inputs that read their nominal value when left open, without a word from
#: autowire: a plant's disturbance and measurement noise.
_NOMINAL_WHEN_OPEN = frozenset({"d", "n"})


# Public API


def add_systems(*systems: System | DiagramSystem) -> DiagramSystem:
    """Return a new flat diagram containing ``systems`` as subsystems.

    Parameters
    ----------
    *systems : System or DiagramSystem
        Systems or diagrams to add, side by side. A diagram operand is laid
        out flat in the result, with its ids, wiring and boundary ports; it is
        not modified. The first operand's name, ``x0`` and camera settings
        are carried into the result when it is a diagram.

    Returns
    -------
    DiagramSystem
        A diagram containing all provided systems as direct subsystems.
        Existing diagram-internal connections are preserved, but no new
        cross-connections are inferred.
    """
    diagram = DiagramSystem()
    for sys in systems:
        if isinstance(sys, DiagramSystem):
            _inline_diagram(diagram, sys, output_collision="unique")
        else:
            _add_system_to_diagram(diagram, sys)
    if systems:
        _carry_left_operand(diagram, systems[0])
    return diagram


def series(
    left: System | DiagramSystem,
    right: System | DiagramSystem,
    *rest: System | DiagramSystem,
) -> DiagramSystem:
    """Connect systems in series: one output of each stage to one input of the next.

    ``A >> B`` wires one output of A to one input of B, the first rule that
    applies: A has one output and B one input; exactly one output of A and
    one input of B share a name; A has ``y`` and B has ``u``; else it refuses.
    A diagram counts its edge (its boundary outputs and its free boundary
    inputs), a leaf its ports. Sizes are a check, never a choice. One guard: a
    source into a block that reads both ``r`` and ``y`` is refused, since the
    name match would feed the measurement. The result keeps A's free inputs as
    its inputs and B's ``y`` (else its only output) as its output ``y``.

    Parameters
    ----------
    left : System or DiagramSystem
        The first system or diagram; a diagram is laid out flat in the result
        and left unchanged, its name, ``x0`` and camera settings carried.
    right : System or DiagramSystem
        System or diagram to connect after ``left``.
    *rest : System or DiagramSystem
        Additional systems or diagrams to append to the series chain.

    Returns
    -------
    DiagramSystem
        A new diagram with each stage connected output-to-input.
    """
    diagram = _as_composition_diagram(left)
    _series_pair(diagram, right)
    for next_sys in rest:
        _series_pair(diagram, next_sys)
    _carry_left_operand(diagram, left)
    return diagram


def feedback(sys, filter=1, *, of=None, sign=-1.0, validate=True) -> DiagramSystem:
    """Close ``sys`` on itself with an :class:`~minilink.blocks.routing.Error`
    block: ``r -> e = r - y -> sys -> y``, the textbook ``T = L / (1 + L H)``.

    Negative feedback (the default) inserts ``Error``, so the diagram shows
    ports ``+``, ``-``, and ``e``. Positive feedback still uses a signed
    :class:`~minilink.blocks.routing.Sum`. The junction drives the command
    input of ``sys`` (:func:`~minilink.core.feedback.command_port`: ``u``,
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
    diagram, entries = _open_loop(sys)
    _close_on(diagram, entries, filter, of=of, sign=sign)
    diagram.name = f"Closed loop of {sys.name}"
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

    One rule, read from the port names (:func:`~minilink.core.feedback.block_roles`):
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
    if not isinstance(plant, System):
        raise TypeError(
            "@ needs a plant on the right; to close an open loop on itself use "
            "feedback(L) or feedback(L, K)"
        )
    roles = block_roles(controller)
    if not roles.measurements and not roles.error_driven:
        raise ValueError(_no_measurement_message(controller, "controller"))
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
    if observer is not None:
        check_observer(controller, roles, observer, plant)
    elif filter is not None and "x" in roles.measurements:
        raise ValueError(
            f"{controller.name!r} reads the state 'x': a filter on the state is an "
            "observer; pass it as observer="
        )

    reads_plant_state = "x" in roles.measurements and observer is None
    plant_box = _plant_box_with_state(plant) if reads_plant_state else plant
    diagram = DiagramSystem()
    controller_id = _add_system_to_diagram(diagram, controller, role="controller")
    plant_id = _add_system_to_diagram(diagram, plant_box, role="plant")
    command, entry = _default_control_out(controller), _default_input_port(plant_box)
    if observer is not None:  # plant y and the command u in, the estimate x out
        observer_id = _add_system_to_diagram(diagram, observer, role="observer")
        diagram.connect(plant_id, "y", observer_id, "y")
        diagram.connect(controller_id, command, observer_id, "u")
        diagram.connect(observer_id, "x", controller_id, "x")
    else:  # plant y -> controller y, plant x -> controller x
        for port in roles.measurements:
            _check_measurement(controller, plant_box, port, controller_id, plant_id)
            diagram.connect(plant_id, port, controller_id, port)
    diagram.connect(controller_id, command, plant_id, entry)
    diagram.connect_new_output_port(plant_id, _default_output_port(plant_box), "y")

    if roles.error_driven:  # e = r - y drives the controller's entry
        measurement = _close_on(diagram, [(controller_id, roles.entry)], filter)
    else:  # the controller (or its observer) reads y, through the filter, and r
        reader = (
            (observer_id, "y")
            if observer is not None
            else (controller_id, roles.measurement)
        )
        measurement = _filter_into(diagram, filter, reader)
        if "r" in controller.inputs:
            _add_input_port_like(diagram, "r", controller.inputs["r"])
            diagram.connect("input", "r", controller_id, "r")

    # the loop's inputs: r, then d on the plant's command, then n on the measurement
    if isinstance(reference, System) and "r" not in diagram.inputs:
        raise ValueError(
            f"reference= has nothing to drive: {controller.name!r} reads no reference 'r'"
        )
    if reference is not True and "r" in diagram.inputs:
        (target,) = _drop_input(diagram, "r")  # False: r reads its nominal value
        if reference is not False:
            drive(diagram, target, "r", reference, "reference")
    if disturbance is not False:
        target = (
            (plant_id, "d")  # the plant owns its disturbance
            if "d" in plant_box.inputs
            else sum_into(diagram, (plant_id, entry), "disturbance_sum")
        )
        drive(diagram, target, "d", disturbance, "disturbance")
    if noise is not False:
        target = (
            (plant_id, "n")  # the plant owns its measurement noise
            if "n" in plant_box.inputs
            else sum_into(diagram, measurement, "noise_sum")
        )
        drive(diagram, target, "n", noise, "noise")

    diagram.name = f"Closed loop {plant.name} with {controller.name}"
    _propagate_animation_camera(diagram, plant_box)
    if validate:
        diagram.check_algebraic_loops()
    return diagram


def autowire(
    diagram: DiagramSystem,
    *,
    strict: bool = False,
    validate: bool = True,
) -> DiagramSystem:
    """Connect unconnected inputs by name: each takes the one output with its name and size.

    Parameters
    ----------
    diagram : DiagramSystem
        Diagram to modify in place.
    strict : bool, optional
        If ``True``, raise when an input stays open (no match, or several). No
        new connections are applied before that error is raised. If ``False``,
        the open inputs are left unconnected and a warning lists them.
    validate : bool, optional
        If ``True``, run algebraic-loop detection after wiring.

    Returns
    -------
    DiagramSystem
        The same diagram, for fluent use.

    Notes
    -----
    Names only, no guessing: an output may feed several inputs, existing
    connections are never overwritten, and a disturbance ``d`` or a noise
    ``n`` left open reads its nominal value without a word. Name a source's
    signal after the input it drives (``Step(port="r")``) and a block's
    ports at creation (``rename_ports``).
    """
    if not isinstance(diagram, DiagramSystem):
        raise TypeError("autowire() expects a DiagramSystem")

    decisions = []
    open_inputs = []
    for target_id, target in diagram.subsystems.items():
        for input_id, input_port in target.inputs.items():
            if diagram.connections[target_id][input_id] is not None:
                continue
            candidates = [
                (source_id, port_id)
                for source_id, source in diagram.subsystems.items()
                if source_id != target_id
                for port_id, output in source.outputs.items()
                if port_id == input_id and output.dim == input_port.dim
            ]
            if len(candidates) == 1:
                source_id, source_port = candidates[0]
                decisions.append((source_id, source_port, target_id, input_id))
            elif len(candidates) > 1:
                rendered = ", ".join(f"{s}:{p}" for s, p in candidates)
                open_inputs.append(
                    f"Ambiguous autowire target {target_id}:{input_id}; "
                    f"candidates: {rendered}"
                )
            elif input_id not in _NOMINAL_WHEN_OPEN:
                open_inputs.append(
                    f"no output named {input_id!r} for {target_id}:{input_id}"
                )

    if open_inputs and strict:
        raise ValueError(open_inputs[0])
    if open_inputs:
        warnings.warn(
            "autowire left these inputs unconnected (wire them with connect): "
            + "; ".join(open_inputs),
            stacklevel=3,
        )

    for source_id, source_port, target_id, input_id in decisions:
        diagram.connect(source_id, source_port, target_id, input_id)

    if validate:
        diagram.check_algebraic_loops()
    return diagram


# Internal machinery
#
# The shortcut bookkeeping, by concern: the operands laid out flat, the series
# rule, the loop, the default ports, the sampled loop's port resolution, the
# ids and the camera.


# The operands


def _as_composition_diagram(sys) -> DiagramSystem:
    """A diagram operand as a copy; a leaf as a diagram with its inputs on the edge."""
    if isinstance(sys, DiagramSystem):
        return _copy_diagram(sys)

    diagram = DiagramSystem()
    sys_id = _add_system_to_diagram(diagram, sys)
    _expose_free_inputs(diagram, sys_id)
    return diagram


def _copy_diagram(diagram):
    """A new diagram with the same blocks, ids, wiring, boundary ports, name, ``x0`` and camera.

    The blocks are shared, not copied; the operators build on the copy and leave
    the user's diagram untouched.
    """
    copy = DiagramSystem()
    _inline_diagram(copy, diagram, output_collision="replace")
    _carry_left_operand(copy, diagram)
    return copy


def _new_diagram_like(sys):
    """A diagram operand as a new diagram with the same ids and wiring; a leaf as is."""
    return _copy_diagram(sys) if isinstance(sys, DiagramSystem) else sys


def _carry_left_operand(diagram, left) -> None:
    """Carry a diagram left operand's name, ``x0`` and camera settings into ``diagram``.

    The left operand's blocks come first in the result, so its ``x0`` is the
    head of the stacked state; a leaf left operand carries nothing.
    """
    if not isinstance(left, DiagramSystem):
        return
    diagram.name = left.name
    diagram.x0 = np.concatenate(
        [np.asarray(left.x0, dtype=float), diagram.x0[left.n :]]
    )
    diagram.camera_scale = left.camera_scale
    diagram.camera_target = np.array(left.camera_target, dtype=float).copy()
    diagram.camera_plot_axes = left.camera_plot_axes
    diagram.camera_follow_frame = left.camera_follow_frame
    diagram.camera_priority = left.camera_priority
    diagram.scene_grid = left.scene_grid


def _inline_diagram(
    diagram,
    source,
    *,
    input_bindings=None,
    output_collision: str,
) -> dict[str, str]:
    """Flatten ``source`` into ``diagram``, remapping ids; return the id map."""
    input_bindings = {} if input_bindings is None else dict(input_bindings)
    source_subsystems = list(source.subsystems.items())
    source_inputs = list(source.inputs.items())
    source_connections = {
        source_id: dict(source.connections[source_id])
        for source_id, _subsystem in source_subsystems
    }
    source_output_connections = dict(source.connections.get("output", {}))
    source_outputs = dict(source.outputs)

    id_map = {}
    for source_id, subsystem in source_subsystems:
        check_reserved_id(source_id, diagram.subsystems)
        target_id = _unique_id(source_id, diagram.subsystems)
        diagram.add_subsystem(subsystem, target_id)
        id_map[source_id] = target_id

    input_port_map = {}
    for port_id, port in source_inputs:
        if port_id in input_bindings:
            continue
        target_port_id = _unique_id(port_id, diagram.inputs)
        _add_input_port_like(diagram, target_port_id, port)
        input_port_map[port_id] = target_port_id

    for source_target_id, subsystem in source_subsystems:
        target_id = id_map[source_target_id]
        for input_id in subsystem.inputs:
            edge = source_connections[source_target_id][input_id]
            if edge is None:
                continue
            source_sys_id, source_port_id = edge
            if source_sys_id == "input":
                if source_port_id in input_bindings:
                    bound_sys_id, bound_port_id = input_bindings[source_port_id]
                    diagram.connect(bound_sys_id, bound_port_id, target_id, input_id)
                else:
                    diagram.connect(
                        "input",
                        input_port_map[source_port_id],
                        target_id,
                        input_id,
                    )
            else:
                diagram.connect(
                    id_map[source_sys_id],
                    source_port_id,
                    target_id,
                    input_id,
                )

    for output_id, edge in source_output_connections.items():
        if edge is None:
            continue
        source_sys_id, source_port_id = edge
        if source_sys_id == "input":
            raise TypeError(
                "Flattening diagram outputs sourced directly from external "
                "inputs is not supported by composition shortcuts."
            )
        target_output_id = _output_port_id(diagram, output_id, output_collision)
        _connect_new_output_port_like(
            diagram,
            id_map[source_sys_id],
            source_port_id,
            target_output_id,
            source_outputs[output_id],
        )

    _propagate_animation_camera(diagram, source)

    return id_map


def _add_system_to_diagram(diagram, sys, *, role=None) -> str:
    """Add ``sys`` under its id, else its role, else its name; a reserved role once per level."""
    if not isinstance(sys, System):
        raise TypeError(f"Expected a System, got {type(sys).__name__}")

    sys_id = _default_subsystem_id(sys, role=role)
    check_reserved_id(sys_id, diagram.subsystems)
    sys_id = _unique_id(sys_id, diagram.subsystems)
    diagram.add_subsystem(sys, sys_id)
    return sys_id


def _add_input_port_like(diagram, port_id: str, port) -> None:
    diagram.add_input_port(
        port_id,
        dim=port.dim,
        nominal_value=port.nominal_value,
        labels=port.labels,
        units=port.units,
        lower_bound=port.lower_bound,
        upper_bound=port.upper_bound,
    )


def _connect_new_output_port_like(
    diagram,
    source_sys_id: str,
    source_port_id: str,
    output_port_id: str,
    output_port,
) -> None:
    diagram.connect_new_output_port(
        source_sys_id,
        source_port_id,
        output_port_id,
        nominal_value=output_port.nominal_value,
        labels=output_port.labels,
        units=output_port.units,
        lower_bound=output_port.lower_bound,
        upper_bound=output_port.upper_bound,
    )


def _output_port_id(diagram, output_id: str, collision: str) -> str:
    if collision == "replace":
        return output_id
    if collision == "unique":
        return _unique_id(output_id, diagram.outputs)
    raise ValueError(f"Unknown output collision policy {collision!r}")


def _expose_free_inputs(diagram, sys_id: str) -> None:
    """Put every input of a leaf on the diagram's edge, under the same name."""
    sys = diagram.subsystems[sys_id]
    for port_id, port in sys.inputs.items():
        _add_input_port_like(diagram, port_id, port)
        diagram.connect("input", port_id, sys_id, port_id)


def _drop_input(diagram, port_id: str) -> list[tuple[str, str]]:
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


# The series rule


def _series_pair(diagram, right) -> None:
    """Append ``right`` to ``diagram`` (the chain so far) by the ``>>`` rule."""
    outputs = _edge_outputs(diagram)
    source_name, target_name = _series_connection(diagram, outputs, right.inputs)
    source_id, source_port = outputs[source_name]
    # The chain's outputs are consumed; the result's output is the right side's.
    for output_id in list(diagram.outputs):
        del diagram.outputs[output_id]
        diagram.connections["output"].pop(output_id, None)

    if isinstance(right, DiagramSystem):
        _inline_diagram(
            diagram,
            right,
            input_bindings={target_name: (source_id, source_port)},
            output_collision="replace",
        )
    else:
        right_id = _add_system_to_diagram(diagram, right)
        diagram.connect(source_id, source_port, right_id, target_name)
        diagram.connect_new_output_port(right_id, _default_output_port(right), "y")
        _propagate_animation_camera(diagram, right)


def _edge_outputs(diagram) -> dict:
    """The chain's outputs by name: its boundary outputs, or the ports of its one block."""
    if diagram.outputs:
        sources = diagram.connections.get("output", {})
        # A box's own state (``expose("x")``) has no source block; it is not a chain output.
        return {
            output_id: tuple(sources[output_id])
            for output_id in diagram.outputs
            if sources.get(output_id) is not None
        }
    if len(diagram.subsystems) == 1:
        ((sys_id, sys),) = diagram.subsystems.items()
        return {port_id: (sys_id, port_id) for port_id in sys.outputs}
    raise ValueError("Left side of '>>' has no output port to connect")


def _series_connection(diagram, outputs, inputs) -> tuple[str, str]:
    """The ``(output, input)`` names ``>>`` connects, by the first rule that applies."""
    if not outputs:
        raise ValueError("Left side of '>>' has no output port to connect")
    if not inputs:
        raise ValueError("Right side of '>>' has no free input port to connect")
    if len(outputs) == 1 and len(inputs) == 1:
        return next(iter(outputs)), next(iter(inputs))
    shared = [name for name in outputs if name in inputs]
    if shared == ["y"] and "r" in inputs and not diagram.inputs:
        raise ValueError(
            "a source into a block that reads 'r' and 'y' would drive the measurement "
            "'y'; name the source's signal after the input it drives (port='r') for "
            "the block alone, or feed the loop: source >> (controller @ plant)"
        )
    if len(shared) == 1:
        return shared[0], shared[0]
    if "y" in outputs and "u" in inputs:
        return "y", "u"
    hint = (
        "; a source for a loop's reference goes in closed_loop(..., reference=source)"
        if "r" in inputs
        else ""
    )
    raise ValueError(
        "'>>' connects one output to one input; wire this one with connect "
        f"(outputs: {', '.join(outputs)}; free inputs: {', '.join(inputs)}){hint}"
    )


# The loop


def _plant_box_with_state(plant):
    """``plant`` with its state on its edge: a box gets its own ``x``, a leaf is as built."""
    if not isinstance(plant, DiagramSystem) or "x" in plant.outputs:
        return plant
    return _copy_diagram(plant).expose("x")


def _check_measurement(controller, plant, measurement, controller_id, plant_id) -> None:
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


def _no_measurement_message(controller, controller_id: str) -> str:
    return (
        f"Cannot wire closed-loop feedback: {controller_id} "
        f"({controller.name!r}) reads neither 'y' nor 'x'; its inputs are "
        f"{list(controller.inputs)}. Name its measurement port y or x "
        "(rename_ports), or wire the loop with DiagramSystem.connect."
    )


def _default_control_out(sys) -> str:
    return require_command(sys) or _default_output_port(sys)


def _filter_into(diagram, filter, target):
    """The return filter on the wire into ``target``; returns where a noise then enters."""
    sys_id, port_id = target
    signal, filter_in = _insert_filter(
        diagram, filter, diagram.connections[sys_id][port_id]
    )
    diagram.connect(*signal, sys_id, port_id)
    return filter_in or target


def _insert_filter(diagram, filter, signal):
    """Put the return filter after ``signal``: a block, a ``Gain``, nothing for ``1`` or ``None``.

    Returns the filtered signal ``(block, port)`` and the filter's input
    ``(block, port)``, where a measurement noise enters, or ``None`` without a
    filter. The block is named ``filter`` either way.
    """
    from minilink.blocks.routing import Gain

    if filter is None or (np.isscalar(filter) and float(filter) == 1.0):
        return signal, None
    signal_sys, signal_port = signal
    if not isinstance(filter, System):
        filter = Gain(
            filter, dim=diagram.subsystems[signal_sys].outputs[signal_port].dim
        )
    filter_id = _add_system_to_diagram(diagram, filter, role="filter")
    filter_in, filter_out = _default_input_port(filter), _default_output_port(filter)
    diagram.connect(signal_sys, signal_port, filter_id, filter_in)
    return (filter_id, filter_out), (filter_id, filter_in)


def sum_into(diagram, target, name):
    """Rewire ``target`` to read its source plus a second signal, through a ``Sum`` named ``name``.

    Returns the ``Sum``'s open input ``(name, "in1")``, where the added signal enters.
    """
    from minilink.blocks.routing import Sum

    sys_id, in_port = target
    source = diagram.connections[sys_id][in_port]
    dim = diagram.subsystems[sys_id].inputs[in_port].dim
    sum_id = _add_system_to_diagram(diagram, Sum(signs=(1.0, 1.0), dim=dim), role=name)
    diagram.connect(*source, sum_id, "in0")
    diagram.connect(sum_id, "y", sys_id, in_port)
    return sum_id, "in1"


def drive(diagram, target, port_id, value, name):
    """Drive the input ``target`` from a new edge input ``port_id`` (``value=True``), or from ``value``, a source block named ``name``."""
    sys_id, in_port = target
    if isinstance(value, System):
        source_id = _add_system_to_diagram(diagram, value, role=name)
        diagram.connect(source_id, _default_output_port(value), sys_id, in_port)
        return
    port = diagram.subsystems[sys_id].inputs[in_port]
    if in_port == port_id:  # a plant's own d or n: the edge keeps its labels and units
        _add_input_port_like(diagram, port_id, port)
    else:  # a junction's input
        diagram.add_input_port(port_id, dim=port.dim)
    diagram.connect("input", port_id, sys_id, in_port)


def _open_loop(sys):
    """``sys`` as a diagram with its command entry open for the junction and ``y`` out.

    Returns the diagram and the inputs the entry fed: a box keeps the rest of
    its edge; a leaf is the block ``plant`` with its default output as ``y``.
    """
    entry = _command_entry(sys)
    diagram = DiagramSystem()
    if isinstance(sys, DiagramSystem):
        _inline_diagram(diagram, sys, output_collision="replace")
        return diagram, _drop_input(diagram, entry)
    sys_id = _add_system_to_diagram(diagram, sys, role="plant")
    diagram.connect_new_output_port(sys_id, _default_output_port(sys), "y")
    return diagram, [(sys_id, entry)]


def _command_entry(sys) -> str:
    """The input the junction drives: ``u``, else the one free input, of a block reading no measurement.

    A block with a measurement port (``y`` or ``x``) is refused: a junction on
    its first input would leave the measurement open and feed ``e = r - y``
    into ``r``; it is closed on its plant with ``@``.
    """
    roles = block_roles(sys)
    if roles.measurements:
        raise ValueError(
            f"{sys.name} reads its measurement on {roles.measurement!r}"
            + (" (on its edge)" if isinstance(sys, DiagramSystem) else "")
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


def _close_on(diagram, entries, filter=1, *, of=None, sign=-1.0):
    """Close ``diagram`` with a junction: ``r`` in, ``y`` back through the filter, ``e`` on each entry.

    Returns where the measurement enters first: the filter's input when there
    is one, else the junction's minus input; a noise is summed in there.
    """
    from minilink.blocks.routing import Error, Sum

    entry_sys, entry_port = entries[0]
    entry_dim = diagram.subsystems[entry_sys].inputs[entry_port].dim
    signal, filter_in = _insert_filter(
        diagram, filter, _return_signal(diagram, of, entry_dim)
    )
    if float(sign) == -1.0:
        junction_id = _add_system_to_diagram(
            diagram, Error(dim=entry_dim), role="error"
        )
        plus_port, minus_port, out_port = "+", "-", "e"
    else:
        junction_id = _add_system_to_diagram(
            diagram, Sum(signs=(1.0, float(sign)), dim=entry_dim), role="sum"
        )
        plus_port, minus_port, out_port = "in0", "in1", "y"
    diagram.add_input_port("r", dim=entry_dim)
    diagram.connect("input", "r", junction_id, plus_port)
    diagram.connect(*signal, junction_id, minus_port)
    for entry_sys, entry_port in entries:
        diagram.connect(junction_id, out_port, entry_sys, entry_port)
    return filter_in or (junction_id, minus_port)


def _return_signal(diagram, of, entry_dim):
    """Source ``(sys_id, port_id)`` of what comes back to the junction."""
    from minilink.blocks.routing import Demux

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
    if index is None:
        if dim == entry_dim:
            return out_sys, out_port
        if entry_dim == 1 and dim > 1:
            index = 0  # the standard choice: the first component (the position)
        else:
            raise ValueError(
                f"Cannot close the loop: the input takes {entry_dim} values and "
                f"the output {port_id!r} has {dim}; pass of=({port_id!r}, index) "
                "or wire a Demux / Mux explicitly."
            )
    index = int(index)
    if not 0 <= index < dim:
        raise ValueError(f"of index must be in [0, {dim - 1}] for {port_id!r}")
    if entry_dim != 1:
        raise ValueError("of=(port, index) selects one component; the input takes more")
    parts = [d for d in (index, 1, dim - index - 1) if d > 0]
    demux_id = _add_system_to_diagram(
        diagram, Demux(dims=parts, port=out_port), role="demux"
    )
    diagram.connect(out_sys, out_port, demux_id, out_port)
    return demux_id, f"{out_port}[{index}]"


# Default ports


def _default_output_port(sys) -> str:
    if "y" in sys.outputs:
        return "y"
    if len(sys.outputs) == 1:
        return next(iter(sys.outputs))
    ports = ", ".join(sys.outputs)
    raise ValueError(
        f"Cannot choose a default output port for {sys.name!r}; candidates: {ports}"
    )


def _default_input_port(sys) -> str:
    for preferred in ("u", "r"):
        if preferred in sys.inputs:
            return preferred
    if len(sys.inputs) == 1:
        return next(iter(sys.inputs))
    ports = ", ".join(sys.inputs)
    raise ValueError(
        f"Cannot choose a default input port for {sys.name!r}; candidates: {ports}"
    )


# The sampled loop's port resolution


@dataclass(frozen=True)
class StandardFeedbackWiring:
    """Resolved port names for controller/computer ↔ plant feedback."""

    control_out: str
    measurement_in: str
    plant_in: str
    plant_out: str


def resolve_standard_feedback(
    controller,
    plant,
    *,
    controller_id: str = "controller",
    plant_id: str = "plant",
) -> StandardFeedbackWiring:
    """Choose the feedback port names shared by ``closed_loop`` and the sampled loop.

    The controller's port names decide (:func:`minilink.core.feedback.block_roles`):
    a block that reads ``y`` closes on the plant's ``y``, one that reads ``x`` on
    its ``x``, and one that reads both on the first the plant can serve. The
    command leaves on ``u`` (``u_ff`` for a solver, else the one output) and
    enters the plant on ``u`` (else its one input). Dimensions are a check,
    never a choice: a mechanical plant's ``y`` is its state ``[q; dq]``.
    """
    roles = block_roles(controller)
    if not roles.measurements:
        raise ValueError(_no_measurement_message(controller, controller_id))
    measurement_in, plant_out = _feedback_path(
        controller,
        plant,
        measurements=roles.measurements,
        controller_id=controller_id,
        plant_id=plant_id,
    )
    return StandardFeedbackWiring(
        _default_control_out(controller),
        measurement_in,
        _default_input_port(plant),
        plant_out,
    )


def _feedback_path(
    controller, plant, *, measurements: tuple, controller_id: str, plant_id: str
) -> tuple[str, str]:
    """The (controller input, plant output) pair: the first measurement the plant serves.

    With several measurements the first wireable one wins, and the first
    one's mismatch is reported when none is.
    """
    first_error = None
    for measurement in measurements:
        try:
            _check_measurement(controller, plant, measurement, controller_id, plant_id)
            return measurement, measurement
        except ValueError as exc:
            first_error = first_error or exc
    raise first_error


def default_computer_boundary_ports(controller) -> tuple[str, str]:
    """Return ``(measurement_in, control_out)`` for step-diagram boundary expose."""
    roles = block_roles(controller)
    measurement_in = roles.measurement or roles.entry or _default_input_port(controller)
    return measurement_in, _default_control_out(controller)


# Ids


def _default_subsystem_id(sys, *, role=None) -> str:
    """The id a block takes: its ``id``, else its role in a loop, else its own name."""
    if sys.id:
        return _normalize_identifier(str(sys.id))
    if role is not None:
        return _normalize_identifier(str(role))
    return _base_system_id(sys)


def _base_system_id(sys) -> str:
    raw = getattr(sys, "name", "") or sys.__class__.__name__
    if raw in {"System", "DynamicSystem"}:
        raw = sys.__class__.__name__
    return _normalize_identifier(raw)


def _normalize_identifier(value: str) -> str:
    value = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", str(value))
    value = re.sub(r"[^0-9A-Za-z]+", "_", value)
    value = value.strip("_").lower()
    if not value:
        value = "system"
    if value[0].isdigit():
        value = f"sys_{value}"
    return value


def _unique_id(base: str, existing) -> str:
    if base not in existing:
        return base
    i = 2
    while f"{base}{i}" in existing:
        i += 1
    return f"{base}{i}"


# The camera


def _propagate_animation_camera(diagram, *sources):
    """Copy camera hints from a plant-like *source* onto *diagram*.

    Prefer a source (or nested leaf) that already carries kinematic geometry —
    e.g. a catalog plant — over a bare ``Step`` / controller that would keep the
    generic auto-fit default (``camera_scale=None``).
    """
    fallback = None
    for source in sources:
        animated = _animated_geometry_source(source)
        if animated is not None:
            _copy_animation_camera_hints(diagram, animated)
            return
        if (
            fallback is None
            and isinstance(source, System)
            and not isinstance(source, DiagramSystem)
        ):
            fallback = source
    if fallback is not None:
        _copy_animation_camera_hints(diagram, fallback)


def _animated_geometry_source(obj):
    """First drawable in *obj* (or its subsystems) with static kinematic geometry."""
    if isinstance(obj, DiagramSystem):
        for sub in obj.subsystems.values():
            if sub.get_kinematic_geometry():
                return sub
        return None
    if obj.get_kinematic_geometry():
        return obj
    return None


def _copy_animation_camera_hints(diagram, animated):
    diagram.camera_scale = animated.camera_scale
    diagram.camera_target = np.array(animated.camera_target, dtype=float).copy()
    diagram.camera_plot_axes = getattr(animated, "camera_plot_axes", (0, 1))
    diagram.camera_follow_frame = _namespaced_camera_follow_frame(
        diagram,
        animated,
        getattr(animated, "camera_follow_frame", None),
    )


def _namespaced_camera_follow_frame(
    diagram: DiagramSystem, animated: System, follow: str | None
) -> str | None:
    if follow is None:
        return None
    if ":" in follow:
        return follow
    sys_id = _subsystem_id_for_leaf(diagram, animated)
    if sys_id is None:
        return follow
    return f"{sys_id}:{follow}"


def _subsystem_id_for_leaf(diagram: DiagramSystem, leaf: System) -> str | None:
    """Return ``sys_id`` when *leaf* is a direct subsystem of *diagram*."""
    for sys_id, subsystem in diagram.subsystems.items():
        if subsystem is leaf:
            return sys_id
    return None
