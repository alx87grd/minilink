"""Composition operators: ``+`` and ``>>`` build a diagram in one line, ``autowire`` wires one by port names."""

from __future__ import annotations

import warnings

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.roles import (
    check_reserved_id,
)
from minilink.core.system import System

#: Inputs that read their nominal value when left open, without a word from
#: autowire: a plant's disturbance and measurement noise.
_NOMINAL_WHEN_OPEN = frozenset({"d", "n"})


###############################################################################
# Public API
###############################################################################

# No operator modifies its operands: each returns a new diagram that shares the
# operands' blocks. ``+`` and ``>>`` lay a diagram operand out flat, with its ids,
# wiring, boundary ports, ``x0`` and camera settings, and keep the left operand's
# name. ``a >> b`` connects one output of ``a`` to one input of ``b`` by a fixed
# rule on their edges, with no memory of how either was built. Only ``@``
# (:mod:`minilink.core.closed_loop`) nests: a box on either side sits under its
# role name as one block. Explicit ``add_subsystem`` / ``connect`` remains the
# canonical way to build any topology.


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

    # Each operand side by side: a diagram laid out flat, a leaf as one block
    for sys in systems:
        if isinstance(sys, DiagramSystem):
            inline_diagram(diagram, sys, output_collision="unique")
        else:
            add_system_to_diagram(diagram, sys)

    # The first operand's name, x0 and camera, when it is a diagram; else the
    # sum as written
    if systems:
        carry_left_operand(diagram, systems[0])
        if not isinstance(systems[0], DiagramSystem):
            diagram.name = " + ".join(sys.name for sys in systems)

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
    # The chain so far: a diagram operand copied, a leaf with its inputs on the edge
    diagram = as_composition_diagram(left)

    # Each next stage on the chain's output, by the ``>>`` rule
    series_pair(diagram, right)
    for next_sys in rest:
        series_pair(diagram, next_sys)

    # The left operand's name, x0 and camera, when it is a diagram; else the
    # chain as written
    carry_left_operand(diagram, left)
    if not isinstance(left, DiagramSystem):
        diagram.name = " >> ".join(sys.name for sys in (left, right, *rest))

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
    require_diagram(diagram)

    # Each open input takes the one output of another block with its name and size;
    # none, or several, leave it open (a d or an n quietly reads its nominal value)
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

    # Strict: an open input is refused before any connection; else a warning
    # lists them
    if strict:
        require_wired(open_inputs)
    if open_inputs:
        warnings.warn(
            "autowire left these inputs unconnected (wire them with connect): "
            + "; ".join(open_inputs),
            stacklevel=3,
        )

    # The connections, once every input is decided
    for source_id, source_port, target_id, input_id in decisions:
        diagram.connect(source_id, source_port, target_id, input_id)

    if validate:
        diagram.check_algebraic_loops()

    return diagram


###############################################################################
# Internal machinery
###############################################################################


# The operands, laid out flat


def inline_diagram(
    diagram,
    source,
    *,
    input_bindings=None,
    output_collision: str,
) -> dict[str, str]:
    """Flatten ``source`` into ``diagram``, remapping ids; return the id map."""
    input_bindings = {} if input_bindings is None else dict(input_bindings)

    # The source's blocks, edge inputs, wiring and boundary outputs
    source_subsystems = list(source.subsystems.items())
    source_inputs = list(source.inputs.items())
    source_connections = {
        source_id: dict(source.connections[source_id])
        for source_id, _subsystem in source_subsystems
    }
    source_output_connections = dict(source.connections.get("output", {}))
    source_outputs = dict(source.outputs)

    # Each block under a fresh id, a reserved role once per level
    id_map = {}
    for source_id, subsystem in source_subsystems:
        check_reserved_id(source_id, diagram.subsystems)
        target_id = unique_id(source_id, diagram.subsystems)
        diagram.add_subsystem(subsystem, target_id)
        id_map[source_id] = target_id

    # Each edge input under a fresh name, except the ones a binding feeds
    input_port_map = {}
    for port_id, port in source_inputs:
        if port_id in input_bindings:
            continue
        target_port_id = unique_id(port_id, diagram.inputs)
        add_input_port_like(diagram, target_port_id, port)
        input_port_map[port_id] = target_port_id

    # The wiring, remapped: an edge input reads its binding, else its new name
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

    # The boundary outputs, remapped; one read straight off an edge input is refused
    for output_id, edge in source_output_connections.items():
        if edge is None:
            continue
        source_sys_id, source_port_id = edge
        if source_sys_id == "input":
            raise TypeError(
                "Flattening diagram outputs sourced directly from external "
                "inputs is not supported by composition shortcuts."
            )
        target_output_id = boundary_output_id(diagram, output_id, output_collision)
        connect_new_output_port_like(
            diagram,
            id_map[source_sys_id],
            source_port_id,
            target_output_id,
            source_outputs[output_id],
        )

    propagate_animation_camera(diagram, source)

    return id_map


def add_system_to_diagram(diagram, sys, *, role=None) -> str:
    """Add ``sys`` under its id, else its role, else its name; a reserved role once per level."""
    if not isinstance(sys, System):
        raise TypeError(f"Expected a System, got {type(sys).__name__}")

    sys_id = default_subsystem_id(sys, role=role)
    check_reserved_id(sys_id, diagram.subsystems)
    sys_id = unique_id(sys_id, diagram.subsystems)
    diagram.add_subsystem(sys, sys_id)

    return sys_id


def carry_left_operand(diagram, left) -> None:
    """Carry a diagram left operand's name, ``x0`` and camera settings into ``diagram``.

    The left operand's blocks come first in the result, so its ``x0`` is the
    head of the stacked state; a leaf left operand carries nothing.
    """
    if not isinstance(left, DiagramSystem):
        return

    # The name, and the left operand's x0 at the head of the stacked state
    diagram.name = left.name
    diagram.x0 = np.concatenate(
        [np.asarray(left.x0, dtype=float), diagram.x0[left.n :]]
    )

    # The camera settings
    diagram.camera_scale = left.camera_scale
    diagram.camera_target = np.array(left.camera_target, dtype=float).copy()
    diagram.camera_plot_axes = left.camera_plot_axes
    diagram.camera_follow_frame = left.camera_follow_frame
    diagram.camera_priority = left.camera_priority
    diagram.scene_grid = left.scene_grid


def as_composition_diagram(sys) -> DiagramSystem:
    """A diagram operand as a copy; a leaf as a diagram with its inputs on the edge."""
    if isinstance(sys, DiagramSystem):
        return copy_diagram(sys)

    diagram = DiagramSystem()
    sys_id = add_system_to_diagram(diagram, sys)
    expose_free_inputs(diagram, sys_id)

    return diagram


def copy_diagram(diagram):
    """A new diagram with the same blocks, ids, wiring, boundary ports, name, ``x0`` and camera.

    The blocks are shared, not copied; the operators build on the copy and leave
    the user's diagram untouched.
    """
    copy = DiagramSystem()
    inline_diagram(copy, diagram, output_collision="replace")
    carry_left_operand(copy, diagram)

    return copy


def new_diagram_like(sys):
    """A diagram operand as a new diagram with the same ids and wiring; a leaf as is."""
    return copy_diagram(sys) if isinstance(sys, DiagramSystem) else sys


def expose_free_inputs(diagram, sys_id: str) -> None:
    """Put every input of a leaf on the diagram's edge, under the same name."""
    sys = diagram.subsystems[sys_id]
    for port_id, port in sys.inputs.items():
        add_input_port_like(diagram, port_id, port)
        diagram.connect("input", port_id, sys_id, port_id)


def add_input_port_like(diagram, port_id: str, port) -> None:
    """Add the boundary input ``port_id`` with ``port``'s size, nominal value, labels, units and bounds."""
    diagram.add_input_port(
        port_id,
        dim=port.dim,
        nominal_value=port.nominal_value,
        labels=port.labels,
        units=port.units,
        lower_bound=port.lower_bound,
        upper_bound=port.upper_bound,
    )


def connect_new_output_port_like(
    diagram,
    source_sys_id: str,
    source_port_id: str,
    output_port_id: str,
    output_port,
) -> None:
    """Expose a block's output as the boundary output ``output_port_id``, with ``output_port``'s metadata."""
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


def boundary_output_id(diagram, output_id: str, collision: str) -> str:
    """The id a flattened boundary output takes: the same (``"replace"``), or a fresh one (``"unique"``)."""
    if collision == "replace":
        return output_id
    if collision == "unique":
        return unique_id(output_id, diagram.outputs)
    raise ValueError(f"Unknown output collision policy {collision!r}")


# The series rule


def series_pair(diagram, right) -> None:
    """Append ``right`` to ``diagram`` (the chain so far) by the ``>>`` rule."""
    # The chain's output and the right side's input the rule connects
    outputs = edge_outputs(diagram)
    source_name, target_name = series_connection(diagram, outputs, right.inputs)
    source_id, source_port = outputs[source_name]

    # The chain's outputs are consumed; the result's output is the right side's
    for output_id in list(diagram.outputs):
        del diagram.outputs[output_id]
        diagram.connections["output"].pop(output_id, None)

    if isinstance(right, DiagramSystem):
        # A box laid out flat, its input bound to the chain's output
        inline_diagram(
            diagram,
            right,
            input_bindings={target_name: (source_id, source_port)},
            output_collision="replace",
        )
    else:
        # A leaf as one block, its default output as the result's y
        right_id = add_system_to_diagram(diagram, right)
        diagram.connect(source_id, source_port, right_id, target_name)
        diagram.connect_new_output_port(right_id, default_output_port(right), "y")
        propagate_animation_camera(diagram, right)


def edge_outputs(diagram) -> dict:
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


def series_connection(diagram, outputs, inputs) -> tuple[str, str]:
    """The ``(output, input)`` names ``>>`` connects, by the first rule that applies."""
    if not outputs:
        raise ValueError("Left side of '>>' has no output port to connect")
    if not inputs:
        raise ValueError("Right side of '>>' has no free input port to connect")

    # One output and one input
    if len(outputs) == 1 and len(inputs) == 1:
        return next(iter(outputs)), next(iter(inputs))

    # One shared name; a source (a chain with no inputs) into a block that reads r
    # and y would feed the measurement, and is refused
    shared = [name for name in outputs if name in inputs]
    if shared == ["y"] and "r" in inputs and not diagram.inputs:
        raise ValueError(
            "a source into a block that reads 'r' and 'y' would drive the measurement "
            "'y'; name the source's signal after the input it drives (port='r') for "
            "the block alone, or feed the loop: source >> (controller @ plant)"
        )
    if len(shared) == 1:
        return shared[0], shared[0]

    # The standard pair: y into u
    if "y" in outputs and "u" in inputs:
        return "y", "u"

    # Else refused, with a hint for a loop's reference
    hint = (
        "; a source for a loop's reference goes in closed_loop(..., reference=source)"
        if "r" in inputs
        else ""
    )
    raise ValueError(
        "'>>' connects one output to one input; wire this one with connect "
        f"(outputs: {', '.join(outputs)}; free inputs: {', '.join(inputs)}){hint}"
    )


# The refusals, in the order the public verbs meet them


def require_diagram(diagram) -> None:
    """Refuse anything but a diagram to wire."""
    if not isinstance(diagram, DiagramSystem):
        raise TypeError("autowire() expects a DiagramSystem")


def require_wired(open_inputs) -> None:
    """Refuse the first input left open: no output of its name and size, or several."""
    if open_inputs:
        raise ValueError(open_inputs[0])


# Default ports


def default_output_port(sys) -> str:
    """The output a block gives by default: ``y``, else its only output."""
    if "y" in sys.outputs:
        return "y"
    if len(sys.outputs) == 1:
        return next(iter(sys.outputs))

    ports = ", ".join(sys.outputs)
    raise ValueError(
        f"Cannot choose a default output port for {sys.name!r}; candidates: {ports}"
    )


# Ids


def default_subsystem_id(sys, *, role=None) -> str:
    """The id a block takes: its ``id``, else its role in a loop, else its class name, verbatim."""
    if sys.id:
        return str(sys.id)
    if role is not None:
        return str(role)

    return base_system_id(sys)


def base_system_id(sys) -> str:
    """The block's class name as an id (``Pendulum``, ``Drone2D``, ``UdeSRacecar``)."""
    return type(sys).__name__


def unique_id(base: str, existing) -> str:
    """``base``, else the first of ``base2``, ``base3``, ... not in ``existing``."""
    if base not in existing:
        return base

    i = 2
    while f"{base}{i}" in existing:
        i += 1

    return f"{base}{i}"


# The camera


def propagate_animation_camera(diagram, *sources):
    """Copy camera hints from a plant-like *source* onto *diagram*.

    Prefer a source (or nested leaf) that already carries kinematic geometry —
    e.g. a catalog plant — over a bare ``Step`` / controller that would keep the
    generic auto-fit default (``camera_scale=None``).
    """
    fallback = None
    for source in sources:
        # The first source with kinematic geometry gives the camera
        animated = animated_geometry_source(source)
        if animated is not None:
            copy_animation_camera_hints(diagram, animated)
            return

        # Else the first leaf, as the fallback
        if (
            fallback is None
            and isinstance(source, System)
            and not isinstance(source, DiagramSystem)
        ):
            fallback = source

    if fallback is not None:
        copy_animation_camera_hints(diagram, fallback)


def animated_geometry_source(obj):
    """First drawable in *obj* (or its subsystems) with static kinematic geometry."""
    if isinstance(obj, DiagramSystem):
        for sub in obj.subsystems.values():
            if sub.get_kinematic_geometry():
                return sub
        return None

    if obj.get_kinematic_geometry():
        return obj
    return None


def copy_animation_camera_hints(diagram, animated):
    """Copy ``animated``'s camera scale, target, plot axes and follow frame onto ``diagram``."""
    diagram.camera_scale = animated.camera_scale
    diagram.camera_target = np.array(animated.camera_target, dtype=float).copy()
    diagram.camera_plot_axes = getattr(animated, "camera_plot_axes", (0, 1))
    diagram.camera_follow_frame = namespaced_camera_follow_frame(
        diagram,
        animated,
        getattr(animated, "camera_follow_frame", None),
    )


def namespaced_camera_follow_frame(
    diagram: DiagramSystem, animated: System, follow: str | None
) -> str | None:
    """The follow frame as ``sys_id:frame`` when ``animated`` is a block of ``diagram``; else as given."""
    if follow is None:
        return None
    if ":" in follow:
        return follow

    sys_id = subsystem_id_for_leaf(diagram, animated)
    if sys_id is None:
        return follow

    return f"{sys_id}:{follow}"


def subsystem_id_for_leaf(diagram: DiagramSystem, leaf: System) -> str | None:
    """Return ``sys_id`` when *leaf* is a direct subsystem of *diagram*."""
    for sys_id, subsystem in diagram.subsystems.items():
        if subsystem is leaf:
            return sys_id
    return None
