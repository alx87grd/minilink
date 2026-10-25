"""Diagram assembly: operands laid out flat into a new diagram, with their ids, default ports and camera."""

from __future__ import annotations

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.roles import check_reserved_id
from minilink.core.system import System

###############################################################################
# Public API
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
