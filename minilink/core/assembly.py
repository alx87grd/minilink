"""Diagram assembly: operands laid out flat into a new diagram, with their ids, default ports and camera."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.roles import check_reserved_id
from minilink.core.system import System
from minilink.core.wiring import check_diagram_params, state_layout

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
    id_prefix: str = "",
    boundary_outputs: bool = True,
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
        target_id = unique_id(f"{id_prefix}{source_id}", diagram.subsystems)
        if not id_prefix:
            check_reserved_id(source_id, diagram.subsystems)
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
    if boundary_outputs:
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


@dataclass(frozen=True)
class FlattenReport:
    """How :func:`flatten_diagram` renamed nested blocks (``parent/child`` → ``parent__child``)."""

    id_map: dict[str, str]


def flatten_diagram(diagram: DiagramSystem) -> tuple[DiagramSystem, FlattenReport]:
    """Return a single-level copy of ``diagram`` with nested boxes inlined.

    Nested block ids join with ``__`` (``inner__plant``), matching topology
    expand. The stacked state ``x = [x_1; x_2; \\ldots]`` (block order from
    :func:`~minilink.core.wiring.state_layout`) matches the unflattened diagram.
    The input diagram is not modified.
    """
    if not isinstance(diagram, DiagramSystem):
        raise TypeError(
            f"flatten_diagram expects DiagramSystem, got {type(diagram).__name__}"
        )

    flat = copy_diagram(diagram)
    id_map: dict[str, str] = {}

    while True:
        nested_id = _first_nested_subsystem_id(flat)
        if nested_id is None:
            break
        inner_flat, _inner_report = flatten_diagram(flat.subsystems[nested_id])
        local_map = _replace_nested_subsystem(flat, nested_id, inner_flat)
        for source_id, target_id in local_map.items():
            id_map[f"{nested_id}/{source_id}"] = target_id

    for sys_id in flat.subsystems:
        id_map.setdefault(sys_id, sys_id)

    return flat, FlattenReport(id_map=id_map)


def remap_params_nested_to_flat(diagram: DiagramSystem, params) -> dict | None:
    """Remap a nested diagram params tree to flat ``__`` block ids for :func:`flatten_diagram`."""
    if params is None:
        return None

    check_diagram_params(params, diagram.subsystems)
    flat, _report = flatten_diagram(diagram)
    remapped = _remap_params_tree(diagram, params, prefix="")
    check_diagram_params(remapped, flat.subsystems)
    return remapped


def _first_nested_subsystem_id(diagram: DiagramSystem) -> str | None:
    for sys_id, subsystem in diagram.subsystems.items():
        if isinstance(subsystem, DiagramSystem):
            return sys_id
    return None


def _reorder_subsystems_after_inline(
    diagram: DiagramSystem,
    order_before: list[str],
    nested_id: str,
    inner_flat: DiagramSystem,
    local_map: dict[str, str],
) -> None:
    """Restore subsystem insertion order after :func:`inline_diagram` appends leaves.

    ``state_layout`` stacks ``x`` in subsystem order; inlined blocks must occupy
    the index where the nested box ``nested_id`` sat in ``order_before``.
    """
    new_order: list[str] = []
    for sid in order_before:
        if sid == nested_id:
            for inner_id in inner_flat.subsystems:
                new_order.append(local_map[inner_id])
        elif sid in diagram.subsystems:
            new_order.append(sid)

    diagram.subsystems = {key: diagram.subsystems[key] for key in new_order}

    output_connections = diagram.connections.get("output")
    diagram.connections = {
        key: diagram.connections[key] for key in new_order if key in diagram.connections
    }
    if output_connections is not None:
        diagram.connections["output"] = output_connections


def _replace_nested_subsystem(
    diagram: DiagramSystem,
    nested_id: str,
    inner_flat: DiagramSystem,
) -> dict[str, str]:
    """Inline ``inner_flat`` in place of nested block ``nested_id`` on ``diagram``."""
    order_before = list(diagram.subsystems.keys())
    input_bindings = {
        port_id: edge
        for port_id, edge in diagram.connections[nested_id].items()
        if edge is not None
    }
    outbound = []
    for target_sys_id, port_map in diagram.connections.items():
        if target_sys_id in (nested_id, "output"):
            continue
        for target_port_id, edge in port_map.items():
            if edge is not None and edge[0] == nested_id:
                outbound.append((target_sys_id, target_port_id, edge[1]))

    local_map = inline_diagram(
        diagram,
        inner_flat,
        input_bindings=input_bindings,
        output_collision="replace",
        id_prefix=f"{nested_id}__",
        boundary_outputs=False,
    )
    for target_sys_id, target_port_id, nested_port_id in outbound:
        inner_edge = inner_flat.connections["output"].get(nested_port_id)
        if inner_edge is None:
            raise ValueError(
                f"Nested block {nested_id!r} output port {nested_port_id!r} is not "
                "exposed on the inner diagram boundary"
            )
        source_sys_id, source_port_id = inner_edge
        if source_sys_id == "input":
            raise TypeError(
                "Flattening nested diagram outputs sourced directly from external "
                "inputs is not supported."
            )
        diagram.connect(
            local_map[source_sys_id],
            source_port_id,
            target_sys_id,
            target_port_id,
        )

    del diagram.subsystems[nested_id]
    del diagram.connections[nested_id]
    # x = [x_block1; x_block2; …] follows subsystem order; inline_diagram appends.
    _reorder_subsystems_after_inline(
        diagram, order_before, nested_id, inner_flat, local_map
    )
    diagram.n, diagram.state_index, diagram.state, diagram.x0, diagram.derived_x0 = (
        state_layout(diagram.subsystems)
    )
    diagram.refresh_solver_info()
    return local_map


def _remap_params_tree(diagram: DiagramSystem, params: Mapping, *, prefix: str) -> dict:
    remapped = {}
    for sys_id, subsystem in diagram.subsystems.items():
        flat_id = f"{prefix}__{sys_id}" if prefix else sys_id
        piece = params[sys_id]
        if isinstance(subsystem, DiagramSystem):
            remapped.update(_remap_params_tree(subsystem, piece, prefix=flat_id))
        else:
            remapped[flat_id] = piece
    return remapped


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
