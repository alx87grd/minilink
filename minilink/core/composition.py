"""Convenience composition helpers for common diagram topologies."""

from __future__ import annotations

import re
from collections.abc import Iterable
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem
    from minilink.core.system import System


@dataclass(frozen=True)
class _DiagramEntry:
    sys_id: str
    port_id: str
    dim: int
    boundary_port_id: str | None = None


def add_systems(*systems: "System | DiagramSystem") -> "DiagramSystem":
    """Return a flat diagram containing ``systems`` as subsystems.

    Parameters
    ----------
    *systems : System or DiagramSystem
        Systems or diagrams to add. Diagram operands are flattened into the
        result; passing an existing diagram as the first argument extends that
        diagram in place.

    Returns
    -------
    DiagramSystem
        A diagram containing all provided systems as direct subsystems.
        Existing diagram-internal connections are preserved, but no new
        cross-connections are inferred.
    """
    from minilink.core.diagram import DiagramSystem

    if len(systems) == 1 and isinstance(systems[0], DiagramSystem):
        return systems[0]

    diagram = None
    for sys in systems:
        if diagram is None:
            if isinstance(sys, DiagramSystem):
                diagram = sys
                _ensure_composition_state(diagram)
            else:
                diagram = _new_diagram()
                _add_system_to_diagram(diagram, sys)
            continue

        if isinstance(sys, DiagramSystem):
            _inline_diagram(diagram, sys, output_collision="unique")
        else:
            _add_system_to_diagram(diagram, sys)

    return _new_diagram() if diagram is None else diagram


def series(
    left: "System | DiagramSystem",
    right: "System | DiagramSystem",
    *rest: "System | DiagramSystem",
) -> "DiagramSystem":
    """Connect systems in series using default output and input ports.

    Parameters
    ----------
    left : System or DiagramSystem
        The first system or an existing shortcut-built diagram.
    right : System or DiagramSystem
        System or diagram to connect after ``left``.
    *rest : System or DiagramSystem
        Additional systems or diagrams to append to the series chain.

    Returns
    -------
    DiagramSystem
        A normal diagram with each stage connected output-to-input.
    """
    diagram = _as_composition_diagram(left, expose_entry=True)
    _series_pair(diagram, right)
    for next_sys in rest:
        _series_pair(diagram, next_sys)
    return diagram


def closed_loop(
    controller: "System",
    plant: "System",
    *,
    ref_port: str = "r",
    measurement_port: str = "y",
    control_port: str = "u",
    plant_input_port: str = "u",
    plant_output_port: str = "y",
    output_port: str = "y",
    validate: bool = True,
) -> "DiagramSystem":
    """Build a Pyro-style controller/plant feedback diagram.

    Parameters
    ----------
    controller : System
        Controller subsystem with reference, measurement, and control ports.
    plant : System
        Plant subsystem with control input and measurement output ports.
    ref_port, measurement_port, control_port, plant_input_port, plant_output_port : str
        Port names used for the standard feedback wiring.
    output_port : str, optional
        Diagram boundary output name.
    validate : bool, optional
        If ``True``, run algebraic-loop detection before returning.

    Returns
    -------
    DiagramSystem
        A diagram exposing the controller reference as input and plant output as
        output.
    """
    diagram = _new_diagram()
    controller_id = _add_system_to_diagram(diagram, controller)
    plant_id = _add_system_to_diagram(diagram, plant)

    _require_input(controller, ref_port, f"controller reference port {ref_port!r}")
    _require_input(
        controller,
        measurement_port,
        f"controller measurement port {measurement_port!r}",
    )
    _require_output(
        controller, control_port, f"controller output port {control_port!r}"
    )
    _require_input(plant, plant_input_port, f"plant input port {plant_input_port!r}")
    _require_output(
        plant, plant_output_port, f"plant output port {plant_output_port!r}"
    )

    _require_same_dim(
        controller.outputs[control_port].dim,
        plant.inputs[plant_input_port].dim,
        f"{controller_id}:{control_port}",
        f"{plant_id}:{plant_input_port}",
    )
    _require_same_dim(
        plant.outputs[plant_output_port].dim,
        controller.inputs[measurement_port].dim,
        f"{plant_id}:{plant_output_port}",
        f"{controller_id}:{measurement_port}",
    )

    ref = controller.inputs[ref_port]
    diagram.add_input_port(
        ref_port,
        dim=ref.dim,
        nominal_value=ref.nominal_value,
        labels=ref.labels,
        units=ref.units,
        lower_bound=ref.lower_bound,
        upper_bound=ref.upper_bound,
    )

    diagram.connect("input", ref_port, controller_id, ref_port)
    diagram.connect(plant_id, plant_output_port, controller_id, measurement_port)
    diagram.connect(controller_id, control_port, plant_id, plant_input_port)
    diagram.connect_new_output_port(plant_id, plant_output_port, output_port)

    diagram.name = f"Closed loop {plant.name} with {controller.name}"
    _set_composition_entry(diagram, controller_id, ref_port)
    _set_composition_output(diagram, plant_id, plant_output_port)

    if validate:
        diagram.check_algebraic_loops()
    return diagram


def autowire(
    diagram: "DiagramSystem",
    *,
    strict: bool = False,
    validate: bool = True,
) -> "DiagramSystem":
    """Fill unconnected diagram inputs when exactly one safe source matches.

    Parameters
    ----------
    diagram : DiagramSystem
        Diagram to modify in place.
    strict : bool, optional
        If ``True``, raise when any unconnected input has multiple candidates.
        No new connections are applied before that error is raised.
    validate : bool, optional
        If ``True``, run algebraic-loop detection after wiring.

    Returns
    -------
    DiagramSystem
        The same diagram, for fluent use.

    Notes
    -----
    The rules are conservative and never overwrite existing connections:

    - exact same port name and dimension, except generic measurement ``y``;
    - source-like ``y`` output to a ``r`` input;
    - non-source-like ``y`` output to a measurement ``y`` input.
    """
    from minilink.core.diagram import DiagramSystem

    if not isinstance(diagram, DiagramSystem):
        raise TypeError("autowire() expects a DiagramSystem")

    decisions = []
    ambiguities = []
    for target_id, target in diagram.subsystems.items():
        for input_id, input_port in target.inputs.items():
            if diagram.connections[target_id][input_id] is not None:
                continue

            candidates = _autowire_candidates(diagram, target_id, input_id, input_port)
            if len(candidates) == 1:
                source_id, source_port = candidates[0]
                decisions.append((source_id, source_port, target_id, input_id))
            elif len(candidates) > 1 and strict:
                ambiguities.append((target_id, input_id, candidates))

    if ambiguities:
        target_id, input_id, candidates = ambiguities[0]
        rendered = ", ".join(f"{sys_id}:{port_id}" for sys_id, port_id in candidates)
        raise ValueError(
            f"Ambiguous autowire target {target_id}:{input_id}; "
            f"candidates: {rendered}"
        )

    for source_id, source_port, target_id, input_id in decisions:
        diagram.connect(source_id, source_port, target_id, input_id)

    if validate:
        diagram.check_algebraic_loops()
    return diagram


def _new_diagram():
    from minilink.core.diagram import DiagramSystem

    diagram = DiagramSystem()
    diagram.connection_verbose = False
    _ensure_composition_state(diagram)
    return diagram


def _as_composition_diagram(sys, *, expose_entry: bool):
    from minilink.core.diagram import DiagramSystem

    if isinstance(sys, DiagramSystem):
        _ensure_composition_state(sys)
        return sys

    diagram = _new_diagram()
    sys_id = _add_system_to_diagram(diagram, sys)
    if expose_entry:
        _expose_entry_input(diagram, sys_id)
    return diagram


def _series_pair(diagram, right):
    from minilink.core.diagram import DiagramSystem

    source = _get_composition_output(diagram)
    if source is None:
        raise ValueError("Left side of '>>' has no output port to connect")
    source_id, source_port = source

    if isinstance(right, DiagramSystem):
        entry = _get_available_diagram_entry(right)
        _require_same_dim(
            diagram.subsystems[source_id].outputs[source_port].dim,
            entry.dim,
            f"{source_id}:{source_port}",
            _entry_label(entry),
        )
        bindings = {}
        if entry.boundary_port_id is not None:
            bindings[entry.boundary_port_id] = source
        result = _inline_diagram(
            diagram,
            right,
            input_bindings=bindings,
            output_collision="replace",
        )
        if entry.boundary_port_id is None:
            diagram.connect(
                source_id,
                source_port,
                result[entry.sys_id],
                entry.port_id,
            )
        return diagram

    right_id = _add_system_to_diagram(diagram, right)
    target_port = _default_input_port(right)

    _require_same_dim(
        diagram.subsystems[source_id].outputs[source_port].dim,
        right.inputs[target_port].dim,
        f"{source_id}:{source_port}",
        f"{right_id}:{target_port}",
    )
    diagram.connect(source_id, source_port, right_id, target_port)
    output_port = _default_output_port(right)
    diagram.connect_new_output_port(right_id, output_port, "y")
    _set_composition_output(diagram, right_id, output_port)
    return diagram


def _inline_diagram(
    diagram,
    source,
    *,
    input_bindings=None,
    output_collision: str,
) -> dict[str, str]:
    _ensure_composition_state(diagram)
    _ensure_composition_state(source)
    input_bindings = {} if input_bindings is None else dict(input_bindings)
    source_subsystems = list(source.subsystems.items())
    source_inputs = list(source.inputs.items())
    source_connections = {
        source_id: dict(source.connections[source_id])
        for source_id, _subsystem in source_subsystems
    }
    source_output_connections = dict(source.connections.get("output", {}))
    source_outputs = dict(source.outputs)
    source_entry = _get_composition_entry(source)
    source_output = _get_composition_output(source)

    id_map = {}
    for source_id, subsystem in source_subsystems:
        target_id = _unique_id(source_id, diagram.subsystems)
        diagram.add_subsystem(subsystem, target_id)
        diagram._composition_system_order.append(target_id)
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

    if diagram._composition_entry is None and source_entry is not None:
        entry_sys_id, entry_port_id = source_entry
        _set_composition_entry(diagram, id_map[entry_sys_id], entry_port_id)

    if source_output is not None:
        output_sys_id, output_port_id = source_output
        _set_composition_output(diagram, id_map[output_sys_id], output_port_id)

    return id_map


def _add_system_to_diagram(diagram, sys) -> str:
    from minilink.core.diagram import DiagramSystem
    from minilink.core.system import System

    if isinstance(sys, DiagramSystem):
        raise TypeError(
            "Nested DiagramSystem operands are not supported by this shortcut. "
            "Use explicit DiagramSystem wiring for nested diagrams."
        )
    if not isinstance(sys, System):
        raise TypeError(f"Expected a System, got {type(sys).__name__}")

    _ensure_composition_state(diagram)
    sys_id = _unique_id(_base_system_id(sys), diagram.subsystems)
    diagram.add_subsystem(sys, sys_id)
    diagram._composition_system_order.append(sys_id)

    if sys.inputs and diagram._composition_entry is None:
        try:
            _set_composition_entry(diagram, sys_id, _default_input_port(sys))
        except ValueError:
            pass
    if sys.outputs:
        try:
            _set_composition_output(diagram, sys_id, _default_output_port(sys))
        except ValueError:
            pass
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
    def compute(x, u, t, params=None):
        return diagram.compute_subsys_output_port(
            x, u, t, source_sys_id, source_port_id, params=params
        )

    diagram.add_output_port(
        output_port_id,
        dim=output_port.dim,
        function=compute,
        dependencies="all",
        nominal_value=output_port.nominal_value,
        labels=output_port.labels,
        units=output_port.units,
        lower_bound=output_port.lower_bound,
        upper_bound=output_port.upper_bound,
    )

    if "output" not in diagram.connections:
        diagram.connections["output"] = {}

    diagram.connect(source_sys_id, source_port_id, "output", output_port_id)


def _output_port_id(diagram, output_id: str, collision: str) -> str:
    if collision == "replace":
        return output_id
    if collision == "unique":
        return _unique_id(output_id, diagram.outputs)
    raise ValueError(f"Unknown output collision policy {collision!r}")


def _ensure_composition_state(diagram) -> None:
    if not hasattr(diagram, "_composition_system_order"):
        diagram._composition_system_order = []
    if not hasattr(diagram, "_composition_entry"):
        diagram._composition_entry = None
    if not hasattr(diagram, "_composition_output"):
        diagram._composition_output = None


def _set_composition_entry(diagram, sys_id: str, port_id: str) -> None:
    diagram._composition_entry = (sys_id, port_id)


def _set_composition_output(diagram, sys_id: str, port_id: str) -> None:
    diagram._composition_output = (sys_id, port_id)


def _get_composition_entry(diagram):
    _ensure_composition_state(diagram)
    entry = diagram._composition_entry
    if entry is not None:
        sys_id, port_id = entry
        if (
            sys_id in diagram.subsystems
            and port_id in diagram.subsystems[sys_id].inputs
        ):
            return entry
    return None


def _get_composition_output(diagram):
    _ensure_composition_state(diagram)
    output = diagram._composition_output
    if output is not None:
        return output

    for sys_id in reversed(diagram._composition_system_order):
        sys = diagram.subsystems[sys_id]
        if sys.outputs:
            return sys_id, _default_output_port(sys)

    boundary_outputs = []
    if "output" in diagram.connections:
        for output_id in diagram.outputs:
            edge = diagram.connections["output"].get(output_id)
            if edge is not None and edge[0] != "input":
                boundary_outputs.append(edge)
    if "y" in diagram.outputs:
        edge = diagram.connections.get("output", {}).get("y")
        if edge is not None and edge[0] != "input":
            return edge
    if len(boundary_outputs) == 1:
        return boundary_outputs[0]
    return None


def _get_available_diagram_entry(diagram) -> _DiagramEntry:
    entry = _get_composition_entry(diagram)
    if entry is not None:
        sys_id, port_id = entry
        target_port = diagram.subsystems[sys_id].inputs[port_id]
        edge = diagram.connections[sys_id][port_id]
        if edge is None:
            return _DiagramEntry(sys_id, port_id, target_port.dim)
        if edge[0] == "input":
            boundary_port_id = edge[1]
            return _DiagramEntry(
                sys_id,
                port_id,
                diagram.inputs[boundary_port_id].dim,
                boundary_port_id,
            )
        raise ValueError(
            "Right side of '>>' has no available boundary input; "
            f"composition entry {sys_id}:{port_id} is already connected. "
            "Use explicit DiagramSystem.connect for this topology."
        )

    boundary_inputs = list(diagram.inputs)
    if len(boundary_inputs) != 1:
        raise ValueError(
            "Right side of '>>' must have exactly one composition entry or "
            "exposed boundary input. Use explicit DiagramSystem.connect for "
            "this topology."
        )

    boundary_port_id = boundary_inputs[0]
    targets = _boundary_input_targets(diagram, boundary_port_id)
    if len(targets) == 0:
        raise ValueError(
            "Right side of '>>' has an exposed boundary input that is not "
            "connected to any subsystem input."
        )
    sys_id, port_id = targets[0]
    return _DiagramEntry(
        sys_id,
        port_id,
        diagram.inputs[boundary_port_id].dim,
        boundary_port_id,
    )


def _boundary_input_targets(diagram, boundary_port_id: str) -> list[tuple[str, str]]:
    targets = []
    for sys_id, subsystem in diagram.subsystems.items():
        for port_id in subsystem.inputs:
            if diagram.connections[sys_id][port_id] == ("input", boundary_port_id):
                targets.append((sys_id, port_id))
    return targets


def _entry_label(entry: _DiagramEntry) -> str:
    if entry.boundary_port_id is not None:
        return f"input:{entry.boundary_port_id}"
    return f"{entry.sys_id}:{entry.port_id}"


def _expose_entry_input(diagram, sys_id: str) -> None:
    sys = diagram.subsystems[sys_id]
    if not sys.inputs:
        return

    port_id = _default_input_port(sys)
    port = sys.inputs[port_id]
    diagram.add_input_port(
        port_id,
        dim=port.dim,
        nominal_value=port.nominal_value,
        labels=port.labels,
        units=port.units,
        lower_bound=port.lower_bound,
        upper_bound=port.upper_bound,
    )
    diagram.connect("input", port_id, sys_id, port_id)
    _set_composition_entry(diagram, sys_id, port_id)


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


def _require_input(sys, port_id: str, label: str) -> None:
    if port_id not in sys.inputs:
        ports = ", ".join(sys.inputs)
        raise ValueError(f"Missing {label}; available input ports: {ports}")


def _require_output(sys, port_id: str, label: str) -> None:
    if port_id not in sys.outputs:
        ports = ", ".join(sys.outputs)
        raise ValueError(f"Missing {label}; available output ports: {ports}")


def _require_same_dim(left_dim: int, right_dim: int, left_label: str, right_label: str):
    if left_dim != right_dim:
        raise ValueError(
            f"Port dimension mismatch: {left_label} has dim {left_dim}, "
            f"{right_label} has dim {right_dim}"
        )


def _autowire_candidates(diagram, target_id: str, input_id: str, input_port):
    candidates = list(_matching_exact_outputs(diagram, target_id, input_id, input_port))
    if input_id == "y":
        candidates = [
            (sys_id, port_id)
            for sys_id, port_id in candidates
            if not _is_source_like(diagram.subsystems[sys_id])
        ]
        candidates.extend(
            _matching_named_y_outputs(
                diagram,
                target_id,
                input_port,
                source_like=False,
                exclude=candidates,
            )
        )
    elif input_id == "r":
        candidates.extend(
            _matching_named_y_outputs(
                diagram,
                target_id,
                input_port,
                source_like=True,
                exclude=candidates,
            )
        )
    return _dedupe(candidates)


def _matching_exact_outputs(diagram, target_id: str, input_id: str, input_port):
    if input_id == "y":
        return []
    for source_id, _source, port_id, output in _iter_output_ports(diagram, target_id):
        if port_id == input_id and output.dim == input_port.dim:
            yield source_id, port_id


def _matching_named_y_outputs(
    diagram,
    target_id: str,
    input_port,
    *,
    source_like: bool,
    exclude: Iterable[tuple[str, str]],
):
    excluded = set(exclude)
    for source_id, source, port_id, output in _iter_output_ports(diagram, target_id):
        if (source_id, port_id) in excluded:
            continue
        if port_id != "y" or output.dim != input_port.dim:
            continue
        if _is_source_like(source) == source_like:
            yield source_id, port_id


def _iter_output_ports(diagram, target_id: str):
    for source_id, source in diagram.subsystems.items():
        if source_id == target_id:
            continue
        for port_id, output in source.outputs.items():
            yield source_id, source, port_id, output


def _is_source_like(sys) -> bool:
    return sys.n == 0 and len(sys.inputs) == 0


def _dedupe(candidates):
    seen = set()
    result = []
    for candidate in candidates:
        if candidate in seen:
            continue
        seen.add(candidate)
        result.append(candidate)
    return result


def _base_system_id(sys) -> str:
    raw = getattr(sys, "name", "") or sys.__class__.__name__
    if raw in {"System", "DynamicSystem", "StaticSystem"}:
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
    while f"{base}_{i}" in existing:
        i += 1
    return f"{base}_{i}"
