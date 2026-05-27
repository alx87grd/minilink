"""Convenience composition helpers for common diagram topologies."""

from __future__ import annotations

import re
from collections.abc import Iterable

import numpy as np


def add_systems(*systems: "System | DiagramSystem") -> "DiagramSystem":
    """Return a diagram containing ``systems`` as subsystems with no wiring.

    Parameters
    ----------
    *systems : System or DiagramSystem
        Systems to add. Passing an existing diagram as the first argument extends
        that diagram in place.

    Returns
    -------
    DiagramSystem
        A diagram containing all provided systems as subsystems. No connections
        are inferred.
    """
    from minilink.core.diagram import DiagramSystem

    if len(systems) == 1 and isinstance(systems[0], DiagramSystem):
        return systems[0]

    diagram = None
    for sys in systems:
        if isinstance(sys, DiagramSystem):
            if diagram is None:
                diagram = sys
                _ensure_composition_state(diagram)
                continue
            raise TypeError(
                "Merging two DiagramSystem objects is not supported by '+'. "
                "Use explicit DiagramSystem wiring for that topology."
            )

        if diagram is None:
            diagram = _new_diagram()
        _add_system_to_diagram(diagram, sys)

    return _new_diagram() if diagram is None else diagram


def series(
    left: "System | DiagramSystem",
    right: "System",
    *rest: "System",
) -> "DiagramSystem":
    """Connect systems in series using default output and input ports.

    Parameters
    ----------
    left : System or DiagramSystem
        The first system or an existing shortcut-built diagram.
    right : System
        System to connect after ``left``.
    *rest : System
        Additional systems to append to the series chain.

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
    ref_port: str = "ref",
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
    _require_output(controller, control_port, f"controller output port {control_port!r}")
    _require_input(plant, plant_input_port, f"plant input port {plant_input_port!r}")
    _require_output(plant, plant_output_port, f"plant output port {plant_output_port!r}")

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
    diagram.add_input_port(ref.dim, ref_port, nominal_value=ref.nominal_value)
    _copy_port_metadata(ref, diagram.inputs[ref_port])

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
    - source-like ``y`` output to a ``ref`` input;
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
    diagram.graphe_building_verbose = False
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

    if isinstance(right, DiagramSystem):
        raise TypeError(
            "Connecting into an existing DiagramSystem with '>>' is not supported yet. "
            "Use explicit DiagramSystem wiring for that topology."
        )

    source = _get_composition_output(diagram)
    if source is None:
        raise ValueError("Left side of '>>' has no output port to connect")

    right_id = _add_system_to_diagram(diagram, right)
    target_port = _default_input_port(right)
    source_id, source_port = source

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


def _get_composition_output(diagram):
    _ensure_composition_state(diagram)
    output = diagram._composition_output
    if output is not None:
        return output

    for sys_id in reversed(diagram._composition_system_order):
        sys = diagram.subsystems[sys_id]
        if sys.outputs:
            return sys_id, _default_output_port(sys)
    return None


def _expose_entry_input(diagram, sys_id: str) -> None:
    sys = diagram.subsystems[sys_id]
    if not sys.inputs:
        return

    port_id = _default_input_port(sys)
    port = sys.inputs[port_id]
    diagram.add_input_port(port.dim, port_id, nominal_value=port.nominal_value)
    _copy_port_metadata(port, diagram.inputs[port_id])
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
    for preferred in ("u", "ref"):
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


def _copy_port_metadata(source, target) -> None:
    target.labels = list(source.labels)
    target.units = list(source.units)
    target.upper_bound = np.asarray(source.upper_bound, dtype=float).copy()
    target.lower_bound = np.asarray(source.lower_bound, dtype=float).copy()


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
    elif input_id == "ref":
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
