"""Backend-neutral hybrid diagram topology views."""

from __future__ import annotations

from dataclasses import dataclass, replace

from minilink.core.hybrid_diagram import HybridDiagram
from minilink.graphical.diagrams.topology import (
    BoundaryPortRef,
    DiagramTopology,
    TopologyEdge,
    build_diagram_topology,
)
from minilink.simulation.computer import StepSchedule

_PLANT_PREFIX = "plant__"
_COMPUTER_PREFIX = "computer__"


@dataclass(frozen=True)
class BoundaryTopologyEdge:
    """One ZOH or sample link across the hybrid boundary."""

    direction: str
    computer_port: str
    plant_port: str
    label: str


@dataclass(frozen=True)
class ComputerClusterMeta:
    """Visual Computer wrapper metadata (not a wiring node)."""

    title: str
    schedule_label: str
    dt_base: float
    fire_summary: tuple[tuple[str, int], ...]


@dataclass(frozen=True)
class HybridTopology:
    """Read-only hybrid topology snapshot for display/export."""

    name: str
    plant: DiagramTopology
    step_diagram: DiagramTopology
    computer: ComputerClusterMeta
    boundary_edges: tuple[BoundaryTopologyEdge, ...]


def build_hybrid_topology(
    hybrid: HybridDiagram, *, abstract_boundary=True
) -> HybridTopology:
    """Build a display/export topology snapshot from a :class:`HybridDiagram`.

    ``abstract_boundary=True`` (default) omits per-side external routing nodes.
    """
    plant = _prefix_topology(
        build_diagram_topology(hybrid.plant, abstract_boundary=abstract_boundary),
        _PLANT_PREFIX,
    )
    step_diagram = _prefix_topology(
        build_diagram_topology(
            hybrid.computer.diagram,
            abstract_boundary=abstract_boundary,
        ),
        _COMPUTER_PREFIX,
    )
    schedule = hybrid.computer.schedule
    fire_summary = tuple(sorted(schedule.fire.items()))
    computer = ComputerClusterMeta(
        title="Computer",
        schedule_label=_format_schedule_label(schedule),
        dt_base=schedule.dt_base,
        fire_summary=fire_summary,
    )
    boundary_edges = tuple(_boundary_edge(conn) for conn in hybrid.connections)
    return HybridTopology(
        name=hybrid.plant.name,
        plant=plant,
        step_diagram=step_diagram,
        computer=computer,
        boundary_edges=boundary_edges,
    )


def export_hybrid_topology(
    hybrid: HybridDiagram,
    *,
    backend="graphviz",
    abstract_boundary=True,
    **kwargs,
):
    """Export a hybrid diagram topology with the selected backend."""
    topology = build_hybrid_topology(hybrid, abstract_boundary=abstract_boundary)
    key = backend.strip().lower() if isinstance(backend, str) else backend
    if key == "graphviz":
        from minilink.graphical.diagrams.hybrid_dot import export_hybrid_graphviz

        return export_hybrid_graphviz(topology, **kwargs)
    if key == "mermaid":
        from minilink.graphical.diagrams.hybrid_mermaid import export_hybrid_mermaid

        return export_hybrid_mermaid(topology, **kwargs)
    raise ValueError(
        "Unknown hybrid topology backend {!r}. Expected 'graphviz' or 'mermaid'.".format(
            backend
        )
    )


def _prefix_topology(topology: DiagramTopology, prefix: str) -> DiagramTopology:
    def pid(node_id: str) -> str:
        return f"{prefix}{node_id}"

    nodes = tuple(replace(node, id=pid(node.id)) for node in topology.nodes)
    edges = tuple(
        TopologyEdge(
            source_node=pid(edge.source_node),
            source_port=edge.source_port,
            target_node=pid(edge.target_node),
            target_port=edge.target_port,
        )
        for edge in topology.edges
    )
    boundary_inputs = tuple(
        BoundaryPortRef(
            diagram_port=ref.diagram_port,
            node_id=pid(ref.node_id),
            port_id=ref.port_id,
        )
        for ref in topology.boundary_inputs
    )
    boundary_outputs = tuple(
        BoundaryPortRef(
            diagram_port=ref.diagram_port,
            node_id=pid(ref.node_id),
            port_id=ref.port_id,
        )
        for ref in topology.boundary_outputs
    )
    return DiagramTopology(
        name=topology.name,
        nodes=nodes,
        edges=edges,
        boundary_inputs=boundary_inputs,
        boundary_outputs=boundary_outputs,
    )


def _format_schedule_label(schedule: StepSchedule) -> str:
    parts = [f"Computer | dt_base={schedule.dt_base} s"]
    if schedule.fire:
        base_hz = 1.0 / schedule.dt_base
        for sys_id, divisor in sorted(schedule.fire.items()):
            rate_hz = base_hz / divisor
            parts.append(f"{sys_id}@{rate_hz:g} Hz")
    return " | ".join(parts)


def _boundary_edge(conn) -> BoundaryTopologyEdge:
    if conn.direction == "computer_to_plant":
        label = "ZOH"
    else:
        label = "sample"
    return BoundaryTopologyEdge(
        direction=conn.direction,
        computer_port=conn.computer_port,
        plant_port=conn.plant_port,
        label=label,
    )


def resolve_computer_boundary_anchor(
    step_diagram: DiagramTopology,
    *,
    direction: str,
    computer_port: str,
) -> tuple[str, str]:
    """Return ``(node_id, port_id)`` for one computer-side hybrid boundary port."""
    if direction == "computer_to_plant":
        return _resolve_boundary_port(
            step_diagram.boundary_outputs,
            computer_port,
            fallback_node=f"{_COMPUTER_PREFIX}output",
        )
    return _resolve_boundary_port(
        step_diagram.boundary_inputs,
        computer_port,
        fallback_node=f"{_COMPUTER_PREFIX}input",
    )


def resolve_plant_boundary_anchor(
    plant: DiagramTopology,
    *,
    direction: str,
    plant_port: str,
) -> tuple[str, str]:
    """Return ``(node_id, port_id)`` for one plant-side hybrid boundary port."""
    if direction == "computer_to_plant":
        return _resolve_boundary_port(
            plant.boundary_inputs,
            plant_port,
            fallback_node=f"{_PLANT_PREFIX}input",
        )
    return _resolve_boundary_port(
        plant.boundary_outputs,
        plant_port,
        fallback_node=f"{_PLANT_PREFIX}output",
    )


def _resolve_boundary_port(
    refs, diagram_port: str, *, fallback_node: str
) -> tuple[str, str]:
    for ref in refs:
        if ref.diagram_port == diagram_port:
            return ref.node_id, ref.port_id
    return fallback_node, diagram_port
