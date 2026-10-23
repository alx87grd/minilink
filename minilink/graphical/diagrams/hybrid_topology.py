"""Hybrid topology: the plant, the computer and the wires across their boundary."""

from __future__ import annotations

from dataclasses import dataclass

from minilink.core.hybrid_diagram import HybridDiagram
from minilink.graphical.diagrams.topology import (
    DiagramTopology,
    build_diagram_topology,
    prefix_topology,
)
from minilink.simulation.computer import StepSchedule

# Each side's node ids carry its prefix, so the two diagrams share one graph
_PLANT_PREFIX = "plant__"
_COMPUTER_PREFIX = "computer__"


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class HybridTopology:
    """Read-only hybrid topology snapshot for display/export."""

    name: str
    plant: DiagramTopology
    step_diagram: DiagramTopology
    computer: ComputerClusterMeta
    boundary_edges: tuple[BoundaryTopologyEdge, ...]


@dataclass(frozen=True)
class ComputerClusterMeta:
    """Visual Computer wrapper metadata (not a wiring node)."""

    title: str
    schedule_label: str
    dt_base: float
    fire_summary: tuple[tuple[str, int], ...]


@dataclass(frozen=True)
class BoundaryTopologyEdge:
    """One ZOH or sample link across the hybrid boundary."""

    direction: str
    computer_port: str
    plant_port: str
    label: str


def build_hybrid_topology(
    hybrid: HybridDiagram, *, abstract_boundary=True
) -> HybridTopology:
    """Build a display/export topology snapshot from a :class:`HybridDiagram`.

    ``abstract_boundary=True`` (default) omits per-side external routing nodes.
    """
    # Each side's own topology, its node ids prefixed by the side
    plant = prefix_topology(
        build_diagram_topology(hybrid.plant, abstract_boundary=abstract_boundary),
        _PLANT_PREFIX,
    )
    step_diagram = prefix_topology(
        build_diagram_topology(
            hybrid.computer.diagram,
            abstract_boundary=abstract_boundary,
        ),
        _COMPUTER_PREFIX,
    )

    # The computer's box: its base period and each block's firing divisor
    schedule = hybrid.computer.schedule
    fire_summary = tuple(sorted(schedule.fire.items()))
    computer = ComputerClusterMeta(
        title="Computer",
        schedule_label=format_schedule_label(schedule),
        dt_base=schedule.dt_base,
        fire_summary=fire_summary,
    )

    # One edge per wire across the boundary
    boundary_edges = tuple(boundary_edge(conn) for conn in hybrid.connections)

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

    # The backend's name, read without case or surrounding spaces
    key = backend.strip().lower() if isinstance(backend, str) else backend
    if key == "graphviz":
        # A real cycle: hybrid_dot imports the boundary anchors from this module
        from minilink.graphical.diagrams.hybrid_dot import export_hybrid_graphviz

        return export_hybrid_graphviz(topology, **kwargs)

    raise ValueError(
        f"Unknown hybrid topology backend {backend!r}. Expected 'graphviz'."
    )


# The boundary anchors the exporters wire to


def resolve_computer_boundary_anchor(
    step_diagram: DiagramTopology,
    *,
    direction: str,
    computer_port: str,
) -> tuple[str, str]:
    """Return ``(node_id, port_id)`` for one computer-side hybrid boundary port."""
    # A ZOH leaves the step diagram at an output; a sample enters it at an input
    if direction == "computer_to_plant":
        return resolve_boundary_port(
            step_diagram.boundary_outputs,
            computer_port,
            fallback_node=f"{_COMPUTER_PREFIX}output",
        )
    return resolve_boundary_port(
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
    # A ZOH enters the plant at an input; a sample leaves it at an output
    if direction == "computer_to_plant":
        return resolve_boundary_port(
            plant.boundary_inputs,
            plant_port,
            fallback_node=f"{_PLANT_PREFIX}input",
        )
    return resolve_boundary_port(
        plant.boundary_outputs,
        plant_port,
        fallback_node=f"{_PLANT_PREFIX}output",
    )


###############################################################################
# Internal machinery
###############################################################################


def format_schedule_label(schedule: StepSchedule) -> str:
    """The computer box's label: the base period, then each fired block's rate."""
    parts = [f"Computer | dt_base={schedule.dt_base} s"]

    # A block fired every d base periods runs at f = (1 / dt_base) / d
    if schedule.fire:
        base_hz = 1.0 / schedule.dt_base
        for sys_id, divisor in sorted(schedule.fire.items()):
            rate_hz = base_hz / divisor
            parts.append(f"{sys_id}@{rate_hz:g} Hz")

    return " | ".join(parts)


def boundary_edge(conn) -> BoundaryTopologyEdge:
    """The edge of one boundary wire: a ZOH into the plant, a sample out of it."""
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


def resolve_boundary_port(
    refs, diagram_port: str, *, fallback_node: str
) -> tuple[str, str]:
    """The ``(node_id, port_id)`` behind a diagram port, else the side's routing node."""
    for ref in refs:
        if ref.diagram_port == diagram_port:
            return ref.node_id, ref.port_id

    return fallback_node, diagram_port
