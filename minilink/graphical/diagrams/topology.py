"""Backend-neutral diagram topology views."""

from __future__ import annotations

from dataclasses import dataclass

from minilink.core.system import StepSystem
from minilink.core.wiring import WiredDiagramMixin


@dataclass(frozen=True)
class TopologyPort:
    """One input or output port on a topology node."""

    id: str
    dim: int


@dataclass(frozen=True)
class TopologyNode:
    """One display node in a system or diagram topology."""

    id: str
    name: str
    display_id: str
    kind: str
    inputs: tuple[TopologyPort, ...]
    outputs: tuple[TopologyPort, ...]


@dataclass(frozen=True)
class TopologyEdge:
    """Directed port-to-port connection."""

    source_node: str
    source_port: str
    target_node: str
    target_port: str


@dataclass(frozen=True)
class BoundaryPortRef:
    """One diagram boundary port routed to an internal block port."""

    diagram_port: str
    node_id: str
    port_id: str


@dataclass(frozen=True)
class DiagramTopology:
    """Read-only topology snapshot for display/export."""

    name: str
    nodes: tuple[TopologyNode, ...]
    edges: tuple[TopologyEdge, ...]
    boundary_inputs: tuple[BoundaryPortRef, ...] = ()
    boundary_outputs: tuple[BoundaryPortRef, ...] = ()


def build_diagram_topology(
    sys_or_diagram, *, abstract_boundary=False
) -> DiagramTopology:
    """Build a display/export topology snapshot from a system or diagram.

    ``abstract_boundary=True`` removes external Inputs/Outputs routing nodes and
    records ``boundary_inputs`` / ``boundary_outputs`` anchors on wired ports.
    """
    if isinstance(sys_or_diagram, WiredDiagramMixin):
        topology = _build_diagram_system_topology(sys_or_diagram)
    else:
        topology = _build_single_system_topology(sys_or_diagram)
    if abstract_boundary:
        topology = abstract_boundary_ports(topology)
    return topology


def abstract_boundary_ports(topology: DiagramTopology) -> DiagramTopology:
    """
    Remove external input/output routing nodes and record boundary port anchors.

    Diagram boundary ports that wire straight through ``input`` / ``output`` nodes
    are mapped to the connected subsystem port. Internal edges between subsystems
    are unchanged. Unconnected boundary ports are omitted from the snapshot.
    """
    input_node = _find_node(topology, "input", kind="external_input")
    output_node = _find_node(topology, "output", kind="external_output")
    if input_node is None and output_node is None:
        return topology

    boundary_inputs = []
    boundary_outputs = []
    skip_nodes = set()

    if input_node is not None:
        skip_nodes.add(input_node.id)
        for edge in topology.edges:
            if edge.source_node == input_node.id:
                boundary_inputs.append(
                    BoundaryPortRef(
                        diagram_port=edge.source_port,
                        node_id=edge.target_node,
                        port_id=edge.target_port,
                    )
                )

    if output_node is not None:
        skip_nodes.add(output_node.id)
        for edge in topology.edges:
            if edge.target_node == output_node.id:
                boundary_outputs.append(
                    BoundaryPortRef(
                        diagram_port=edge.target_port,
                        node_id=edge.source_node,
                        port_id=edge.source_port,
                    )
                )

    nodes = tuple(node for node in topology.nodes if node.id not in skip_nodes)
    edges = tuple(
        edge
        for edge in topology.edges
        if edge.source_node not in skip_nodes and edge.target_node not in skip_nodes
    )
    return DiagramTopology(
        name=topology.name,
        nodes=nodes,
        edges=edges,
        boundary_inputs=tuple(boundary_inputs),
        boundary_outputs=tuple(boundary_outputs),
    )


def _build_single_system_topology(sys) -> DiagramTopology:
    node = _node_from_system(sys.name, sys, display_id="sys1", kind="system")
    return DiagramTopology(name=sys.name, nodes=(node,), edges=())


def _build_diagram_system_topology(diagram) -> DiagramTopology:
    nodes = []
    edges = []

    if len(diagram.inputs) != 0:
        nodes.append(
            _node_from_ports(
                "input",
                name="",
                display_id="Inputs",
                kind="external_input",
                inputs={},
                outputs=diagram.inputs,
            )
        )

    for sys_id, sys in diagram.subsystems.items():
        kind = "step_system" if isinstance(sys, StepSystem) else "system"
        nodes.append(_node_from_system(sys_id, sys, display_id=sys_id, kind=kind))

    if len(diagram.outputs) != 0:
        nodes.append(
            _node_from_ports(
                "output",
                name="",
                display_id="Outputs",
                kind="external_output",
                inputs=diagram.outputs,
                outputs={},
            )
        )

    for sys_id, sys in diagram.subsystems.items():
        for port_id in sys.inputs:
            edge = diagram.connections[sys_id][port_id]
            if edge is not None:
                edges.append(
                    TopologyEdge(
                        source_node=edge[0],
                        source_port=edge[1],
                        target_node=sys_id,
                        target_port=port_id,
                    )
                )

    if "output" in diagram.connections:
        for port_id, edge in diagram.connections["output"].items():
            if edge is not None:
                edges.append(
                    TopologyEdge(
                        source_node=edge[0],
                        source_port=edge[1],
                        target_node="output",
                        target_port=port_id,
                    )
                )

    return DiagramTopology(name=diagram.name, nodes=tuple(nodes), edges=tuple(edges))


def _find_node(topology: DiagramTopology, node_id: str, *, kind: str | None = None):
    for node in topology.nodes:
        if node.id == node_id and (kind is None or node.kind == kind):
            return node
    return None


def _node_from_system(node_id: str, sys, *, display_id: str, kind: str):
    return _node_from_ports(
        node_id,
        name=sys.name,
        display_id=display_id,
        kind=kind,
        inputs=sys.inputs,
        outputs=sys.outputs,
    )


def _node_from_ports(
    node_id: str,
    *,
    name: str,
    display_id: str,
    kind: str,
    inputs,
    outputs,
) -> TopologyNode:
    return TopologyNode(
        id=node_id,
        name=name,
        display_id=display_id,
        kind=kind,
        inputs=tuple(
            TopologyPort(id=port_id, dim=port.dim) for port_id, port in inputs.items()
        ),
        outputs=tuple(
            TopologyPort(id=port_id, dim=port.dim) for port_id, port in outputs.items()
        ),
    )
