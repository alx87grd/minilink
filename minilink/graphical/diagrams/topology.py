"""Backend-neutral diagram topology views."""

from __future__ import annotations

from dataclasses import dataclass

from minilink.core.diagram import DiagramSystem


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
class DiagramTopology:
    """Read-only topology snapshot for display/export."""

    name: str
    nodes: tuple[TopologyNode, ...]
    edges: tuple[TopologyEdge, ...]


def build_diagram_topology(sys_or_diagram) -> DiagramTopology:
    """Build a display/export topology snapshot from a system or diagram."""
    if isinstance(sys_or_diagram, DiagramSystem):
        return _build_diagram_system_topology(sys_or_diagram)
    return _build_single_system_topology(sys_or_diagram)


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
        nodes.append(_node_from_system(sys_id, sys, display_id=sys_id, kind="system"))

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
