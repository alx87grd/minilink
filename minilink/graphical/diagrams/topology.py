"""Backend-neutral diagram topology views."""

from __future__ import annotations

from dataclasses import dataclass, replace

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
class TopologyCluster:
    """One nested diagram, drawn as a labelled box around its own blocks."""

    id: str
    name: str
    display_id: str
    node_ids: tuple[str, ...]
    clusters: tuple[TopologyCluster, ...] = ()


@dataclass(frozen=True)
class DiagramTopology:
    """Read-only topology snapshot for display/export.

    ``nodes`` and ``edges`` are always a complete flat diagram. ``clusters``
    only groups the nodes that came out of an expanded nested diagram.
    """

    name: str
    nodes: tuple[TopologyNode, ...]
    edges: tuple[TopologyEdge, ...]
    boundary_inputs: tuple[BoundaryPortRef, ...] = ()
    boundary_outputs: tuple[BoundaryPortRef, ...] = ()
    clusters: tuple[TopologyCluster, ...] = ()


def build_diagram_topology(
    sys_or_diagram, *, abstract_boundary=False, expand=False
) -> DiagramTopology:
    """Build a display/export topology snapshot from a system or diagram.

    ``abstract_boundary=True`` removes external Inputs/Outputs routing nodes and
    records ``boundary_inputs`` / ``boundary_outputs`` anchors on wired ports.
    ``expand=True`` replaces every nested diagram by its own blocks, at every
    depth, and records one :class:`TopologyCluster` per nested diagram.
    """
    if isinstance(sys_or_diagram, WiredDiagramMixin):
        topology = _build_diagram_system_topology(sys_or_diagram)
        if expand:
            topology = expand_nested_diagrams(topology, sys_or_diagram)
    else:
        topology = _build_single_system_topology(sys_or_diagram)
    if abstract_boundary:
        topology = abstract_boundary_ports(topology)
    return topology


def expand_nested_diagrams(topology: DiagramTopology, diagram) -> DiagramTopology:
    """
    Replace every nested diagram block by its own blocks, grouped in a cluster.

    The blocks of a nested diagram keep their ids behind the diagram's id
    (``controller__pi``). An edge that reached a boundary port of a nested
    diagram goes straight to the block ports wired to that port inside: one
    edge per inner reader of a boundary input, none when nothing is wired
    behind the port. A boundary port left unconnected outside draws nothing.
    """
    inner = {
        sys_id: prefix_topology(
            build_diagram_topology(sys, abstract_boundary=True, expand=True),
            f"{sys_id}__",
        )
        for sys_id, sys in diagram.subsystems.items()
        if isinstance(sys, WiredDiagramMixin)
    }
    if not inner:
        return topology

    nodes = []
    clusters = []
    for node in topology.nodes:
        if node.id not in inner:
            nodes.append(node)
            continue
        sub = inner[node.id]
        boxed = clustered_node_ids(sub.clusters)
        nodes.extend(sub.nodes)
        clusters.append(
            TopologyCluster(
                id=node.id,
                name=node.name,
                display_id=node.display_id,
                node_ids=tuple(n.id for n in sub.nodes if n.id not in boxed),
                clusters=sub.clusters,
            )
        )

    inner_inputs = {sys_id: sub.boundary_inputs for sys_id, sub in inner.items()}
    inner_outputs = {sys_id: sub.boundary_outputs for sys_id, sub in inner.items()}
    edges = []
    for edge in topology.edges:
        sources = port_anchors(inner_outputs, edge.source_node, edge.source_port)
        targets = port_anchors(inner_inputs, edge.target_node, edge.target_port)
        for source_node, source_port in sources:
            for target_node, target_port in targets:
                edges.append(
                    TopologyEdge(
                        source_node=source_node,
                        source_port=source_port,
                        target_node=target_node,
                        target_port=target_port,
                    )
                )
    for sub in inner.values():
        edges.extend(sub.edges)

    return DiagramTopology(
        name=topology.name,
        nodes=tuple(nodes),
        edges=tuple(edges),
        clusters=tuple(clusters),
    )


def prefix_topology(topology: DiagramTopology, prefix: str) -> DiagramTopology:
    """Return ``topology`` with ``prefix`` in front of every node id."""

    def pid(node_id: str) -> str:
        return f"{prefix}{node_id}"

    def prefix_refs(refs):
        return tuple(replace(ref, node_id=pid(ref.node_id)) for ref in refs)

    def prefix_clusters(clusters):
        return tuple(
            replace(
                cluster,
                id=pid(cluster.id),
                node_ids=tuple(pid(node_id) for node_id in cluster.node_ids),
                clusters=prefix_clusters(cluster.clusters),
            )
            for cluster in clusters
        )

    return DiagramTopology(
        name=topology.name,
        nodes=tuple(replace(node, id=pid(node.id)) for node in topology.nodes),
        edges=tuple(
            replace(
                edge,
                source_node=pid(edge.source_node),
                target_node=pid(edge.target_node),
            )
            for edge in topology.edges
        ),
        boundary_inputs=prefix_refs(topology.boundary_inputs),
        boundary_outputs=prefix_refs(topology.boundary_outputs),
        clusters=prefix_clusters(topology.clusters),
    )


def clustered_node_ids(clusters) -> set[str]:
    """Return the ids of every node inside ``clusters``, at any depth."""
    node_ids = set()
    for cluster in clusters:
        node_ids.update(cluster.node_ids)
        node_ids.update(clustered_node_ids(cluster.clusters))
    return node_ids


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
        clusters=topology.clusters,
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


def port_anchors(boundary_refs, node_id: str, port_id: str) -> list[tuple[str, str]]:
    """Return the ``(node_id, port_id)`` ends behind one end of an edge.

    ``boundary_refs`` maps each nested diagram id to its boundary port anchors;
    a port of any other node is its own anchor.
    """
    if node_id not in boundary_refs:
        return [(node_id, port_id)]
    return [
        (ref.node_id, ref.port_id)
        for ref in boundary_refs[node_id]
        if ref.diagram_port == port_id
    ]


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
