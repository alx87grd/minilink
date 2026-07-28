"""Graphviz exporter for hybrid topology snapshots."""

from __future__ import annotations

from minilink.graphical.diagrams.dot import _render_diagram_graph, block_html
from minilink.graphical.diagrams.hybrid_topology import (
    resolve_computer_boundary_anchor,
    resolve_plant_boundary_anchor,
)
from minilink.graphical.diagrams.topology import DiagramTopology


def export_hybrid_graphviz(topology, **kwargs):
    """Return a ``graphviz.Digraph`` for one hybrid topology snapshot."""
    try:
        import graphviz
    except ImportError as exc:
        raise ImportError(
            "Graphviz hybrid export requires the graphviz Python package."
        ) from exc

    graph = graphviz.Digraph(topology.name, engine=kwargs.pop("engine", "dot"))
    graph.attr(rankdir=kwargs.pop("rankdir", "TB"))

    with graph.subgraph(name="cluster_plant") as plant_cluster:
        plant_cluster.attr(label="Plant")
        _render_diagram_cluster(plant_cluster, topology.plant)

    with graph.subgraph(name="cluster_computer") as computer_cluster:
        computer_cluster.attr(label=topology.computer.schedule_label)
        with computer_cluster.subgraph(name="cluster_step_diagram") as step_cluster:
            step_cluster.attr(label="StepDiagram")
            _render_diagram_cluster(step_cluster, topology.step_diagram)

    _render_boundary_edges(graph, topology)
    return graph


def plot_hybrid_diagram(
    hybrid,
    filename=None,
    show=True,
    show_inline=None,
    show_pdf=None,
    *,
    abstract_boundary=True,
):
    """Render Plant + Computer clusters with boundary ZOH/sample edges."""
    from minilink.graphical.diagrams.hybrid_topology import build_hybrid_topology

    graph = export_hybrid_graphviz(
        build_hybrid_topology(hybrid, abstract_boundary=abstract_boundary)
    )
    _render_diagram_graph(
        graph,
        show=show,
        show_inline=show_inline,
        show_pdf=show_pdf,
        filename=filename,
    )
    return graph


def _render_diagram_cluster(graph, topology: DiagramTopology) -> None:
    for node in topology.nodes:
        graph.node(
            node.id,
            shape="none",
            label=f"<{block_html(node)}>",
        )

    for edge in topology.edges:
        graph.edge(
            f"{edge.source_node}:{edge.source_port}:e",
            f"{edge.target_node}:{edge.target_port}:w",
        )


def _render_boundary_edges(graph, topology) -> None:
    for edge in topology.boundary_edges:
        c_node, c_port = resolve_computer_boundary_anchor(
            topology.step_diagram,
            direction=edge.direction,
            computer_port=edge.computer_port,
        )
        p_node, p_port = resolve_plant_boundary_anchor(
            topology.plant,
            direction=edge.direction,
            plant_port=edge.plant_port,
        )
        source = f"{c_node}:{c_port}:e"
        target = f"{p_node}:{p_port}:w"
        if edge.direction == "plant_to_computer":
            source = f"{p_node}:{p_port}:e"
            target = f"{c_node}:{c_port}:w"
        graph.edge(source, target, label=edge.label, style="dashed")
