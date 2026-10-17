"""Hybrid Graphviz diagrams: the plant and the computer's step diagram in two boxes."""

from __future__ import annotations

import warnings

from minilink.graphical.diagrams.dot import (
    MISSING_GRAPHVIZ_MESSAGE,
    graphviz_port_id,
    render_block,
    render_diagram_graph,
)
from minilink.graphical.diagrams.hybrid_topology import (
    build_hybrid_topology,
    resolve_computer_boundary_anchor,
    resolve_plant_boundary_anchor,
)
from minilink.graphical.diagrams.topology import DiagramTopology

###############################################################################
# Public API
###############################################################################


def export_hybrid_graphviz(topology, **kwargs):
    """Return a ``graphviz.Digraph`` for one hybrid topology snapshot."""
    graphviz = require_graphviz()

    graph = graphviz.Digraph(topology.name, engine=kwargs.pop("engine", "dot"))
    graph.attr(rankdir=kwargs.pop("rankdir", "TB"))

    # The plant, in its box
    with graph.subgraph(name="cluster_plant") as plant_cluster:
        plant_cluster.attr(label="Plant")
        render_diagram_cluster(plant_cluster, topology.plant)

    # The computer's box, labelled with its schedule, around its step diagram
    with graph.subgraph(name="cluster_computer") as computer_cluster:
        computer_cluster.attr(label=topology.computer.schedule_label)
        with computer_cluster.subgraph(name="cluster_step_diagram") as step_cluster:
            step_cluster.attr(label="StepDiagram")
            render_diagram_cluster(step_cluster, topology.step_diagram)

    # The dashed ZOH and sample edges across the boundary
    render_boundary_edges(graph, topology)

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
    # The snapshot's graph; without the graphviz package, a warning and no figure
    try:
        graph = export_hybrid_graphviz(
            build_hybrid_topology(hybrid, abstract_boundary=abstract_boundary)
        )
    except ImportError:
        warnings.warn(MISSING_GRAPHVIZ_MESSAGE, stacklevel=2)
        return None

    drawn = render_diagram_graph(
        graph,
        show=show,
        show_inline=show_inline,
        show_pdf=show_pdf,
        filename=filename,
    )

    # Nothing for the notebook cell to render when the figure could not be drawn
    return graph if drawn else None


###############################################################################
# Internal machinery
###############################################################################


def require_graphviz():
    """Refuse a hybrid export without the graphviz Python package; return the package."""
    try:
        import graphviz
    except ImportError as exc:
        raise ImportError(
            "Graphviz hybrid export requires the graphviz Python package."
        ) from exc

    return graphviz


def render_diagram_cluster(graph, topology: DiagramTopology) -> None:
    """Declare one side's blocks and wires inside its box."""
    for node in topology.nodes:
        render_block(graph, node)

    # One wire per edge, from the east side of an output port to the west side
    # of an input port
    for edge in topology.edges:
        graph.edge(
            f"{edge.source_node}:{graphviz_port_id(edge.source_port, 'out')}:e",
            f"{edge.target_node}:{graphviz_port_id(edge.target_port, 'in')}:w",
        )


def render_boundary_edges(graph, topology) -> None:
    """Draw each boundary edge, dashed, between the step diagram and the plant."""
    for edge in topology.boundary_edges:
        # The block and port each side of the edge wires to
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

        # A ZOH runs from the computer to the plant, a sample back from the plant
        source = f"{c_node}:{graphviz_port_id(c_port, 'out')}:e"
        target = f"{p_node}:{graphviz_port_id(p_port, 'in')}:w"
        if edge.direction == "plant_to_computer":
            source = f"{p_node}:{graphviz_port_id(p_port, 'out')}:e"
            target = f"{c_node}:{graphviz_port_id(c_port, 'in')}:w"

        graph.edge(source, target, label=edge.label, style="dashed")
