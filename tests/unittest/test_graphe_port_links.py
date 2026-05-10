"""Port link targets in diagram Graphviz output (editor support)."""

from minilink.core.diagram_topology import (
    DiagramTopology,
    EdgeSpec,
    NodeSpec,
    build_diagram_from_topology,
    default_mvp_catalog,
)
from minilink.graphical.graphe import get_diagram_graphe


def test_diagram_svg_port_links_contain_ml_query():
    top = DiagramTopology(
        nodes=(
            NodeSpec("a", "integrator", {}),
            NodeSpec("b", "integrator", {}),
        ),
        edges=(EdgeSpec("a", "y", "b", "u"),),
    )
    diagram = build_diagram_from_topology(top, default_mvp_catalog())
    g = get_diagram_graphe(diagram, port_links=True)
    assert g is not None
    svg = g.pipe(format="svg").decode()
    assert "ml?" in svg
    assert "dir=out" in svg
