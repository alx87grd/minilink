"""Block diagrams: a system's topology, exported to Graphviz and drawn."""

from minilink.graphical.diagrams.dot import (
    get_diagram,
    get_system_block_html,
    plot_diagram,
)
from minilink.graphical.diagrams.export import export_diagram_topology
from minilink.graphical.diagrams.topology import (
    abstract_boundary_ports,
    build_diagram_topology,
)

__all__ = [
    "abstract_boundary_ports",
    "build_diagram_topology",
    "export_diagram_topology",
    "get_diagram",
    "get_system_block_html",
    "plot_diagram",
]
