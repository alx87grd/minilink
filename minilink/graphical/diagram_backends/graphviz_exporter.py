"""Graphviz exporter for diagram topology snapshots."""

from __future__ import annotations

from minilink.graphical.diagram_export import TopologyExporter


class GraphvizTopologyExporter(TopologyExporter):
    """Export topology snapshots to ``graphviz.Digraph``."""

    name = "graphviz"

    def export(self, topology, **kwargs):
        try:
            import graphviz
        except ImportError as exc:
            raise ImportError(
                "Graphviz topology export requires the graphviz Python package."
            ) from exc

        graph = graphviz.Digraph(topology.name, engine=kwargs.pop("engine", "dot"))
        graph.attr(rankdir=kwargs.pop("rankdir", "LR"))

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

        return graph


def block_html(node):
    """Return the Graphviz HTML-like label for one topology node."""
    title = f"{node.name}::{node.display_id}"
    html = (
        f'<TABLE BORDER="0" CELLSPACING="0">\n'
        f"<TR>\n"
        f'<TD align="left" BORDER="1" COLSPAN="2">{title}</TD>\n'
        f"</TR>\n"
    )

    n_rows = max(len(node.inputs), len(node.outputs))
    for j in range(n_rows):
        html += "<TR>\n"
        if j < len(node.inputs):
            port_id = node.inputs[j].id
            html += f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
        else:
            html += '<TD BORDER="1"> </TD>\n'

        if j < len(node.outputs):
            port_id = node.outputs[j].id
            html += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'
        else:
            html += '<TD BORDER="1"> </TD>\n'
        html += "</TR>\n"

    html += "</TABLE>"
    return html
