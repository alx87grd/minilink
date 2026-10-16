"""Mermaid exporter for diagram topology snapshots."""

from __future__ import annotations

import re

from minilink.graphical.diagrams.export import TopologyExporter
from minilink.graphical.diagrams.topology import clustered_node_ids

###############################################################################
# Public API
###############################################################################


class MermaidTopologyExporter(TopologyExporter):
    """Export topology snapshots to Mermaid flowchart source text."""

    name = "mermaid"

    def export(self, topology, **kwargs) -> str:
        """Return the Mermaid flowchart source of one topology snapshot."""
        lines = ["flowchart LR"]
        nodes = {node.id: node for node in topology.nodes}
        boxed = clustered_node_ids(topology.clusters)

        # The blocks outside every nested diagram, then one subgraph per nested diagram
        for node in topology.nodes:
            if node.id not in boxed:
                lines.append(block_line(node, indent="  "))

        for cluster in topology.clusters:
            lines.extend(cluster_lines(cluster, nodes, indent="  "))

        # One labeled arrow per connection, from its source port to its target port
        for edge in topology.edges:
            source = mermaid_id(edge.source_node)
            target = mermaid_id(edge.target_node)
            label = escape_label(f"{edge.source_port} -> {edge.target_port}")
            lines.append(f'  {source} -- "{label}" --> {target}')

        return "\n".join(lines)


###############################################################################
# Internal machinery
###############################################################################


def block_line(node, *, indent: str) -> str:
    """Return the Mermaid declaration of one block."""
    label = escape_label(node_label(node))

    return f'{indent}{mermaid_id(node.id)}["{label}"]'


def cluster_lines(cluster, nodes, *, indent: str) -> list[str]:
    """Return the Mermaid ``subgraph`` of one nested diagram, recursively."""
    label = escape_label(f"{cluster.name}::{cluster.display_id}")
    lines = [f'{indent}subgraph {mermaid_id(cluster.id)}["{label}"]']

    # Its own blocks, then its nested diagrams, one level further in
    for node_id in cluster.node_ids:
        lines.append(block_line(nodes[node_id], indent=indent + "  "))

    for child in cluster.clusters:
        lines.extend(cluster_lines(child, nodes, indent=indent + "  "))

    lines.append(f"{indent}end")

    return lines


def node_label(node) -> str:
    """The label of a block: ``Inputs`` or ``Outputs`` on the boundary, else ``name::id``."""
    if node.kind == "external_input":
        return "Inputs"

    if node.kind == "external_output":
        return "Outputs"

    return f"{node.name}::{node.display_id}"


_node_label = node_label  # ripple


def mermaid_id(value: str) -> str:
    """A Mermaid node id: each character outside ``[0-9A-Za-z_]`` as ``_``, ``node`` when empty, ``n_`` before a digit."""
    text = re.sub(r"[^0-9A-Za-z_]", "_", value)
    if not text:
        text = "node"

    if text[0].isdigit():
        text = "n_" + text

    return text


def escape_label(value: str) -> str:
    """The label with its backslashes and double quotes escaped for Mermaid."""
    return value.replace("\\", "\\\\").replace('"', '\\"')


_escape_label = escape_label  # ripple
