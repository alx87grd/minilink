"""Mermaid exporter for diagram topology snapshots."""

from __future__ import annotations

import re

from minilink.graphical.diagrams.export import TopologyExporter
from minilink.graphical.diagrams.topology import clustered_node_ids


class MermaidTopologyExporter(TopologyExporter):
    """Export topology snapshots to Mermaid flowchart source text."""

    name = "mermaid"

    def export(self, topology, **kwargs) -> str:
        lines = ["flowchart LR"]
        nodes = {node.id: node for node in topology.nodes}
        boxed = clustered_node_ids(topology.clusters)
        for node in topology.nodes:
            if node.id not in boxed:
                lines.append(block_line(node, indent="  "))
        for cluster in topology.clusters:
            lines.extend(cluster_lines(cluster, nodes, indent="  "))

        for edge in topology.edges:
            source = _mermaid_id(edge.source_node)
            target = _mermaid_id(edge.target_node)
            label = _escape_label(f"{edge.source_port} -> {edge.target_port}")
            lines.append(f'  {source} -- "{label}" --> {target}')

        return "\n".join(lines)


def block_line(node, *, indent: str) -> str:
    """Return the Mermaid declaration of one block."""
    label = _escape_label(_node_label(node))
    return f'{indent}{_mermaid_id(node.id)}["{label}"]'


def cluster_lines(cluster, nodes, *, indent: str) -> list[str]:
    """Return the Mermaid ``subgraph`` of one nested diagram, recursively."""
    label = _escape_label(f"{cluster.name}::{cluster.display_id}")
    lines = [f'{indent}subgraph {_mermaid_id(cluster.id)}["{label}"]']
    for node_id in cluster.node_ids:
        lines.append(block_line(nodes[node_id], indent=indent + "  "))
    for child in cluster.clusters:
        lines.extend(cluster_lines(child, nodes, indent=indent + "  "))
    lines.append(f"{indent}end")
    return lines


def _node_label(node) -> str:
    if node.kind == "external_input":
        return "Inputs"
    if node.kind == "external_output":
        return "Outputs"
    return f"{node.name}::{node.display_id}"


def _mermaid_id(value: str) -> str:
    text = re.sub(r"[^0-9A-Za-z_]", "_", value)
    if not text:
        text = "node"
    if text[0].isdigit():
        text = "n_" + text
    return text


def _escape_label(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')
