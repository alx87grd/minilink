"""Mermaid exporter for diagram topology snapshots."""

from __future__ import annotations

import re

from minilink.graphical.diagram_export import TopologyExporter


class MermaidTopologyExporter(TopologyExporter):
    """Export topology snapshots to Mermaid flowchart source text."""

    name = "mermaid"

    def export(self, topology, **kwargs) -> str:
        lines = ["flowchart LR"]
        for node in topology.nodes:
            node_id = _mermaid_id(node.id)
            label = _escape_label(_node_label(node))
            lines.append(f'  {node_id}["{label}"]')

        for edge in topology.edges:
            source = _mermaid_id(edge.source_node)
            target = _mermaid_id(edge.target_node)
            label = _escape_label(f"{edge.source_port} -> {edge.target_port}")
            lines.append(f'  {source} -- "{label}" --> {target}')

        return "\n".join(lines)


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
