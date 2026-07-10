"""Mermaid exporter for hybrid topology snapshots."""

from __future__ import annotations

import re

from minilink.graphical.diagrams.hybrid_topology import (
    resolve_computer_boundary_anchor,
    resolve_plant_boundary_anchor,
)
from minilink.graphical.diagrams.mermaid import _escape_label, _node_label


def export_hybrid_mermaid(topology, **kwargs) -> str:
    """Return Mermaid flowchart source for one hybrid topology snapshot."""
    direction = kwargs.pop("direction", "TB")
    lines = [f"flowchart {direction}"]

    lines.append('  subgraph Plant["Plant"]')
    lines.extend(_diagram_subgraph_lines(topology.plant, indent="    "))
    lines.append("  end")

    schedule = _escape_label(topology.computer.schedule_label)
    lines.append(f'  subgraph Computer["{schedule}"]')
    lines.append('    subgraph StepDiagram["StepDiagram"]')
    lines.extend(_diagram_subgraph_lines(topology.step_diagram, indent="      "))
    lines.append("    end")
    lines.append("  end")

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
        c_id = _mermaid_id(c_node)
        p_id = _mermaid_id(p_node)
        label = _escape_label(f"{edge.label}: {c_port} -> {p_port}")
        if edge.direction == "computer_to_plant":
            lines.append(f'  {c_id} -. "{label}" .-> {p_id}')
        else:
            lines.append(f'  {p_id} -. "{label}" .-> {c_id}')

    return "\n".join(lines)


def _diagram_subgraph_lines(topology, *, indent: str) -> list[str]:
    lines = []
    for node in topology.nodes:
        node_id = _mermaid_id(node.id)
        label = _escape_label(_node_label(node))
        if node.kind == "step_system":
            label = _escape_label(f"Step: {_node_label(node)}")
        lines.append(f'{indent}{node_id}["{label}"]')

    for edge in topology.edges:
        source = _mermaid_id(edge.source_node)
        target = _mermaid_id(edge.target_node)
        label = _escape_label(f"{edge.source_port} -> {edge.target_port}")
        lines.append(f'{indent}{source} -- "{label}" --> {target}')
    return lines


def _mermaid_id(value: str) -> str:
    text = re.sub(r"[^0-9A-Za-z_]", "_", value)
    if not text:
        text = "node"
    if text[0].isdigit():
        text = "n_" + text
    return text
