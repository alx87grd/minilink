"""Hybrid Mermaid diagrams: the plant and the computer's step diagram in two subgraphs."""

from __future__ import annotations

import re

from minilink.graphical.diagrams.hybrid_topology import (
    resolve_computer_boundary_anchor,
    resolve_plant_boundary_anchor,
)
from minilink.graphical.diagrams.mermaid import _escape_label, _node_label

###############################################################################
# Public API
###############################################################################


def export_hybrid_mermaid(topology, **kwargs) -> str:
    """Return Mermaid flowchart source for one hybrid topology snapshot."""
    direction = kwargs.pop("direction", "TB")
    lines = [f"flowchart {direction}"]

    # The plant's subgraph
    lines.append('  subgraph Plant["Plant"]')
    lines.extend(diagram_subgraph_lines(topology.plant, indent="    "))
    lines.append("  end")

    # The computer's subgraph, labelled with its schedule, around its step diagram
    schedule = _escape_label(topology.computer.schedule_label)
    lines.append(f'  subgraph Computer["{schedule}"]')
    lines.append('    subgraph StepDiagram["StepDiagram"]')
    lines.extend(diagram_subgraph_lines(topology.step_diagram, indent="      "))
    lines.append("    end")
    lines.append("  end")

    # The dotted ZOH and sample edges across the boundary, each labelled with
    # its two ports
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
        c_id = mermaid_id(c_node)
        p_id = mermaid_id(p_node)
        label = _escape_label(f"{edge.label}: {c_port} -> {p_port}")

        # A ZOH runs from the computer to the plant, a sample back from the plant
        if edge.direction == "computer_to_plant":
            lines.append(f'  {c_id} -. "{label}" .-> {p_id}')
        else:
            lines.append(f'  {p_id} -. "{label}" .-> {c_id}')

    return "\n".join(lines)


###############################################################################
# Internal machinery
###############################################################################


def diagram_subgraph_lines(topology, *, indent: str) -> list[str]:
    """Return the Mermaid lines of one side's blocks and wires, at ``indent``."""
    lines = []

    # The blocks, a step system marked as such
    for node in topology.nodes:
        node_id = mermaid_id(node.id)
        label = _escape_label(_node_label(node))
        if node.kind == "step_system":
            label = _escape_label(f"Step: {_node_label(node)}")
        lines.append(f'{indent}{node_id}["{label}"]')

    # One labelled arrow per wire, from its source port to its target port
    for edge in topology.edges:
        source = mermaid_id(edge.source_node)
        target = mermaid_id(edge.target_node)
        label = _escape_label(f"{edge.source_port} -> {edge.target_port}")
        lines.append(f'{indent}{source} -- "{label}" --> {target}')

    return lines


def mermaid_id(value: str) -> str:
    """A Mermaid node id: each character outside ``[0-9A-Za-z_]`` as ``_``, ``node`` when empty, ``n_`` before a digit."""
    text = re.sub(r"[^0-9A-Za-z_]", "_", value)
    if not text:
        text = "node"

    if text[0].isdigit():
        text = "n_" + text

    return text
