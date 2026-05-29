"""Diagram topology exporter dispatch."""

from __future__ import annotations

from minilink.graphical.diagrams.topology import build_diagram_topology


class TopologyExporter:
    """Small topology exporter contract."""

    name = "base"

    def export(self, topology, **kwargs):
        """Export a topology snapshot."""
        raise NotImplementedError


def export_diagram_topology(sys_or_diagram, *, backend="graphviz", **kwargs):
    """Export a system or diagram topology with the selected backend."""
    topology = build_diagram_topology(sys_or_diagram)
    return _resolve_topology_exporter(backend).export(topology, **kwargs)


def _resolve_topology_exporter(backend):
    if isinstance(backend, TopologyExporter):
        return backend
    if not isinstance(backend, str):
        return backend

    key = backend.strip().lower()
    if key == "graphviz":
        from minilink.graphical.diagrams.dot import (
            GraphvizTopologyExporter,
        )

        return GraphvizTopologyExporter()
    if key == "mermaid":
        from minilink.graphical.diagrams.mermaid import (
            MermaidTopologyExporter,
        )

        return MermaidTopologyExporter()
    raise ValueError(
        "Unknown topology backend {!r}. Expected 'graphviz' or 'mermaid'.".format(
            backend
        )
    )
