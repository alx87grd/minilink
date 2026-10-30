"""Diagram export: a system's block topology in Graphviz form."""

from __future__ import annotations

from minilink.graphical.diagrams.topology import build_diagram_topology

###############################################################################
# Public API
###############################################################################


class TopologyExporter:
    """Small topology exporter contract."""

    name = "base"

    def export(self, topology, **kwargs):
        """Export a topology snapshot."""
        raise NotImplementedError


def export_diagram_topology(
    sys_or_diagram, *, backend="graphviz", expand=True, **kwargs
):
    """Export a system or diagram topology with the selected backend.

    ``expand=True`` exports the blocks of every nested diagram, grouped in a
    labelled box; ``expand=False`` exports a nested diagram as one block.
    """
    topology = build_diagram_topology(sys_or_diagram, expand=expand)
    exporter = resolve_topology_exporter(backend)

    return exporter.export(topology, **kwargs)


###############################################################################
# Internal machinery
###############################################################################


def resolve_topology_exporter(backend):
    """The exporter ``backend`` names: an exporter passed as is, or ``"graphviz"``."""
    # An exporter, or any other object with an export method, is used as given
    if isinstance(backend, TopologyExporter):
        return backend
    if not isinstance(backend, str):
        require_exporter(backend)
        return backend

    # A name, read without case or surrounding spaces
    key = backend.strip().lower()
    if key == "graphviz":
        # A real cycle: dot imports TopologyExporter from this module
        from minilink.graphical.diagrams.dot import GraphvizTopologyExporter

        return GraphvizTopologyExporter()

    raise ValueError(
        f"Unknown topology backend {backend!r}. Expected 'graphviz' or an exporter."
    )


def require_exporter(backend):
    """Refuse an object that is neither a backend name nor an exporter with an ``export`` method."""
    if not callable(getattr(backend, "export", None)):
        raise TypeError(
            "backend must be 'graphviz' or an exporter with an "
            f"export(topology) method, got {type(backend).__name__}"
        )
