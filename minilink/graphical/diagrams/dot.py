"""Graphviz exporter for diagram topology snapshots."""

from __future__ import annotations

import warnings

from minilink.graphical.diagrams.export import TopologyExporter
from minilink.graphical.diagrams.topology import build_diagram_topology


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
    if node.kind == "step_system":
        title = f"Step: {title}"
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


def _render_diagram_graph(
    graph, show=True, show_inline=None, show_pdf=None, filename=None
):
    """
    Display a Graphviz object.

    ``show_inline=None`` defaults to inline SVG in Jupyter / Colab.
    Outside notebooks, ``show=True`` opens a Matplotlib window with the
    Graphviz PNG (same blocking policy as trajectory plots). Pass
    ``show_pdf=True`` for the legacy OS PDF viewer; pass ``filename`` to
    write Graphviz output to disk. ``show=False`` builds/returns only.
    """
    if graph is None:
        warnings.warn(
            "No graph to display (graphviz Python package unavailable or "
            "graph build failed).",
            stacklevel=2,
        )
        return

    from minilink.graphical.common.environment import (
        is_blocking_needed,
        is_inline_capable,
    )

    inline_env = is_inline_capable()
    if show_inline is None:
        show_inline = inline_env
    if show_pdf is None:
        show_pdf = False

    if show and show_inline:
        try:
            import IPython.display as display

            display.display(graph)
        except ImportError:
            warnings.warn("IPython is not available for inline display", stacklevel=2)

    show_mpl = show and (not show_inline) and (not show_pdf)
    if show_mpl:
        try:
            import io

            import matplotlib.pyplot as plt

            # Graphviz default DPI is ~96; bump for a sharper Matplotlib window.
            prev_dpi = graph.graph_attr.get("dpi")
            graph.graph_attr["dpi"] = "200"
            try:
                png = graph.pipe(format="png")
            finally:
                if prev_dpi is None:
                    graph.graph_attr.pop("dpi", None)
                else:
                    graph.graph_attr["dpi"] = prev_dpi

            img = plt.imread(io.BytesIO(png), format="png")
            height, width = img.shape[:2]
            fig_dpi = 100.0
            # High-res PNG; shrink the window a bit vs 1:1 pixel mapping.
            scale = 0.65
            fig, ax = plt.subplots(
                figsize=(scale * width / fig_dpi, scale * height / fig_dpi),
                dpi=fig_dpi,
            )
            ax.imshow(img)
            ax.axis("off")
            fig.tight_layout(pad=0)
            if plt.get_backend().lower() != "agg":
                plt.show(block=is_blocking_needed())
        except Exception as exc:
            warnings.warn(
                f"Could not show diagram in Matplotlib. "
                f"Is Graphviz installed? Error: {exc}",
                stacklevel=2,
            )

    need_disk = show_pdf or filename is not None
    if not need_disk:
        return

    if filename is None:
        import tempfile

        with tempfile.NamedTemporaryFile(
            suffix="_" + graph.name + ".gv",
            delete=False,
        ) as tmp:
            filename = tmp.name

    try:
        graph.render(filename=filename, view=bool(show and show_pdf))
    except Exception as exc:
        warnings.warn(
            f"Could not render graph. Is Graphviz installed? Error: {exc}",
            stacklevel=2,
        )


def get_system_block_html(sys, html_id="sys1"):
    """Return the Graphviz HTML-like block label for a system."""
    topology = build_diagram_topology(sys)
    node = topology.nodes[0]
    if html_id != node.display_id:
        from dataclasses import replace

        node = replace(node, display_id=html_id)
    return block_html(node)


def get_diagram(sys_or_diagram):
    """Return the renderable diagram object for a system or assembled diagram."""
    from minilink.graphical.diagrams.export import export_diagram_topology

    try:
        return export_diagram_topology(sys_or_diagram, backend="graphviz")
    except ImportError:
        warnings.warn("graphviz is not available", stacklevel=2)
        return None


def plot_diagram(
    sys_or_diagram, filename=None, show=True, show_inline=None, show_pdf=None
):
    """Render a system or assembled diagram."""
    graph = get_diagram(sys_or_diagram)
    _render_diagram_graph(
        graph,
        show=show,
        show_inline=show_inline,
        show_pdf=show_pdf,
        filename=filename,
    )
    return graph
