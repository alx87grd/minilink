"""Graphviz exporter for diagram topology snapshots."""

from __future__ import annotations

import warnings

from minilink.graphical.diagrams.export import TopologyExporter
from minilink.graphical.diagrams.topology import (
    build_diagram_topology,
    clustered_node_ids,
)

# One warning when the graphviz Python wrapper is missing: diagrams are
# optional, so plot_diagram skips the figure and the notebook keeps running.
MISSING_GRAPHVIZ_MESSAGE = (
    "Diagram skipped: the graphviz Python package is not installed "
    "(pip install graphviz, or pip install minilink[diagrams])."
)


def graphviz_port_id(port_id, role):
    """Return the Graphviz HTML ``PORT`` identifier of one minilink port.

    Graphviz matches port names case-insensitively and the same table holds
    the block's inputs and outputs, so the identifier carries the port's
    ``role`` (``"in"`` or ``"out"``) and an escaped name that stays unique
    under case folding: ``+`` / ``-`` become ``plus`` / ``minus``, an
    underscore doubles, an uppercase letter becomes ``_`` plus its lowercase,
    and any other character outside ``[0-9a-z]`` (the brackets of a ``y[0]``
    Demux id) becomes ``_``. The cell still displays the original port id.
    """
    if role not in ("in", "out"):
        raise ValueError(f"port role must be 'in' or 'out', got {role!r}")
    if port_id == "+":
        text = "plus"
    elif port_id == "-":
        text = "minus"
    else:
        text = "".join(escape_port_char(char) for char in port_id)
    return f"{role}_{text}"


def escape_port_char(char):
    """Map one character of a port id onto ``[0-9a-z_]``, injectively."""
    if char == "_":
        return "__"
    if "A" <= char <= "Z":
        return "_" + char.lower()
    if "a" <= char <= "z" or "0" <= char <= "9":
        return char
    return "_"


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

        render_blocks(graph, topology)

        for edge in topology.edges:
            graph.edge(
                f"{edge.source_node}:{graphviz_port_id(edge.source_port, 'out')}:e",
                f"{edge.target_node}:{graphviz_port_id(edge.target_port, 'in')}:w",
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
            port = node.inputs[j]
            html += (
                f'<TD PORT="{graphviz_port_id(port.id, "in")}" align="left" '
                f'BORDER="1">{port.text}</TD>\n'
            )
        else:
            html += '<TD BORDER="1"> </TD>\n'

        if j < len(node.outputs):
            port = node.outputs[j]
            html += (
                f'<TD PORT="{graphviz_port_id(port.id, "out")}" '
                f'BORDER="1">{port.text}</TD>\n'
            )
        else:
            html += '<TD BORDER="1"> </TD>\n'
        html += "</TR>\n"

    html += "</TABLE>"
    return html


def render_blocks(graph, topology):
    """Declare the blocks of a topology, each nested diagram inside its box."""
    nodes = {node.id: node for node in topology.nodes}
    boxed = clustered_node_ids(topology.clusters)
    for node in topology.nodes:
        if node.id not in boxed:
            graph.node(node.id, shape="none", label=f"<{block_html(node)}>")
    for cluster in topology.clusters:
        render_cluster(graph, cluster, nodes)


def render_cluster(graph, cluster, nodes):
    """Draw one nested diagram as a labelled Graphviz cluster around its blocks."""
    with graph.subgraph(name=f"cluster_{cluster.id}") as box:
        box.attr(label=f"{cluster.name}::{cluster.display_id}")
        for node_id in cluster.node_ids:
            node = nodes[node_id]
            box.node(node.id, shape="none", label=f"<{block_html(node)}>")
        for child in cluster.clusters:
            render_cluster(box, child, nodes)


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
        # get_diagram already warned that graphviz is missing.
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
        # Probe Graphviz here so a missing ``dot`` binary warns instead of
        # raising. Do not IPython.display the graph: plot_diagram returns it,
        # and the notebook auto-displays that last expression. Calling
        # display() *and* returning the object renders the figure twice
        # (same rule as SharedSystemFacades.animate).
        try:
            graph.pipe(format="svg")
        except Exception as exc:
            # graphviz raises ExecutableNotFound when the ``dot`` binary is
            # missing (a bare Colab runtime, or a pip install without the
            # system package). The notebook keeps running; the diagram is
            # simply not drawn.
            warnings.warn(
                "Could not render the diagram inline. Is the Graphviz binary "
                "installed? (conda install graphviz / apt install graphviz / "
                f"brew install graphviz). Error: {exc}",
                stacklevel=2,
            )

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


def get_diagram(sys_or_diagram, *, expand=True):
    """Return the renderable diagram object for a system or assembled diagram.

    ``expand=True`` draws the blocks of every nested diagram inside a labelled
    box; ``expand=False`` draws a nested diagram as one block.
    """
    from minilink.graphical.diagrams.export import export_diagram_topology

    try:
        return export_diagram_topology(
            sys_or_diagram, backend="graphviz", expand=expand
        )
    except ImportError:
        warnings.warn(MISSING_GRAPHVIZ_MESSAGE, stacklevel=2)
        return None


def plot_diagram(
    sys_or_diagram,
    filename=None,
    show=True,
    show_inline=None,
    show_pdf=None,
    *,
    expand=True,
):
    """Render a system or assembled diagram.

    ``expand=True`` draws the blocks of every nested diagram inside a labelled
    box; ``expand=False`` draws a nested diagram as one block.
    """
    graph = get_diagram(sys_or_diagram, expand=expand)
    _render_diagram_graph(
        graph,
        show=show,
        show_inline=show_inline,
        show_pdf=show_pdf,
        filename=filename,
    )
    return graph
