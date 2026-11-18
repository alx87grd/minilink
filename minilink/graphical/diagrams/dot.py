"""Graphviz diagrams: a system's blocks and wires as a Graphviz graph."""

from __future__ import annotations

import getpass
import hashlib
import re
import tempfile
import warnings
from dataclasses import replace
from pathlib import Path

from minilink.graphical.common.plot_result import NoAxes, PlotResult
from minilink.graphical.common.show import show_figure
from minilink.graphical.diagrams.export import (
    TopologyExporter,
    export_diagram_topology,
)
from minilink.graphical.diagrams.topology import (
    build_diagram_topology,
    clustered_node_ids,
)

# One warning when the graphviz Python wrapper is missing: diagrams are
# optional, so plot_diagram skips the figure and the notebook keeps running.
MISSING_GRAPHVIZ_MESSAGE = (
    "Diagram skipped: Graphviz is not installed. It needs a system install: "
    "`conda install -c conda-forge python-graphviz` (the conda environment has it), "
    "or install Graphviz from graphviz.org then `pip install graphviz`."
)


###############################################################################
# Public API
###############################################################################


class GraphvizTopologyExporter(TopologyExporter):
    """Export topology snapshots to ``graphviz.Digraph``."""

    name = "graphviz"

    def export(self, topology, *, engine="dot", rankdir="LR"):
        """Build the snapshot's ``graphviz.Digraph``: its blocks, then its wires.

        ``engine`` is the Graphviz layout program and ``rankdir`` the flow
        direction; any other keyword is refused.
        """
        graphviz = require_graphviz()

        graph = graphviz.Digraph(topology.name, engine=engine)
        graph.attr(rankdir=rankdir, nodesep="0.4", ranksep="0.7", fontname=FONT)
        graph.node_attr.update(fontname=FONT, fontsize="11")
        graph.edge_attr.update(arrowsize="0.7")

        # The blocks, each nested diagram inside its box
        render_blocks(graph, topology)

        # One wire per edge, from the east side of an output port to the west side
        # of an input port
        for edge in topology.edges:
            graph.edge(
                f"{edge.source_node}:{graphviz_port_id(edge.source_port, 'out')}:e",
                f"{edge.target_node}:{graphviz_port_id(edge.target_port, 'in')}:w",
            )

        return graph


def plot_diagram(sys_or_diagram, *, show=True, expand=True):
    """Draw a system's or an assembled diagram's blocks and wires: a :class:`PlotResult`.

    ``expand=True`` draws the blocks of every nested diagram inside a labelled
    box; ``expand=False`` draws a nested diagram as one block. ``show=True``
    shows it where the session puts it: an SVG under the notebook cell, the
    vector PDF in the system's viewer elsewhere; ``"inline"`` or ``"window"``
    names the place. ``.save("loop.svg")`` writes it in the format of the
    extension. Without Graphviz the figure is skipped with a warning.
    """
    graph = get_diagram(sys_or_diagram, expand=expand)
    show_figure(graph, "graphviz", show)

    return PlotResult(renderer="graphviz", figure=graph, axes=NoAxes("graphviz"))


def get_diagram(sys_or_diagram, *, expand=True):
    """Return the renderable diagram object for a system or assembled diagram.

    ``expand=True`` draws the blocks of every nested diagram inside a labelled
    box; ``expand=False`` draws a nested diagram as one block.
    """
    try:
        return export_diagram_topology(
            sys_or_diagram, backend="graphviz", expand=expand
        )
    except ImportError:
        warnings.warn(MISSING_GRAPHVIZ_MESSAGE, stacklevel=2)
        return None


def get_system_block_html(sys, html_id="sys1"):
    """Return the Graphviz HTML-like block label for a system."""
    topology = build_diagram_topology(sys)
    node = topology.nodes[0]

    # The label shows the requested id in place of the block's own
    if html_id != node.display_id:
        node = replace(node, display_id=html_id)

    return block_html(node)


# The HTML-like block labels and their port identifiers

FONT = "Helvetica"
TITLE_FILL = "#e5e7eb"
BOX_FILL = "#f4f4f5"
BOUNDARY_COLOR = "#71717a"


def block_html(node):
    """Return the Graphviz HTML-like label for one topology node."""
    # The diagram's edge: its inputs and outputs as plain port cells, no title
    if node.kind in ("external_input", "external_output"):
        return boundary_html(node)

    # The title row: the block's name and id, a step system marked as such
    title = f"{node.name}::{node.display_id}"
    if node.kind == "step_system":
        title = f"Step: {title}"
    html = (
        f'<TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">\n'
        f"<TR>\n"
        f'<TD align="left" COLSPAN="2" BGCOLOR="{TITLE_FILL}"><B>{title}</B></TD>\n'
        f"</TR>\n"
    )

    # One row per port pair: the inputs in the left column, the outputs in the
    # right one, an empty cell where a side has run out of ports
    n_rows = max(len(node.inputs), len(node.outputs))
    for j in range(n_rows):
        html += "<TR>\n"
        if j < len(node.inputs):
            port = node.inputs[j]
            html += (
                f'<TD PORT="{graphviz_port_id(port.id, "in")}" align="left">'
                f"{port.text}</TD>\n"
            )
        else:
            html += "<TD> </TD>\n"

        if j < len(node.outputs):
            port = node.outputs[j]
            html += (
                f'<TD PORT="{graphviz_port_id(port.id, "out")}" align="right">'
                f"{port.text}</TD>\n"
            )
        else:
            html += "<TD> </TD>\n"
        html += "</TR>\n"

    html += "</TABLE>"

    return html


def boundary_html(node):
    """Return the label of the diagram's edge: one grey cell per boundary port."""
    role = "out" if node.kind == "external_input" else "in"
    ports = node.outputs if node.kind == "external_input" else node.inputs
    html = '<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="3">\n'
    for port in ports:
        html += (
            f'<TR><TD PORT="{graphviz_port_id(port.id, role)}">'
            f'<FONT COLOR="{BOUNDARY_COLOR}">{port.text}</FONT></TD></TR>\n'
        )
    html += "</TABLE>"

    return html


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
    require_port_role(role)

    # The signs spelled out, every other id escaped one character at a time
    if port_id == "+":
        text = "plus"
    elif port_id == "-":
        text = "minus"
    else:
        text = "".join(escape_port_char(char) for char in port_id)

    return f"{role}_{text}"


###############################################################################
# Internal machinery
###############################################################################


def require_graphviz():
    """Refuse to export without the graphviz Python package; return the package."""
    try:
        import graphviz
    except ImportError as exc:
        raise ImportError(
            "Graphviz topology export requires the graphviz Python package."
        ) from exc

    return graphviz


def render_blocks(graph, topology):
    """Declare the blocks of a topology, each nested diagram inside its box."""
    nodes = {node.id: node for node in topology.nodes}
    boxed = clustered_node_ids(topology.clusters)

    # The blocks outside every box
    for node in topology.nodes:
        if node.id not in boxed:
            render_block(graph, node)

    # Each nested diagram, boxed with its own blocks
    for cluster in topology.clusters:
        render_cluster(graph, cluster, nodes)


def render_block(graph, node):
    """Declare one block, drawn as its HTML-like port table."""
    graph.node(node.id, shape="none", label=f"<{block_html(node)}>")


def render_cluster(graph, cluster, nodes):
    """Draw one nested diagram as a labelled Graphviz cluster around its blocks."""
    with graph.subgraph(name=f"cluster_{cluster.id}") as box:
        box.attr(**box_attrs(f"{cluster.name}::{cluster.display_id}"))

        # Its own blocks, then the diagrams nested inside it, each in a box of its own
        for node_id in cluster.node_ids:
            node = nodes[node_id]
            render_block(box, node)
        for child in cluster.clusters:
            render_cluster(box, child, nodes)


def box_attrs(title):
    """The Graphviz attributes of a box around a nested diagram: a grey container, its title top-left."""
    return dict(
        label=f"<<B>{title}</B>>",
        labeljust="l",
        fontname=FONT,
        fontsize="12",
        style="filled",
        fillcolor=BOX_FILL,
        color=BOUNDARY_COLOR,
        penwidth="1.2",
        margin="12",
    )


def require_port_role(role):
    """Refuse a port role other than ``"in"`` or ``"out"``."""
    if role not in ("in", "out"):
        raise ValueError(f"port role must be 'in' or 'out', got {role!r}")


def escape_port_char(char):
    """Map one character of a port id onto ``[0-9a-z_]``, injectively."""
    if char == "_":
        return "__"
    if "A" <= char <= "Z":
        return "_" + char.lower()
    if "a" <= char <= "z" or "0" <= char <= "9":
        return char
    return "_"


def present_diagram(graph, place):
    """Show ``graph``: its SVG under the notebook cell, or its vector PDF in the system's viewer."""
    try:
        if place == "inline":
            # Lazy import: IPython is optional, needed only in a notebook
            from IPython.display import SVG, display

            display(SVG(graph.pipe(format="svg")))
            return

        # The viewer's file: one per diagram, named by its name and its source
        path = viewer_path_of(graph)
        path.write_bytes(graph.pipe(format="pdf"))
        open_in_viewer(path)
    except Exception as exc:
        # The graphviz wrapper raises ExecutableNotFound without the ``dot`` binary
        warnings.warn(
            "Could not draw the diagram. Is the Graphviz binary installed? "
            "(conda install graphviz / apt install graphviz / brew install graphviz). "
            f"Error: {exc}",
            stacklevel=3,
        )


def viewer_path_of(graph):
    """A per-user temporary PDF for ``graph``: its name made portable, and a hash of its source."""
    folder = Path(tempfile.gettempdir()) / f"minilink-{getpass.getuser()}"
    folder.mkdir(parents=True, exist_ok=True)
    name = re.sub(r"[^A-Za-z0-9_.-]+", "_", graph.name or "diagram").strip("_")[:60]
    digest = hashlib.sha1(graph.source.encode("utf-8")).hexdigest()[:8]

    return folder / f"{name or 'diagram'}-{digest}.pdf"


def open_in_viewer(path):
    """Open ``path`` in the system's viewer for its type."""
    graphviz = require_graphviz()
    graphviz.view(str(path))
