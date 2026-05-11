"""Graph display compatibility helpers."""

from minilink.graphical.diagram_backends.graphviz_exporter import block_html
from minilink.graphical.diagram_export import export_diagram_topology
from minilink.graphical.topology import build_diagram_topology


def plot_graphviz(graphe, show_inline=None, show_pdf=None, filename=None):
    """
    Display a graphviz object.

    ``show_inline=None`` defaults to inline display in Jupyter / Colab.
    ``show_pdf=None`` defaults to external PDF display in bare scripts.
    """
    if graphe is None:
        print(
            "No graph to display (graphviz Python package unavailable or graph build failed)."
        )
        return

    from minilink.graphical.environment import is_inline_capable

    inline_env = is_inline_capable()
    if show_inline is None:
        show_inline = inline_env
    if show_pdf is None:
        show_pdf = not inline_env

    if show_inline:
        try:
            import IPython.display as display

            display.display(graphe)
        except ImportError:
            print("IPython is not available for inline display")

    need_disk = show_pdf or filename is not None
    if not need_disk:
        return

    if filename is None:
        import tempfile

        with tempfile.NamedTemporaryFile(
            suffix="_" + graphe.name + ".gv", delete=False
        ) as tmp:
            filename = tmp.name

    try:
        graphe.render(filename=filename, view=show_pdf)
    except Exception as e:
        print(f"Warning: Could not render graph. Is Graphviz installed? Error: {e}")


def get_system_block_html(sys, html_id="sys1"):
    """Return the Graphviz HTML-like block label for a system."""
    topology = build_diagram_topology(sys)
    node = topology.nodes[0]
    if html_id != node.display_id:
        from dataclasses import replace

        node = replace(node, display_id=html_id)
    return block_html(node)


def get_system_graphe(sys):
    """Return the Graphviz object for a single system."""
    try:
        return export_diagram_topology(sys, backend="graphviz")
    except ImportError:
        print("graphviz is not available")
        return None


def get_diagram_graphe(diagram):
    """Return the Graphviz representation of a diagram."""
    try:
        return export_diagram_topology(diagram, backend="graphviz")
    except ImportError:
        print("graphviz is not available, cannot plot the diagram")
        return None


if __name__ == "__main__":
    import graphviz

    graphe = graphviz.Digraph("G", engine="dot")
    graphe.node("A", label="A", shape="circle")

    plot_graphviz(graphe, show_inline=False, show_pdf=True)
