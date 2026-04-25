def plot_graphviz(graphe, show_inline=None, show_pdf=None, filename=None):
    """
    Display a graphviz object.

    If ``graphe`` is ``None`` (e.g. the ``graphviz`` Python package is not
    installed, :func:`get_system_graphe` / :func:`get_diagram_graphe` return
    ``None``), this function returns quietly after a short message.

    The two display switches auto-resolve based on the current environment via
    :func:`minilink.graphical.environment.is_inline_capable`:

    - ``show_inline=None`` defaults to ``True`` in Jupyter / Colab (inline
      SVG via ``IPython.display``) and ``False`` elsewhere.
    - ``show_pdf=None`` defaults to ``False`` in Jupyter / Colab (no external
      viewer pop-up) and ``True`` in bare scripts and IPython REPLs (legacy
      behavior: render to a temp file and open the OS PDF viewer).
    - Explicit ``True`` / ``False`` values are always honored.

    A file is only written to disk when ``filename`` is explicitly provided
    or when ``show_pdf`` is true (which requires an on-disk artifact to open).
    Pure-notebook use therefore leaves no ``.gv`` / ``.pdf`` litter behind.
    """
    if graphe is None:
        print("No graph to display (graphviz Python package unavailable or graph build failed).")
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
    """
    Get the HTML representation of the block for the label in the diagram
    """
    import numpy as np

    n_ports_out = len(sys.outputs)
    n_ports_in = len(sys.inputs)

    html = (
        f'<TABLE BORDER="0" CELLSPACING="0">\n'
        f"<TR>\n"
        f'<TD align="left" BORDER="1" COLSPAN="2">{sys.name}::{html_id}</TD>\n'
        f"</TR>\n"
    )

    for j in range(np.max((n_ports_out, n_ports_in))):
        html += "<TR>\n"

        if j < n_ports_in and j < n_ports_out:
            port_id = list(sys.inputs.keys())[j]
            html += f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
            port_id = list(sys.outputs.keys())[j]
            html += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

        elif j < n_ports_in:
            port_id = list(sys.inputs.keys())[j]
            html += f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
            html += '<TD BORDER="1"> </TD>\n'

        elif j < n_ports_out:
            port_id = list(sys.outputs.keys())[j]
            html += '<TD BORDER="1"> </TD>\n'
            html += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

        html += "</TR>\n"
    html += "</TABLE>"

    return html


def get_system_graphe(sys):
    """
    Get the Graphviz object for a single system
    """
    try:
        import graphviz
    except ImportError:
        print("graphviz is not available")
        return None

    g = graphviz.Digraph(sys.name, engine="dot")
    g.attr(rankdir="LR")

    g.node(
        sys.name,
        shape="none",
        label=f"<{get_system_block_html(sys)}>",
    )

    return g


def get_diagram_graphe(diagram):
    """
    Get the Graphviz representation of a diagram
    """
    try:
        import graphviz
    except ImportError:
        print("graphviz is not available, cannot plot the diagram")
        return None

    g = graphviz.Digraph(diagram.name, engine="dot")
    g.attr(rankdir="LR")

    class _PortStub:
        """Lightweight stand-in for System so get_system_block_html can render I/O nodes."""

        def __init__(self, name, inputs, outputs):
            self.name = name
            self.inputs = inputs
            self.outputs = outputs

    # If diagram has external inputs
    if len(diagram.inputs) != 0:
        input_block = _PortStub("", {}, diagram.inputs)
        g.node(
            "input",
            shape="none",
            label="<" + get_system_block_html(input_block, "Inputs") + ">",
        )

    # Add subsystems nodes
    for sys_id, sys in diagram.subsystems.items():
        label = f"<{get_system_block_html(sys, sys_id)}>"
        g.node(
            sys_id,
            shape="none",
            label=label,
        )

    # If diagram has external outputs
    if len(diagram.outputs) != 0:
        output_block = _PortStub("", diagram.outputs, {})
        g.node(
            "output",
            shape="none",
            label="<" + get_system_block_html(output_block, "Outputs") + ">",
        )

    # Add edges
    for sys_id, sys in diagram.subsystems.items():
        for port_id in sys.inputs:
            edge = diagram.connections[sys_id][port_id]
            if edge is not None:
                g.edge(
                    edge[0] + ":" + edge[1] + ":e",
                    sys_id + ":" + port_id + ":w",
                )

    # Add edges to the output node
    if "output" in diagram.connections:
        for port_id, edge in diagram.connections["output"].items():
            if edge is not None:
                g.edge(
                    edge[0] + ":" + edge[1] + ":e",
                    "output" + ":" + port_id + ":w",
                )

    return g


if __name__ == "__main__":
    import graphviz

    graphe = graphviz.Digraph("G", engine="dot")
    graphe.node("A", label="A", shape="circle")

    plot_graphviz(graphe, show_inline=False, show_pdf=True)
