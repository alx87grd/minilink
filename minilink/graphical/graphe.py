def plot_graphviz(graphe, show_inline=False, show_pdf=True, filename=None):
    """
    Display a graphviz object in the notebook
    """
    if show_inline:
        try:
            import IPython.display as display

            display.display(graphe)
        except ImportError:
            print("IPython is not available for inline display")

    if filename is None:
        try:
            import tempfile

            filename = tempfile.mktemp("_" + graphe.name + ".gv")
        except ImportError:
            filename = "temp_" + graphe.name + ".gv"
    else:
        filename = filename

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
        html += f"<TR>\n"

        if j < n_ports_in and j < n_ports_out:
            port_id = list(sys.inputs.keys())[j]
            html += f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
            port_id = list(sys.outputs.keys())[j]
            html += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

        elif j < n_ports_in:
            port_id = list(sys.inputs.keys())[j]
            html += f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
            html += f'<TD BORDER="1"> </TD>\n'

        elif j < n_ports_out:
            port_id = list(sys.outputs.keys())[j]
            html += f'<TD BORDER="1"> </TD>\n'
            html += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

        html += f"</TR>\n"
    html += f"</TABLE>"

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

    # If diagram has external inputs
    if not len(diagram.inputs) == 0:
        # Import System inside to avoid circular dependencies if possible,
        # but here we just need a dummy structure that has name, inputs, outputs
        class DummySystem:
            def __init__(self, name, inputs, outputs):
                self.name = name
                self.inputs = inputs
                self.outputs = outputs

        input_block = DummySystem("", {}, diagram.inputs)
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
    if not len(diagram.outputs) == 0:

        class DummySystem:
            def __init__(self, name, inputs, outputs):
                self.name = name
                self.inputs = inputs
                self.outputs = outputs

        output_block = DummySystem("", diagram.outputs, {})
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


###############################################################################
######################################################################
if __name__ == "__main__":

    import numpy as np
    import graphviz
    import matplotlib.pyplot as plt
    import IPython.display as display

    graphe = graphviz.Digraph("G", filename="temp.gv", engine="dot")
    graphe = graphviz.Digraph("G", filename="temp.gv", engine="dot")
    graphe.node("A", label="A", shape="circle")

    plot_graphviz(graphe, show_inline=True, show_pdf=True, filename="aaa")
