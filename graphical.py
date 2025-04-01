import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys as python_system

###############################################################################
#  Note: Modify here matplolib setting to fit your environment
###############################################################################

# Use interactive backend
try:
    # Default usage for interactive mode
    matplotlib.use("Qt5Agg")
    plt.ion()  # Set interactive mode

except:

    try:
        # For MacOSX
        matplotlib.use("MacOSX")
        plt.ion()

    except:

        print("Warning: Could not load validated backend mode for matplotlib")
        print(
            "Matplotlib list of interactive backends:",
            matplotlib.rcsetup.interactive_bk,
        )
        plt.ion()  # Set interactive mode


# Default figure settings
default_figsize = (4, 3)
default_dpi = 150
default_linestyle = "-"
default_fontsize = 10

# True if running in IPython, False if running the file in terminal
if hasattr(python_system, "ps1"):
    figure_blocking = False  # Set to not block the code when showing a figure
else:
    # We want to block figure to prevent the script from terminating
    figure_blocking = True  # Set to block the code when showing a figure

# Embed font type in PDF when exporting
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42


print("Graphical settings:\n---------------------------------")
print("Matplotlib backend:", plt.get_backend())
print("Matplotlib interactive:", matplotlib.is_interactive())
# # print('Matplotlib list of interactive backends:', matplotlib.rcsetup.interactive_bk)
print("Matplotlib figure blocking:", figure_blocking)


###############################################################################
###############################################################################
###############################################################################


def plot_graphviz(graphe, show_inline=False, show_pdf=True, filename=None):
    """
    Display a graphviz object in the notebook
    """
    if show_inline:
        import IPython.display as display

        display.display(graphe)

    if filename is None:
        try:
            import tempfile

            filename = tempfile.mktemp("_" + graphe.name + ".gv")
        except ImportError:
            print("tempfile is not available")
            filename = "temp_" + graphe.name + ".gv"
    else:
        filename = filename

    graphe.render(filename=filename, view=show_pdf)


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
