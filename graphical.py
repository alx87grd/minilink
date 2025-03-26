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
default_dpi = 250
default_linestyle = "-"
default_fontsize = 5

# True if running in IPython, False if running the file in terminal
if hasattr(python_system, "ps1"):
    figure_blocking = False  # Set to not block the code when showing a figure
else:
    # We want to block figure to prevent the script from terminating
    figure_blocking = True  # Set to block the code when showing a figure

# Embed font type in PDF when exporting
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42

# print('Matplotlib backend:', plt.get_backend())
# print('Matplotlib interactive:', matplotlib.is_interactive())
# # print('Matplotlib list of interactive backends:', matplotlib.rcsetup.interactive_bk)
# print('Matplotlib figure blocking:', figure_blocking)
