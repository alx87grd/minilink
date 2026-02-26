import matplotlib
import matplotlib.pyplot as plt
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


############################################################
def plot_trajectory(sys, traj):

    # Extract the system dimensions and labels
    n = sys.n
    m = sys.m
    name = sys.name
    state_labels, state_units = sys.state.labels, sys.state.units
    input_labels, input_units = sys.get_all_input_labels_and_units()

    # Extract the trajectory data
    t_traj = traj.t
    x_traj = traj.x
    u_traj = traj.u

    # Compute the number of plots
    n_plots = n + m

    # Create the figure
    fig, ax = plt.subplots(
        n_plots,
        1,
        figsize=(10, 2 * n_plots),
        sharex=True,
        # dpi=default_dpi,
        frameon=True,
    )
    if fig.canvas and hasattr(fig.canvas, "manager") and fig.canvas.manager:
        try:
            fig.canvas.manager.set_window_title("Trajectory for " + name)
        except Exception:
            pass

    if n_plots == 1:
        ax = [ax]

    # Plot the signals
    idx = 0
    for i in range(n):
        ax[idx].plot(t_traj, x_traj[i, :], "b")
        ax[idx].set_ylabel(
            f"{state_labels[i]}[{state_units[i]}]", fontsize=default_fontsize
        )
        ax[idx].grid()
        ax[idx].tick_params(labelsize=default_fontsize)
        idx += 1
    for i in range(m):
        ax[idx].plot(t_traj, u_traj[i, :], "r")
        ax[idx].set_ylabel(
            f"{input_labels[i]} {input_units[i]}", fontsize=default_fontsize
        )
        ax[idx].grid()
        ax[idx].tick_params(labelsize=default_fontsize)
        idx += 1

    ax[-1].set_xlabel("Time [s]", fontsize=default_fontsize)

    # Show the figure
    plt.show(block=figure_blocking)

    return fig, ax
