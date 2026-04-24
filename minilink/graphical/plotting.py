import matplotlib
import matplotlib.pyplot as plt

from minilink.graphical.environment import is_blocking_needed

###############################################################################
#  Note: Modify here matplolib setting to fit your environment
###############################################################################

# Use interactive backend
try:
    # Default usage for interactive mode
    # matplotlib.use("Qt5Agg")
    plt.ion()  # Set interactive mode

except Exception:
    try:
        # For MacOSX
        # matplotlib.use("MacOSX")
        plt.ion()

    except Exception:
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

# Embed font type in PDF when exporting
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42


print("Graphical settings:\n---------------------------------")
print("Matplotlib backend:", plt.get_backend())
print("Matplotlib interactive:", matplotlib.is_interactive())
# # print('Matplotlib list of interactive backends:', matplotlib.rcsetup.interactive_bk)


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
        figsize=(8, 1.8 * n_plots),
        sharex=True,
        frameon=True,
    )
    if fig.canvas and hasattr(fig.canvas, "manager") and fig.canvas.manager:
        try:
            fig.canvas.manager.set_window_title("Trajectory for " + name)
        except Exception:
            pass

    if n_plots == 1:
        ax = [ax]

    fig.subplots_adjust(hspace=0.15)  # Tighter subplots

    # Plot the signals
    idx = 0
    for i in range(n):
        ax[idx].plot(
            t_traj, x_traj[i, :], "b", linewidth=1.5, alpha=0.8, label=state_labels[i]
        )
        ylabel_text = (
            f"{state_labels[i]}\n[{state_units[i]}]"
            if state_units[i]
            else f"{state_labels[i]}"
        )
        ax[idx].set_ylabel(
            ylabel_text, fontsize=default_fontsize, multialignment="center"
        )
        ax[idx].grid(True, linestyle="--", alpha=0.6)
        ax[idx].tick_params(labelsize=default_fontsize)
        ax[idx].legend(loc="upper right")
        idx += 1

    for i in range(m):
        ax[idx].plot(
            t_traj, u_traj[i, :], "r", linewidth=1.5, alpha=0.8, label=input_labels[i]
        )
        ylabel_text = (
            f"{input_labels[i]}\n[{input_units[i]}]"
            if input_units[i]
            else f"{input_labels[i]}"
        )
        ax[idx].set_ylabel(
            ylabel_text, fontsize=default_fontsize, multialignment="center"
        )
        ax[idx].grid(True, linestyle="--", alpha=0.6)
        ax[idx].tick_params(labelsize=default_fontsize)
        ax[idx].legend(loc="upper right")
        idx += 1

    ax[-1].set_xlabel("Time [s]", fontsize=default_fontsize)

    # Show the figure
    plt.show(block=is_blocking_needed())

    return fig, ax


############################################################
def plot_signals(sys, traj, signals):
    """
    Plots specifically requested internal or external signals on a dynamic number of subplots.

    Parameters
    ----------
    sys : System
        The minilink system (e.g. DiagramSystem)
    traj : Trajectory
        The simulated trajectory object
    signals : list of dict
        A list of signal specification dictionaries. Each dictionary represents one subplot.
        Example:
        [
            {"sys": "plant", "state": ["theta", "theta_dot"]},
            {"sys": "controller", "output": "u", "label": "Control Action"},
            {"input": "w", "label": "Disturbance"}
        ]
    """
    n_plots = len(signals)
    if n_plots == 0:
        return None, None

    fig, ax = plt.subplots(
        n_plots, 1, figsize=(8, 2.0 * n_plots), sharex=True, frameon=True
    )
    if n_plots == 1:
        ax = [ax]

    if fig.canvas and hasattr(fig.canvas, "manager") and fig.canvas.manager:
        try:
            fig.canvas.manager.set_window_title("Signal Plot for " + sys.name)
        except Exception:
            pass

    fig.subplots_adjust(hspace=0.2)
    t_traj = traj.t

    for idx, signal_req in enumerate(signals):
        axis = ax[idx]

        for key in ["state", "output", "input"]:
            if key in signal_req:
                items = signal_req[key]
                if not isinstance(items, list):
                    items = [items]

                labels = signal_req.get("label", items)
                if not isinstance(labels, list):
                    labels = [labels]

                sys_id = signal_req.get("sys")

                for k, item in enumerate(items):
                    lbl = labels[k] if k < len(labels) else item

                    lookup_item = f"{sys_id}:{item}" if sys_id else item

                    if key == "state":
                        if lookup_item in sys.state.labels:
                            state_idx = sys.state.labels.index(lookup_item)
                        elif item in sys.state.labels:
                            state_idx = sys.state.labels.index(item)
                        else:
                            state_idx = -1

                        if state_idx >= 0:
                            axis.plot(
                                t_traj,
                                traj.x[state_idx, :],
                                linewidth=1.5,
                                alpha=0.8,
                                label=lbl,
                            )
                        else:
                            print(
                                f"Warning: State '{lookup_item}' or '{item}' not found."
                            )

                    elif key == "output":
                        if traj.has_signal(lookup_item):
                            data = traj.get_signal(lookup_item)
                            for d in range(data.shape[0]):
                                label_str = f"{lbl}[{d}]" if data.shape[0] > 1 else lbl
                                axis.plot(
                                    t_traj,
                                    data[d, :],
                                    linewidth=1.5,
                                    alpha=0.8,
                                    label=label_str,
                                )
                        else:
                            print(
                                f"Warning: Output '{lookup_item}' not found in trajectory signals."
                            )

                    elif key == "input":
                        inp_labels, _ = sys.get_all_input_labels_and_units()
                        if lookup_item in inp_labels:
                            u_idx = inp_labels.index(lookup_item)
                            axis.plot(
                                t_traj,
                                traj.u[u_idx, :],
                                linewidth=1.5,
                                alpha=0.8,
                                label=lbl,
                            )
                        else:
                            print(
                                f"Warning: Input '{lookup_item}' not found in system inputs."
                            )

        axis.grid(True, linestyle="--", alpha=0.6)
        axis.tick_params(labelsize=default_fontsize)
        axis.legend(loc="upper right")

    ax[-1].set_xlabel("Time [s]", fontsize=default_fontsize)
    plt.show(block=is_blocking_needed())
    return fig, ax
