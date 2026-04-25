from __future__ import annotations

import matplotlib
import matplotlib.pyplot as plt
from typing import Literal

from minilink.graphical.environment import allow_tall_stacked_figures, is_blocking_needed
from minilink.graphical.matplotlib_style import (
    DPI_FIGURE,
    FIGSIZE_BASE,
    FONT_SIZE,
    signal_stack_figsize,
    style_trajectory_subplot,
    trajectory_stack_figsize,
)

###############################################################################
#  Note: modify matplotlib settings here to fit your environment
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


# Default figure settings (shared with :mod:`minilink.graphical.matplotlib_style`)
default_figsize = FIGSIZE_BASE
default_dpi = DPI_FIGURE
default_linestyle = "-"
default_fontsize = FONT_SIZE

# Embed font type in PDF when exporting
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42


print("Graphical settings:\n---------------------------------")
print("Matplotlib backend:", plt.get_backend())
print("Matplotlib interactive:", matplotlib.is_interactive())
# # print('Matplotlib list of interactive backends:', matplotlib.rcsetup.interactive_bk)


TrajectoryPlotMode = Literal["x", "u", "xu"]


############################################################
def plot_trajectory(
    sys,
    traj,
    *,
    plot: TrajectoryPlotMode = "xu",
):
    """
    Plot state and/or input time series in a shared-time column of subplots.

    Parameters
    ----------
    sys : System
        Model providing state/input labels and dimensions.
    traj : Trajectory
        Simulated trajectory.
    plot : {'x', 'u', 'xu'}, optional
        ``'x'`` — states only; ``'u'`` — inputs only; ``'xu'`` — states then inputs
        (default). Mirrors Pyro-style ``plot`` selection for simple trajectories.

    Returns
    -------
    fig : matplotlib.figure.Figure
    ax : list of matplotlib.axes.Axes

    Notes
    -----
    For arbitrary internal signals, use :func:`plot_signals` instead.
    """
    if plot not in ("x", "u", "xu"):
        raise ValueError("plot must be 'x', 'u', or 'xu'")

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

    want_x = plot in ("x", "xu")
    want_u = plot in ("u", "xu")
    n_plots = (n if want_x else 0) + (m if want_u else 0)
    if n_plots == 0:
        raise ValueError(
            f"Nothing to plot for plot={plot!r} with n={n}, m={m}."
        )

    # Create the figure (tall stack in notebook/Colab; capped height for pop-up windows)
    fig, ax = plt.subplots(
        n_plots,
        1,
        figsize=trajectory_stack_figsize(
            n_plots, allow_tall=allow_tall_stacked_figures()
        ),
        sharex=True,
        frameon=True,
        dpi=DPI_FIGURE,
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
    if want_x:
        for i in range(n):
            ax[idx].plot(
                t_traj,
                x_traj[i, :],
                "b",
                linewidth=1.5,
                alpha=0.8,
                label=state_labels[i],
            )
            ylabel_text = (
                f"{state_labels[i]}\n[{state_units[i]}]"
                if state_units[i]
                else f"{state_labels[i]}"
            )
            ax[idx].set_ylabel(
                ylabel_text, fontsize=default_fontsize, multialignment="center"
            )
            style_trajectory_subplot(ax[idx])
            ax[idx].legend(loc="upper right")
            idx += 1

    if want_u:
        for i in range(m):
            ax[idx].plot(
                t_traj,
                u_traj[i, :],
                "r",
                linewidth=1.5,
                alpha=0.8,
                label=input_labels[i],
            )
            ylabel_text = (
                f"{input_labels[i]}\n[{input_units[i]}]"
                if input_units[i]
                else f"{input_labels[i]}"
            )
            ax[idx].set_ylabel(
                ylabel_text, fontsize=default_fontsize, multialignment="center"
            )
            style_trajectory_subplot(ax[idx])
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
        n_plots,
        1,
        figsize=signal_stack_figsize(n_plots, allow_tall=allow_tall_stacked_figures()),
        sharex=True,
        frameon=True,
        dpi=DPI_FIGURE,
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

        style_trajectory_subplot(axis)
        axis.legend(loc="upper right")

    ax[-1].set_xlabel("Time [s]", fontsize=default_fontsize)
    plt.show(block=is_blocking_needed())
    return fig, ax
