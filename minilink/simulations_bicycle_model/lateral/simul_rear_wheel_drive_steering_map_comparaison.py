import time

import matplotlib.pyplot as plt
import numpy as np
from simul_rear_wheel_drive_steering_MAP_w_PID import (
    create_diagram as create_diagram_pid_w_steer_map,
)
from simul_rear_wheel_drive_steering_PID import (
    create_diagram as create_diagram_pid_only,
)

# Optional, only needed if you still want animations
from minilink.simulations_bicycle_model.vehicule_helper import (
    create_vehicle,
)


def plot_multiple_speed_comparisons(results):
    """
    Plot speed tracking comparisons grouped by target_r.

    One subplot is created for each target_r; each subplot shows curves
    for all target_speed values corresponding to that target_r.
    The reference trajectory is plotted once per subplot (since it is
    identical for all speeds with the same target_r). A single legend
    is drawn for the whole figure (not per subplot).
    """

    from matplotlib.lines import Line2D

    # find unique target_r values and sort them
    target_rs = sorted({res["target_r"] for res in results})
    # find unique speeds globally (used to build legend entries)
    unique_speeds = sorted({res["target_speed"] for res in results})

    # build a simple colormap for speeds (keeps colors consistent if desired)
    cmap = plt.get_cmap("tab10", max(1, len(unique_speeds)))
    speed_to_color = {s: cmap(i) for i, s in enumerate(unique_speeds)}

    n = len(target_rs)

    fig, axes = plt.subplots(
        nrows=n,
        ncols=1,
        figsize=(10, 3.0 * n),
        sharex=True,
    )

    if n == 1:
        axes = [axes]

    for ax, tr in zip(axes, target_rs):
        # collect all results with this target_r
        group = [res for res in results if res["target_r"] == tr]

        if not group:
            continue

        # plot the reference once (take from the first result in the group)
        ref_t = group[0]["t"]
        ref_fixed = group[0]["pid_only"]["ref"]
        ax.plot(ref_t, ref_fixed, "k--", linewidth=1.2)  # no per-axis label

        for res in group:
            t_simul = res["t"]
            meas_fixed = res["pid_only"]["meas"]
            meas_meas = res["fancy_ctrl"]["meas"]
            speed = res["target_speed"]
            color = speed_to_color[speed]

            # plot controller measurements without labels (legend will be global)
            ax.plot(t_simul, meas_fixed, color=color, lw=1.5)
            ax.plot(t_simul, meas_meas, color=color, lw=1.5, linestyle="--")

        ax.set_ylabel("Speed [rad/s]")
        ax.set_title(f"Speed tracking for R_REF = {tr:.2f} rad/s (all VX)")
        ax.grid(True)

    axes[-1].set_xlabel("Time [s]")

    # Create a single, global legend for the figure
    # controller handles
    ctrl_handles = [
        Line2D([0], [0], color="k", lw=1.2, ls="--", label="Reference"),
        Line2D([0], [0], color="gray", lw=1.5, label="PID only"),
        Line2D([0], [0], color="gray", lw=1.5, ls="--", label="PID w map"),
    ]
    # speed handles (colors)
    speed_handles = [
        Line2D([0], [0], color=speed_to_color[s], lw=2, label=f"{s:.2f} m/s")
        for s in unique_speeds
    ]

    all_handles = ctrl_handles + speed_handles

    # create a separate legend figure (will open in another window with a GUI backend)
    legend_fig = plt.figure(figsize=(max(6, 1.2 * len(unique_speeds)), 1.6))
    legend_ax = legend_fig.add_subplot(111)
    legend_ax.axis("off")
    legend_ax.legend(handles=all_handles, loc="center", ncol=min(6, len(all_handles)))

    # draw both figures (non-blocking) so the legend appears in its own window
    fig.tight_layout()
    legend_fig.tight_layout()
    plt.show()


def run_single_comparison(
    target_speed,
    target_r,
    simul_time=10,
    dt=0.005,
):
    print("\n============================================================")
    print(
        f"Running comparison for VX_REF = {target_speed:.2f} m/s AND R_REF = {target_r:.2f} "
    )
    print("============================================================")
    vehicle = create_vehicle(vx=target_speed)

    diagram_pid_only = create_diagram_pid_only(
        vehicle, vx_ref=target_speed, r_ref=target_r
    )
    diagram_fancy = create_diagram_pid_w_steer_map(
        vehicle, vx_ref=target_speed, r_ref=target_r
    )

    traj_pid_only = diagram_pid_only.reconstruct_internal_signals(
        diagram_pid_only.compute_trajectory(
            tf=simul_time,
            dt=dt,
            verbose=False,
        )
    )
    pid_logs = traj_pid_only.get_signal("r_pid:logs")

    pid_only_ref = pid_logs[0, :]
    pid_only_meas = pid_logs[1, :]

    traj_fancy = diagram_fancy.reconstruct_internal_signals(
        diagram_fancy.compute_trajectory(
            tf=simul_time,
            dt=dt,
            verbose=False,
        )
    )
    pid_logs = traj_fancy.get_signal("r_pid:logs")
    # vx_pid_logs = vx_traj.get_signal("v_pid:logs")

    vx_ref = pid_logs[0, :]
    vx_meas = pid_logs[1, :]

    # TODO: mettre aussi les info de v_pid ici. Permetterait de comparer VX selon targ en vitesse et steering.
    # PRENDRE EN COMPTE L'ANGLE DE STEERING DANS LE CONTROLLEUR VX ??????
    return {
        "target_speed": target_speed,
        "target_r": target_r,
        "t": traj_pid_only.t,
        "pid_only": {
            "ref": pid_only_ref,
            "meas": pid_only_meas,
        },
        "fancy_ctrl": {
            "ref": vx_ref,
            "meas": vx_meas,
        },
    }


def main():
    simul_time = 5
    dt = 0.005

    target_rs = np.linspace(0.1, 0.6, 6)

    target_speeds = np.linspace(1.0, 30.0, 8)

    results = []

    total_start = time.perf_counter()

    for target_speed in target_speeds:
        for target_r in target_rs:
            sim_start = time.perf_counter()

            result = run_single_comparison(
                target_speed=target_speed,
                target_r=target_r,
                simul_time=simul_time,
                dt=dt,
            )

            sim_end = time.perf_counter()

            sim_elapsed = sim_end - sim_start

            print(
                f"Computation time for {target_r:.2f} rad/s: {sim_elapsed:.3f} seconds"
            )

            results.append(result)

    total_end = time.perf_counter()

    total_elapsed = total_end - total_start

    print(f"\nTotal computation time: {total_elapsed:.3f} seconds")
    print(f"Average time per speed: {total_elapsed / len(target_rs):.3f} seconds")

    # Recommended: one subplot per speed
    plot_multiple_speed_comparisons(results)

    # Optional: all curves on one figure
    # plot_all_speeds_on_one_figure(results)


if __name__ == "__main__":
    main()
