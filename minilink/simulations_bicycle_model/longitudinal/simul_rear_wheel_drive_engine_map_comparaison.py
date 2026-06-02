"""Comparison demo for two DynamicBicycleRearWheelDrive PID configurations.

This script compares two diagrams:

1. DUMB_VX_motor_map
2. VX_motor_map

It runs multiple simulations at different target speeds and plots the speed
tracking result of both PID controllers.
"""

import time

import matplotlib.pyplot as plt
import numpy as np
from simul_rear_wheel_drive_engine_DUMB_VX_motor_map import (
    create_diagram as create_diagram_dumb,
)
from simul_rear_wheel_drive_engine_VX_motor_map import (
    create_diagram as create_diagram_vx,
)

# Optional, only needed if you still want animations
from minilink.simulations_bicycle_model.vehicule_helper import (
    create_vehicle,
)

DELTA_REF = 0.0  # rad


def run_single_comparison(
    target_speed,
    simul_time=10,
    dt=0.005,
):

    print("\n============================================================")
    print(f"Running comparison for VX_REF = {target_speed:.2f} m/s")
    print("============================================================")

    vehicle = create_vehicle()

    diagram_dumb = create_diagram_dumb(vehicle, vx_ref=target_speed)
    diagram_vx = create_diagram_vx(vehicle, vx_ref=target_speed)

    traj = diagram_dumb.reconstruct_internal_signals(
        diagram_dumb.compute_trajectory(
            tf=simul_time,
            dt=dt,
            verbose=False,
        )
    )
    pid_logs = traj.get_signal("v_pid:logs")
    # dumb_pid_logs = dumb_traj.get_signal("v_pid:logs")

    dumb_ref = pid_logs[0, :]
    dumb_meas = pid_logs[1, :]

    traj = diagram_vx.reconstruct_internal_signals(
        diagram_vx.compute_trajectory(
            tf=simul_time,
            dt=dt,
            verbose=False,
        )
    )
    pid_logs = traj.get_signal("v_pid:logs")
    # vx_pid_logs = vx_traj.get_signal("v_pid:logs")

    vx_ref = pid_logs[0, :]
    vx_meas = pid_logs[1, :]

    return {
        "target_speed": target_speed,
        "fixed_w": {
            "t": traj.t,
            "ref": dumb_ref,
            "meas": dumb_meas,
        },
        "meas_w": {
            "t": traj.t,
            "ref": vx_ref,
            "meas": vx_meas,
        },
    }


def plot_multiple_speed_comparisons(results):
    """
    Plot speed tracking comparisons for multiple target speeds.

    One subplot is created for each target speed.
    """

    n = len(results)

    fig, axes = plt.subplots(
        nrows=n,
        ncols=1,
        figsize=(10, 3.0 * n),
        sharex=True,
    )

    if n == 1:
        axes = [axes]

    for ax, result in zip(axes, results):
        target_speed = result["target_speed"]

        t_fixed = result["fixed_w"]["t"]
        ref_fixed = result["fixed_w"]["ref"]
        meas_fixed = result["fixed_w"]["meas"]

        t_meas = result["meas_w"]["t"]
        meas_meas = result["meas_w"]["meas"]

        ax.plot(
            t_fixed,
            ref_fixed,
            "k--",
            label="Reference speed",
        )

        ax.plot(
            t_fixed,
            meas_fixed,
            label="Controller - fixed w",
        )

        ax.plot(
            t_meas,
            meas_meas,
            label="Controller - map w",
        )

        ax.set_ylabel("Speed [m/s]")
        ax.set_title(f"Speed tracking comparison at {target_speed:.2f} m/s")
        ax.grid(True)
        ax.legend()

    axes[-1].set_xlabel("Time [s]")

    plt.tight_layout()
    plt.show()


def plot_all_speeds_on_one_figure(results):
    """
    Plot all speeds on one figure.

    For each target speed:
    - all curves use the same color
    - target/fixed/measured are distinguished by line style
    """

    plt.figure(figsize=(11, 6))

    cmap = plt.get_cmap("viridis")
    colors = cmap(np.linspace(0, 1, len(results)))

    for result, color in zip(results, colors):
        target_speed = result["target_speed"]

        t_fixed = result["fixed_w"]["t"]
        meas_fixed = result["fixed_w"]["meas"]

        t_meas = result["meas_w"]["t"]
        meas_meas = result["meas_w"]["meas"]

        # Use the longest available time vector for the target line
        t_target = t_fixed if len(t_fixed) >= len(t_meas) else t_meas

        # Target/reference speed
        plt.plot(
            t_target,
            np.ones_like(t_target) * target_speed,
            color=color,
            linestyle=":",
            linewidth=2.5,
            label=f"Target - {target_speed:.2f} m/s",
        )

        # Fixed wheel-speed version
        plt.plot(
            t_fixed,
            meas_fixed,
            color=color,
            linestyle="-",
            linewidth=1.8,
            label=f"Fixed w - {target_speed:.2f} m/s",
        )

        # Measured wheel-speed version
        plt.plot(
            t_meas,
            meas_meas,
            color=color,
            linestyle="--",
            linewidth=1.8,
            label=f"Measured w - {target_speed:.2f} m/s",
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed tracking comparison for multiple target speeds")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    simul_time = 10
    dt = 0.005

    target_speeds = np.linspace(1.0, 30.0, 8)

    results = []

    total_start = time.perf_counter()

    for target_speed in target_speeds:
        sim_start = time.perf_counter()

        result = run_single_comparison(
            target_speed=target_speed,
            simul_time=simul_time,
            dt=dt,
        )

        sim_end = time.perf_counter()

        sim_elapsed = sim_end - sim_start

        print(f"Computation time for {target_speed:.2f} m/s: {sim_elapsed:.3f} seconds")

        results.append(result)

    total_end = time.perf_counter()

    total_elapsed = total_end - total_start

    print(f"\nTotal computation time: {total_elapsed:.3f} seconds")
    print(f"Average time per speed: {total_elapsed / len(target_speeds):.3f} seconds")

    # Recommended: one subplot per speed
    plot_multiple_speed_comparisons(results)

    # Optional: all curves on one figure
    plot_all_speeds_on_one_figure(results)


if __name__ == "__main__":
    main()
