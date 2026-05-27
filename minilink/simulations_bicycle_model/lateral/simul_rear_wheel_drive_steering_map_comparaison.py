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
from simul_rear_wheel_drive_steering_MAP_w_PID import (
    create_diagram as create_diagram_pid_w_steer_map,
)
from simul_rear_wheel_drive_steering_PID import (
    create_diagram as create_diagram_pid_only,
)

# Optional, only needed if you still want animations
from minilink.simulations_bicycle_model.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)

DELTA_REF = 0.0  # rad


def get_clean_pid_history(pid):
    """
    Extract, sort, and remove duplicate time entries from a PID history.

    Returns
    -------
    t : np.ndarray
        Time vector.
    ref : np.ndarray
        Reference signal history.
    meas : np.ndarray
        Measured signal history.
    """
    t = np.array(pid.t_hist)
    ref = np.array(pid.ref_hist)
    meas = np.array(pid.meas_hist)

    idx = np.argsort(t)
    t = t[idx]
    ref = ref[idx]
    meas = meas[idx]

    t_unique, unique_idx = np.unique(t, return_index=True)
    t = t_unique
    ref = ref[unique_idx]
    meas = meas[unique_idx]

    return t, ref, meas


def run_single_comparison(
    target_speed, target_r, simul_time, dt, plot_graph=False, animate=False
):
    """
    Run both simulations for one target speed.

    Parameters
    ----------
    target_speed : float
        Desired speed reference in rad/s.
    simul_time : float
        Simulation duration in seconds.
    dt : float
        Simulation time step.
    plot_graph : bool
        If True, plot the block diagrams.
    animate : bool
        If True, animate both simulations.

    Returns
    -------
    result : dict
        Dictionary containing cleaned PID histories for both simulations.
    """

    print("\n============================================================")
    print(f"Running comparison for R SPEED = {target_r:.2f} rad/s")
    print("============================================================")

    # -------------------------------------------------------------------------
    # First simulation: PID onlyheel-speed map / dumb version
    # -------------------------------------------------------------------------
    vehicle = create_vehicle(vx=target_speed)
    # To keep the constant Vx value seems like a reasonable solution (??)
    vehicle.engine_dry_resistance = 0.0
    vehicle.engine_rolling_resistance = 0.0

    diagram_fixed_w, v_pid_fixed_w = create_diagram_pid_only(
        vehicle, vx_ref=target_speed, r_ref=target_r
    )

    if plot_graph:
        diagram_fixed_w.plot_graphe()

    print("Starting trajectory computation for PID only simulation...")

    diagram_fixed_w.compute_trajectory(
        tf=simul_time,
        dt=dt,
        show=False,
        verbose=False,
        solver="scipy_lsoda",
    )

    print("Trajectory computation done for PID only simulation.")

    if animate:
        attach_vehicle_centered_diagram_camera(diagram_fixed_w, vehicle)
        diagram_fixed_w.animate(renderer="matplotlib")

    t_fixed, ref_fixed, meas_fixed = get_clean_pid_history(v_pid_fixed_w)

    # -------------------------------------------------------------------------
    # Second simulation: PID w mapheel-speed map / vx version
    # -------------------------------------------------------------------------
    diagram_meas_w, v_pid_meas_w = create_diagram_pid_w_steer_map(
        vehicle, vx_ref=target_speed, r_ref=target_r
    )

    if plot_graph:
        diagram_meas_w.plot_graphe()

    print("Starting trajectory computation for PID w map simulation...")

    diagram_meas_w.compute_trajectory(
        tf=simul_time,
        dt=dt,
        show=False,
        verbose=False,
        solver="scipy_lsoda",
    )

    print("Trajectory computation done for PID w map simulation.")

    if animate:
        attach_vehicle_centered_diagram_camera(diagram_meas_w, vehicle)
        diagram_meas_w.animate(renderer="matplotlib")

    t_meas, ref_meas, meas_meas = get_clean_pid_history(v_pid_meas_w)

    return {
        "target_r_speed": target_r,
        "fixed_w": {
            "t": t_fixed,
            "ref": ref_fixed,
            "meas": meas_fixed,
        },
        "meas_w": {
            "t": t_meas,
            "ref": ref_meas,
            "meas": meas_meas,
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
        target_r_speed = result["target_r_speed"]

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
            label="Controller - PID only",
        )

        ax.plot(
            t_meas,
            meas_meas,
            label="Controller - PID w map",
        )

        ax.set_ylabel("Speed [rad/s]")
        ax.set_title(f"Speed tracking comparison at {target_r_speed:.2f} rad/s")
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
        target_r_speed = result["target_r_speed"]

        t_fixed = result["fixed_w"]["t"]
        meas_fixed = result["fixed_w"]["meas"]

        t_meas = result["meas_w"]["t"]
        meas_meas = result["meas_w"]["meas"]

        # Use the longest available time vector for the target line
        t_target = t_fixed if len(t_fixed) >= len(t_meas) else t_meas

        # Target/reference speed
        plt.plot(
            t_target,
            np.ones_like(t_target) * target_r_speed,
            color=color,
            linestyle=":",
            linewidth=2.5,
            label=f"Target - {target_r_speed:.2f} rad/s",
        )

        # PID onlyheel-speed version
        plt.plot(
            t_fixed,
            meas_fixed,
            color=color,
            linestyle="-",
            linewidth=1.8,
            label=f"PID only - {target_r_speed:.2f} rad/s",
        )

        # PID w mapheel-speed version
        plt.plot(
            t_meas,
            meas_meas,
            color=color,
            linestyle="--",
            linewidth=1.8,
            label=f"PID w map - {target_r_speed:.2f} rad/s",
        )

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [rad/s]")
    plt.title("Speed tracking comparison for multiple target speeds")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    simul_time = 10.0
    dt = 0.02

    target_rs = np.linspace(0.1, 0.6, 6)

    target_speed = 2.0

    results = []

    total_start = time.perf_counter()

    for target_r in target_rs:
        sim_start = time.perf_counter()

        result = run_single_comparison(
            target_speed=target_speed,
            target_r=target_r,
            simul_time=simul_time,
            dt=dt,
            plot_graph=False,
            animate=False,
        )

        sim_end = time.perf_counter()

        sim_elapsed = sim_end - sim_start

        print(f"Computation time for {target_r:.2f} rad/s: {sim_elapsed:.3f} seconds")

        results.append(result)

    total_end = time.perf_counter()

    total_elapsed = total_end - total_start

    print(f"\nTotal computation time: {total_elapsed:.3f} seconds")
    print(f"Average time per speed: {total_elapsed / len(target_rs):.3f} seconds")

    # Recommended: one subplot per speed
    plot_multiple_speed_comparisons(results)

    # Optional: all curves on one figure
    plot_all_speeds_on_one_figure(results)


if __name__ == "__main__":
    main()
