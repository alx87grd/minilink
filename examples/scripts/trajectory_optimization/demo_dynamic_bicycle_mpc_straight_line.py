"""Receding-horizon MPC for straight-line tracking on the dynamic bicycle.

MPC at 10 Hz (ZOH on the first planned input); plant rolled out with RK4 at 50 Hz
between ticks. Optional top-down GIF overlays the active MPC plan.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_dynamic_bicycle_mpc_straight_line.py
"""

from pathlib import Path

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleCar3D,
    JaxDynamicBicycle,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- Scenario ---
U_TARGET = 5.0
TF_SIM = 12.0
MPC_HZ = 10.0
SIM_HZ = 50.0
MPC_HORIZON = 2.5
MPC_STEPS = 18
W_REAR_MAX = 90.0
DELTA_MAX = 0.55
SHOW_PLOTS = True
SAVE_GIF = None
SAVE_NATIVE_PLOT = True

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs" / "mpc_straight_line"
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

if __name__ == "__main__":
    sys = JaxDynamicBicycle()
    sys.inputs["w_rear"].lower_bound[0] = 0.0
    sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
    sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
    sys.inputs["delta"].upper_bound[0] = DELTA_MAX

    x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0])
    ubar = np.array([U_TARGET / sys.params["r_r"], 0.0])
    cost = QuadraticCost.from_system(
        sys,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0]),
        R=np.diag([0.05, 35.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0]),
        xbar=x_ref,
        ubar=ubar,
    )

    x0 = np.array([0.0, 1.8, 0.18, U_TARGET * 0.92, 0.35, 0.08])
    sim_evaluator = sys.compile(backend="numpy", verbose=False)

    t_hist = [0.0]
    x_hist = [x0.copy()]
    u_hist = [np.zeros(sys.m)]
    mpc_plans = []
    x = x0.copy()
    t = 0.0
    u_hold = np.zeros(sys.m)
    prev_plan = None
    next_mpc_t = 0.0

    while t < TF_SIM - 1e-12:
        if t >= next_mpc_t - 1e-12:
            problem = PlanningProblem(sys=sys, x_start=x, cost=cost)
            planner = TrajectoryOptimizationPlanner(
                problem,
                transcription=DirectCollocationTranscription(
                    DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
                ),
                options=TrajectoryOptimizationOptions(
                    compile_backend="jax",
                    solve_disp=False,
                    optimizer_options={"maxiter": 120, "ftol": 1e-4},
                ),
            )

            guess = None
            if prev_plan is not None and prev_plan.n_samples >= 3:
                t_shift = prev_plan.t + MPC_DT
                mask = t_shift <= t + MPC_HORIZON + 1e-9
                if np.count_nonzero(mask) >= 3:
                    x_guess = prev_plan.x[:, mask].copy()
                    x_guess[:, 0] = x
                    guess = Trajectory(
                        t=t_shift[mask] - t,
                        x=x_guess,
                        u=prev_plan.u[:, mask],
                    )

            plan = planner.compute_solution(initial_guess=guess)
            prev_plan = plan
            u_hold = plan.u[:, 0].copy()
            if SAVE_GIF is not None:
                mpc_plans.append(
                    (t, Trajectory(t=plan.t + t, x=plan.x.copy(), u=plan.u.copy()))
                )
            next_mpc_t += MPC_DT

        for _ in range(SUBSTEPS):
            if t >= TF_SIM:
                break
            x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
            t += SIM_DT
            t_hist.append(t)
            x_hist.append(x.copy())
            u_hist.append(u_hold.copy())

    traj = Trajectory(
        t=np.asarray(t_hist),
        x=np.asarray(x_hist).T,
        u=np.asarray(u_hist).T,
    )

    tail = slice(-int(0.25 * traj.n_samples), None)
    print("MPC straight-line tracking")
    print(f"  mpc_hz={MPC_HZ}, sim_hz={SIM_HZ}, horizon={MPC_HORIZON}s")
    print(f"  rms_y_tail={np.sqrt(np.mean(traj.x[1, tail] ** 2)):.4f} m")
    print(
        "  rms_vx_err_tail="
        f"{np.sqrt(np.mean((traj.x[3, tail] - U_TARGET) ** 2)):.4f} m/s"
    )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if SHOW_PLOTS:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
        axes[0].plot(traj.t, traj.x[0, :], label="x")
        axes[0].plot(traj.t, traj.x[1, :], label="y")
        axes[0].axhline(0.0, color="k", ls="--", lw=0.8)
        axes[0].set_ylabel("position [m]")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(traj.t, np.rad2deg(traj.x[2, :]))
        axes[1].set_ylabel("heading [deg]")
        axes[1].grid(True, alpha=0.3)
        axes[2].plot(traj.t, traj.x[3, :], label="vx")
        axes[2].axhline(U_TARGET, color="k", ls="--", lw=0.8)
        axes[2].set_ylabel("surge [m/s]")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        axes[3].plot(traj.t, traj.u[0, :], label="w_rear")
        axes[3].plot(traj.t, traj.u[1, :], label="delta")
        axes[3].set_ylabel("inputs")
        axes[3].set_xlabel("time [s]")
        axes[3].legend()
        axes[3].grid(True, alpha=0.3)
        fig.suptitle("MPC closed loop — straight-line tracking")
        fig.tight_layout()
        plot_path = OUTPUT_DIR / "tracking_signals.png"
        fig.savefig(plot_path, dpi=120)
        plt.close(fig)
        print(f"  plot: {plot_path}")

    if SAVE_GIF is not None:
        import matplotlib.animation as animation
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        from matplotlib.transforms import Affine2D

        from minilink.graphical.animation.renderers.timing import (
            sim_index_for_frame,
            trajectory_frame_schedule,
        )

        schedule = trajectory_frame_schedule(traj, time_factor_video=2.5)
        fig, ax = plt.subplots(figsize=(10, 7))
        x_pad = 8.0
        y_pad = max(2.0, 1.25 * float(np.max(np.abs(traj.x[1, :]))) + 0.25)
        ax.set_xlim(float(traj.x[0, 0]) - x_pad, float(traj.x[0, -1]) + x_pad)
        ax.set_ylim(-y_pad, y_pad)
        ax.set_aspect("auto")
        ax.grid(True, alpha=0.3)
        ax.plot(
            [ax.get_xlim()[0], ax.get_xlim()[1]],
            [0.0, 0.0],
            "k--",
            lw=1.0,
            alpha=0.7,
            label="reference y=0",
        )
        (executed_line,) = ax.plot([], [], color="tab:blue", lw=2.0, label="executed")
        (plan_line,) = ax.plot(
            [], [], color="tab:orange", ls="--", lw=2.0, label="MPC plan"
        )
        vehicle = Polygon(
            [[0.6, 0.25], [0.6, -0.25], [-0.6, 0.0]],
            closed=True,
            facecolor="tab:blue",
            edgecolor="black",
            alpha=0.85,
            zorder=5,
        )
        ax.add_patch(vehicle)
        status = ax.text(
            0.02,
            0.97,
            "",
            transform=ax.transAxes,
            va="top",
            fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
        )
        ax.legend(loc="upper right")

        def update(frame_idx):
            sim_idx = sim_index_for_frame(frame_idx, schedule)
            t_now = float(traj.t[sim_idx])
            x_now = traj.x[:, sim_idx]
            executed_line.set_data(traj.x[0, : sim_idx + 1], traj.x[1, : sim_idx + 1])
            active = None
            for t_solve, plan in mpc_plans:
                if t_solve <= t_now + 1e-9:
                    active = (t_solve, plan)
            if active is None:
                plan_line.set_data([], [])
                label = "MPC plan: —"
            else:
                t_solve, plan = active
                mask = plan.t >= t_now - 1e-9
                plan_line.set_data(plan.x[0, mask], plan.x[1, mask])
                label = f"MPC plan @ t={t_solve:.1f}s"
            rot = (
                Affine2D()
                .rotate(float(x_now[2]))
                .translate(float(x_now[0]), float(x_now[1]))
            )
            vehicle.set_xy(
                rot.transform(np.array([[0.6, 0.25], [0.6, -0.25], [-0.6, 0.0]]))
            )
            status.set_text(
                f"t = {t_now:.2f} s\n{label}\n"
                f"y = {x_now[1]:+.3f} m, vx = {x_now[3]:.2f} m/s"
            )
            return executed_line, plan_line, vehicle, status

        ani = animation.FuncAnimation(
            fig,
            update,
            frames=schedule.n_frames,
            interval=schedule.interval_ms,
            blit=True,
        )
        gif_path = Path(SAVE_GIF)
        if gif_path.suffix.lower() != ".gif":
            gif_path = gif_path.with_suffix(".gif")
        ani.save(
            gif_path, writer=animation.PillowWriter(fps=schedule.target_fps), dpi=120
        )
        plt.close(fig)
        print(f"  animation: {gif_path}")

    viz = DynamicBicycleCar3D()
    viz.params = dict(sys.params)
    viz.traj = traj
    np.savez(OUTPUT_DIR / "executed_traj.npz", t=traj.t, x=traj.x, u=traj.u)
    print(f"  executed traj: {OUTPUT_DIR / 'executed_traj.npz'}")
    if SAVE_NATIVE_PLOT:
        viz.plot_trajectory(signals=("x", "u"), show=False)
        print("  native plot_trajectory: done")
