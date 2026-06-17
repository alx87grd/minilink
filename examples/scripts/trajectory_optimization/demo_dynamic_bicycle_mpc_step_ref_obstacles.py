"""MPC lane step + corridor bounds + obstacle avoidance on the dynamic bicycle.

- Step reference: ``y_ref`` jumps at ``T_STEP`` (after the obstacle in x).
- Corridor: ``y`` in ``[Y_CORRIDOR_LO, Y_CORRIDOR_HI]``.
- Obstacle: forbidden ``(x, y)`` box via :class:`~minilink.core.sets.CallableSet`.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_dynamic_bicycle_mpc_step_ref_obstacles.py
"""

from pathlib import Path

import numpy as np

from minilink.core.backends import array_module, configure_jax
from minilink.core.costs import CostFunction
from minilink.core.sets import BoxSet, CallableSet, IntersectionSet
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


class LaneStepTrackingCost(CostFunction):
    """Quadratic tracking with a step change in lateral reference vs time."""

    def __init__(self, Q, R, S, ubar, vx_target, y_before, y_after, t_step):
        self.Q = Q
        self.R = R
        self.S = S
        self.ubar = ubar
        self.vx_target = vx_target
        self.y_before = y_before
        self.y_after = y_after
        self.t_step = t_step
        self.t_origin = 0.0

    def xbar(self, t_abs):
        xp = array_module(t_abs)
        y_ref = xp.where(t_abs < self.t_step, self.y_before, self.y_after)
        return xp.stack([0.0, y_ref, 0.0, self.vx_target, 0.0, 0.0])

    def g(self, x, u, t=0.0, params=None):
        xb = self.xbar(t + self.t_origin)
        dx = x - xb
        du = u - self.ubar
        return dx.T @ self.Q @ dx + du.T @ self.R @ du

    def h(self, x, t=0.0, params=None):
        xb = self.xbar(t + self.t_origin)
        dx = x - xb
        return dx.T @ self.S @ dx


# --- Scenario ---
U_TARGET = 5.0
TF_SIM = 20.0
T_STEP = 11.0
Y_REF_BEFORE = 0.0
Y_REF_AFTER = 2.8
Y_CORRIDOR_LO = -2.5
Y_CORRIDOR_HI = 4.2
OBS_X = (36.0, 48.0)
OBS_Y = (0.6, 2.4)
OBS_INFLATE = 0.35

MPC_HZ = 10.0
SIM_HZ = 50.0
MPC_HORIZON = 3.0
MPC_STEPS = 20
W_REAR_MAX = 90.0
DELTA_MAX = 0.55
SHOW_PLOTS = True
SAVE_GIF = None
SAVE_NATIVE_PLOT = True

OUTPUT_DIR = Path(__file__).resolve().parent / "outputs" / "mpc_step_ref_obstacles"
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

if __name__ == "__main__":
    sys = JaxDynamicBicycle()
    sys.inputs["w_rear"].lower_bound[0] = 0.0
    sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
    sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
    sys.inputs["delta"].upper_bound[0] = DELTA_MAX

    ubar = np.array([U_TARGET / sys.params["r_r"], 0.0])
    cost = LaneStepTrackingCost(
        Q=np.diag([0.0, 14.0, 20.0, 0.6, 5.0, 7.0]),
        R=np.diag([0.05, 40.0]),
        S=np.diag([0.0, 35.0, 45.0, 2.5, 14.0, 20.0]),
        ubar=ubar,
        vx_target=U_TARGET,
        y_before=Y_REF_BEFORE,
        y_after=Y_REF_AFTER,
        t_step=T_STEP,
    )

    obs_x_lo = OBS_X[0] - OBS_INFLATE
    obs_x_hi = OBS_X[1] + OBS_INFLATE
    obs_y_lo = OBS_Y[0] - OBS_INFLATE
    obs_y_hi = OBS_Y[1] + OBS_INFLATE

    def obstacle_margin(z, t=0.0, params=None):
        xp = array_module(z)
        outside = xp.maximum(
            xp.maximum(obs_x_lo - z[0], z[0] - obs_x_hi),
            xp.maximum(obs_y_lo - z[1], z[1] - obs_y_hi),
        )
        return xp.reshape(outside, (1,))

    big = 1.0e6
    state_bounds = IntersectionSet(
        [
            BoxSet(
                lower=np.array([-big, Y_CORRIDOR_LO, -np.pi, 0.0, -big, -big]),
                upper=np.array([big, Y_CORRIDOR_HI, np.pi, big, big, big]),
            ),
            CallableSet(margin_fn=obstacle_margin),
        ]
    )

    x0 = np.array([0.0, 0.15, 0.03, U_TARGET, 0.05, 0.02])
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
            cost.t_origin = t
            problem = PlanningProblem(sys=sys, x_start=x, cost=cost, X=state_bounds)
            planner = TrajectoryOptimizationPlanner(
                problem,
                transcription=DirectCollocationTranscription(
                    DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
                ),
                options=TrajectoryOptimizationOptions(
                    compile_backend="jax",
                    solve_disp=False,
                    optimizer_options={"maxiter": 160, "ftol": 1e-4},
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

    obs_margin = np.maximum.reduce(
        [
            OBS_X[0] - traj.x[0, :],
            traj.x[0, :] - OBS_X[1],
            OBS_Y[0] - traj.x[1, :],
            traj.x[1, :] - OBS_Y[1],
        ],
        axis=0,
    )
    obs_pen = float(np.max(np.maximum(0.0, -obs_margin)))

    tail = slice(-int(0.25 * traj.n_samples), None)
    y_ref_tail = np.where(traj.t[tail] >= T_STEP, Y_REF_AFTER, Y_REF_BEFORE)
    print("MPC step reference + obstacle avoidance")
    print(
        f"  y_ref: {Y_REF_BEFORE} -> {Y_REF_AFTER} at t={T_STEP}s, "
        f"corridor y in [{Y_CORRIDOR_LO}, {Y_CORRIDOR_HI}]"
    )
    print(f"  obstacle x in {OBS_X}, forbidden y in {OBS_Y}")
    print(f"  max_obstacle_penetration: {obs_pen:.4f} m")
    print(
        "  rms_y_err_tail="
        f"{np.sqrt(np.mean((traj.x[1, tail] - y_ref_tail) ** 2)):.4f} m"
    )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if SHOW_PLOTS:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle

        y_ref = np.where(traj.t < T_STEP, Y_REF_BEFORE, Y_REF_AFTER)
        x_step = U_TARGET * T_STEP

        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
        axes[0].plot(traj.t, traj.x[0, :], label="x")
        axes[0].plot(traj.t, traj.x[1, :], label="y")
        axes[0].plot(traj.t, y_ref, "k--", lw=1.0, alpha=0.8, label="y_ref")
        axes[0].axhline(Y_CORRIDOR_LO, color="0.5", ls=":", lw=0.8)
        axes[0].axhline(Y_CORRIDOR_HI, color="0.5", ls=":", lw=0.8)
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
        fig.suptitle("MPC — step reference with corridor and obstacle bounds")
        fig.tight_layout()
        fig.savefig(OUTPUT_DIR / "tracking_signals.png", dpi=120)
        plt.close(fig)

        fig2, ax2 = plt.subplots(figsize=(11, 6))
        ax2.plot(traj.x[0, :], traj.x[1, :], color="tab:blue", lw=2.0, label="executed")
        ax2.plot(
            [0.0, x_step, x_step, float(traj.x[0, -1]) + 5.0],
            [Y_REF_BEFORE, Y_REF_BEFORE, Y_REF_AFTER, Y_REF_AFTER],
            "k--",
            lw=1.4,
            label="y_ref step",
        )
        ax2.axhline(Y_CORRIDOR_LO, color="0.55", ls=":", lw=1.0)
        ax2.axhline(Y_CORRIDOR_HI, color="0.55", ls=":", lw=1.0)
        ax2.add_patch(
            Rectangle(
                (OBS_X[0], OBS_Y[0]),
                OBS_X[1] - OBS_X[0],
                OBS_Y[1] - OBS_Y[0],
                facecolor="tab:red",
                edgecolor="darkred",
                alpha=0.35,
                label="obstacle",
            )
        )
        ax2.set_aspect("auto")
        ax2.grid(True, alpha=0.3)
        ax2.set_xlabel("x [m]")
        ax2.set_ylabel("y [m]")
        ax2.legend(loc="upper left")
        ax2.set_title("Top-down path vs reference and bounds")
        fig2.tight_layout()
        fig2.savefig(OUTPUT_DIR / "path_xy.png", dpi=120)
        plt.close(fig2)
        print(f"  plot: {OUTPUT_DIR / 'tracking_signals.png'}")
        print(f"  plot: {OUTPUT_DIR / 'path_xy.png'}")

    if SAVE_GIF is not None:
        import matplotlib.animation as animation
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
        from matplotlib.transforms import Affine2D

        from minilink.graphical.animation.renderers.timing import (
            sim_index_for_frame,
            trajectory_frame_schedule,
        )

        schedule = trajectory_frame_schedule(traj, time_factor_video=2.5)
        fig, ax = plt.subplots(figsize=(11, 7))
        x_pad = 8.0
        ax.set_xlim(float(traj.x[0, 0]) - x_pad, float(traj.x[0, -1]) + x_pad)
        ax.set_ylim(Y_CORRIDOR_LO - 0.4, Y_CORRIDOR_HI + 0.4)
        ax.set_aspect("auto")
        ax.grid(True, alpha=0.3)
        ax.axhline(Y_CORRIDOR_LO, color="0.55", ls=":", lw=1.0)
        ax.axhline(Y_CORRIDOR_HI, color="0.55", ls=":", lw=1.0)
        ax.add_patch(
            Rectangle(
                (OBS_X[0], OBS_Y[0]),
                OBS_X[1] - OBS_X[0],
                OBS_Y[1] - OBS_Y[0],
                facecolor="tab:red",
                edgecolor="darkred",
                alpha=0.35,
                label="obstacle",
            )
        )
        x_step = U_TARGET * T_STEP
        ax.plot(
            [0.0, x_step, x_step, float(traj.x[0, -1]) + x_pad],
            [Y_REF_BEFORE, Y_REF_BEFORE, Y_REF_AFTER, Y_REF_AFTER],
            "k--",
            lw=1.4,
            alpha=0.85,
            label="y_ref step",
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
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
        )
        ax.legend(loc="upper right")

        def update(frame_idx):
            sim_idx = sim_index_for_frame(frame_idx, schedule)
            t_now = float(traj.t[sim_idx])
            x_now = traj.x[:, sim_idx]
            y_ref_now = Y_REF_BEFORE if t_now < T_STEP else Y_REF_AFTER
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
                f"y = {x_now[1]:+.3f} m, y_ref = {y_ref_now:.1f} m"
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
