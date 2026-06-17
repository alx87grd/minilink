"""Receding-horizon MPC for straight-line tracking on the dynamic bicycle.

Uses the existing trajectory-optimization pipeline as the MPC optimizer. A custom
simulation loop runs the plant between MPC updates:

- **MPC** at 10 Hz (zero-order hold on the first planned input).
- **Plant rollout** with compiled RK4 at a finer timestep between MPC ticks.

Optional top-down animation overlays the current MPC plan (world-frame ``x,y``)
at each 10 Hz replanning tick.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_dynamic_bicycle_mpc_straight_line.py
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import JaxDynamicBicycle
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
ANIMATE_MPC = False
ANIMATE_MPC_SAVE = None
ANIMATE_MPC_SHOW = True
ANIMATE_TIME_FACTOR = 2.5

MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))


@dataclass(frozen=True)
class MpcSnapshot:
    """One MPC solve stored in absolute simulation time."""

    t_solve: float
    plan: Trajectory


@dataclass(frozen=True)
class MpcRunResult:
    """Closed-loop rollout plus optional MPC plan history."""

    executed: Trajectory
    plans: tuple[MpcSnapshot, ...]


def make_plant() -> JaxDynamicBicycle:
    sys = JaxDynamicBicycle()
    sys.inputs["w_rear"].lower_bound[0] = 0.0
    sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
    sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
    sys.inputs["delta"].upper_bound[0] = DELTA_MAX
    return sys


def make_cost(sys: JaxDynamicBicycle) -> QuadraticCost:
    # Straight-line cruise along +x: y = 0, heading = 0, body slip suppressed.
    x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0])
    ubar = np.array([U_TARGET / sys.params["r_r"], 0.0])
    Q = np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0])
    R = np.diag([0.05, 35.0])
    S = np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0])
    return QuadraticCost.from_system(
        sys,
        Q=Q,
        R=R,
        S=S,
        xbar=x_ref,
        ubar=ubar,
    )


def make_planner(
    sys: JaxDynamicBicycle,
    cost: QuadraticCost,
    x_start: np.ndarray,
) -> TrajectoryOptimizationPlanner:
    problem = PlanningProblem(sys=sys, x_start=x_start, cost=cost)
    return TrajectoryOptimizationPlanner(
        problem,
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            solve_disp=False,
            warm_start=False,
            optimizer_options={"maxiter": 120, "ftol": 1e-4},
        ),
    )


def shift_warm_start(
    prev: Trajectory | None,
    x_current: np.ndarray,
    t_current: float,
) -> Trajectory | None:
    """Shift the previous MPC plan one step forward in time."""
    if prev is None or prev.n_samples < 3:
        return None

    t_shift = prev.t + MPC_DT
    mask = t_shift <= t_current + MPC_HORIZON + 1e-9
    if np.count_nonzero(mask) < 3:
        return None

    t_new = t_shift[mask]
    x_new = prev.x[:, mask]
    u_new = prev.u[:, mask]
    x_new[:, 0] = x_current
    return Trajectory(t=t_new - t_current, x=x_new, u=u_new)


def run_mpc_closed_loop(
    sys: JaxDynamicBicycle,
    cost: QuadraticCost,
    x0: np.ndarray,
    *,
    tf: float,
    record_plans: bool = False,
) -> MpcRunResult:
    sim_evaluator = sys.compile(backend="numpy", verbose=False)

    t_hist = [0.0]
    x_hist = [np.asarray(x0, dtype=float).copy()]
    u_hist = [np.zeros(sys.m)]
    plans: list[MpcSnapshot] = []

    x = np.asarray(x0, dtype=float).copy()
    t = 0.0
    u_hold = np.zeros(sys.m)
    prev_plan: Trajectory | None = None
    next_mpc_t = 0.0

    while t < tf - 1e-12:
        if t >= next_mpc_t - 1e-12:
            planner = make_planner(sys, cost, x)
            guess = shift_warm_start(prev_plan, x, t)
            plan = planner.compute_solution(initial_guess=guess)
            prev_plan = plan
            u_hold = plan.u[:, 0].copy()
            if record_plans:
                plans.append(
                    MpcSnapshot(
                        t_solve=t,
                        plan=Trajectory(
                            t=plan.t + t,
                            x=plan.x.copy(),
                            u=plan.u.copy(),
                        ),
                    )
                )
            next_mpc_t += MPC_DT

        for _ in range(SUBSTEPS):
            if t >= tf:
                break
            x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
            t += SIM_DT
            t_hist.append(t)
            x_hist.append(x.copy())
            u_hist.append(u_hold.copy())

    executed = Trajectory(
        t=np.asarray(t_hist),
        x=np.asarray(x_hist).T,
        u=np.asarray(u_hist).T,
    )
    return MpcRunResult(executed=executed, plans=tuple(plans))


def active_mpc_snapshot(
    plans: tuple[MpcSnapshot, ...],
    t_current: float,
) -> MpcSnapshot | None:
    active: MpcSnapshot | None = None
    for snap in plans:
        if snap.t_solve <= t_current + 1e-9:
            active = snap
    return active


def animate_mpc_plans(
    result: MpcRunResult,
    *,
    time_factor_video: float = ANIMATE_TIME_FACTOR,
    save_path: str | None = None,
    show: bool = True,
) -> str | None:
    """Top-down animation with the active MPC plan overlaid at each replan tick."""
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.transforms import Affine2D

    from minilink.graphical.animation.renderers.timing import (
        sim_index_for_frame,
        trajectory_frame_schedule,
    )

    traj = result.executed
    schedule = trajectory_frame_schedule(traj, time_factor_video)

    fig, ax = plt.subplots(figsize=(10, 4.5))
    x_pad = 8.0
    y_pad = 4.0
    x0, x1 = float(traj.x[0, 0]), float(traj.x[0, -1])
    ax.set_xlim(x0 - x_pad, x1 + x_pad)
    ax.set_ylim(-y_pad, y_pad)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    (ref_line,) = ax.plot([], [], "k--", lw=1.0, alpha=0.7, label="reference y=0")
    (executed_line,) = ax.plot([], [], color="tab:blue", lw=2.0, label="executed")
    (plan_line,) = ax.plot(
        [],
        [],
        color="tab:orange",
        ls="--",
        lw=2.0,
        label="MPC plan",
    )
    vehicle_patch = Polygon(
        np.array([[0.6, 0.25], [0.6, -0.25], [-0.6, 0.0]]),
        closed=True,
        facecolor="tab:blue",
        edgecolor="black",
        alpha=0.85,
        zorder=5,
    )
    ax.add_patch(vehicle_patch)
    time_text = ax.text(
        0.02,
        0.97,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
    )
    ax.legend(loc="upper right")

    ref_x = np.linspace(x0 - x_pad, x1 + x_pad, 2)
    ref_line.set_data(ref_x, np.zeros_like(ref_x))

    def _vehicle_triangle(x_pose: float, y_pose: float, theta: float) -> np.ndarray:
        verts = np.array([[0.6, 0.25], [0.6, -0.25], [-0.6, 0.0]])
        rot = Affine2D().rotate(theta).translate(x_pose, y_pose)
        return rot.transform(verts)

    def _update(frame_idx: int):
        sim_idx = sim_index_for_frame(frame_idx, schedule)
        t_now = float(traj.t[sim_idx])
        x_now = traj.x[:, sim_idx]

        executed_line.set_data(traj.x[0, : sim_idx + 1], traj.x[1, : sim_idx + 1])

        snap = active_mpc_snapshot(result.plans, t_now)
        if snap is None:
            plan_line.set_data([], [])
            plan_label = "MPC plan: —"
        else:
            mask = snap.plan.t >= t_now - 1e-9
            plan_line.set_data(snap.plan.x[0, mask], snap.plan.x[1, mask])
            plan_label = f"MPC plan @ t={snap.t_solve:.1f}s"

        vehicle_patch.set_xy(
            _vehicle_triangle(float(x_now[0]), float(x_now[1]), float(x_now[2]))
        )
        time_text.set_text(
            f"t = {t_now:.2f} s\n{plan_label}\n"
            f"y = {x_now[1]:+.3f} m, vx = {x_now[3]:.2f} m/s"
        )
        artists = (executed_line, plan_line, vehicle_patch, time_text)
        return artists

    ani = animation.FuncAnimation(
        fig,
        _update,
        frames=schedule.n_frames,
        interval=schedule.interval_ms,
        blit=True,
    )
    fig.suptitle("MPC closed loop — planned trajectory overlay")

    output_path: str | None = None
    if save_path is not None:
        output_path = save_path
        if not output_path.lower().endswith(".gif"):
            output_path = f"{output_path}.gif"
        writer = animation.PillowWriter(fps=schedule.target_fps)
        ani.save(output_path, writer=writer, dpi=120)
        print(f"  animation: {output_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return output_path


def tracking_metrics(traj: Trajectory) -> dict[str, float]:
    y = traj.x[1, :]
    theta = traj.x[2, :]
    vx = traj.x[3, :]
    vy = traj.x[4, :]
    yaw_rate = traj.x[5, :]
    tail = slice(-int(0.25 * traj.n_samples), None)
    return {
        "max_abs_y": float(np.max(np.abs(y))),
        "rms_y_tail": float(np.sqrt(np.mean(y[tail] ** 2))),
        "max_abs_theta": float(np.max(np.abs(theta))),
        "rms_vx_err_tail": float(np.sqrt(np.mean((vx[tail] - U_TARGET) ** 2))),
        "max_abs_vy": float(np.max(np.abs(vy))),
        "max_abs_yaw_rate": float(np.max(np.abs(yaw_rate))),
    }


def main() -> None:
    sys = make_plant()
    cost = make_cost(sys)

    # Perturbed start: lateral offset, heading error, and body slip.
    x0 = np.array([0.0, 1.8, 0.18, U_TARGET * 0.92, 0.35, 0.08])

    result = run_mpc_closed_loop(
        sys,
        cost,
        x0,
        tf=TF_SIM,
        record_plans=ANIMATE_MPC or ANIMATE_MPC_SAVE is not None,
    )
    traj = result.executed
    metrics = tracking_metrics(traj)

    print("MPC straight-line tracking")
    print(
        f"  mpc_hz={MPC_HZ}, sim_hz={SIM_HZ}, horizon={MPC_HORIZON}s, steps={MPC_STEPS}"
    )
    print(f"  substeps_per_mpc={SUBSTEPS}")
    for key, value in metrics.items():
        print(f"  {key}: {value:.4f}")

    if SHOW_PLOTS:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))
        axes[0].plot(traj.t, traj.x[0, :], label="x")
        axes[0].plot(traj.t, traj.x[1, :], label="y")
        axes[0].axhline(0.0, color="k", ls="--", lw=0.8)
        axes[0].set_ylabel("position [m]")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(traj.t, np.rad2deg(traj.x[2, :]), label="heading")
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
        plt.tight_layout()
        plt.savefig("/tmp/mpc_straight_line.png", dpi=120)
        print("  plot: /tmp/mpc_straight_line.png")

    if ANIMATE_MPC or ANIMATE_MPC_SAVE is not None:
        animate_mpc_plans(
            result,
            time_factor_video=ANIMATE_TIME_FACTOR,
            save_path=ANIMATE_MPC_SAVE,
            show=ANIMATE_MPC_SHOW,
        )


if __name__ == "__main__":
    main()
