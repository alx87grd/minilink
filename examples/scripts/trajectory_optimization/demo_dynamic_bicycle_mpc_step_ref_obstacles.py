"""MPC lane step + corridor bounds + obstacle avoidance on the dynamic bicycle.

Extends the straight-line MPC prototype with:

- **Step reference:** ``y_ref`` jumps from 0 to ``Y_REF_AFTER`` at ``T_STEP``.
- **Corridor bounds:** state ``y`` kept inside ``[Y_CORRIDOR_LO, Y_CORRIDOR_HI]``.
- **Obstacle:** axis-aligned forbidden region in ``(x, y)`` enforced through a
  :class:`~minilink.core.sets.CallableSet` path constraint.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_dynamic_bicycle_mpc_step_ref_obstacles.py
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from minilink.core.backends import array_module, configure_jax
from minilink.core.costs import CostFunction
from minilink.core.sets import BoxSet, CallableSet, IntersectionSet
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
TF_SIM = 20.0
T_STEP = 6.0
Y_REF_BEFORE = 0.0
Y_REF_AFTER = 2.8
Y_CORRIDOR_LO = -2.5
Y_CORRIDOR_HI = 4.2
OBS_X = (34.0, 46.0)
OBS_Y = (-1.0, 2.2)

MPC_HZ = 10.0
SIM_HZ = 50.0
MPC_HORIZON = 3.0
MPC_STEPS = 20
W_REAR_MAX = 90.0
DELTA_MAX = 0.55
SHOW_PLOTS = True
ANIMATE_MPC_SAVE = None
ANIMATE_MPC_SHOW = False
ANIMATE_TIME_FACTOR = 2.5

_SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = _SCRIPT_DIR / "outputs" / "mpc_step_ref_obstacles"
PLOT_SAVE = OUTPUT_DIR / "tracking_signals.png"
ANIMATION_SAVE = OUTPUT_DIR / "mpc_plan_overlay.gif"

MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))


@dataclass
class LaneStepTrackingCost(CostFunction):
    """Quadratic tracking with a step change in lateral reference vs time."""

    Q: np.ndarray
    R: np.ndarray
    S: np.ndarray
    ubar: np.ndarray
    vx_target: float
    y_before: float
    y_after: float
    t_step: float
    t_origin: float = 0.0

    def y_ref(self, t_abs: float) -> float:
        return self.y_before if t_abs < self.t_step else self.y_after

    def xbar(self, t_abs):
        xp = array_module(t_abs)
        y_ref = xp.where(t_abs < self.t_step, self.y_before, self.y_after)
        return xp.stack([0.0, y_ref, 0.0, self.vx_target, 0.0, 0.0])

    def g(self, x, u, t=0.0, params=None):
        t_abs = t + self.t_origin
        xb = self.xbar(t_abs)
        dx = x - xb
        du = u - self.ubar
        return dx.T @ self.Q @ dx + du.T @ self.R @ du

    def h(self, x, t=0.0, params=None):
        t_abs = t + self.t_origin
        xb = self.xbar(t_abs)
        dx = x - xb
        return dx.T @ self.S @ dx


@dataclass(frozen=True)
class ObstacleSpec:
    x_lo: float
    x_hi: float
    y_lo: float
    y_hi: float


@dataclass(frozen=True)
class MpcSnapshot:
    t_solve: float
    plan: Trajectory


@dataclass(frozen=True)
class MpcRunResult:
    executed: Trajectory
    plans: tuple[MpcSnapshot, ...]


def make_plant() -> JaxDynamicBicycle:
    sys = JaxDynamicBicycle()
    sys.inputs["w_rear"].lower_bound[0] = 0.0
    sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
    sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
    sys.inputs["delta"].upper_bound[0] = DELTA_MAX
    return sys


def make_cost(sys: JaxDynamicBicycle, *, t_origin: float = 0.0) -> LaneStepTrackingCost:
    ubar = np.array([U_TARGET / sys.params["r_r"], 0.0])
    Q = np.diag([0.0, 14.0, 20.0, 0.6, 5.0, 7.0])
    R = np.diag([0.05, 40.0])
    S = np.diag([0.0, 35.0, 45.0, 2.5, 14.0, 20.0])
    return LaneStepTrackingCost(
        Q=Q,
        R=R,
        S=S,
        ubar=ubar,
        vx_target=U_TARGET,
        y_before=Y_REF_BEFORE,
        y_after=Y_REF_AFTER,
        t_step=T_STEP,
        t_origin=t_origin,
    )


def obstacle_margin_fn(spec: ObstacleSpec):
    """Nonnegative margin when the pose stays outside the forbidden box."""

    def margin(z, t=0.0, params=None):
        xp = array_module(z)
        x = z[0]
        y = z[1]
        outside = xp.maximum(
            xp.maximum(spec.x_lo - x, x - spec.x_hi),
            xp.maximum(spec.y_lo - y, y - spec.y_hi),
        )
        return xp.reshape(outside, (1,))

    return margin


def make_state_bounds() -> IntersectionSet:
    big = 1.0e6
    corridor = BoxSet(
        lower=np.array([-big, Y_CORRIDOR_LO, -np.pi, 0.0, -big, -big]),
        upper=np.array([big, Y_CORRIDOR_HI, np.pi, big, big, big]),
    )
    obstacle = CallableSet(margin_fn=obstacle_margin_fn(ObstacleSpec(*OBS_X, *OBS_Y)))
    return IntersectionSet([corridor, obstacle])


def make_planner(
    sys: JaxDynamicBicycle,
    cost: LaneStepTrackingCost,
    x_start: np.ndarray,
) -> TrajectoryOptimizationPlanner:
    problem = PlanningProblem(
        sys=sys,
        x_start=x_start,
        cost=cost,
        X=make_state_bounds(),
    )
    return TrajectoryOptimizationPlanner(
        problem,
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            solve_disp=False,
            warm_start=False,
            optimizer_options={"maxiter": 160, "ftol": 1e-4},
        ),
    )


def shift_warm_start(
    prev: Trajectory | None,
    x_current: np.ndarray,
    t_current: float,
) -> Trajectory | None:
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
    cost_template: LaneStepTrackingCost,
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
            cost = replace(cost_template, t_origin=t)
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


def reference_xy_polyline(x_max: float) -> tuple[np.ndarray, np.ndarray]:
    x_step = U_TARGET * T_STEP
    xs = np.array([0.0, x_step, x_step, x_max])
    ys = np.array([Y_REF_BEFORE, Y_REF_BEFORE, Y_REF_AFTER, Y_REF_AFTER])
    return xs, ys


def active_mpc_snapshot(
    plans: tuple[MpcSnapshot, ...],
    t_current: float,
) -> MpcSnapshot | None:
    active: MpcSnapshot | None = None
    for snap in plans:
        if snap.t_solve <= t_current + 1e-9:
            active = snap
    return active


def animate_mpc_scenario(
    result: MpcRunResult,
    *,
    time_factor_video: float = ANIMATE_TIME_FACTOR,
    save_path: str | None = None,
    show: bool = True,
) -> str | None:
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    from matplotlib.transforms import Affine2D

    from minilink.graphical.animation.renderers.timing import (
        sim_index_for_frame,
        trajectory_frame_schedule,
    )

    traj = result.executed
    schedule = trajectory_frame_schedule(traj, time_factor_video)

    fig, ax = plt.subplots(figsize=(11, 7))
    x_pad = 8.0
    x0, x1 = float(traj.x[0, 0]), float(traj.x[0, -1])
    ax.set_xlim(x0 - x_pad, x1 + x_pad)
    ax.set_ylim(Y_CORRIDOR_LO - 0.4, Y_CORRIDOR_HI + 0.4)
    ax.set_aspect("auto")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

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
            zorder=1,
        )
    )

    ref_x, ref_y = reference_xy_polyline(x1 + x_pad)
    (ref_line,) = ax.plot(ref_x, ref_y, "k--", lw=1.4, alpha=0.85, label="y_ref step")
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
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )
    ax.legend(loc="upper right")

    def _vehicle_triangle(x_pose: float, y_pose: float, theta: float) -> np.ndarray:
        verts = np.array([[0.6, 0.25], [0.6, -0.25], [-0.6, 0.0]])
        rot = Affine2D().rotate(theta).translate(x_pose, y_pose)
        return rot.transform(verts)

    def _update(frame_idx: int):
        sim_idx = sim_index_for_frame(frame_idx, schedule)
        t_now = float(traj.t[sim_idx])
        x_now = traj.x[:, sim_idx]
        y_ref_now = Y_REF_BEFORE if t_now < T_STEP else Y_REF_AFTER

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
            f"y = {x_now[1]:+.3f} m, y_ref = {y_ref_now:.1f} m\n"
            f"vx = {x_now[3]:.2f} m/s"
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
    fig.suptitle(
        "MPC — step reference, corridor bounds, obstacle (y scaled for visibility)"
    )

    output_path: str | None = None
    if save_path is not None:
        output_path = save_path
        if not output_path.lower().endswith(".gif"):
            output_path = f"{output_path}.gif"
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        writer = animation.PillowWriter(fps=schedule.target_fps)
        ani.save(output_path, writer=writer, dpi=120)
        print(f"  animation: {output_path}")
        _export_gif_preview_frames(Path(output_path))

    if show:
        plt.show()
    else:
        plt.close(fig)

    return output_path


def _export_gif_preview_frames(gif_path: Path) -> None:
    from PIL import Image

    gif = Image.open(gif_path)
    n_frames = int(getattr(gif, "n_frames", 1))
    preview_indices = tuple(
        sorted({0, n_frames // 5, n_frames // 2, 4 * n_frames // 5, n_frames - 1})
    )
    for idx in preview_indices:
        gif.seek(idx)
        frame_path = gif_path.with_name(f"{gif_path.stem}_frame_{idx:03d}.png")
        gif.save(frame_path)
        print(f"  frame: {frame_path}")


def tracking_metrics(traj: Trajectory) -> dict[str, float]:
    y = traj.x[1, :]
    t = traj.t
    tail_mask = t >= 0.75 * t[-1]
    y_ref_tail = np.where(t[tail_mask] >= T_STEP, Y_REF_AFTER, Y_REF_BEFORE)
    y_err_tail = y[tail_mask] - y_ref_tail
    return {
        "max_abs_y": float(np.max(np.abs(y))),
        "rms_y_err_tail": float(np.sqrt(np.mean(y_err_tail**2))),
        "min_y": float(np.min(y)),
        "max_y": float(np.max(y)),
        "rms_vx_err_tail": float(
            np.sqrt(np.mean((traj.x[3, tail_mask] - U_TARGET) ** 2))
        ),
    }


def plot_results(traj: Trajectory) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

    y_ref = np.where(traj.t < T_STEP, Y_REF_BEFORE, Y_REF_AFTER)
    axes[0].plot(traj.t, traj.x[0, :], label="x")
    axes[0].plot(traj.t, traj.x[1, :], label="y")
    axes[0].plot(traj.t, y_ref, "k--", lw=1.0, alpha=0.8, label="y_ref")
    axes[0].axhline(Y_CORRIDOR_LO, color="0.5", ls=":", lw=0.8)
    axes[0].axhline(Y_CORRIDOR_HI, color="0.5", ls=":", lw=0.8)
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
    fig.suptitle("MPC — step reference with corridor and obstacle bounds")

    fig2, ax2 = plt.subplots(figsize=(11, 6))
    ax2.plot(traj.x[0, :], traj.x[1, :], color="tab:blue", lw=2.0, label="executed")
    ref_x, ref_y = reference_xy_polyline(float(traj.x[0, -1]) + 5.0)
    ax2.plot(ref_x, ref_y, "k--", lw=1.4, label="y_ref step")
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

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(PLOT_SAVE, dpi=120)
    xy_path = OUTPUT_DIR / "path_xy.png"
    fig2.tight_layout()
    fig2.savefig(xy_path, dpi=120)
    print(f"  plot: {PLOT_SAVE}")
    print(f"  plot: {xy_path}")
    plt.close(fig)
    plt.close(fig2)


def main() -> None:
    sys = make_plant()
    cost = make_cost(sys)

    x0 = np.array([0.0, 0.15, 0.03, U_TARGET, 0.05, 0.02])
    record = ANIMATE_MPC_SAVE is not None

    result = run_mpc_closed_loop(
        sys,
        cost,
        x0,
        tf=TF_SIM,
        record_plans=record,
    )
    traj = result.executed
    metrics = tracking_metrics(traj)

    print("MPC step reference + obstacle avoidance")
    print(
        f"  y_ref: {Y_REF_BEFORE} -> {Y_REF_AFTER} at t={T_STEP}s, "
        f"corridor y in [{Y_CORRIDOR_LO}, {Y_CORRIDOR_HI}]"
    )
    print(f"  obstacle x in {OBS_X}, forbidden y in {OBS_Y}")
    for key, value in metrics.items():
        print(f"  {key}: {value:.4f}")

    if SHOW_PLOTS:
        plot_results(traj)

    if ANIMATE_MPC_SAVE is not None:
        animate_mpc_scenario(
            result,
            time_factor_video=ANIMATE_TIME_FACTOR,
            save_path=ANIMATE_MPC_SAVE,
            show=ANIMATE_MPC_SHOW,
        )


if __name__ == "__main__":
    main()
