"""Compare obstacle-scene TrajOpt across six JAX plants.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_bicycle_trajopt_obstacle_scene_compare.py

Same single-obstacle mission as
``demo_kinematic_bicycle_trajopt_obstacle_scene.py``, solved with direct
collocation on (simplest → most complex by state dimension):

- :class:`~minilink.dynamics.catalog.vehicles.steering.JaxHolonomicMobileRobot` (n=2)
- :class:`~minilink.dynamics.catalog.vehicles.steering.JaxKinematicBicycle` (n=3)
- :class:`~minilink.dynamics.catalog.vehicles.steering.JaxDynamicHolonomicMobileRobot` (n=4)
- :class:`~minilink.dynamics.catalog.vehicles.steering.JaxKinematicBicycleRateInputs` (n=5)
- :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycle` (n=6)
- :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.JaxDynamicBicycleRateInputs` (n=8)

Prints solve-time summary, shows side-by-side path and speed plots, then plays
six animations (one per plant).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycle,
    JaxDynamicBicycleRateInputs,
)
from minilink.dynamics.catalog.vehicles.steering import (
    JaxDynamicHolonomicMobileRobot,
    JaxHolonomicMobileRobot,
    JaxKinematicBicycle,
    JaxKinematicBicycleRateInputs,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, point_probe
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import inverse_barrier
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

TF = 4.0
N_STEPS = 20
U_0 = 8.0
U_TARGET = 10.0
Y_START = 0.0
Y_GOAL = 0.0
HEADING_TARGET = 0.0
DELTA_MAX = 0.25
V_MAX = 20.0
SPEED_DOT_MAX = 3.0
STEERING_DOT_MAX = 2.0
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0

OBSTACLE_CENTER = (12.0, 0.2)
OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.2
OBSTACLE_REPULSION_WEIGHT = 1000.0
OBSTACLE_REPULSION_EPS = 1.0
PLOT_BOUNDS = ((-2.0, U_TARGET * TF + 2.0), (-3.5, 2.0))

PATH_COLORS = (
    "tab:blue",
    "tab:orange",
    "tab:green",
    "tab:red",
    "tab:purple",
    "tab:brown",
)


@dataclass(frozen=True)
class ModelCase:
    key: str
    title: str
    color: str


@dataclass
class SolveRun:
    case: ModelCase
    sys: object
    traj: Trajectory
    success: bool
    cost: float | None
    solve_s: float | None
    total_s: float


MODEL_CASES = (
    ModelCase("holonomic", "Holonomic", PATH_COLORS[0]),
    ModelCase("kinematic", "Kinematic", PATH_COLORS[1]),
    ModelCase("holonomic_dyn", "Holonomic dyn", PATH_COLORS[2]),
    ModelCase("kinematic_rate", "Kinematic rate", PATH_COLORS[3]),
    ModelCase("dynamic", "Dynamic", PATH_COLORS[4]),
    ModelCase("dynamic_rate", "Dynamic rate", PATH_COLORS[5]),
)


def _terminal_x() -> float:
    return U_TARGET * TF


def _build_holonomic() -> tuple[
    object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    sys = JaxHolonomicMobileRobot()
    sys.inputs["u"].lower_bound = np.array([0.0, -V_MAX])
    sys.inputs["u"].upper_bound = np.array([V_MAX, V_MAX])

    x_start = np.array([0.0, Y_START])
    x_ref = np.array([_terminal_x(), Y_GOAL])
    ubar = np.array([U_TARGET, 0.0])
    Q = np.diag([0.0, 10.0])
    R = np.diag([0.5, 0.5])
    S = np.diag([1.0, 10.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _build_holonomic_dyn() -> tuple[
    object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    sys = JaxDynamicHolonomicMobileRobot()
    sys.state.lower_bound[2] = 0.0
    sys.state.upper_bound[2] = V_MAX
    sys.state.lower_bound[3] = -V_MAX
    sys.state.upper_bound[3] = V_MAX
    sys.inputs["u"].lower_bound = np.array([-SPEED_DOT_MAX, -SPEED_DOT_MAX])
    sys.inputs["u"].upper_bound = np.array([SPEED_DOT_MAX, SPEED_DOT_MAX])

    x_start = np.array([0.0, Y_START, U_0, 0.0])
    x_ref = np.array([_terminal_x(), Y_GOAL, U_TARGET, 0.0])
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 0.1, 0.1])
    R = np.diag([1.0, 1.0])
    S = np.diag([1.0, 10.0, 10.0, 1.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _build_dynamic() -> tuple[
    object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    sys = JaxDynamicBicycle()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * U_TARGET / r_r
    sys.inputs["w_rear"].lower_bound[0] = 0.0
    sys.inputs["w_rear"].upper_bound[0] = w_rear_max
    sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
    sys.inputs["delta"].upper_bound[0] = DELTA_MAX

    x_start = np.array([0.0, Y_START, 0.0, U_0, 0.0, 0.0])
    x_ref = np.array([_terminal_x(), Y_GOAL, HEADING_TARGET, U_TARGET, 0.0, 0.0])
    ubar = np.array([U_TARGET / r_r, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1])
    R = np.diag([0.5, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1])
    return sys, x_start, x_ref, ubar, Q, R, S


def _build_kinematic() -> tuple[
    object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    sys = JaxKinematicBicycle()
    sys.inputs["u"].lower_bound = np.array([0.0, -DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([V_MAX, DELTA_MAX])

    x_start = np.array([0.0, Y_START, 0.0])
    x_ref = np.array([_terminal_x(), Y_GOAL, HEADING_TARGET])
    ubar = np.array([U_TARGET, 0.0])
    Q = np.diag([0.0, 10.0, 1.0])
    R = np.diag([0.5, 10.0])
    S = np.diag([1.0, 10.0, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _build_kinematic_rate() -> tuple[
    object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    sys = JaxKinematicBicycleRateInputs()
    sys.state.lower_bound[3] = 0.0
    sys.state.upper_bound[3] = V_MAX
    sys.state.lower_bound[4] = -DELTA_MAX
    sys.state.upper_bound[4] = DELTA_MAX
    sys.inputs["speed_dot"].lower_bound[0] = -SPEED_DOT_MAX
    sys.inputs["speed_dot"].upper_bound[0] = SPEED_DOT_MAX
    sys.inputs["steering_dot"].lower_bound[0] = -STEERING_DOT_MAX
    sys.inputs["steering_dot"].upper_bound[0] = STEERING_DOT_MAX

    x_start = np.array([0.0, Y_START, 0.0, U_0, 0.0])
    x_ref = np.array([_terminal_x(), Y_GOAL, HEADING_TARGET, U_TARGET, 0.0])
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 100.0])
    R = np.diag([1.0, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _build_dynamic_rate() -> tuple[
    object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray
]:
    sys = JaxDynamicBicycleRateInputs()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * U_TARGET / r_r
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = w_rear_max
    sys.state.lower_bound[7] = -DELTA_MAX
    sys.state.upper_bound[7] = DELTA_MAX
    sys.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
    sys.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
    sys.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
    sys.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX

    x_start = np.array([0.0, Y_START, 0.0, U_0, 0.0, 0.0, U_0 / r_r, 0.0])
    x_ref = np.array(
        [
            _terminal_x(),
            Y_GOAL,
            HEADING_TARGET,
            U_TARGET,
            0.0,
            0.0,
            U_TARGET / r_r,
            0.0,
        ]
    )
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1, 0.1, 100.0])
    R = np.diag([1.0, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.1, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


BUILDERS = {
    "holonomic": _build_holonomic,
    "holonomic_dyn": _build_holonomic_dyn,
    "dynamic": _build_dynamic,
    "kinematic": _build_kinematic,
    "kinematic_rate": _build_kinematic_rate,
    "dynamic_rate": _build_dynamic_rate,
}


def _speed_profile(case: ModelCase, traj: Trajectory) -> np.ndarray:
    if case.key == "holonomic":
        return np.hypot(traj.u[0, :], traj.u[1, :])
    if case.key == "holonomic_dyn":
        return np.hypot(traj.x[2, :], traj.x[3, :])
    if case.key == "kinematic":
        return traj.u[0, :]
    if case.key == "kinematic_rate":
        return traj.x[3, :]
    return np.hypot(traj.x[3, :], traj.x[4, :])


def _solve_case(case: ModelCase, scene: Scene) -> SolveRun:
    sys, x_start, x_ref, ubar, Q, R, S = BUILDERS[case.key]()
    tracking_cost = QuadraticCost.from_system(sys, Q=Q, R=R, S=S, xbar=x_ref, ubar=ubar)
    obstacle_cost = scene.clearance_field(bind(sys, point_probe())).as_cost(
        weight=OBSTACLE_REPULSION_WEIGHT,
        shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
    )
    problem = PlanningProblem(
        sys=sys, tf=TF, x_start=x_start, cost=tracking_cost + obstacle_cost
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=N_STEPS,
        transcription="direct_collocation",
        compile_backend="jax",
        record_solve_time=True,
        optimizer_options={"maxiter": 500, "ftol": 1e-1},
    )

    t0 = time.perf_counter()
    traj = planner.solve().trajectory
    total_s = time.perf_counter() - t0
    result = planner.last_optimization_result
    return SolveRun(
        case=case,
        sys=sys,
        traj=traj,
        success=bool(result.success),
        cost=None if result.cost is None else float(result.cost),
        solve_s=None if result.solve_time_s is None else float(result.solve_time_s),
        total_s=total_s,
    )


def _print_summary(runs: list[SolveRun]) -> None:
    print("\nTrajOpt obstacle scene — model comparison")
    print(f"  tf={TF}s, n_steps={N_STEPS}, u_target={U_TARGET} m/s")
    print(
        f"  obstacle={OBSTACLE_CENTER}, keepout R={OBSTACLE_RADIUS + OBSTACLE_MARGIN}"
    )
    print(
        f"{'model':<18} {'success':>8} {'solve_s':>9} {'total_s':>9} {'J*':>12} {'x_f':>8}"
    )
    for run in runs:
        solve_s = "—" if run.solve_s is None else f"{run.solve_s:.3f}"
        cost = "—" if run.cost is None else f"{run.cost:.1f}"
        print(
            f"{run.case.title:<18} {str(run.success):>8} {solve_s:>9} "
            f"{run.total_s:>9.3f} {cost:>12} {run.traj.x[0, -1]:>8.1f}"
        )


def _plot_paths(runs: list[SolveRun], scene: Scene) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(14.0, 9.0), sharex=True, sharey=True)
    for ax, run in zip(axes.ravel(), runs, strict=True):
        scene.plot(show=False, ax=ax, bounds=PLOT_BOUNDS, show_density=False, title="")
        ax.plot(
            run.traj.x[0, :],
            run.traj.x[1, :],
            color=run.case.color,
            linewidth=1.8,
            label="planned",
        )
        solve_s = "—" if run.solve_s is None else f"{run.solve_s:.2f}s"
        ax.set_title(f"{run.case.title}  (solve {solve_s})")
        ax.legend(loc="upper left", fontsize=8)
    fig.suptitle("Planned paths (scene obstacle overlay)")
    fig.tight_layout()
    plt.show()


def _plot_overlay_and_speed(runs: list[SolveRun], scene: Scene) -> None:
    fig, (ax_path, ax_speed) = plt.subplots(1, 2, figsize=(12.0, 5.0))

    scene.plot(show=False, ax=ax_path, bounds=PLOT_BOUNDS, show_density=False, title="")
    for run in runs:
        ax_path.plot(
            run.traj.x[0, :],
            run.traj.x[1, :],
            color=run.case.color,
            linewidth=1.8,
            label=run.case.title,
        )
    ax_path.set_title("All models — XY path")
    ax_path.legend(loc="upper left")

    for run in runs:
        ax_speed.plot(
            run.traj.t,
            _speed_profile(run.case, run.traj),
            color=run.case.color,
            linewidth=1.8,
            label=run.case.title,
        )
    ax_speed.set_xlabel("t [s]")
    ax_speed.set_ylabel("speed [m/s]")
    ax_speed.set_title("Longitudinal speed")
    ax_speed.grid(True, alpha=0.25)
    ax_speed.legend(loc="best", fontsize=8)

    fig.tight_layout()
    plt.show()


def main() -> None:
    configure_jax(enable_x64=True)
    keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
    scene = Scene(obstacles=[Sphere(OBSTACLE_CENTER, keepout_radius)])
    overlay = scene.as_visualizer(color="tab:red", opacity=0.45)

    runs = [_solve_case(case, scene) for case in MODEL_CASES]
    _print_summary(runs)
    _plot_paths(runs, scene)
    _plot_overlay_and_speed(runs, scene)

    for run in runs:
        print(f"Animating {run.case.title}...")
        run.sys.traj = run.traj
        run.sys.animate(run.traj, overlays=[overlay])


if __name__ == "__main__":
    main()
