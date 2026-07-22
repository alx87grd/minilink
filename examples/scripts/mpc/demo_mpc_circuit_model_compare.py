"""Compare hybrid MPC models on a large rounded-rectangle circuit.

Path + corridor only (no obstacles, no chicane). Runs closed-loop MPC for
each plant, overlays XY paths, then animates each result:

- :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleKin`
- :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleAcc` (KinRate)
- :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleDynRate`
- :class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleDynEngine`

Run from repo root::

    python examples/scripts/mpc/demo_mpc_circuit_model_compare.py
"""

from __future__ import annotations

from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from minilink.control.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
)
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.car_profile import apply_car_profile
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleAcc,
    BicycleDynEngine,
    BicycleDynRate,
    BicycleKin,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.shaping import quadratic_excess, quadratic_hinge
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- knobs ---
U_TARGET = 10.0
VX0 = 5.0
TF_SIM = 10.0
MPC_DT = 0.2
SIM_DT = 0.005
MPC_HORIZON = 2.0
MPC_STEPS = 20.0
DELTA_MAX = 0.55
A_X_MAX = 4.0
DELTA_DOT_MAX = 2.0

CORRIDOR_HALF_WIDTH = 3.0
PATH_COST_WEIGHT = 20.0
CORRIDOR_COST_WEIGHT = 25.0
CAR_LENGTH = 2.4
CAR_WIDTH = 0.2
CAR_MARGIN = 0.05
ANIMATE_EACH = True
STEP_DISP = True

# Large rounded rectangle (no chicane): half-extents Lx×Ly, corner radius R.
CIRCUIT_LX = 30.0
CIRCUIT_LY = 18.0
CIRCUIT_R = 10.0


def _rounded_rectangle_path(Lx, Ly, R, *, nseg=4, narc=8):
    """CCW rounded-rectangle waypoints centered at the origin."""
    R = min(float(R), float(Lx), float(Ly))
    c_tr = (Lx - R, Ly - R)
    c_tl = (-(Lx - R), Ly - R)
    c_bl = (-(Lx - R), -(Ly - R))
    c_br = (Lx - R, -(Ly - R))

    def arc(cx, cy, a0_deg, a1_deg):
        thetas = np.linspace(np.deg2rad(a0_deg), np.deg2rad(a1_deg), narc)
        return [(cx + R * np.cos(t), cy + R * np.sin(t)) for t in thetas]

    pts = []
    # Start mid-bottom, go right → SE → east → NE → north → NW → west → SW → close.
    pts.extend([(x, -Ly) for x in np.linspace(0.0, Lx - R, nseg, endpoint=True)])
    pts.extend(arc(*c_br, 270, 360))
    pts.extend([(Lx, y) for y in np.linspace(-(Ly - R), Ly - R, nseg, endpoint=True)])
    pts.extend(arc(*c_tr, 0, 90))
    pts.extend([(x, Ly) for x in np.linspace(Lx - R, -(Lx - R), nseg, endpoint=True)])
    pts.extend(arc(*c_tl, 90, 180))
    pts.extend([(-Lx, y) for y in np.linspace(Ly - R, -(Ly - R), nseg, endpoint=True)])
    pts.extend(arc(*c_bl, 180, 270))
    pts.extend([(x, -Ly) for x in np.linspace(-(Lx - R), 0.0, nseg, endpoint=True)])
    return np.asarray(pts, dtype=float)


LOOP_XY = _rounded_rectangle_path(CIRCUIT_LX, CIRCUIT_LY, CIRCUIT_R)

COLORS = {
    "kinematic": "#1f77b4",
    "kinematic_rate": "#9467bd",
    "dynamic_rate": "#ff7f0e",
    "dynamic_engine": "#2ca02c",
}


@dataclass(frozen=True)
class ModelCase:
    key: str
    title: str


CASES = (
    ModelCase("kinematic", "BicycleKin"),
    ModelCase("kinematic_rate", "BicycleAcc (KinRate)"),
    ModelCase("dynamic_rate", "BicycleDynRate"),
    ModelCase("dynamic_engine", "BicycleDynEngine"),
)


def _start_pose(track: ReferenceTrack) -> tuple[np.ndarray, float]:
    start_xy = LOOP_XY[0].copy()
    s_start, _ = track.path.project(start_xy)
    tangent = track.path.tangent(s_start)
    theta0 = float(np.arctan2(tangent[1], tangent[0]))
    if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
        theta0 += 1e-4
    return start_xy, theta0


def _track_costs(sys, track: ReferenceTrack):
    body = bind(sys, car_outline(CAR_LENGTH, CAR_WIDTH, margin=CAR_MARGIN))
    path_cost = track.distance_field(body).as_cost(
        weight=PATH_COST_WEIGHT, shaping=quadratic_excess(threshold=0.1)
    )
    corridor_cost = track.corridor_field(body).as_cost(
        weight=CORRIDOR_COST_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
    )
    return path_cost + corridor_cost


def _build_kinematic(track: ReferenceTrack, start_xy, theta0):
    sys_mpc = BicycleKin()
    sys_mpc.inputs["u"].lower_bound = np.array([0.0, -DELTA_MAX])
    sys_mpc.inputs["u"].upper_bound = np.array([U_TARGET * 1.25, DELTA_MAX])
    x0 = np.array([start_xy[0], start_xy[1], theta0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0]),
        R=np.diag([0.5, 22.0]),
        S=np.diag([0.0, 0.0, 0.0]),
        xbar=np.zeros(3),
        ubar=np.array([U_TARGET, 0.0]),
    ) + _track_costs(sys_mpc, track)
    sys_sim = BicycleKin()
    sys_sim.params["length"] = 1.03 * sys_mpc.params["length"]
    return sys_mpc, sys_sim, x0, cost


def _build_kinematic_rate(track: ReferenceTrack, start_xy, theta0):
    """BicycleAcc: kinematic pose + ``u = [a_x, delta_dot]`` (KinRate)."""
    sys_mpc = BicycleAcc()
    sys_mpc.state.lower_bound[3] = 0.0
    sys_mpc.state.upper_bound[3] = U_TARGET * 1.25
    sys_mpc.state.lower_bound[4] = -DELTA_MAX
    sys_mpc.state.upper_bound[4] = DELTA_MAX
    sys_mpc.inputs["u"].lower_bound = np.array([-A_X_MAX, -DELTA_DOT_MAX])
    sys_mpc.inputs["u"].upper_bound = np.array([A_X_MAX, DELTA_DOT_MAX])
    x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0])
    x0 = np.array([start_xy[0], start_xy[1], theta0, VX0, 0.0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 0.15, 80.0]),
        R=np.diag([1.0, 22.0]),
        S=np.diag([0.0, 0.0, 0.0, 0.15, 80.0]),
        xbar=x_cruise,
        ubar=np.zeros(2),
    ) + _track_costs(sys_mpc, track)
    sys_sim = BicycleAcc()
    sys_sim.params["length"] = 1.03 * sys_mpc.params["length"]
    return sys_mpc, sys_sim, x0, cost


def _build_dynamic_rate(track: ReferenceTrack, start_xy, theta0):
    sys_mpc = BicycleDynRate()
    sys_mpc.state.lower_bound[6] = 0.0
    sys_mpc.state.upper_bound[6] = 90.0
    sys_mpc.state.lower_bound[7] = -DELTA_MAX
    sys_mpc.state.upper_bound[7] = DELTA_MAX
    sys_mpc.inputs["u"].lower_bound = np.array([-80.0, -2.0])
    sys_mpc.inputs["u"].upper_bound = np.array([80.0, 2.0])
    r_r = sys_mpc.params["r_r"]
    x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
    x0 = np.array([start_xy[0], start_xy[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        R=np.diag([1.0, 22.0]),
        S=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        xbar=x_cruise,
        ubar=np.zeros(2),
    ) + _track_costs(sys_mpc, track)
    sys_sim = BicycleDynRate()
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
    sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]
    return sys_mpc, sys_sim, x0, cost


def _engine_cruise_power(params, vx: float) -> float:
    """Steady shaft power to hold ``vx`` against engine brake + aero."""
    r_r = float(params["r_r"])
    w = vx / max(r_r, 1e-6)
    tau_brake = float(params["bw_engine"]) * w + float(params["tau_fric"])
    F_aero = 0.5 * float(params["rho"]) * float(params["CdA"]) * vx * abs(vx)
    return float(tau_brake * w + F_aero * vx)


def _build_dynamic_engine(track: ReferenceTrack, start_xy, theta0):
    # Default catalog engine is a heavy sedan (1500 kg + aero). Use the racecar
    # profile so power / mass / τ_sat match the LOS-scale plant and MPC scaling.
    sys_mpc = apply_car_profile(BicycleDynEngine(), "racecar")
    r_r = float(sys_mpc.params["r_r"])
    P_peak = float(sys_mpc.inputs["u"].upper_bound[0])
    P_cruise = _engine_cruise_power(sys_mpc.params, U_TARGET)
    # Bias ``ubar`` above cruise so SLSQP accelerates toward ``U_TARGET``
    # (pure cruise ubar kept plans near ~kW and barely sped up).
    P_ubar = max(P_cruise, 0.35 * P_peak)
    P0 = P_ubar

    x_cruise = np.array(
        [0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0, P_cruise]
    )
    x0 = np.array([start_xy[0], start_xy[1], theta0, VX0, 0.0, 0.0, VX0 / r_r, 0.0, P0])
    R_P = 2e-7
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 0.0, 0.0, 5.0, 4.0, 6.0, 0.1, 80.0, 0.0]),
        R=np.diag([R_P, 22.0]),
        S=np.diag([0.0, 0.0, 0.0, 10.0, 4.0, 6.0, 0.1, 80.0, 0.0]),
        xbar=x_cruise,
        ubar=np.array([P_ubar, 0.0]),
    ) + _track_costs(sys_mpc, track)
    sys_sim = apply_car_profile(BicycleDynEngine(), "racecar")
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
    sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]
    return sys_mpc, sys_sim, x0, cost


BUILDERS = {
    "kinematic": _build_kinematic,
    "kinematic_rate": _build_kinematic_rate,
    "dynamic_rate": _build_dynamic_rate,
    "dynamic_engine": _build_dynamic_engine,
}


@dataclass
class CaseResult:
    case: ModelCase
    planner: TrajectoryOptimizationPlanner
    result: object
    hybrid: object


def _run_case(case: ModelCase, track: ReferenceTrack, start_xy, theta0) -> CaseResult:
    sys_mpc, sys_sim, x0, cost = BUILDERS[case.key](track, start_xy, theta0)
    sys_sim.camera_scale = 28.0
    sys_sim.x0 = x0.copy()

    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=MPC_HORIZON),
        n_steps=MPC_STEPS,
        transcription="direct_collocation",
        compile_backend="jax",
        record_solve_time=True,
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 50, "ftol": 1.0},
    )
    mpc = ModelPredictiveController(
        planner, dt_mpc=MPC_DT, warm_start=True, step_disp=STEP_DISP
    )
    hybrid = mpc @ sys_sim
    print(f"\n=== simulating {case.title} ===")
    result = hybrid.compute_trajectory(
        tf=TF_SIM,
        x0_plant=x0,
        plant_dt_inner=SIM_DT,
        compile_backend="jax",
        verbose=False,
    )
    return CaseResult(case=case, planner=planner, result=result, hybrid=hybrid)


def _plot_path_overlay(track: ReferenceTrack, runs: list[CaseResult]) -> None:
    fig, ax = plt.subplots(figsize=(9.0, 7.0))
    track.plot(show=False, ax=ax, title="")
    for run in runs:
        traj = run.result.plant
        color = COLORS[run.case.key]
        ax.plot(
            traj.x[0, :],
            traj.x[1, :],
            color=color,
            linewidth=2.0,
            label=run.case.title,
        )
        ax.plot(traj.x[0, 0], traj.x[1, 0], "o", color=color, markersize=5)
        ax.plot(traj.x[0, -1], traj.x[1, -1], "s", color=color, markersize=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("MPC circuit model compare (rounded rectangle)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def _animate_runs(track: ReferenceTrack, runs: list[CaseResult]) -> None:
    for run in runs:
        print(f"\n=== animating {run.case.title} ===")
        run.hybrid.animate(
            overlays=mpc_animation_overlays(run.result, run.planner, track=track)
        )


def _print_summary(runs: list[CaseResult]) -> None:
    print("\nMPC circuit model compare")
    print(
        f"  TF_SIM={TF_SIM}, U_TARGET={U_TARGET}, rounded rect {2 * CIRCUIT_LX:.0f}×{2 * CIRCUIT_LY:.0f} m"
    )
    print(f"{'model':<24} {'n':>4} {'t_end':>8} {'x_f':>8} {'y_f':>8}")
    for run in runs:
        traj = run.result.plant
        print(
            f"{run.case.title:<24} {traj.x.shape[0]:>4} "
            f"{float(traj.t[-1]):>8.2f} "
            f"{float(traj.x[0, -1]):>8.2f} {float(traj.x[1, -1]):>8.2f}"
        )


track = ReferenceTrack(from_waypoints(LOOP_XY), half_width=CORRIDOR_HALF_WIDTH)
start_xy, theta0 = _start_pose(track)

if __name__ == "__main__":
    runs = [_run_case(case, track, start_xy, theta0) for case in CASES]
    _print_summary(runs)
    _plot_path_overlay(track, runs)
    if ANIMATE_EACH:
        _animate_runs(track, runs)
