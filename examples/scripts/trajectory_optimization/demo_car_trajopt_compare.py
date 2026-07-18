"""Compare TrajOpt across the jax_vehicles fidelity ladder on two missions.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_car_trajopt_compare.py

Missions
--------
1. **Obstacle** — straight cruise with a soft sphere keepout (JAX collocation).
2. **Corner** — 90° path + corridor from the wide-circuit SE bend (no obstacles).
   Bicycle plants use JAX + ``car_outline``; holonomic plants use NumPy + ``disc``
   (path-distance fields under JAX are unreliable for the n=2/4 point plants).

Pedagogy twin: ``examples/notebooks/driving_model_trajopt_analysis.ipynb``.
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
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleAcc,
    BicycleDyn,
    BicycleDynEngine,
    BicycleDynRate,
    BicycleDynServo,
    BicycleDynTauRate,
    BicycleKin,
    Holonomic,
    HolonomicAccel,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, car_outline, disc, point_probe
from minilink.planning.spatial.overlays import TrackCorridorOverlay
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import (
    inverse_barrier,
    quadratic_excess,
    quadratic_hinge,
)
from minilink.planning.spatial.track import ReferenceTrack
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

# ---------------------------------------------------------------------------
# Shared ladder
# ---------------------------------------------------------------------------

PATH_COLORS = (
    "tab:blue",
    "tab:orange",
    "tab:green",
    "tab:red",
    "tab:purple",
    "tab:brown",
    "tab:pink",
    "tab:gray",
    "tab:olive",
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
    ModelCase("kinematic", "BicycleKin", PATH_COLORS[1]),
    ModelCase("holonomic_dyn", "HolonomicAccel", PATH_COLORS[2]),
    ModelCase("bicycle_acc", "BicycleAcc", PATH_COLORS[3]),
    ModelCase("dynamic", "BicycleDyn", PATH_COLORS[4]),
    ModelCase("dynamic_rate", "BicycleDynRate", PATH_COLORS[5]),
    ModelCase("dynamic_taurate", "BicycleDynTauRate", PATH_COLORS[6]),
    ModelCase("dynamic_servo", "BicycleDynServo", PATH_COLORS[7]),
    ModelCase("dynamic_engine", "BicycleDynEngine", PATH_COLORS[8]),
)

# ---------------------------------------------------------------------------
# Mission A — obstacle (straight)
# ---------------------------------------------------------------------------

OBS_TF = 4.0
OBS_N_STEPS = 20
OBS_U_0 = 8.0
OBS_U_TARGET = 10.0
OBS_Y_START = 0.0
OBS_Y_GOAL = 0.0
OBS_HEADING_TARGET = 0.0
OBS_DELTA_MAX = 0.25
OBS_V_MAX = 20.0
OBS_SPEED_DOT_MAX = 3.0
OBS_STEERING_DOT_MAX = 2.0
OBS_W_REAR_DOT_MAX = 80.0
OBS_DELTA_DOT_MAX = 2.0
OBS_TAU_REAR_MAX = 5000.0
OBS_P_CMD_MAX = 100000.0

OBSTACLE_CENTER = (12.0, 0.2)
OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.2
OBSTACLE_REPULSION_WEIGHT = 1000.0
OBSTACLE_REPULSION_EPS = 1.0
OBS_PLOT_BOUNDS = ((-2.0, OBS_U_TARGET * OBS_TF + 2.0), (-3.5, 2.0))


def _obs_terminal_x() -> float:
    return OBS_U_TARGET * OBS_TF


def _obs_build_holonomic():
    sys = Holonomic()
    sys.inputs["u"].lower_bound = np.array([0.0, -OBS_V_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_V_MAX, OBS_V_MAX])
    x_start = np.array([0.0, OBS_Y_START])
    x_ref = np.array([_obs_terminal_x(), OBS_Y_GOAL])
    ubar = np.array([OBS_U_TARGET, 0.0])
    Q = np.diag([0.0, 10.0])
    R = np.diag([0.5, 0.5])
    S = np.diag([1.0, 10.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_holonomic_dyn():
    sys = HolonomicAccel()
    sys.state.lower_bound[2] = 0.0
    sys.state.upper_bound[2] = OBS_V_MAX
    sys.state.lower_bound[3] = -OBS_V_MAX
    sys.state.upper_bound[3] = OBS_V_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_SPEED_DOT_MAX, -OBS_SPEED_DOT_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_SPEED_DOT_MAX, OBS_SPEED_DOT_MAX])
    x_start = np.array([0.0, OBS_Y_START, OBS_U_0, 0.0])
    x_ref = np.array([_obs_terminal_x(), OBS_Y_GOAL, OBS_U_TARGET, 0.0])
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 0.1, 0.1])
    R = np.diag([1.0, 1.0])
    S = np.diag([1.0, 10.0, 10.0, 1.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_kinematic():
    sys = BicycleKin()
    sys.inputs["u"].lower_bound = np.array([0.0, -OBS_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_V_MAX, OBS_DELTA_MAX])
    x_start = np.array([0.0, OBS_Y_START, 0.0])
    x_ref = np.array([_obs_terminal_x(), OBS_Y_GOAL, OBS_HEADING_TARGET])
    ubar = np.array([OBS_U_TARGET, 0.0])
    Q = np.diag([0.0, 10.0, 1.0])
    R = np.diag([0.5, 10.0])
    S = np.diag([1.0, 10.0, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_bicycle_acc():
    sys = BicycleAcc()
    sys.state.lower_bound[3] = 0.0
    sys.state.upper_bound[3] = OBS_V_MAX
    sys.state.lower_bound[4] = -OBS_DELTA_MAX
    sys.state.upper_bound[4] = OBS_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_SPEED_DOT_MAX, -OBS_STEERING_DOT_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_SPEED_DOT_MAX, OBS_STEERING_DOT_MAX])
    x_start = np.array([0.0, OBS_Y_START, 0.0, OBS_U_0, 0.0])
    x_ref = np.array(
        [_obs_terminal_x(), OBS_Y_GOAL, OBS_HEADING_TARGET, OBS_U_TARGET, 0.0]
    )
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 100.0])
    R = np.diag([1.0, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_dynamic():
    sys = BicycleDyn()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * OBS_U_TARGET / r_r
    sys.inputs["u"].lower_bound = np.array([0.0, -OBS_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([w_rear_max, OBS_DELTA_MAX])
    x_start = np.array([0.0, OBS_Y_START, 0.0, OBS_U_0, 0.0, 0.0])
    x_ref = np.array(
        [
            _obs_terminal_x(),
            OBS_Y_GOAL,
            OBS_HEADING_TARGET,
            OBS_U_TARGET,
            0.0,
            0.0,
        ]
    )
    ubar = np.array([OBS_U_TARGET / r_r, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1])
    R = np.diag([0.5, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_dynamic_rate():
    sys = BicycleDynRate()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * OBS_U_TARGET / r_r
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = w_rear_max
    sys.state.lower_bound[7] = -OBS_DELTA_MAX
    sys.state.upper_bound[7] = OBS_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_W_REAR_DOT_MAX, -OBS_DELTA_DOT_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_W_REAR_DOT_MAX, OBS_DELTA_DOT_MAX])
    x_start = np.array([0.0, OBS_Y_START, 0.0, OBS_U_0, 0.0, 0.0, OBS_U_0 / r_r, 0.0])
    x_ref = np.array(
        [
            _obs_terminal_x(),
            OBS_Y_GOAL,
            OBS_HEADING_TARGET,
            OBS_U_TARGET,
            0.0,
            0.0,
            OBS_U_TARGET / r_r,
            0.0,
        ]
    )
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1, 0.1, 100.0])
    R = np.diag([1.0, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.1, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_dynamic_taurate():
    sys = BicycleDynTauRate()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * OBS_U_TARGET / r_r
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = w_rear_max
    sys.state.lower_bound[7] = -OBS_DELTA_MAX
    sys.state.upper_bound[7] = OBS_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_TAU_REAR_MAX, -OBS_DELTA_DOT_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_TAU_REAR_MAX, OBS_DELTA_DOT_MAX])
    x_start = np.array([0.0, OBS_Y_START, 0.0, OBS_U_0, 0.0, 0.0, OBS_U_0 / r_r, 0.0])
    x_ref = np.array(
        [
            _obs_terminal_x(),
            OBS_Y_GOAL,
            OBS_HEADING_TARGET,
            OBS_U_TARGET,
            0.0,
            0.0,
            OBS_U_TARGET / r_r,
            0.0,
        ]
    )
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1, 0.1, 100.0])
    R = np.diag([1e-4, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.1, 100.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_dynamic_servo():
    sys = BicycleDynServo()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * OBS_U_TARGET / r_r
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = w_rear_max
    sys.state.lower_bound[7] = -OBS_DELTA_MAX
    sys.state.upper_bound[7] = OBS_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_TAU_REAR_MAX, -OBS_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_TAU_REAR_MAX, OBS_DELTA_MAX])
    x_start = np.array(
        [0.0, OBS_Y_START, 0.0, OBS_U_0, 0.0, 0.0, OBS_U_0 / r_r, 0.0, 0.0]
    )
    x_ref = np.array(
        [
            _obs_terminal_x(),
            OBS_Y_GOAL,
            OBS_HEADING_TARGET,
            OBS_U_TARGET,
            0.0,
            0.0,
            OBS_U_TARGET / r_r,
            0.0,
            0.0,
        ]
    )
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1, 0.1, 100.0, 0.0])
    R = np.diag([1e-4, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.1, 100.0, 0.0])
    return sys, x_start, x_ref, ubar, Q, R, S


def _obs_build_dynamic_engine():
    sys = BicycleDynEngine()
    r_r = sys.params["r_r"]
    w_rear_max = 1.2 * OBS_U_TARGET / r_r
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = w_rear_max
    sys.state.lower_bound[7] = -OBS_DELTA_MAX
    sys.state.upper_bound[7] = OBS_DELTA_MAX
    sys.state.lower_bound[8] = -OBS_P_CMD_MAX
    sys.state.upper_bound[8] = OBS_P_CMD_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_P_CMD_MAX, -OBS_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_P_CMD_MAX, OBS_DELTA_MAX])
    x_start = np.array(
        [0.0, OBS_Y_START, 0.0, OBS_U_0, 0.0, 0.0, OBS_U_0 / r_r, 0.0, 0.0]
    )
    x_ref = np.array(
        [
            _obs_terminal_x(),
            OBS_Y_GOAL,
            OBS_HEADING_TARGET,
            OBS_U_TARGET,
            0.0,
            0.0,
            OBS_U_TARGET / r_r,
            0.0,
            0.0,
        ]
    )
    ubar = np.array([0.0, 0.0])
    Q = np.diag([0.0, 10.0, 1.0, 0.1, 0.1, 0.1, 0.1, 100.0, 0.0])
    R = np.diag([1e-10, 10.0])
    S = np.diag([1.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.1, 100.0, 0.0])
    return sys, x_start, x_ref, ubar, Q, R, S


OBS_BUILDERS = {
    "holonomic": _obs_build_holonomic,
    "holonomic_dyn": _obs_build_holonomic_dyn,
    "kinematic": _obs_build_kinematic,
    "bicycle_acc": _obs_build_bicycle_acc,
    "dynamic": _obs_build_dynamic,
    "dynamic_rate": _obs_build_dynamic_rate,
    "dynamic_taurate": _obs_build_dynamic_taurate,
    "dynamic_servo": _obs_build_dynamic_servo,
    "dynamic_engine": _obs_build_dynamic_engine,
}

# ---------------------------------------------------------------------------
# Mission B — 90° corner (path + corridor, no obstacles)
# ---------------------------------------------------------------------------

# SE bend from the wide technical circuit (demo_mpc_circuit), with lead-in/out.
CORNER_XY = np.array(
    [
        [5.0, -10.0],
        [10.5, -10.0],
        [13.7101, -9.6985],
        [14.5, -9.3301],
        [15.2139, -8.8302],
        [15.8302, -8.2139],
        [16.3301, -7.5],
        [16.6985, -6.7101],
        [16.9240, -5.8682],
        [17.0, -5.0],
        [17.0, -2.0],
        [17.0, 2.0],
        [17.0, 5.0],
    ]
)
CORNER_HALF_WIDTH = 2.5
CORNER_S0 = 5.0
CORNER_TF = 3.0
CORNER_N_STEPS = 15
CORNER_U_0 = 6.0
CORNER_U_TARGET = 8.0
CORNER_DELTA_MAX = 0.55
CORNER_V_MAX = 20.0
CORNER_PATH_WEIGHT = 40.0
CORNER_CORRIDOR_WEIGHT = 25.0
CORNER_CAR_LENGTH = 2.4
CORNER_CAR_WIDTH = 0.2
CORNER_CAR_MARGIN = 0.05
CORNER_DISC_RADIUS = 0.3


def _corner_track() -> ReferenceTrack:
    return ReferenceTrack(from_waypoints(CORNER_XY), half_width=CORNER_HALF_WIDTH)


def _pose_on_track(track: ReferenceTrack, s: float) -> tuple[np.ndarray, float]:
    xy = np.asarray(track.path.sample(s), dtype=float)
    tangent = track.path.tangent(s)
    theta = float(np.arctan2(tangent[1], tangent[0]))
    if abs(np.cos(2.0 * theta)) > 1.0 - 1e-9:
        theta += 1e-4
    return xy, theta


def _corner_plot_bounds(track: ReferenceTrack):
    pad = CORNER_HALF_WIDTH + 1.0
    return (
        (float(CORNER_XY[:, 0].min()) - pad, float(CORNER_XY[:, 0].max()) + pad),
        (float(CORNER_XY[:, 1].min()) - pad, float(CORNER_XY[:, 1].max()) + pad),
    )


def _corner_build_holonomic(track: ReferenceTrack):
    xy0, _ = _pose_on_track(track, CORNER_S0)
    xyf, _ = _pose_on_track(track, track.path.total_length - 0.5)
    sys = Holonomic()
    sys.inputs["u"].lower_bound = np.array([-CORNER_V_MAX, -CORNER_V_MAX])
    sys.inputs["u"].upper_bound = np.array([CORNER_V_MAX, CORNER_V_MAX])
    x_start = np.array([xy0[0], xy0[1]])
    x_ref = np.array([xyf[0], xyf[1]])
    ubar = np.array([CORNER_U_TARGET, 0.0])
    Q = np.diag([0.0, 0.0])
    R = np.diag([0.5, 0.5])
    S = np.diag([30.0, 30.0])
    return sys, x_start, x_ref, ubar, Q, R, S, "numpy", disc(CORNER_DISC_RADIUS)


def _corner_build_holonomic_dyn(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    xyf, thf = _pose_on_track(track, track.path.total_length - 0.5)
    sys = HolonomicAccel()
    sys.state.lower_bound[2] = -CORNER_V_MAX
    sys.state.upper_bound[2] = CORNER_V_MAX
    sys.state.lower_bound[3] = -CORNER_V_MAX
    sys.state.upper_bound[3] = CORNER_V_MAX
    sys.inputs["u"].lower_bound = np.array([-3.0, -3.0])
    sys.inputs["u"].upper_bound = np.array([3.0, 3.0])
    x_start = np.array(
        [xy0[0], xy0[1], CORNER_U_0 * np.cos(th0), CORNER_U_0 * np.sin(th0)]
    )
    x_ref = np.array(
        [
            xyf[0],
            xyf[1],
            CORNER_U_TARGET * np.cos(thf),
            CORNER_U_TARGET * np.sin(thf),
        ]
    )
    ubar = np.zeros(2)
    Q = np.diag([0.0, 0.0, 0.15, 0.15])
    R = np.diag([1.0, 1.0])
    S = np.diag([30.0, 30.0, 1.0, 1.0])
    return sys, x_start, x_ref, ubar, Q, R, S, "numpy", disc(CORNER_DISC_RADIUS)


def _corner_build_kinematic(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleKin()
    sys.inputs["u"].lower_bound = np.array([0.0, -CORNER_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([CORNER_V_MAX, CORNER_DELTA_MAX])
    x_start = np.array([xy0[0], xy0[1], th0])
    x_ref = np.zeros(3)
    ubar = np.array([CORNER_U_TARGET, 0.0])
    Q = np.diag([0.0, 0.0, 0.0])
    R = np.diag([0.5, 10.0])
    S = np.diag([0.0, 0.0, 0.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


def _corner_build_bicycle_acc(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleAcc()
    sys.state.lower_bound[3] = 0.0
    sys.state.upper_bound[3] = CORNER_V_MAX
    sys.state.lower_bound[4] = -CORNER_DELTA_MAX
    sys.state.upper_bound[4] = CORNER_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-3.0, -2.5])
    sys.inputs["u"].upper_bound = np.array([3.0, 2.5])
    x_start = np.array([xy0[0], xy0[1], th0, CORNER_U_0, 0.0])
    x_ref = np.array([0.0, 0.0, 0.0, CORNER_U_TARGET, 0.0])
    ubar = np.zeros(2)
    Q = np.diag([0.0, 0.0, 0.0, 0.15, 40.0])
    R = np.diag([1.0, 10.0])
    S = np.diag([0.0, 0.0, 0.0, 0.15, 40.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


def _corner_build_dynamic(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleDyn()
    r_r = sys.params["r_r"]
    sys.inputs["u"].lower_bound = np.array([0.0, -CORNER_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([90.0, CORNER_DELTA_MAX])
    x_start = np.array([xy0[0], xy0[1], th0, CORNER_U_0, 0.0, 0.0])
    x_ref = np.array([0.0, 0.0, 0.0, CORNER_U_TARGET, 0.0, 0.0])
    ubar = np.array([CORNER_U_TARGET / r_r, 0.0])
    Q = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0])
    R = np.diag([0.5, 22.0])
    S = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


def _corner_build_dynamic_rate(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleDynRate()
    r_r = sys.params["r_r"]
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = 90.0
    sys.state.lower_bound[7] = -CORNER_DELTA_MAX
    sys.state.upper_bound[7] = CORNER_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-80.0, -2.0])
    sys.inputs["u"].upper_bound = np.array([80.0, 2.0])
    x_start = np.array(
        [xy0[0], xy0[1], th0, CORNER_U_0, 0.0, 0.0, CORNER_U_0 / r_r, 0.0]
    )
    x_ref = np.array(
        [0.0, 0.0, 0.0, CORNER_U_TARGET, 0.0, 0.0, CORNER_U_TARGET / r_r, 0.0]
    )
    ubar = np.zeros(2)
    Q = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0])
    R = np.diag([1.0, 22.0])
    S = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


def _corner_build_dynamic_taurate(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleDynTauRate()
    r_r = sys.params["r_r"]
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = 90.0
    sys.state.lower_bound[7] = -CORNER_DELTA_MAX
    sys.state.upper_bound[7] = CORNER_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-5000.0, -2.0])
    sys.inputs["u"].upper_bound = np.array([5000.0, 2.0])
    x_start = np.array(
        [xy0[0], xy0[1], th0, CORNER_U_0, 0.0, 0.0, CORNER_U_0 / r_r, 0.0]
    )
    x_ref = np.array(
        [0.0, 0.0, 0.0, CORNER_U_TARGET, 0.0, 0.0, CORNER_U_TARGET / r_r, 0.0]
    )
    ubar = np.zeros(2)
    Q = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0])
    R = np.diag([1e-4, 22.0])
    S = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


def _corner_build_dynamic_servo(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleDynServo()
    r_r = sys.params["r_r"]
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = 90.0
    sys.state.lower_bound[7] = -CORNER_DELTA_MAX
    sys.state.upper_bound[7] = CORNER_DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-5000.0, -CORNER_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([5000.0, CORNER_DELTA_MAX])
    x_start = np.array(
        [xy0[0], xy0[1], th0, CORNER_U_0, 0.0, 0.0, CORNER_U_0 / r_r, 0.0, 0.0]
    )
    x_ref = np.array(
        [0.0, 0.0, 0.0, CORNER_U_TARGET, 0.0, 0.0, CORNER_U_TARGET / r_r, 0.0, 0.0]
    )
    ubar = np.zeros(2)
    Q = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0, 0.0])
    R = np.diag([1e-4, 22.0])
    S = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0, 0.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


def _corner_build_dynamic_engine(track: ReferenceTrack):
    xy0, th0 = _pose_on_track(track, CORNER_S0)
    sys = BicycleDynEngine()
    r_r = sys.params["r_r"]
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = 90.0
    sys.state.lower_bound[7] = -CORNER_DELTA_MAX
    sys.state.upper_bound[7] = CORNER_DELTA_MAX
    sys.state.lower_bound[8] = -OBS_P_CMD_MAX
    sys.state.upper_bound[8] = OBS_P_CMD_MAX
    sys.inputs["u"].lower_bound = np.array([-OBS_P_CMD_MAX, -CORNER_DELTA_MAX])
    sys.inputs["u"].upper_bound = np.array([OBS_P_CMD_MAX, CORNER_DELTA_MAX])
    x_start = np.array(
        [xy0[0], xy0[1], th0, CORNER_U_0, 0.0, 0.0, CORNER_U_0 / r_r, 0.0, 0.0]
    )
    x_ref = np.array(
        [0.0, 0.0, 0.0, CORNER_U_TARGET, 0.0, 0.0, CORNER_U_TARGET / r_r, 0.0, 0.0]
    )
    ubar = np.zeros(2)
    Q = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0, 0.0])
    R = np.diag([1e-10, 22.0])
    S = np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0, 0.0])
    geom = car_outline(CORNER_CAR_LENGTH, CORNER_CAR_WIDTH, margin=CORNER_CAR_MARGIN)
    return sys, x_start, x_ref, ubar, Q, R, S, "jax", geom


CORNER_BUILDERS = {
    "holonomic": _corner_build_holonomic,
    "holonomic_dyn": _corner_build_holonomic_dyn,
    "kinematic": _corner_build_kinematic,
    "bicycle_acc": _corner_build_bicycle_acc,
    "dynamic": _corner_build_dynamic,
    "dynamic_rate": _corner_build_dynamic_rate,
    "dynamic_taurate": _corner_build_dynamic_taurate,
    "dynamic_servo": _corner_build_dynamic_servo,
    "dynamic_engine": _corner_build_dynamic_engine,
}


# ---------------------------------------------------------------------------
# Solve / plot helpers
# ---------------------------------------------------------------------------


def _speed_profile(case: ModelCase, traj: Trajectory) -> np.ndarray:
    if case.key == "holonomic":
        return np.hypot(traj.u[0, :], traj.u[1, :])
    if case.key == "holonomic_dyn":
        return np.hypot(traj.x[2, :], traj.x[3, :])
    if case.key == "kinematic":
        return traj.u[0, :]
    if case.key == "bicycle_acc":
        return traj.x[3, :]
    return np.hypot(traj.x[3, :], traj.x[4, :])


def _solve_obstacle_case(case: ModelCase, scene: Scene) -> SolveRun:
    sys, x_start, x_ref, ubar, Q, R, S = OBS_BUILDERS[case.key]()
    tracking_cost = QuadraticCost.from_system(sys, Q=Q, R=R, S=S, xbar=x_ref, ubar=ubar)
    obstacle_cost = scene.clearance_field(bind(sys, point_probe())).as_cost(
        weight=OBSTACLE_REPULSION_WEIGHT,
        shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
    )
    problem = PlanningProblem(
        sys=sys, tf=OBS_TF, x_start=x_start, cost=tracking_cost + obstacle_cost
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=OBS_N_STEPS,
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


def _solve_corner_case(case: ModelCase, track: ReferenceTrack) -> SolveRun:
    sys, x_start, x_ref, ubar, Q, R, S, backend, geom = CORNER_BUILDERS[case.key](track)
    body = bind(sys, geom)
    tracking_cost = QuadraticCost.from_system(sys, Q=Q, R=R, S=S, xbar=x_ref, ubar=ubar)
    path_cost = track.distance_field(body).as_cost(
        weight=CORNER_PATH_WEIGHT, shaping=quadratic_excess(threshold=0.1)
    )
    corridor_cost = track.corridor_field(body).as_cost(
        weight=CORNER_CORRIDOR_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
    )
    problem = PlanningProblem(
        sys=sys,
        tf=CORNER_TF,
        x_start=x_start,
        cost=tracking_cost + path_cost + corridor_cost,
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=CORNER_N_STEPS,
        transcription="direct_collocation",
        compile_backend=backend,
        record_solve_time=True,
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 150, "ftol": 0.1},
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


def _print_summary(title: str, runs: list[SolveRun], detail: str) -> None:
    print(f"\n{title}")
    print(f"  {detail}")
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


def _plot_obstacle_paths(runs: list[SolveRun], scene: Scene) -> None:
    fig, axes = plt.subplots(3, 3, figsize=(12.0, 10.0), sharex=True, sharey=True)
    flat = axes.ravel()
    for ax, run in zip(flat, runs):
        scene.plot(
            show=False, ax=ax, bounds=OBS_PLOT_BOUNDS, show_density=False, title=""
        )
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
    for ax in flat[len(runs) :]:
        ax.set_visible(False)
    fig.suptitle("Mission A — obstacle: planned paths")
    fig.tight_layout()
    plt.show()


def _plot_corner_paths(runs: list[SolveRun], track: ReferenceTrack) -> None:
    bounds = _corner_plot_bounds(track)
    fig, axes = plt.subplots(3, 3, figsize=(12.0, 10.0), sharex=True, sharey=True)
    flat = axes.ravel()
    for ax, run in zip(flat, runs):
        track.plot(show=False, ax=ax, bounds=bounds, title="")
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
        ax.set_aspect("equal", adjustable="box")
    for ax in flat[len(runs) :]:
        ax.set_visible(False)
    fig.suptitle("Mission B — 90° corner: path + corridor")
    fig.tight_layout()
    plt.show()


def _plot_overlay_and_speed(
    runs: list[SolveRun],
    *,
    title: str,
    draw_background,
) -> None:
    fig, (ax_path, ax_speed) = plt.subplots(1, 2, figsize=(12.0, 5.0))
    draw_background(ax_path)
    for run in runs:
        ax_path.plot(
            run.traj.x[0, :],
            run.traj.x[1, :],
            color=run.case.color,
            linewidth=1.8,
            label=run.case.title,
        )
    ax_path.set_title(f"{title} — XY path")
    ax_path.legend(loc="best", fontsize=8)
    ax_path.set_aspect("equal", adjustable="box")

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

    # --- Mission A: obstacle ---
    keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
    scene = Scene(obstacles=[Sphere(OBSTACLE_CENTER, keepout_radius)])
    obstacle_overlay = scene.as_visualizer(color="tab:red", opacity=0.45)

    obs_runs = [_solve_obstacle_case(case, scene) for case in MODEL_CASES]
    _print_summary(
        "TrajOpt obstacle scene — model comparison",
        obs_runs,
        (
            f"tf={OBS_TF}s, n_steps={OBS_N_STEPS}, u_target={OBS_U_TARGET} m/s, "
            f"obstacle={OBSTACLE_CENTER}, keepout R={keepout_radius}"
        ),
    )
    _plot_obstacle_paths(obs_runs, scene)
    _plot_overlay_and_speed(
        obs_runs,
        title="Obstacle",
        draw_background=lambda ax: scene.plot(
            show=False, ax=ax, bounds=OBS_PLOT_BOUNDS, show_density=False, title=""
        ),
    )

    # --- Mission B: 90° corner ---
    track = _corner_track()
    corner_runs = [_solve_corner_case(case, track) for case in MODEL_CASES]
    _print_summary(
        "TrajOpt 90° corner — path + corridor",
        corner_runs,
        (
            f"tf={CORNER_TF}s, n_steps={CORNER_N_STEPS}, u_target={CORNER_U_TARGET} m/s, "
            f"half_width={CORNER_HALF_WIDTH} m, circuit SE bend"
        ),
    )
    _plot_corner_paths(corner_runs, track)
    bounds = _corner_plot_bounds(track)
    _plot_overlay_and_speed(
        corner_runs,
        title="Corner",
        draw_background=lambda ax: track.plot(
            show=False, ax=ax, bounds=bounds, title=""
        ),
    )

    corridor_overlay = TrackCorridorOverlay(track)
    for run in obs_runs:
        print(f"Animating obstacle / {run.case.title}...")
        run.sys.traj = run.traj
        run.sys.animate(run.traj, overlays=[obstacle_overlay])
    for run in corner_runs:
        print(f"Animating corner / {run.case.title}...")
        run.sys.traj = run.traj
        run.sys.camera_scale = 18.0
        run.sys.animate(run.traj, overlays=[corridor_overlay])


if __name__ == "__main__":
    main()
