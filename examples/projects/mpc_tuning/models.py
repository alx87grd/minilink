"""Plant builders for MPC lap tuning (config-driven)."""

from __future__ import annotations

from typing import Any

import numpy as np

from examples.projects.mpc_tuning.track import CORRIDOR_HALF_WIDTH
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.car_profile import apply_car_profile
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleAcc,
    BicycleDynEngine,
    BicycleDynRate,
    BicycleKin,
)
from minilink.planning.spatial.collision import bind, car_outline
from minilink.planning.spatial.shaping import quadratic_excess, quadratic_hinge
from minilink.planning.spatial.track import ReferenceTrack

MODEL_KEYS = (
    "kinematic",
    "kinematic_rate",
    "dynamic_rate",
    "dynamic_engine",
)

CAR_LENGTH = 2.4
CAR_WIDTH = 0.2
CAR_MARGIN = 0.05
DELTA_MAX = 0.55
A_X_MAX = 4.0
DELTA_DOT_MAX = 2.0


def default_config(model_key: str) -> dict[str, Any]:
    """Seed hyperparameters for ``model_key``."""
    base = {
        "model": model_key,
        "MPC_DT": 0.2,
        "MPC_HORIZON": 2.0,
        "MPC_STEPS": 10,
        "optimizer_method": "scipy_slsqp",
        "maxiter": 100,
        "ftol": 0.1,
        "U_TARGET": 10.0,
        "PATH_COST_WEIGHT": 20.0,
        "CORRIDOR_COST_WEIGHT": 25.0,
        "warm_start": True,
        "compile_backend": "jax",
        "transcription": "direct_collocation",
        "plant_dt_inner": 0.01,
        "TF_SIM_MAX": 90.0,
    }
    if model_key == "kinematic":
        base.update(
            {
                "R_v": 0.5,
                "R_delta": 22.0,
            }
        )
    elif model_key == "kinematic_rate":
        base.update(
            {
                "Q_v": 0.15,
                "Q_delta": 80.0,
                "R_ax": 1.0,
                "R_delta_dot": 22.0,
            }
        )
    elif model_key == "dynamic_rate":
        base.update(
            {
                "Q_vx": 0.15,
                "Q_vy": 4.0,
                "Q_r": 6.0,
                "Q_w": 0.1,
                "Q_delta": 80.0,
                "R_wdot": 1.0,
                "R_ddot": 22.0,
            }
        )
    elif model_key == "dynamic_engine":
        base.update(
            {
                "U_TARGET": 10.0,
                "Q_vx": 5.0,
                "Q_vy": 4.0,
                "Q_r": 6.0,
                "Q_w": 0.1,
                "Q_delta": 80.0,
                "R_P": 2.0e-7,
                "R_delta": 22.0,
                "P_ubar_frac": 0.35,
                "PATH_COST_WEIGHT": 40.0,
                "CORRIDOR_COST_WEIGHT": 40.0,
                "MPC_STEPS": 10,
                "maxiter": 100,
            }
        )
    else:
        raise KeyError(f"unknown model {model_key!r}")
    return base


def _track_costs(sys, track: ReferenceTrack, path_w: float, corridor_w: float):
    body = bind(sys, car_outline(CAR_LENGTH, CAR_WIDTH, margin=CAR_MARGIN))
    path_cost = track.distance_field(body).as_cost(
        weight=float(path_w), shaping=quadratic_excess(threshold=0.1)
    )
    corridor_cost = track.corridor_field(body).as_cost(
        weight=float(corridor_w), shaping=quadratic_hinge(threshold=0.0)
    )
    return path_cost + corridor_cost


def _engine_cruise_power(params, vx: float) -> float:
    r_r = float(params["r_r"])
    w = vx / max(r_r, 1e-6)
    tau_brake = float(params["bw_engine"]) * w + float(params["tau_fric"])
    F_aero = 0.5 * float(params["rho"]) * float(params["CdA"]) * vx * abs(vx)
    return float(tau_brake * w + F_aero * vx)


def build_plant(
    model_key: str,
    track: ReferenceTrack,
    config: dict[str, Any],
    start_xy: np.ndarray,
    theta0: float,
):
    """Return ``(sys_mpc, sys_sim, x0, cost)`` from rest on the track."""
    U = float(config["U_TARGET"])
    path_w = float(config["PATH_COST_WEIGHT"])
    corridor_w = float(config["CORRIDOR_COST_WEIGHT"])
    # From rest (plan: VX0 = 0).
    v0 = 0.0

    if model_key == "kinematic":
        sys_mpc = BicycleKin()
        sys_mpc.inputs["u"].lower_bound = np.array([0.0, -DELTA_MAX])
        sys_mpc.inputs["u"].upper_bound = np.array([U * 1.25, DELTA_MAX])
        x0 = np.array([start_xy[0], start_xy[1], theta0])
        cost = QuadraticCost.from_system(
            sys_mpc,
            Q=np.diag([0.0, 0.0, 0.0]),
            R=np.diag([float(config["R_v"]), float(config["R_delta"])]),
            S=np.diag([0.0, 0.0, 0.0]),
            xbar=np.zeros(3),
            ubar=np.array([U, 0.0]),
        ) + _track_costs(sys_mpc, track, path_w, corridor_w)
        sys_sim = BicycleKin()
        sys_sim.params.update(dict(sys_mpc.params))
        return sys_mpc, sys_sim, x0, cost

    if model_key == "kinematic_rate":
        sys_mpc = BicycleAcc()
        sys_mpc.state.lower_bound[3] = 0.0
        sys_mpc.state.upper_bound[3] = U * 1.25
        sys_mpc.state.lower_bound[4] = -DELTA_MAX
        sys_mpc.state.upper_bound[4] = DELTA_MAX
        sys_mpc.inputs["u"].lower_bound = np.array([-A_X_MAX, -DELTA_DOT_MAX])
        sys_mpc.inputs["u"].upper_bound = np.array([A_X_MAX, DELTA_DOT_MAX])
        x_cruise = np.array([0.0, 0.0, 0.0, U, 0.0])
        x0 = np.array([start_xy[0], start_xy[1], theta0, v0, 0.0])
        cost = QuadraticCost.from_system(
            sys_mpc,
            Q=np.diag([0.0, 0.0, 0.0, float(config["Q_v"]), float(config["Q_delta"])]),
            R=np.diag([float(config["R_ax"]), float(config["R_delta_dot"])]),
            S=np.diag([0.0, 0.0, 0.0, float(config["Q_v"]), float(config["Q_delta"])]),
            xbar=x_cruise,
            ubar=np.zeros(2),
        ) + _track_costs(sys_mpc, track, path_w, corridor_w)
        sys_sim = BicycleAcc()
        sys_sim.params.update(dict(sys_mpc.params))
        return sys_mpc, sys_sim, x0, cost

    if model_key == "dynamic_rate":
        sys_mpc = BicycleDynRate()
        sys_mpc.state.lower_bound[6] = 0.0
        sys_mpc.state.upper_bound[6] = 90.0
        sys_mpc.state.lower_bound[7] = -DELTA_MAX
        sys_mpc.state.upper_bound[7] = DELTA_MAX
        sys_mpc.inputs["u"].lower_bound = np.array([-80.0, -2.0])
        sys_mpc.inputs["u"].upper_bound = np.array([80.0, 2.0])
        r_r = float(sys_mpc.params["r_r"])
        x_cruise = np.array([0.0, 0.0, 0.0, U, 0.0, 0.0, U / r_r, 0.0])
        x0 = np.array([start_xy[0], start_xy[1], theta0, v0, 0.0, 0.0, 0.0, 0.0])
        Q = np.diag(
            [
                0.0,
                0.0,
                0.0,
                float(config["Q_vx"]),
                float(config["Q_vy"]),
                float(config["Q_r"]),
                float(config["Q_w"]),
                float(config["Q_delta"]),
            ]
        )
        cost = QuadraticCost.from_system(
            sys_mpc,
            Q=Q,
            R=np.diag([float(config["R_wdot"]), float(config["R_ddot"])]),
            S=Q.copy(),
            xbar=x_cruise,
            ubar=np.zeros(2),
        ) + _track_costs(sys_mpc, track, path_w, corridor_w)
        sys_sim = BicycleDynRate()
        sys_sim.params.update(dict(sys_mpc.params))
        return sys_mpc, sys_sim, x0, cost

    if model_key == "dynamic_engine":
        sys_mpc = apply_car_profile(BicycleDynEngine(), "racecar")
        r_r = float(sys_mpc.params["r_r"])
        P_peak = float(sys_mpc.inputs["u"].upper_bound[0])
        P_cruise = _engine_cruise_power(sys_mpc.params, U)
        P_ubar = max(P_cruise, float(config["P_ubar_frac"]) * P_peak)
        P0 = P_ubar
        x_cruise = np.array([0.0, 0.0, 0.0, U, 0.0, 0.0, U / r_r, 0.0, P_cruise])
        x0 = np.array([start_xy[0], start_xy[1], theta0, v0, 0.0, 0.0, 0.0, 0.0, P0])
        Q = np.diag(
            [
                0.0,
                0.0,
                0.0,
                float(config["Q_vx"]),
                float(config["Q_vy"]),
                float(config["Q_r"]),
                float(config["Q_w"]),
                float(config["Q_delta"]),
                0.0,
            ]
        )
        S = Q.copy()
        S[3, 3] = max(float(config["Q_vx"]), 10.0)
        cost = QuadraticCost.from_system(
            sys_mpc,
            Q=Q,
            R=np.diag([float(config["R_P"]), float(config["R_delta"])]),
            S=S,
            xbar=x_cruise,
            ubar=np.array([P_ubar, 0.0]),
        ) + _track_costs(sys_mpc, track, path_w, corridor_w)
        sys_sim = apply_car_profile(BicycleDynEngine(), "racecar")
        return sys_mpc, sys_sim, x0, cost

    raise KeyError(f"unknown model {model_key!r}")


def corridor_half_width() -> float:
    return float(CORRIDOR_HALF_WIDTH)
