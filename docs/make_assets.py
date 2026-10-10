"""Build every README figure: showcase GIFs, their 3-D pages, the diagram PNG, the bridges SVG."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
STATIC = ROOT / "docs" / "_static"
SHOWCASE = STATIC / "showcase"  # interactive 3-D pages, served by the docs site

import numpy as np  # noqa: E402
from asset_export import (  # noqa: E402
    bridges_svg,
    html_to_gif,
    shrink_gif,
    write_text_asset,
)

# The "one model, every tool" figure, in reading order: the top row left to
# right, the middle row's two ends, the bottom row left to right.
BRIDGES_TILES = [
    ("Simulation", "trajectories"),
    ("Analysis", "phase plane"),
    ("Classical control", "PID, Bode"),
    ("Nonlinear control", "impedance, SMC"),
    ("Linearization", "Jacobian"),
    ("Optimal control", "LQR, MPC"),
    ("Optimization", "collocation"),
    ("Animation", "2D and 3D"),
    ("Reinforcement\nlearning", ""),
    ("Planning", "RRT, DP"),
]
BRIDGES_CORE = ("System", "f, h, tf")


def save_gif(
    sys_or_diagram, traj, name: str, *, time_factor_video: float = 2.0, camera=None
) -> None:
    out = STATIC / name
    sys_or_diagram.animate(
        traj,
        renderer="matplotlib",
        html=False,
        show=False,
        save=True,
        file_name=str(out.with_suffix("")),
        time_factor_video=time_factor_video,
        camera=camera,
    )
    shrink_gif(out)


def oblique_camera(target, *, azimuth_deg, elevation_deg, distance):
    """A constant 3-D view: the eye ``distance`` from ``target``, at the given bearing."""
    azimuth, elevation = np.radians(azimuth_deg), np.radians(elevation_deg)
    camera = np.eye(4)
    camera[:3, 2] = [
        np.cos(elevation) * np.cos(azimuth),
        np.cos(elevation) * np.sin(azimuth),
        np.sin(elevation),
    ]
    camera[:3, 3] = target
    camera[3, 3] = distance
    return camera


def save_meshcat(sys_or_diagram, traj, page: str, gif: str, **animate_kwargs) -> None:
    """Write the interactive 3-D page, then the GIF screenshot from that page."""
    SHOWCASE.mkdir(exist_ok=True)
    html = SHOWCASE / page
    sys_or_diagram.animate(
        traj,
        renderer="meshcat",
        html=False,
        show=False,
        save=True,
        file_name=str(html),
        **animate_kwargs,
    )
    print(f"{html.name}: {html.stat().st_size / 1e6:.2f} MB")
    html_to_gif(html, STATIC / gif)


def svg_bridges() -> None:
    write_text_asset(STATIC / "bridges.svg", bridges_svg(BRIDGES_TILES, BRIDGES_CORE))


def diagram_png() -> None:
    from minilink import ImpedanceController, Pendulum
    from minilink.graphical.diagrams.dot import get_diagram

    diagram = ImpedanceController() @ Pendulum()
    get_diagram(diagram).render(
        filename=str(STATIC / "diagram_closed_loop"), format="png", cleanup=True
    )
    print("diagram_closed_loop.png")


def gif_pendulum() -> None:
    from minilink import ImpedanceController, Pendulum

    plant = Pendulum()  # catalog defaults: the camera frames the rod
    plant.x0[0] = 2.0
    diagram = ImpedanceController() @ plant
    traj = diagram.compute_trajectory(tf=8.0, verbose=False)
    save_gif(diagram, traj, "pendulum_impedance.gif")


def gif_cartpole() -> None:
    from minilink import (
        CartPole,
        PlanningProblem,
        QuadraticCost,
        TrajectoryOptimizationPlanner,
    )

    plant = CartPole()
    plant.inputs["u"].lower_bound[0] = -10.0
    plant.inputs["u"].upper_bound[0] = 10.0
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    problem = PlanningProblem(
        sys=plant,
        tf=4.0,
        x_start=np.array([-2.0, 1.0, 0.0, 0.0]),
        x_goal=x_goal,
        cost=QuadraticCost.from_system(
            plant, Q=np.diag([1.0, 1.0, 0.0, 0.0]), R=np.diag([0.01]), xbar=x_goal
        ),
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=40,
        transcription="direct_collocation",
        compile_backend="jax",
        optimizer_method="ipopt",
        verbose=False,
    )
    traj = planner.solve().trajectory.resample(n_samples=240)
    save_gif(
        plant, traj, "cartpole_swingup.gif", time_factor_video=1.0
    )  # default camera


def ur5_impedance() -> None:
    from minilink.blocks.sources import TrajectorySource
    from minilink.control.robotic import TaskImpedance
    from minilink.dynamics.catalog.manipulators import UR5Manipulator

    # The hand's target glides through four waypoints and back, holding at each.
    HOLD, GLIDE = 0.5, 1.5  # [s]
    waypoints = np.array(
        [
            [-0.5, 0.0, 0.5],
            [0.0, 0.5, 0.5],
            [-0.3, 0.3, 0.2],
            [-0.5, -0.3, 0.4],
            [-0.5, 0.0, 0.5],
        ]
    )
    t_knots, p_knots = [0.0], [waypoints[0]]
    for p_from, p_to in zip(waypoints[:-1], waypoints[1:]):
        t_knots += [t_knots[-1] + HOLD, t_knots[-1] + HOLD + GLIDE]
        p_knots += [p_from, p_to]
    TF = t_knots[-1] + HOLD

    arm = UR5Manipulator()
    q0 = arm.inverse_kinematics(
        waypoints[0], q_guess=np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
    )
    arm.x0 = arm.q2x(q0, np.zeros(6))

    ref = TrajectorySource(t_knots, np.array(p_knots).T)
    ctl = TaskImpedance(arm, gravity_comp=True, show_task_force=True)
    ctl.params["Kp"] = np.array([200.0, 200.0, 200.0])
    ctl.params["Kd"] = np.array([40.0, 40.0, 40.0])
    ctl.task_force_scale = 0.03  # [m/N]

    diagram = ref >> ctl @ arm
    traj = diagram.compute_trajectory(tf=TF, n_steps=int(30 * TF) + 1)
    camera = oblique_camera(
        [-0.25, 0.1, 0.35], azimuth_deg=25.0, elevation_deg=18.0, distance=1.1
    )
    save_meshcat(
        diagram,
        traj,
        "ur5_impedance.html",
        "ur5_meshcat.gif",
        time_factor_video=1.0,
        camera=camera,
    )


def racecar_mpc() -> None:
    from minilink import PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner
    from minilink.catalog import UdeSRacecarDyn
    from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
    from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
    from minilink.planning import (
        ReferenceTrack,
        Scene,
        Sphere,
        bind,
        car_outline,
        circuit_waypoints,
        from_waypoints,
        point_probe,
        quadratic_excess,
        quadratic_hinge,
    )

    # examples/demos/udes_racecar/mpc_racecar_dynamic.py: a lap past two cones,
    # the plan believing in a little more grip than the floor gives.
    LENGTH, WIDTH, RADIUS = 12.0, 8.0, 2.0  # [m]
    V_TARGET, P_CRUISE = 6.0, 15.0  # [m/s], [W]
    TF_SIM, SIM_DT = 5.0, 0.002  # [s]
    MPC_DT, MPC_HORIZON, MPC_STEPS = 0.1, 1.2, 12
    BODY_LENGTH, BODY_WIDTH, BODY_MARGIN = 0.34, 0.20, 0.02  # [m]
    CORRIDOR_HALF_WIDTH = 0.6  # [m]
    CONE_RADIUS, CONE_MARGIN, CONE_CLEARANCE = 0.10, 0.15, 0.30  # [m]
    CONE_X, CONE_LEAN = (2.0, -2.0), (-0.13, 0.13)  # [m]
    PLANNER_MU, PLANT_MU = 0.7, 0.6
    LATERAL_START, HEADING_NUDGE = 0.15, 1.0e-4  # [m], [rad]
    CAMERA_SCALE = 2.6  # [m] follow-camera distance

    path = circuit_waypoints(length=LENGTH, width=WIDTH, radius=RADIUS)
    track = ReferenceTrack(from_waypoints(path), half_width=CORRIDOR_HALF_WIDTH)
    cones = [(x, 0.5 * WIDTH + lean) for x, lean in zip(CONE_X, CONE_LEAN)]
    scene = Scene(obstacles=[Sphere(c, CONE_RADIUS + CONE_MARGIN) for c in cones])

    design = UdeSRacecarDyn(named_ports=False)
    design.params["mu"] = PLANNER_MU
    design.state.lower_bound[6] = 0.0
    design.state.upper_bound[6] = 3.0 * V_TARGET / design.params["r_r"]
    design.state.lower_bound[7] = -design.params["delta_max"]
    design.state.upper_bound[7] = design.params["delta_max"]
    design.state.lower_bound[8] = -design.params["P_max"]
    design.state.upper_bound[8] = design.params["P_max"]

    centre = bind(design, point_probe())
    body = bind(
        design, car_outline(length=BODY_LENGTH, width=BODY_WIDTH, margin=BODY_MARGIN)
    )
    x_cruise = np.zeros(design.n)
    x_cruise[3] = V_TARGET
    x_cruise[6] = V_TARGET / design.params["r_r"]
    x_cruise[8] = P_CRUISE
    cost = (
        QuadraticCost.from_system(
            design,
            Q=np.diag([0.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 2.0, 0.0]),
            R=np.diag([0.0005, 60.0]),
            S=np.diag([0.0, 0.0, 0.0, 8.0, 2.0, 0.0, 0.0, 2.0, 0.0]),
            xbar=x_cruise,
            ubar=np.array([P_CRUISE, 0.0]),
        )
        + track.distance_field(centre).as_cost(
            weight=60.0, shaping=quadratic_excess(threshold=0.05)
        )
        + track.corridor_field(body).as_cost(
            weight=200.0, shaping=quadratic_hinge(threshold=0.0)
        )
        + scene.clearance_field(body).as_cost(
            weight=400.0, shaping=quadratic_hinge(threshold=CONE_CLEARANCE)
        )
    )

    first = int(np.argmin(np.abs(path[:, 0]) + np.abs(path[:, 1] + 0.5 * WIDTH)))
    step = path[(first + 1) % path.shape[0]] - path[first]
    heading = np.arctan2(step[1], step[0]) + HEADING_NUDGE
    start = path[first] + LATERAL_START * np.array([-np.sin(heading), np.cos(heading)])
    x0 = np.array(
        [
            start[0],
            start[1],
            heading,
            V_TARGET,
            0.0,
            0.0,
            V_TARGET / design.params["r_r"],
            0.0,
            P_CRUISE,
        ]
    )

    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=design, x_start=x0, cost=cost, tf=MPC_HORIZON, X=design.state.box
        ),
        n_steps=MPC_STEPS,
        transcription="direct_collocation",
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 60, "ftol": 1.0},
    )
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)

    car = UdeSRacecarDyn(named_ports=False)
    car.params["mu"] = PLANT_MU
    car.x0 = x0.copy()
    car.camera_scale = CAMERA_SCALE
    car.skin = racecar_skin_3d

    lap = mpc @ car
    result = lap.compute_trajectory(
        tf=TF_SIM, x0_plant=x0, plant_dt_inner=SIM_DT, compile_backend="jax"
    )
    # one sample per video frame: the trail's pages stay small
    traj = result.plant.resample(n_samples=int(30 * TF_SIM) + 1)
    overlays = mpc_animation_overlays(
        result, planner, scene=scene, track=track, traj=traj
    )
    save_meshcat(
        lap,
        traj,
        "racecar_mpc.html",
        "racecar_mpc.gif",
        time_factor_video=1.0,
        overlays=overlays,
    )


def gif_rocket() -> None:
    from minilink import (
        CostFunction,
        Gaussian,
        ReinforcementLearningPlanner,
        Rocket,
        StochasticPlanningProblem,
    )

    # examples/demos/rl/rocket_landing_rl.py: PPO learns to land from 20 m.
    TRAINING_TIMESTEPS = 4_000_000
    DT, TF = 0.05, 15.0  # [s]
    GIMBAL, THRUST_TO_WEIGHT = 0.05, 2.0  # [rad], [-]
    X_LANDED = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    FEATURE_SCALE = np.array([0.25, 0.25, 1.0, 0.25, 0.25, 1.0])
    TF_SHOWN = 12.0  # [s] the clip: the descent, the touchdown, a beat after

    plant = Rocket()
    plant.params["inertia"] = 1000.0
    weight = plant.params["mass"] * plant.params["gravity"]
    plant.inputs["u"].lower_bound = np.array([0.0, -GIMBAL])
    plant.inputs["u"].upper_bound = np.array([THRUST_TO_WEIGHT * weight, GIMBAL])
    plant.state.lower_bound = np.array([-100.0, 0.0, -1.0, -50.0, -50.0, -5.0])
    plant.state.upper_bound = np.array([100.0, 200.0, 1.0, 50.0, 50.0, 5.0])

    class LandingCost(CostFunction):
        Q = 0.1 * np.diag([1.0, 1.0, 10.0, 0.1, 0.1, 1.0])
        R = 0.1 * np.diag([1e-8, 1.0])

        def g(self, x, u, t=0.0, params=None):
            Q, R = self.Q, self.R
            dx = x - X_LANDED
            return dx @ Q @ dx + u @ R @ u

        def h(self, x, t=0.0, params=None):
            return 0.0

    problem = StochasticPlanningProblem(
        plant,
        cost=LandingCost(),
        tf=TF,
        x0_distribution=Gaussian(
            [0.0, 20.0, 0.0, 0.0, 0.0, 0.0], [10.0, 8.0, 0.3, 2.0, 2.0, 0.3]
        ),
        infeasible_cost=100.0,
        X=plant.state.box,
    )
    planner = ReinforcementLearningPlanner(
        problem,
        dt=DT,
        features=lambda x: (x - X_LANDED) * FEATURE_SCALE,
        hidden=(64, 64),
        algorithm="ppo",
        n_envs=64,
        n_steps=32,
        batch_size=256,
        learning_rate=1e-3,
        gamma=0.995,
        log_std_init=-1.0,
        seed=0,
    )
    planner.solve(timesteps=TRAINING_TIMESTEPS)

    plant.x0 = np.array([10.0, 30.0, 0.0, 0.0, 0.0, 0.0])
    cl_sys = planner.get_controller() @ plant
    traj = cl_sys.compute_trajectory(tf=TF_SHOWN, dt=0.01)
    save_gif(cl_sys, traj, "rocket_landing.gif", time_factor_video=2.0)


def gif_mpc_car() -> None:
    from minilink import (
        BicycleDynRate,
        PlanningProblem,
        QuadraticCost,
        TrajectoryOptimizationPlanner,
    )
    from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
    from minilink.graphical.animation.camera import follow_frame_camera

    u_target = 4.0
    plant = BicycleDynRate()
    r_r = plant.params["r_r"]
    x_ref = np.array([0.0, 0.0, 0.0, u_target, 0.0, 0.0, u_target / r_r, 0.0])
    x0 = np.array([0.0, 3.0, 0.0, 0.8 * u_target, 0.0, 0.0, 0.8 * u_target / r_r, 0.0])
    plant.x0 = x0.copy()
    mpc_planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=plant,
            tf=2.0,
            x_start=x0,
            cost=QuadraticCost.from_system(
                plant,
                Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
                R=np.diag([1.0, 25.0]),
                S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
                xbar=x_ref,
            ),
        ),
        n_steps=5,
        transcription="direct_collocation",
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 10, "ftol": 1.0},
    )
    mpc = ModelPredictiveController(
        mpc_planner, dt_mpc=0.2, warm_start=True, verbose=False
    )
    hybrid = mpc @ plant
    result = hybrid.compute_trajectory(
        tf=8.0, x0_plant=x0, plant_dt_inner=0.02, compile_backend="jax"
    )
    out = STATIC / "mpc_car.gif"
    hybrid.animate(
        renderer="matplotlib",
        html=False,
        show=False,
        save=True,
        file_name=str(out.with_suffix("")),
        time_factor_video=2.0,
        overlays=mpc_animation_overlays(result, mpc_planner, reference_pad=20.0),
        camera=follow_frame_camera("plant:body", scale=7.0),
    )
    shrink_gif(out)


# name: (builder, the files it writes under docs/_static)
ASSETS = {
    "bridges": (svg_bridges, ["bridges.svg"]),
    "diagram": (diagram_png, ["diagram_closed_loop.png"]),
    "cartpole": (gif_cartpole, ["cartpole_swingup.gif"]),
    "ur5": (ur5_impedance, ["ur5_meshcat.gif", "showcase/ur5_impedance.html"]),
    "racecar": (racecar_mpc, ["racecar_mpc.gif", "showcase/racecar_mpc.html"]),
    "rocket": (gif_rocket, ["rocket_landing.gif"]),
    "pendulum": (gif_pendulum, ["pendulum_impedance.gif"]),
    "mpc": (gif_mpc_car, ["mpc_car.gif"]),
}


def list_assets() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for name, (_, outputs) in ASSETS.items():
        role = "README" if any(out in readme for out in outputs) else "extra"
        print(f"{name:10s} {role:7s} {', '.join(outputs)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "names", nargs="*", help=f"assets to build, default all: {', '.join(ASSETS)}"
    )
    parser.add_argument("--list", action="store_true", help="list the assets")
    args = parser.parse_args()
    unknown = [name for name in args.names if name not in ASSETS]
    if unknown:
        parser.error(f"unknown asset {', '.join(unknown)}; choose from {list(ASSETS)}")
    if args.list:
        list_assets()
    for name in [] if args.list else args.names or list(ASSETS):
        print(f"--- {name}")
        ASSETS[name][0]()
