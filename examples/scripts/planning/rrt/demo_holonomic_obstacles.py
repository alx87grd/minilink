"""Holonomic obstacle avoidance — scene, RRT vs RRT*, and extenders.

Run from repo root::

    python examples/scripts/planning/rrt/demo_holonomic_obstacles.py

Intro: one hard obstacle plus a soft Gaussian workspace field — clearance and
cost fields for ``point_probe`` vs ``disc`` robot bodies (via :func:`bind`).

Section A (default): RRT stops at the first goal; RRT* keeps searching with
``optimize_after_goal=True`` until path cost stops improving.

Section B: steering vs kinodynamic extenders on the same 18-sphere scene.

Set ``RUN_SCENE_INTRO = False`` to skip the intro plot.
Set ``RUN_EXTENDER_COMPARISON = True`` to also run section B.
"""

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.rrt_star import RRTStarOptions, RRTStarPlanner
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.collision import bind, disc, point_probe
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.workspace_fields import GaussianField

RUN_SCENE_INTRO = True
RUN_EXTENDER_COMPARISON = False

SEED = 0
GOAL_TOLERANCE = 0.1
MAX_NODES = 10000
CONVERGENCE_PATIENCE = 800
COST_TOL = 0.01
STEERING_MAX_DISTANCE = 1.0
STEERING_RESOLUTION = 0.05
EXTENDER_GOAL_TOLERANCE = 0.4
EXTENDER_MAX_NODES = 5000

X_START = np.array([-4.0, -4.0])
X_GOAL = np.array([4.0, 4.0])
PLOT_BOUNDS = ((-6, 6), (-6, 6))

HOLONOMIC_OBSTACLES = (
    Sphere([0.0, 0.0], 1.0),
    Sphere([2.0, -1.5], 0.8),
    Sphere([-2.0, -1.0], 0.75),
    Sphere([-1.0, 1.5], 0.7),
    Sphere([1.0, 2.0], 0.65),
    Sphere([3.0, 0.5], 0.7),
    Sphere([0.5, -2.5], 0.75),
    Sphere([-3.0, 1.0], 0.6),
    Sphere([-0.5, -3.5], 0.65),
    Sphere([2.5, 2.5], 0.6),
    Sphere([-2.5, 2.0], 0.65),
    Sphere([1.5, -0.5], 0.55),
    Sphere([-1.5, -2.5], 0.7),
    Sphere([4.0, -1.0], 0.65),
    Sphere([-4.0, 0.0], 0.6),
    Sphere([0.0, 3.5], 0.55),
    Sphere([3.5, -2.5], 0.6),
    Sphere([-3.5, -2.0], 0.55),
)


def make_holonomic_problem(*, robot_radius=0.25):
    """18-sphere holonomic obstacle scene shared by sections A and B."""
    sys = HolonomicMobileRobot()
    sys.state.lower_bound = np.array([-6.0, -6.0])
    sys.state.upper_bound = np.array([6.0, 6.0])
    sys.inputs["u"].lower_bound = np.array([-1.0, -1.0])
    sys.inputs["u"].upper_bound = np.array([1.0, 1.0])

    scene = Scene(obstacles=HOLONOMIC_OBSTACLES)
    body = bind(sys, disc(robot_radius))
    X = BoxSet.from_system_state(sys) & scene.clearance_field(body).as_constraint()
    problem = PlanningProblem(sys=sys, x_start=X_START, x_goal=X_GOAL, X=X)
    return sys, scene, body, problem


def make_steering_extender(*, max_distance=1.0, resolution=0.05, speed=1.0):
    return SteeringExtender(
        StraightLineSteering(speed=speed),
        max_distance=max_distance,
        resolution=resolution,
    )


def make_kinodynamic_extender(*, horizon=0.6, n_substeps=6, n_primitives=8):
    primitives = [
        np.array([np.cos(a), np.sin(a)])
        for a in np.linspace(0.0, 2.0 * np.pi, n_primitives, endpoint=False)
    ]
    return KinodynamicExtender(
        controls=primitives, horizon=horizon, n_substeps=n_substeps
    )


def run_scene_intro():
    """Hard obstacle + soft workspace field — point vs disc clearance."""
    sys = HolonomicMobileRobot()
    scene = Scene(
        obstacles=(Sphere((4.0, 0.0), 0.5),),
        workspace_fields=(GaussianField((2.0, 1.5), 2.0, 1.0),),
    )
    robot_disc = bind(sys, disc(0.25))
    robot_point = bind(sys, point_probe())
    sample = np.array([3.5, 0.0])
    point_clearance = float(scene.clearance_field(robot_point).value(sample))
    disc_clearance = float(scene.clearance_field(robot_disc).value(sample))
    print("Scene intro: hard obstacle + Gaussian workspace patch")
    print(f"  sample x=({sample[0]:.1f}, {sample[1]:.1f})")
    print(f"  point robot clearance = {point_clearance:.3f}")
    print(f"  disc robot clearance  = {disc_clearance:.3f}  (subtracts R=0.25)")
    scene.plot(
        bounds=((-1.0, 7.0), (-3.0, 3.0)),
        title="Scene: clearance (point vs disc robot at sample pose)",
        show_clearance_contour=True,
        body=robot_disc,
        x=sample,
    )


def path_cost(planner) -> float:
    return float(planner.solution_node.cost)


def path_hops(planner) -> int:
    hops = 0
    node = planner.solution_node
    while node is not None and node.parent is not None:
        hops += 1
        node = node.parent
    return hops


def run_rrt_vs_rrt_star():
    """Compare baseline RRT with post-goal RRT* on the holonomic fixture."""
    import matplotlib.pyplot as plt

    _, scene, _, problem = make_holonomic_problem()
    extender = make_steering_extender(
        max_distance=STEERING_MAX_DISTANCE,
        resolution=STEERING_RESOLUTION,
    )

    rrt = RRTPlanner(
        problem,
        extender=extender,
        options=RRTOptions(
            seed=SEED, goal_tolerance=GOAL_TOLERANCE, max_nodes=MAX_NODES
        ),
    )
    rrt_traj = rrt.compute_solution()

    rrt_star = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(
            seed=SEED,
            goal_tolerance=GOAL_TOLERANCE,
            max_nodes=MAX_NODES,
            optimize_after_goal=True,
            cost_tol=COST_TOL,
            convergence_patience=CONVERGENCE_PATIENCE,
        ),
    )
    rrt_star_traj = rrt_star.compute_solution()

    print(
        f"RRT:  {len(rrt.tree.nodes)} nodes, "
        f"{path_hops(rrt)} hops, cost={path_cost(rrt):.2f}, "
        f"goal err={np.linalg.norm(rrt_traj.x[:, -1] - X_GOAL):.2f} m"
    )
    print(
        f"RRT*: {len(rrt_star.tree.nodes)} nodes, "
        f"{path_hops(rrt_star)} hops, cost={path_cost(rrt_star):.2f}, "
        f"converged={rrt_star.converged}, "
        f"goal err={np.linalg.norm(rrt_star_traj.x[:, -1] - X_GOAL):.2f} m"
    )

    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.5), sharex=True, sharey=True)
    planners = (("RRT", rrt), ("RRT*", rrt_star))
    for ax, (title, planner) in zip(axes, planners):
        scene.plot(
            bounds=PLOT_BOUNDS,
            show_clearance_contour=True,
            show=False,
            ax=ax,
        )
        extra = ""
        if title == "RRT*" and planner.reached_goal:
            extra = f", converged={planner.converged}"
        planner.plot_tree(
            ax=ax,
            show=False,
            path_style="waypoints",
            title=(
                f"{title} seed={SEED} "
                f"({path_hops(planner)} hops, cost={path_cost(planner):.2f}{extra})"
            ),
        )

    fig.suptitle("RRT (first goal) vs RRT* (post-goal convergence)")
    plt.tight_layout()
    plt.show()


def run_extender_comparison():
    """Steering vs kinodynamic extenders on the holonomic fixture."""
    import matplotlib.pyplot as plt

    _, scene, _, problem = make_holonomic_problem()
    extenders = {
        "steering": make_steering_extender(max_distance=0.6, resolution=0.05),
        "kinodynamic": make_kinodynamic_extender(horizon=0.6, n_substeps=6),
    }

    planners = {}
    for label, extender in extenders.items():
        planner = RRTPlanner(
            problem,
            extender=extender,
            options=RRTOptions(
                seed=SEED,
                goal_tolerance=EXTENDER_GOAL_TOLERANCE,
                max_nodes=EXTENDER_MAX_NODES,
            ),
        )
        traj = planner.compute_solution()
        goal_error = float(np.linalg.norm(traj.x[:, -1] - X_GOAL))
        planners[label] = planner
        print(
            f"{label:12s}: {len(planner.tree.nodes)} nodes, "
            f"goal err={goal_error:.2f} m, "
            f"success={goal_error <= EXTENDER_GOAL_TOLERANCE}"
        )

    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.5), sharex=True, sharey=True)
    for ax, label in zip(axes, extenders):
        planner = planners[label]
        scene.plot(
            bounds=PLOT_BOUNDS,
            show_clearance_contour=True,
            show=False,
            ax=ax,
        )
        planner.plot_tree(
            ax=ax,
            show=False,
            title=f"{label} seed={SEED} ({len(planner.tree.nodes)} nodes)",
        )

    fig.suptitle("RRT extenders on the same holonomic obstacle scene")
    plt.tight_layout()
    plt.show()


if RUN_SCENE_INTRO:
    run_scene_intro()

run_rrt_vs_rrt_star()

if RUN_EXTENDER_COMPARISON:
    run_extender_comparison()
