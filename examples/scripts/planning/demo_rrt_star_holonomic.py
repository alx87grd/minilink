"""Holonomic obstacle avoidance — compare RRT and RRT* on the same problem.

Run from repo root::

    python examples/scripts/planning/demo_rrt_star_holonomic.py

The scene and ``PlanningProblem`` are shared. Two planners differ in the
algorithm: baseline RRT stops at the first goal hit; RRT* keeps searching with
``optimize_after_goal=True`` until the best path cost stops improving (or
``max_nodes``). The ``SteeringExtender`` and ``StraightLineSteering`` extender
are identical.

Both use ``X = bounds & scene.clearance_field(robot).as_constraint()`` for
collision. The script repeats the search over several seeds, prints per-run and
average statistics, then plots the final seed side by side on the clearance
heatmap.

RRT* convergence here stops when the **path length** (sum of edge costs) stops
improving — not when the polyline becomes straight. Jaggedness comes from (1)
many short tree segments (``STEERING_MAX_DISTANCE``) and (2) collision detours;
more patience lowers cost but rarely removes every corner. Larger steering
steps yield fewer, straighter segments when the gaps still fit.
"""

import time

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import SteeringExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.rrt_star import RRTStarOptions, RRTStarPlanner
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.robot import sphere
from minilink.planning.spatial.scene import Scene

SEEDS = (0,)
GOAL_TOLERANCE = 0.1
MAX_NODES = 10000
CONVERGENCE_PATIENCE = 800
COST_TOL = 0.01
STEERING_MAX_DISTANCE = 1.0
STEERING_RESOLUTION = 0.05
STRAIGHT_LINE_DIST = float(np.linalg.norm(np.array([8.0, 8.0])))

sys = HolonomicMobileRobot()  # dx = u
sys.state.lower_bound = np.array([-6.0, -6.0])
sys.state.upper_bound = np.array([6.0, 6.0])
sys.inputs["u"].lower_bound = np.array([-1.0, -1.0])
sys.inputs["u"].upper_bound = np.array([1.0, 1.0])

scene = Scene(
    obstacles=(
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
)
robot = sphere(radius=0.25, position=(0, 1))
X = BoxSet.from_system_state(sys) & scene.clearance_field(robot).as_constraint()

x_start = np.array([-4.0, -4.0])
x_goal = np.array([4.0, 4.0])
problem = PlanningProblem(sys=sys, x_start=x_start, x_goal=x_goal, X=X)

extender = SteeringExtender(
    StraightLineSteering(speed=1.0),
    max_distance=STEERING_MAX_DISTANCE,
    resolution=STEERING_RESOLUTION,
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


def make_planner(label, seed):
    if label == "rrt":
        options = RRTOptions(
            seed=seed, goal_tolerance=GOAL_TOLERANCE, max_nodes=MAX_NODES
        )
        return RRTPlanner(problem, extender=extender, options=options)

    options = RRTStarOptions(
        seed=seed,
        goal_tolerance=GOAL_TOLERANCE,
        max_nodes=MAX_NODES,
        optimize_after_goal=True,
        cost_tol=COST_TOL,
        convergence_patience=CONVERGENCE_PATIENCE,
    )
    return RRTStarPlanner(problem, extender=extender, options=options)


labels = ("rrt", "rrt*")
runs = {label: [] for label in labels}
last_planners = {}

print(
    f"RRT vs RRT* ({len(SEEDS)} seeds, max_nodes={MAX_NODES}, "
    f"steer step={STEERING_MAX_DISTANCE} m, unobstructed dist≈{STRAIGHT_LINE_DIST:.1f} m)"
)
print(
    f"  RRT* patience={CONVERGENCE_PATIENCE}, cost_tol={COST_TOL}"
)
print(
    f"{'seed':>4s}  {'rrt ok':>6s}  {'rrt hops':>8s}  {'rrt cost':>8s}  "
    f"{'rrt err':>7s}  {'rrt t':>6s}  "
    f"{'star ok':>7s}  {'star cv':>7s}  {'star hops':>9s}  {'star cost':>9s}  "
    f"{'star err':>8s}  {'star t':>7s}"
)
print("-" * 118)

for seed in SEEDS:
    row = {}
    for label in labels:
        planner = make_planner(label, seed)
        t0 = time.perf_counter()
        traj = planner.compute_solution()
        elapsed = time.perf_counter() - t0
        goal_error = float(np.linalg.norm(traj.x[:, -1] - x_goal))
        row[label] = {
            "reached_goal": planner.reached_goal,
            "converged": getattr(planner, "converged", False),
            "iterations": getattr(planner, "iterations", len(planner.tree.nodes)),
            "hops": path_hops(planner),
            "nodes": len(planner.tree.nodes),
            "path_cost": path_cost(planner),
            "goal_error": goal_error,
            "elapsed_s": elapsed,
        }
        runs[label].append(row[label])
        last_planners[label] = planner

    print(
        f"{seed:4d}  "
        f"{str(row['rrt']['reached_goal']):>6s}  {row['rrt']['hops']:8d}  "
        f"{row['rrt']['path_cost']:8.2f}  {row['rrt']['goal_error']:7.2f}  "
        f"{row['rrt']['elapsed_s']:6.2f}  "
        f"{str(row['rrt*']['reached_goal']):>7s}  "
        f"{str(row['rrt*']['converged']):>7s}  {row['rrt*']['hops']:9d}  "
        f"{row['rrt*']['path_cost']:9.2f}  {row['rrt*']['goal_error']:8.2f}  "
        f"{row['rrt*']['elapsed_s']:7.2f}"
    )

print("-" * 118)
for label in labels:
    stats = runs[label]
    hops = [run["hops"] for run in stats]
    costs = [run["path_cost"] for run in stats]
    errors = [run["goal_error"] for run in stats]
    times = [run["elapsed_s"] for run in stats]
    successes = sum(run["reached_goal"] for run in stats)
    summary = (
        f"{label:5s} avg: {np.mean(hops):4.1f} path hops, "
        f"{np.mean(costs):.2f} path cost, "
        f"{np.mean(errors):.2f} m goal error, "
        f"{np.mean(times):.2f} s, "
        f"{successes}/{len(SEEDS)} reached goal"
    )
    if label == "rrt*":
        converged = sum(run["converged"] for run in stats)
        summary += f", {converged}/{len(SEEDS)} converged"
    print(summary)

import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.5), sharex=True, sharey=True)
plot_bounds = ((-6, 6), (-6, 6))
titles = {"rrt": "RRT", "rrt*": "RRT*"}
for ax, label in zip(axes, labels):
    planner = last_planners[label]
    scene.plot(
        bounds=plot_bounds,
        show_clearance_contour=True,
        show=False,
        ax=ax,
    )
    extra = ""
    if label == "rrt*" and planner.reached_goal:
        extra = f", converged={planner.converged}"
    planner.plot_tree(
        ax=ax,
        show=False,
        path_style="waypoints",
        title=(
            f"{titles[label]} seed={SEEDS[-1]} "
            f"({path_hops(planner)} hops, cost={path_cost(planner):.2f}{extra})"
        ),
    )

fig.suptitle(
    "RRT (first goal) vs RRT* (post-goal convergence) on the same holonomic scene"
)
plt.tight_layout()
plt.show()
