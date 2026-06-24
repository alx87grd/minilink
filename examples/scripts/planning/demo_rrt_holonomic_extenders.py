"""Holonomic obstacle avoidance — compare RRT extenders on the same problem.

Run from repo root::

    python examples/scripts/planning/demo_rrt_holonomic_extenders.py

The scene and ``PlanningProblem`` are shared. Two planners differ only in the
injected extender:

- **Steering** — ``SteeringExtender(StraightLineSteering(...))`` reaches toward
  each sample along a straight segment (exact for ``dx = u``).
- **Kinodynamic** — ``KinodynamicExtender`` forward-integrates ``sys.f`` under
  discrete velocity primitives.

Both use ``X = bounds & scene.clearance_field(robot).as_constraint()`` for
collision. The script repeats the search over several seeds, prints per-run and
average statistics, then plots the final seed side by side on the clearance
heatmap.
"""

import time

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.robot import sphere
from minilink.planning.spatial.scene import Scene

SEEDS = (0, 1, 2, 3, 4)
GOAL_TOLERANCE = 0.4
MAX_NODES = 5000

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

primitives = [
    np.array([np.cos(a), np.sin(a)])
    for a in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
]


def make_planner(label, seed):
    options = RRTOptions(seed=seed, goal_tolerance=GOAL_TOLERANCE, max_nodes=MAX_NODES)
    if label == "steering":
        extender = SteeringExtender(
            StraightLineSteering(speed=1.0), max_distance=0.6, resolution=0.05
        )
    else:
        extender = KinodynamicExtender(controls=primitives, horizon=0.6, n_substeps=6)
    return RRTPlanner(problem, extender=extender, options=options)


labels = ("steering", "kinodynamic")
runs = {label: [] for label in labels}
last_planners = {}

print(f"RRT extender comparison ({len(SEEDS)} seeds, max_nodes={MAX_NODES})")
print(
    f"{'seed':>4s}  {'steering nodes':>14s}  {'steering err':>12s}  {'steering t':>10s}  "
    f"{'kinodynamic nodes':>17s}  {'kinodynamic err':>15s}  {'kinodynamic t':>13s}"
)
print("-" * 95)

for seed in SEEDS:
    row = {}
    for label in labels:
        planner = make_planner(label, seed)
        t0 = time.perf_counter()
        traj = planner.compute_solution()
        elapsed = time.perf_counter() - t0
        goal_error = float(np.linalg.norm(traj.x[:, -1] - x_goal))
        row[label] = {
            "nodes": len(planner.tree.nodes),
            "goal_error": goal_error,
            "path_knots": traj.x.shape[1],
            "elapsed_s": elapsed,
            "success": goal_error <= GOAL_TOLERANCE,
        }
        runs[label].append(row[label])
        last_planners[label] = planner

    print(
        f"{seed:4d}  "
        f"{row['steering']['nodes']:14d}  {row['steering']['goal_error']:12.2f}  "
        f"{row['steering']['elapsed_s']:10.2f}  "
        f"{row['kinodynamic']['nodes']:17d}  {row['kinodynamic']['goal_error']:15.2f}  "
        f"{row['kinodynamic']['elapsed_s']:13.2f}"
    )

print("-" * 95)
for label in labels:
    stats = runs[label]
    nodes = [run["nodes"] for run in stats]
    errors = [run["goal_error"] for run in stats]
    knots = [run["path_knots"] for run in stats]
    times = [run["elapsed_s"] for run in stats]
    successes = sum(run["success"] for run in stats)
    print(
        f"{label:12s} avg: {np.mean(nodes):5.1f} nodes, "
        f"{np.mean(errors):.2f} m goal error, "
        f"{np.mean(knots):.0f} path knots, "
        f"{np.mean(times):.2f} s, "
        f"{successes}/{len(SEEDS)} reached goal"
    )

import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.5), sharex=True, sharey=True)
plot_bounds = ((-6, 6), (-6, 6))
for ax, label in zip(axes, labels):
    planner = last_planners[label]
    scene.plot(
        bounds=plot_bounds,
        show_clearance_contour=True,
        show=False,
        ax=ax,
    )
    planner.plot_tree(
        ax=ax,
        show=False,
        title=f"{label} seed={SEEDS[-1]} ({len(planner.tree.nodes)} nodes)",
    )

fig.suptitle("RRT extenders on the same holonomic obstacle scene")
plt.tight_layout()
plt.show()
