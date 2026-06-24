"""Compare nearest backends on a 4D double-pendulum kinodynamic RRT.

Run from repo root::

    python examples/scripts/planning/demo_rrt_kdtree_speed_double_pendulum.py

The double pendulum swings up from hanging at rest toward the inverted equilibrium
``[pi, 0, 0, 0]`` under bounded joint torques. There are no obstacles — cost is
dominated by forward integration (``KinodynamicExtender``) and nearest-neighbour
queries in the 4D state space. At a few thousand nodes, ``kd_tree`` is typically
~1.5× faster than brute force here, whereas 2D holonomic demos often show less
because collision checking and post-goal tree scans dominate there.

See ``demo_rrt_kdtree_speed.py`` for the obstacle-avoidance case and notes on
where wall time goes when ``optimize_after_goal=True``.
"""

import time

import numpy as np

from minilink.core.sets import BallSet, BoxSet
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner

SEEDS = (0,)
MAX_NODES = 8000
GOAL_BIAS = 0.05
GOAL_TOLERANCE = 0.35
HORIZON = 0.25
N_SUBSTEPS = 5
TORQUE_LEVELS = (-3.0, 0.0, 3.0)

sys = DoublePendulum()
sys.state.lower_bound = np.array([-2.0 * np.pi, -2.0 * np.pi, -8.0, -8.0])
sys.state.upper_bound = np.array([2.0 * np.pi, 2.0 * np.pi, 8.0, 8.0])
sys.inputs["u"].lower_bound = np.array([-3.0, -3.0])
sys.inputs["u"].upper_bound = np.array([3.0, 3.0])

x_start = np.zeros(4)
x_goal = np.array([np.pi, 0.0, 0.0, 0.0])
problem = PlanningProblem(
    sys=sys,
    x_start=x_start,
    x_goal=x_goal,
    Xf=BallSet(x_goal, GOAL_TOLERANCE),
    X=BoxSet.from_system_state(sys),
)

torques = [
    np.array([t1, t2])
    for t1 in TORQUE_LEVELS
    for t2 in TORQUE_LEVELS
]
extender = KinodynamicExtender(
    controls=torques, horizon=HORIZON, n_substeps=N_SUBSTEPS
)


def run_planner(seed, nearest_backend):
    planner = RRTPlanner(
        problem,
        extender=extender,
        options=RRTOptions(
            seed=seed,
            goal_bias=GOAL_BIAS,
            max_nodes=MAX_NODES,
            nearest_backend=nearest_backend,
        ),
    )
    t0 = time.perf_counter()
    traj = planner.compute_solution()
    elapsed = time.perf_counter() - t0
    goal_error = float(np.linalg.norm(traj.x[:, -1] - x_goal))
    return {
        "elapsed_s": elapsed,
        "nodes": len(planner.tree.nodes),
        "goal_error": goal_error,
        "reached_goal": planner.reached_goal,
    }


def speedup(brute_s, kd_s):
    if kd_s <= 0.0:
        return float("inf")
    return brute_s / kd_s


print(
    f"Double-pendulum RRT nearest-backend comparison "
    f"({len(SEEDS)} seeds, n=4, max_nodes={MAX_NODES})"
)
print(
    f"{'seed':>4s}  {'nodes':>6s}  {'brute t':>9s}  {'kd t':>8s}  "
    f"{'speedup':>8s}  {'reached':>7s}  {'goal err':>9s}"
)
print("-" * 62)

speedups = []
for seed in SEEDS:
    brute = run_planner(seed, "brute_force")
    kd = run_planner(seed, "kd_tree")
    sp = speedup(brute["elapsed_s"], kd["elapsed_s"])
    speedups.append(sp)
    print(
        f"{seed:4d}  "
        f"{brute['nodes']:6d}  "
        f"{brute['elapsed_s']:9.2f}  {kd['elapsed_s']:8.2f}  "
        f"{sp:8.2f}x  "
        f"{str(brute['reached_goal']):>7s}  "
        f"{brute['goal_error']:9.3f}"
    )

print("-" * 62)
print(f"avg speedup: {np.mean(speedups):.2f}x")
