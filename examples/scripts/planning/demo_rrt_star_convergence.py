"""RRT* live convergence — watch the tree rewire toward a shorter path.

Run from repo root::

    python examples/scripts/planning/demo_rrt_star_convergence.py

A holonomic robot steers through a moderate obstacle field. RRT* runs with
``optimize_after_goal=True``: after the first goal connection the search keeps
extending and rewiring until the best goal path cost has not improved for
``convergence_patience`` extensions (or ``max_nodes``).

Set ``LIVE_PLOT = True`` to redraw the tree during the solve (pyro-style live
display). With ``LIVE_PLOT_AFTER_GOAL_ONLY = True`` updates begin once the
first goal is reached — the convergence / path-shortening phase.
"""

import time

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import SteeringExtender
from minilink.planning.search.rrt_star import RRTStarOptions, RRTStarPlanner
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.robot import sphere
from minilink.planning.spatial.scene import Scene

SEED = 2
GOAL_TOLERANCE = 0.35
MAX_NODES = 12000
CONVERGENCE_PATIENCE = 600
COST_TOL = 0.02
LIVE_PLOT = True
LIVE_PLOT_AFTER_GOAL_ONLY = True
LIVE_PLOT_EVERY = 15
LIVE_PLOT_PAUSE = 0.001
REPLAY_HISTORY = False
HISTORY_STRIDE = 30

sys = HolonomicMobileRobot()  # dx = u
sys.state.lower_bound = np.array([-6.0, -6.0])
sys.state.upper_bound = np.array([6.0, 6.0])
sys.inputs["u"].lower_bound = np.array([-1.0, -1.0])
sys.inputs["u"].upper_bound = np.array([1.0, 1.0])

scene = Scene(
    obstacles=(
        Sphere([0.0, 0.0], 1.1),
        Sphere([2.5, 1.0], 0.85),
        Sphere([-2.0, -1.5], 0.8),
        Sphere([1.0, -2.5], 0.75),
        Sphere([-3.0, 2.0], 0.7),
        Sphere([3.5, -2.0], 0.7),
        Sphere([-1.0, 3.0], 0.65),
        Sphere([4.0, 2.5], 0.6),
    )
)
robot = sphere(radius=0.22, position=(0, 1))
X = BoxSet.from_system_state(sys) & scene.clearance_field(robot).as_constraint()

x_start = np.array([-4.5, -4.5])
x_goal = np.array([4.5, 4.5])
problem = PlanningProblem(sys=sys, x_start=x_start, x_goal=x_goal, X=X)

extender = SteeringExtender(
    StraightLineSteering(speed=1.0), max_distance=0.55, resolution=0.05
)

live_ax = None
if LIVE_PLOT:
    import matplotlib.pyplot as plt

    fig, live_ax = plt.subplots(figsize=(7.5, 6.5), frameon=True)
    scene.plot(
        bounds=((-6, 6), (-6, 6)),
        show_clearance_contour=True,
        show=False,
        ax=live_ax,
    )
    fig.suptitle(
        "Live RRT* convergence — blue path shortens as the tree rewires",
        fontsize=11,
    )

options = RRTStarOptions(
    seed=SEED,
    goal_bias=0.08,
    goal_tolerance=GOAL_TOLERANCE,
    max_nodes=MAX_NODES,
    optimize_after_goal=True,
    cost_tol=COST_TOL,
    convergence_patience=CONVERGENCE_PATIENCE,
    record_history=REPLAY_HISTORY,
    history_stride=HISTORY_STRIDE,
    live_plot=LIVE_PLOT,
    live_plot_ax=live_ax,
    live_plot_every=LIVE_PLOT_EVERY,
    live_plot_pause=LIVE_PLOT_PAUSE,
    live_plot_after_goal_only=LIVE_PLOT_AFTER_GOAL_ONLY,
)

planner = RRTStarPlanner(problem, extender=extender, options=options)

print("RRT* post-goal convergence demo")
print(
    f"  max_nodes={MAX_NODES}  patience={CONVERGENCE_PATIENCE}  "
    f"cost_tol={COST_TOL}  live_plot={LIVE_PLOT}"
)
if LIVE_PLOT:
    print(
        f"  live_plot_after_goal_only={LIVE_PLOT_AFTER_GOAL_ONLY}  "
        f"every={LIVE_PLOT_EVERY}  pause={LIVE_PLOT_PAUSE}"
    )

t0 = time.perf_counter()
traj = planner.compute_solution()
elapsed = time.perf_counter() - t0

goal_error = float(np.linalg.norm(traj.x[:, -1] - x_goal))
stop_reason = (
    "converged"
    if planner.converged
    else ("max_nodes" if planner.iterations >= MAX_NODES else "first_goal")
)

print(f"  reached_goal={planner.reached_goal}  converged={planner.converged}")
print(f"  extensions={planner.iterations}  tree nodes={len(planner.tree.nodes)}")
print(f"  best path cost={planner.best_goal_cost:.3f}  goal error={goal_error:.3f} m")
print(f"  elapsed={elapsed:.2f} s  stop reason: {stop_reason}")

if REPLAY_HISTORY and planner.history:
    first_goal = next(
        (frame for frame in planner.history if frame.reached_goal), None
    )
    if first_goal is not None:
        print(
            f"  first goal at extension {first_goal.iteration}, "
            f"cost={first_goal.best_cost:.3f}"
        )
        final_cost = planner.history[-1].best_cost
        print(
            f"  final cost={final_cost:.3f} "
            f"(Δ={first_goal.best_cost - final_cost:.3f})"
        )

if LIVE_PLOT:
    import matplotlib.pyplot as plt

    plt.tight_layout()
    plt.show()
elif REPLAY_HISTORY:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7.5, 6.5), frameon=True)
    scene.plot(
        bounds=((-6, 6), (-6, 6)),
        show_clearance_contour=True,
        show=False,
        ax=ax,
    )
    planner.animate_convergence(
        ax=ax,
        interval=60,
        show=False,
        title_prefix="RRT* shortest-path convergence",
    )
    fig.suptitle(
        "Recorded replay — blue path shortens as the tree rewires",
        fontsize=11,
    )
    plt.tight_layout()
    plt.show()
