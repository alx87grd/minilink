"""RRT* live convergence — watch the tree rewire toward a shorter path."""

import time

import numpy as np

from minilink import (
    BallSet,
    HolonomicMobileRobot,
    PlanningProblem,
    RRTStarPlanner,
)
from minilink.core.geometry import Sphere
from minilink.planning import LiveSearchPlot
from minilink.planning.search.extenders import SteeringExtender
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.collision import disc, mount
from minilink.planning.spatial.scene import Scene

SEED = 2
GOAL_TOLERANCE = 0.35
MAX_NODES = 12000
CONVERGENCE_PATIENCE = 600
COST_TOL = 0.02
LIVE_PLOT = True  # redraw the tree during the solve
LIVE_PLOT_AFTER_GOAL_ONLY = True  # start once the first goal is reached
LIVE_PLOT_EVERY = 15
LIVE_PLOT_PAUSE = 0.001
REPLAY_HISTORY = False
HISTORY_STRIDE = 30

sys = HolonomicMobileRobot()  # dx = u
sys.state.lower = np.array([-6.0, -6.0])
sys.state.upper = np.array([6.0, 6.0])
sys.inputs["u"].lower = np.array([-1.0, -1.0])
sys.inputs["u"].upper = np.array([1.0, 1.0])

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
body = mount(disc(0.22), on=sys)
X = sys.state.box & scene.clearance_field(body).as_set()

x0 = np.array([-4.5, -4.5])
x_goal = np.array([4.5, 4.5])
problem = PlanningProblem(
    sys=sys, x0=x0, Xf=BallSet(x_goal, GOAL_TOLERANCE), X=X, U=sys.input_box
)

extender = SteeringExtender(
    StraightLineSteering(speed=1.0), max_distance=0.55, resolution=0.05
)

live_plot = None
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
    live_plot = LiveSearchPlot(
        every=LIVE_PLOT_EVERY,
        pause=LIVE_PLOT_PAUSE,
        after_goal_only=LIVE_PLOT_AFTER_GOAL_ONLY,
        ax=live_ax,
    )


planner = RRTStarPlanner(
    problem,
    extender=extender,
    seed=SEED,
    goal_bias=0.08,
    max_iterations=MAX_NODES,
    optimize_after_goal=True,
    cost_tol=COST_TOL,
    convergence_patience=CONVERGENCE_PATIENCE,
)

print("RRT* post-goal convergence demo")
print(
    f"  max_iterations={MAX_NODES}  patience={CONVERGENCE_PATIENCE}  "
    f"cost_tol={COST_TOL}  live_plot={LIVE_PLOT}"
)
if LIVE_PLOT:
    print(
        f"  after_goal_only={LIVE_PLOT_AFTER_GOAL_ONLY}  "
        f"every={LIVE_PLOT_EVERY}  pause={LIVE_PLOT_PAUSE}"
    )

t0 = time.perf_counter()
solution = planner.solve(
    record_history=REPLAY_HISTORY,
    history_stride=HISTORY_STRIDE,
    callback=live_plot,
)
elapsed = time.perf_counter() - t0
traj, record = solution.trajectory, solution.record

goal_error = float(np.linalg.norm(traj.x[:, -1] - x_goal))
stop_reason = (
    "converged"
    if planner.converged
    else ("max_iterations" if record.n_iterations >= MAX_NODES else "first_goal")
)

print(f"  reached_goal={record.reached_goal}  converged={planner.converged}")
print(f"  extensions={record.n_iterations}  tree nodes={len(planner.tree.nodes)}")
print(f"  best path cost={planner.best_path_cost:.3f}  goal error={goal_error:.3f} m")
print(f"  elapsed={elapsed:.2f} s  stop reason: {stop_reason}")

if REPLAY_HISTORY and planner.history:
    first_goal = next((frame for frame in planner.history if frame.reached_goal), None)
    if first_goal is not None:
        print(
            f"  first goal at extension {first_goal.iteration}, "
            f"cost={first_goal.best_cost_to_come:.3f}"
        )
        final_cost = planner.history[-1].best_cost_to_come
        print(
            f"  final cost={final_cost:.3f} (Δ={first_goal.best_cost_to_come - final_cost:.3f})"
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
