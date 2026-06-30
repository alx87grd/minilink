"""Forward car parking: kinodynamic RRT vs Dubins-steering RRT, side by side.

Run from repo root::

    python examples/scripts/planning/rrt/demo_car_parking.py

The same parking problem is solved twice, changing only the two swappable parts
of the planner -- the ``TrajectoryExtender`` and the nearest-neighbour
``metric``:

* **kinodynamic** -- forward-integrates the car under a fixed set of steering
  motion primitives and selects nodes by a weighted state distance. It explores
  with short rollouts and only ever reaches the goal *region*.
* **dubins-steering** -- connects two poses along the exact shortest forward
  Dubins curve (min turning radius ``wheelbase / tan(max_steering)``) and selects
  nodes by Dubins path length. Each connect is dynamically feasible for the same
  ``KinematicCar`` and can reach the goal pose *exactly*.

Both run forward-only (Dubins cannot reverse), on the ``KinematicCar`` whose drawn
skin is a ``length x width`` box centred on the state -- matched here by an
oriented box ``car`` footprint, so the heading genuinely decides whether the car
fits the slot. The exact connector typically needs far fewer nodes, lands the
goal pose exactly, and returns a shorter, smoother path.
"""

import time

import numpy as np

from minilink.core.geometry import Box
from minilink.core.sets import BallSet, BoxSet
from minilink.dynamics.catalog.vehicles.steering import KinematicCar
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.metric import weighted
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.steering import DubinsSteering
from minilink.planning.spatial.collision import bind, car_outline
from minilink.planning.spatial.scene import Scene

# --- car + workspace -------------------------------------------------------
sys = KinematicCar()  # state [x, y, theta], input [speed, steering], car skin
wheelbase = sys.params["length"]
sys.state.lower_bound = np.array([-6.0, -6.0, -np.pi])
sys.state.upper_bound = np.array([26.0, 16.0, np.pi])
# forward only -- Dubins has no reverse gear
sys.inputs["u"].lower_bound = np.array([0.0, -0.6])
sys.inputs["u"].upper_bound = np.array([4.0, 0.6])

# two parked cars and a back wall leave a slot at x in [9, 13], y in [6, 10]
scene = Scene(
    obstacles=(
        Box([3.0, 6.0], [9.0, 10.0]),
        Box([13.0, 6.0], [19.0, 10.0]),
        Box([-6.0, 10.5], [26.0, 12.0]),
    )
)
body = bind(sys, car_outline(length=5.0, width=2.0))
X = BoxSet.from_system_state(sys) & scene.clearance_field(body).as_constraint()

x_start = np.array([0.0, 0.0, 0.0])  # on the lot, facing east
x_goal = np.array([11.0, 8.0, np.pi / 2])  # parked nose-in, facing north
problem = PlanningProblem(
    sys=sys, x_start=x_start, x_goal=x_goal, X=X, Xf=BallSet(x_goal, 1.0)
)

# --- the two swappable extenders -------------------------------------------
primitives = [np.array([4.0, s]) for s in (-0.6, -0.3, 0.0, 0.3, 0.6)]
dubins = DubinsSteering(wheelbase=wheelbase, max_steering=0.6, speed=4.0)

runs = {
    "kinodynamic": dict(
        extender=KinodynamicExtender(controls=primitives, horizon=1.0, n_substeps=4),
        metric=weighted([1.0, 1.0, 3.0]),  # weighted state distance
    ),
    "dubins-steering": dict(
        extender=SteeringExtender(dubins, max_distance=6.0, resolution=0.3),
        metric=dubins.distance,  # exact Dubins path length
    ),
}

# --- solve both ------------------------------------------------------------
print(
    f"turn radius = {dubins.radius:.1f} m  (wheelbase {wheelbase:.0f} m, max steer 0.6 rad)\n"
)
print(f"{'extender':<16}{'nodes':>7}{'goal err':>10}{'path (m)':>10}{'solve (s)':>11}")
solutions = {}
for name, cfg in runs.items():
    t0 = time.time()
    planner = RRTPlanner(
        problem, options=RRTOptions(seed=2, goal_bias=0.2, max_nodes=20000), **cfg
    )
    traj = planner.compute_solution()
    elapsed = time.time() - t0

    goal_err = float(np.linalg.norm(traj.x[:, -1] - x_goal))
    path_len = float(np.sum(np.linalg.norm(np.diff(traj.x[:2], axis=1), axis=0)))
    print(
        f"{name:<16}{len(planner.tree.nodes):>7}{goal_err:>10.2f}{path_len:>10.1f}{elapsed:>11.1f}"
    )
    solutions[name] = (planner, traj)

# --- visualize both trees + parked-car footprints --------------------------
try:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(13, 6), sharex=True, sharey=True)
    for ax, (name, (planner, traj)) in zip(axes, solutions.items()):
        step = max(1, traj.x.shape[1] // 12)
        poses = [traj.x[:, i] for i in range(0, traj.x.shape[1], step)]
        scene.plot(
            bounds=((-6, 22), (-6, 13)), robot=body, states=poses, ax=ax, show=False
        )
        planner.plot_tree(ax=ax, show=False)
        ax.set_title(f"{name}: {len(planner.tree.nodes)} nodes")
    fig.tight_layout()
    fig.savefig("/tmp/demo_rrt_car_parking.png", dpi=130)
    print("\nsaved /tmp/demo_rrt_car_parking.png")
except ImportError:
    pass

# Test drive: signals + animation of the exact Dubins maneuver.
planner, _ = solutions["dubins-steering"]
planner.plot_solution(signals=("x", "u"))
planner.animate_solution()
