"""RRT* among obstacles: the tree rewires and the path shortens, drawn while it solves."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import BallSet, HolonomicMobileRobot, PlanningProblem, RRTStarPlanner
from minilink.core.geometry import Sphere
from minilink.planning import LiveSearchPlot
from minilink.planning.search.extenders import SteeringExtender
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.collision import disc, mount
from minilink.planning.spatial.scene import Scene

########################################################
### Tunables
########################################################

MAX_ITERATIONS = 12000

########################################################
### Robot and scene
########################################################

robot = HolonomicMobileRobot()
robot.state.lower = np.array([-6.0, -6.0])
robot.state.upper = np.array([6.0, 6.0])
robot.inputs["u"].lower = np.array([-1.0, -1.0])
robot.inputs["u"].upper = np.array([1.0, 1.0])

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
body = mount(disc(0.22), on=robot)
free_space = robot.state.box & scene.clearance_field(body).as_set()

########################################################
### The task
########################################################

x_goal = np.array([4.5, 4.5])

problem = PlanningProblem(
    robot,
    x0=np.array([-4.5, -4.5]),
    Xf=BallSet(x_goal, 0.35),
    X=free_space,
    U=robot.input_box,
)

########################################################
### RRT* planner
########################################################

extender = SteeringExtender(
    StraightLineSteering(speed=1.0), max_distance=0.55, resolution=0.05
)
planner = RRTStarPlanner(
    problem,
    extender=extender,
    seed=2,
    goal_bias=0.08,
    max_iterations=MAX_ITERATIONS,
    optimize_after_goal=True,
    cost_tol=0.02,
    convergence_patience=600,
)

########################################################
### Solve with live plot
########################################################

fig, ax = plt.subplots(figsize=(7.5, 6.5))
scene.plot(bounds=((-6, 6), (-6, 6)), show_clearance_contour=True, show=False, ax=ax)

solution = planner.solve(
    callback=LiveSearchPlot(every=15, pause=0.001, after_goal_only=True, ax=ax)
)
print(solution)
plt.show()
