"""Hard obstacle plus soft traversability on the environment pipeline.

Run from repo root::

    python examples/scripts/planning/demo_environment_traversability.py

The scene has one hard sphere obstacle and a Gaussian traversability patch.
Obstacles and soft fields export separately; compose at PlanningProblem:

    X    = bounds & env.clearance_field(robot).as_constraint()
    cost = base + w * env.cost_field(robot).as_cost()
"""

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.planning.environment.environment import Environment
from minilink.planning.environment.features import GaussianField
from minilink.planning.environment.robot import disc

OBSTACLE_CENTER = (4.0, 0.0)
OBSTACLE_RADIUS = 0.5
PATCH_CENTER = (2.0, 1.5)
PATCH_AMPLITUDE = 2.0
PATCH_SIGMA = 1.0
ROBOT_RADIUS = 0.25

env = Environment(
    obstacles=(Sphere(OBSTACLE_CENTER, OBSTACLE_RADIUS),),
    fields=(GaussianField(PATCH_CENTER, PATCH_AMPLITUDE, PATCH_SIGMA),),
)
robot = disc(radius=ROBOT_RADIUS, position=(0, 1))

free = env.clearance_field(robot).as_constraint()
terrain = env.cost_field(robot).as_cost(weight=3.0)

# Sample a line through the workspace (headless sanity check).
xs = np.linspace(0.0, 6.0, 13)
print("Environment: hard obstacle + soft Gaussian patch")
print(f"  obstacle center={OBSTACLE_CENTER} R={OBSTACLE_RADIUS}")
print(f"  patch center={PATCH_CENTER} amp={PATCH_AMPLITUDE} sigma={PATCH_SIGMA}")
print(f"  robot disc R={ROBOT_RADIUS}")
print()
print("x        clearance[0]  density[0]  free?  terrain_cost.g")
print("-" * 58)

for x_val in xs:
    x = np.array([x_val, 0.0])
    clearance = float(free.field.value(x)[0])
    density = float(terrain.field.value(x)[0])
    feasible = free.contains(x)
    penalty = float(terrain.g(x, np.zeros(1)))
    print(
        f"{x_val:5.1f}   {clearance:11.3f}  {density:9.3f}  "
        f"{'yes' if feasible else ' no':>4}  {penalty:11.3f}"
    )

print()
print("PlanningProblem composition (separate exports):")
print("  X    = BoxSet.from_system_state(sys) & free")
print("  cost = tracking + terrain")

# Illustrate the merged constraint/cost types without a full dynamics solve.
bounds = BoxSet(lower=np.array([-1.0, -3.0]), upper=np.array([7.0, 3.0]))
X = bounds & free
x_test = np.array([2.0, 0.0])
print()
print(f"  bounds & free contains {x_test} -> {X.contains(x_test)}")
print(f"  terrain cost at patch peak -> {terrain.g(np.array(PATCH_CENTER), np.zeros(1)):.3f}")
