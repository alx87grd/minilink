"""Hard obstacle plus soft workspace field — point and disc robot samples.

Run from repo root::

    python examples/scripts/planning/demo_scene_workspace.py

One hard sphere and one Gaussian patch. Two robot bodies share the same scene:

- :func:`point` — clearance is the raw obstacle SDF at the body center.
- :func:`sphere` — clearance subtracts the disc radius (``obstacle SDF − R``).

For each sample pose we print field values plus cross-exports:

- ``free_set`` — ``clearance_field.as_constraint(lower=0)``
- ``density_threshold_set`` — ``cost_field.as_constraint(upper=...)``
- ``occupancy_cost`` — ``clearance_field.as_cost(shaping=occupancy(...))``
- ``barrier_cost`` — ``clearance_field.as_cost(shaping=inverse_barrier(...))``

Then ``scene.plot(...)`` for each case.
"""

import numpy as np

from minilink.core.geometry import Sphere
from minilink.planning.spatial.robot import point, sphere
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import inverse_barrier, occupancy
from minilink.planning.spatial.workspace_fields import GaussianField

OBSTACLE_CENTER = (4.0, 0.0)
OBSTACLE_RADIUS = 0.5
ROBOT_RADIUS = 0.25
PATCH_CENTER = (2.0, 1.5)
PATCH_AMPLITUDE = 2.0
PATCH_SIGMA = 1.0
DENSITY_THRESHOLD = 0.5
OCCUPANCY_SCALE = 0.5
BARRIER_EPS = 0.08
PLOT_BOUNDS = ((-1.0, 7.0), (-3.0, 3.0))
U = np.zeros(1)

scene = Scene(
    obstacles=(Sphere(OBSTACLE_CENTER, OBSTACLE_RADIUS),),
    workspace_fields=(GaussianField(PATCH_CENTER, PATCH_AMPLITUDE, PATCH_SIGMA),),
)

POINT_CASES = (
    ("open space", np.array([1.0, 0.0])),
    ("on patch peak", np.array(PATCH_CENTER)),
    ("clearance border", np.array([3.5, 0.0])),
    ("inside obstacle", np.array(OBSTACLE_CENTER)),
)

DISC_BORDER_X = OBSTACLE_CENTER[0] - (OBSTACLE_RADIUS + ROBOT_RADIUS)
DISC_CASES = (
    ("open space", np.array([1.0, 0.0])),
    ("on patch peak", np.array(PATCH_CENTER)),
    ("clearance border", np.array([DISC_BORDER_X, 0.0])),
    ("inside obstacle", np.array(OBSTACLE_CENTER)),
)


def run_cases(robot_label, robot, cases):
    clearance_field = scene.clearance_field(robot)
    density_field = scene.cost_field(robot)

    free_set = clearance_field.as_constraint()
    density_threshold_set = density_field.as_constraint(
        lower=None, upper=DENSITY_THRESHOLD
    )
    occupancy_cost = clearance_field.as_cost(shaping=occupancy(scale=OCCUPANCY_SCALE))
    barrier_cost = clearance_field.as_cost(shaping=inverse_barrier(epsilon=BARRIER_EPS))

    print(f"--- {robot_label} ---")
    print("  exports:")
    print("    free_set                = clearance_field.as_constraint(lower=0)")
    print(
        f"    density_threshold_set   = cost_field.as_constraint(upper={DENSITY_THRESHOLD})"
    )
    print(
        f"    occupancy_cost          = clearance_field.as_cost(shaping=occupancy({OCCUPANCY_SCALE}))"
    )
    print(
        f"    barrier_cost            = clearance_field.as_cost(shaping=inverse_barrier({BARRIER_EPS}))"
    )
    print()

    for label, x in cases:
        clearance = float(clearance_field.value(x))
        density = float(density_field.value(x))
        free_ok = free_set.contains(x)
        below_density_threshold = density_threshold_set.contains(x)
        occupancy_g = float(occupancy_cost.g(x, U))
        barrier_g = float(barrier_cost.g(x, U))

        print(f"  {label}  x=({x[0]:.1f}, {x[1]:.1f})")
        print(f"    clearance = {clearance:7.3f}   in free_set = {'yes' if free_ok else 'no'}")
        print(
            f"    density   = {density:7.3f}   in density_threshold_set = "
            f"{'yes' if below_density_threshold else 'no'}"
        )
        print(f"    occupancy cost g        = {occupancy_g:7.3f}")
        print(f"    barrier cost g          = {barrier_g:7.3f}")
        print()

        scene.plot(
            bounds=PLOT_BOUNDS,
            title=(
                f"{robot_label} / {label}: "
                f"clearance={clearance:.2f}, density={density:.2f}"
            ),
            show_clearance_contour=True,
            robot=robot,
            x=x,
        )


print("Scene: hard obstacle + soft Gaussian workspace field")
print(f"  obstacle  center={OBSTACLE_CENTER}  R={OBSTACLE_RADIUS}")
print(f"  patch     center={PATCH_CENTER}  amp={PATCH_AMPLITUDE}  sigma={PATCH_SIGMA}")
print()

run_cases("point robot", point(position=(0, 1)), POINT_CASES)
run_cases(f"disc robot (R={ROBOT_RADIUS})", sphere(ROBOT_RADIUS, position=(0, 1)), DISC_CASES)
