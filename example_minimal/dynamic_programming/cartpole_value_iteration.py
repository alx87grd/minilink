"""Cart-pole swing-up by value iteration on a 4-D grid."""

import jax.numpy as jnp
import numpy as np

from minilink import (
    CartPole,
    CostFunction,
    DynamicProgrammingPlanner,
    PlanningProblem,
    StateSpaceGrid,
)

########################################################
### Tunables
########################################################

N_STEPS = 1000

########################################################
### The plant
########################################################

plant = CartPole()
plant.inputs["u"].lower[0] = -10.0
plant.inputs["u"].upper[0] = 10.0
plant.state.lower = np.array([-5.0, -1.0 * np.pi, -5.0, -10.0])
plant.state.upper = np.array([5.0, +3.0 * np.pi, 5.0, 10.0])


class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        x_cart, theta, dx, dtheta = x
        return (
            (1.0 + jnp.cos(theta))
            + 0.1 * x_cart**2
            + 0.001 * (dx**2 + dtheta**2)
            + 0.001 * u[0] ** 2
        )

    def h(self, x, t=0.0, params=None):
        return 0.0


########################################################
### The task
########################################################

problem = PlanningProblem(plant, cost=SwingUpCost(), horizon=np.inf, U=plant.input_box)

########################################################
### Value iteration on a grid
########################################################

grid = StateSpaceGrid(
    problem, x_grid=(31, 51, 31, 51), u_grid=(5,), dt=0.02, precompute=False
)

# grid = StateSpaceGrid(
#     problem, x_grid=(51, 51, 51, 51), u_grid=(5,), dt=0.02, precompute=False
# )


planner = DynamicProgrammingPlanner(
    problem, grid=grid, backend="jax", gamma=1.0, out_of_bound_cost=500.0
)
solution = planner.solve_steps(N_STEPS)

planner.plot_cost_to_go(x_axis=1, y_axis=3)
planner.plot_control_law(x_axis=1, y_axis=3)

########################################################
### Closed-loop rollout
########################################################

plant.x0 = np.array([0.0, 0.05, 0.0, 0.0])

closed_loop = planner.controller @ plant
closed_loop.plot_diagram()

closed_loop.compute_trajectory(tf=10.0, dt=0.01)

closed_loop.plot_trajectory()
closed_loop.animate()
