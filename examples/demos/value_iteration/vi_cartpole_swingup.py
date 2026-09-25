"""Cart-pole swing-up by value iteration on a coarse 4-D grid."""

import jax.numpy as jnp
import numpy as np

from minilink import (
    CartPole,
    CostFunction,
    DynamicProgrammingPlanner,
    PlanningProblem,
    StateSpaceGrid,
)

DT = 0.02
INF = 500.0
N_STEPS = 1000
UPRIGHT = np.array([0.0, np.pi, 0.0, 0.0])

# Plant: catalog cart-pole, theta = 0 hanging, pi upright, force |F| <= 10 N.
# The state bounds are the grid box; leaving it is charged INF.
# Theta is a finite mesh (VI has no periodic features), denser on the pole.
plant = CartPole()
plant.inputs["u"].lower_bound = np.array([-10.0])
plant.inputs["u"].upper_bound = np.array([10.0])
plant.state.lower_bound = np.array([-5.0, -1.0 * np.pi, -5.0, -10.0])
plant.state.upper_bound = np.array([5.0, +3.0 * np.pi, 5.0, 10.0])


# Cost: periodic angle term, small cart, velocity and effort penalties (the
# velocity weight stays small so the fast swing is not punished)
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        x_cart, theta, dx, dtheta = x
        return (
            (1.0 + jnp.cos(theta))  # traces under JAX
            + 0.1 * x_cart**2
            + 0.001 * (dx**2 + dtheta**2)
            + 0.001 * u[0] ** 2
        )

    def h(self, x, t=0.0, params=None):
        return 0.0


problem = PlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
)

grid = StateSpaceGrid(
    problem,
    x_grid_shape=(31, 51, 31, 51),
    u_grid_shape=(5,),
    dt=DT,
    precompute=False,
    verbose=True,
)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    backend="jax",
    alpha=1.0,
    out_of_bound_cost=INF,
    verbose=True,
)
solution = planner.solve_steps(N_STEPS)
print(solution.solver)

# planner.plot_cost2go(jmax=INF, axes=(1, 3), anchor=UPRIGHT)
# planner.plot_policy(axis=0, axes=(1, 3), anchor=UPRIGHT)

vi_ctl = planner.get_controller()
vi_ctl.plot_control_law(x_axis=1, y_axis=3, u_axis=0)  # force vs (theta, dtheta)

# report = MonteCarloEvaluator(
#     problem, dt=DT, n_trials=100, seed=1, backend="numpy"
# ).evaluate(vi_ctl)
# print(report)

plant.x0 = np.array([0.0, 0.05, 0.0, 0.0])  # hanging, a tiny tip
cl_sys = vi_ctl @ plant
cl_sys.name = "Cart-pole with the tabulated law"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)
cl_sys.animate(traj)
