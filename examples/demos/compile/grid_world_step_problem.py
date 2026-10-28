"""A grid world as a step problem: a native step system, its cost per tick, solved by value iteration on its step face."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import StepCost, StepPlanningProblem, StepSystem
from minilink.planning import DynamicProgrammingPlanner

SIZE = 9  # cells per side
GOAL = np.array([7.0, 2.0])  # the goal cell
WALL = [(4.0, row) for row in range(1, 9)]  # a wall with a gap at the bottom row


class Walker(StepSystem):
    """A walker on the cells: ``x_{k+1} = x_k + u_k`` (a move to any of the 8 neighbours, or none), blocked by the wall and the edges."""

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2)
        self.state.lower_bound[:] = [0.0, 0.0]
        self.state.upper_bound[:] = [SIZE - 1.0, SIZE - 1.0]
        self.inputs["u"].lower_bound[:] = [-1.0, -1.0]
        self.inputs["u"].upper_bound[:] = [1.0, 1.0]
        self.params = {"wall": np.array(WALL)}

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        wall = params["wall"]

        # x_{k+1} = x_k + u_k inside the grid, x_k when the move hits the wall
        x_next = np.clip(x + u, 0.0, SIZE - 1.0)
        blocked = np.any(np.all(np.abs(wall - x_next) < 0.5, axis=1))
        x_next = x if blocked else x_next

        return x_next


class Moves(StepCost):
    """``c_k = 1`` per tick away from the goal, ``0`` on it: ``J`` counts the moves."""

    def __init__(self, goal):
        self.params = {"goal": np.asarray(goal, dtype=float)}

    def c(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        goal = params["goal"]

        # c_k = 1 off the goal cell
        c = float(np.any(np.abs(x - goal) > 0.5))

        return c

    def h(self, x, k=0, params=None):
        return 0.0


# The task: N ticks, no discount, from the bottom-left cell
walker = Walker()
problem = StepPlanningProblem(
    walker,
    N=None,
    gamma=1.0,
    cost=Moves(GOAL),
    x_start=[0.0, 8.0],
    X=walker.state.box,
    U=walker.input_box,
    infeasible_cost=100.0,
)

# Value iteration on the face's transition, one cell per grid node
vi = DynamicProgrammingPlanner(
    problem, x_grid=(SIZE, SIZE), u_grid=(3, 3), tol=1e-9, max_iterations=100
)
solution = vi.solve()
print(solution.solver)

# The greedy walk on the same face: J = Σ c_k, the moves to the goal
face = problem.compile()
law = vi.get_controller()
xs, us, cs, J = face.run(
    lambda x, k: np.asarray(law.outputs["u"].compute(None, x, k)), n_steps=20
)
print(
    f"moves to the goal: J = {J:g}, the cost-to-go at the start {vi.value_at(problem.x_start):g}"
)

# The cost-to-go and the walk
fig, ax = plt.subplots()
J_grid = vi.result.J.reshape(SIZE, SIZE).T
image = ax.imshow(J_grid, origin="lower", cmap="viridis")
ax.plot(xs[:, 0], xs[:, 1], "w.-", label="greedy walk")
ax.plot(*GOAL, "r*", markersize=14, label="goal")
fig.colorbar(image, label="moves to the goal")
ax.set_xlabel("cell x")
ax.set_ylabel("cell y")
ax.legend(loc="upper right")
plt.show()
