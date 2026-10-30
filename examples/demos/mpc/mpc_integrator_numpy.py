"""Minimal sampled MPC without JAX — the NumPy program rebuilt each tick."""

import numpy as np

from minilink import (
    DynamicSystem,
    PlanningProblem,
    QuadraticCost,
    SingletonSet,
    TrajectoryOptimizationPlanner,
)
from minilink.control import ModelPredictiveController


class SingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.state.lower_bound = np.array([-10.0])
        self.state.upper_bound = np.array([10.0])
        self.inputs["u"].lower_bound = np.array([-10.0])
        self.inputs["u"].upper_bound = np.array([10.0])

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


TF_SIM = 2.0
MPC_DT = 0.2
SIM_DT = 0.01
VERBOSE = True

sys = SingleIntegrator()
x0 = np.array([0.5])
x_goal = np.array([2.0])
sys.x0 = x0.copy()

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        sys=sys,
        tf=1.0,
        x_start=x0,
        Xf=SingletonSet(x_goal),
        cost=QuadraticCost.from_system(
            sys,
            Q=np.eye(1),
            R=0.1 * np.eye(1),
            xbar=x_goal,
        ),
        X=sys.state.box,
        U=sys.input_box,
    ),
    n_steps=8,
    transcription="direct_collocation",
    backend="numpy",
    optimizer_options={"maxiter": 80, "ftol": 1e-6},
)

mpc = ModelPredictiveController(planner, delay=False)

# The MPC on its clock, the plant sampled at it with SIM_DT steps inside each period
hybrid = (mpc % MPC_DT) @ sys

hybrid.plot_diagram()
result = hybrid.compute_trajectory(tf=TF_SIM, dt=SIM_DT)
if VERBOSE:
    start, end = hybrid.state_index["plant"]
    print(f"x(tf) = {result.x[start:end, -1]}, the goal {x_goal}")
hybrid.plot_trajectory()
