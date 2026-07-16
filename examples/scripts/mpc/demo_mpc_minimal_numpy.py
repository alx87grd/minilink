"""Minimal hybrid MPC without JAX — NumPy rebuild each replan tick.

``compile_backend='numpy'`` skips parametric compile; each MPC tick
transcribes and solves a fresh NLP. Suitable for teaching and small plants.

Run from repo root::

    python examples/scripts/mpc/demo_mpc_minimal_numpy.py
"""

import numpy as np

from minilink.control.mpc import ModelPredictiveController
from minilink.core.costs import QuadraticCost
from minilink.core.system import DynamicSystem
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)


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
STEP_DISP = True


def _build_hybrid(*, maxiter: int, step_disp: bool):
    sys = SingleIntegrator()
    x0 = np.array([0.5])
    x_goal = np.array([2.0])
    sys.x0 = x0.copy()

    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=sys,
            tf=1.0,
            x_start=x0,
            x_goal=x_goal,
            cost=QuadraticCost.from_system(
                sys,
                Q=np.eye(1),
                R=0.1 * np.eye(1),
                S=np.zeros((1, 1)),
                xbar=x_goal,
            ),
        ),
        n_steps=8,
        transcription="direct_collocation",
        compile_backend="numpy",
        optimizer_options={"maxiter": maxiter, "ftol": 1e-6},
    )

    mpc = ModelPredictiveController(
        planner, dt_mpc=MPC_DT, warm_start=True, step_disp=step_disp
    )
    return mpc @ sys, x0


def run_smoke(*, tf: float = 0.5) -> None:
    """Headless demo check: short NumPy-rebuild MPC rollout."""
    hybrid, x0 = _build_hybrid(maxiter=40, step_disp=False)
    hybrid.compute_trajectory(
        tf=tf,
        x0_plant=x0,
        plant_dt_inner=SIM_DT,
        compile_backend="numpy",
    )


def main() -> None:
    hybrid, x0 = _build_hybrid(maxiter=80, step_disp=STEP_DISP)

    hybrid.plot_diagram()
    hybrid.compute_trajectory(
        tf=TF_SIM,
        x0_plant=x0,
        plant_dt_inner=SIM_DT,
        compile_backend="numpy",
    )
    hybrid.plot_trajectory()


if __name__ == "__main__":
    main()
