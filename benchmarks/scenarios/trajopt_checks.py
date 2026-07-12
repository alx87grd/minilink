"""Trajectory-optimization scenarios for integration regression."""

from __future__ import annotations

import time

import numpy as np

from benchmarks.scenarios.common import (
    TrajoptScenarioResult,
    assert_simulation_finite,
    sample_trajectory_at_times,
)
from benchmarks.trajopt import jax_trajopt_available
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

SHOWCASE_TRAJOPT_CHECKPOINTS = (0.0, 2.0, 4.0)
SHOWCASE_TRAJOPT_TF = 4.0
SHOWCASE_TRAJOPT_N_STEPS = 20
SHOWCASE_TRAJOPT_MAXITER = 200
SHOWCASE_TRAJOPT_FTOL = 1e-2


def build_showcase_cartpole_problem() -> PlanningProblem:
    """Cart-pole swing-up aligned with ``demo_showcase.ipynb`` section 9."""
    configure_jax(enable_x64=True)
    sys = JaxCartPole()
    sys.inputs["u"].lower_bound[0] = -10.0
    sys.inputs["u"].upper_bound[0] = 10.0

    x_start = np.array([-2.0, 0.0, 0.0, 0.0])
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    cost = QuadraticCost.from_system(
        sys,
        Q=np.diag([1.0, 1.0, 0.0, 0.0]),
        xbar=x_goal,
    )
    return PlanningProblem(
        sys=sys,
        x_start=x_start,
        x_goal=x_goal,
        cost=cost,
    )


def run_showcase_cartpole_trajopt(*, n_runs: int = 1) -> TrajoptScenarioResult:
    """Showcase cart-pole swing-up with SciPy SLSQP on the JAX compile backend."""
    if not jax_trajopt_available():
        raise ModuleNotFoundError(
            "JAX is required for the showcase cart-pole trajopt check"
        )

    problem = build_showcase_cartpole_problem()
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(
                tf=SHOWCASE_TRAJOPT_TF,
                n_steps=SHOWCASE_TRAJOPT_N_STEPS,
            )
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            optimizer_method="scipy_slsqp",
            optimizer_options={
                "maxiter": SHOWCASE_TRAJOPT_MAXITER,
                "ftol": SHOWCASE_TRAJOPT_FTOL,
                "disp": False,
            },
            record_solve_time=True,
            solve_disp=False,
        ),
    )

    total_times: list[float] = []
    solve_times: list[float] = []
    traj = None
    optimization_result = None
    eq_inf = float("inf")

    for _ in range(n_runs):
        t0 = time.perf_counter()
        traj = planner.compute_solution()
        total_s = time.perf_counter() - t0
        total_times.append(total_s)

        optimization_result = planner.last_optimization_result
        optimizer = planner.last_optimizer
        if optimization_result is None or optimizer is None:
            raise RuntimeError("trajectory optimization did not store a solution")
        solve_times.append(float(optimization_result.solve_time_s))
        max_eq, _, _ = optimizer.program_evaluator.constraint_violations(
            optimization_result.z
        )
        eq_inf = float(max_eq)

    assert traj is not None
    assert optimization_result is not None
    assert_simulation_finite(traj)
    samples = sample_trajectory_at_times(traj, SHOWCASE_TRAJOPT_CHECKPOINTS)

    return TrajoptScenarioResult(
        checkpoint_times=SHOWCASE_TRAJOPT_CHECKPOINTS,
        checkpoint_x={
            t: [float(v) for v in samples[t]] for t in SHOWCASE_TRAJOPT_CHECKPOINTS
        },
        x_tf=[float(v) for v in traj.x[:, -1].reshape(-1)],
        cost=float(optimization_result.cost),
        eq_inf=eq_inf,
        success=bool(optimization_result.success),
        total_s=float(np.mean(total_times)),
        transcribe_s=0.0,
        solve_s=float(np.mean(solve_times)),
    )
