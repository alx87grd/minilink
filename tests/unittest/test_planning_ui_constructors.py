"""Tier-1 flat constructor contracts for planning UI simplification."""

import unittest

import numpy as np
import pytest

from minilink.core.costs import QuadraticCost
from minilink.core.system import DynamicSystem
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from tests.unittest.planning_helpers import make_holonomic_obstacle_problem


class _SingleIntegrator(DynamicSystem):
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


class _DoubleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
        self.state.lower_bound = np.array([-3.0, -3.0])
        self.state.upper_bound = np.array([3.0, 3.0])
        self.inputs["u"].lower_bound = np.array([-1.0])
        self.inputs["u"].upper_bound = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.array([x[1], u[0]])


def _trajopt_problem():
    sys = _SingleIntegrator()
    return PlanningProblem(
        sys=sys,
        tf=1.0,
        x_start=np.array([0.0]),
        cost=QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        ),
    )


class TestTrajoptFlatConstructor(unittest.TestCase):
    def test_flat_matches_nested(self):
        problem = _trajopt_problem()
        nested = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                record_solve_time=True,
                optimizer_options={"maxiter": 20, "ftol": 1e-3},
            ),
        )
        flat = TrajectoryOptimizationPlanner(
            problem,
            n_steps=5,
            transcription="direct_collocation",
            compile_backend="numpy",
            record_solve_time=True,
            optimizer_options={"maxiter": 20, "ftol": 1e-3},
        )
        self.assertIsInstance(flat.transcription, DirectCollocationTranscription)
        self.assertEqual(flat.transcription.options.n_steps, 5)
        self.assertEqual(flat.options.compile_backend, nested.options.compile_backend)
        self.assertEqual(
            flat.options.record_solve_time, nested.options.record_solve_time
        )
        self.assertEqual(
            flat.options.optimizer_options, nested.options.optimizer_options
        )

    def test_string_presets(self):
        problem = _trajopt_problem()
        dc = TrajectoryOptimizationPlanner(
            problem, n_steps=4, transcription="direct_collocation"
        )
        ms = TrajectoryOptimizationPlanner(
            problem, n_steps=4, transcription="multiple_shooting"
        )
        self.assertIsInstance(dc.transcription, DirectCollocationTranscription)
        self.assertIsInstance(ms.transcription, MultipleShootingTranscription)

    def test_n_steps_required_for_preset(self):
        problem = _trajopt_problem()
        with self.assertRaises(ValueError):
            TrajectoryOptimizationPlanner(problem, transcription="direct_collocation")

    def test_unknown_transcription(self):
        problem = _trajopt_problem()
        with self.assertRaises(ValueError):
            TrajectoryOptimizationPlanner(problem, n_steps=4, transcription="bogus")


class TestRrtFlatConstructor(unittest.TestCase):
    def test_flat_matches_options(self):
        problem, _ = make_holonomic_obstacle_problem()
        extender = KinodynamicExtender(
            controls=[np.array([1.0, 0.0])], horizon=0.5, n_substeps=4
        )
        nested = RRTPlanner(
            problem,
            extender,
            options=RRTOptions(max_nodes=100, goal_bias=0.2, seed=1),
        )
        flat = RRTPlanner(problem, extender, max_nodes=100, goal_bias=0.2, seed=1)
        self.assertEqual(flat.options.max_nodes, nested.options.max_nodes)
        self.assertEqual(flat.options.goal_bias, nested.options.goal_bias)
        self.assertEqual(flat.options.seed, nested.options.seed)


class TestDpFlatConstructor(unittest.TestCase):
    def test_flat_matches_options(self):
        sys = _DoubleIntegrator()
        problem = PlanningProblem(
            sys=sys,
            x_start=np.array([0.0, 0.0]),
            cost=QuadraticCost.from_system(sys),
        )
        grid = StateSpaceGrid(problem, x_grid_shape=(5, 5), u_grid_shape=(3,), dt=0.1)
        nested = DynamicProgrammingPlanner(
            problem,
            grid=grid,
            options=DynamicProgrammingOptions(alpha=0.9, tol=0.2, max_iterations=10),
        )
        flat = DynamicProgrammingPlanner(
            problem, grid=grid, alpha=0.9, tol=0.2, max_iterations=10
        )
        self.assertEqual(flat.options.alpha, nested.options.alpha)
        self.assertEqual(flat.options.tol, nested.options.tol)
        self.assertEqual(flat.options.max_iterations, nested.options.max_iterations)


@pytest.mark.optional
@pytest.mark.jax
class TestHybridDefaultComputerX0(unittest.TestCase):
    def test_omit_x0_computer_matches_helper(self):
        pytest.importorskip("jax")
        import jax.numpy as jnp

        from minilink.control.mpc import ModelPredictiveController
        from minilink.control.mpc.utilities import mpc_default_computer_x0
        from minilink.core.backends import configure_jax

        class JaxSingleIntegrator(DynamicSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
                self.state.lower_bound = np.array([-10.0])
                self.state.upper_bound = np.array([10.0])
                self.inputs["u"].lower_bound = np.array([-10.0])
                self.inputs["u"].upper_bound = np.array([10.0])

            def f(self, x, u, t=0, params=None):
                return jnp.array([u[0]])

            def h(self, x, u, t=0, params=None):
                return x

        configure_jax(enable_x64=True)
        sys = JaxSingleIntegrator()
        sys.x0 = np.array([0.0])
        problem = PlanningProblem(
            sys=sys,
            tf=0.4,
            x_start=np.array([0.0]),
            cost=QuadraticCost.from_system(
                sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
            ),
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            n_steps=4,
            transcription="direct_collocation",
            compile_backend="jax",
            optimizer_options={"maxiter": 30, "ftol": 1e-3},
        )
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        hybrid = mpc @ sys
        z0 = mpc_default_computer_x0(planner)
        with_helper = hybrid.compute_trajectory(
            tf=0.4,
            x0_plant=np.array([0.0]),
            x0_computer=z0,
            compile_backend="numpy",
            verbose=False,
        )
        without = hybrid.compute_trajectory(
            tf=0.4,
            x0_plant=np.array([0.0]),
            compile_backend="numpy",
            verbose=False,
        )
        np.testing.assert_allclose(
            without.plant.x[:, -1], with_helper.plant.x[:, -1], atol=1e-8, rtol=1e-8
        )


if __name__ == "__main__":
    unittest.main()
