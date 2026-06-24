"""Tests for dynamic-programming policy synthesis."""

import unittest

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
    DynamicProgrammingResult,
)
from minilink.planning.policy_synthesis.policy_eval import PolicyEvaluator
from minilink.planning.problems import PlanningProblem


class DoubleIntegrator(DynamicSystem):
    """Minimal double integrator: dx = [x[1], u[0]]."""

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
        self.state.lower_bound = np.array([-3.0, -3.0])
        self.state.upper_bound = np.array([3.0, 3.0])
        self.inputs["u"].lower_bound = np.array([-1.0])
        self.inputs["u"].upper_bound = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.array([x[1], u[0]])


def make_problem():
    sys = DoubleIntegrator()
    cost = QuadraticCost.from_system(sys, xbar=np.zeros(2))
    return PlanningProblem(sys, x_goal=np.zeros(2), cost=cost)


def solve(problem, *, precompute=True, **opt_kwargs):
    grid = StateSpaceGrid(
        problem, x_grid_shape=(31, 31), u_grid_shape=(7,), dt=0.1, precompute=precompute
    )
    options = DynamicProgrammingOptions(
        alpha=0.95, tol=1e-3, max_iterations=400, **opt_kwargs
    )
    planner = DynamicProgrammingPlanner(problem, grid=grid, options=options)
    return planner, planner.compute_solution()


class TestStateSpaceGrid(unittest.TestCase):
    def test_dimensions_and_meshgrid_order(self):
        grid = StateSpaceGrid(
            make_problem(), x_grid_shape=(5, 4), u_grid_shape=(3,), dt=0.1
        )
        self.assertEqual(grid.nodes_n, 20)
        self.assertEqual(grid.actions_n, 3)
        self.assertEqual(grid.states.shape, (20, 2))
        # grid_from_array is the inverse reshape of the node ordering
        values = np.arange(grid.nodes_n, dtype=float)
        self.assertTrue(np.array_equal(grid.grid_from_array(values).ravel(), values))

    def test_nearest_lookups(self):
        grid = StateSpaceGrid(
            make_problem(), x_grid_shape=(7, 7), u_grid_shape=(5,), dt=0.1
        )
        node = grid.nearest_node([0.0, 0.0])
        self.assertTrue(np.allclose(grid.states[node], [0.0, 0.0]))
        action = grid.nearest_action([1.0])
        self.assertTrue(np.allclose(grid.inputs[action], [1.0]))

    def test_infinite_bounds_raise(self):
        sys = DoubleIntegrator()
        sys.state.upper_bound = np.array([np.inf, 3.0])
        problem = PlanningProblem(sys, x_goal=np.zeros(2))
        with self.assertRaises(ValueError):
            StateSpaceGrid(problem, x_grid_shape=(5, 5), u_grid_shape=(3,), dt=0.1)


class TestValueIteration(unittest.TestCase):
    def test_converges(self):
        _, result = solve(make_problem())
        self.assertLess(result.delta, 1e-3)
        self.assertGreater(result.iterations, 1)

    def test_value_zero_at_goal_and_grows_with_distance(self):
        _, result = solve(make_problem())
        grid = result.grid
        goal = grid.nearest_node([0.0, 0.0])
        near = grid.nearest_node([0.5, 0.0])
        far = grid.nearest_node([2.0, 0.0])
        self.assertAlmostEqual(result.J[goal], 0.0, places=4)
        self.assertLess(result.J[near], result.J[far])

    def test_greedy_action_opposes_error(self):
        _, result = solve(make_problem())
        grid = result.grid
        for x, sign in [
            ([2.0, 0.0], -1),
            ([-2.0, 0.0], 1),
            ([0.0, 2.0], -1),
            ([0.0, -2.0], 1),
        ]:
            u = grid.inputs[result.pi[grid.nearest_node(x)]][0]
            self.assertEqual(np.sign(u), sign)

    def test_solve_steps_runs_fixed_count(self):
        problem = make_problem()
        grid = StateSpaceGrid(problem, x_grid_shape=(21, 21), u_grid_shape=(5,), dt=0.1)
        planner = DynamicProgrammingPlanner(problem, grid=grid)
        result = planner.solve_steps(5)
        self.assertEqual(result.iterations, 5)

    def test_out_of_bound_penalty_and_cleanup(self):
        planner, result = solve(make_problem())
        penalty = planner.options.out_of_bound_cost
        # the double integrator cannot stay bounded from every corner
        self.assertTrue(np.any(result.J > penalty - 1.0))
        planner.clean_infeasible_set()
        self.assertTrue(np.all(result.J[result.J > penalty - 1.0] == penalty))

    def test_memory_mode_parity(self):
        problem = make_problem()
        _, fast = solve(problem, precompute=True)
        _, slow = solve(problem, precompute=False)
        self.assertTrue(np.allclose(fast.J, slow.J))
        self.assertTrue(np.array_equal(fast.pi, slow.pi))

    def test_deterministic(self):
        problem = make_problem()
        _, a = solve(problem)
        _, b = solve(problem)
        self.assertTrue(np.array_equal(a.J, b.J))
        self.assertTrue(np.array_equal(a.pi, b.pi))

    def test_record_history(self):
        _, result = solve(make_problem(), record_history=True)
        self.assertEqual(len(result.history), result.iterations + 1)


class TestControllerAndEvaluation(unittest.TestCase):
    def test_closed_loop_reaches_goal(self):
        problem = make_problem()
        planner, result = solve(problem)
        planner.clean_infeasible_set()
        controller = result.controller()

        plant = problem.sys
        plant.x0 = np.array([2.0, 0.0])
        diagram = DiagramSystem()
        diagram.add_subsystem(controller, "controller")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "x", "controller", "x")
        diagram.connect("controller", "u", "plant", "u")

        traj = diagram.compute_trajectory(tf=8.0)
        self.assertLess(np.linalg.norm(traj.x[:, -1]), 0.3)

    def test_policy_evaluator_matches_optimal_value(self):
        problem = make_problem()
        planner, result = solve(problem)
        controller = result.controller()
        evaluator = PolicyEvaluator(
            problem, grid=result.grid, policy=controller.action, options=planner.options
        )
        J_pi = evaluator.compute_solution()
        feasible = result.J < planner.options.out_of_bound_cost - 1.0
        self.assertLess(np.max(np.abs(J_pi[feasible] - result.J[feasible])), 0.05)

    def test_result_save_load_round_trip(self):
        import os
        import tempfile

        _, result = solve(make_problem())
        path = os.path.join(tempfile.mkdtemp(), "dp.npz")
        result.save(path)
        loaded = DynamicProgrammingResult.load(path, result.grid)
        self.assertTrue(np.array_equal(result.J, loaded.J))
        self.assertTrue(np.array_equal(result.pi, loaded.pi))


if __name__ == "__main__":
    unittest.main()
