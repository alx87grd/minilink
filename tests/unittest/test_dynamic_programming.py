"""Value iteration: grid, planner backends, lookup controller and policy evaluation."""

import unittest

import numpy as np

from minilink.core.costs import QuadraticCost, TimeCost
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem
from minilink.planning.policy_synthesis import dp_jax, plotting
from minilink.planning.policy_synthesis.approximation import (
    LinearApproximator,
    QuadraticFeatures,
    RadialBasisFeatures,
)
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


def make_pendulum_problem():
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    sys = Pendulum()
    sys.state.lower_bound = np.array([-2.0, -2.0])
    sys.state.upper_bound = np.array([2.0, 2.0])
    sys.inputs["u"].lower_bound = np.array([-1.0])
    sys.inputs["u"].upper_bound = np.array([1.0])
    return PlanningProblem(sys, x_goal=np.zeros(2), cost=QuadraticCost.from_system(sys))


def solve(problem, *, precompute=True, **opt_kwargs):
    grid = StateSpaceGrid(
        problem, x_grid_shape=(31, 31), u_grid_shape=(7,), dt=0.1, precompute=precompute
    )
    options = DynamicProgrammingOptions(
        alpha=0.95, tol=0.001, max_iterations=400, **opt_kwargs
    )
    planner = DynamicProgrammingPlanner(problem, grid=grid, options=options)
    planner.solve()
    return (planner, planner.result)


class TestStateSpaceGrid(unittest.TestCase):
    def test_ensure_jax_transition_builds_deferred_grid(self):
        import pytest

        pytest.importorskip("jax")
        problem = make_pendulum_problem()
        grid = StateSpaceGrid(
            problem, x_grid_shape=(11, 11), u_grid_shape=(5,), dt=0.1, precompute=False
        )
        self.assertFalse(grid.precomputed)
        DynamicProgrammingPlanner(
            problem,
            grid=grid,
            options=DynamicProgrammingOptions(backend="jax", max_iterations=1),
        )
        self.assertTrue(grid.precomputed)
        self.assertEqual(grid.x_next.shape, (121, 5, 2))

    def test_jax_precompute_matches_numpy(self):
        import pytest

        pytest.importorskip("jax")
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        sys = Pendulum()
        sys.state.lower_bound = np.array([-2.0, -2.0])
        sys.state.upper_bound = np.array([2.0, 2.0])
        sys.inputs["u"].lower_bound = np.array([-1.0])
        sys.inputs["u"].upper_bound = np.array([1.0])
        problem = PlanningProblem(
            sys, x_goal=np.zeros(2), cost=QuadraticCost.from_system(sys)
        )
        grid_np = StateSpaceGrid(
            problem, x_grid_shape=(11, 11), u_grid_shape=(5,), dt=0.1
        )
        grid_jax = StateSpaceGrid(
            problem,
            x_grid_shape=(11, 11),
            u_grid_shape=(5,),
            dt=0.1,
            precompute_backend="jax",
        )
        self.assertTrue(np.allclose(grid_np.x_next, grid_jax.x_next))
        self.assertTrue(np.array_equal(grid_np.action_ok, grid_jax.action_ok))
        self.assertTrue(np.array_equal(grid_np.x_next_ok, grid_jax.x_next_ok))

    def test_dimensions_and_meshgrid_order(self):
        grid = StateSpaceGrid(
            make_problem(), x_grid_shape=(5, 4), u_grid_shape=(3,), dt=0.1
        )
        self.assertEqual(grid.nodes_n, 20)
        self.assertEqual(grid.actions_n, 3)
        self.assertEqual(grid.states.shape, (20, 2))
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
        self.assertLess(result.delta, 0.001)
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
        planner.solve_steps(5)
        result = planner.result
        self.assertEqual(result.iterations, 5)

    def test_out_of_bound_penalty_and_cleanup(self):
        planner, result = solve(make_problem())
        penalty = planner.options.out_of_bound_cost
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
    def test_get_controller_builds_an_interpolation_variant(self):
        problem = make_problem()
        planner = DynamicProgrammingPlanner(
            problem, x_grid=(11, 11), u_grid=(3,), dt=0.1, max_iterations=20
        )
        planner.solve()
        self.assertIs(planner.get_controller(), planner.require_solution().policy)
        nearest = planner.get_controller(interpolation="nearest")
        self.assertIsNot(nearest, planner.get_controller())
        np.testing.assert_array_equal(nearest.pi, planner.result.pi)

    def test_verbs_before_solve_raise_the_no_solution_error(self):
        planner = DynamicProgrammingPlanner(
            make_problem(), x_grid=(5, 5), u_grid=(3,), dt=0.1
        )
        verbs = {
            "value_at": lambda: planner.value_at([0.0, 0.0]),
            "get_controller": lambda: planner.get_controller(interpolation="nearest"),
            "plot_cost2go": planner.plot_cost2go,
            "plot_policy": planner.plot_policy,
            "animate_cost2go": planner.animate_cost2go,
            "animate_policy": planner.animate_policy,
        }
        for name, verb in verbs.items():
            with self.subTest(verb=name):
                with self.assertRaisesRegex(ValueError, "No solution"):
                    verb()

    def test_closed_loop_reaches_goal(self):
        problem = make_problem()
        planner, result = solve(problem)
        planner.clean_infeasible_set()
        controller = plotting.get_controller(result)
        plant = problem.sys
        plant.x0 = np.array([2.0, 0.0])
        diagram = DiagramSystem()
        diagram.add_subsystem(controller, "controller")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "x", "controller", "x")
        diagram.connect("controller", "u", "plant", "u")
        traj = diagram.compute_trajectory(tf=8.0, verbose=False)
        self.assertLess(np.linalg.norm(traj.x[:, -1]), 0.3)

    def test_policy_evaluator_matches_optimal_value(self):
        problem = make_problem()
        planner, result = solve(problem)
        controller = plotting.get_controller(result)
        evaluator = PolicyEvaluator(
            problem, grid=result.grid, policy=controller.action, options=planner.options
        )
        J_pi = evaluator.solve()
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


class TestJaxPrecompute(unittest.TestCase):
    def _pendulum_grid(self):
        import pytest

        pytest.importorskip("jax")
        problem = make_pendulum_problem()
        grid = StateSpaceGrid(problem, x_grid_shape=(11, 11), u_grid_shape=(5,), dt=0.1)
        return (problem, grid)

    def test_jax_g_table_matches_numpy(self):
        problem, grid = self._pendulum_grid()
        np_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy")
        )
        jax_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="jax")
        )
        G_np = np_planner.running_cost_table(0.0)
        G_jax = dp_jax.running_cost_table(jax_planner, 0.0)
        self.assertTrue(np.allclose(G_np, G_jax))

    def test_jax_j0_matches_numpy(self):
        problem, grid = self._pendulum_grid()
        np_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy")
        )
        jax_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="jax")
        )
        J0_np = np_planner.terminal_cost(0.0)
        J0_jax = dp_jax.terminal_cost(jax_planner, 0.0)
        self.assertTrue(np.allclose(J0_np, J0_jax))

    def test_jax_time_cost_g_table_matches_numpy(self):
        import pytest

        pytest.importorskip("jax")
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        sys = Pendulum()
        sys.state.lower_bound = np.array([-2.0, -2.0])
        sys.state.upper_bound = np.array([2.0, 2.0])
        sys.inputs["u"].lower_bound = np.array([-1.0])
        sys.inputs["u"].upper_bound = np.array([1.0])
        problem = PlanningProblem(
            sys, x_goal=np.zeros(2), cost=TimeCost.from_system(sys, eps=0.1)
        )
        grid = StateSpaceGrid(
            problem, x_grid_shape=(11, 11), u_grid_shape=(5,), dt=0.1, precompute=False
        )
        DynamicProgrammingPlanner(
            problem,
            grid=grid,
            options=DynamicProgrammingOptions(backend="jax", max_iterations=1),
        )
        np_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy")
        )
        jax_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="jax")
        )
        G_np = np_planner.running_cost_table(0.0)
        G_jax = dp_jax.running_cost_table(jax_planner, 0.0)
        self.assertTrue(np.allclose(G_np, G_jax))


class TestBackends(unittest.TestCase):
    def test_loop_matches_numpy(self):
        problem = make_problem()
        _, loop = solve(problem, backend="loop")
        _, vectorized = solve(problem, backend="numpy")
        self.assertTrue(np.allclose(loop.J, vectorized.J))
        self.assertTrue(np.array_equal(loop.pi, vectorized.pi))

    def test_jax_matches_numpy(self):
        import pytest

        pytest.importorskip("jax")
        problem = make_pendulum_problem()
        _, vectorized = solve(problem, backend="numpy", precompute=False)
        _, jax_result = solve(problem, backend="jax", precompute=False)
        feasible = vectorized.J < 100000.0
        gap = np.max(np.abs(jax_result.J[feasible] - vectorized.J[feasible]))
        self.assertLess(gap, 0.0001)
        self.assertGreater(np.mean(jax_result.pi == vectorized.pi), 0.98)

    def test_value_iteration_is_the_algorithm_behind_every_verb(self):
        """solve_steps is value_iteration wrapped in a solution, on every backend."""
        backends = ["loop", "numpy"]
        try:
            import jax  # noqa: F401

            backends.append("jax")
        except ModuleNotFoundError:
            pass
        problem = make_pendulum_problem()
        for backend in backends:
            grid = StateSpaceGrid(
                problem,
                x_grid_shape=(11, 11),
                u_grid_shape=(5,),
                dt=0.1,
                precompute=False,
            )
            planner = DynamicProgrammingPlanner(
                problem, grid=grid, backend=backend, alpha=0.95, clean_infeasible=False
            )
            tables = planner.value_iteration(4, stop_on_tol=False)
            solution = planner.solve_steps(4)
            self.assertEqual(tables.iterations, 4, backend)
            np.testing.assert_array_equal(tables.J, planner.result.J, err_msg=backend)
            np.testing.assert_array_equal(tables.pi, planner.result.pi)
            self.assertTrue(solution.success)

    def test_jax_fixed_horizon_matches_numpy(self):
        import pytest

        pytest.importorskip("jax")
        problem = make_pendulum_problem()
        results = {}
        for backend in ("numpy", "jax"):
            grid = StateSpaceGrid(
                problem,
                x_grid_shape=(31, 31),
                u_grid_shape=(7,),
                dt=0.1,
                precompute=False,
            )
            options = DynamicProgrammingOptions(backend=backend, alpha=0.95)
            planner = DynamicProgrammingPlanner(problem, grid=grid, options=options)
            planner.solve_steps(15)
            results[backend] = planner.result

        numpy_J, jax_J = results["numpy"].J, results["jax"].J
        self.assertEqual(results["jax"].iterations, 15)
        rel_gap = np.max(np.abs(jax_J - numpy_J) / np.maximum(np.abs(numpy_J), 1.0))
        self.assertLess(rel_gap, 1e-2)
        self.assertGreater(np.mean(results["jax"].pi == results["numpy"].pi), 0.98)

    def test_jax_sweep_by_sweep_matches_the_device_loop(self):
        """verbose or record_history run the JAX sweeps one at a time, to the same result."""
        import contextlib
        import io

        import pytest

        pytest.importorskip("jax")
        problem = make_pendulum_problem()

        def jax_solve(**options):
            grid = StateSpaceGrid(
                problem,
                x_grid_shape=(21, 21),
                u_grid_shape=(5,),
                dt=0.1,
                precompute=False,
            )
            planner = DynamicProgrammingPlanner(
                problem, grid=grid, backend="jax", alpha=0.95, tol=0.01, **options
            )
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                planner.solve()
            return planner.result, stdout.getvalue()

        device, _ = jax_solve()
        recorded, _ = jax_solve(record_history=True)
        reported, report = jax_solve(verbose=True)

        for result in (recorded, reported):
            self.assertEqual(result.iterations, device.iterations)
            np.testing.assert_allclose(result.J, device.J, rtol=1e-12, atol=1e-9)
            np.testing.assert_array_equal(result.pi, device.pi)
        self.assertEqual(len(recorded.history), recorded.iterations + 1)
        self.assertIn("Bellman equation solved!", report)

    def test_time_varying_dynamics_on_a_grid_rebuilt_each_sweep(self):
        """precompute=False rebuilds the tables at each sweep's time: loop and table agree."""

        class DrivenIntegrator(DoubleIntegrator):
            def f(self, x, u, t=0, params=None):
                return np.array([x[1], u[0] + 0.5 * np.sin(2.0 * t)])

        sys = DrivenIntegrator()
        cost = QuadraticCost.from_system(sys, xbar=np.zeros(2))
        problem = PlanningProblem(sys, x_goal=np.zeros(2), cost=cost)

        def fixed_horizon(backend, final_time):
            grid = StateSpaceGrid(
                problem,
                x_grid_shape=(15, 15),
                u_grid_shape=(5,),
                dt=0.1,
                precompute=False,
            )
            options = DynamicProgrammingOptions(backend=backend, final_time=final_time)
            planner = DynamicProgrammingPlanner(problem, grid=grid, options=options)
            planner.solve_steps(6)
            return planner.result

        loop = fixed_horizon("loop", final_time=1.0)
        table = fixed_horizon("numpy", final_time=1.0)
        np.testing.assert_allclose(loop.J, table.J)
        np.testing.assert_array_equal(loop.pi, table.pi)

        # The sweeps read the time: another horizon gives another cost-to-go
        shifted = fixed_horizon("numpy", final_time=0.0)
        self.assertFalse(np.allclose(shifted.J, table.J))


class TestDynamicProgrammingPlotting(unittest.TestCase):
    def test_plot_cost2go_smoke(self):
        import matplotlib

        matplotlib.use("Agg")
        _, result = solve(make_problem())
        fig, ax = plotting.plot_cost2go(result, show=False)
        self.assertIsNotNone(fig)
        self.assertIsNotNone(ax)

    def test_plot_policy_smoke(self):
        import matplotlib

        matplotlib.use("Agg")
        _, result = solve(make_problem())
        fig, ax = plotting.plot_policy(result, show=False)
        self.assertIsNotNone(fig)
        self.assertIsNotNone(ax)

    def test_planner_plot_delegates(self):
        import matplotlib

        matplotlib.use("Agg")
        problem = make_problem()
        planner, _ = solve(problem)
        fig, ax = planner.plot_cost2go(show=False)
        self.assertIsNotNone(fig)
        controller = planner.get_controller()
        self.assertTrue(hasattr(controller, "action"))

    def test_get_controller_smoke(self):
        _, result = solve(make_problem())
        controller = plotting.get_controller(result)
        u = controller.action(result.grid.states[0])
        self.assertEqual(u.shape, (1,))

    def test_plot_policy_trajectory_overlay(self):
        import matplotlib

        matplotlib.use("Agg")
        problem = make_problem()
        planner, result = solve(problem)
        planner.clean_infeasible_set()
        plant = problem.sys
        plant.x0 = np.array([2.0, 0.0])
        diagram = plotting.get_controller(result) @ plant
        trajectory = diagram.compute_trajectory(tf=8.0, verbose=False)
        _, ax = plotting.plot_policy(result, trajectory=trajectory, show=False)
        self.assertGreaterEqual(len(ax.lines), 1)

    def test_lookup_controller_matmul_wires_state_feedback(self):
        problem = make_problem()
        planner, _ = solve(problem)
        diagram = planner.get_controller() @ problem.sys
        self.assertEqual(diagram.connections["ctl"]["x"], ("sys", "x"))
        self.assertEqual(diagram.connections["sys"]["u"], ("ctl", "u"))
        self.assertNotIn("r", diagram.inputs)

    def test_lookup_controller_plot_control_law_smoke(self):
        import matplotlib

        matplotlib.use("Agg")
        problem = make_problem()
        planner, _ = solve(problem)
        controller = planner.get_controller()
        res = controller.plot_control_law(grid_shape=(9, 9), show=False)
        self.assertIsNotNone(res.figure)

    def test_policy_evaluator_accepts_declared_block(self):
        problem = make_problem()
        planner, result = solve(problem)
        controller = planner.get_controller()
        options = planner.options
        J_callable = PolicyEvaluator(
            problem, grid=result.grid, policy=controller.action, options=options
        ).solve()
        J_block = PolicyEvaluator(
            problem, grid=result.grid, policy=controller, options=options
        ).solve()
        np.testing.assert_allclose(J_block, J_callable)

    def test_policy_evaluator_rejects_undeclared_block(self):
        from minilink.blocks.basic import Integrator

        problem = make_problem()
        _, result = solve(problem)
        with self.assertRaisesRegex(ValueError, "feedback declaration"):
            PolicyEvaluator(problem, grid=result.grid, policy=Integrator())


class TestFunctionApproximation(unittest.TestCase):
    def test_quadratic_features_recover_an_exact_quadratic(self):
        xbar = np.array([1.0, -2.0])
        S = np.array([[2.0, 0.5], [0.5, 3.0]])
        b = np.array([0.3, -0.7])
        rng = np.random.default_rng(0)
        X = rng.uniform(-3.0, 3.0, size=(60, 2))
        dx = X - xbar
        y = 4.0 + dx @ b + np.einsum("ij,jk,ik->i", dx, S, dx)

        features = QuadraticFeatures(xbar)
        approx = LinearApproximator(features)
        w = approx.fit(X, y)

        c_hat, b_hat, S_hat = features.quadratic_form(w)
        self.assertAlmostEqual(c_hat, 4.0)
        np.testing.assert_allclose(b_hat, b, atol=1e-9)
        np.testing.assert_allclose(S_hat, S, atol=1e-9)
        np.testing.assert_allclose(approx(X), y, atol=1e-9)
        self.assertAlmostEqual(approx(X[0]), y[0])

    def test_radial_bases_on_a_grid_and_concatenation(self):
        rbf = RadialBasisFeatures.on_grid([-1.0, -1.0], [1.0, 1.0], (3, 5))
        self.assertEqual(rbf.n_features, 15)
        self.assertAlmostEqual(rbf.sigma, 1.0)  # largest mesh spacing
        self.assertAlmostEqual(rbf.phi(rbf.centers[4])[4], 1.0)
        both = QuadraticFeatures(np.zeros(2)) + rbf
        self.assertEqual(both.n_features, 6 + 15)
        self.assertEqual(both.matrix(rbf.centers).shape, (15, 21))

    def test_sgd_steps_move_toward_the_least_squares_weights(self):
        rng = np.random.default_rng(1)
        X = rng.uniform(-1.0, 1.0, size=(200, 1))
        y = 1.0 + 2.0 * X[:, 0]
        features = QuadraticFeatures(np.zeros(1))
        target = LinearApproximator(features).fit(X, y)

        online = LinearApproximator(features)
        for _ in range(20):
            for x_i, y_i in zip(X, y):
                online.sgd_step(x_i, y_i, eta=0.1)
        np.testing.assert_allclose(online.w, target, atol=1e-3)


class TestPolicyEvaluatorPlots(unittest.TestCase):
    def test_value_at_and_plots_after_solve(self):
        import matplotlib

        matplotlib.use("Agg")
        problem = make_problem()
        planner, result = solve(problem)
        evaluator = PolicyEvaluator(
            problem,
            grid=result.grid,
            policy=planner.get_controller(),
            options=planner.options,
        )
        with self.assertRaisesRegex(ValueError, "solve"):
            evaluator.value_at(np.zeros(2))
        evaluator.solve()
        self.assertAlmostEqual(
            evaluator.value_at(np.zeros(2)),
            float(result.grid.interpolate(evaluator.last_J, np.zeros((1, 2)))[0]),
        )
        fig, _ = evaluator.plot_cost2go(vmax=10.0, show=False)
        self.assertIsNotNone(fig)
        fig, _ = result.grid.plot_value(evaluator.last_J, show_3d=True, show=False)
        self.assertIsNotNone(fig)


class TestDpOneObjectSetup(unittest.TestCase):
    """DynamicProgrammingPlanner builds its grid from x_grid / u_grid / dt."""

    def _problem(self):
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        plant = Pendulum()
        goal = np.array([np.pi, 0.0])
        return PlanningProblem(
            plant,
            x_goal=goal,
            cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=goal),
        )

    def test_shapes_and_dt_build_the_grid(self):
        from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner

        planner = DynamicProgrammingPlanner(
            self._problem(), x_grid=(11, 11), u_grid=(3,), dt=0.05
        )
        self.assertEqual(planner.grid.x_grid_shape, (11, 11))
        self.assertEqual(planner.grid.u_grid_shape, (3,))
        self.assertAlmostEqual(planner.grid.dt, 0.05)

    def test_success_reports_convergence(self):
        from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner

        problem = make_problem()
        capped = DynamicProgrammingPlanner(
            problem, x_grid=(11, 11), u_grid=(3,), dt=0.05, max_iterations=2
        ).solve()
        self.assertFalse(capped.success)
        self.assertIn("max_iterations", str(capped.solver))
        self.assertEqual(capped.solver.iterations, 2)

        planner = DynamicProgrammingPlanner(
            problem, x_grid=(11, 11), u_grid=(3,), dt=0.05, tol=1.0, max_iterations=500
        )
        converged = planner.solve()
        self.assertTrue(converged.success)
        self.assertIn("converged", str(converged.solver))
        self.assertLessEqual(converged.solver.delta, 1.0)
        # the solution is the pair: the greedy lookup law and the interpolated cost-to-go
        self.assertIs(converged.policy, planner.get_controller())
        self.assertEqual(
            converged.cost_to_go(problem.x_start), planner.value_at(problem.x_start)
        )
        self.assertIsNone(converged.trajectory)  # not rolled out unless asked

        fixed = DynamicProgrammingPlanner(
            problem, x_grid=(11, 11), u_grid=(3,), dt=0.05
        ).solve_steps(3, evaluate=True)
        self.assertTrue(fixed.success)
        self.assertEqual(fixed.solver.iterations, 3)
        self.assertEqual(
            fixed.evaluation.n_trials, 1
        )  # a deterministic problem: one trial
        np.testing.assert_allclose(fixed.trajectory.x[:, 0], problem.x_start)
        self.assertAlmostEqual(fixed.trajectory.t[1] - fixed.trajectory.t[0], 0.05)

    def test_final_time_reads_the_problem_horizon(self):
        from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner

        sys = DoubleIntegrator()
        cost = QuadraticCost.from_system(sys, xbar=np.zeros(2))
        timed = PlanningProblem(sys, x_goal=np.zeros(2), cost=cost, tf=2.0)
        planner = DynamicProgrammingPlanner(timed, x_grid=(5, 5), u_grid=(3,), dt=0.1)
        self.assertEqual(planner.options.final_time, 2.0)
        explicit = DynamicProgrammingPlanner(
            timed, x_grid=(5, 5), u_grid=(3,), dt=0.1, final_time=0.5
        )
        self.assertEqual(explicit.options.final_time, 0.5)
        untimed = DynamicProgrammingPlanner(
            make_problem(), x_grid=(5, 5), u_grid=(3,), dt=0.1
        )
        self.assertEqual(untimed.options.final_time, 0.0)

    def test_grid_and_shapes_are_exclusive(self):
        from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
        from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner

        problem = self._problem()
        grid = StateSpaceGrid(
            problem, x_grid_shape=(11, 11), u_grid_shape=(3,), dt=0.05
        )
        with self.assertRaises(ValueError):
            DynamicProgrammingPlanner(problem, grid=grid, dt=0.05)
        with self.assertRaises(ValueError):
            DynamicProgrammingPlanner(problem, x_grid=(11, 11))


class TestCleanInfeasible(unittest.TestCase):
    """clean_infeasible runs after each DP solve."""

    def test_dp_solve_cleans_infeasible_by_default(self):
        planner, result = solve(make_problem())
        penalty = planner.options.out_of_bound_cost
        self.assertTrue(planner.options.clean_infeasible)
        saturated = result.J > penalty - 1.0
        self.assertTrue(np.all(result.J[saturated] == penalty))

    def test_dp_clean_infeasible_can_be_disabled(self):
        from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner

        problem = make_problem()
        planner = DynamicProgrammingPlanner(
            problem,
            x_grid=(11, 11),
            u_grid=(3,),
            dt=0.05,
            max_iterations=5,
            clean_infeasible=False,
            verbose=False,
        )
        self.assertFalse(planner.options.clean_infeasible)
        from unittest import mock

        with mock.patch.object(
            DynamicProgrammingPlanner, "clean_infeasible_set"
        ) as cleanup:
            planner.solve()
        cleanup.assert_not_called()
        planner.options.clean_infeasible = True
        with mock.patch.object(
            DynamicProgrammingPlanner, "clean_infeasible_set"
        ) as cleanup:
            planner.solve()
        cleanup.assert_called_once()


def exit_price(x, t):
    """A price of leaving that grows with the exit state: 200 + 50 |x|^2."""
    return 200.0 + 50.0 * (x[0] ** 2 + x[1] ** 2)


class TestInfeasibleCostPrice(unittest.TestCase):
    """The problem's infeasible_cost(x, t) prices each inadmissible pair at its successor."""

    def planner(self, price, backend="numpy", **kwargs):
        from dataclasses import replace

        problem = replace(make_pendulum_problem(), infeasible_cost=price)
        return DynamicProgrammingPlanner(
            problem,
            x_grid=(11, 11),
            u_grid=(5,),
            dt=0.1,
            backend=backend,
            alpha=0.95,
            tol=1e-3,
            max_iterations=200,
            **kwargs,
        )

    def test_numpy_table_charges_the_price_of_each_successor(self):
        planner = self.planner(exit_price)
        self.assertIs(planner.options.out_of_bound_cost, exit_price)
        x_next, action_ok, x_next_ok = planner.grid.transition(0.0)
        inadmissible = ~(action_ok & x_next_ok)
        self.assertTrue(np.any(inadmissible))

        G = planner.running_cost_table(0.0)
        expected = [exit_price(x, 0.0) for x in x_next[inadmissible]]
        np.testing.assert_allclose(G[inadmissible], expected)

        # A scalar price (an int like a float) keeps the scalar table
        scalar = self.planner(500)
        self.assertEqual(scalar.options.out_of_bound_cost, 500.0)
        G_scalar = scalar.running_cost_table(0.0)
        np.testing.assert_array_equal(G_scalar[~inadmissible], G[~inadmissible])
        self.assertTrue(np.all(G_scalar[inadmissible] == 500.0))

    def test_a_constant_price_solves_like_the_scalar(self):
        for backend in ("numpy", "loop"):
            with self.subTest(backend=backend):
                scalar = self.planner(500.0, backend, clean_infeasible=False)
                constant = self.planner(
                    lambda x, t: 500.0, backend, clean_infeasible=False
                )
                scalar.solve_steps(30)
                constant.solve_steps(30)
                np.testing.assert_array_equal(constant.result.J, scalar.result.J)
                np.testing.assert_array_equal(constant.result.pi, scalar.result.pi)

    def test_loop_numpy_and_policy_evaluation_agree_on_the_price(self):
        table = self.planner(exit_price)
        loop = self.planner(exit_price, "loop")
        table.solve()
        loop.solve_steps(table.result.iterations)
        np.testing.assert_allclose(loop.result.J, table.result.J)
        # the priced exits, not the 1e6 default, bound the cost-to-go
        self.assertLess(np.max(table.result.J), 1.0e3)

        evaluator = PolicyEvaluator(
            table.problem,
            grid=table.grid,
            policy=table.get_controller().action,
            options=table.options,
        )
        J_pi = evaluator.solve()
        self.assertLess(np.max(np.abs(J_pi - table.result.J)), 0.05)

    def test_jax_table_charges_the_price(self):
        import pytest

        pytest.importorskip("jax")
        numpy_planner = self.planner(exit_price)
        jax_planner = self.planner(exit_price, "jax")
        G_np = numpy_planner.running_cost_table(0.0)
        G_jax = dp_jax.running_cost_table(jax_planner, 0.0)
        np.testing.assert_allclose(G_jax, G_np)

        numpy_planner.solve()
        jax_planner.solve()
        np.testing.assert_allclose(jax_planner.result.J, numpy_planner.result.J)
        self.assertLess(np.max(jax_planner.result.J), 1.0e3)

    def test_jax_rejects_an_untraceable_price(self):
        import pytest

        pytest.importorskip("jax")
        planner = self.planner(lambda x, t: 100.0 + float(x[0] ** 2), "jax")
        with self.assertRaisesRegex(ValueError, "JAX-traceable"):
            planner.solve()


class TestDpFlatConstructor(unittest.TestCase):
    def test_flat_matches_options(self):
        sys = DoubleIntegrator()
        problem = PlanningProblem(
            sys=sys, x_start=np.array([0.0, 0.0]), cost=QuadraticCost.from_system(sys)
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


def test_grid_exposes_the_boxes_it_spans():
    from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid

    class DoubleIntegrator(DynamicSystem):
        def __init__(self):
            super().__init__(n=2, input_dim=1, output_dim=2)
            self.state.lower_bound = np.array([-2.0, -3.0])
            self.state.upper_bound = np.array([2.0, 3.0])
            self.inputs["u"].lower_bound = np.array([-1.0])
            self.inputs["u"].upper_bound = np.array([1.0])

        def f(self, x, u, t=0, params=None):
            return np.array([x[1], u[0]])

    problem = PlanningProblem(DoubleIntegrator(), x_goal=np.zeros(2))
    grid = StateSpaceGrid(problem, x_grid_shape=(5, 5), u_grid_shape=(3,), dt=0.1)
    np.testing.assert_allclose(grid.X.lower, [-2.0, -3.0])
    np.testing.assert_allclose(grid.X.upper, [2.0, 3.0])
    np.testing.assert_allclose(grid.U.box.lower, [-1.0])
    assert all(grid.X.contains(x) for x in grid.states)
    assert all(grid.U.contains(u) for u in grid.inputs)
