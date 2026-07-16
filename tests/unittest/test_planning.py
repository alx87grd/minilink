import contextlib
import io
import tempfile
import unittest

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.sets import BallSet, BoxSet, SingletonSet
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.planning.initial_guess import (
    default_initial_trajectory,
    mechanical_cubic_initial_trajectory,
)
from minilink.planning.problems import PlanningProblem, ProblemParameters
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.live_plot import (
    LiveTrajectoryPlotCallback,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingOptions,
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationIteration,
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.shooting import (
    ShootingOptions,
    ShootingTranscription,
)


class TestPlanningArchitecture(unittest.TestCase):
    def make_system(self):
        class TestPlant(DynamicSystem):
            def __init__(self):
                super().__init__(n=2, input_dim=1, output_dim=2, y_dependencies=())
                self.state.lower_bound = np.array([-1.0, -2.0])
                self.state.upper_bound = np.array([1.0, 2.0])
                self.inputs["u"].lower_bound = np.array([-3.0])
                self.inputs["u"].upper_bound = np.array([4.0])
                self.x0 = np.array([0.1, -0.2])

            def f(self, x, u, t=0, params=None):
                return np.zeros(2)

            def h(self, x, u, t=0, params=None):
                return x

        return TestPlant()

    def make_single_integrator(self):
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

        return SingleIntegrator()

    def make_single_integrator_problem(self):
        sys = self.make_single_integrator()
        cost = QuadraticCost.from_system(
            sys,
            Q=np.zeros((1, 1)),
            R=np.eye(1),
            S=np.zeros((1, 1)),
        )
        return PlanningProblem(
            sys=sys,
            tf=1.0,
            x_start=np.array([0.0]),
            x_goal=np.array([1.0]),
            cost=cost,
        )

    def test_box_and_boundary_sets(self):
        box = BoxSet(lower=np.array([-1.0, -2.0]), upper=np.array([1.0, 2.0]))
        self.assertTrue(box.contains(np.array([0.0, 0.0])))
        self.assertFalse(box.contains(np.array([2.0, 0.0])))
        np.testing.assert_allclose(
            box.margin(np.array([0.0, 0.0])),
            np.array([1.0, 2.0, 1.0, 2.0]),
        )

        singleton = SingletonSet(np.array([1.0, 2.0]))
        np.testing.assert_allclose(
            singleton.residual(np.array([1.5, 1.0])), [0.5, -1.0]
        )
        self.assertTrue(singleton.contains(np.array([1.0, 2.0])))

        ball = BallSet(center=np.zeros(2), radius=1.0)
        self.assertTrue(ball.contains(np.array([0.5, 0.0])))
        self.assertFalse(ball.contains(np.array([2.0, 0.0])))

    def test_planning_problem_defaults(self):
        sys = self.make_system()
        problem = PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]))

        np.testing.assert_allclose(problem.x_start, sys.x0)
        self.assertIsNone(problem.tf)
        self.assertTrue(problem.X.contains(np.array([0.0, 0.0])))
        self.assertTrue(problem.U.contains(np.array([0.0])))
        self.assertFalse(problem.U.contains(np.array([10.0])))
        self.assertTrue(problem.has_goal)
        self.assertFalse(problem.has_cost)

    def test_planning_problem_tf_validation_and_grid(self):
        sys = self.make_system()
        with self.assertRaisesRegex(ValueError, "tf must be positive"):
            PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]), tf=0.0)
        infinite = PlanningProblem(
            sys=sys, x_goal=np.array([0.0, 0.0]), tf=float("inf")
        )
        self.assertTrue(np.isposinf(infinite.tf))
        with self.assertRaisesRegex(ValueError, "finite problem.tf"):
            infinite.require_finite_tf()
        problem = PlanningProblem(
            sys=sys,
            x_goal=np.array([0.0, 0.0]),
            cost=QuadraticCost.from_system(sys),
            tf=2.0,
        )
        options = DirectCollocationOptions(n_steps=5)
        np.testing.assert_allclose(options.t(problem), np.linspace(0.0, 2.0, 5))
        self.assertAlmostEqual(options.dt(problem), 0.5)
        self.assertNotIn("tf", options.__dataclass_fields__)
        unset = PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]))
        with self.assertRaisesRegex(ValueError, "finite problem.tf"):
            options.t(unset)

    def test_planning_problem_open_boundary_sets_use_representatives(self):
        sys = self.make_system()
        X0 = BallSet(center=np.zeros(2), radius=0.5)
        Xf = BallSet(center=np.array([0.5, 0.0]), radius=0.25)
        x_start = np.array([0.1, -0.1])
        x_goal = np.array([0.5, 0.0])

        problem = PlanningProblem(
            sys=sys,
            x_start=x_start,
            x_goal=x_goal,
            X0=X0,
            Xf=Xf,
        )

        self.assertIs(problem.X0, X0)
        self.assertIs(problem.Xf, Xf)
        np.testing.assert_allclose(problem.x_start, x_start)
        np.testing.assert_allclose(problem.x_goal, x_goal)

    def test_planning_problem_derives_representatives_from_singleton_sets(self):
        sys = self.make_system()
        x_start = np.array([0.25, -0.5])
        x_goal = np.array([0.75, 0.5])

        problem = PlanningProblem(
            sys=sys,
            X0=SingletonSet(x_start),
            Xf=SingletonSet(x_goal),
        )

        np.testing.assert_allclose(problem.x_start, x_start)
        np.testing.assert_allclose(problem.x_goal, x_goal)

    def test_planning_problem_rejects_representatives_outside_boundary_sets(self):
        sys = self.make_system()

        with self.assertRaisesRegex(ValueError, "x_start must belong to X0"):
            PlanningProblem(
                sys=sys,
                x_start=np.array([1.0, 0.0]),
                X0=BallSet(center=np.zeros(2), radius=0.25),
            )

        with self.assertRaisesRegex(ValueError, "x_goal must belong to Xf"):
            PlanningProblem(
                sys=sys,
                x_goal=np.array([1.0, 0.0]),
                Xf=BallSet(center=np.zeros(2), radius=0.25),
            )

    def test_planning_problem_boundary_types_are_validated(self):
        sys = self.make_system()
        params = ProblemParameters(
            system={"mass": 1.0},
            cost={"weight": 2.0},
            sets={"radius": 0.25},
        )
        problem = PlanningProblem(
            sys=sys,
            x_goal=np.array([0.0, 0.0]),
            params=params,
            metadata={"tag": "demo"},
        )

        self.assertIs(problem.params, params)
        self.assertEqual(problem.metadata["tag"], "demo")
        with self.assertRaises(TypeError):
            problem.metadata["tag"] = "changed"
        with self.assertRaisesRegex(TypeError, "params must be"):
            PlanningProblem(sys=sys, params={"system": {}})
        with self.assertRaisesRegex(TypeError, "metadata must be"):
            PlanningProblem(sys=sys, metadata=[("tag", "demo")])

    def test_quadratic_cost_evaluates_trajectory(self):
        sys = self.make_system()
        cost = QuadraticCost.from_system(sys)
        traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0], [0.0, 0.0]]),
            u=np.array([[0.0, 0.0]]),
        )

        evaluated = cost.evaluate_trajectory(traj)
        self.assertTrue(evaluated.has_signal("cost_rate"))
        self.assertTrue(evaluated.has_signal("cost"))
        self.assertGreaterEqual(cost.total_cost(traj), 0.0)

    def test_trajectory_save_load_roundtrip(self):
        traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
            signals={"dx": np.array([[1.0, 1.0]])},
        )

        with tempfile.TemporaryDirectory() as tmp:
            path = f"{tmp}/traj.npz"
            traj.save(path)
            loaded = Trajectory.load(path)

        np.testing.assert_allclose(loaded.t, traj.t)
        np.testing.assert_allclose(loaded.x, traj.x)
        np.testing.assert_allclose(loaded.u, traj.u)
        np.testing.assert_allclose(loaded.signals["dx"], traj.signals["dx"])

    def test_mechanical_initial_guess_respects_boundary_state(self):
        sys = MechanicalSystem(dof=1)
        x_start = np.array([0.0, 0.5])
        x_goal = np.array([1.0, -0.25])
        problem = PlanningProblem(sys=sys, x_start=x_start, x_goal=x_goal)

        guess = mechanical_cubic_initial_trajectory(problem, np.linspace(0.0, 2.0, 5))

        np.testing.assert_allclose(guess.x[:, 0], x_start)
        np.testing.assert_allclose(guess.x[:, -1], x_goal)

    def test_planner_require_trajectory_plan_before_solve(self):
        sys = self.make_system()
        cost = QuadraticCost.from_system(sys)
        problem = PlanningProblem(
            sys=sys, x_goal=np.array([0.0, 0.0]), cost=cost, tf=1.0
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
        )
        with self.assertRaises(ValueError):
            planner.require_trajectory_plan()

    def test_mathematical_program_constraints_are_backend_neutral(self):
        program = MathematicalProgram(
            n_z=2,
            J=lambda z: z.T @ z,
            h=lambda z: np.array([z[0] + z[1] - 1.0]),
            g=lambda z: np.array([z[0], z[1]]),
            lower=np.zeros(2),
            upper=np.ones(2),
        )
        program_evaluator = compile_program_evaluator(
            program,
            sample_z=np.array([0.5, 0.5]),
        )

        self.assertEqual(program.n_z, 2)
        self.assertEqual(program_evaluator.objective(np.array([1.0, 2.0])), 5.0)
        np.testing.assert_allclose(
            program_evaluator.equality_residual(np.array([0.25, 0.75])),
            [0.0],
        )
        np.testing.assert_allclose(
            program_evaluator.inequality_margin(np.array([0.25, 0.75])),
            [0.25, 0.75],
        )

        with self.assertRaises(ValueError):
            MathematicalProgram(
                n_z=2,
                J=lambda z: z @ z,
                lower=np.zeros(3),
            )

    def test_solver_skeletons_validate_architecture_inputs(self):
        sys = self.make_system()
        cost = QuadraticCost.from_system(sys)
        problem = PlanningProblem(
            sys=sys, x_goal=np.array([0.0, 0.0]), cost=cost, tf=1.0
        )

        to = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(compile_backend="numpy"),
        )
        self.assertEqual(to.transcription.options.n_steps, 5)
        self.assertEqual(to.options.compile_backend, "numpy")
        guess = default_initial_trajectory(
            problem,
            to.transcription.initial_guess_time_grid(problem),
        )
        program = to.transcription.transcribe(
            problem,
            compile_backend=to.options.compile_backend,
        )
        z0 = to.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(program, sample_z=z0)
        self.assertEqual(program.n_z, 15)
        self.assertEqual(program_evaluator.n_h, 12)

    def test_direct_collocation_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        traj = planner.solve().trajectory
        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)
        residual_norm = np.linalg.norm(
            planner.last_optimizer.program_evaluator.equality_residual(
                planner.last_optimization_result.z
            )
        )
        self.assertLess(residual_norm, 1e-6)
        self.assertTrue(traj.has_signal("dx"))
        self.assertTrue(traj.has_signal("cost"))

    def test_trajopt_solve_disp_prints_planning_report(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
                solve_disp=True,
            ),
        )
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            planner.solve()

        report = stdout.getvalue()
        self.assertIn("Trajectory Optimization Program", report)
        self.assertIn("transcription: DirectCollocationTranscription", report)
        self.assertIn("method='scipy_slsqp'", report)
        self.assertNotIn("===               Optimization Program", report)
        self.assertIn("terminal_error_inf:", report)

    def test_generic_trajopt_planner_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="direct",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
                record_history=True,
            ),
        )

        traj = planner.solve().trajectory
        warm_started = planner.solve(warm_start=True).trajectory
        self.assertTrue(planner.last_optimization_result.success)
        self.assertEqual(planner.last_program.metadata["compile_backend"], "direct")
        self.assertGreater(len(planner.iteration_history), 0)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)
        np.testing.assert_allclose(warm_started.x[:, -1], [1.0], atol=1e-7)

    def test_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        transcription = ShootingTranscription(ShootingOptions(n_steps=5))
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        traj = planner.solve().trajectory
        self.assertTrue(planner.last_optimization_result.success)
        self.assertEqual(planner.last_program.metadata["compile_backend"], "numpy")
        self.assertEqual(planner.last_program.n_z, problem.sys.m * 5)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)
        self.assertTrue(traj.has_signal("dx"))
        self.assertTrue(traj.has_signal("cost"))

    def test_multiple_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        transcription = MultipleShootingTranscription(
            MultipleShootingOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        traj = planner.solve().trajectory
        self.assertTrue(planner.last_optimization_result.success)
        self.assertEqual(planner.last_program.metadata["compile_backend"], "numpy")
        self.assertEqual(planner.last_program.n_z, (problem.sys.n + problem.sys.m) * 5)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)
        self.assertTrue(traj.has_signal("dx"))
        self.assertTrue(traj.has_signal("cost"))

    def test_dynamics_function_routes_system_params_through_compiled_evaluator(self):
        """System params flow through the parametric tier f_p, not direct sys.f."""
        from minilink.blocks.basic import Integrator
        from minilink.planning.trajectory_optimization.transcription import (
            dynamics_function,
        )

        sys = Integrator()  # dx = k * u, default k = 1.0
        problem = PlanningProblem(
            sys=sys,
            x_goal=np.array([1.0]),
            cost=QuadraticCost.from_system(sys),
            params=ProblemParameters(system={"k": 3.0}),
        )

        x = np.array([0.0])
        u = np.array([2.0])
        for backend in ("numpy", "direct"):
            f = dynamics_function(problem, backend)
            np.testing.assert_allclose(f(x, u, 0.0), [6.0], atol=1e-12)

        # Without explicit params, the compiled evaluator uses block defaults.
        plain = PlanningProblem(
            sys=sys,
            x_goal=np.array([1.0]),
            cost=QuadraticCost.from_system(sys),
        )
        f = dynamics_function(plain, "numpy")
        np.testing.assert_allclose(f(x, u, 0.0), [2.0], atol=1e-12)

    def test_shooting_packs_trajectory_guess_as_inputs_only(self):
        problem = self.make_single_integrator_problem()
        transcription = ShootingTranscription(ShootingOptions(n_steps=5))
        guess = default_initial_trajectory(
            problem,
            transcription.initial_guess_time_grid(problem),
        )
        program = transcription.transcribe(
            problem,
            compile_backend="direct",
        )
        z0 = transcription.pack_initial_guess(problem, guess)

        self.assertEqual(program.n_z, 5)
        np.testing.assert_allclose(z0, guess.u.reshape(-1))
        self.assertEqual(program.metadata["compile_backend"], "direct")

    def test_evaluator_forced_rk4_rollout_matches_integrator(self):
        sys = self.make_single_integrator()
        evaluator = sys.compile(backend="numpy", verbose=False)

        x = evaluator.rk4_integrate_linear(
            np.array([0.0]),
            np.ones((5, 1)),
            0.0,
            0.25,
        )

        np.testing.assert_allclose(x.reshape(-1), np.linspace(0.0, 1.0, 5))

    def test_live_trajectory_plot_callback_reuses_artists(self):
        import matplotlib

        matplotlib.use("Agg")

        sys = self.make_single_integrator()
        traj0 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
        )
        traj1 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 0.9]]),
            u=np.array([[0.9, 0.9]]),
        )
        callback = LiveTrajectoryPlotCallback(sys, signals=("x", "u"), pause=0.0)

        callback(
            TrajectoryOptimizationIteration(
                iteration=0,
                z=np.zeros(1),
                trajectory=traj0,
                cost=1.0,
                max_eq=0.0,
                min_ineq=None,
            )
        )
        fig_id = id(callback.fig)
        line_ids = [id(line) for line in callback.lines]
        callback(
            TrajectoryOptimizationIteration(
                iteration=1,
                z=np.zeros(1),
                trajectory=traj1,
                cost=0.5,
                max_eq=0.0,
                min_ineq=None,
            )
        )

        self.assertEqual(id(callback.fig), fig_id)
        self.assertEqual([id(line) for line in callback.lines], line_ids)

    def test_trajopt_warm_start_passes_previous_trajectory(self):
        class RecordingTranscription(DirectCollocationTranscription):
            def __init__(self, options):
                super().__init__(options)
                self.guesses = []

            def pack_initial_guess(self, problem, guess):
                self.guesses.append(guess)
                return super().pack_initial_guess(problem, guess)

        problem = self.make_single_integrator_problem()
        transcription = RecordingTranscription(DirectCollocationOptions(n_steps=5))
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                warm_start=True,
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        first = planner.solve().trajectory
        planner.solve()

        self.assertIs(transcription.guesses[1], first)


if __name__ == "__main__":
    unittest.main()


# --- merged from test_planning_ui_constructors.py ---

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


# --- merged from test_rrt.py ---

import numpy as np
import pytest

from minilink.core.sets import BallSet, BoxSet
from minilink.dynamics.catalog.vehicles.steering import (
    KinematicBicycle,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.edge import Edge
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.metric import euclidean
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.rrt_star import RRTStarOptions, RRTStarPlanner
from minilink.planning.search.steering import DubinsSteering, StraightLineSteering
from minilink.planning.search.tree import NEAREST_KD_TREE, Node, Tree
from tests.unittest.planning_helpers import (
    X_GOAL,
    X_START,
    make_holonomic_obstacle_problem,
)

COMPASS = [
    np.array([np.cos(a), np.sin(a)])
    for a in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
]


# --- tree -----------------------------------------------------------------


def test_tree_nearest_and_near():
    tree = Tree(Node(np.array([0.0, 0.0]), None, None, 0.0))
    tree.add(Node(np.array([1.0, 0.0]), tree.root, None, 1.0))
    tree.add(Node(np.array([5.0, 0.0]), tree.root, None, 5.0))
    assert tree.nearest(np.array([0.9, 0.0]), euclidean).x[0] == pytest.approx(1.0)
    near = tree.near(np.array([0.0, 0.0]), 1.5, euclidean)
    assert len(near) == 2  # root and (1,0)


def test_kdtree_nearest_and_near_match_brute_force():
    rng = np.random.default_rng(0)
    brute = Tree(Node(np.array([0.0, 0.0]), None, None, 0.0))
    kd = Tree(
        Node(np.array([0.0, 0.0]), None, None, 0.0),
        nearest_backend=NEAREST_KD_TREE,
    )
    for _ in range(40):
        x = rng.uniform(-5.0, 5.0, size=2)
        node = Node(x, brute.root, None, float(np.linalg.norm(x)))
        brute.add(node)
        kd.add(Node(x, kd.root, None, float(np.linalg.norm(x))))

    for _ in range(30):
        query = rng.uniform(-5.0, 5.0, size=2)
        brute_nearest = brute.nearest(query, euclidean)
        kd_nearest = kd.nearest(query, euclidean)
        assert np.allclose(brute_nearest.x, kd_nearest.x)

        radius = float(rng.uniform(0.5, 3.0))
        brute_near = {
            tuple(np.asarray(node.x, dtype=float))
            for node in brute.near(query, radius, euclidean)
        }
        kd_near = {
            tuple(np.asarray(node.x, dtype=float))
            for node in kd.near(query, radius, euclidean)
        }
        assert brute_near == kd_near


def test_kinodynamic_reaches_goal_with_kdtree_backend():
    problem, X = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
        options=RRTOptions(
            seed=0, goal_tolerance=0.5, max_nodes=4000, nearest_backend="kd_tree"
        ),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_kdtree_requires_euclidean_metric():
    sys = make_dubins_problem()
    problem = PlanningProblem(
        sys=sys,
        x_start=np.array([-3.0, -3.0, 0.0]),
        x_goal=np.array([3.0, 3.0, np.pi / 2]),
        X=BoxSet.from_system_state(sys),
    )
    dubins = DubinsSteering(wheelbase=sys.params["length"], max_steering=0.5, speed=1.5)
    planner = RRTPlanner(
        problem,
        extender=SteeringExtender(dubins, max_distance=1.5, resolution=0.1),
        metric=dubins.distance,
        options=RRTOptions(seed=0, nearest_backend="kd_tree"),
    )
    with pytest.raises(ValueError, match="metric=euclidean"):
        planner.solve()


def test_unknown_nearest_backend_raises():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
        options=RRTOptions(seed=0, nearest_backend="invalid"),
    )
    with pytest.raises(ValueError, match="nearest_backend"):
        planner.solve()


# --- kinodynamic ----------------------------------------------------------


def test_kinodynamic_reaches_goal_and_stays_free():
    problem, X = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))
    assert traj.x.shape[0] == 2
    assert traj.u.shape == traj.x.shape  # (n, N), m == n here
    assert traj.t.shape[0] == traj.x.shape[1]


def test_kinodynamic_random_controls_reaches_goal():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=12, horizon=0.6, n_substeps=6),
        options=RRTOptions(seed=1, goal_tolerance=0.6, max_nodes=6000),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.6


def test_seeded_run_is_deterministic():
    problem, _ = make_holonomic_obstacle_problem()

    def run():
        return (
            RRTPlanner(
                problem,
                extender=KinodynamicExtender(
                    controls=COMPASS, horizon=0.6, n_substeps=6
                ),
                options=RRTOptions(seed=7, goal_tolerance=0.5, max_nodes=4000),
            )
            .solve()
            .trajectory
        )

    a, b = run(), run()
    assert a.x.shape == b.x.shape
    assert np.allclose(a.x, b.x)


# --- steering -------------------------------------------------------------


def test_steering_reaches_goal_and_stays_free():
    problem, X = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=SteeringExtender(
            StraightLineSteering(speed=1.0), max_distance=0.6, resolution=0.05
        ),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_steering_edge_is_dynamically_feasible():
    problem, _ = make_holonomic_obstacle_problem()
    evaluator = problem.sys.compile(backend="numpy", verbose=False)
    (edge,) = list(
        SteeringExtender(
            StraightLineSteering(1.0), max_distance=0.6, resolution=0.05
        ).propose(X_START, X_GOAL, problem, np.random.default_rng(0))
    )
    x = np.asarray(edge.states[0], dtype=float)
    for k in range(len(edge.inputs)):
        dt = float(edge.times[k + 1] - edge.times[k])
        x = np.asarray(evaluator.rk4_step(x, edge.inputs[k], 0.0, dt), dtype=float)
        assert np.allclose(x, edge.states[k + 1], atol=1e-6)


# --- dubins steering ------------------------------------------------------


def make_dubins_problem():
    sys = KinematicBicycle()  # state [x, y, theta], input [speed, steering]
    sys.state.lower_bound = np.array([-6.0, -6.0, -np.pi])
    sys.state.upper_bound = np.array([6.0, 6.0, np.pi])
    sys.inputs["u"].lower_bound = np.array([0.0, -0.5])  # forward only
    sys.inputs["u"].upper_bound = np.array([1.5, 0.5])
    return sys


def test_dubins_connect_reaches_pose_exactly_and_distance_is_finite():
    dubins = DubinsSteering(wheelbase=1.0, max_steering=0.5, speed=1.0)
    x0 = np.array([0.0, 0.0, 0.0])
    x1 = np.array([2.0, 1.5, np.pi / 2])

    states, inputs, times, cost = dubins.connect(
        x0, x1, max_distance=1e3, resolution=0.05
    )
    assert np.allclose(states[-1, :2], x1[:2], atol=1e-9)  # exact xy
    assert times.shape[0] == states.shape[0] == inputs.shape[0] + 1
    assert cost == pytest.approx(dubins.distance(x0, x1))  # arc length == metric


def test_dubins_edge_is_dynamically_feasible():
    sys = make_dubins_problem()
    evaluator = sys.compile(backend="numpy", verbose=False)
    dubins = DubinsSteering(wheelbase=sys.params["length"], max_steering=0.5, speed=1.0)

    states, inputs, times, _ = dubins.connect(
        np.array([0.0, 0.0, 0.3]),
        np.array([3.0, -1.0, -0.6]),
        max_distance=1e3,
        resolution=0.1,
    )
    x = np.asarray(states[0], dtype=float)
    for k in range(len(inputs)):
        dt = float(times[k + 1] - times[k])
        x = np.asarray(evaluator.rk4_step(x, inputs[k], 0.0, dt), dtype=float)
        assert np.allclose(x, states[k + 1], atol=1e-6)  # analytic arc == bicycle f


def test_dubins_rrt_reaches_goal_pose():
    sys = make_dubins_problem()
    x_start = np.array([-3.0, -3.0, 0.0])
    x_goal = np.array([3.0, 3.0, np.pi / 2])
    X = BoxSet.from_system_state(sys)
    problem = PlanningProblem(
        sys=sys, x_start=x_start, x_goal=x_goal, X=X, Xf=BallSet(x_goal, 0.6)
    )
    dubins = DubinsSteering(wheelbase=sys.params["length"], max_steering=0.5, speed=1.5)

    planner = RRTPlanner(
        problem,
        extender=SteeringExtender(dubins, max_distance=1.5, resolution=0.1),
        metric=dubins.distance,
        options=RRTOptions(seed=0, goal_bias=0.2, max_nodes=8000),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - x_goal) < 0.6
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


# --- selection ------------------------------------------------------------


def test_select_rejects_colliding_candidate_for_free_one():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(seed=0),
    )
    x_rand = np.array([4.0, 4.0])
    # closer to x_rand but its endpoint sits inside the obstacle
    colliding = Edge(
        states=np.array([[-0.1, -0.1], [0.0, 0.0]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 0.1]),
        cost=0.1,
    )
    # farther but collision-free
    free = Edge(
        states=np.array([[-4.0, -4.0], [-3.5, -3.5]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 0.1]),
        cost=0.1,
    )
    assert planner._select([colliding, free], x_rand) is free


def test_reached_goal_false_on_budget_exhaustion():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
        options=RRTOptions(seed=0, max_nodes=10, goal_bias=0.0),
    )
    planner.solve()
    assert not planner.reached_goal
    assert planner.solution_node is not None


def test_return_best_effort_false_raises():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
        options=RRTOptions(
            seed=0, max_nodes=5, goal_bias=0.0, return_best_effort=False
        ),
    )
    with pytest.raises(RuntimeError, match="failed to reach goal"):
        planner.solve()


def test_free_state_sampling_stays_in_X():
    problem, X = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(goal_bias=0.0),
    )
    rng = np.random.default_rng(42)
    samples = [planner._sample_free_state(rng) for _ in range(40)]
    assert all(X.contains(sample) for sample in samples)
    assert all(np.linalg.norm(sample) > 1.0 for sample in samples)


def test_edge_resolution_rejects_segment_through_obstacle():
    problem, _ = make_holonomic_obstacle_problem()
    planner_fine = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(edge_resolution=0.05),
    )
    planner_coarse = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(edge_resolution=None),
    )
    edge = Edge(
        states=np.array([[-2.0, 0.0], [2.0, 0.0]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 4.0]),
        cost=4.0,
    )
    assert not planner_fine._edge_is_free(edge)
    assert planner_coarse._edge_is_free(edge)


def test_tree_rewire_and_propagate_cost():
    edge_ab = Edge(
        states=np.array([[0.0, 0.0], [1.0, 0.0]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 1.0]),
        cost=1.0,
    )
    edge_bc = Edge(
        states=np.array([[1.0, 0.0], [2.0, 0.0]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 1.0]),
        cost=1.0,
    )
    edge_rc = Edge(
        states=np.array([[0.0, 1.0], [2.0, 0.0]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 2.0]),
        cost=2.0,
    )
    tree = Tree(Node(np.array([0.0, 0.0]), None, None, 0.0))
    a = tree.add(Node(np.array([1.0, 0.0]), tree.root, edge_ab, 1.0))
    b = tree.add(Node(np.array([2.0, 0.0]), a, edge_bc, 2.0))
    assert any(child is b for child in a.children)

    tree.rewire(b, tree.root, edge_rc)
    assert not any(child is b for child in a.children)
    assert any(child is b for child in tree.root.children)
    assert b.cost == pytest.approx(2.0)
    assert a.children == []


# --- built-in visualization -----------------------------------------------


def test_plot_tree_and_animate_search_smoke():
    mpl = pytest.importorskip("matplotlib")
    mpl.use("Agg")
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=2000),
    )
    planner.solve()

    fig, ax = planner.plot_tree(x_axis=0, y_axis=1, show=False)
    assert fig is not None and ax is not None
    anim = planner.animate_search(x_axis=0, y_axis=1, step=20, show=False)
    assert anim is not None


# --- RRT* -----------------------------------------------------------------


def make_steering_extender():
    return SteeringExtender(
        StraightLineSteering(speed=1.0), max_distance=0.6, resolution=0.05
    )


def path_cost(planner) -> float:
    node = planner.solution_node
    if node is None:
        return float("inf")
    return float(node.cost)


def test_rrt_star_reaches_goal():
    problem, X = make_holonomic_obstacle_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_rrt_star_reaches_goal_with_kdtree_backend():
    problem, X = make_holonomic_obstacle_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=0, goal_tolerance=0.5, max_nodes=4000, nearest_backend="kd_tree"
        ),
    )
    traj = planner.solve().trajectory
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_rrt_star_improves_path_cost_over_rrt():
    problem, _ = make_holonomic_obstacle_problem()
    extender = make_steering_extender()
    star_costs = []
    rrt_costs = []
    for seed in range(12):
        options = RRTStarOptions(seed=seed, goal_tolerance=0.5, max_nodes=3000)
        rrt = RRTPlanner(problem, extender=extender, options=options)
        rrt.solve()
        star = RRTStarPlanner(problem, extender=extender, options=options)
        star.solve()
        if rrt.reached_goal and star.reached_goal:
            rrt_costs.append(path_cost(rrt))
            star_costs.append(path_cost(star))

    assert len(star_costs) >= 8
    assert np.mean(star_costs) <= np.mean(rrt_costs)
    assert sum(s <= r for s, r in zip(star_costs, rrt_costs)) >= 4


def test_rewire_false_is_at_least_as_costly():
    problem, _ = make_holonomic_obstacle_problem()
    extender = make_steering_extender()
    with_rewire = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(seed=5, goal_tolerance=0.5, max_nodes=2500, rewire=True),
    )
    without_rewire = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(
            seed=5, goal_tolerance=0.5, max_nodes=2500, rewire=False
        ),
    )
    with_rewire.solve()
    without_rewire.solve()
    assert with_rewire.reached_goal
    assert without_rewire.reached_goal
    assert path_cost(with_rewire) <= path_cost(without_rewire) + 1e-9


def test_rrt_star_is_deterministic():
    problem, _ = make_holonomic_obstacle_problem()
    extender = make_steering_extender()
    options = RRTStarOptions(seed=11, goal_tolerance=0.5, max_nodes=2000)

    def run():
        planner = RRTStarPlanner(problem, extender=extender, options=options)
        traj = planner.solve().trajectory
        return traj, path_cost(planner)

    (traj_a, cost_a), (traj_b, cost_b) = run(), run()
    assert np.allclose(traj_a.x, traj_b.x)
    assert cost_a == pytest.approx(cost_b)


def test_rrt_star_infers_rewire_eta_from_extender():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(seed=0, max_nodes=10),
    )
    assert planner._rewire_eta() == pytest.approx(0.6)


def test_rrt_star_requires_rewire_eta_for_unknown_extender():
    problem, _ = make_holonomic_obstacle_problem()

    class DummyExtender:
        def propose(self, *args, **kwargs):
            return []

    planner = RRTStarPlanner(
        problem,
        extender=DummyExtender(),
        options=RRTStarOptions(seed=0),
    )
    with pytest.raises(ValueError, match="rewire_eta"):
        planner._rewire_eta()


def test_search_callback_invoked_on_rrt():
    calls = []

    class RecordingCallback:
        def __call__(self, step):
            calls.append(step)

    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=0,
            goal_tolerance=0.5,
            max_nodes=200,
            callback=RecordingCallback(),
            live_plot_every=1,
        ),
    )
    planner.solve()
    assert calls
    assert all(step.iteration > 0 for step in calls)
    assert calls[-1].phase == "explore"


def test_live_plot_after_goal_only_skips_explore_phase():
    calls = []

    class RecordingCallback:
        def __call__(self, step):
            calls.append(step.phase)

    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=1,
            goal_tolerance=0.5,
            max_nodes=2500,
            optimize_after_goal=True,
            convergence_patience=100,
            callback=RecordingCallback(),
            live_plot_every=1,
            live_plot_after_goal_only=True,
        ),
    )
    planner.solve()
    assert planner.reached_goal
    assert calls
    assert all(phase == "optimize" for phase in calls)


def test_live_plot_option_builds_callback():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTOptions(
            seed=0,
            max_nodes=10,
            live_plot=True,
            live_plot_every=1,
        ),
    )
    callback = planner._resolve_search_callback()
    from minilink.planning.search.live_plot import LiveSearchPlotCallback

    assert isinstance(callback, LiveSearchPlotCallback)


def test_optimize_after_goal_runs_longer_and_refines_cost():
    problem, _ = make_holonomic_obstacle_problem()
    extender = make_steering_extender()
    base = dict(seed=4, goal_tolerance=0.5, max_nodes=3500, goal_bias=0.05)

    first_hit = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(**base, optimize_after_goal=False),
    )
    optimized = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(
            **base,
            optimize_after_goal=True,
            cost_tol=0.05,
            convergence_patience=400,
        ),
    )
    first_hit.solve()
    optimized.solve()

    assert first_hit.reached_goal
    assert optimized.reached_goal
    assert optimized.iterations > first_hit.iterations
    assert path_cost(optimized) <= path_cost(first_hit) + 1e-9


def test_convergence_patience_stops_search():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=6,
            goal_tolerance=0.5,
            max_nodes=8000,
            optimize_after_goal=True,
            cost_tol=0.05,
            convergence_patience=50,
        ),
    )
    planner.solve()
    assert planner.reached_goal
    assert planner.converged
    assert planner.iterations < 8000


def test_record_history_for_animation():
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=0,
            goal_tolerance=0.5,
            max_nodes=800,
            record_history=True,
            history_stride=25,
        ),
    )
    planner.solve()
    assert len(planner.history) >= 2
    assert planner.history[0].iteration == 1
    assert all(frame.tree_edges for frame in planner.history[1:])


# --- merged from test_spatial.py ---

from dataclasses import dataclass

import numpy as np
import pytest

from minilink.core.backends import array_module
from minilink.core.geometry import Sphere
from minilink.core.kinematics import apply
from minilink.dynamics.catalog.vehicles.steering import (
    HolonomicMobileRobot,
    HolonomicMobileRobot3D,
    KinematicCar,
)
from minilink.planning.spatial.collision import (
    CollisionBody,
    bind,
    car_outline,
    disc,
    point_probe,
)
from minilink.planning.spatial.grid import sample_field_costs
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import (
    inverse_barrier,
    occupancy,
    quadratic_hinge,
)
from minilink.planning.spatial.state_fields import StateField
from minilink.planning.spatial.workspace_fields import GaussianField

# --- test doubles ----------------------------------------------------------


@dataclass(frozen=True)
class _ConstField(StateField):
    val: float

    def value(self, x, u=None, t=0.0, params=None):
        return float(self.val)


class _TwoSphereBody(CollisionBody):
    """Two parts: one at the state position, one offset by +2 in x."""

    @property
    def shapes(self):
        return (Sphere(np.zeros(2), 0.1), Sphere(np.zeros(2), 0.2))

    def body_poses(self, x, u=None, t=0.0, params=None):
        def planar(tx, ty):
            return np.array([[1.0, 0.0, tx], [0.0, 1.0, ty], [0.0, 0.0, 1.0]])

        return (planar(x[0], x[1]), planar(x[0] + 2.0, x[1]))


# --- robot body helpers ----------------------------------------------------


def _holonomic_disc(radius=0.3):
    return bind(HolonomicMobileRobot(), disc(radius))


def _holonomic_point():
    return bind(HolonomicMobileRobot(), point_probe())


def _kinematic_car_body(length=1.6, width=0.5):
    return bind(KinematicCar(), car_outline(length, width))


# --- robot body ------------------------------------------------------------


def test_bind_disc_body_pose():
    body = _holonomic_disc(0.3)
    (T,) = body.body_poses(np.array([1.5, -2.0]))
    assert T.shape == (4, 4)
    assert apply(T, np.zeros(2)) == pytest.approx([1.5, -2.0])


def test_bind_disc_in_3d():
    body = bind(HolonomicMobileRobot3D(), Sphere(np.zeros(3), 0.5))
    (T,) = body.body_poses(np.array([1.0, 2.0, 3.0]))
    assert T.shape == (4, 4)
    assert apply(T, np.zeros(3)) == pytest.approx([1.0, 2.0, 3.0])


def test_multibody_clearance_is_worst_case_over_parts():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    value = scene.clearance_field(_TwoSphereBody()).value(np.array([1.0, 0.0]))
    # part 0 at (1,0) radius 0.1 -> 0.4; part 1 at (3,0) radius 0.2 -> 2.3; min wins
    assert np.ndim(value) == 0
    assert value == pytest.approx(min(1.0 - 0.5 - 0.1, 3.0 - 0.5 - 0.2))


def test_bound_disc_clearance_samples():
    scene = Scene(obstacles=(Sphere([2.0, 2.0], 0.5),))
    field = scene.clearance_field(_holonomic_disc(0.3))
    assert field.value(np.array([0.0, 0.0])) == pytest.approx(
        np.hypot(2.0, 2.0) - 0.5 - 0.3
    )
    assert field.value(np.array([2.0, 2.0])) == pytest.approx(-0.5 - 0.3)


def test_bound_car_clearance_depends_on_heading():
    from minilink.core.geometry import Box

    scene = Scene(
        obstacles=(Box([-3.0, -3.0], [-0.4, 3.0]), Box([0.4, -3.0], [3.0, 3.0]))
    )
    field = scene.clearance_field(_kinematic_car_body())
    assert field.value(np.array([0.0, 0.0, np.pi / 2])) > 0.0
    assert field.value(np.array([0.0, 0.0, 0.0])) < 0.0


def test_bound_frame_mismatch_raises():
    sys = HolonomicMobileRobot()
    body = bind(sys, disc(0.1), frame="missing")
    with pytest.raises(KeyError):
        body.body_poses(np.array([0.0, 0.0]))


class _TwoFramePlant:
    def tf(self, x, u=None, t=0.0, params=None):
        from minilink.core.kinematics import translation

        return {
            "f1": translation(x[0], x[1], 0.0),
            "f2": translation(x[0] + 1.0, x[1], 0.0),
        }


def test_bind_multi_frame():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    body = bind(_TwoFramePlant(), [("f1", disc(0.1)), ("f2", disc(0.2))])
    value = scene.clearance_field(body).value(np.array([0.0, 0.0]))
    assert value == pytest.approx(min(-0.5 - 0.1, 1.0 - 0.5 - 0.2))


def test_bound_jax_twin_margin_matches_and_differentiates():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    scene = Scene(obstacles=(Sphere([2.0, 2.0], 0.5),))
    sys = HolonomicMobileRobot()
    free = scene.clearance_field(bind(sys, disc(0.3))).as_constraint()
    x = np.array([0.0, 0.0])

    assert float(free.margin(jnp.array(x))[0]) == pytest.approx(
        float(free.margin(x)[0])
    )

    gradient = jax.grad(lambda q: jnp.sum(free.margin(q)))(jnp.array(x))
    assert np.asarray(gradient).shape == (2,)


# --- scene + clearance field -----------------------------------------------


def test_clearance_value_against_obstacle():
    scene = Scene(obstacles=(Sphere([2.0, 2.0], 0.5),))
    field = scene.clearance_field(_holonomic_disc(0.3))
    assert field.value(np.array([0.0, 0.0])) == pytest.approx(
        np.hypot(2.0, 2.0) - 0.5 - 0.3
    )
    assert field.value(np.array([2.0, 2.0])) == pytest.approx(-0.5 - 0.3)


def test_clearance_pipeline_in_3d():
    scene = Scene(obstacles=(Sphere([0.0, 0.0, 0.0], 1.0),))
    body = bind(HolonomicMobileRobot3D(), Sphere(np.zeros(3), 0.5))
    field = scene.clearance_field(body)
    assert field.value(np.array([3.0, 0.0, 0.0])) == pytest.approx(3.0 - 1.0 - 0.5)
    free = field.as_constraint()
    assert free.contains(np.array([3.0, 0.0, 0.0]))
    assert not free.contains(np.array([0.5, 0.0, 0.0]))


def test_empty_scene_raises():
    scene = Scene()
    with pytest.raises(ValueError):
        scene.clearance(np.zeros(2))
    with pytest.raises(ValueError):
        scene.clearance_field(_holonomic_disc(0.3))


# --- workspace fields ------------------------------------------------------


def test_gaussian_field_peak_and_decay():
    field = GaussianField(center=[1.0, 2.0], amplitude=3.0, sigma=0.5)
    assert field.density(np.array([1.0, 2.0])) == pytest.approx(3.0)
    assert field.density(np.array([2.0, 2.0])) == pytest.approx(
        3.0 * np.exp(-0.5 * (1.0 / 0.5) ** 2)
    )


def test_scene_cost_density_sums_workspace_fields():
    f1 = GaussianField(center=[0.0, 0.0], amplitude=1.0, sigma=1.0)
    f2 = GaussianField(center=[1.0, 0.0], amplitude=2.0, sigma=1.0)
    scene = Scene(workspace_fields=(f1, f2))
    p = np.array([0.0, 0.0])
    assert scene.cost_density(p) == pytest.approx(f1.density(p) + f2.density(p))


def test_cost_field_value_at_robot_position():
    center = np.array([2.0, 2.0])
    scene = Scene(
        workspace_fields=(GaussianField(center=center, amplitude=1.5, sigma=0.8),)
    )
    x = np.array([0.0, 0.0])
    assert scene.cost_field(_holonomic_disc(0.3)).value(x) == pytest.approx(
        scene.cost_density(x)
    )


def test_cost_field_as_cost_weight_and_peak():
    scene = Scene(
        workspace_fields=(GaussianField(center=[0.0, 0.0], amplitude=2.0, sigma=0.5),)
    )
    cost = scene.cost_field(_holonomic_disc(0.2)).as_cost(weight=4.0)
    at_peak = cost.g(np.array([0.0, 0.0]), np.zeros(1))
    far = cost.g(np.array([5.0, 0.0]), np.zeros(1))
    assert at_peak == pytest.approx(8.0)
    assert at_peak > far


def test_cost_field_as_constraint_keep_out_band():
    scene = Scene(
        workspace_fields=(GaussianField(center=[0.0, 0.0], amplitude=1.0, sigma=0.5),)
    )
    keep_out = scene.cost_field(_holonomic_point()).as_constraint(upper=0.2)
    assert keep_out.contains(np.array([2.0, 0.0]))
    assert not keep_out.contains(np.array([0.0, 0.0]))


def test_empty_workspace_fields_return_zero_density_and_cost_field():
    scene = Scene()
    assert scene.cost_density(np.zeros(2)) == pytest.approx(0.0)
    assert scene.cost_field(_holonomic_disc(0.3)).value(
        np.array([1.0, 2.0])
    ) == pytest.approx(0.0)


# --- FieldSet --------------------------------------------------------------


def test_fieldset_contains_matches_minkowski_sum():
    center, obstacle_r, robot_r = np.array([2.0, 2.0]), 0.5, 0.3
    scene = Scene(obstacles=(Sphere(center, obstacle_r),))
    free = scene.clearance_field(_holonomic_disc(robot_r)).as_constraint()

    rng = np.random.default_rng(0)
    for x in rng.uniform(-1.0, 5.0, size=(200, 2)):
        expected_free = np.linalg.norm(x - center) >= obstacle_r + robot_r
        assert free.contains(x) == expected_free


def test_sphere_radius_shifts_free_boundary():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    x = np.array([0.65, 0.0])  # 0.5 < dist < 0.5 + 0.3
    assert scene.clearance_field(_holonomic_point()).as_constraint().contains(x)
    assert not scene.clearance_field(_holonomic_disc(0.3)).as_constraint().contains(x)


def test_fieldset_bound_mechanics():
    f = _ConstField(0.5)
    assert f.as_constraint(lower=0.0).contains(np.zeros(1))
    assert not f.as_constraint(lower=None, upper=0.2).contains(np.zeros(1))
    assert f.as_constraint(lower=0.0, upper=1.0).contains(np.zeros(1))


def test_fieldset_requires_a_bound():
    with pytest.raises(ValueError):
        _ConstField(1.0).as_constraint(lower=None, upper=None)


# --- FieldCost + cross export ---------------------------------------------


def test_fieldcost_weight_and_shaping():
    f = _ConstField(2.0)
    assert f.as_cost(weight=3.0).g(np.zeros(2), np.zeros(1)) == pytest.approx(6.0)
    squared = f.as_cost(weight=1.0, shaping=lambda v: v**2)
    assert squared.g(np.zeros(2), np.zeros(1)) == pytest.approx(4.0)


def test_cross_export_constraint_and_barrier_cost():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    field = scene.clearance_field(_holonomic_disc(0.3))
    free = field.as_constraint()

    def barrier(v):
        xp = array_module(v)
        return xp.maximum(0.2 - v, 0.0) ** 2

    cost = field.as_cost(weight=10.0, shaping=barrier)
    inside, far = np.array([0.0, 0.0]), np.array([3.0, 0.0])

    assert not free.contains(inside)
    assert free.contains(far)
    assert cost.g(inside, np.zeros(2)) > 1.0
    assert cost.g(far, np.zeros(2)) == pytest.approx(0.0)


# --- shaping ---------------------------------------------------------------


def test_quadratic_hinge():
    s = quadratic_hinge(threshold=1.0)
    assert s(np.asarray(2.0)) == pytest.approx(0.0)  # free of the margin
    assert s(np.asarray(0.0)) == pytest.approx(1.0)  # (1 - 0)^2
    assert s(np.asarray(-1.0)) == pytest.approx(4.0)  # (1 - (-1))^2


def test_inverse_barrier_blows_up_at_contact():
    s = inverse_barrier(epsilon=0.1)
    assert s(np.asarray(1.0)) == pytest.approx(1.0)
    assert s(np.asarray(0.0)) == pytest.approx(100.0)  # clamped at epsilon
    assert s(np.asarray(-5.0)) == pytest.approx(100.0)


def test_occupancy_is_bounded_unit_interval():
    s = occupancy(scale=0.5)
    assert s(np.asarray(0.0)) == pytest.approx(0.5)  # boundary
    assert s(np.asarray(5.0)) < 1e-3  # free -> ~0
    assert s(np.asarray(-5.0)) > 1.0 - 1e-3  # collision -> ~1


def test_shaping_composes_with_clearance_field():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    field = scene.clearance_field(_holonomic_disc(0.2))
    bounded = field.as_cost(weight=3.0, shaping=occupancy(scale=0.1))
    # bounded occupancy keeps the per-step obstacle cost in [0, weight]
    assert 0.0 <= bounded.g(np.array([0.0, 0.0]), np.zeros(2)) <= 3.0
    assert bounded.g(np.array([5.0, 0.0]), np.zeros(2)) < 1e-2  # ~free


# --- JAX twin --------------------------------------------------------------


def test_jax_twin_margin_matches_and_differentiates():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    scene = Scene(obstacles=(Sphere([2.0, 2.0], 0.5),))
    free = scene.clearance_field(_holonomic_disc(0.3)).as_constraint()
    x = np.array([0.0, 0.0])

    assert float(free.margin(jnp.array(x))[0]) == pytest.approx(
        float(free.margin(x)[0])
    )

    gradient = jax.grad(lambda q: jnp.sum(free.margin(q)))(jnp.array(x))
    assert np.asarray(gradient).shape == (2,)


# --- plotting --------------------------------------------------------------


def test_scene_plot_smoke():
    import os

    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg", force=True)

    scene = Scene(
        obstacles=(Sphere([4.0, 0.0], 0.5),),
        workspace_fields=(GaussianField([2.0, 1.5], 2.0, 1.0),),
    )
    fig, ax = scene.plot(show=False)
    assert fig is not None
    assert ax is not None

    fig, ax = scene.plot(show=False, body=_holonomic_disc(0.25), x=np.array([2.0, 0.0]))
    assert len(ax.patches) >= 2

    _, ax = scene.plot(show=False, body=_holonomic_point(), x=np.array([2.0, 0.0]))
    assert len(ax.lines) >= 1


# --- workspace cost raster -------------------------------------------------


def test_sample_field_costs_matches_point_probe_cost():
    scene = Scene(obstacles=(Sphere([2.0, 0.0], 0.5),))
    body = _holonomic_point()
    cost = scene.clearance_field(body).as_cost(
        weight=4.0, shaping=inverse_barrier(epsilon=0.1)
    )
    bounds = ((-1.0, 4.0), (-2.0, 2.0))
    grid = sample_field_costs([cost], bounds=bounds, grid=(5, 4))
    p = np.array([grid.xs[2], grid.ys[1]])
    assert grid.Z[1, 2] == pytest.approx(cost.g(p, np.zeros(1)))


def test_sample_field_costs_obstacle_is_radially_symmetric():
    scene = Scene(obstacles=(Sphere([2.0, 0.0], 0.5),))
    cost = scene.clearance_field(_holonomic_point()).as_cost(weight=1.0)
    bounds = ((0.0, 4.0), (-2.0, 2.0))
    grid = sample_field_costs([cost], bounds=bounds, grid=(41, 21))
    center_j = int(np.argmin(np.abs(grid.ys)))
    row = grid.Z[center_j, :]
    xs = grid.xs
    left = row[xs < 2.0]
    right = row[xs > 2.0]
    assert np.allclose(left, right[::-1], rtol=0.0, atol=1e-12)


def test_cost_field_cmap_low_to_high_contrast():
    from minilink.planning.spatial.plotting import cost_field_cmap

    cmap = cost_field_cmap()
    low = cmap(0.0)
    high = cmap(1.0)
    assert low[2] > low[0]  # blue end
    assert high[0] > high[2]  # red end


def test_plot_cost_field_exports_smoke(tmp_path):
    import os

    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg", force=True)

    from minilink.planning.spatial.plotting import plot_cost_field_exports

    scene = Scene(obstacles=(Sphere([1.0, 0.0], 0.4),))
    body = _holonomic_point()
    bounds = ((-1.0, 3.0), (-1.0, 1.0))
    path = scene.clearance_field(body).as_cost(weight=1.0)
    corridor = scene.clearance_field(body).as_cost(weight=2.0)
    obstacle = scene.clearance_field(body).as_cost(weight=3.0)
    grids = {
        "path": sample_field_costs([path], bounds=bounds, grid=(16, 14)),
        "corridor": sample_field_costs([corridor], bounds=bounds, grid=(16, 14)),
        "obstacle": sample_field_costs([obstacle], bounds=bounds, grid=(16, 14)),
        "combined": sample_field_costs(
            [path, corridor, obstacle], bounds=bounds, grid=(16, 14)
        ),
    }
    out = plot_cost_field_exports(
        grids,
        scene=scene,
        overlay_bounds=bounds,
        log_scale=True,
        show=False,
        save_dir=tmp_path,
    )
    assert set(out) == {"path", "corridor", "obstacle", "combined"}
    assert out["obstacle"]["2d"][1].get_title() == "Obstacle cost (2D)"
    assert out["combined"]["3d"][1].get_title() == "Combined cost (3D)"
    for entry in out.values():
        fig2d, ax2d = entry["2d"]
        fig3d, ax3d = entry["3d"]
        assert fig2d is not None and ax2d is not None
        assert fig3d is not None and ax3d is not None
        assert len(fig2d.axes) == 2
        assert len(fig3d.axes) == 2
    assert (tmp_path / "combined_2d.png").exists()
    assert (tmp_path / "combined_3d.png").exists()


# --- merged from test_reference_paths.py ---

"""Tests for reference paths and corridor tracking fields."""

import os

import numpy as np
import pytest

from minilink.core.geometry import Sphere
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.spatial.collision import bind, disc, point_probe
from minilink.planning.spatial.paths import PolylinePath, from_waypoints
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import quadratic_excess
from minilink.planning.spatial.track import ReferenceTrack

_HOLONOMIC = HolonomicMobileRobot()


def _disc(radius=0.3):
    return bind(_HOLONOMIC, disc(radius))


def _point():
    return bind(_HOLONOMIC, point_probe())


def test_polyline_distance_to_segment():
    path = PolylinePath([[0.0, 0.0], [10.0, 0.0]])
    assert path.distance(np.array([5.0, 2.0])) == pytest.approx(2.0)
    assert path.distance(np.array([-1.0, 0.0])) == pytest.approx(1.0)
    assert path.distance(np.array([11.0, 0.0])) == pytest.approx(1.0)


def test_polyline_project_and_sample_roundtrip():
    path = PolylinePath([[0.0, 0.0], [3.0, 4.0], [3.0, 9.0]])
    s, closest = path.project(np.array([3.0, 4.0]))
    assert s == pytest.approx(5.0)
    assert closest == pytest.approx([3.0, 4.0])
    assert path.sample(5.0) == pytest.approx([3.0, 4.0])
    assert path.total_length == pytest.approx(10.0)


def test_from_waypoints_default_is_polyline():
    path = from_waypoints([[0, 0], [1, 0], [1, 1]])
    assert isinstance(path, PolylinePath)
    assert path.distance(np.array([0.5, 0.5])) == pytest.approx(0.5)


def test_corridor_margin_inside_and_outside():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.5)
    assert track.corridor_margin(np.array([5.0, 1.0])) == pytest.approx(0.5)
    assert track.corridor_margin(np.array([5.0, 2.0])) == pytest.approx(-0.5)


def test_corridor_field_subtracts_robot_radius():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    field = track.corridor_field(_disc(0.3))
    # center at y=1 -> margin 0; body edge at 0.7 -> field value -0.3 + ...
    # half_width - dist - r = 1 - 1 - 0.3 = -0.3
    assert field.value(np.array([5.0, 1.0])) == pytest.approx(-0.3)


def test_path_distance_field_uses_robot_probes():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=2.0)
    field = track.distance_field(_disc(0.3))
    assert field.value(np.array([5.0, 1.0])) == pytest.approx(0.7)


def test_corridor_field_as_constraint():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    inside = track.corridor_field(_point()).as_constraint(lower=0.0)
    assert inside.contains(np.array([5.0, 0.5]))
    assert not inside.contains(np.array([5.0, 2.0]))


def test_path_distance_as_soft_cost():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    cost = track.distance_field(_point()).as_cost(
        weight=2.0, shaping=quadratic_excess(threshold=0.0)
    )
    on_path = np.array([5.0, 0.0])
    off_path = np.array([5.0, 2.0])
    assert cost.g(on_path, np.zeros(1)) == pytest.approx(0.0)
    assert cost.g(off_path, np.zeros(1)) == pytest.approx(2.0 * 2.0**2)


def test_obstacle_and_corridor_compose():
    scene = Scene(obstacles=(Sphere([5.0, 0.0], 0.5),))
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    body = _point()
    free = scene.clearance_field(body).as_constraint() & track.corridor_field(
        body
    ).as_constraint(lower=0.0)
    assert free.contains(np.array([1.0, 0.2]))
    assert not free.contains(np.array([5.0, 0.0]))
    assert not free.contains(np.array([1.0, 2.0]))


def test_jax_path_distance_matches():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    field = track.corridor_field(_point())
    x = np.array([4.0, 0.3])
    np_val = float(field.value(x))
    jax_val = float(field.value(jnp.array(x)))
    assert jax_val == pytest.approx(np_val)

    grad = jax.grad(lambda q: field.value(q))(jnp.array(x))
    assert np.asarray(grad).shape == (2,)


def test_plot_track_smoke():
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg", force=True)

    track = ReferenceTrack(from_waypoints([[0, 0], [5, 1], [10, 0]]), half_width=0.8)
    fig, ax = track.plot(show=False)
    assert fig is not None
    assert len(ax.lines) >= 1


# --- merged from test_dynamic_programming.py ---

"""Tests for dynamic-programming policy synthesis."""

import unittest

import numpy as np

from minilink.core.costs import QuadraticCost, TimeCost
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
        alpha=0.95, tol=1e-3, max_iterations=400, **opt_kwargs
    )
    planner = DynamicProgrammingPlanner(problem, grid=grid, options=options)
    return planner, planner.solve().policy


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
        self.assertTrue(grid._jax_transition)
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
        result = planner.solve_steps(5).policy
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

        traj = diagram.compute_trajectory(tf=8.0, verbose=False)
        self.assertLess(np.linalg.norm(traj.x[:, -1]), 0.3)

    def test_policy_evaluator_matches_optimal_value(self):
        problem = make_problem()
        planner, result = solve(problem)
        controller = result.controller()
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
        return problem, grid

    def test_jax_g_table_matches_numpy(self):
        problem, grid = self._pendulum_grid()
        x_next, action_ok, x_next_ok = grid.transition(0.0)
        np_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy")
        )
        jax_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="jax")
        )
        G_np = np_planner._running_cost(action_ok, x_next_ok, 0.0)
        G_jax = jax_planner._running_cost(action_ok, x_next_ok, 0.0)
        self.assertTrue(np.allclose(G_np, G_jax))

    def test_jax_j0_matches_numpy(self):
        problem, grid = self._pendulum_grid()
        np_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy")
        )
        jax_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="jax")
        )
        J0_np = np_planner._terminal_cost(0.0)
        J0_jax = jax_planner._terminal_cost(0.0)
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
        x_next, action_ok, x_next_ok = grid.transition(0.0)
        np_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="numpy")
        )
        jax_planner = DynamicProgrammingPlanner(
            problem, grid=grid, options=DynamicProgrammingOptions(backend="jax")
        )
        G_np = np_planner._running_cost(action_ok, x_next_ok, 0.0)
        G_jax = jax_planner._running_cost(action_ok, x_next_ok, 0.0)
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

        feasible = vectorized.J < 1e5
        gap = np.max(np.abs(jax_result.J[feasible] - vectorized.J[feasible]))
        self.assertLess(gap, 1e-4)
        # policies agree everywhere except rare argmin ties
        self.assertGreater(np.mean(jax_result.pi == vectorized.pi), 0.98)


if __name__ == "__main__":
    unittest.main()
