import contextlib
import io
import tempfile
import unittest

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.sets import BallSet, BoxSet, SingletonSet
from minilink.core.system import DynamicSystem, System
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
        sys = System(n=2)
        sys.add_input_port("u")
        sys.add_output_port("y", dim=2, function=sys.h)
        sys.state.lower_bound = np.array([-1.0, -2.0])
        sys.state.upper_bound = np.array([1.0, 2.0])
        sys.inputs["u"].lower_bound = np.array([-3.0])
        sys.inputs["u"].upper_bound = np.array([4.0])
        sys.x0 = np.array([0.1, -0.2])
        return sys

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
        self.assertTrue(problem.X.contains(np.array([0.0, 0.0])))
        self.assertTrue(problem.U.contains(np.array([0.0])))
        self.assertFalse(problem.U.contains(np.array([10.0])))
        self.assertTrue(problem.has_goal)
        self.assertFalse(problem.has_cost)

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

    def test_planner_require_result_before_solve(self):
        sys = self.make_system()
        cost = QuadraticCost.from_system(sys)
        problem = PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]), cost=cost)
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(tf=1.0, n_steps=5)
            ),
        )
        with self.assertRaises(ValueError):
            planner.require_result()

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
        problem = PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]), cost=cost)

        to = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(tf=1.0, n_steps=5)
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
                DirectCollocationOptions(tf=1.0, n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        traj = planner.compute_solution()

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
                DirectCollocationOptions(tf=1.0, n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
                solve_disp=True,
            ),
        )
        stdout = io.StringIO()

        with contextlib.redirect_stdout(stdout):
            planner.compute_solution()

        report = stdout.getvalue()
        self.assertIn("Trajectory Optimization Program", report)
        self.assertIn("transcription: DirectCollocationTranscription", report)
        self.assertIn("method='scipy_slsqp'", report)
        self.assertNotIn("===               Optimization Program", report)
        self.assertIn("terminal_error_inf:", report)

    def test_generic_trajopt_planner_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(tf=1.0, n_steps=5)
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

        traj = planner.compute_solution()
        warm_started = planner.compute_solution(warm_start=True)

        self.assertTrue(planner.last_optimization_result.success)
        self.assertEqual(planner.last_program.metadata["compile_backend"], "direct")
        self.assertGreater(len(planner.iteration_history), 0)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)
        np.testing.assert_allclose(warm_started.x[:, -1], [1.0], atol=1e-7)

    def test_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        transcription = ShootingTranscription(ShootingOptions(tf=1.0, n_steps=5))
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        traj = planner.compute_solution()

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
            MultipleShootingOptions(tf=1.0, n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="numpy",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        traj = planner.compute_solution()

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
        transcription = ShootingTranscription(ShootingOptions(tf=1.0, n_steps=5))
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

        x = evaluator.rk4_rollout_forced(
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
        transcription = RecordingTranscription(
            DirectCollocationOptions(tf=1.0, n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                warm_start=True,
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        first = planner.compute_solution()
        planner.compute_solution()

        self.assertIs(transcription.guesses[1], first)


if __name__ == "__main__":
    unittest.main()
