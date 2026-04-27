import unittest

import numpy as np

from minilink.core.system import System
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    InequalityConstraint,
    MathematicalProgram,
    VariableBounds,
)
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.costs import QuadraticCost
from minilink.planning.policy_synthesis.dynamic_programming import (
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.rrt import RRTPlanner
from minilink.planning.sets import BallSet, BoxSet, SingletonSet
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationPlanner,
)


class TestPlanningArchitecture(unittest.TestCase):
    def make_system(self):
        sys = System(n=2, m=1, p=2)
        sys.state.lower_bound = np.array([-1.0, -2.0])
        sys.state.upper_bound = np.array([1.0, 2.0])
        sys.inputs["u"].lower_bound = np.array([-3.0])
        sys.inputs["u"].upper_bound = np.array([4.0])
        sys.x0 = np.array([0.1, -0.2])
        return sys

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

    def test_planner_require_result_before_solve(self):
        sys = self.make_system()
        cost = QuadraticCost.from_system(sys)
        problem = PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]), cost=cost)
        dp = DynamicProgrammingPlanner(
            problem,
            x_grid_shape=(5, 5),
            u_grid_shape=(3,),
            dt=0.1,
        )
        with self.assertRaises(ValueError):
            dp.require_result()

    def test_mathematical_program_constraints_are_backend_neutral(self):
        equality = EqualityConstraint(
            h=lambda z: np.array([z[0] + z[1] - 1.0]),
            name="sum_to_one",
        )
        inequality = InequalityConstraint(
            g=lambda z: np.array([z[0], z[1]]),
            name="nonnegative",
        )
        program = MathematicalProgram(
            J=lambda z: float(z.T @ z),
            z0=np.array([0.5, 0.5]),
            bounds=VariableBounds(lower=np.zeros(2), upper=np.ones(2)),
            equalities=(equality,),
            inequalities=(inequality,),
        )

        self.assertEqual(program.n_z, 2)
        self.assertEqual(program.objective(np.array([1.0, 2.0])), 5.0)
        np.testing.assert_allclose(equality.residual(np.array([0.25, 0.75])), [0.0])
        np.testing.assert_allclose(
            inequality.margin(np.array([0.25, 0.75])), [0.25, 0.75]
        )

        with self.assertRaises(ValueError):
            MathematicalProgram(
                J=lambda z: 0.0,
                z0=np.zeros(2),
                bounds=VariableBounds(lower=np.zeros(3)),
            )

    def test_solver_skeletons_validate_architecture_inputs(self):
        sys = self.make_system()
        cost = QuadraticCost.from_system(sys)
        problem = PlanningProblem(sys=sys, x_goal=np.array([0.0, 0.0]), cost=cost)

        to = DirectCollocationPlanner(
            problem,
            tf=1.0,
            n_steps=5,
            optimizer=ScipyMinimizeOptimizer(),
        )
        self.assertEqual(to.options.n_steps, 5)

        rrt = RRTPlanner(problem, dt=0.1)
        self.assertEqual(rrt.options.max_nodes, 2000)

        dp = DynamicProgrammingPlanner(
            problem,
            x_grid_shape=(5, 5),
            u_grid_shape=(3,),
            dt=0.1,
        )
        self.assertEqual(dp.options.x_grid_shape, (5, 5))

        with self.assertRaises(NotImplementedError):
            to.compute_solution()


if __name__ == "__main__":
    unittest.main()
