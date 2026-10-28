import copy
import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.core.costs import QuadraticCost
from minilink.core.distributions import Gaussian, Particles, Uniform
from minilink.core.fields import Field
from minilink.core.inspect import latex_inspect
from minilink.core.sets import BallSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizer import Optimizer
from minilink.planning.problems import PlanningProblem, StochasticPlanningProblem


class TestInspectPrint(unittest.TestCase):
    def test_system(self):
        plant = Integrator()
        text = str(plant)
        self.assertIn("Integrator, n=1", text)
        self.assertIn("inputs:", text)

    def test_trajectory(self):
        traj = Trajectory(
            t=np.linspace(0.0, 2.0, 5),
            x=np.zeros((2, 5)),
            u=np.zeros((1, 5)),
            signals={"y": np.zeros((2, 5))},
        )
        text = str(traj)
        self.assertIn("Trajectory, N=5, t=0–2", text)
        self.assertIn("x (2, 5)", text)
        self.assertIn("u (1, 5)", text)
        self.assertIn("signals: y", text)
        self.assertNotIn("array(", text)

    def test_sets(self):
        box = BoxSet([-1.0, -2.0], [1.0, 2.0])
        self.assertIn("BoxSet, dim=2", str(box))
        self.assertIn("<= z <=", str(box))
        self.assertIn("BallSet, dim=2", str(BallSet([0.0, 0.0], 1.5)))
        self.assertIn("SingletonSet, dim=1", str(SingletonSet([0.0])))
        both = box & BallSet([0.0, 0.0], 1.0)
        self.assertIn("IntersectionSet of BoxSet, BallSet", str(both))

    def test_distributions(self):
        self.assertIn("Gaussian, dim=2", str(Gaussian([0.0, 0.0], [1.0, 2.0])))
        self.assertIn("Uniform, dim=1", str(Uniform([-1.0], [1.0])))
        self.assertIn("Particles, dim=2, N=3", str(Particles(np.zeros((3, 2)))))

    def test_cost(self):
        cost = QuadraticCost.from_system(Integrator())
        text = str(cost)
        self.assertIn("QuadraticCost", text)
        self.assertIn("xbar:", text)
        self.assertNotIn("array(", text)

    def test_sum_cost(self):
        plant = Integrator()
        cost = QuadraticCost.from_system(plant) + copy.deepcopy(
            QuadraticCost.from_system(plant)
        )
        text = str(cost)
        self.assertIn("SumCost", text)
        self.assertIn("terms:", text)

    def test_planning_problem(self):
        plant = Integrator()
        problem = PlanningProblem(
            plant,
            tf=2.0,
            cost=QuadraticCost.from_system(plant),
        )
        text = str(problem)
        self.assertIn("PlanningProblem, Integrator, tf=2, rho=0", text)
        self.assertIn("X0:", text)
        self.assertIn("cost: QuadraticCost", text)

    def test_stochastic_planning_problem(self):
        plant = Integrator()
        law = Gaussian([0.0], [0.1])
        problem = StochasticPlanningProblem(
            plant,
            tf=1.0,
            x0_distribution=law,
            cost=QuadraticCost.from_system(plant),
        )
        text = str(problem)
        self.assertIn("StochasticPlanningProblem", text)
        self.assertIn("criterion=expectation", text)
        self.assertIn("x0:", text)
        self.assertIn("Gaussian", text)

    def test_field_and_field_set(self):
        class Height(Field):
            def value(self, x, u=None, t=0.0, params=None):
                return x[0]

        field = Height()
        self.assertEqual(str(field), "Height")
        self.assertIn("FieldSet (Height)", str(field.as_set(lower=0.0)))

    def test_mathematical_program(self):
        program = MathematicalProgram(n_z=3, J=lambda z: z @ z, lower=np.zeros(3))
        text = str(program)
        self.assertIn("MathematicalProgram, n_z=3", text)
        self.assertIn("class: bounded", text)
        self.assertIn("h: no", text)
        self.assertIn("g: no", text)
        self.assertIn("bounds: lower", text)
        self.assertIn("backend: numpy", text)

    def test_optimizer_and_result(self):
        program = MathematicalProgram(n_z=2, J=lambda z: z @ z)
        opt = Optimizer(program, z0=[0.0, 0.0], method="l-bfgs")
        text = str(opt)
        self.assertIn("Optimizer, method='l-bfgs'", text)
        self.assertIn("program:", text)

        result = OptimizationResult(z=[0.0, 0.0], success=True, cost=0.0)
        result_text = str(result)
        self.assertIn("OptimizationResult, success=yes", result_text)
        self.assertIn("J*:", result_text)

    def test_latex_inspect(self):
        plant = Integrator()
        cost = QuadraticCost.from_system(plant)
        self.assertIn(
            r"\dot{x}", latex_inspect(LTISystem(np.eye(1), [[1]], [[1]], [[0]]))
        )
        self.assertIn("Q", latex_inspect(cost))
        program = MathematicalProgram(n_z=1, J=lambda z: z @ z)
        self.assertIn(r"\min_{z}", latex_inspect(program))
