import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.core.costs import QuadraticCost
from minilink.core.distributions import Gaussian, Particles, Uniform
from minilink.core.sets import BallSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.state_fields import StateField


class TestInspectPrint(unittest.TestCase):
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
        self.assertIn("rho=0", text)
        self.assertNotIn("array(", text)

    def test_planning_problem(self):
        plant = Integrator()
        problem = PlanningProblem(
            plant,
            tf=2.0,
            cost=QuadraticCost.from_system(plant),
        )
        text = str(problem)
        self.assertIn("PlanningProblem, Integrator, tf=2", text)
        self.assertIn("X0:", text)
        self.assertIn("cost: QuadraticCost", text)

    def test_field_and_field_set(self):
        class Height(StateField):
            def value(self, x, u=None, t=0.0, params=None):
                return x[0]

        field = Height()
        self.assertEqual(str(field), "Height")
        self.assertIn("FieldSet (Height)", str(field.as_constraint(lower=0.0)))

    def test_mathematical_program(self):
        program = MathematicalProgram(n_z=3, J=lambda z: z @ z, lower=np.zeros(3))
        text = str(program)
        self.assertIn("MathematicalProgram, n_z=3", text)
        self.assertIn("h: no, g: no", text)
        self.assertIn("bounds: lower", text)
