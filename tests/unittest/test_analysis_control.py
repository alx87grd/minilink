"""Unit tests for analysis verbs (linearize, structural, equilibria) and LQR."""

import unittest

import numpy as np

from minilink.analysis.equilibria import find_equilibrium
from minilink.analysis.linearize import linearize
from minilink.analysis.structural import controllability, observability
from minilink.control.lqr import lqr, lqr_gain
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.mass_spring_damper.linear import SingleMass
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum, Pendulum


class TestLinearize(unittest.TestCase):
    def test_recovers_linear_system_matrices(self):
        # Linearizing a linear plant must return its own A, B (to fd tolerance).
        plant = SingleMass(mass=2.0, k=3.0, b=0.5)
        lti = linearize(plant, x_bar=[0.0, 0.0])
        np.testing.assert_allclose(lti.A(), plant.A(), atol=1e-5)
        np.testing.assert_allclose(lti.B(), plant.B(), atol=1e-5)

    def test_pendulum_mode(self):
        lti = linearize(Pendulum(), x_bar=[0.0, 0.0])
        # downward pendulum: undamped oscillator, A = [[0,1],[-g l m/(m l^2+I),0]]
        np.testing.assert_allclose(lti.A(), [[0.0, 1.0], [-4.905, 0.0]], atol=1e-4)


class TestStructural(unittest.TestCase):
    def test_double_integrator_is_controllable_observable(self):
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        C = np.array([[1.0, 0.0]])
        self.assertTrue(controllability(A, B).is_full_rank)
        self.assertTrue(observability(A, C).is_full_rank)

    def test_uncontrollable_pair_detected(self):
        A = np.array([[1.0, 0.0], [0.0, 2.0]])
        B = np.array([[1.0], [0.0]])  # second mode unreachable
        result = controllability(A, B)
        self.assertFalse(result.is_full_rank)
        self.assertEqual(result.rank, 1)


class TestEquilibria(unittest.TestCase):
    def test_pendulum_hanging_equilibrium(self):
        x_eq = find_equilibrium(Pendulum(), x_guess=[0.3, 0.0])
        np.testing.assert_allclose(x_eq, [0.0, 0.0], atol=1e-6)


class TestLQR(unittest.TestCase):
    def test_gain_stabilizes_double_integrator(self):
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        K = lqr_gain(A, B, Q=np.eye(2), R=np.array([[1.0]]))
        closed_poles = np.linalg.eigvals(A - B @ K)
        self.assertTrue(np.all(closed_poles.real < 0.0))

    def test_lqr_closed_loop_stabilizes_inverted_pendulum(self):
        plant = InvertedPendulum()
        lti = linearize(plant, x_bar=[0.0, 0.0])
        controller = lqr(
            lti.A(),
            lti.B(),
            Q=np.diag([10.0, 1.0]),
            R=np.array([[1.0]]),
            xbar=[0.0, 0.0],
            ubar=[0.0],
        )

        diagram = DiagramSystem()
        diagram.connection_verbose = False
        diagram.add_subsystem(controller, "ctl")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "x", "ctl", "x")  # full-state feedback
        diagram.connect("ctl", "u", "plant", "u")

        plant.x0 = np.array([0.2, 0.0])  # small tip from upright
        traj = diagram.compute_trajectory(tf=10.0, n_steps=1001)
        np.testing.assert_allclose(traj.x[:, -1], [0.0, 0.0], atol=1e-2)


if __name__ == "__main__":
    unittest.main()
