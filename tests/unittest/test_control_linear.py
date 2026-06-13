"""Unit tests for the migrated linear control laws."""

import unittest

import numpy as np

from minilink.control.linear import (
    LinearFeedbackController,
    PIDController,
    ProportionalController,
)
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator


class TestProportionalController(unittest.TestCase):
    def test_mimo_error_law(self):
        ctl = ProportionalController([[2.0, 0.0], [0.0, 3.0]])
        u = ctl.ctl(None, np.array([1.0, 1.0, 0.0, 0.0]))  # r=[1,1], y=[0,0]
        np.testing.assert_allclose(u, [2.0, 3.0])


class TestLinearFeedbackController(unittest.TestCase):
    def test_reference_defaults_to_xbar(self):
        ctl = LinearFeedbackController(
            np.array([[1.0, 2.0]]), xbar=[0.5, 0.0], ubar=[0.1]
        )
        np.testing.assert_allclose(ctl.inputs["r"].nominal_value, [0.5, 0.0])

    def test_state_feedback_law(self):
        K = np.array([[1.0, 2.0]])
        ctl = LinearFeedbackController(K, xbar=[0.0, 0.0], ubar=[0.5])
        # u = ubar - K (x - r), with x=[1,1], r=[0,0]
        u = ctl.ctl(None, np.array([1.0, 1.0, 0.0, 0.0]))
        np.testing.assert_allclose(u, [0.5 - 3.0])


class TestPIDController(unittest.TestCase):
    def test_equations(self):
        pid = PIDController()
        # integral state derivative is the position error r - position
        np.testing.assert_allclose(
            pid.f(np.array([0.0]), np.array([1.0, 0.2, 0.5])), [0.8]
        )
        # u = Kp e + Ki xi - Kd rate = 10*0.8 + 1*0.5 - 1*0.5 = 8.0
        np.testing.assert_allclose(
            pid.ctl(np.array([0.5]), np.array([1.0, 0.2, 0.5])), [8.0]
        )

    def test_closed_loop_removes_steady_state_error(self):
        plant = DoubleIntegrator()  # ddx = u, x port = [position, speed]
        pid = PIDController()
        pid.params.update({"Kp": 5.0, "Ki": 1.0, "Kd": 4.0})

        setpoint = 1.0
        pid.inputs["r"].nominal_value = np.array(
            [setpoint]
        )  # unconnected r holds setpoint

        diagram = DiagramSystem()
        diagram.connection_verbose = False
        diagram.add_subsystem(pid, "pid")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "x", "pid", "y")  # feed [position, speed]
        diagram.connect("pid", "u", "plant", "u")

        traj = diagram.compute_trajectory(tf=40.0, n_steps=4001)
        # plant position is the controller-state-after index; find the plant block slice
        # state order: [pid integral, plant position, plant speed]
        position = traj.x[1, -1]
        speed = traj.x[2, -1]
        self.assertAlmostEqual(position, setpoint, places=2)
        self.assertAlmostEqual(speed, 0.0, places=2)


if __name__ == "__main__":
    unittest.main()
