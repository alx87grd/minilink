"""Unit tests for control law blocks."""

import unittest

import numpy as np

from minilink.control.impedance import ImpedanceController, ImpedanceIntegralController
from minilink.control.output import ProportionalController
from minilink.control.state import StateFeedbackController
from minilink.control.siso import FilteredController
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator


class TestProportionalController(unittest.TestCase):
    def test_mimo_error_law(self):
        ctl = ProportionalController([[2.0, 0.0], [0.0, 3.0]])
        u = ctl.ctl(None, np.array([1.0, 1.0, 0.0, 0.0]))  # r=[1,1], y=[0,0]
        np.testing.assert_allclose(u, [2.0, 3.0])


class TestStateFeedbackController(unittest.TestCase):
    def test_reference_defaults_to_xbar(self):
        ctl = StateFeedbackController(
            np.array([[1.0, 2.0]]), xbar=[0.5, 0.0], ubar=[0.1]
        )
        np.testing.assert_allclose(ctl.inputs["r"].nominal_value, [0.5, 0.0])

    def test_state_feedback_law(self):
        K = np.array([[1.0, 2.0]])
        ctl = StateFeedbackController(K, xbar=[0.0, 0.0], ubar=[0.5])
        # u = ubar - K (x - r), with x=[1,1], r=[0,0]
        u = ctl.ctl(None, np.array([1.0, 1.0, 0.0, 0.0]))
        np.testing.assert_allclose(u, [0.5 - 3.0])


class TestImpedanceIntegralController(unittest.TestCase):
    def test_equations(self):
        pid = ImpedanceIntegralController()
        # integral state derivative is the position error r - position
        np.testing.assert_allclose(
            pid.f(np.array([0.0]), np.array([1.0, 0.2, 0.5])), [0.8]
        )
        # u = kp e + ki e_int - kd dy_dt = 10*0.8 + 1*0.5 - 1*0.5 = 8.0
        np.testing.assert_allclose(
            pid.ctl(np.array([0.5]), np.array([1.0, 0.2, 0.5])), [8.0]
        )

    def test_closed_loop_removes_steady_state_error(self):
        plant = DoubleIntegrator()  # ddx = u, x port = [position, speed]
        pid = ImpedanceIntegralController()
        pid.params.update({"kp": 5.0, "ki": 1.0, "kd": 4.0})

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
        # state order: [pid e_int, plant position, plant speed]
        position = traj.x[1, -1]
        speed = traj.x[2, -1]
        self.assertAlmostEqual(position, setpoint, places=2)
        self.assertAlmostEqual(speed, 0.0, places=2)


class TestFilteredController(unittest.TestCase):
    def test_equations(self):
        pid = FilteredController(kp=10.0, ki=1.0, kd=1.0, tau=0.1)
        # de_int = e = r - y; dy_filt = 0 when y matches the filter state
        np.testing.assert_allclose(
            pid.f(np.array([0.0, 0.2]), np.array([1.0, 0.2])), [0.8, 0.0]
        )
        # u = kp e + ki e_int - kd dy_filt = 10*0.8 + 1*0.5 = 8.5
        np.testing.assert_allclose(
            pid.ctl(np.array([0.5, 0.2]), np.array([1.0, 0.2])), [8.5]
        )

    def test_f_is_jax_traceable(self):
        pytest = __import__("pytest")
        jax = pytest.importorskip("jax")
        import jax.numpy as jnp

        pid = FilteredController(
            kp=10.0,
            ki=1.0,
            kd=1.0,
            u_min=-1.0,
            u_max=1.0,
            e_int_min=-0.5,
            e_int_max=0.5,
        )
        x = jnp.array([0.0, 0.2])
        u = jnp.array([1.0, 0.2])

        jax.make_jaxpr(lambda x, u: pid.f(x, u))(x, u)

    def test_closed_loop_removes_steady_state_error(self):
        plant = DoubleIntegrator()  # ddx = u, y port = position
        pid = FilteredController()
        pid.params.update({"kp": 5.0, "ki": 1.0, "kd": 4.0})

        setpoint = 1.0
        pid.inputs["r"].nominal_value = np.array([setpoint])

        diagram = DiagramSystem()
        diagram.connection_verbose = False
        diagram.add_subsystem(pid, "pid")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "y", "pid", "y")
        diagram.connect("pid", "u", "plant", "u")

        traj = diagram.compute_trajectory(tf=40.0, n_steps=4001)
        # state order: [pid e_int, pid y_filt, plant position, plant speed]
        position = traj.x[2, -1]
        speed = traj.x[3, -1]
        self.assertAlmostEqual(position, setpoint, places=2)
        self.assertAlmostEqual(speed, 0.0, places=2)


class TestImpedanceController(unittest.TestCase):
    def test_vector_regulation(self):
        ctl = ImpedanceController(dof=2)
        ctl.params.update({"Kp": [2.0, 3.0], "Kd": [0.5, 0.5]})
        u = ctl.ctl(None, np.array([1.0, 2.0, 0.1, 0.2, 0.3, 0.4]))
        np.testing.assert_allclose(u, [2.0 * 0.9 - 0.5 * 0.3, 3.0 * 1.8 - 0.5 * 0.4])

    def test_vector_tracking_ref(self):
        ctl = ImpedanceController(dof=2, tracking_ref=True)
        ctl.params.update({"Kp": [1.0, 1.0], "Kd": [1.0, 1.0]})
        u = ctl.ctl(None, np.array([1.0, 2.0, 0.0, 0.0, 0.5, 0.1, 0.2, 0.3]))
        np.testing.assert_allclose(
            u,
            [1.0 * (1.0 - 0.5) + 1.0 * (0.0 - 0.2), 1.0 * (2.0 - 0.1) + 1.0 * (0.0 - 0.3)],
        )


class TestFilteredControllerMIMO(unittest.TestCase):
    def test_dof_two_diagonal(self):
        pid = FilteredController(dof=2, kp=2.0, ki=1.0, kd=0.5, tau=0.1)
        np.testing.assert_allclose(
            pid.f(np.zeros(4), np.array([1.0, 2.0, 0.0, 0.0])),
            [1.0, 2.0, 0.0, 0.0],
        )


if __name__ == "__main__":
    unittest.main()
