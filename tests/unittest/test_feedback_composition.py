"""The summing-junction side of ``@``: compensators, ``feedback``, port layouts."""

from __future__ import annotations

import unittest

import numpy as np

from minilink import (
    PID,
    DoubleIntegrator,
    ImpedanceController,
    Integrator,
    Lag,
    Lead,
    Pendulum,
    ProportionalController,
    Step,
    TransferFunction,
    feedback,
)
from minilink.core.feedback import error_input
from minilink.core.system import DynamicSystem


class _TwoInTwoOut(DynamicSystem):
    """dx = -x + u, y = x on two channels."""

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2)

    def f(self, x, u, t=0, params=None):
        return -x + u


class _TwoInThreeOut(DynamicSystem):
    def __init__(self):
        super().__init__(n=3, input_dim=2, output_dim=3)

    def f(self, x, u, t=0, params=None):
        return -x + np.array([u[0], u[1], u[0] + u[1]])


def _poles(diagram):
    return np.sort_complex(np.linalg.eigvals(diagram.jacobian("f", "x")))


def _damped_pendulum():
    plant = Pendulum()
    plant.params["d"] = 0.5
    return plant


class TestJunction(unittest.TestCase):
    def test_compensator_at_plant_inserts_junction_and_selector(self):
        T = PID(Kp=20.0, Kd=2.0, tau=0.05) @ _damped_pendulum()
        self.assertEqual(list(T.inputs), ["r"])
        self.assertEqual(list(T.outputs), ["y"])
        self.assertEqual(list(T.subsystems), ["ctl", "sys", "demux", "sum"])
        self.assertEqual(
            T.subsystems["demux"].dims, [1, 1]
        )  # theta, the first component
        self.assertEqual(
            T.connections["sum"], {"in0": ("input", "r"), "in1": ("demux", "out0")}
        )
        self.assertEqual(T.connections["ctl"]["e"], ("sum", "y"))
        self.assertEqual(T.connections["demux"]["u"], ("sys", "y"))

    def test_series_then_unity_equals_compensator_at_plant_and_leaves_the_loop_gain_alone(
        self,
    ):
        C, G = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05), _damped_pendulum()
        L = C >> G
        T1 = C @ G
        T2 = L @ 1
        np.testing.assert_allclose(_poles(T1), _poles(T2), atol=1e-9)
        self.assertEqual(list(L.inputs), ["e"])
        self.assertEqual(list(L.subsystems), ["ctl", "sys"])

    def test_closed_loop_poles_are_the_roots_of_one_plus_L(self):
        C, G = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05), _damped_pendulum()
        L = C >> G
        T = C @ G
        loop_tf = L.transfer_function()  # L(s) of the wired diagram
        roots = np.roots(np.polyadd(loop_tf.denominator, loop_tf.numerator))
        np.testing.assert_allclose(_poles(T), np.sort_complex(roots), atol=1e-6)

    def test_vector_loop_uses_a_vector_junction(self):
        T = PID(Kp=[3.0, 5.0], dof=2) @ _TwoInTwoOut()
        self.assertEqual(list(T.subsystems), ["ctl", "sys", "sum"])
        self.assertEqual(T.subsystems["sum"].dim, 2)
        self.assertEqual(T.inputs["r"].dim, 2)
        self.assertEqual(T.jacobian("f", "x").shape, (6, 6))

    def test_component_gain_and_sensor_return_paths(self):
        L = PID(Kp=20.0, Kd=2.0) >> _damped_pendulum()
        T = feedback(L, of=("y", 1))
        self.assertEqual(T.subsystems["demux"].dims, [1, 1])
        self.assertEqual(T.connections["sum"]["in1"], ("demux", "out1"))
        T = L @ 0.5
        self.assertEqual(list(T.subsystems), ["ctl", "sys", "demux", "gain", "sum"])
        np.testing.assert_allclose(T.subsystems["gain"].params["K"], [[0.5]])
        T = feedback(L, through=TransferFunction([1.0], [0.1, 1.0]))
        self.assertIn("sensor", T.subsystems)
        self.assertEqual(T.connections["sum"]["in1"], ("sensor", "y"))

    def test_sign_and_plant_alone(self):
        T = feedback(_damped_pendulum(), sign=+1.0)
        np.testing.assert_allclose(T.subsystems["sum"].signs, [1.0, 1.0])
        T = _damped_pendulum() @ 1
        self.assertEqual(list(T.subsystems), ["sys", "demux", "sum"])

    def test_mismatches_are_refused_with_guidance(self):
        with self.assertRaisesRegex(ValueError, "Cannot close the loop"):
            feedback(_TwoInThreeOut())
        with self.assertRaisesRegex(ValueError, "selects one component"):
            feedback(_TwoInTwoOut(), of=("y", 0))
        with self.assertRaisesRegex(ValueError, "of index must be in"):
            feedback(_damped_pendulum(), of=("y", 5))

    def test_two_port_wiring_is_unchanged(self):
        T = ImpedanceController() @ Pendulum()
        self.assertEqual(list(T.subsystems), ["ctl", "sys"])
        T = PID(Kp=5.0, ports="reference") @ Integrator()
        self.assertEqual(list(T.subsystems), ["ctl", "sys"])
        self.assertIsNone(error_input(PID(ports="reference")))
        self.assertEqual(error_input(PID()), "e")
        self.assertEqual(error_input(TransferFunction([1.0], [1.0, 1.0])), "u")

    def test_step_reference_drives_the_classical_loop(self):
        plant = _damped_pendulum()
        loop = (
            Step(final_value=0.3, step_time=1.0)
            >> PID(Kp=20.0, Kd=2.0, tau=0.05) @ plant
        )
        self.assertEqual(list(loop.subsystems), ["ref", "ctl", "sys", "demux", "sum"])
        traj = loop.compute_trajectory(tf=8.0, verbose=False)
        static_gain = 20.0 / (20.0 + 4.905 / 0.5)  # Kp / (Kp + wn^2 / k)
        self.assertAlmostEqual(traj.x[-2, -1], 0.3 * static_gain, delta=0.01)


class TestPortLayouts(unittest.TestCase):
    def test_pid_layouts_share_one_law(self):
        plant = Integrator()
        poles_error = _poles(PID(Kp=4.0, Ki=1.0, Kd=0.5, tau=0.1) @ plant)
        poles_reference = _poles(
            PID(Kp=4.0, Ki=1.0, Kd=0.5, tau=0.1, ports="reference") @ plant
        )
        np.testing.assert_allclose(poles_error, poles_reference, atol=1e-9)
        pid = PID(Kp=2.0, Ki=1.0, Kd=0.0, ports="reference")
        self.assertEqual(list(pid.inputs), ["r", "y"])
        np.testing.assert_allclose(pid.ctl(np.zeros(2), np.array([1.0, 0.25])), [1.5])
        pid = PID(Kp=2.0, Ki=1.0, Kd=0.0)
        np.testing.assert_allclose(pid.ctl(np.zeros(2), np.array([0.75])), [1.5])
        with self.assertRaises(ValueError):
            PID(ports="both")

    def test_transfer_function_and_proportional_layouts(self):
        plant = Integrator()
        C = TransferFunction([2.0, 20.0], [0.05, 1.0])
        np.testing.assert_allclose(
            _poles(C @ plant),
            _poles(
                TransferFunction([2.0, 20.0], [0.05, 1.0], ports="reference") @ plant
            ),
            atol=1e-9,
        )
        reference = TransferFunction([1.0], [1.0, 1.0], ports="reference")
        self.assertEqual(
            (list(reference.inputs), list(reference.outputs)), (["r", "y"], ["u"])
        )
        np.testing.assert_allclose(
            reference.f(np.array([0.0]), np.array([1.0, 0.25])), [0.75]
        )
        np.testing.assert_allclose(
            _poles(ProportionalController(3.0, ports="error") @ plant),
            _poles(ProportionalController(3.0) @ plant),
            atol=1e-9,
        )

    def test_lead_and_lag(self):
        lead = Lead(K=2.0, z=1.0, p=10.0)
        np.testing.assert_allclose(lead.numerator, [2.0, 2.0])
        np.testing.assert_allclose(lead.denominator, [1.0, 10.0])
        np.testing.assert_allclose(Lag(K=1.0, z=1.0, p=0.1).poles, [-0.1])
        with self.assertRaises(ValueError):
            Lead(z=10.0, p=1.0)
        with self.assertRaises(ValueError):
            Lag(z=0.1, p=1.0)
        self.assertEqual(list((Lead() >> DoubleIntegrator()).inputs), ["u"])


if __name__ == "__main__":
    unittest.main()
