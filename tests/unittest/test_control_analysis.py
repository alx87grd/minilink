"""Unit tests for analysis verbs (linearize, structural, equilibria) and LQR."""

import unittest
import numpy as np
import pytest
from minilink.analysis.equilibria import find_equilibrium
from minilink.analysis.linearize import (
    LinearizationFallbackWarning,
    linearize,
    linearize_matrices,
)
from minilink.analysis.structural import controllability, observability
from minilink.control.lqr import lqr, lqr_at_operating_point, lqr_gain
from minilink.core.backends import array_module
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.mass_spring_damper.linear import SingleMass
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum, Pendulum


class _PortLinearSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=2)
        self.name = "Port Linear System"
        self.add_input_port("force")
        self.add_input_port("bias")
        self.add_output_port("y", dim=1, function=self.h, dependencies=("bias",))
        self.add_output_port("speed", dim=1, function=self.speed)

    def f(self, x, u, t=0, params=None):
        force, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)
        return xp.array([x[1] + bias[0], -2.0 * x[0] + 3.0 * force[0] + 5.0 * bias[0]])

    def h(self, x, u, t=0, params=None):
        _, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)
        return xp.array([x[0] + 2.0 * bias[0]])

    def speed(self, x, u, t=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[1]])


class _JaxIncompatibleOutputSystem(_PortLinearSystem):
    def __init__(self):
        super().__init__()
        self.add_output_port("bad", dim=1, function=self.bad)

    def bad(self, x, u, t=0, params=None):
        return np.asarray([x[0]], dtype=float)


def _build_port_diagram():
    diagram = DiagramSystem()
    diagram.connection_verbose = False
    diagram.add_subsystem(_PortLinearSystem(), "plant")
    diagram.add_input_port("force")
    diagram.connect("input", "force", "plant", "force")
    diagram.connect_new_output_port("plant", "speed", "y_meas")
    return diagram


class TestLinearize(unittest.TestCase):
    def test_recovers_linear_system_matrices(self):
        plant = SingleMass(mass=2.0, k=3.0, b=0.5)
        lti = linearize(plant, x_bar=[0.0, 0.0])
        np.testing.assert_allclose(lti.A(), plant.A(), atol=1e-05)
        np.testing.assert_allclose(lti.B(), plant.B(), atol=1e-05)

    def test_pendulum_mode(self):
        lti = linearize(Pendulum(), x_bar=[0.0, 0.0])
        np.testing.assert_allclose(lti.A(), [[0.0, 1.0], [-4.905, 0.0]], atol=0.0001)

    def test_linearize_matrices_returns_raw_matrices(self):
        A, B, C, D = linearize_matrices(
            _PortLinearSystem(), x_bar=[1.0, 2.0], u_bar=[3.0, 4.0]
        )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(B, [[0.0, 1.0], [3.0, 5.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[0.0, 2.0]], atol=1e-06)

    def test_linearize_wrapper_matches_matrices(self):
        plant = _PortLinearSystem()
        A, B, C, D = linearize_matrices(plant, x_bar=[1.0, 2.0], u_bar=[3.0, 4.0])
        lti = linearize(plant, x_bar=[1.0, 2.0], u_bar=[3.0, 4.0])
        np.testing.assert_allclose(lti.A(), A)
        np.testing.assert_allclose(lti.B(), B)
        np.testing.assert_allclose(lti.C(), C)
        np.testing.assert_allclose(lti.D(), D)

    def test_selected_input_ports_reduce_b_and_d_columns(self):
        A, B, C, D = linearize_matrices(
            _PortLinearSystem(), x_bar=[1.0, 2.0], u_bar=[3.0, 4.0], inputs=["bias"]
        )
        self.assertEqual(B.shape, (2, 1))
        self.assertEqual(D.shape, (1, 1))
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(B, [[1.0], [5.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[2.0]], atol=1e-06)

    def test_selected_leaf_output_ports_reduce_c_and_d_rows(self):
        _, B, C, D = linearize_matrices(
            _PortLinearSystem(), x_bar=[1.0, 2.0], u_bar=[3.0, 4.0], outputs=["speed"]
        )
        self.assertEqual(C.shape, (1, 2))
        self.assertEqual(D.shape, (1, 2))
        np.testing.assert_allclose(B, [[0.0, 1.0], [3.0, 5.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[0.0, 1.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[0.0, 0.0]], atol=1e-06)

    def test_diagram_boundary_output_ports(self):
        diagram = _build_port_diagram()
        A, B, C, D = linearize_matrices(
            diagram, x_bar=[1.0, 2.0], u_bar=[3.0], outputs=["y_meas"]
        )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(B, [[0.0], [3.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[0.0, 1.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[0.0]], atol=1e-06)

    def test_diagram_internal_output_ports_fd(self):
        diagram = _build_port_diagram()
        A, B, C, D = linearize_matrices(
            diagram, x_bar=[1.0, 2.0], u_bar=[3.0], outputs=[("plant", "y")]
        )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(B, [[0.0], [3.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[0.0]], atol=1e-06)

    def test_jax_internal_output_falls_back_to_fd_with_warning(self):
        diagram = _build_port_diagram()
        with pytest.warns(LinearizationFallbackWarning):
            A, B, C, D = linearize_matrices(
                diagram,
                x_bar=[1.0, 2.0],
                u_bar=[3.0],
                outputs=[("plant", "y")],
                method="jax",
            )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(B, [[0.0], [3.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[0.0]], atol=1e-06)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_jax_selected_leaf_output_matches_fd(self):
        pytest.importorskip("jax")
        plant = _PortLinearSystem()
        fd = linearize_matrices(
            plant, x_bar=[1.0, 2.0], u_bar=[3.0, 4.0], outputs=["speed"], method="fd"
        )
        exact = linearize_matrices(
            plant, x_bar=[1.0, 2.0], u_bar=[3.0, 4.0], outputs=["speed"], method="jax"
        )
        for fd_matrix, exact_matrix in zip(fd, exact):
            np.testing.assert_allclose(fd_matrix, exact_matrix, atol=1e-06)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_jax_diagram_boundary_output_matches_fd(self):
        pytest.importorskip("jax")
        diagram = _build_port_diagram()
        fd = linearize_matrices(
            diagram, x_bar=[1.0, 2.0], u_bar=[3.0], outputs=["y_meas"], method="fd"
        )
        exact = linearize_matrices(
            diagram, x_bar=[1.0, 2.0], u_bar=[3.0], outputs=["y_meas"], method="jax"
        )
        for fd_matrix, exact_matrix in zip(fd, exact):
            np.testing.assert_allclose(fd_matrix, exact_matrix, atol=1e-06)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_incompatible_jax_output_falls_back_with_warning(self):
        pytest.importorskip("jax")
        with pytest.warns(LinearizationFallbackWarning):
            A, B, C, D = linearize_matrices(
                _JaxIncompatibleOutputSystem(),
                x_bar=[1.0, 2.0],
                u_bar=[3.0, 4.0],
                outputs=["bad"],
                method="jax",
            )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(B, [[0.0, 1.0], [3.0, 5.0]], atol=1e-06)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-06)
        np.testing.assert_allclose(D, [[0.0, 0.0]], atol=1e-06)


class TestStructural(unittest.TestCase):
    def test_double_integrator_is_controllable_observable(self):
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        C = np.array([[1.0, 0.0]])
        self.assertTrue(controllability(A, B).is_full_rank)
        self.assertTrue(observability(A, C).is_full_rank)

    def test_uncontrollable_pair_detected(self):
        A = np.array([[1.0, 0.0], [0.0, 2.0]])
        B = np.array([[1.0], [0.0]])
        result = controllability(A, B)
        self.assertFalse(result.is_full_rank)
        self.assertEqual(result.rank, 1)


class TestEquilibria(unittest.TestCase):
    def test_pendulum_hanging_equilibrium(self):
        x_eq = find_equilibrium(Pendulum(), x_guess=[0.3, 0.0])
        np.testing.assert_allclose(x_eq, [0.0, 0.0], atol=1e-06)


class TestLQR(unittest.TestCase):
    def test_gain_stabilizes_double_integrator(self):
        A = np.array([[0.0, 1.0], [0.0, 0.0]])
        B = np.array([[0.0], [1.0]])
        K = lqr_gain(A, B, Q=np.eye(2), R=np.array([[1.0]]))
        closed_poles = np.linalg.eigvals(A - B @ K)
        self.assertTrue(np.all(closed_poles.real < 0.0))

    def test_lqr_at_operating_point_matches_two_step_design(self):
        plant = InvertedPendulum()
        x_bar = [0.0, 0.0]
        Q = np.diag([10.0, 1.0])
        R = np.array([[1.0]])
        lti = linearize(plant, x_bar=x_bar)
        expected = lqr(lti.A(), lti.B(), Q, R, xbar=x_bar, ubar=[0.0])
        controller = lqr_at_operating_point(plant, x_bar, Q, R)
        np.testing.assert_allclose(controller.params["K"], expected.params["K"])
        np.testing.assert_allclose(
            controller.inputs["r"].nominal_value, expected.inputs["r"].nominal_value
        )

    def test_lqr_closed_loop_stabilizes_inverted_pendulum(self):
        plant = InvertedPendulum()
        controller = lqr_at_operating_point(
            plant, x_bar=[0.0, 0.0], Q=np.diag([10.0, 1.0]), R=np.array([[1.0]])
        )
        diagram = DiagramSystem()
        diagram.connection_verbose = False
        diagram.add_subsystem(controller, "ctl")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "x", "ctl", "x")
        diagram.connect("ctl", "u", "plant", "u")
        plant.x0 = np.array([0.2, 0.0])
        traj = diagram.compute_trajectory(tf=10.0, n_steps=1001, verbose=False)
        np.testing.assert_allclose(traj.x[:, -1], [0.0, 0.0], atol=0.01)


# from test_control_linear.py
from minilink.control.impedance import ImpedanceController, ImpedanceIntegralController
from minilink.control.output import ProportionalController
from minilink.control.siso import FilteredController
from minilink.control.state import StateFeedbackController
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator


class TestProportionalController(unittest.TestCase):
    def test_mimo_error_law(self):
        ctl = ProportionalController([[2.0, 0.0], [0.0, 3.0]])
        u = ctl.ctl(None, np.array([1.0, 1.0, 0.0, 0.0]))
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
        u = ctl.ctl(None, np.array([1.0, 1.0, 0.0, 0.0]))
        np.testing.assert_allclose(u, [0.5 - 3.0])


class TestImpedanceIntegralController(unittest.TestCase):
    def test_equations(self):
        pid = ImpedanceIntegralController()
        np.testing.assert_allclose(
            pid.f(np.array([0.0]), np.array([1.0, 0.2, 0.5])), [0.8]
        )
        np.testing.assert_allclose(
            pid.ctl(np.array([0.5]), np.array([1.0, 0.2, 0.5])), [8.0]
        )

    def test_closed_loop_removes_steady_state_error(self):
        plant = DoubleIntegrator()
        pid = ImpedanceIntegralController()
        pid.params.update({"kp": 5.0, "ki": 1.0, "kd": 4.0})
        setpoint = 1.0
        pid.inputs["r"].nominal_value = np.array([setpoint])
        diagram = DiagramSystem()
        diagram.connection_verbose = False
        diagram.add_subsystem(pid, "pid")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "x", "pid", "y")
        diagram.connect("pid", "u", "plant", "u")
        traj = diagram.compute_trajectory(tf=40.0, n_steps=4001, verbose=False)
        position = traj.x[1, -1]
        speed = traj.x[2, -1]
        self.assertAlmostEqual(position, setpoint, places=2)
        self.assertAlmostEqual(speed, 0.0, places=2)


class TestFilteredController(unittest.TestCase):
    def test_equations(self):
        pid = FilteredController(kp=10.0, ki=1.0, kd=1.0, tau=0.1)
        np.testing.assert_allclose(
            pid.f(np.array([0.0, 0.2]), np.array([1.0, 0.2])), [0.8, 0.0]
        )
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
        plant = DoubleIntegrator()
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
        traj = diagram.compute_trajectory(tf=40.0, n_steps=4001, verbose=False)
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
            [
                1.0 * (1.0 - 0.5) + 1.0 * (0.0 - 0.2),
                1.0 * (2.0 - 0.1) + 1.0 * (0.0 - 0.3),
            ],
        )


class TestFilteredControllerMIMO(unittest.TestCase):
    def test_dof_two_diagonal(self):
        pid = FilteredController(dof=2, kp=2.0, ki=1.0, kd=0.5, tau=0.1)
        np.testing.assert_allclose(
            pid.f(np.zeros(4), np.array([1.0, 2.0, 0.0, 0.0])), [1.0, 2.0, 0.0, 0.0]
        )


# from test_modal.py
import os
from minilink.analysis.linearize import linearize
from minilink.analysis.modal import animate_modal, modal_analysis
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.dynamics.catalog.mass_spring_damper.linear import TwoMass


class _JAXFriendlyOscillator(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=1)
        self.name = "JAX Friendly Oscillator"

    def f(self, x, u, t=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[1], -4.0 * x[0] + u[0]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[0]])


class TestModalAnalysis(unittest.TestCase):
    def test_two_mass_has_four_modes(self):
        poles, modes = modal_analysis(TwoMass(), x_bar=np.zeros(4))
        self.assertEqual(len(poles), 4)

    def test_pendulum_has_two_modes(self):
        poles, modes = modal_analysis(Pendulum(), x_bar=[0.0, 0.0])
        self.assertEqual(len(poles), 2)

    def test_oscillator_poles(self):
        w = 2.0
        A = np.array([[0.0, 1.0], [-(w**2), 0.0]])
        poles, modes = modal_analysis(LTISystem(A, np.zeros((2, 1))), x_bar=[0.0, 0.0])
        np.testing.assert_allclose(
            np.sort(poles), np.sort([1j * w, -1j * w]), atol=1e-10
        )


class TestAnimateModal(unittest.TestCase):
    def test_mode_trajectory_starts_at_eigenvector(self):
        x_bar = np.array([0.0, 0.0])
        poles, modes = modal_analysis(Pendulum(), x_bar)
        time = np.linspace(0.0, 2.0, 101)
        delta_x = 0.5 * np.real(modes[:, 0:1] * np.exp(poles[0] * time))
        expected = x_bar + delta_x[:, 0]
        np.testing.assert_allclose(
            expected, x_bar + 0.5 * np.real(modes[:, 0]), atol=1e-10
        )

    @pytest.mark.optional
    def test_animate_one_mode_headless(self):
        os.environ.setdefault("MPLBACKEND", "Agg")
        poles, modes = animate_modal(Pendulum(), [0.0, 0.0], 0, show=False)
        self.assertEqual(len(poles), 2)

    @pytest.mark.optional
    def test_animate_all_modes(self):
        os.environ.setdefault("MPLBACKEND", "Agg")
        poles, modes = animate_modal(Pendulum(), [0.0, 0.0], "all", show=False)
        self.assertEqual(len(poles), 2)


class TestModalFacade(unittest.TestCase):
    def test_returns_poles_and_modes(self):
        poles, modes = Pendulum().modal_analysis(x_bar=[0.0, 0.0])
        self.assertEqual(len(poles), 2)

    @pytest.mark.optional
    def test_facade_animate(self):
        os.environ.setdefault("MPLBACKEND", "Agg")
        poles, modes = Pendulum().modal_analysis(x_bar=[0.0, 0.0], mode=0, show=False)
        self.assertEqual(len(poles), 2)


@pytest.mark.optional
@pytest.mark.jax
class TestModalJAXLinearization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pytest.importorskip("jax")

    def test_jax_poles_match_fd(self):
        plant = _JAXFriendlyOscillator()
        x_bar = [0.0, 0.0]
        fd_poles, _ = modal_analysis(plant, x_bar, method="fd")
        jax_poles, _ = modal_analysis(plant, x_bar, method="jax")
        np.testing.assert_allclose(np.sort(fd_poles), np.sort(jax_poles), atol=0.0001)
        fd_lti = linearize(plant, x_bar, method="fd")
        jax_lti = linearize(plant, x_bar, method="jax")
        np.testing.assert_allclose(fd_lti.A(), jax_lti.A(), atol=0.0001)


class TestInvertedPendulumModes(unittest.TestCase):
    def test_upright_has_unstable_real_mode(self):
        poles, modes = modal_analysis(InvertedPendulum(), x_bar=[0.0, 0.0])
        self.assertTrue(any((pole.real > 0.0 for pole in poles)))


class TestModalAPI(unittest.TestCase):
    def test_linearization_keyword_removed(self):
        with self.assertRaises(TypeError):
            modal_analysis(Pendulum(), x_bar=[0.0, 0.0], linearization="fd")


# from test_frequency.py
from minilink.analysis.frequency import bode, pzmap
from minilink.graphical.common import PlotResult


class FrequencyPlant(DynamicSystem):
    def __init__(self):
        super().__init__(n=1)
        self.name = "Frequency Plant"
        self.add_input_port("force", dim=2, labels=["left_force", "right_force"])
        self.add_input_port("bias", labels=["bias"])
        self.add_output_port(
            "y",
            dim=2,
            function=self.h,
            dependencies="all",
            labels=["position", "speed"],
        )
        self.add_output_port(
            "sensor",
            dim=1,
            function=self.sensor,
            dependencies=("bias",),
            labels=["sensor"],
        )

    def f(self, x, u, t=0, params=None):
        force, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)
        dx = xp.array([-2.0 * x[0] + 3.0 * force[0] + 5.0 * force[1] + 7.0 * bias[0]])
        return dx

    def h(self, x, u, t=0, params=None):
        force, _ = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)
        y = xp.array([x[0] + 11.0 * force[0], 2.0 * x[0] + 13.0 * force[1]])
        return y

    def sensor(self, x, u, t=0, params=None):
        _, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)
        y = xp.array([4.0 * x[0] + 17.0 * bias[0]])
        return y


def build_frequency_diagram():
    diagram = DiagramSystem()
    diagram.connection_verbose = False
    diagram.add_subsystem(FrequencyPlant(), "plant")
    diagram.add_input_port("force", dim=2)
    diagram.add_input_port("bias")
    diagram.connect("input", "force", "plant", "force")
    diagram.connect("input", "bias", "plant", "bias")
    return diagram


def test_bode_selects_named_port_and_component():
    plant = FrequencyPlant()
    w, mag, phase = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0],
    )
    G = 13.0 + 10.0 / (2.0 + 1j)
    np.testing.assert_allclose(w, [1.0])
    np.testing.assert_allclose(mag, [20.0 * np.log10(abs(G))], atol=1e-06)
    np.testing.assert_allclose(phase, [np.angle(G) * 180.0 / np.pi], atol=1e-06)


def test_bode_selects_nonprimary_output_port():
    plant = FrequencyPlant()
    _, mag, phase = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="bias",
        output_port="sensor",
        w=[1.0],
    )
    G = 17.0 + 28.0 / (2.0 + 1j)
    np.testing.assert_allclose(mag, [20.0 * np.log10(abs(G))], atol=1e-06)
    np.testing.assert_allclose(phase, [np.angle(G) * 180.0 / np.pi], atol=1e-06)


def test_bode_selects_internal_diagram_output_port():
    diagram = build_frequency_diagram()
    _, mag, phase = bode(
        diagram,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="bias",
        output_port=("plant", "sensor"),
        w=[1.0],
    )
    G = 17.0 + 28.0 / (2.0 + 1j)
    np.testing.assert_allclose(mag, [20.0 * np.log10(abs(G))], atol=1e-06)
    np.testing.assert_allclose(phase, [np.angle(G) * 180.0 / np.pi], atol=1e-06)


def test_pzmap_returns_zeros_poles_and_gain_for_selected_channel():
    plant = FrequencyPlant()
    zeros, poles, gain = pzmap(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
    )
    np.testing.assert_allclose(zeros, [-36.0 / 13.0], atol=1e-06)
    np.testing.assert_allclose(poles, [-2.0], atol=1e-06)
    np.testing.assert_allclose(gain, 13.0, atol=1e-06)


@pytest.mark.optional
@pytest.mark.jax
def test_bode_jax_matches_fd_for_selected_channel():
    pytest.importorskip("jax")
    plant = FrequencyPlant()
    fd = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0, 10.0],
        method="fd",
    )
    exact = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0, 10.0],
        method="jax",
    )
    for fd_array, exact_array in zip(fd, exact):
        np.testing.assert_allclose(fd_array, exact_array, atol=1e-06)


@pytest.mark.optional
@pytest.mark.jax
def test_pzmap_jax_matches_fd_for_selected_channel():
    pytest.importorskip("jax")
    plant = FrequencyPlant()
    fd = pzmap(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        method="fd",
    )
    exact = pzmap(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        method="jax",
    )
    for fd_value, exact_value in zip(fd, exact):
        np.testing.assert_allclose(fd_value, exact_value, atol=1e-06)


def test_plot_bode_facade_returns_plot_result():
    import matplotlib.pyplot as plt

    plant = FrequencyPlant()
    result = plant.plot_bode(
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0, 10.0],
        show=False,
    )
    assert isinstance(result, PlotResult)
    assert len(result.axes) == 2
    assert "y[1] / force[1]" in result.axes[0].get_ylabel()
    plt.close(result.figure)


def test_plot_pzmap_facade_returns_plot_result():
    import matplotlib.pyplot as plt

    plant = FrequencyPlant()
    result = plant.plot_pzmap(
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        show=False,
    )
    assert isinstance(result, PlotResult)
    assert result.axes is not None
    assert "y[1] / force[1]" in result.axes.get_title()
    plt.close(result.figure)


# from test_phase_plane.py
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from minilink.core.trajectory import Trajectory
from minilink.graphical.phase_plane import build_phase_plane_spec, plot_phase_plane


class PhasePlaneTestSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, y_dependencies=())
        self.name = "PhasePlaneTestSystem"
        self.state.labels = ["position", "velocity"]
        self.state.units = ["m", "m/s"]
        self.state.lower_bound = np.array([-2.0, -3.0])
        self.state.upper_bound = np.array([2.0, 3.0])
        self.inputs["u"].set_nominal_value(np.array([0.5]))

    def f(self, x, u, t=0, params=None):
        return np.array([x[1] + u[0], -2.0 * x[0] + t])


class OneStateSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, output_dim=1, y_dependencies=())
        self.name = "OneStateSystem"
        self.state.lower_bound = np.array([-1.0])
        self.state.upper_bound = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.array([2.0 * x[0]])


class UnboundedPlanarSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, output_dim=2, y_dependencies=())

    def f(self, x, u, t=0, params=None):
        return np.array([x[1], -x[0]])


class TestPhasePlane(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_build_phase_plane_spec_evaluates_vector_field(self):
        sys = PhasePlaneTestSystem()
        spec = build_phase_plane_spec(
            sys,
            x_axis=0,
            y_axis=1,
            t=0.25,
            bounds=((-1.0, 1.0), (-2.0, 2.0)),
            grid_shape=(3, 2),
        )
        self.assertEqual(spec.X.shape, (2, 3))
        self.assertEqual(spec.Y.shape, (2, 3))
        self.assertEqual(spec.x_label, "position")
        self.assertEqual(spec.y_label, "velocity")
        self.assertEqual(spec.x_unit, "m")
        self.assertEqual(spec.y_unit, "m/s")
        np.testing.assert_allclose(spec.X[0, 2], 1.0)
        np.testing.assert_allclose(spec.Y[0, 2], -2.0)
        np.testing.assert_allclose(spec.V[0, 2], -1.5)
        np.testing.assert_allclose(spec.W[0, 2], -1.75)

    def test_one_state_system_can_use_same_axis_twice(self):
        sys = OneStateSystem()
        spec = build_phase_plane_spec(sys, x_axis=0, y_axis=0, grid_shape=(2, 2))
        self.assertEqual(spec.x_axis, 0)
        self.assertEqual(spec.y_axis, 0)
        np.testing.assert_allclose(spec.V, spec.W)
        np.testing.assert_allclose(spec.V, 2.0 * spec.X)

    def test_infinite_bounds_fall_back_to_trajectory_with_padding(self):
        sys = UnboundedPlanarSystem()
        traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[2.0, 4.0], [-1.0, 3.0]]),
            u=np.zeros((0, 2)),
        )
        spec = build_phase_plane_spec(sys, traj)
        np.testing.assert_allclose(spec.x_bounds, (1.9, 4.1))
        np.testing.assert_allclose(spec.y_bounds, (-1.2, 3.2))
        np.testing.assert_allclose(spec.trajectory.x, np.array([2.0, 4.0]))
        np.testing.assert_allclose(spec.trajectory.y, np.array([-1.0, 3.0]))

    def test_plot_phase_plane_matplotlib_with_overlay(self):
        sys = PhasePlaneTestSystem()
        traj = Trajectory(
            t=np.array([0.0, 1.0, 2.0]),
            x=np.array([[0.0, 0.5, 1.0], [1.0, 0.0, -1.0]]),
            u=np.zeros((1, 3)),
        )
        result = plot_phase_plane(
            sys, traj, bounds=((-1.0, 1.0), (-1.5, 1.5)), grid_shape=(3, 3), show=False
        )
        self.assertEqual(result.backend, "matplotlib")
        self.assertIsNotNone(result.figure)
        self.assertIs(result.axes.figure, result.figure)
        self.assertEqual(len(result.axes.lines), 3)

    def test_system_facade_uses_cached_trajectory(self):
        sys = PhasePlaneTestSystem()
        sys.traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0], [0.0, -1.0]]),
            u=np.zeros((1, 2)),
        )
        result = sys.plot_phase_plane(
            bounds=((-1.0, 1.0), (-1.0, 1.0)), grid_shape=(3, 3), show=False
        )
        self.assertEqual(result.backend, "matplotlib")
        self.assertEqual(len(result.axes.lines), 3)

    def test_unsupported_backend_reports_clear_error(self):
        sys = PhasePlaneTestSystem()
        with self.assertRaisesRegex(ValueError, "backend='plotly'.*not implemented"):
            plot_phase_plane(sys, backend="plotly", show=False)


# from test_discretize.py
from minilink.analysis.discretize import discretize


def _rk4_step(f, x, u, t, dt, params):
    k1 = f(x, u, t, params)
    k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
    k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
    k4 = f(x + dt * k3, u, t + dt, params)
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


class _GainIntegrator(DynamicSystem):
    def __init__(self, gain=1.0):
        super().__init__(n=1, input_dim=1, output_dim=1, expose_state=True)
        self.params = {"gain": float(gain), "dt": 0.05}

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        gain = p["gain"]
        return np.array([gain * u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])


class TestDiscretize(unittest.TestCase):
    def test_discretize_rk4_matches_source_step(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt)
        p = step_leaf.params
        x0 = np.array([0.2, -0.1])
        u = np.array([0.4])
        x1_ref = _rk4_step(plant.f, x0, u, 0.0, dt, p)
        x1 = step_leaf.step(x0, u, k=0)
        np.testing.assert_allclose(x1, x1_ref, rtol=1e-09, atol=1e-09)
        self.assertEqual(step_leaf.params["dt"], dt)

    def test_discretize_h_delegates_to_source(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt)
        p = step_leaf.params
        x = np.array([0.2, -0.1])
        u = np.array([0.4])
        np.testing.assert_allclose(
            step_leaf.h(x, u, k=2), plant.h(x, u, t=2 * dt, params=p)
        )

    def test_discretize_euler_matches_source_step(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt, method="euler")
        p = step_leaf.params
        x0 = np.array([0.2, -0.1])
        u = np.array([0.4])
        x1_ref = x0 + dt * plant.f(x0, u, 0.0, p)
        x1 = step_leaf.step(x0, u, k=0)
        np.testing.assert_allclose(x1, x1_ref, rtol=1e-09, atol=1e-09)
        self.assertEqual(step_leaf.method, "euler")

    def test_discretize_accepts_dt_in_params_only(self):
        step_leaf = discretize(_GainIntegrator(), params={"dt": 0.02})
        self.assertEqual(step_leaf.params["dt"], 0.02)

    def test_step_params_override_gain(self):
        plant = _GainIntegrator(gain=1.0)
        step_leaf = discretize(plant, 0.1)
        u = np.array([1.0])
        x0 = np.array([0.0])
        x_nom = step_leaf.step(x0, u, k=0)
        p_fast = {**step_leaf.params, "gain": 3.0}
        x_fast = step_leaf.step(x0, u, k=0, params=p_fast)
        self.assertGreater(x_fast[0], x_nom[0])

    def test_step_params_override_dt(self):
        plant = _GainIntegrator(gain=1.0)
        step_leaf = discretize(plant, 0.1)
        u = np.array([1.0])
        x0 = np.array([0.0])
        p_short = {**step_leaf.params, "dt": 0.02}
        p_long = {**step_leaf.params, "dt": 0.2}
        x_short = step_leaf.step(x0, u, k=0, params=p_short)
        x_long = step_leaf.step(x0, u, k=0, params=p_long)
        self.assertLess(x_short[0], x_long[0])

    def test_discretize_rejects_unknown_method(self):
        plant = DoubleIntegrator()
        with self.assertRaises(ValueError):
            discretize(plant, 0.01, method="bdf")

    def test_discretize_rejects_missing_dt(self):
        plant = DoubleIntegrator()
        with self.assertRaises(ValueError):
            discretize(plant)

    def test_discretize_rejects_static_system(self):
        with self.assertRaises(TypeError):
            discretize(ProportionalController(1.0), dt=0.01)


# from test_state_space_system.py
from minilink.dynamics.abstraction.state_space import LTISystem, StateSpaceSystem


class TestLTISystem(unittest.TestCase):
    def test_defaults_to_full_state_output(self):
        A = np.array([[0.0, 1.0], [-2.0, -3.0]])
        B = np.array([[0.0], [1.0]])
        sys = LTISystem(A, B)
        self.assertEqual(sys.n, 2)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.p, 2)
        np.testing.assert_allclose(sys.C(), np.eye(2))
        np.testing.assert_allclose(sys.D(), np.zeros((2, 1)))
        self.assertEqual(sys.outputs["y"].dependencies, ())
        x = np.array([1.0, 2.0])
        u = np.array([0.5])
        np.testing.assert_allclose(sys.f(x, u), np.array([2.0, -7.5]))
        np.testing.assert_allclose(sys.h(x, u), x)

    def test_explicit_output_matrices(self):
        A = np.array([[0.0, 1.0], [-1.0, -0.2]])
        B = np.array([[0.0], [2.0]])
        C = np.array([[1.0, -1.0]])
        D = np.array([[3.0]])
        sys = LTISystem(A, B, C, D)
        self.assertIs(sys.A(), A)
        self.assertIs(sys.B(), B)
        self.assertIs(sys.C(), C)
        self.assertIs(sys.D(), D)
        self.assertEqual(sys.p, 1)
        self.assertEqual(sys.outputs["y"].dependencies, ("u",))
        x = np.array([2.0, -4.0])
        u = np.array([0.5])
        np.testing.assert_allclose(sys.f(x, u), np.array([-4.0, -0.2]))
        np.testing.assert_allclose(sys.h(x, u), np.array([7.5]))

    def test_constant_matrices_ignore_params(self):
        sys = LTISystem(np.array([[2.0]]), np.array([[3.0]]))
        np.testing.assert_allclose(
            sys.f(np.array([4.0]), np.array([5.0]), params={"A": np.array([[0.0]])}),
            np.array([23.0]),
        )

    def test_rejects_invalid_dimensions(self):
        with self.assertRaisesRegex(ValueError, "A must be"):
            LTISystem(np.array([[1.0, 2.0]]), np.array([[1.0], [2.0]]))
        with self.assertRaisesRegex(ValueError, "B row count"):
            LTISystem(np.eye(2), np.ones((3, 1)))
        with self.assertRaisesRegex(ValueError, "C column count"):
            LTISystem(np.eye(2), np.ones((2, 1)), C=np.ones((1, 3)))
        with self.assertRaisesRegex(ValueError, "C must be"):
            LTISystem(np.eye(2), np.ones((2, 1)), C=np.ones(2))
        with self.assertRaisesRegex(ValueError, "D shape"):
            LTISystem(np.eye(2), np.ones((2, 1)), C=np.ones((1, 2)), D=np.ones((2, 1)))

    @pytest.mark.optional
    @pytest.mark.jax
    def test_jax_arrays_are_preserved_and_traceable_if_available(self):
        jax = pytest.importorskip("jax")
        jnp = pytest.importorskip("jax.numpy")
        A = jnp.array([[0.0, 1.0], [-2.0, -3.0]])
        B = jnp.array([[0.0], [1.0]])
        sys = LTISystem(A, B)
        self.assertIs(sys.A(), A)
        self.assertIs(sys.B(), B)
        x = jnp.array([1.0, 2.0])
        u = jnp.array([0.5])
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        jax.make_jaxpr(lambda xx, uu: sys.h(xx, uu))(x, u)
        np.testing.assert_allclose(np.asarray(sys.f(x, u)), np.array([2.0, -7.5]))
        np.testing.assert_allclose(np.asarray(sys.h(x, u)), np.array([1.0, 2.0]))


class TestStateSpaceSystem(unittest.TestCase):
    def test_subclass_builds_matrices_from_params(self):

        class Gain(StateSpaceSystem):
            def __init__(self):
                super().__init__(n=1, m=1, p=1, name="Gain")
                self.params = {"a": -2.0, "b": 3.0}

            def A(self, t=0.0, params=None):
                params = self.params if params is None else params
                return np.array([[params["a"]]])

            def B(self, t=0.0, params=None):
                params = self.params if params is None else params
                return np.array([[params["b"]]])

        sys = Gain()
        x = np.array([4.0])
        u = np.array([5.0])
        np.testing.assert_allclose(sys.f(x, u), np.array([-8.0 + 15.0]))
        np.testing.assert_allclose(sys.h(x, u), x)
        np.testing.assert_allclose(
            sys.f(x, u, params={"a": 0.0, "b": 1.0}), np.array([5.0])
        )
