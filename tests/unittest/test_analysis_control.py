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

        # dx = A x + B u
        return xp.array([x[1] + bias[0], -2.0 * x[0] + 3.0 * force[0] + 5.0 * bias[0]])

    def h(self, x, u, t=0, params=None):
        _, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)

        # y = x0 + 2 bias
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
        # Linearizing a linear plant must return its own A, B (to fd tolerance).
        plant = SingleMass(mass=2.0, k=3.0, b=0.5)
        lti = linearize(plant, x_bar=[0.0, 0.0])
        np.testing.assert_allclose(lti.A(), plant.A(), atol=1e-5)
        np.testing.assert_allclose(lti.B(), plant.B(), atol=1e-5)

    def test_pendulum_mode(self):
        lti = linearize(Pendulum(), x_bar=[0.0, 0.0])
        # downward pendulum: undamped oscillator, A = [[0,1],[-g l m/(m l^2+I),0]]
        np.testing.assert_allclose(lti.A(), [[0.0, 1.0], [-4.905, 0.0]], atol=1e-4)

    def test_linearize_matrices_returns_raw_matrices(self):
        A, B, C, D = linearize_matrices(
            _PortLinearSystem(), x_bar=[1.0, 2.0], u_bar=[3.0, 4.0]
        )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(B, [[0.0, 1.0], [3.0, 5.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[0.0, 2.0]], atol=1e-6)

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
            _PortLinearSystem(),
            x_bar=[1.0, 2.0],
            u_bar=[3.0, 4.0],
            inputs=["bias"],
        )
        self.assertEqual(B.shape, (2, 1))
        self.assertEqual(D.shape, (1, 1))
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(B, [[1.0], [5.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[2.0]], atol=1e-6)

    def test_selected_leaf_output_ports_reduce_c_and_d_rows(self):
        _, B, C, D = linearize_matrices(
            _PortLinearSystem(),
            x_bar=[1.0, 2.0],
            u_bar=[3.0, 4.0],
            outputs=["speed"],
        )
        self.assertEqual(C.shape, (1, 2))
        self.assertEqual(D.shape, (1, 2))
        np.testing.assert_allclose(B, [[0.0, 1.0], [3.0, 5.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[0.0, 1.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[0.0, 0.0]], atol=1e-6)

    def test_diagram_boundary_output_ports(self):
        diagram = _build_port_diagram()
        A, B, C, D = linearize_matrices(
            diagram,
            x_bar=[1.0, 2.0],
            u_bar=[3.0],
            outputs=["y_meas"],
        )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(B, [[0.0], [3.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[0.0, 1.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[0.0]], atol=1e-6)

    def test_diagram_internal_output_ports_fd(self):
        diagram = _build_port_diagram()
        A, B, C, D = linearize_matrices(
            diagram,
            x_bar=[1.0, 2.0],
            u_bar=[3.0],
            outputs=[("plant", "y")],
        )
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(B, [[0.0], [3.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[0.0]], atol=1e-6)

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
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(B, [[0.0], [3.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[0.0]], atol=1e-6)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_jax_selected_leaf_output_matches_fd(self):
        pytest.importorskip("jax")
        plant = _PortLinearSystem()
        fd = linearize_matrices(
            plant,
            x_bar=[1.0, 2.0],
            u_bar=[3.0, 4.0],
            outputs=["speed"],
            method="fd",
        )
        exact = linearize_matrices(
            plant,
            x_bar=[1.0, 2.0],
            u_bar=[3.0, 4.0],
            outputs=["speed"],
            method="jax",
        )
        for fd_matrix, exact_matrix in zip(fd, exact):
            np.testing.assert_allclose(fd_matrix, exact_matrix, atol=1e-6)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_jax_diagram_boundary_output_matches_fd(self):
        pytest.importorskip("jax")
        diagram = _build_port_diagram()
        fd = linearize_matrices(
            diagram,
            x_bar=[1.0, 2.0],
            u_bar=[3.0],
            outputs=["y_meas"],
            method="fd",
        )
        exact = linearize_matrices(
            diagram,
            x_bar=[1.0, 2.0],
            u_bar=[3.0],
            outputs=["y_meas"],
            method="jax",
        )
        for fd_matrix, exact_matrix in zip(fd, exact):
            np.testing.assert_allclose(fd_matrix, exact_matrix, atol=1e-6)

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
        np.testing.assert_allclose(A, [[0.0, 1.0], [-2.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(B, [[0.0, 1.0], [3.0, 5.0]], atol=1e-6)
        np.testing.assert_allclose(C, [[1.0, 0.0]], atol=1e-6)
        np.testing.assert_allclose(D, [[0.0, 0.0]], atol=1e-6)


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
            plant,
            x_bar=[0.0, 0.0],
            Q=np.diag([10.0, 1.0]),
            R=np.array([[1.0]]),
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
