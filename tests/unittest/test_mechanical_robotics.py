import unittest
import numpy as np
import pytest
from minilink.dynamics.abstraction.mechanical import MechanicalSystem


class TestMechanicalSystem(unittest.TestCase):
    def test_dimensions_fully_actuated(self):
        sys = MechanicalSystem(dof=2)
        self.assertEqual(sys.dof, 2)
        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.p, 4)

    def test_dimensions_custom_actuators(self):
        sys = MechanicalSystem(dof=3, actuators=2)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.n, 6)

    def test_f_double_integrator_chain(self):
        """Default H=I, C=g=d=0, u=0 => acceleration=0, dx = [dq; 0]."""
        sys = MechanicalSystem(dof=2)
        x = np.array([0.1, -0.2, 0.3, 0.4])
        u = np.zeros(2)
        dx = sys.f(x, u, t=0.0)
        self.assertEqual(dx.shape, (4,))
        np.testing.assert_allclose(dx[:2], x[2:4])
        np.testing.assert_allclose(dx[2:4], 0.0)

    def test_f_constant_acceleration(self):
        sys = MechanicalSystem(dof=1)
        x = np.array([0.0, 0.0])
        u = np.array([2.0])
        dx = sys.f(x, u)
        np.testing.assert_allclose(dx, np.array([0.0, 2.0]))

    def test_f_uses_explicit_params_in_matrix_hooks(self):

        class MassSystem(MechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def H(self, q, params=None):
                return np.array([[params["mass"]]])

        sys = MassSystem()
        x = np.array([0.0, 0.0])
        u = np.array([8.0])
        np.testing.assert_allclose(sys.f(x, u), np.array([0.0, 4.0]))
        np.testing.assert_allclose(
            sys.f(x, u, params={"mass": 4.0}), np.array([0.0, 2.0])
        )

    def test_inverse_dynamics_uses_explicit_params_in_matrix_hooks(self):

        class MassSystem(MechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def H(self, q, params=None):
                return np.array([[params["mass"]]])

        sys = MassSystem()
        q = np.array([0.0])
        dq = np.array([0.0])
        acceleration = np.array([3.0])
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, dq, acceleration), np.array([6.0])
        )
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, dq, acceleration, params={"mass": 5.0}),
            np.array([15.0]),
        )

    def test_default_generalized_force_is_actuator_map(self):
        sys = MechanicalSystem(dof=2, actuators=1)
        q = np.array([0.0, 0.0])
        dq = np.array([0.0, 0.0])
        u = np.array([3.0])
        np.testing.assert_allclose(
            sys.generalized_force(q, dq, u), np.array([3.0, 0.0])
        )

    def test_forward_and_inverse_dynamics_are_consistent(self):

        class MassSystem(MechanicalSystem):
            def H(self, q, params=None):
                return np.array([[2.0]])

            def d(self, q, dq, u=None, t=0.0, params=None):
                return np.array([0.5 * dq[0]])

        sys = MassSystem(dof=1)
        q = np.array([0.0])
        dq = np.array([4.0])
        u = np.array([10.0])
        acceleration = sys.forward_dynamics(q, dq, u)
        np.testing.assert_allclose(acceleration, np.array([4.0]))
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, dq, acceleration, u),
            sys.generalized_force(q, dq, u),
        )

    def test_h_equals_state(self):
        sys = MechanicalSystem(dof=1)
        x = np.array([1.0, 2.0])
        y = sys.h(x, np.zeros(1))
        np.testing.assert_array_equal(y, x)

    def test_y_port_dependencies(self):
        sys = MechanicalSystem(dof=1)
        self.assertEqual(sys.outputs["y"].dependencies, ())

    @pytest.mark.optional
    @pytest.mark.jax
    def test_default_base_is_jax_traceable_if_available(self):
        jax = pytest.importorskip("jax")
        jnp = pytest.importorskip("jax.numpy")
        sys = MechanicalSystem(dof=2)
        x = jnp.array([0.1, -0.2, 0.3, 0.4])
        u = jnp.array([1.0, -2.0])
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        np.testing.assert_allclose(
            np.asarray(sys.f(x, u)), np.array([0.3, 0.4, 1.0, -2.0])
        )


class TestJaxMechanicalInheritance(unittest.TestCase):
    def test_jax_mechanical_subclasses_numpy_mechanical(self):
        from minilink.dynamics.abstraction.mechanical import (
            JaxMechanicalSystem,
            MechanicalSystem,
        )

        self.assertTrue(issubclass(JaxMechanicalSystem, MechanicalSystem))

    def test_jax_mechanical_init_matches_numpy_layout(self):
        from minilink.dynamics.abstraction.mechanical import (
            JaxMechanicalSystem,
            MechanicalSystem,
        )

        a = MechanicalSystem(dof=3, actuators=2)
        b = JaxMechanicalSystem(dof=3, actuators=2)
        self.assertEqual((a.n, a.m, a.p), (b.n, b.m, b.p))
        self.assertEqual(a.state.labels, b.state.labels)


from minilink.dynamics.abstraction.generalized_mechanical import (
    GeneralizedMechanicalSystem,
)


class TestGeneralizedMechanicalSystem(unittest.TestCase):
    def test_dimensions_default_to_square_configuration_velocity_system(self):
        sys = GeneralizedMechanicalSystem(dof=2)
        self.assertEqual(sys.dof, 2)
        self.assertEqual(sys.pos, 2)
        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.p, 4)

    def test_default_dynamics_are_double_integrator_chain(self):
        sys = GeneralizedMechanicalSystem(dof=2)
        x = np.array([0.1, -0.2, 0.3, 0.4])
        u = np.array([1.0, -2.0])
        dx = sys.f(x, u)
        np.testing.assert_allclose(dx, np.array([0.3, 0.4, 1.0, -2.0]))

    def test_nontrivial_kinematic_map(self):

        class PlanarBody(GeneralizedMechanicalSystem):
            def __init__(self):
                super().__init__(dof=3, pos=3, actuators=0)

            def N(self, q, params=None):
                theta = q[2]
                c, s = (np.cos(theta), np.sin(theta))
                return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

        sys = PlanarBody()
        q = np.array([1.0, 2.0, np.pi / 2.0])
        v = np.array([1.0, 2.0, 3.0])
        x = sys.qv2x(q, v)
        u = np.zeros(0)
        dx = sys.f(x, u)
        np.testing.assert_allclose(dx[:3], np.array([-2.0, 1.0, 3.0]), atol=1e-12)
        np.testing.assert_allclose(dx[3:], np.zeros(3))

    def test_mixed_inputs_use_generalized_force_hook(self):

        class MixedInputBody(GeneralizedMechanicalSystem):
            def __init__(self):
                super().__init__(dof=2, pos=2, actuators=2)

            def generalized_force(self, q, v, u, t=0.0, params=None):
                thrust = u[0]
                angle = u[1]
                return np.array([thrust * np.cos(angle), thrust * np.sin(angle)])

        sys = MixedInputBody()
        x = np.zeros(4)
        u = np.array([2.0, np.pi / 2.0])
        dx = sys.f(x, u)
        np.testing.assert_allclose(dx[:2], np.zeros(2))
        np.testing.assert_allclose(dx[2:], np.array([0.0, 2.0]), atol=1e-12)

    def test_explicit_params_reach_matrix_hooks(self):

        class MassBody(GeneralizedMechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def M(self, q, params=None):
                return np.array([[params["mass"]]])

        sys = MassBody()
        x = np.array([0.0, 0.0])
        u = np.array([8.0])
        np.testing.assert_allclose(sys.f(x, u), np.array([0.0, 4.0]))
        np.testing.assert_allclose(
            sys.f(x, u, params={"mass": 4.0}), np.array([0.0, 2.0])
        )

    def test_default_generalized_force_is_actuator_map(self):
        sys = GeneralizedMechanicalSystem(dof=2, pos=2, actuators=1)
        q = np.array([0.0, 0.0])
        v = np.array([0.0, 0.0])
        u = np.array([3.0])
        np.testing.assert_allclose(sys.generalized_force(q, v, u), np.array([3.0, 0.0]))

    def test_forward_and_inverse_dynamics_are_consistent(self):

        class MassBody(GeneralizedMechanicalSystem):
            def M(self, q, params=None):
                return np.array([[2.0]])

            def d(self, q, v, u=None, t=0.0, params=None):
                return np.array([0.5 * v[0]])

        sys = MassBody(dof=1)
        q = np.array([0.0])
        v = np.array([4.0])
        u = np.array([10.0])
        acceleration = sys.forward_dynamics(q, v, u)
        np.testing.assert_allclose(acceleration, np.array([4.0]))
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, v, acceleration, u), sys.generalized_force(q, v, u)
        )

    def test_h_equals_state(self):
        sys = GeneralizedMechanicalSystem(dof=1)
        x = np.array([1.0, 2.0])
        y = sys.h(x, np.zeros(1))
        np.testing.assert_array_equal(y, x)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_default_base_is_jax_traceable_if_available(self):
        jax = pytest.importorskip("jax")
        jnp = pytest.importorskip("jax.numpy")
        sys = GeneralizedMechanicalSystem(dof=2)
        x = jnp.array([0.1, -0.2, 0.3, 0.4])
        u = jnp.array([1.0, -2.0])
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        np.testing.assert_allclose(
            np.asarray(sys.f(x, u)), np.array([0.3, 0.4, 1.0, -2.0])
        )


pytest.importorskip("jax")
import jax
import jax.numpy as jnp
from minilink.core.backends import configure_jax
from minilink.dynamics.abstraction.mechanical import (
    JaxMechanicalSystem,
    MechanicalSystem,
)


@pytest.mark.optional
@pytest.mark.jax
class TestMechanicalSystemJax(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def test_f_is_jaxpr_traceable(self):
        sys = JaxMechanicalSystem(dof=2)
        x = jnp.zeros(4)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_f_matches_numpy_1dof_linear_regime(self):
        sys_j = JaxMechanicalSystem(dof=1)
        sys_n = MechanicalSystem(dof=1)
        x = jnp.array([0.2, -0.1])
        u = jnp.zeros(1)
        dx_j = sys_j.f(x, u)
        xn = np.array([0.2, -0.1])
        un = np.zeros(1)
        dx_n = sys_n.f(xn, un)
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-05, atol=1e-05)

    def test_f_matches_numpy_2dof_nontrivial_regime(self):

        class _NumpyTwoLink(MechanicalSystem):
            def H(self, q, params=None):
                m1, m2, l = (1.5, 0.7, 0.4)
                c2 = np.cos(q[1])
                return np.array(
                    [
                        [m1 + m2 + 2 * m2 * l * c2, m2 + m2 * l * c2],
                        [m2 + m2 * l * c2, m2],
                    ]
                )

            def C(self, q, dq, params=None):
                m2, l = (0.7, 0.4)
                s2 = np.sin(q[1])
                return np.array(
                    [
                        [-m2 * l * s2 * dq[1], -m2 * l * s2 * (dq[0] + dq[1])],
                        [m2 * l * s2 * dq[0], 0.0],
                    ]
                )

            def g(self, q, params=None):
                return np.array([2.0 * np.sin(q[0]), 1.0 * np.sin(q[1])])

            def d(self, q, dq, u=None, t=0.0, params=None):
                return 0.05 * np.asarray(dq)

        class _JaxTwoLink(JaxMechanicalSystem):
            def H(self, q, params=None):
                m1, m2, l = (1.5, 0.7, 0.4)
                c2 = jnp.cos(q[1])
                return jnp.array(
                    [
                        [m1 + m2 + 2 * m2 * l * c2, m2 + m2 * l * c2],
                        [m2 + m2 * l * c2, m2],
                    ]
                )

            def C(self, q, dq, params=None):
                m2, l = (0.7, 0.4)
                s2 = jnp.sin(q[1])
                return jnp.array(
                    [
                        [-m2 * l * s2 * dq[1], -m2 * l * s2 * (dq[0] + dq[1])],
                        [m2 * l * s2 * dq[0], 0.0],
                    ]
                )

            def g(self, q, params=None):
                return jnp.array([2.0 * jnp.sin(q[0]), 1.0 * jnp.sin(q[1])])

            def d(self, q, dq, u=None, t=0.0, params=None):
                return 0.05 * dq

        sys_n = _NumpyTwoLink(dof=2)
        sys_j = _JaxTwoLink(dof=2)
        x = np.array([0.5, -0.7, 0.3, -0.2])
        u = np.array([0.4, -0.6])
        dx_n = sys_n.f(x, u)
        dx_j = np.asarray(sys_j.f(jnp.asarray(x), jnp.asarray(u)))
        np.testing.assert_allclose(dx_j, dx_n, rtol=1e-09, atol=1e-09)

    def test_f_uses_explicit_params_in_matrix_hooks(self):

        class MassSystem(JaxMechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def H(self, q, params=None):
                return jnp.array([[params["mass"]]])

        sys = MassSystem()
        x = jnp.array([0.0, 0.0])
        u = jnp.array([8.0])
        np.testing.assert_allclose(np.asarray(sys.f(x, u)), np.array([0.0, 4.0]))
        np.testing.assert_allclose(
            np.asarray(sys.f(x, u, params={"mass": 4.0})), np.array([0.0, 2.0])
        )

    def test_jax_pendulum_subclasses_numpy_reference(self):
        from benchmarks.systems.basic import JaxPendulum, NumpyPendulum

        self.assertTrue(issubclass(JaxPendulum, NumpyPendulum))
        p = JaxPendulum(gravity=9.81, length=1.0, damping=0.05)
        self.assertEqual(p.n, 2)

    def test_jax_pendulum_f_matches_numpy(self):
        from benchmarks.systems.basic import JaxPendulum, NumpyPendulum

        np_sys = NumpyPendulum(gravity=9.81, length=1.0, damping=0.1)
        jx_sys = JaxPendulum(gravity=9.81, length=1.0, damping=0.1)
        x = np.array([0.7, -0.3])
        u = np.array([0.5])
        dx_jx = np.asarray(jx_sys.f(jnp.asarray(x), jnp.asarray(u)))
        np.testing.assert_allclose(dx_jx, np_sys.f(x, u), rtol=1e-09, atol=1e-09)


from minilink.dynamics.abstraction.manipulator import Manipulator


class TestMechanicalJointPorts(unittest.TestCase):
    def test_q_and_dq_ports_match_state(self):
        sys = MechanicalSystem(dof=2)
        x = np.array([0.1, -0.2, 0.3, 0.4])
        q = sys.h_q(x, np.zeros(sys.m))
        dq = sys.h_dq(x, np.zeros(sys.m))
        np.testing.assert_allclose(q, x[:2])
        np.testing.assert_allclose(dq, x[2:])
        self.assertEqual(sys.outputs["q"].dim, 2)
        self.assertEqual(sys.outputs["dq"].dim, 2)


class TestManipulator(unittest.TestCase):
    def test_default_kinematics_ports_are_zero(self):
        sys = Manipulator(dof=2, task_dim=2)
        x = np.array([0.2, -0.1, 0.0, 0.0])
        u = np.zeros(sys.m)
        p = sys.h_p(x, u)
        pdot = sys.h_pdot(x, u)
        np.testing.assert_allclose(p, np.zeros(2))
        np.testing.assert_allclose(pdot, np.zeros(2))

    def test_subclass_forward_kinematics_and_jacobian(self):

        class TwoLinkStub(Manipulator):
            def forward_kinematics(self, q, params=None):
                return np.array([q[0], q[0] + q[1]])

            def J(self, q, params=None):
                return np.array([[1.0, 0.0], [1.0, 1.0]])

        sys = TwoLinkStub(dof=2, task_dim=2)
        q = np.array([0.5, -0.25])
        dq = np.array([1.0, 2.0])
        x = sys.q2x(q, dq)
        u = np.zeros(sys.m)
        np.testing.assert_allclose(sys.h_p(x, u), np.array([0.5, 0.25]))
        np.testing.assert_allclose(sys.h_pdot(x, u), np.array([1.0, 3.0]))


from minilink.blocks.sources import Source
from minilink.control.robotic import (
    JointImpedance,
    ModelJointImpedance,
    TaskImpedance,
    TaskKinematic,
    TaskKinematicNullspace,
)
from minilink.core.composition import closed_loop, closed_loop_qdq
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    OneLinkManipulator,
    SpeedControlledManipulator,
    TwoLinkManipulator,
)
from minilink.dynamics.catalog.manipulators import UR5Manipulator
from minilink.graphical.animation.primitives import Arrow
from tests.unittest.graphics_contract_helpers import resolve_draw_frame


class TestManipulatorPorts(unittest.TestCase):
    def test_h_p_matches_forward_kinematics(self):
        arm = OneLinkManipulator()
        q = np.array([0.3])
        x = arm.q2x(q, np.zeros(1))
        np.testing.assert_allclose(arm.h_p(x, np.zeros(1)), arm.forward_kinematics(q))

    def test_h_pdot_matches_jacobian(self):
        arm = TwoLinkManipulator()
        q = np.array([0.2, -0.1])
        dq = np.array([0.5, -0.3])
        x = arm.q2x(q, dq)
        np.testing.assert_allclose(arm.h_pdot(x, np.zeros(2)), arm.J(q) @ dq)


class TestRoboticWrappers(unittest.TestCase):
    def test_joint_impedance_closed_loop_qdq(self):
        diagram = closed_loop_qdq(JointImpedance(dof=2), TwoLinkManipulator())
        self.assertIn("mux", diagram.subsystems)
        self.assertEqual(diagram.connections["ctl"]["y"], ("mux", "y"))

    def test_model_joint_impedance_gravity_at_setpoint(self):
        arm = TwoLinkManipulator()
        ctl = JointImpedance(arm, gravity_comp=True)
        q = np.array([0.8, -0.5])
        r = q.copy()
        u = ctl.ctl(None, np.concatenate([r, q, np.zeros(2)]))
        np.testing.assert_allclose(u, arm.g(q))

    def test_model_joint_impedance_no_gravity_by_default(self):
        arm = TwoLinkManipulator()
        ctl = JointImpedance(arm)
        q = np.array([0.8, -0.5])
        u = ctl.ctl(None, np.concatenate([q, q, np.zeros(2)]))
        np.testing.assert_allclose(u, np.zeros(2))

    def test_model_joint_impedance_custom_gravity_hook(self):
        arm = TwoLinkManipulator()
        ctl = ModelJointImpedance(
            arm, gravity_comp=True, gravity=lambda q: np.array([1.0, 2.0])
        )
        q = np.array([0.2, 0.3])
        u = ctl.ctl(None, np.concatenate([q, q, np.zeros(2)]))
        np.testing.assert_allclose(u, [1.0, 2.0])

    def test_joint_impedance_gravity_requires_plant(self):
        with self.assertRaises(ValueError):
            JointImpedance(dof=2, gravity_comp=True)

    def test_task_impedance_uses_internal_kinematics(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(arm)
        q = np.array([0.2, -0.1])
        dq = np.array([0.1, 0.05])
        p_d = arm.forward_kinematics(np.array([0.0, 0.0]))
        u = ctl.ctl(None, np.concatenate([p_d, q, dq]))
        self.assertEqual(u.shape, (2,))
        p = arm.forward_kinematics(q)
        J = arm.J(q)
        pdot = J @ dq
        e_p = p_d - p
        expected = J.T @ (ctl.params["Kp"] * e_p - ctl.params["Kd"] * pdot)
        np.testing.assert_allclose(u, expected)

    def test_task_impedance_gravity_feedforward(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(arm, gravity_comp=True)
        q = np.array([0.8, -0.5])
        p_d = arm.forward_kinematics(q)
        u = ctl.ctl(None, np.concatenate([p_d, q, np.zeros(2)]))
        np.testing.assert_allclose(u, arm.g(q))

    def test_task_impedance_closed_loop_qdq(self):
        diagram = closed_loop_qdq(
            TaskImpedance(TwoLinkManipulator()), TwoLinkManipulator()
        )
        self.assertIn("mux", diagram.subsystems)
        self.assertEqual(diagram.connections["ctl"]["y"], ("mux", "y"))

    def test_task_impedance_force_arrow_off_by_default(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(arm)
        q = np.array([0.2, -0.1])
        p_d = arm.forward_kinematics(np.zeros(2))
        u = np.concatenate([p_d, q, np.zeros(2)])
        self.assertEqual(ctl.tf(None, u), {})
        self.assertEqual(ctl.get_dynamic_geometry(None, u), {})

    def test_task_impedance_force_arrow_matches_task_wrench(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(
            arm, gravity_comp=True, show_task_force=True, task_force_scale=0.02
        )
        q = np.array([0.4, -0.2])
        dq = np.array([0.1, -0.05])
        p_d = arm.forward_kinematics(np.zeros(2))
        u = np.concatenate([p_d, q, dq])

        p = arm.forward_kinematics(q)
        J = arm.J(q)
        pdot = J @ dq
        f_task = ctl.params["Kp"] * (p_d - p) - ctl.params["Kd"] * pdot
        tau = ctl.ctl(None, u)
        np.testing.assert_allclose(tau, J.T @ f_task + arm.g(q))

        frames = ctl.tf(None, u)
        np.testing.assert_allclose(frames["task_force"][:2, 3], p)
        np.testing.assert_allclose(frames["task_force"][2, 3], 0.0)
        geom = ctl.get_dynamic_geometry(None, u)
        arrow = geom["task_force"][0]
        self.assertIsInstance(arrow, Arrow)
        np.testing.assert_allclose(arrow.vector, f_task)
        self.assertEqual(arrow.scale, 0.02)
        # Gravity must not appear in the visualized task force.
        self.assertFalse(np.allclose(arrow.vector, tau))

    def test_task_impedance_force_arrow_in_diagram_and_ur5(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(arm, show_task_force=True)
        diagram = closed_loop_qdq(ctl, arm)
        q = np.array([0.3, -0.15])
        p_d = arm.forward_kinematics(np.zeros(2))
        x = arm.q2x(q, np.zeros(2))
        # Diagram external input is controller reference ``r``.
        frames = diagram.tf(x, p_d)
        geom = diagram.get_dynamic_geometry(x, p_d)
        self.assertIn("ctl:task_force", frames)
        self.assertIn("ctl:task_force", geom)
        np.testing.assert_allclose(
            frames["ctl:task_force"][:2, 3], arm.forward_kinematics(q)
        )
        resolve_draw_frame(diagram, x, p_d)

        ur5 = UR5Manipulator()
        ctl3 = TaskImpedance(ur5, show_task_force=True, task_force_scale=0.005)
        q3 = np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
        p_d3 = ur5.forward_kinematics(q3) + np.array([0.05, -0.02, 0.03])
        u3 = np.concatenate([p_d3, q3, np.zeros(6)])
        frames3 = ctl3.tf(None, u3)
        geom3 = ctl3.get_dynamic_geometry(None, u3)
        np.testing.assert_allclose(
            frames3["task_force"][:3, 3], ur5.forward_kinematics(q3)
        )
        self.assertEqual(geom3["task_force"][0].vector.size, 3)

    def test_task_kinematic_law(self):
        arm = SpeedControlledManipulator.from_manipulator(TwoLinkManipulator())
        ctl = TaskKinematic(arm, Kp=[1.0, 1.0])
        q = np.array([0.2, -0.1])
        p_d = np.array([0.5, 0.5])
        dq = ctl.ctl(None, np.concatenate([p_d, q]))
        p = arm.forward_kinematics(q)
        J = arm.J(q)
        expected = np.linalg.solve(J, p_d - p)
        np.testing.assert_allclose(dq, expected)

    def test_task_kinematic_closed_loop_y(self):
        arm = SpeedControlledManipulator.from_manipulator(TwoLinkManipulator())
        diagram = closed_loop(TaskKinematic(arm), arm)
        self.assertEqual(diagram.connections["ctl"]["y"], ("sys", "y"))
        self.assertNotIn("mux", diagram.subsystems)

    def test_task_kinematic_nullspace_projection(self):
        arm = SpeedControlledManipulator.from_manipulator(FiveLinkPlanarManipulator())
        ctl = TaskKinematicNullspace(arm, Kp=[1.0, 1.0], K_null=[10.0] * 5)
        q = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
        p_d = np.array([1.0, 1.0])
        q_null = np.array([-1.0] * 5)
        dq = ctl.ctl(None, np.concatenate([p_d, q_null, q]))
        J = arm.J(q)
        J_pinv = np.linalg.pinv(J)
        null_proj = np.eye(5) - J_pinv @ J
        q_e = q_null - q
        dq_null_only = null_proj @ (ctl.params["K_null"] * q_e)
        np.testing.assert_allclose(J @ dq_null_only, np.zeros(2), atol=1e-10)
        dq_task = J_pinv @ (ctl.params["Kp"] * (p_d - arm.forward_kinematics(q)))
        np.testing.assert_allclose(dq, dq_task + dq_null_only)

    def test_task_kinematic_nullspace_dual_ref_autowire(self):
        arm = SpeedControlledManipulator.from_manipulator(FiveLinkPlanarManipulator())
        ref_p = Source(2)
        ref_p.params["value"] = np.array([1.0, 1.0])
        ref_q = Source(5)
        ref_q.params["value"] = np.array([-1.0] * 5)
        ctl = TaskKinematicNullspace(arm, Kp=[1.0, 1.0], K_null=[10.0] * 5)
        diagram = (ref_q + (ref_p >> ctl @ arm)).autowire(strict=True)
        self.assertEqual(diagram.connections["ctl"]["r"][1], "y")
        self.assertEqual(diagram.connections["ctl"]["r_null"][1], "y")
        self.assertNotEqual(
            diagram.connections["ctl"]["r"][0], diagram.connections["ctl"]["r_null"][0]
        )


from minilink.dynamics.catalog.manipulators.arms import (
    OneLinkManipulator,
    TwoLinkManipulator,
)


class TestInverseKinematics(unittest.TestCase):
    def test_one_link_roundtrip(self):
        arm = OneLinkManipulator()
        q_true = np.array([0.4])
        p = arm.forward_kinematics(q_true)
        q = arm.inverse_kinematics(p, np.array([0.0]))
        np.testing.assert_allclose(arm.forward_kinematics(q), p, atol=1e-08)
        np.testing.assert_allclose(q, q_true, atol=1e-08)

    def test_default_q_guess_uses_q_nominal(self):
        arm = OneLinkManipulator()
        q_true = np.array([0.4])
        p = arm.forward_kinematics(q_true)
        q = arm.inverse_kinematics(p)
        np.testing.assert_allclose(arm.forward_kinematics(q), p, atol=1e-08)
        np.testing.assert_allclose(q, q_true, atol=1e-08)

    def test_two_link_roundtrip(self):
        arm = TwoLinkManipulator()
        q_true = np.array([0.5, -0.3])
        p = arm.forward_kinematics(q_true)
        q = arm.inverse_kinematics(p, q_true + 0.05)
        np.testing.assert_allclose(arm.forward_kinematics(q), p, atol=1e-08)

    def test_two_link_different_branch_from_guess(self):
        arm = TwoLinkManipulator()
        p = arm.forward_kinematics(np.array([0.6, -0.2]))
        q_a = arm.inverse_kinematics(p, np.array([0.2, 0.2]))
        q_b = arm.inverse_kinematics(p, np.array([1.0, -1.0]))
        np.testing.assert_allclose(arm.forward_kinematics(q_a), p, atol=1e-08)
        np.testing.assert_allclose(arm.forward_kinematics(q_b), p, atol=1e-08)
        self.assertGreater(np.linalg.norm(q_a - q_b), 0.1)


from minilink.blocks.sources import Step
from minilink.control.modelbased import ComputedTorqueController, SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestModelBasedControllers(unittest.TestCase):
    def test_computed_torque_pd_plus_inverse_dynamics(self):
        plant = Pendulum()
        ctl = ComputedTorqueController(plant, Kp=[30.0], Kd=[6.0])
        q = np.array([0.2])
        dq = np.array([0.1])
        r = np.array([0.5, 0.0])
        u = ctl.ctl(None, np.concatenate([r, q, dq]))
        qdd = 30.0 * (0.5 - 0.2) + 6.0 * (0.0 - 0.1)
        np.testing.assert_allclose(u, plant.inverse_dynamics(q, dq, np.array([qdd])))

    def test_sliding_mode_matches_pyro_law(self):
        plant = Pendulum(length=1.0, mass=1.0)
        lam = np.array([2.0])
        gain = np.array([3.0])
        nab = np.array([0.2])
        ctl = SlidingModeController(plant, lam=lam, gain=gain, nab=nab)
        q = np.array([0.3])
        dq = np.array([-0.15])
        q_d = np.array([0.0])
        dq_d = np.array([0.0])
        r = np.concatenate([q_d, dq_d])
        boundary = np.concatenate([r, q, dq])
        q_e = q - q_d
        dq_e = dq - dq_d
        s = dq_e + lam * q_e
        ddq_r = -lam * dq_e
        H = plant.H(q)
        K = np.diag(gain) + H @ np.diag(nab)
        expected = plant.inverse_dynamics(q, dq, ddq_r) - K @ np.sign(s)
        np.testing.assert_allclose(ctl.ctl(None, boundary), expected)

    def test_sliding_mode_ports_match_computed_torque(self):
        plant = Pendulum()
        ctl = SlidingModeController(plant)
        self.assertIn("r", ctl.inputs)
        self.assertIn("y", ctl.inputs)
        self.assertIn("u", ctl.outputs)
        self.assertEqual(ctl.inputs["y"].dim, 2)

    def test_sliding_mode_sets_discontinuous_behavior(self):
        ctl = SlidingModeController(Pendulum())
        self.assertTrue(ctl.solver_info["discontinuous_behavior"])

    def test_closed_loop_qdq_aggregates_discontinuous_behavior(self):
        plant = Pendulum()
        smc = SlidingModeController(plant)
        ref = Step(initial_value=np.zeros(2), final_value=np.zeros(2), step_time=1.0)
        diagram = ref >> closed_loop_qdq(smc, plant)
        self.assertTrue(diagram.solver_info["discontinuous_behavior"])
