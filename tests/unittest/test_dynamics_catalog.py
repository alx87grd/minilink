"""Catalog equation reference values and graphics primitive contracts.

Broad catalog plant checks (instantiate, ``f`` finite, geometry) live in the
demo-check layer: ``tests/demo_checks/run_catalog_checks.py``.
"""

import unittest
import numpy as np
from minilink.dynamics.catalog.aerial.drone import (
    ConstantSpeedHelicopterTunnel,
    Drone2D,
    Drone2DWithSideThruster,
    SpeedControlledDrone2D,
)
from minilink.dynamics.catalog.aerial.plane import Plane2D
from minilink.dynamics.catalog.aerial.rocket import Rocket
from minilink.dynamics.catalog.equations.integrators import (
    DoubleIntegrator,
    SimpleIntegrator,
    TripleIntegrator,
)
from minilink.dynamics.catalog.equations.oscillators import VanderPol
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    OneLinkManipulator,
    SpeedControlledManipulator,
    TwoLinkManipulator,
)
from minilink.dynamics.catalog.marine.boat import Boat2D, Boat2DWithCurrent
from minilink.dynamics.catalog.mass_spring_damper.linear import (
    SingleMass,
    ThreeMass,
    TwoMass,
)
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.dynamics.catalog.pendulum.double_pendulum import Acrobot
from minilink.dynamics.catalog.pendulum.pendulum import (
    Pendulum,
    TwoIndependentPendulums,
)
from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
from minilink.dynamics.catalog.vehicles.propulsion import (
    LongitudinalFrontWheelDriveCarWithTorqueInput,
    LongitudinalFrontWheelDriveCarWithWheelSlipInput,
)
from minilink.dynamics.catalog.vehicles.steering import (
    DynamicHolonomicMobileRobot,
    HolonomicMobileRobot,
    HolonomicMobileRobot3D,
    KinematicBicycle,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCarOnRoughTerrain
from minilink.graphical.animation.primitives import Arrow, TorqueArrow
from tests.unittest.graphics_contract_helpers import geometry_smoke as _geometry_smoke
from tests.unittest.graphics_contract_helpers import (
    resolved_primitive_count as _primitive_count,
)


class TestCatalogSmoke(unittest.TestCase):
    def test_low_risk_equation_reference_values(self):
        np.testing.assert_allclose(
            SimpleIntegrator().f(np.array([2.0]), np.array([3.0])), [3.0]
        )
        np.testing.assert_allclose(
            DoubleIntegrator().f(np.array([2.0, 4.0]), np.array([3.0])), [4.0, 3.0]
        )
        np.testing.assert_allclose(
            TripleIntegrator().f(np.array([2.0, 4.0, 6.0]), np.array([3.0])),
            [4.0, 6.0, 3.0],
        )
        np.testing.assert_allclose(
            VanderPol(mu=0.5).f(np.array([1.0, 2.0]), np.array([0.0])), [2.0, -1.0]
        )

    def test_mass_spring_damper_reference_matrices(self):
        single = SingleMass(mass=2.0, k=4.0, b=6.0)
        np.testing.assert_allclose(single.A(), [[0.0, 1.0], [-2.0, -3.0]])
        np.testing.assert_allclose(single.B(), [[0.0], [0.5]])
        two = TwoMass(m=1.0, k=2.0, b=0.2, output_mass=1)
        np.testing.assert_allclose(two.C(), [[1.0, 0.0, 0.0, 0.0]])
        three = ThreeMass(m=2.0, k=4.0, b=0.0, output_mass=3)
        np.testing.assert_allclose(three.B()[-1], [0.5])

    def test_vehicle_reference_values(self):
        bicycle = KinematicBicycle()
        np.testing.assert_allclose(
            bicycle.f(np.array([0.0, 0.0, 0.0]), np.array([2.0, 0.0])), [2.0, 0.0, 0.0]
        )
        np.testing.assert_allclose(
            HolonomicMobileRobot().f(np.array([1.0, 2.0]), np.array([3.0, 4.0])),
            [3.0, 4.0],
        )
        np.testing.assert_allclose(
            DynamicHolonomicMobileRobot().f(
                np.array([1.0, 2.0, 3.0, 4.0]), np.array([5.0, 6.0])
            ),
            [3.0, 4.0, 5.0, 6.0],
        )
        car = LongitudinalFrontWheelDriveCarWithWheelSlipInput()
        self.assertGreater(car.acceleration(speed=0.0, slip=0.1), 0.0)
        mountain = MountainCar()
        np.testing.assert_allclose(mountain.H(np.array([0.0])), [[1.0]])

    def test_aerial_and_marine_force_split_reference_values(self):
        drone = Drone2D()
        q = np.zeros(3)
        dq = np.zeros(3)
        hover = np.array([drone.params["mass"] * drone.params["gravity"] / 2.0] * 2)
        np.testing.assert_allclose(
            drone.forward_dynamics(q, dq, hover), np.zeros(3), atol=1e-12
        )
        rocket = Rocket()
        np.testing.assert_allclose(
            rocket.generalized_force(np.zeros(3), np.zeros(3), np.array([10.0, 0.0])),
            [0.0, 10.0, 0.0],
        )
        boat = Boat2D()
        np.testing.assert_allclose(
            boat.generalized_force(np.zeros(3), np.zeros(3), np.array([3.0, 4.0])),
            [3.0, 4.0, -12.0],
        )

    def test_manipulator_kinematics_reference_values(self):
        one = OneLinkManipulator()
        np.testing.assert_allclose(
            one.forward_kinematics(np.array([0.0])), [0.0, one.params["l1"]]
        )
        five = FiveLinkPlanarManipulator()
        np.testing.assert_allclose(
            five.forward_kinematics(np.zeros(5)), [0.0, np.sum(five.params["l"])]
        )

    def test_pyro_designed_force_velocity_and_torque_arrows_are_present(self):
        arrow_cases = [
            (Plane2D(), 6),
            (Drone2D(), 2),
            (Drone2DWithSideThruster(), 3),
            (SpeedControlledDrone2D(), 1),
            (ConstantSpeedHelicopterTunnel(), 1),
            (Rocket(), 1),
            (Boat2D(), 1),
            (Boat2DWithCurrent(), 2),
            (QuarterCarOnRoughTerrain(), 1),
            (MountainCar(), 1),
            (HolonomicMobileRobot(), 1),
            (DynamicHolonomicMobileRobot(), 1),
            (HolonomicMobileRobot3D(), 1),
            (LongitudinalFrontWheelDriveCarWithTorqueInput(), 1),
            (CartPole(), 1),
            (SpeedControlledManipulator(2, 2), 1),
        ]
        for system, expected in arrow_cases:
            with self.subTest(system=system.name):
                self.assertEqual(_primitive_count(system, Arrow), expected)
                _geometry_smoke(system)
        boat = Boat2D()
        boat.show_hydrodynamic_forces = True
        self.assertEqual(_primitive_count(boat, Arrow), 2)
        self.assertEqual(_primitive_count(boat, TorqueArrow), 1)
        _geometry_smoke(boat)
        torque_cases = [
            (Pendulum(), 1),
            (TwoIndependentPendulums(), 2),
            (Acrobot(), 1),
            (OneLinkManipulator(), 1),
            (TwoLinkManipulator(), 2),
        ]
        for system, expected in torque_cases:
            with self.subTest(system=system.name):
                self.assertEqual(_primitive_count(system, TorqueArrow), expected)
                _geometry_smoke(system)


from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.primitives import Arrow, Box, Rod
from tests.unittest.graphics_contract_helpers import geometry_smoke, resolve_draw_frame


class TestCartPole(unittest.TestCase):
    def test_dimensions_and_reference_matrices(self):
        sys = CartPole()
        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.state.labels, ["x", "theta", "dx", "dtheta"])
        q = np.array([0.2, 0.3])
        dq = np.array([0.4, 0.5])
        H = sys.H(q)
        C = sys.C(q, dq)
        np.testing.assert_allclose(
            H,
            np.array(
                [
                    [1.1, 0.1 * 0.5 * np.cos(0.3)],
                    [0.1 * 0.5 * np.cos(0.3), 0.1 * 0.5**2],
                ]
            ),
        )
        self.assertAlmostEqual(C[0, 1], -0.1 * 0.5 * np.sin(0.3) * 0.5)
        np.testing.assert_allclose(sys.B(q), np.array([[1.0], [0.0]]))
        np.testing.assert_allclose(
            sys.g(q), np.array([0.0, 0.1 * 9.81 * 0.5 * np.sin(0.3)])
        )

    def test_compile_and_graphics_contract(self):
        sys = CartPole()
        x = np.zeros(sys.n)
        u = np.zeros(sys.m)
        np.testing.assert_allclose(sys.compile("numpy").f(x, u, 0.0), np.zeros(sys.n))
        frame = resolve_draw_frame(sys)
        cart_boxes = [
            primitive for primitive in frame["primitives"] if isinstance(primitive, Box)
        ]
        self.assertEqual(len(cart_boxes), 1)
        self.assertEqual(cart_boxes[0].length_z, sys.cart_depth)
        self.assertTrue(any((isinstance(p, Rod) for p in frame["primitives"])))
        geometry_smoke(sys)


class TestDoublePendulum(unittest.TestCase):
    def test_reference_matrices_and_compile(self):
        sys = DoublePendulum()
        q = np.array([0.2, 0.3])
        dq = np.array([0.4, 0.5])
        c2 = np.cos(q[1])
        s1, s12 = (np.sin(q[0]), np.sin(q[0] + q[1]))
        h = np.sin(q[1])
        np.testing.assert_allclose(
            sys.H(q), np.array([[3.0 + 2.0 * c2, 1.0 + c2], [1.0 + c2, 1.0]])
        )
        np.testing.assert_allclose(
            sys.C(q, dq),
            np.array([[-h * dq[1], -h * (dq[0] + dq[1])], [h * dq[0], 0.0]]),
        )
        np.testing.assert_allclose(
            sys.g(q), np.array([-19.62 * s1 - 9.81 * s12, -9.81 * s12])
        )
        x = np.zeros(sys.n)
        np.testing.assert_allclose(
            sys.compile("numpy").f(x, np.zeros(sys.m), 0.0), np.zeros(sys.n)
        )
        geometry_smoke(sys)


class TestDynamicBicycle(unittest.TestCase):
    def test_dynamics_and_camera_follow(self):
        sys = DynamicBicycle()
        self.assertEqual(sys.n, 6)
        self.assertEqual(sys.m, 2)
        x = np.array([10.0, 3.0, 0.25, 4.0, 0.0, 0.0])
        u = np.zeros(sys.m)
        sys.camera_target[:] = (1.0, -2.0, 0.5)
        sys.camera_scale = 7.0
        frames = sys.tf(x, u, 0.0)
        camera = resolve_camera_from_hints(sys, frames, x, u, 0.0)
        np.testing.assert_allclose(camera[:3, 3], np.array([11.0, 1.0, 0.5]))
        x = np.array([0.1, -0.2, 0.3, 5.0, 0.4, 0.05])
        dx = sys.f(x, np.array([20.0, 0.1]))
        np.testing.assert_allclose(dx[:3], sys.N(x[:3]) @ x[3:])

    def test_graphics_resolves_dynamic_arrows(self):
        sys = DynamicBicycle()
        x = np.zeros(sys.n)
        x[3] = 1.0
        frame = resolve_draw_frame(sys, x, np.array([10.0, 0.1]), 0.0)
        self.assertEqual(len(frame["primitives"]), 7)
        self.assertEqual(sum((isinstance(p, Arrow) for p in frame["primitives"])), 4)
        geometry_smoke(sys, x, np.array([10.0, 0.1]), 0.0)


from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    OneLinkManipulator,
    SpeedControlledManipulator,
    ThreeLinkManipulator3D,
    TwoLinkManipulator,
    _planar_joint_positions,
)

_MANIPULATORS = (
    OneLinkManipulator,
    TwoLinkManipulator,
    ThreeLinkManipulator3D,
    FiveLinkPlanarManipulator,
)


class TestManipulatorCatalog(unittest.TestCase):
    def test_all_subclass_manipulator_with_task_ports(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                self.assertIsInstance(arm, Manipulator)
                for port in ("q", "dq", "p", "pdot", "y"):
                    self.assertIn(port, arm.outputs)

    def test_camera_scale_matches_reach_not_default(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                self.assertLess(arm.camera_scale, 10.0)
        speed = SpeedControlledManipulator(2, 2)
        self.assertLess(speed.camera_scale, 10.0)

    def test_h_p_and_h_pdot_match_kinematics(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                dof = arm.dof
                q = np.linspace(-0.4, 0.4, dof)
                dq = np.linspace(-0.3, 0.3, dof)
                x = arm.q2x(q, dq)
                np.testing.assert_allclose(
                    arm.h_p(x, np.zeros(dof)), arm.forward_kinematics(q)
                )
                np.testing.assert_allclose(arm.h_pdot(x, np.zeros(dof)), arm.J(q) @ dq)

    def test_jacobian_matches_numeric_forward_kinematics(self):
        eps = 1e-07
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                dof = arm.dof
                q = np.linspace(-0.3, 0.3, dof)
                J = arm.J(q)
                Jnum = np.column_stack(
                    [
                        (
                            arm.forward_kinematics(q + eps * e)
                            - arm.forward_kinematics(q - eps * e)
                        )
                        / (2 * eps)
                        for e in np.eye(dof)
                    ]
                )
                np.testing.assert_allclose(J, Jnum, atol=1e-05, rtol=1e-05)

    def test_two_link_fk_matches_planar_geometry_tip(self):
        arm = TwoLinkManipulator()
        q = np.array([0.2, -0.1])
        tip, _ = _planar_joint_positions(q, arm._lengths())
        np.testing.assert_allclose(arm.forward_kinematics(q), tip[-1])

    def test_three_link_fk_matches_tf_end_effector(self):
        arm = ThreeLinkManipulator3D()
        q = np.array([0.1, -0.2, 0.15])
        x = arm.q2x(q, np.zeros(3))
        fk = arm.forward_kinematics(q)
        tip = arm.tf(x, np.zeros(3))["joint3"][:3, 3]
        np.testing.assert_allclose(fk, tip)

    def test_five_link_uses_placeholder_inertia(self):
        arm = FiveLinkPlanarManipulator()
        np.testing.assert_allclose(arm.H(np.zeros(5)), np.eye(5))
        np.testing.assert_allclose(
            arm.inverse_dynamics(np.zeros(5), np.zeros(5), np.ones(5)), np.ones(5)
        )

    def test_torque_arrow_sign_matches_gravity_hold(self):
        """Static hold uses ``tau = g(q)``; arc sweep is ``-u`` (Pyro / double-pendulum)."""
        arm = TwoLinkManipulator()
        q = np.array([0.8, -0.5])
        x = arm.q2x(q, np.zeros(2))
        tau = arm.g(q)
        self.assertTrue(np.all(tau < 0.0))
        geom = arm.get_dynamic_geometry(x, tau)
        for i in range(arm.dof):
            self.assertGreater(geom[f"link{i}"][0].sweep, 0.0)
            np.testing.assert_allclose(
                geom[f"link{i}"][0].sweep,
                -tau[i]
                * (2.0 * np.pi / 3.0)
                / max(abs(arm.inputs["u"].upper_bound[i]), 1.0),
            )

    def test_from_manipulator_inherits_kinematics(self):
        source = TwoLinkManipulator()
        speed = SpeedControlledManipulator.from_manipulator(source)
        q = np.array([0.2, -0.1])
        np.testing.assert_allclose(
            speed.forward_kinematics(q), source.forward_kinematics(q)
        )
        np.testing.assert_allclose(speed.J(q), source.J(q))
        self.assertEqual(speed.n, 2)
        np.testing.assert_allclose(speed.f(q, np.array([0.3, -0.2])), [0.3, -0.2])

    def test_speed_plant_p_port(self):
        speed = SpeedControlledManipulator.from_manipulator(TwoLinkManipulator())
        q = np.array([0.2, -0.1])
        np.testing.assert_allclose(
            speed.h_p(q, np.zeros(2)), speed.forward_kinematics(q)
        )


import pytest

pytest.importorskip("jax")
from minilink.core.backends import configure_jax
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
    JaxDynamicBicycleRateInputsUY,
    JaxDynamicBicycleServoInputs,
    JaxDynamicBicycleServoInputsUY,
)


class TestDynamicBicycleUY(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = JaxDynamicBicycleRateInputs()
        self.uy = JaxDynamicBicycleRateInputsUY()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)

    def test_f_matches_named_rate_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 4.0, 0.1])
        u = np.array([0.5, -0.1])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)


class TestDynamicBicycleServoInputs(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = JaxDynamicBicycleServoInputs()
        self.uy = JaxDynamicBicycleServoInputsUY()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)

    def test_f_matches_named_servo_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 4.0, 0.1])
        u = np.array([200.0, 0.05])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)

    def test_zero_torque_cruise_actuators_near_equilibrium(self):
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0 / 0.3, 0.0])
        u = np.array([0.0, 0.0])
        dx = np.asarray(self.named.f(x, u))
        np.testing.assert_allclose(dx[6:8], np.zeros(2), atol=0.5)
