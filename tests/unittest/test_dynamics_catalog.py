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
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynEngine,
    BicycleDynEnginePorts,
    BicycleDynRate,
    BicycleDynRatePorts,
    BicycleDynServo,
    BicycleDynServoPorts,
)


class TestBicycleDynRate(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynRatePorts()
        self.uy = BicycleDynRate()

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

    def test_inverse_propulsion_dynamics_inverts_wheel_spin(self):
        rate = self.named
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 4.0, 0.1])
        u_rate = np.array([0.5, -0.1])

        tau = float(np.asarray(rate.inverse_propulsion_dynamics(x, u_rate)))
        tau_ground = float(
            np.asarray(rate.rear_wheel_ground_torque(x[3:6], x[6], x[7], rate.params))
        )
        w_dot = (tau - tau_ground) / rate.params["Jw_rear"]
        self.assertAlmostEqual(w_dot, u_rate[0], places=9)

    def test_params_override_mass(self):
        sys = BicycleDynRate()
        x = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 5.0 / 0.3, 0.0])
        u = np.array([0.0, 0.0])
        params = {**sys.params, "mass": 2.0 * sys.params["mass"]}
        dx_nom = np.asarray(sys.f(x, u))
        dx_heavy = np.asarray(sys.f(x, u, params=params))
        self.assertFalse(np.allclose(dx_nom, dx_heavy))


class TestBicycleDynServo(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynServoPorts()
        self.uy = BicycleDynServo()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertEqual(self.uy.n, 9)
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)

    def test_f_matches_named_servo_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 4.0, 0.1, 0.0])
        u = np.array([200.0, 0.05])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)

    def test_zero_torque_cruise_actuators_near_equilibrium(self):
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0 / 0.3, 0.0, 0.0])
        u = np.array([0.0, 0.0])
        dx = np.asarray(self.named.f(x, u))
        np.testing.assert_allclose(dx[6:8], np.zeros(2), atol=0.5)

    def test_params_override_Ca(self):
        sys = BicycleDynServo()
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.5, 0.0, 10.0 / 0.3, 0.05, 0.0])
        u = np.array([0.0, 0.0])
        params = {**sys.params, "Ca": 0.5 * sys.params["Ca"]}
        dx_nom = np.asarray(sys.f(x, u))
        dx_soft = np.asarray(sys.f(x, u, params=params))
        self.assertFalse(np.allclose(dx_nom, dx_soft))


class TestBicycleDynEngine(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynEnginePorts()
        self.uy = BicycleDynEngine()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertEqual(self.uy.n, 9)
        self.assertEqual(self.uy.inputs["u"].labels, ["P_cmd", "delta_cmd"])
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)
        self.assertNotIn("torque_tau", self.uy.params)
        self.assertNotIn("transmission_ratio", self.uy.params)
        self.assertNotIn("engine_power_peak", self.uy.params)

    def test_f_matches_named_engine_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 10.0, 0.1, 5000.0])
        u = np.array([8000.0, 0.05])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)

    def test_unsaturated_torque_is_power_over_omega(self):
        sys = BicycleDynEngine()
        w = 50.0
        P = 5000.0
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, w, 0.0, P])
        u = np.array([P, 0.0])
        # Mid-speed unsaturated: τ ≈ P/ω ≪ tau_sat
        self.assertLess(abs(P / w), sys.params["tau_sat"])
        dx = np.asarray(sys.f(x, u))
        # P_dot ≈ 0 when P_cmd = P
        np.testing.assert_allclose(dx[8], 0.0, atol=1e-09)

    def test_stall_torque_saturates_near_zero_omega(self):
        sys = BicycleDynEngine()
        tau_sat = sys.params["tau_sat"]
        P = 1e6
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P])
        u = np.array([P, 0.0])
        dx = np.asarray(sys.f(x, u))
        # ω=0, P>0 → τ = +τ_sat; brake≈0 → ω̇ = τ_sat / Jw
        np.testing.assert_allclose(dx[6], tau_sat / sys.params["Jw_rear"], rtol=1e-06)

    def test_params_override_engine_brake(self):
        sys = BicycleDynEngine()
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0 / 0.3, 0.0, 0.0])
        u = np.array([0.0, 0.0])
        params = {**sys.params, "bw_engine": 10.0 * sys.params["bw_engine"]}
        dx_nom = np.asarray(sys.f(x, u))
        dx_heavy = np.asarray(sys.f(x, u, params=params))
        self.assertFalse(np.allclose(dx_nom[6], dx_heavy[6]))


class TestCarProfile(unittest.TestCase):
    def test_registered_profiles(self):
        from minilink.dynamics.catalog.vehicles.car_profile import (
            CAR_PROFILES,
            get_car_profile,
            list_car_profiles,
        )

        self.assertEqual(list_car_profiles(), ("passenger_car", "racecar", "udes_1_5"))
        for name in list_car_profiles():
            self.assertIs(get_car_profile(name), CAR_PROFILES[name])

    def test_racecar_matches_demo_vehicle_geometry(self):
        from minilink.dynamics.catalog.vehicles.car_profile import racecar_profile

        profile = racecar_profile()
        self.assertEqual(profile.mass, 700.0)
        self.assertEqual(profile.a, 1.2)
        self.assertEqual(profile.b, 1.0)
        self.assertEqual(profile.r_r, 0.34)
        self.assertEqual(profile.engine_power_peak, 100000.0)
        self.assertEqual(profile.v_nom, 10.0)
        self.assertLess(profile.v_nom, profile.limits.vx_max)
        self.assertAlmostEqual(profile.limits.delta_max, np.pi / 4.0, places=2)
        self.assertEqual(profile.limits.delta_dot_max, 3.0)
        self.assertEqual(profile.limits.w_rear_dot_max, 41.0)
        self.assertEqual(profile.limits.tau_rear_max, 3400.0)
        self.assertEqual(profile.limits.tau_rear_min, -3400.0)

    def test_propulsion_limits_from_power_at_nominal(self):
        from minilink.dynamics.catalog.vehicles.car_profile import (
            get_car_profile,
            list_car_profiles,
        )

        for name in list_car_profiles():
            profile = get_car_profile(name)
            tau = profile.propulsion_torque_nominal()
            wdot = profile.propulsion_wheel_accel_nominal()
            if abs(tau) >= 100.0:
                expected_tau = round(tau, -1)
            else:
                expected_tau = round(tau, 1)
            self.assertEqual(profile.limits.tau_rear_max, expected_tau)
            self.assertEqual(profile.limits.tau_rear_min, -expected_tau)
            self.assertEqual(profile.limits.w_rear_dot_max, round(wdot))
            self.assertAlmostEqual(
                profile.limits.v_dot_max,
                round(profile.propulsion_longitudinal_accel_nominal(), 1),
            )
            self.assertEqual(profile.limits.a_long_max, profile.limits.v_dot_max)
            self.assertEqual(profile.limits.delta_dot_max, profile.steer_rate_max)
            self.assertLessEqual(profile.v_nom, profile.limits.vx_max)

    def test_actuator_limits_exceed_traction_reference(self):
        from minilink.dynamics.catalog.vehicles.car_profile import (
            get_car_profile,
            list_car_profiles,
        )

        for name in list_car_profiles():
            profile = get_car_profile(name)
            if profile.propulsion_torque_nominal() > profile.traction_torque_max():
                self.assertGreater(
                    profile.limits.tau_rear_max,
                    profile.traction_torque_max(),
                    msg=name,
                )
                self.assertGreater(
                    profile.limits.w_rear_dot_max,
                    round(profile.traction_wheel_accel_reference()),
                    msg=name,
                )
                self.assertGreater(profile.actuator_traction_headroom(), 1.0, msg=name)

    def test_udes_matches_kinematic_racecar_geometry(self):
        from minilink.dynamics.catalog.vehicles.car_profile import udes_1_5_profile
        from minilink.dynamics.catalog.vehicles.steering import UdeSRacecar

        udes = UdeSRacecar()
        profile = udes_1_5_profile()
        self.assertAlmostEqual(profile.a, udes.params["a"])
        self.assertAlmostEqual(profile.b, udes.params["b"])
        self.assertAlmostEqual(profile.length, udes.params["length"])

    def test_apply_car_profile_rate_plant(self):
        from minilink.dynamics.catalog.vehicles.car_profile import (
            apply_car_profile,
            passenger_car_profile,
        )

        sys = BicycleDynRate()
        apply_car_profile(sys, passenger_car_profile())
        profile = passenger_car_profile()
        self.assertAlmostEqual(sys.params["mass"], profile.mass)
        self.assertAlmostEqual(sys.params["length"], profile.a + profile.b)
        self.assertAlmostEqual(sys.params["Ca"], profile.Ca)
        self.assertAlmostEqual(sys.a, profile.a)
        self.assertAlmostEqual(sys.b, profile.b)
        self.assertFalse(hasattr(sys, "tire_model_f"))
        self.assertAlmostEqual(sys.state.upper_bound[6], profile.limits.w_rear_max)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.limits.w_rear_dot_max
        )
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[1], profile.limits.delta_dot_max
        )

    def test_apply_car_profile_servo_plant(self):
        from minilink.dynamics.catalog.vehicles.car_profile import (
            apply_car_profile,
            racecar_profile,
        )

        sys = BicycleDynServo()
        apply_car_profile(sys, racecar_profile())
        profile = racecar_profile()
        self.assertAlmostEqual(sys.params["steering_tau"], profile.steering_tau)
        self.assertAlmostEqual(sys.params["torque_tau"], 0.05)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.limits.tau_rear_max
        )
        self.assertAlmostEqual(sys.inputs["u"].upper_bound[1], profile.limits.delta_max)

    def test_apply_car_profile_engine_plant(self):
        from minilink.dynamics.catalog.vehicles.car_profile import (
            apply_car_profile,
            racecar_profile,
        )

        sys = BicycleDynEngine()
        apply_car_profile(sys, racecar_profile())
        profile = racecar_profile()
        self.assertAlmostEqual(sys.params["engine_tau"], profile.engine_tau)
        self.assertAlmostEqual(sys.params["steering_tau"], profile.steering_tau)
        self.assertAlmostEqual(sys.params["tau_sat"], profile.tau_sat)
        self.assertAlmostEqual(sys.params["bw_engine"], profile.bw_engine)
        self.assertAlmostEqual(sys.params["tau_fric"], profile.tau_fric)
        self.assertNotIn("engine_power_peak", sys.params)
        self.assertNotIn("transmission_ratio", sys.params)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.engine_power_peak
        )
        self.assertAlmostEqual(
            sys.inputs["u"].lower_bound[0], -profile.engine_power_peak
        )
        self.assertAlmostEqual(sys.inputs["u"].upper_bound[1], profile.limits.delta_max)
        self.assertAlmostEqual(sys.state.upper_bound[8], profile.engine_power_peak)
        self.assertAlmostEqual(sys.state.lower_bound[8], -profile.engine_power_peak)
