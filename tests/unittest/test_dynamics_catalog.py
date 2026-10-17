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
from minilink.dynamics.catalog.aerial.plane import Plane2D, Plane3D
from minilink.dynamics.catalog.aerial.rocket import Rocket
from minilink.dynamics.catalog.equations.integrators import (
    DoubleIntegrator,
    SimpleIntegrator,
    TripleIntegrator,
)
from minilink.dynamics.catalog.equations.oscillators import Lorenz, VanderPol
from minilink.dynamics.catalog.astro.three_body import ThreeBodyProblem
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
from minilink.dynamics.catalog.pendulum.cartpole import CartPole, CartPoleWithNoisePort
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
from examples.projects.car_trajopt.vehicles.extras import (
    DynamicHolonomicMobileRobot,
    HolonomicMobileRobot3D,
)
from minilink.dynamics.catalog.vehicles.steering import (
    HolonomicMobileRobot,
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
        np.testing.assert_allclose(
            Lorenz().f(np.array([0.0, 0.0, 0.0]), np.array([])), [0.0, 0.0, 0.0]
        )
        np.testing.assert_allclose(
            Lorenz().f(np.array([1.0, 1.0, 1.0]), np.array([])),
            [0.0, 26.0, -5.0 / 3.0],
        )

    def test_three_body_energy_and_dynamics(self):
        sys = ThreeBodyProblem(preset="figure_eight")
        x0 = sys.x0
        dx = sys.f(x0, np.array([]))
        self.assertEqual(dx.shape, (18,))
        np.testing.assert_allclose(dx[0:3], x0[3:6])
        e0 = sys.total_energy(x0)
        self.assertLess(e0, 0.0)
        traj = sys.compute_trajectory(tf=1.0, n_steps=400, show=False, verbose=False)
        e_drift = abs(sys.total_energy(traj.x[:, -1]) - e0) / abs(e0)
        self.assertLess(e_drift, 1e-2)

    def test_mass_spring_damper_reference_matrices(self):
        single = SingleMass(mass=2.0, k=4.0, b=6.0)
        np.testing.assert_allclose(single.A(), [[0.0, 1.0], [-2.0, -3.0]])
        np.testing.assert_allclose(single.B(), [[0.0], [0.5]])
        two = TwoMass(m=1.0, k=2.0, b=0.2, output_mass=1)
        np.testing.assert_allclose(two.C(), [[1.0, 0.0, 0.0, 0.0]])
        three = ThreeMass(m=2.0, k=4.0, b=0.0, output_mass=3)
        np.testing.assert_allclose(three.B()[-1], [0.5])

    def test_mass_chain_rejects_an_output_mass_outside_the_chain(self):
        # A typo in the measured mass must not silently measure another mass.
        for plant_class, count in ((TwoMass, 2), (ThreeMass, 3)):
            for output_mass in (0, count + 1):
                with self.assertRaisesRegex(ValueError, f"between 1 and {count}"):
                    plant_class(output_mass=output_mass)

    def test_the_3d_car_skin_dresses_a_plant_without_wheel_radii(self):
        from minilink.dynamics.catalog.vehicles.steering import KinematicCar
        from minilink.graphical.catalog.skins import car_skin_3d

        car = KinematicCar()
        self.assertNotIn("r_f", car.params)
        geometry = car_skin_3d(car)
        wheel_rl = geometry["body"][1]
        np.testing.assert_allclose(wheel_rl.local_transform[:3, 3], [-3.0, 0.8, 0.3])

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

    def test_quarter_car_damps_against_the_road_vertical_velocity(self):
        # The road rises under the wheel at z' = dz/dx * vx, so the damper
        # force b (y' - z') grows linearly with the forward speed vx.
        car = QuarterCarOnRoughTerrain()
        b, mass = car.params["b"], car.params["mass"]
        x = np.array([0.0, 0.0, 2.0])
        u = np.zeros(1)
        a0, a3 = (car.f(x, u, params=dict(car.params, vx=vx))[0] for vx in (0.0, 3.0))
        np.testing.assert_allclose(a3 - a0, 3.0 * b * car.dz(2.0) / mass)
        np.testing.assert_allclose(a3, 1.5436826659862837, rtol=1e-12)

    def test_drone_body_is_drawn_with_its_width_and_height(self):
        drone = Drone2D()
        drone.width = 2.0
        drone.height = 0.3
        body = drone.get_kinematic_geometry()["body"][0]
        self.assertEqual((body.length_x, body.length_y), (2.0, 0.3))
        default = Drone2D().get_kinematic_geometry()["body"][0]
        self.assertEqual((default.length_x, default.length_y), (1.2, 0.18))

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

    def test_plane_inverse_dynamics_without_u_reads_the_nominal_input(self):
        # Model-based controllers call inverse_dynamics(q, v, a) without u;
        # the control-surface loads then come from the u port's nominal value.
        for plane in (Plane2D(), Plane3D()):
            q = np.zeros(plane.dof)
            v = np.zeros(plane.dof)
            v[0] = 10.0
            acceleration = np.zeros(plane.dof)
            nominal = np.zeros(plane.m)
            nominal[1] = 0.05
            plane.inputs["u"].set_nominal_value(nominal)
            np.testing.assert_allclose(
                plane.inverse_dynamics(q, v, acceleration),
                plane.inverse_dynamics(q, v, acceleration, nominal),
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
from minilink.graphical.animation.primitives import Arrow, Box, Circle, Rod
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
        self.assertEqual(sys.camera_scale, 10.0)
        wheels = [
            primitive
            for primitive in frame["primitives"]
            if isinstance(primitive, Circle) and primitive.color == "black"
        ]
        self.assertEqual(len(wheels), 2)
        self.assertAlmostEqual(wheels[0].radius, sys.wheel_radius)
        pole = next(p for p in frame["primitives"] if isinstance(p, Rod))
        self.assertAlmostEqual(pole.linewidth, sys.line_width)
        self.assertTrue(any((isinstance(p, Rod) for p in frame["primitives"])))
        geometry_smoke(sys)

    def test_noise_ports_add_disturbance_and_sensor_noise(self):
        sys = CartPoleWithNoisePort()
        self.assertEqual(sys.name, "Cart Pole with noise ports")
        self.assertEqual(sys.inputs["d"].dim, 1)
        self.assertEqual(sys.inputs["n"].dim, 4)
        x = np.array([0.1, 0.2, 0.3, 0.4])
        F = np.array([1.0])
        w = np.array([0.5])
        v = np.array([0.01, 0.02, 0.03, 0.04])
        u = np.concatenate([F, w, v])
        np.testing.assert_allclose(sys.h(x, u), x + v)
        q, dq = x[:2], x[2:]
        np.testing.assert_allclose(
            sys.generalized_force(q, dq, u),
            sys.B(q) @ (F + w),
        )


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
        self.assertEqual(sys.camera_follow_frame, "body")
        self.assertEqual(sys.camera_scale, 10.0)
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
    planar_joint_positions,
)
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator

_MANIPULATORS = (
    OneLinkManipulator,
    TwoLinkManipulator,
    ThreeLinkManipulator3D,
    FiveLinkPlanarManipulator,
    UR5Manipulator,
)


class TestManipulatorCatalog(unittest.TestCase):
    def test_all_subclass_manipulator_with_task_ports(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                self.assertIsInstance(arm, Manipulator)
                for port in ("p", "pdot", "y"):
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
        tip, _ = planar_joint_positions(q, arm.lengths())
        np.testing.assert_allclose(arm.forward_kinematics(q), tip[-1])

    def test_three_link_fk_matches_tf_end_effector(self):
        arm = ThreeLinkManipulator3D()
        q = np.array([0.1, -0.2, 0.15])
        x = arm.q2x(q, np.zeros(3))
        fk = arm.forward_kinematics(q)
        tip = arm.tf(x, np.zeros(3))["joint3"][:3, 3]
        np.testing.assert_allclose(fk, tip)

    def test_ur5_zero_pose_matches_standard_dh_chain(self):
        arm = UR5Manipulator()
        expected = np.array([-0.81725, -0.19145, -0.005491])
        np.testing.assert_allclose(arm.forward_kinematics(np.zeros(6)), expected)

    def test_ur5_dynamics_are_well_conditioned_and_power_consistent(self):
        arm = UR5Manipulator()
        q = np.linspace(-0.5, 0.4, 6)
        dq = np.linspace(0.2, -0.3, 6)
        H = arm.H(q)

        np.testing.assert_allclose(H, H.T, atol=1e-12)
        self.assertTrue(np.all(np.linalg.eigvalsh(H) > 0.0))

        eps = 1e-6
        Hdot = (arm.H(q + eps * dq) - arm.H(q - eps * dq)) / (2.0 * eps)
        coriolis_power = dq @ (arm.C(q, dq) @ dq)
        inertia_rate_power = 0.5 * dq @ Hdot @ dq
        np.testing.assert_allclose(coriolis_power, inertia_rate_power, atol=1e-8)

    def test_ur5_aba_matches_rnea_forward_dynamics(self):
        arm = UR5Manipulator()
        rng = np.random.default_rng(0)
        for _ in range(8):
            q = rng.uniform(-1.0, 1.0, 6)
            v = rng.uniform(-1.0, 1.0, 6)
            u = rng.uniform(-5.0, 5.0, 6)
            qdd_rnea = arm.forward_dynamics_rnea_h(q, v, u)
            qdd_aba = arm.forward_dynamics(q, v, u)
            np.testing.assert_allclose(qdd_aba, qdd_rnea, rtol=0.0, atol=1e-10)

    def test_ur5_rnea_inverse_dynamics_matches_matrix(self):
        arm = UR5Manipulator()
        rng = np.random.default_rng(0)
        for _ in range(8):
            q = rng.uniform(-1.0, 1.0, 6)
            v = rng.uniform(-1.0, 1.0, 6)
            qdd = rng.uniform(-2.0, 2.0, 6)
            u = rng.uniform(-5.0, 5.0, 6)
            tau_rnea = arm.inverse_dynamics(q, v, qdd, u)
            tau_matrix = arm.inverse_dynamics_matrix(q, v, qdd, u)
            np.testing.assert_allclose(tau_rnea, tau_matrix, rtol=0.0, atol=1e-10)
            np.testing.assert_allclose(
                tau_rnea,
                arm.inverse_dynamics_rnea(q, v, qdd, u),
                rtol=0.0,
                atol=1e-12,
            )

    def test_ur5_forward_inverse_dynamics_roundtrip(self):
        arm = UR5Manipulator()
        rng = np.random.default_rng(1)
        for _ in range(8):
            q = rng.uniform(-1.0, 1.0, 6)
            v = rng.uniform(-1.0, 1.0, 6)
            u = rng.uniform(-5.0, 5.0, 6)
            qdd = arm.forward_dynamics(q, v, u)
            tau = arm.inverse_dynamics(q, v, qdd, u)
            np.testing.assert_allclose(
                tau, arm.generalized_force(q, v, u), rtol=0.0, atol=1e-10
            )
            qdd_rnea_h = arm.forward_dynamics_rnea_h(q, v, u)
            np.testing.assert_allclose(
                arm.inverse_dynamics_matrix(q, v, qdd_rnea_h, u),
                arm.generalized_force(q, v, u),
                rtol=0.0,
                atol=1e-10,
            )

    def test_ur5_params_override_and_skin(self):
        arm = UR5Manipulator()
        q = np.linspace(-0.4, 0.3, 6)
        params = dict(arm.params)
        params["gravity"] = 0.0
        np.testing.assert_allclose(arm.g(q, params), np.zeros(6), atol=1e-12)

        x = arm.q2x(q, np.zeros(6))
        frames = arm.tf(x, np.zeros(6))
        np.testing.assert_allclose(frames["joint6"][:3, 3], arm.forward_kinematics(q))
        geometry_smoke(arm, x, np.zeros(6))

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


import minilink.catalog as catalog
from minilink.dynamics.abstraction.mechanical import MechanicalSystem


class TestMechanicalJointPortLabels(unittest.TestCase):
    def test_a_mechanical_plant_has_no_joint_slice_ports(self):
        checked = []
        for name in catalog.__all__:
            cls = getattr(catalog, name)
            if not issubclass(cls, MechanicalSystem):
                continue
            with self.subTest(plant=name):
                plant = cls()
                self.assertNotIn("q", plant.outputs)
                self.assertNotIn("dq", plant.outputs)
                self.assertEqual(plant.outputs["y"].dim, 2 * plant.dof)
                self.assertEqual(
                    list(plant.outputs["y"].labels), list(plant.state.labels)
                )
                self.assertEqual(
                    list(plant.outputs["y"].units), list(plant.state.units)
                )
            checked.append(name)
        self.assertIn("Pendulum", checked)
        self.assertIn("TwoLinkManipulator", checked)


import pytest

from minilink.core.backends import configure_jax
from examples.projects.car_trajopt.vehicles.ladder import (
    BicycleDynEngine,
    BicycleDynServo,
)
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import BicycleDynRate


@pytest.mark.jax
class TestBicycleDynRate(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynRate(named_ports=True)
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


@pytest.mark.jax
class TestBicycleDynServo(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynServo(named_ports=True)
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

    def test_f_matches_named_servo_inputs_with_large_steer_command(self):
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 33.0, 0.0, 0.0])
        u = np.array([0.0, 1.0])
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


@pytest.mark.jax
class TestBicycleDynEngine(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynEngine(named_ports=True)
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
        from examples.projects.car_trajopt.vehicles.car_profile import (
            CAR_PROFILES,
            get_car_profile,
            list_car_profiles,
        )

        self.assertEqual(list_car_profiles(), ("passenger_car", "racecar", "udes_1_5"))
        for name in list_car_profiles():
            self.assertIs(get_car_profile(name), CAR_PROFILES[name])

    def test_racecar_matches_demo_vehicle_geometry(self):
        from examples.projects.car_trajopt.vehicles.car_profile import racecar_profile

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
        from examples.projects.car_trajopt.vehicles.car_profile import (
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
        from examples.projects.car_trajopt.vehicles.car_profile import (
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
        from examples.projects.car_trajopt.vehicles.car_profile import udes_1_5_profile
        from minilink.catalog import UdeSRacecar

        udes = UdeSRacecar()
        profile = udes_1_5_profile()
        self.assertAlmostEqual(profile.a, udes.params["a"])
        self.assertAlmostEqual(profile.b, udes.params["b"])
        self.assertAlmostEqual(profile.length, udes.params["length"])

    def test_apply_car_profile_rate_plant(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            apply_car_profile,
            passenger_car_profile,
        )

        sys = BicycleDynRate()
        apply_car_profile(sys, passenger_car_profile())
        profile = passenger_car_profile()
        self.assertAlmostEqual(sys.params["mass"], profile.mass)
        self.assertAlmostEqual(sys.params["a"] + sys.params["b"], profile.a + profile.b)
        self.assertAlmostEqual(sys.params["Ca"], profile.Ca)
        self.assertAlmostEqual(sys.params["a"], profile.a)
        self.assertAlmostEqual(sys.params["b"], profile.b)
        self.assertFalse(hasattr(sys, "tire_model_f"))
        self.assertAlmostEqual(sys.state.upper_bound[6], profile.limits.w_rear_max)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.limits.w_rear_dot_max
        )
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[1], profile.limits.delta_dot_max
        )

    def test_apply_car_profile_servo_plant(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
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
        from examples.projects.car_trajopt.vehicles.car_profile import (
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


# The params family: a method reads the params it is given

from minilink.dynamics.catalog.manipulators.arms import ThreeLinkManipulator3D
from minilink.dynamics.catalog.pendulum.cartpole import (
    RotatingCartPole,
    UnderactuatedRotatingCartPole,
)
from minilink.dynamics.catalog.mass_spring_damper.linear import FloatingTwoMass
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D
from minilink.dynamics.catalog.vehicles.steering import KinematicCar


def stored(cls, **attributes):
    """A factory of ``cls`` whose stored params are the ones given (the defaults for None)."""

    def make(params=None):
        plant = cls()
        for name, value in attributes.items():
            setattr(plant, name, value)
        if params is not None:
            plant.params = params
        return plant

    make.__name__ = cls.__name__
    make.defaults = lambda: cls().params
    return make


def speed_two_link(params=None):
    """A velocity-controlled two-link arm; ``params`` are the arm's."""
    arm = TwoLinkManipulator()
    if params is not None:
        arm.params = params
    return SpeedControlledManipulator.from_manipulator(arm)


speed_two_link.defaults = lambda: TwoLinkManipulator().params


PARAMS_CASES = [
    # The arms: kinematics, effector ports, frames and torque arcs
    (stored(OneLinkManipulator), "forward_kinematics", "l1", 3.0),
    (stored(OneLinkManipulator), "J", "l1", 3.0),
    (stored(OneLinkManipulator), "h_p", "l1", 3.0),
    (stored(OneLinkManipulator), "h_pdot", "l1", 3.0),
    (stored(OneLinkManipulator), "get_dynamic_geometry", "l1", 3.0),
    (stored(TwoLinkManipulator), "forward_kinematics", "l1", 0.7),
    (stored(TwoLinkManipulator), "J", "l2", 0.4),
    (stored(TwoLinkManipulator), "h_p", "l1", 0.7),
    (stored(TwoLinkManipulator), "h_pdot", "l2", 0.4),
    (stored(TwoLinkManipulator), "tf", "l1", 0.7),
    (stored(TwoLinkManipulator), "get_dynamic_geometry", "l1", 0.7),
    (stored(ThreeLinkManipulator3D), "forward_kinematics", "l1", 1.4),
    (stored(ThreeLinkManipulator3D), "J", "l3", 1.4),
    (stored(ThreeLinkManipulator3D), "h_p", "l2", 1.4),
    (stored(ThreeLinkManipulator3D), "h_pdot", "l3", 1.4),
    (stored(ThreeLinkManipulator3D), "tf", "l2", 1.4),
    (stored(FiveLinkPlanarManipulator), "forward_kinematics", "l", np.full(5, 0.6)),
    (stored(FiveLinkPlanarManipulator), "J", "l", np.full(5, 0.6)),
    (stored(FiveLinkPlanarManipulator), "h_p", "l", np.full(5, 0.6)),
    (stored(FiveLinkPlanarManipulator), "h_pdot", "l", np.full(5, 0.6)),
    (stored(FiveLinkPlanarManipulator), "tf", "l", np.full(5, 0.6)),
    (stored(FiveLinkPlanarManipulator), "get_dynamic_geometry", "l", np.full(5, 0.6)),
    (speed_two_link, "tf", "l1", 0.7),
    (speed_two_link, "get_dynamic_geometry", "l2", 0.4),
    # The double pendulum and the acrobot: frames and torque arcs
    (stored(DoublePendulum), "tf", "l1", 1.5),
    (stored(DoublePendulum), "get_dynamic_geometry", "l1", 1.5),
    (stored(Acrobot), "tf", "l1", 1.5),
    (stored(Acrobot), "get_dynamic_geometry", "l1", 1.5),
    # The rotating cart-poles: frames
    (stored(RotatingCartPole), "tf", "l1", 1.4),
    (stored(UnderactuatedRotatingCartPole), "tf", "l2", 1.4),
    # The single pendulums: frames and torque arcs
    (stored(Pendulum), "get_dynamic_geometry", "l", 1.5),
    (stored(InvertedPendulum), "get_dynamic_geometry", "l", 1.5),
    (stored(TwoIndependentPendulums), "tf", "l", 1.5),
    (stored(TwoIndependentPendulums), "get_dynamic_geometry", "l", 1.5),
    # The mass-spring-damper chains: the ground spring drawn only when k ≠ 0
    (stored(SingleMass), "get_dynamic_geometry", "k", 0.0),
    (stored(TwoMass), "get_dynamic_geometry", "k1", 0.0),
    (stored(ThreeMass), "get_dynamic_geometry", "k1", 0.0),
    (stored(FloatingTwoMass), "get_dynamic_geometry", "k1", 1.0),
    # The mountain car: the bead on the hill
    (stored(MountainCar), "tf", "a", 0.7),
    (stored(MountainCar), "forward_kinematic_effector", "a", 0.7),
    # The dynamic bicycles: contact velocity and tire force arrows
    (stored(DynamicBicycle), "get_dynamic_geometry", "a", 1.3),
    (stored(DynamicBicycle), "get_dynamic_geometry", "Ca", 80000.0),
    (stored(DynamicBicycleCar3D), "get_dynamic_geometry", "r_f", 0.4),
    (stored(DynamicBicycleCar3D), "get_dynamic_geometry", "Ca", 80000.0),
    # The kinematic car: its steered front wheels
    (stored(KinematicCar), "tf", "length", 6.0),
    # The longitudinal cars: the traction arrow
    (
        stored(LongitudinalFrontWheelDriveCarWithWheelSlipInput),
        "get_dynamic_geometry",
        "length",
        3.0,
    ),
    (
        stored(LongitudinalFrontWheelDriveCarWithWheelSlipInput),
        "get_dynamic_geometry",
        "mu_max",
        1.2,
    ),
    (
        stored(LongitudinalFrontWheelDriveCarWithTorqueInput),
        "get_dynamic_geometry",
        "mu_max",
        1.2,
    ),
    (
        # Rolling at 10 m/s with the wheel near its rolling speed: the slip is not clipped
        stored(
            LongitudinalFrontWheelDriveCarWithTorqueInput,
            x0=np.array([0.0, 10.0, 33.0, 0.0]),
        ),
        "get_dynamic_geometry",
        "wheel_radius",
        0.4,
    ),
    # The quarter car: the spring down to the road
    (stored(QuarterCarOnRoughTerrain), "get_dynamic_geometry", "a", np.full(6, 0.4)),
    # The planar aircraft: its chords and its lift and drag arrows
    (stored(Plane2D), "tf", "l_t", 1.3),
    (stored(Plane2D), "get_dynamic_geometry", "rho", 1.5),
    # The boats: the thrust, hydrodynamic and current arrows
    (stored(Boat2D), "get_dynamic_geometry", "l_t", 4.0),
    (
        stored(Boat2D, show_hydrodynamic_forces=True),
        "get_dynamic_geometry",
        "damping_coef",
        np.array([20.0, 20.0, 200.0]),
    ),
    (stored(Boat2DWithCurrent), "tf", "loa", 7.0),
    (
        stored(Boat2DWithCurrent),
        "get_dynamic_geometry",
        "current_velocity",
        np.array([1.0, -0.5]),
    ),
    # The planar drones: the thrust arrows at the thrusters
    (stored(Drone2D), "get_dynamic_geometry", "thruster_offset", 0.7),
    (stored(Drone2DWithSideThruster), "get_dynamic_geometry", "thruster_offset", 0.7),
]


def opened(value):
    """A drawing opened into nested values: each primitive into its fields."""
    if isinstance(value, dict):
        return {key: opened(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [opened(item) for item in value]
    if hasattr(value, "__dict__"):
        return {key: opened(item) for key, item in vars(value).items()}
    return np.asarray(value)


def method_args(plant, method):
    """A state and input off the nominal ones; the joint angles for the kinematics."""
    rng = np.random.default_rng(0)
    x0 = np.asarray(plant.x0, dtype=float)
    u0 = np.asarray(plant.get_u_from_input_ports(), dtype=float)
    x = x0 + 0.3 * rng.standard_normal(x0.shape)
    u = u0 + 0.5 + 0.3 * rng.standard_normal(u0.shape)
    if method in ("forward_kinematics", "J", "forward_kinematic_effector"):
        return (x[: plant.dof],)
    return (x, u, 0.0)


def case_id(value):
    return getattr(value, "__name__", None)


@pytest.mark.parametrize("make, method, key, value", PARAMS_CASES, ids=case_id)
def test_a_method_reads_the_params_it_is_given(make, method, key, value):
    plant = make()
    params = {**make.defaults(), key: value}
    args = method_args(plant, method)

    given = opened(getattr(plant, method)(*args, params=params))
    from_stored = opened(getattr(make(params), method)(*args))
    default = opened(getattr(plant, method)(*args))

    np.testing.assert_equal(given, from_stored)
    with pytest.raises(AssertionError):
        np.testing.assert_equal(given, default)


# The pathtracking project's vehicle: the rolling resistance, the tires, the inputs

from examples.projects.pathtracking.common.vehicle import (
    LinearTire,
    Pacejka,
    RearDriveBicycle,
    create_vehicle,
)


class TestPathtrackingVehicle(unittest.TestCase):
    def test_the_front_rolling_resistance_opposes_the_front_wheel_spin(self):
        vehicle = create_vehicle()
        vehicle.rolling_friction_constant = 0.015
        v = np.array(
            [0.0, 0.0, 0.0, 2.0, -2.0]
        )  # The rear spins forward, the front back
        u_in = np.array([0.0, 0.0])
        Fx_front, _, _, _ = vehicle.compute_tire_physics(v, u_in)
        Fz_f = vehicle.mass * vehicle.gravity * vehicle.b / vehicle.L
        d = vehicle.generalized_d(np.zeros(3), v, u_in)
        tau_front = vehicle.r_f * Fx_front + vehicle.bw_front * v[4] - 0.015 * Fz_f
        self.assertAlmostEqual(d[4], tau_front, places=9)

    def test_the_lateral_weight_is_one_without_longitudinal_slip(self):
        # G_yκ(κ = 0) = 1 whatever the curvature E (Pacejka 2006, §4.3.2)
        tire = Pacejka()
        tire.r_Ey1 = 0.5  # E = r_Ey1 + r_Ey2 d_fz = −0.5
        for alpha in (0.0, 0.1, -0.2):
            self.assertEqual(tire.Gyk(0.0, alpha), 1.0)
        self.assertLess(tire.Gyk(0.3, 0.1), 1.0)

    def test_the_rear_drive_bicycle_leaves_the_callers_input_alone(self):
        vehicle = RearDriveBicycle()
        vehicle.tire_model_f = LinearTire()
        vehicle.tire_model_r = LinearTire()
        x = np.zeros(8)
        x[3] = 5.0
        x[6] = 5.0 / vehicle.r_r
        x[7] = 5.0 / vehicle.r_f
        u = np.array([10.0, 2.0])  # A steer command past the limit
        dx = vehicle.f(x, u)
        np.testing.assert_array_equal(u, [10.0, 2.0])
        clipped = np.array([10.0, vehicle.max_steer])
        np.testing.assert_array_equal(dx, vehicle.f(x, clipped))

    def test_the_friction_circle_caps_each_contact_of_an_array(self):
        # One contact inside the circle, one beyond it: each as its own scalar call
        alpha = np.array([0.01, 0.5])
        kappa = np.array([0.005, 0.9])
        Fz = np.array([3000.0, 3000.0])
        for tire in (LinearTire(), Pacejka(combined_slip_mode="c")):
            with self.subTest(tire=type(tire).__name__):
                Fx, Fy = tire.slip2forces(alpha, kappa, Fz)
                for i in range(2):
                    Fx_i, Fy_i = tire.slip2forces(alpha[i], kappa[i], Fz[i])
                    self.assertEqual((Fx[i], Fy[i]), (Fx_i, Fy_i))
                self.assertLessEqual(
                    np.hypot(Fx[1], Fy[1]), tire.mu * Fz[1] * (1 + 1e-12)
                )


# The pathtracking project's path helpers: the point ahead along the path, both copies

PATH_MODULES = (
    "examples.projects.pathtracking.bicycle_los.los",
    "examples.projects.pathtracking.bicycle_los_v2.path_tracking",
)


@pytest.mark.parametrize("module", PATH_MODULES)
def test_the_point_ahead_starts_on_the_next_segment_after_a_degenerate_one(module):
    import importlib

    ahead = importlib.import_module(module).point_ahead_along_path
    for path in (
        [(0.0, 0.0), (0.0, 0.0), (4.0, 0.0), (4.0, 3.0)],
        [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (4.0, 0.0), (4.0, 3.0)],
    ):
        ax, ay, i, lam, heading = ahead(path, 0, 0.7, 1.0, closed=False)
        assert (ax, ay, heading) == (1.0, 0.0, 0.0)
        assert i == len(path) - 3


@pytest.mark.parametrize("module", PATH_MODULES)
def test_an_open_path_stops_at_its_end_with_its_last_heading(module):
    import importlib

    ahead = importlib.import_module(module).point_ahead_along_path
    square = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
    reached = ahead(square, 1, 0.2, 9.0, closed=False)
    assert reached == (0.0, 3.0, 2, 1.0, np.pi)
    assert [type(value) for value in reached] == [float, float, int, float, float]
    clamped = ahead(square, 3, 0.5, 1.0, closed=False)  # No closing segment
    assert clamped[:3] == (1.0, 3.0, 2)
    with pytest.raises(ValueError, match="non-negative"):
        ahead(square, 1, 0.2, -1.0, closed=False)
    wrapped = ahead(square, 0, 0.5, -1.0, closed=True)  # A loop wraps a negative ds
    assert [type(value) for value in wrapped] == [float, float, int, float, float]


def test_a_closed_rounded_rectangle_does_not_repeat_its_first_waypoint():
    from examples.projects.pathtracking.common.path_generator import (
        rounded_rectangle_path,
    )

    closed = rounded_rectangle_path(40.0, 20.0, 7.0, nseg=2, narc=4, closed=True)
    open_loop = rounded_rectangle_path(40.0, 20.0, 7.0, nseg=2, narc=4, closed=False)
    # The closing segment returns to the first waypoint: no zero-length segment
    assert np.hypot(*(closed[-1] - closed[0])) > 0.1
    # The open path runs the whole loop, back onto its first waypoint
    np.testing.assert_array_equal(open_loop[-1], open_loop[0])
    np.testing.assert_array_equal(open_loop[:-1], closed)
