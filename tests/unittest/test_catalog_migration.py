import unittest

import numpy as np

from minilink.blocks.transfer_function import TransferFunction
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
    ThreeLinkManipulator3D,
    TwoLinkManipulator,
)
from minilink.dynamics.catalog.marine.boat import Boat2D, Boat2DWithCurrent
from minilink.dynamics.catalog.mass_spring_damper.linear import (
    FloatingSingleMass,
    FloatingThreeMass,
    FloatingTwoMass,
    SingleMass,
    ThreeMass,
    TwoMass,
)
from minilink.dynamics.catalog.pendulum.cartpole import (
    CartPole,
    RotatingCartPole,
    UnderactuatedRotatingCartPole,
)
from minilink.dynamics.catalog.pendulum.double_pendulum import Acrobot
from minilink.dynamics.catalog.pendulum.pendulum import (
    InvertedPendulum,
    Pendulum,
    TwoIndependentPendulums,
)
from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
from minilink.dynamics.catalog.vehicles.propulsion import (
    LongitudinalFrontWheelDriveCarWithTorqueInput,
    LongitudinalFrontWheelDriveCarWithWheelSlipInput,
)
from minilink.dynamics.catalog.vehicles.steering import (
    ConstantSpeedKinematicCar,
    HolonomicMobileRobot,
    HolonomicMobileRobot3D,
    KinematicBicycle,
    KinematicCar,
    UdeSRacecar,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCarOnRoughTerrain
from minilink.graphical.animation.primitives import Arrow, TorqueArrow
from tests.unittest.graphics_contract_helpers import (
    geometry_smoke as _geometry_smoke,
)
from tests.unittest.graphics_contract_helpers import (
    resolved_primitive_count as _primitive_count,
)


def _zero_f_smoke(system):
    x = np.asarray(system.x0, dtype=float)
    if x.shape != (system.n,):
        x = np.zeros(system.n)
    u = system.get_u_from_input_ports()
    dx = np.asarray(system.f(x, u, 0.0), dtype=float)
    assert dx.shape == (system.n,)
    assert np.all(np.isfinite(dx))


class TestCatalogSmoke(unittest.TestCase):
    def test_low_risk_equation_reference_values(self):
        np.testing.assert_allclose(
            SimpleIntegrator().f(np.array([2.0]), np.array([3.0])),
            [3.0],
        )
        np.testing.assert_allclose(
            DoubleIntegrator().f(np.array([2.0, 4.0]), np.array([3.0])),
            [4.0, 3.0],
        )
        np.testing.assert_allclose(
            TripleIntegrator().f(np.array([2.0, 4.0, 6.0]), np.array([3.0])),
            [4.0, 6.0, 3.0],
        )
        np.testing.assert_allclose(
            VanderPol(mu=0.5).f(np.array([1.0, 2.0]), np.array([0.0])),
            [2.0, -1.0],
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
            bicycle.f(np.array([0.0, 0.0, 0.0]), np.array([2.0, 0.0])),
            [2.0, 0.0, 0.0],
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
            drone.forward_dynamics(q, dq, hover),
            np.zeros(3),
            atol=1e-12,
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
            one.forward_kinematic_effector(np.array([0.0])),
            [0.0, one.params["l1"]],
        )

        five = FiveLinkPlanarManipulator()
        np.testing.assert_allclose(
            five.forward_kinematic_effector(np.zeros(5)),
            [0.0, np.sum(five.params["l"])],
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
            (KinematicBicycle(), 1),
            (HolonomicMobileRobot(), 1),
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

    def test_catalog_class_smoke(self):
        systems = [
            SimpleIntegrator(),
            DoubleIntegrator(),
            TripleIntegrator(),
            VanderPol(),
            TransferFunction([1.0], [1.0, 1.0]),
            SingleMass(),
            TwoMass(),
            ThreeMass(),
            FloatingSingleMass(),
            FloatingTwoMass(),
            FloatingThreeMass(),
            Pendulum(),
            InvertedPendulum(),
            TwoIndependentPendulums(),
            Acrobot(),
            RotatingCartPole(),
            UnderactuatedRotatingCartPole(),
            CartPole(),
            KinematicBicycle(),
            KinematicCar(),
            ConstantSpeedKinematicCar(),
            HolonomicMobileRobot(),
            HolonomicMobileRobot3D(),
            UdeSRacecar(),
            LongitudinalFrontWheelDriveCarWithWheelSlipInput(),
            LongitudinalFrontWheelDriveCarWithTorqueInput(),
            QuarterCarOnRoughTerrain(),
            MountainCar(),
            Drone2D(),
            Drone2DWithSideThruster(),
            SpeedControlledDrone2D(),
            ConstantSpeedHelicopterTunnel(),
            Rocket(),
            Plane2D(),
            Boat2D(),
            Boat2DWithCurrent(),
            SpeedControlledManipulator(2, 2),
            OneLinkManipulator(),
            TwoLinkManipulator(),
            ThreeLinkManipulator3D(),
            FiveLinkPlanarManipulator(),
        ]

        for system in systems:
            with self.subTest(system=system.name):
                _zero_f_smoke(system)
                _geometry_smoke(system)


if __name__ == "__main__":
    unittest.main()
