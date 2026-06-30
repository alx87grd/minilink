import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.blocks.sources import Step, WhiteNoise
from minilink.control.impedance import ImpedanceController
from minilink.control.output import ProportionalController
from minilink.core.composition import closed_loop, closed_loop_qdq
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class AugmentedMechanicalPlant(MechanicalSystem):
    """Mechanical plant with extra actuator state in ``x`` (``y = x``)."""

    def __init__(self):
        super().__init__(dof=1, actuators=1)
        self.n = 3
        self.state.labels = ["theta", "dtheta", "z_act"]
        self.state.units = ["rad", "rad/s", ""]
        self.x0 = np.zeros(3)
        self.outputs["y"].dim = 3
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.outputs["y"].nominal_value = np.zeros(3)

    def x2q(self, x):
        return x[0:1], x[1:2]

    def f(self, x, u, t=0.0, params=None):
        q, v = self.x2q(x)
        acceleration = self.forward_dynamics(q, v, u, t, params)
        xp = np.asarray(acceleration).reshape(-1)[0]
        return np.array([v[0], xp, 0.0])


class TestDiagramCompositionShortcuts(unittest.TestCase):
    def test_add_operator_adds_subsystems_without_wiring(self):
        diagram = Step(final_value=[1.0]) + ProportionalController() + Integrator()

        self.assertIsInstance(diagram, DiagramSystem)
        self.assertEqual(
            list(diagram.subsystems),
            ["ref", "ctl", "sys"],
        )
        self.assertEqual(diagram.connections["ctl"]["r"], None)
        self.assertEqual(diagram.connections["ctl"]["y"], None)
        self.assertEqual(diagram.connections["sys"]["u"], None)

    def test_add_operator_uniquifies_repeated_subsystem_ids(self):
        diagram = Integrator() + Integrator()

        self.assertEqual(list(diagram.subsystems), ["sys", "sys2"])
        self.assertEqual(diagram.connections["sys"]["u"], None)
        self.assertEqual(diagram.connections["sys2"]["u"], None)

    def test_series_operator_wires_source_to_default_input(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> Integrator()

        self.assertEqual(diagram.connections["sys"]["u"], ("ref", "y"))
        self.assertEqual(diagram.connections["output"]["y"], ("sys", "y"))

    def test_series_operator_exposes_first_input_and_final_output(self):
        diagram = Integrator() >> Integrator()

        self.assertIn("u", diagram.inputs)
        self.assertEqual(diagram.connections["sys"]["u"], ("input", "u"))
        self.assertEqual(
            diagram.connections["sys2"]["u"],
            ("sys", "y"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("sys2", "y"))

        dx = diagram.f(np.array([1.0, 2.0]), np.array([3.0]))
        np.testing.assert_allclose(dx, np.array([3.0, 1.0]))

    def test_matmul_operator_builds_closed_loop_diagram(self):
        diagram = ImpedanceController() @ Pendulum()

        self.assertIn("r", diagram.inputs)
        self.assertIn("y", diagram.outputs)
        self.assertEqual(
            diagram.connections["ctl"]["r"],
            ("input", "r"),
        )
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("sys", "y"),
        )
        self.assertEqual(
            diagram.connections["sys"]["u"],
            ("ctl", "u"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("sys", "y"))
        self.assertEqual(diagram.outputs["y"].dim, 2)

    def test_matmul_operator_wires_state_feedback_when_controller_uses_x(self):
        from minilink.control.lqr import lqr_at_operating_point
        from minilink.dynamics.catalog.pendulum.cartpole import CartPole

        plant = CartPole()
        controller = lqr_at_operating_point(
            plant,
            [0.0, np.pi, 0.0, 0.0],
            np.diag([1.0, 10.0, 1.0, 1.0]),
            np.array([[0.1]]),
        )
        diagram = controller @ plant

        self.assertEqual(
            diagram.connections["ctl"]["x"],
            ("sys", "x"),
        )
        self.assertEqual(
            diagram.connections["sys"]["u"],
            ("ctl", "u"),
        )

    def test_series_operator_flattens_source_into_closed_loop_diagram(self):
        diagram = (
            Step(final_value=[1.0], step_time=0.0) >> ImpedanceController() @ Pendulum()
        )

        self.assertEqual(list(diagram.subsystems), ["ref", "ctl", "sys"])
        self.assertFalse(
            any(isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values())
        )
        self.assertEqual(diagram.connections["ctl"]["r"], ("ref", "y"))
        self.assertEqual(diagram.connections["ctl"]["y"], ("sys", "y"))
        self.assertEqual(diagram.connections["sys"]["u"], ("ctl", "u"))
        self.assertEqual(diagram.connections["output"]["y"], ("sys", "y"))
        self.assertNotIn("r", diagram.inputs)

    def test_add_operator_flattens_diagrams_without_cross_wiring(self):
        left = Step(final_value=[1.0]) + Integrator()
        right = ImpedanceController() @ Pendulum()

        diagram = left + right

        self.assertEqual(
            list(diagram.subsystems),
            ["ref", "sys", "ctl", "sys2"],
        )
        self.assertFalse(
            any(isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values())
        )
        self.assertEqual(diagram.connections["sys"]["u"], None)
        self.assertEqual(diagram.connections["ctl"]["r"], ("input", "r"))
        self.assertEqual(diagram.connections["ctl"]["y"], ("sys2", "y"))
        self.assertEqual(diagram.connections["sys2"]["u"], ("ctl", "u"))

    def test_series_operator_flattens_diagram_to_diagram_boundary(self):
        left = Step(final_value=[1.0], step_time=0.0) >> ProportionalController()
        right = Integrator() >> Integrator()

        diagram = left >> right

        self.assertEqual(
            list(diagram.subsystems),
            ["ref", "ctl", "sys", "sys2"],
        )
        self.assertFalse(
            any(isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values())
        )
        self.assertEqual(diagram.connections["ctl"]["r"], ("ref", "y"))
        self.assertEqual(diagram.connections["sys"]["u"], ("ctl", "u"))
        self.assertEqual(
            diagram.connections["sys2"]["u"],
            ("sys", "y"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("sys2", "y"))
        self.assertNotIn("u", diagram.inputs)

    def test_series_into_closed_loop_without_free_boundary_input_fails(self):
        diagram = (
            Step(final_value=[1.0], step_time=0.0) >> ImpedanceController() @ Pendulum()
        )

        with self.assertRaisesRegex(ValueError, "no available boundary input"):
            WhiteNoise() >> diagram

        self.assertEqual(diagram.connections["sys"]["u"], ("ctl", "u"))

    def test_autowire_connects_unique_matches_without_overwrites(self):
        diagram = (
            Step(final_value=[1.0]) + ProportionalController() + Integrator()
        ).autowire(strict=True)

        self.assertEqual(
            diagram.connections["ctl"]["r"],
            ("ref", "y"),
        )
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("sys", "y"),
        )
        self.assertEqual(
            diagram.connections["sys"]["u"],
            ("ctl", "u"),
        )

        diagram.connect("ref", "y", "sys", "u")
        diagram.autowire(strict=True)
        self.assertEqual(diagram.connections["sys"]["u"], ("ref", "y"))

    def test_autowire_handles_pendulum_closed_loop_convention(self):
        diagram = (
            Step(final_value=[1.0]) + ImpedanceController() + Pendulum()
        ).autowire(strict=True)

        self.assertEqual(
            diagram.connections["ctl"]["r"],
            ("ref", "y"),
        )
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("sys", "y"),
        )
        self.assertEqual(
            diagram.connections["sys"]["u"],
            ("ctl", "u"),
        )

    def test_autowire_strict_refuses_ambiguous_matches(self):
        diagram = ProportionalController() + ProportionalController() + Integrator()

        with self.assertRaisesRegex(ValueError, "Ambiguous autowire target"):
            diagram.autowire(strict=True)

        self.assertEqual(diagram.connections["ctl"]["y"], None)
        self.assertEqual(diagram.connections["ctl2"]["y"], None)
        self.assertEqual(diagram.connections["sys"]["u"], None)

    def test_closed_loop_qdq_inserts_mux(self):
        diagram = closed_loop_qdq(ImpedanceController(), Pendulum())

        self.assertIn("mux", diagram.subsystems)
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("mux", "y"),
        )
        self.assertEqual(
            diagram.connections["mux"]["in0"],
            ("sys", "q"),
        )
        self.assertEqual(
            diagram.connections["mux"]["in1"],
            ("sys", "dq"),
        )

    def test_closed_loop_qdq_two_link_requires_matching_controller_dim(self):
        with self.assertRaisesRegex(ValueError, "feedback='qdq' expects"):
            closed_loop_qdq(ImpedanceController(dof=1), TwoLinkManipulator())

    def test_closed_loop_qdq_two_link(self):
        diagram = closed_loop_qdq(ImpedanceController(dof=2), TwoLinkManipulator())
        self.assertIn("mux", diagram.subsystems)

    def test_closed_loop_auto_uses_mux_for_augmented_plant(self):
        diagram = closed_loop(ImpedanceController(), AugmentedMechanicalPlant())

        self.assertIn("mux", diagram.subsystems)
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("mux", "y"),
        )

    def test_closed_loop_feedback_y_raises_on_dim_mismatch(self):
        with self.assertRaisesRegex(ValueError, "Cannot wire closed-loop feedback"):
            closed_loop(ImpedanceController(), AugmentedMechanicalPlant(), feedback="y")


if __name__ == "__main__":
    unittest.main()
