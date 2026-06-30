import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.blocks.sources import Step, WhiteNoise
from minilink.control.linear import PDController, ProportionalController
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestDiagramCompositionShortcuts(unittest.TestCase):
    def test_add_operator_adds_subsystems_without_wiring(self):
        diagram = Step(final_value=[1.0]) + ProportionalController() + Integrator()

        self.assertIsInstance(diagram, DiagramSystem)
        self.assertEqual(
            list(diagram.subsystems),
            ["step", "p_controller", "integrator"],
        )
        self.assertEqual(diagram.connections["p_controller"]["r"], None)
        self.assertEqual(diagram.connections["p_controller"]["y"], None)
        self.assertEqual(diagram.connections["integrator"]["u"], None)

    def test_add_operator_uniquifies_repeated_subsystem_ids(self):
        diagram = Integrator() + Integrator()

        self.assertEqual(list(diagram.subsystems), ["integrator", "integrator_2"])
        self.assertEqual(diagram.connections["integrator"]["u"], None)
        self.assertEqual(diagram.connections["integrator_2"]["u"], None)

    def test_series_operator_wires_source_to_default_input(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> Integrator()

        self.assertEqual(diagram.connections["integrator"]["u"], ("step", "y"))
        self.assertEqual(diagram.connections["output"]["y"], ("integrator", "y"))

    def test_series_operator_exposes_first_input_and_final_output(self):
        diagram = Integrator() >> Integrator()

        self.assertIn("u", diagram.inputs)
        self.assertEqual(diagram.connections["integrator"]["u"], ("input", "u"))
        self.assertEqual(
            diagram.connections["integrator_2"]["u"],
            ("integrator", "y"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("integrator_2", "y"))

        dx = diagram.f(np.array([1.0, 2.0]), np.array([3.0]))
        np.testing.assert_allclose(dx, np.array([3.0, 1.0]))

    def test_matmul_operator_builds_closed_loop_diagram(self):
        diagram = PDController() @ Pendulum()

        self.assertIn("r", diagram.inputs)
        self.assertIn("y", diagram.outputs)
        self.assertEqual(
            diagram.connections["pd_controller"]["r"],
            ("input", "r"),
        )
        self.assertEqual(
            diagram.connections["pd_controller"]["y"],
            ("pendulum", "y"),
        )
        self.assertEqual(
            diagram.connections["pendulum"]["u"],
            ("pd_controller", "u"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("pendulum", "y"))
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
            diagram.connections["linear_state_feedback_controller"]["x"],
            ("cart_pole", "x"),
        )
        self.assertEqual(
            diagram.connections["cart_pole"]["u"],
            ("linear_state_feedback_controller", "u"),
        )

    def test_series_operator_flattens_source_into_closed_loop_diagram(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> PDController() @ Pendulum()

        self.assertEqual(
            list(diagram.subsystems), ["step", "pd_controller", "pendulum"]
        )
        self.assertFalse(
            any(isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values())
        )
        self.assertEqual(diagram.connections["pd_controller"]["r"], ("step", "y"))
        self.assertEqual(diagram.connections["pd_controller"]["y"], ("pendulum", "y"))
        self.assertEqual(diagram.connections["pendulum"]["u"], ("pd_controller", "u"))
        self.assertEqual(diagram.connections["output"]["y"], ("pendulum", "y"))
        self.assertNotIn("r", diagram.inputs)

    def test_add_operator_flattens_diagrams_without_cross_wiring(self):
        left = Step(final_value=[1.0]) + Integrator()
        right = PDController() @ Pendulum()

        diagram = left + right

        self.assertEqual(
            list(diagram.subsystems),
            ["step", "integrator", "pd_controller", "pendulum"],
        )
        self.assertFalse(
            any(isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values())
        )
        self.assertEqual(diagram.connections["integrator"]["u"], None)
        self.assertEqual(diagram.connections["pd_controller"]["r"], ("input", "r"))
        self.assertEqual(diagram.connections["pd_controller"]["y"], ("pendulum", "y"))
        self.assertEqual(diagram.connections["pendulum"]["u"], ("pd_controller", "u"))

    def test_series_operator_flattens_diagram_to_diagram_boundary(self):
        left = Step(final_value=[1.0], step_time=0.0) >> ProportionalController()
        right = Integrator() >> Integrator()

        diagram = left >> right

        self.assertEqual(
            list(diagram.subsystems),
            ["step", "p_controller", "integrator", "integrator_2"],
        )
        self.assertFalse(
            any(isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values())
        )
        self.assertEqual(diagram.connections["p_controller"]["r"], ("step", "y"))
        self.assertEqual(diagram.connections["integrator"]["u"], ("p_controller", "u"))
        self.assertEqual(
            diagram.connections["integrator_2"]["u"],
            ("integrator", "y"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("integrator_2", "y"))
        self.assertNotIn("u", diagram.inputs)

    def test_series_into_closed_loop_without_free_boundary_input_fails(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> PDController() @ Pendulum()

        with self.assertRaisesRegex(ValueError, "no available boundary input"):
            WhiteNoise() >> diagram

        self.assertEqual(diagram.connections["pendulum"]["u"], ("pd_controller", "u"))

    def test_autowire_connects_unique_matches_without_overwrites(self):
        diagram = (
            Step(final_value=[1.0]) + ProportionalController() + Integrator()
        ).autowire(strict=True)

        self.assertEqual(
            diagram.connections["p_controller"]["r"],
            ("step", "y"),
        )
        self.assertEqual(
            diagram.connections["p_controller"]["y"],
            ("integrator", "y"),
        )
        self.assertEqual(
            diagram.connections["integrator"]["u"],
            ("p_controller", "u"),
        )

        diagram.connect("step", "y", "integrator", "u")
        diagram.autowire(strict=True)
        self.assertEqual(diagram.connections["integrator"]["u"], ("step", "y"))

    def test_autowire_handles_pendulum_closed_loop_convention(self):
        diagram = (Step(final_value=[1.0]) + PDController() + Pendulum()).autowire(
            strict=True
        )

        self.assertEqual(
            diagram.connections["pd_controller"]["r"],
            ("step", "y"),
        )
        self.assertEqual(
            diagram.connections["pd_controller"]["y"],
            ("pendulum", "y"),
        )
        self.assertEqual(
            diagram.connections["pendulum"]["u"],
            ("pd_controller", "u"),
        )

    def test_autowire_strict_refuses_ambiguous_matches(self):
        diagram = ProportionalController() + ProportionalController() + Integrator()

        with self.assertRaisesRegex(ValueError, "Ambiguous autowire target"):
            diagram.autowire(strict=True)

        self.assertEqual(diagram.connections["p_controller"]["y"], None)
        self.assertEqual(diagram.connections["p_controller_2"]["y"], None)
        self.assertEqual(diagram.connections["integrator"]["u"], None)


if __name__ == "__main__":
    unittest.main()
