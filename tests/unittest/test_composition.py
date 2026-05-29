import unittest

import numpy as np

from minilink.control.pendulum_pd import PendulumPDController
from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestDiagramCompositionShortcuts(unittest.TestCase):
    def test_add_operator_adds_subsystems_without_wiring(self):
        diagram = Step(final_value=[1.0]) + PropController() + Integrator()

        self.assertIsInstance(diagram, DiagramSystem)
        self.assertEqual(
            list(diagram.subsystems),
            ["step", "controller", "integrator"],
        )
        self.assertEqual(diagram.connections["controller"]["r"], None)
        self.assertEqual(diagram.connections["controller"]["y"], None)
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
        diagram = PendulumPDController() @ Pendulum()

        self.assertIn("r", diagram.inputs)
        self.assertIn("y", diagram.outputs)
        self.assertEqual(
            diagram.connections["controller"]["r"],
            ("input", "r"),
        )
        self.assertEqual(
            diagram.connections["controller"]["y"],
            ("pendulum", "y"),
        )
        self.assertEqual(
            diagram.connections["pendulum"]["u"],
            ("controller", "u"),
        )
        self.assertEqual(diagram.connections["output"]["y"], ("pendulum", "y"))
        self.assertEqual(diagram.outputs["y"].dim, 2)

    def test_autowire_connects_unique_matches_without_overwrites(self):
        diagram = (Step(final_value=[1.0]) + PropController() + Integrator()).autowire(
            strict=True
        )

        self.assertEqual(
            diagram.connections["controller"]["r"],
            ("step", "y"),
        )
        self.assertEqual(
            diagram.connections["controller"]["y"],
            ("integrator", "y"),
        )
        self.assertEqual(
            diagram.connections["integrator"]["u"],
            ("controller", "u"),
        )

        diagram.connect("step", "y", "integrator", "u")
        diagram.autowire(strict=True)
        self.assertEqual(diagram.connections["integrator"]["u"], ("step", "y"))

    def test_autowire_handles_pendulum_closed_loop_convention(self):
        diagram = (
            Step(final_value=[1.0]) + PendulumPDController() + Pendulum()
        ).autowire(strict=True)

        self.assertEqual(
            diagram.connections["controller"]["r"],
            ("step", "y"),
        )
        self.assertEqual(
            diagram.connections["controller"]["y"],
            ("pendulum", "y"),
        )
        self.assertEqual(
            diagram.connections["pendulum"]["u"],
            ("controller", "u"),
        )
        self.assertEqual(diagram.connections["pendulum"]["w"], None)
        self.assertEqual(diagram.connections["pendulum"]["v"], None)

    def test_autowire_strict_refuses_ambiguous_matches(self):
        diagram = PropController() + PropController() + Integrator()

        with self.assertRaisesRegex(ValueError, "Ambiguous autowire target"):
            diagram.autowire(strict=True)

        self.assertEqual(diagram.connections["controller"]["y"], None)
        self.assertEqual(diagram.connections["controller_2"]["y"], None)
        self.assertEqual(diagram.connections["integrator"]["u"], None)


if __name__ == "__main__":
    unittest.main()
