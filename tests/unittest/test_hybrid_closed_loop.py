"""Tests for :func:`~minilink.core.hybrid_composition.hybrid_closed_loop`."""

import unittest

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.simulation.computer import Computer, StepSchedule


def _build_step_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(ProportionalController(0.5), "ctl")
    diagram.add_input_port("r")
    diagram.add_input_port("y")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("input", "y", "ctl", "y")
    diagram.connect_new_output_port("ctl", "u", "u")
    return diagram


def _build_plant():
    plant = DiagramSystem()
    plant.add_subsystem(Integrator(), "plant")
    plant.add_input_port("u")
    plant.connect("input", "u", "plant", "u")
    plant.connect_new_output_port("plant", "y", "y")
    return plant


class TestHybridClosedLoop(unittest.TestCase):
    def test_shortcut_matches_manual_wiring(self):
        schedule = StepSchedule(dt_base=0.01)
        shortcut = hybrid_closed_loop(
            _build_step_diagram(), _build_plant(), schedule=schedule
        )

        manual = HybridDiagram(
            computer=Computer(_build_step_diagram(), schedule),
            plant=_build_plant(),
        )
        manual.connect_boundary(
            direction="computer_to_plant",
            computer_port="u",
            plant_port="u",
        )
        manual.connect_boundary(
            direction="plant_to_computer",
            computer_port="y",
            plant_port="y",
        )

        self.assertEqual(len(shortcut.connections), len(manual.connections))
        self.assertEqual(shortcut.connections[0], manual.connections[0])
        self.assertEqual(shortcut.connections[1], manual.connections[1])

    def test_computer_matmul_leaf_plant(self):
        computer = Computer(_build_step_diagram(), StepSchedule(dt_base=0.01))
        hybrid = computer @ Integrator()
        self.assertEqual(len(hybrid.connections), 2)
        self.assertEqual(hybrid.connections[0].computer_port, "u")
        self.assertEqual(hybrid.connections[1].computer_port, "y")

    def test_computer_matmul_plant(self):
        computer = Computer(_build_step_diagram(), StepSchedule(dt_base=0.01))
        hybrid = computer @ _build_plant()
        self.assertEqual(len(hybrid.connections), 2)

    def test_leaf_system_wrapping(self):
        hybrid = hybrid_closed_loop(
            ProportionalController(0.3),
            Integrator(),
            schedule=0.01,
        )
        self.assertIn("ctl", hybrid.computer.diagram.subsystems)
        self.assertIn("plant", hybrid.plant.subsystems)


if __name__ == "__main__":
    unittest.main()
