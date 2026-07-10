"""Tests for :class:`~minilink.core.hybrid_diagram.HybridDiagram` boundary wiring."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_diagram import BoundaryConnection, HybridDiagram
from minilink.simulation.computer import Computer, StepSchedule


def _build_computer_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(ProportionalController(0.5), "ctl")
    diagram.add_input_port("r")
    diagram.add_input_port("y")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("input", "y", "ctl", "y")
    diagram.connect_new_output_port("ctl", "u", "u")
    return diagram


def _build_plant_diagram():
    diagram = DiagramSystem()
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("u")
    diagram.connect("input", "u", "plant", "u")
    diagram.connect_new_output_port("plant", "y", "y")
    return diagram


class TestHybridBoundaryConnect(unittest.TestCase):
    def test_valid_multi_channel_connect(self):
        computer = Computer(_build_computer_diagram(), StepSchedule(dt_base=0.01))
        plant = _build_plant_diagram()
        hybrid = HybridDiagram(computer=computer, plant=plant)
        hybrid.connect_boundary(
            direction="computer_to_plant",
            computer_port="u",
            plant_port="u",
        )
        hybrid.connect_boundary(
            direction="plant_to_computer",
            computer_port="y",
            plant_port="y",
        )
        self.assertEqual(len(hybrid.connections), 2)

    def test_dimension_mismatch_raises(self):
        step_diagram = _build_computer_diagram()

        def wide_output(x, u, t=0, params=None):
            return np.array([0.0, 0.0])

        step_diagram.add_output_port(
            "u2",
            dim=2,
            function=wide_output,
            dependencies=(),
        )
        computer = Computer(step_diagram, StepSchedule(dt_base=0.01))
        hybrid = HybridDiagram(computer=computer, plant=_build_plant_diagram())
        with self.assertRaises(ValueError):
            hybrid.connect_boundary(
                direction="computer_to_plant",
                computer_port="u2",
                plant_port="u",
            )

    def test_unknown_port_raises(self):
        computer = Computer(_build_computer_diagram(), StepSchedule(dt_base=0.01))
        hybrid = HybridDiagram(computer=computer, plant=_build_plant_diagram())
        with self.assertRaises(ValueError):
            hybrid.connect_boundary(
                direction="computer_to_plant",
                computer_port="missing",
                plant_port="u",
            )

    def test_legacy_direction_alias(self):
        conn = BoundaryConnection(
            direction="step_to_plant",
            computer_port="u",
            plant_port="u",
        )
        self.assertEqual(conn.direction, "computer_to_plant")

    def test_from_diagrams(self):
        hybrid = HybridDiagram.from_diagrams(
            _build_computer_diagram(),
            _build_plant_diagram(),
            schedule=0.01,
        )
        self.assertIsNotNone(hybrid.computer)
        self.assertEqual(hybrid.plant.n, 1)


if __name__ == "__main__":
    unittest.main()
