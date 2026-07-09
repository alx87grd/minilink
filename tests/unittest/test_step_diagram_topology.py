"""Tests for step-diagram topology export."""

import unittest

from minilink.core.backends import array_module
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem
from minilink.graphical.diagrams import build_diagram_topology
from minilink.graphical.diagrams.dot import block_html


class Accumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = array_module().zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


class TestStepDiagramTopology(unittest.TestCase):
    def test_step_leaf_kind(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Accumulator(), "acc")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "acc", "u")
        diagram.connect_new_output_port("acc", "y", "y")

        topology = build_diagram_topology(diagram)
        kinds = {node.id: node.kind for node in topology.nodes}
        self.assertEqual(kinds["acc"], "step_system")

    def test_block_html_step_prefix(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Accumulator(), "acc")
        topology = build_diagram_topology(diagram)
        acc_node = next(node for node in topology.nodes if node.id == "acc")
        html = block_html(acc_node)
        self.assertIn("Step:", html)


if __name__ == "__main__":
    unittest.main()
