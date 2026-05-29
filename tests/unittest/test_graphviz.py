import unittest

import pytest

from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.diagram import DiagramSystem
from minilink.graphical.graphe import (
    get_diagram_graphe,
    get_system_block_html,
    get_system_graphe,
    plot_graphviz,
)
from minilink.graphical.diagram_export import export_diagram_topology
from minilink.graphical.topology import build_diagram_topology


class TestGraphviz(unittest.TestCase):
    def test_system_block_html_includes_named_ports(self):
        html = get_system_block_html(PropController(), "ctl")

        self.assertIn("Controller::ctl", html)
        self.assertIn('PORT="r"', html)
        self.assertIn('PORT="u"', html)

    def test_system_graphe_contains_block_label(self):
        pytest.importorskip("graphviz")

        graph = get_system_graphe(Integrator())

        self.assertIsNotNone(graph)
        self.assertIn("Integrator", graph.source)
        self.assertIn('PORT="y"', graph.source)

    def test_diagram_graphe_contains_subsystems_and_connections(self):
        pytest.importorskip("graphviz")

        diagram = self._make_diagram()

        graph = get_diagram_graphe(diagram)

        self.assertIsNotNone(graph)
        self.assertIn("ctl", graph.source)
        self.assertIn("plant", graph.source)
        self.assertIn("input:r:e -> ctl:r:w", graph.source)
        self.assertIn("output:y_meas:w", graph.source)

    def test_plot_graphviz_accepts_none_graph(self):
        self.assertIsNone(plot_graphviz(None, show_inline=False, show_pdf=False))

    def test_topology_builder_contains_ports_and_edges(self):
        topology = build_diagram_topology(self._make_diagram())

        node_ids = [node.id for node in topology.nodes]
        self.assertEqual(node_ids, ["input", "ctl", "plant", "output"])
        edges = [
            (edge.source_node, edge.source_port, edge.target_node, edge.target_port)
            for edge in topology.edges
        ]
        self.assertIn(("input", "r", "ctl", "r"), edges)
        self.assertIn(("ctl", "u", "plant", "u"), edges)
        self.assertIn(("plant", "y", "output", "y_meas"), edges)

    def test_mermaid_exporter_returns_deterministic_source(self):
        source = export_diagram_topology(self._make_diagram(), backend="mermaid")

        self.assertIn("flowchart LR", source)
        self.assertIn('ctl["Controller::ctl"]', source)
        self.assertIn('input -- "r -> r" --> ctl', source)
        self.assertIn('plant -- "y -> y_meas" --> output', source)

    @staticmethod
    def _make_diagram():
        diagram = DiagramSystem()
        diagram.graphe_building_verbose = False
        diagram.add_subsystem(PropController(), "ctl")
        diagram.add_subsystem(Integrator(), "plant")
        diagram.add_input_port("r")
        diagram.connect("input", "r", "ctl", "r")
        diagram.connect("plant", "y", "ctl", "y")
        diagram.connect("ctl", "u", "plant", "u")
        diagram.connect_new_output_port("plant", "y", "y_meas")
        return diagram


if __name__ == "__main__":
    unittest.main()
