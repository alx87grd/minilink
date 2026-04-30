import unittest

from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.diagram import DiagramSystem
from minilink.graphical.graphe import (
    get_diagram_graphe,
    get_system_block_html,
    get_system_graphe,
    plot_graphviz,
)


class TestGraphviz(unittest.TestCase):
    def test_system_block_html_includes_named_ports(self):
        html = get_system_block_html(PropController(), "ctl")

        self.assertIn("Controller::ctl", html)
        self.assertIn('PORT="ref"', html)
        self.assertIn('PORT="u"', html)

    def test_system_graphe_contains_block_label(self):
        graph = get_system_graphe(Integrator())

        self.assertIsNotNone(graph)
        self.assertIn("Integrator", graph.source)
        self.assertIn('PORT="y"', graph.source)

    def test_diagram_graphe_contains_subsystems_and_connections(self):
        diagram = DiagramSystem()
        diagram.graphe_building_verbose = False
        diagram.add_subsystem(PropController(), "ctl")
        diagram.add_subsystem(Integrator(), "plant")
        diagram.add_input_port(1, "ref")
        diagram.connect("input", "ref", "ctl", "ref")
        diagram.connect("plant", "y", "ctl", "y")
        diagram.connect("ctl", "u", "plant", "u")
        diagram.connect_new_output_port("plant", "y", "y_meas")

        graph = get_diagram_graphe(diagram)

        self.assertIsNotNone(graph)
        self.assertIn("ctl", graph.source)
        self.assertIn("plant", graph.source)
        self.assertIn("input:ref:e -> ctl:ref:w", graph.source)
        self.assertIn("output:y_meas:w", graph.source)

    def test_plot_graphviz_accepts_none_graph(self):
        self.assertIsNone(plot_graphviz(None, show_inline=False, show_pdf=False))


if __name__ == "__main__":
    unittest.main()
