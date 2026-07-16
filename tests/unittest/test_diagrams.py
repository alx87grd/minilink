import unittest

import pytest

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.diagram import DiagramSystem
from minilink.graphical.diagrams import (
    build_diagram_topology,
    export_diagram_topology,
    get_diagram,
    get_system_block_html,
    plot_diagram,
)


class TestDiagrams(unittest.TestCase):
    def test_system_block_html_includes_named_ports(self):
        html = get_system_block_html(ProportionalController(), "ctl")

        self.assertIn("P Controller::ctl", html)
        self.assertIn('PORT="r"', html)
        self.assertIn('PORT="u"', html)

    def test_system_diagram_contains_block_label(self):
        pytest.importorskip("graphviz")

        graph = get_diagram(Integrator())

        self.assertIsNotNone(graph)
        self.assertIn("Integrator", graph.source)
        self.assertIn('PORT="y"', graph.source)

    def test_diagram_graph_contains_subsystems_and_connections(self):
        pytest.importorskip("graphviz")

        diagram = self._make_diagram()

        graph = get_diagram(diagram)

        self.assertIsNotNone(graph)
        self.assertIn("ctl", graph.source)
        self.assertIn("plant", graph.source)
        self.assertIn("input:r:e -> ctl:r:w", graph.source)
        self.assertIn("output:y_meas:w", graph.source)

    def test_plot_diagram_no_display_returns_graph(self):
        pytest.importorskip("graphviz")

        graph = plot_diagram(Integrator(), show_inline=False, show_pdf=False)

        self.assertIsNotNone(graph)
        self.assertIn("Integrator", graph.source)

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

    def test_topology_abstract_boundary_collapses_external_nodes(self):
        topology = build_diagram_topology(
            self._make_diagram(),
            abstract_boundary=True,
        )

        node_ids = [node.id for node in topology.nodes]
        self.assertEqual(node_ids, ["ctl", "plant"])
        edges = [
            (edge.source_node, edge.source_port, edge.target_node, edge.target_port)
            for edge in topology.edges
        ]
        self.assertIn(("ctl", "u", "plant", "u"), edges)
        self.assertNotIn(("input", "r", "ctl", "r"), edges)

        inputs = {
            ref.diagram_port: (ref.node_id, ref.port_id)
            for ref in topology.boundary_inputs
        }
        outputs = {
            ref.diagram_port: (ref.node_id, ref.port_id)
            for ref in topology.boundary_outputs
        }
        self.assertEqual(inputs["r"], ("ctl", "r"))
        self.assertEqual(outputs["y_meas"], ("plant", "y"))

    def test_mermaid_exporter_returns_deterministic_source(self):
        source = export_diagram_topology(self._make_diagram(), backend="mermaid")

        self.assertIn("flowchart LR", source)
        self.assertIn('ctl["P Controller::ctl"]', source)
        self.assertIn('input -- "r -> r" --> ctl', source)
        self.assertIn('plant -- "y -> y_meas" --> output', source)

    @staticmethod
    def _make_diagram():
        diagram = DiagramSystem()
        diagram.connection_verbose = False
        diagram.add_subsystem(ProportionalController(), "ctl")
        diagram.add_subsystem(Integrator(), "plant")
        diagram.add_input_port("r")
        diagram.connect("input", "r", "ctl", "r")
        diagram.connect("plant", "y", "ctl", "y")
        diagram.connect("ctl", "u", "plant", "u")
        diagram.connect_new_output_port("plant", "y", "y_meas")
        return diagram


if __name__ == "__main__":
    unittest.main()


# --- merged from test_wiring_mixin.py ---

"""Parity tests for WiredDiagramMixin wiring and topology export."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.compile.compiler import check_algebraic_loops
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from minilink.core.wiring import WiredDiagramMixin, validate_diagram_params
from minilink.graphical.diagrams import build_diagram_topology


def _build_closed_loop():
    diagram = DiagramSystem()
    diagram.connection_verbose = False
    diagram.add_subsystem(ProportionalController(2.5), "ctl")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")
    return diagram


def _build_feedthrough_loop():
    class FeedthroughSystem(System):
        def __init__(self, name):
            super().__init__()
            self.name = name
            self.add_input_port("u")
            self.add_output_port("y", function=self.h, dependencies=("u",))

        def h(self, x, u, t=0, params=None):
            return u * 2

    diagram = DiagramSystem()
    diagram.connection_verbose = False
    diagram.add_subsystem(FeedthroughSystem("A"), "A")
    diagram.add_subsystem(FeedthroughSystem("B"), "B")
    diagram.add_subsystem(FeedthroughSystem("C"), "C")
    diagram.connect("A", "y", "B", "u")
    diagram.connect("B", "y", "C", "u")
    diagram.connect("C", "y", "A", "u")
    return diagram


class TestWiringMixin(unittest.TestCase):
    def test_diagram_inherits_mixin(self):
        diagram = DiagramSystem()
        self.assertIsInstance(diagram, WiredDiagramMixin)

    def test_topology_parity(self):
        diagram = _build_closed_loop()
        diagram.connect_new_output_port("plant", "y", "y_meas")

        topology = build_diagram_topology(diagram)
        node_ids = [node.id for node in topology.nodes]
        self.assertEqual(node_ids, ["input", "ctl", "plant", "output"])
        edges = [
            (edge.source_node, edge.source_port, edge.target_node, edge.target_port)
            for edge in topology.edges
        ]
        self.assertIn(("input", "r", "ctl", "r"), edges)
        self.assertIn(("ctl", "u", "plant", "u"), edges)
        self.assertIn(("plant", "y", "output", "y_meas"), edges)

    def test_connect_dimension_mismatch_raises(self):
        diagram = DiagramSystem()
        diagram.add_subsystem(ProportionalController(), "ctl")
        diagram.add_input_port("r", dim=2)
        with self.assertRaises(ValueError) as ctx:
            diagram.connect("input", "r", "ctl", "r")
        self.assertIn("dimension mismatch", str(ctx.exception).lower())

    def test_gather_chain_matches_ports(self):
        diagram = _build_closed_loop()
        x = np.array([0.25])
        u = np.array([1.0])
        t = 0.5

        plant_y = diagram.compute_subsys_output_port(x, u, t, "plant", "y")
        ctl_u = diagram.compute_subsys_output_port(x, u, t, "ctl", "u")

        np.testing.assert_allclose(plant_y, np.array([0.25]))
        np.testing.assert_allclose(ctl_u, np.array([2.5 * (1.0 - 0.25)]))

        local_u = diagram.get_local_input(x, u, t, "ctl")
        self.assertEqual(local_u.shape, (2,))

    def test_check_algebraic_loops_valid_order(self):
        diagram = _build_closed_loop()
        order = diagram.check_algebraic_loops()
        self.assertEqual(order, check_algebraic_loops(diagram))
        self.assertGreaterEqual(len(order), 1)

    def test_check_algebraic_loops_detects_cycle(self):
        diagram = _build_feedthrough_loop()
        with self.assertRaises(RuntimeError) as ctx:
            diagram.check_algebraic_loops()
        self.assertIn("Algebraic loop detected", str(ctx.exception))

    def test_state_index_after_multi_subsystem_add(self):
        diagram = _build_closed_loop()
        self.assertEqual(diagram.n, 1)
        self.assertEqual(diagram.state_index["ctl"], (0, 0))
        self.assertEqual(diagram.state_index["plant"], (0, 1))

    def test_validate_diagram_params_unknown_sys_id(self):
        diagram = _build_closed_loop()
        with self.assertRaises(ValueError) as ctx:
            validate_diagram_params({"unknown": {"Kp": 1.0}}, diagram.subsystems)
        self.assertIn("Unknown subsystem ids", str(ctx.exception))

    def test_params_setter_unknown_sys_id(self):
        diagram = _build_closed_loop()
        with self.assertRaises(ValueError):
            diagram.params = {"typo": {"Kp": 1.0}}

    def test_closed_loop_euler_trajectory_matches_compiled_f(self):
        """Reference ``diagram.f`` and compiled evaluator stay aligned over rollout."""
        diagram = _build_closed_loop()
        evaluator = diagram.compile(backend="numpy")
        x0 = np.array([0.0])
        u = np.array([1.0])
        dt = 0.1
        n_steps = 10

        x_ref = x0.copy()
        x_comp = x0.copy()
        for step in range(n_steps):
            t = step * dt
            dx_ref = diagram.f(x_ref, u, t)
            dx_comp = evaluator.f(x_comp, u, t)
            np.testing.assert_allclose(dx_comp, dx_ref, atol=1e-10)
            x_ref = x_ref + dx_ref * dt
            x_comp = x_comp + dx_comp * dt

        np.testing.assert_allclose(x_comp, x_ref, atol=1e-10)


if __name__ == "__main__":
    unittest.main()


# --- merged from test_facades_split.py ---

"""Façade mixin split: MRO surface, sim guards, compile types."""

import unittest

from minilink.blocks.basic import Integrator
from minilink.blocks.routing import Gain
from minilink.core.compile.evaluators.numpy_evaluators import (
    NumpyDiagramEvaluator,
    NumpyDynamicEvaluator,
    NumpyStaticEvaluator,
)
from minilink.core.diagram import DiagramSystem
from minilink.core.facades import (
    DynamicSystemFacades,
    SharedSystemFacades,
    StepSystemFacades,
)
from minilink.core.system import DynamicSystem, StepSystem
from minilink.simulation.simulator import Simulator
from minilink.simulation.static_simulator import StaticSimulator


class _Counter(StepSystem):
    def __init__(self):
        super().__init__(n=1)


def _unity_feedback_diagram():
    from minilink.control.output import ProportionalController

    diagram = DiagramSystem()
    diagram.add_subsystem(ProportionalController(), "ctl")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")
    return diagram


class TestFacadeMixinMRO(unittest.TestCase):
    def test_gain_shared_facades_only(self):
        gain = Gain(K=1.0, dim=1)
        self.assertIsInstance(gain, SharedSystemFacades)
        self.assertNotIsInstance(gain, DynamicSystemFacades)
        self.assertNotIsInstance(gain, StepSystemFacades)
        self.assertTrue(hasattr(gain, "compute_trajectory"))
        self.assertTrue(hasattr(gain, "animate"))
        self.assertFalse(hasattr(type(gain), "plot_phase_plane"))
        self.assertFalse(hasattr(type(gain), "game"))
        self.assertFalse(hasattr(type(gain), "compute_rollout"))

    def test_integrator_dynamic_facades(self):
        plant = Integrator()
        self.assertIsInstance(plant, DynamicSystemFacades)
        self.assertIsInstance(plant, SharedSystemFacades)
        self.assertTrue(hasattr(plant, "plot_phase_plane"))
        self.assertTrue(hasattr(plant, "game"))
        self.assertFalse(hasattr(type(plant), "compute_rollout"))

    def test_step_system_rollout_facade(self):
        counter = _Counter()
        self.assertIsInstance(counter, StepSystemFacades)
        self.assertTrue(hasattr(counter, "compute_rollout"))
        self.assertFalse(hasattr(type(counter), "plot_phase_plane"))

    def test_diagram_is_dynamic_system(self):
        diagram = _unity_feedback_diagram()
        self.assertIsInstance(diagram, DynamicSystem)
        self.assertIsInstance(diagram, DynamicSystemFacades)


class TestSimBoundaries(unittest.TestCase):
    def test_static_leaf_rejects_simulator(self):
        gain = Gain(K=1.0, dim=1)
        with self.assertRaises(TypeError):
            Simulator(gain, t0=0, tf=1, n_steps=3)

    def test_static_simulator_accepts_gain(self):
        gain = Gain(K=1.0, dim=1)
        sim = StaticSimulator(gain, t0=0, tf=1, n_steps=3)
        self.assertIsInstance(sim, StaticSimulator)

    def test_static_simulator_rejects_diagram(self):
        diagram = _unity_feedback_diagram()
        with self.assertRaises(TypeError):
            StaticSimulator(diagram, t0=0, tf=1, n_steps=3)


class TestCompileTypes(unittest.TestCase):
    def test_compile_gain_static_evaluator(self):
        gain = Gain(K=2.0, dim=1)
        self.assertIsInstance(gain.compile(), NumpyStaticEvaluator)

    def test_compile_integrator_dynamic_evaluator(self):
        plant = Integrator()
        self.assertIsInstance(plant.compile(), NumpyDynamicEvaluator)

    def test_compile_diagram_diagram_evaluator(self):
        diagram = _unity_feedback_diagram()
        self.assertIsInstance(diagram.compile(), NumpyDiagramEvaluator)


if __name__ == "__main__":
    unittest.main()
