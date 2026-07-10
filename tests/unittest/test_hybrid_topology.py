"""Tests for hybrid diagram topology export and rendering."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.backends import array_module
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.core.system import StepSystem
from minilink.graphical.diagrams.hybrid_topology import (
    build_hybrid_topology,
    export_hybrid_topology,
)
from minilink.simulation.computer import Computer, StepSchedule


class DiscreteLowPass(StepSystem):
    def __init__(self, alpha=0.25):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        alpha = params["alpha"]
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def _build_step_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(DiscreteLowPass(alpha=0.3), "filter")
    diagram.add_subsystem(ProportionalController(0.35), "ctl")
    diagram.add_input_port("r")
    diagram.add_input_port("y_meas")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("input", "y_meas", "filter", "u")
    diagram.connect("filter", "y", "ctl", "y")
    diagram.connect_new_output_port("filter", "y", "y_f")
    diagram.connect_new_output_port("ctl", "u", "u_cmd")
    return diagram


def _build_plant():
    plant = DiagramSystem()
    plant.add_subsystem(Integrator(), "plant")
    plant.add_input_port("u")
    plant.connect("input", "u", "plant", "u")
    plant.connect_new_output_port("plant", "y", "y")
    return plant


def _build_hybrid():
    schedule = StepSchedule.from_rates(
        dt_base=0.01,
        rates_hz={"filter": 100.0, "ctl": 10.0},
    )
    computer = Computer(_build_step_diagram(), schedule)
    plant = _build_plant()
    hybrid = HybridDiagram(computer=computer, plant=plant)
    hybrid.connect_boundary(
        direction="computer_to_plant",
        computer_port="u_cmd",
        plant_port="u",
    )
    hybrid.connect_boundary(
        direction="plant_to_computer",
        computer_port="y_meas",
        plant_port="y",
    )
    return hybrid


class TestHybridTopology(unittest.TestCase):
    def test_schedule_label_and_prefixed_ids(self):
        topology = build_hybrid_topology(_build_hybrid())

        self.assertIn("dt_base=0.01", topology.computer.schedule_label)
        self.assertIn("filter@100 Hz", topology.computer.schedule_label)
        self.assertIn("ctl@10 Hz", topology.computer.schedule_label)

        step_ids = {node.id for node in topology.step_diagram.nodes}
        self.assertIn("computer__filter", step_ids)
        self.assertIn("computer__ctl", step_ids)
        self.assertNotIn("computer__input", step_ids)
        self.assertNotIn("computer__output", step_ids)

        plant_ids = {node.id for node in topology.plant.nodes}
        self.assertIn("plant__plant", plant_ids)
        self.assertNotIn("plant__input", plant_ids)
        self.assertNotIn("plant__output", plant_ids)

    def test_boundary_anchors_skip_external_nodes(self):
        topology = build_hybrid_topology(_build_hybrid())

        zoh = next(edge for edge in topology.boundary_edges if edge.label == "ZOH")
        sample = next(
            edge for edge in topology.boundary_edges if edge.label == "sample"
        )

        from minilink.graphical.diagrams.hybrid_topology import (
            resolve_computer_boundary_anchor,
            resolve_plant_boundary_anchor,
        )

        c_out, c_port = resolve_computer_boundary_anchor(
            topology.step_diagram,
            direction="computer_to_plant",
            computer_port=zoh.computer_port,
        )
        self.assertEqual((c_out, c_port), ("computer__ctl", "u"))

        c_in, c_in_port = resolve_computer_boundary_anchor(
            topology.step_diagram,
            direction="plant_to_computer",
            computer_port=sample.computer_port,
        )
        self.assertEqual((c_in, c_in_port), ("computer__filter", "u"))

        p_in, p_in_port = resolve_plant_boundary_anchor(
            topology.plant,
            direction="computer_to_plant",
            plant_port=zoh.plant_port,
        )
        self.assertEqual((p_in, p_in_port), ("plant__plant", "u"))

        p_out, p_out_port = resolve_plant_boundary_anchor(
            topology.plant,
            direction="plant_to_computer",
            plant_port=sample.plant_port,
        )
        self.assertEqual((p_out, p_out_port), ("plant__plant", "y"))

    def test_external_nodes_kept_when_disabled(self):
        topology = build_hybrid_topology(_build_hybrid(), abstract_boundary=False)
        step_ids = {node.id for node in topology.step_diagram.nodes}
        self.assertIn("computer__input", step_ids)
        self.assertIn("computer__output", step_ids)

    def test_boundary_edges(self):
        topology = build_hybrid_topology(_build_hybrid())
        labels = {edge.label for edge in topology.boundary_edges}
        self.assertEqual(labels, {"ZOH", "sample"})

        zoh = next(edge for edge in topology.boundary_edges if edge.label == "ZOH")
        self.assertEqual(zoh.computer_port, "u_cmd")
        self.assertEqual(zoh.plant_port, "u")

        sample = next(
            edge for edge in topology.boundary_edges if edge.label == "sample"
        )
        self.assertEqual(sample.computer_port, "y_meas")
        self.assertEqual(sample.plant_port, "y")

    def test_graphviz_source(self):
        try:
            import graphviz  # noqa: F401
        except ImportError:
            self.skipTest("graphviz package not installed")

        graph = export_hybrid_topology(_build_hybrid(), backend="graphviz")
        source = graph.source

        self.assertIn("cluster_plant", source)
        self.assertIn("cluster_computer", source)
        self.assertIn("cluster_step_diagram", source)
        self.assertIn("computer__filter", source)
        self.assertIn("computer__ctl", source)
        self.assertIn("ZOH", source)
        self.assertIn("sample", source)
        self.assertIn("dt_base=0.01", source)
        self.assertNotIn("computer__input", source)
        self.assertNotIn("computer__output", source)

    def test_graphviz_renders(self):
        try:
            import graphviz  # noqa: F401
        except ImportError:
            self.skipTest("graphviz package not installed")

        import os
        import shutil
        import subprocess
        import tempfile

        if shutil.which("dot") is None:
            self.skipTest("graphviz dot binary not installed")

        graph = export_hybrid_topology(_build_hybrid(), backend="graphviz")
        with tempfile.NamedTemporaryFile(suffix=".gv", delete=False) as tmp:
            path = tmp.name
        try:
            graph.save(path)
            result = subprocess.run(
                ["dot", "-Tsvg", "-O", path],
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr.decode())
        finally:
            os.unlink(path)
            svg_path = path + ".svg"
            if os.path.exists(svg_path):
                os.unlink(svg_path)

    def test_mermaid_source(self):
        source = export_hybrid_topology(_build_hybrid(), backend="mermaid")

        self.assertIn('subgraph Plant["Plant"]', source)
        self.assertIn('subgraph Computer["', source)
        self.assertIn('subgraph StepDiagram["StepDiagram"]', source)
        self.assertIn("plant__plant", source)
        self.assertIn("computer__filter", source)
        self.assertIn("ZOH", source)
        self.assertIn("sample", source)
        self.assertIn("dt_base=0.01", source)


if __name__ == "__main__":
    unittest.main()
