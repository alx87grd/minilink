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
            direction="computer_to_plant", computer_port="u", plant_port="u"
        )
        hybrid.connect_boundary(
            direction="plant_to_computer", computer_port="y", plant_port="y"
        )
        self.assertEqual(len(hybrid.connections), 2)

    def test_dimension_mismatch_raises(self):
        step_diagram = _build_computer_diagram()

        def wide_output(x, u, t=0, params=None):
            return np.array([0.0, 0.0])

        step_diagram.add_output_port("u2", dim=2, function=wide_output, dependencies=())
        computer = Computer(step_diagram, StepSchedule(dt_base=0.01))
        hybrid = HybridDiagram(computer=computer, plant=_build_plant_diagram())
        with self.assertRaises(ValueError):
            hybrid.connect_boundary(
                direction="computer_to_plant", computer_port="u2", plant_port="u"
            )

    def test_unknown_port_raises(self):
        computer = Computer(_build_computer_diagram(), StepSchedule(dt_base=0.01))
        hybrid = HybridDiagram(computer=computer, plant=_build_plant_diagram())
        with self.assertRaises(ValueError):
            hybrid.connect_boundary(
                direction="computer_to_plant", computer_port="missing", plant_port="u"
            )

    def test_legacy_direction_alias(self):
        conn = BoundaryConnection(
            direction="step_to_plant", computer_port="u", plant_port="u"
        )
        self.assertEqual(conn.direction, "computer_to_plant")

    def test_from_diagrams(self):
        hybrid = HybridDiagram.from_diagrams(
            _build_computer_diagram(), _build_plant_diagram(), schedule=0.01
        )
        self.assertIsNotNone(hybrid.computer)
        self.assertEqual(hybrid.plant.n, 1)


from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.core.hybrid_diagram import HybridDiagram


class TestHybridClosedLoop(unittest.TestCase):
    def test_shortcut_matches_manual_wiring(self):
        schedule = StepSchedule(dt_base=0.01)
        shortcut = hybrid_closed_loop(
            _build_computer_diagram(), _build_plant_diagram(), schedule=schedule
        )
        manual = HybridDiagram(
            computer=Computer(_build_computer_diagram(), schedule),
            plant=_build_plant_diagram(),
        )
        manual.connect_boundary(
            direction="computer_to_plant", computer_port="u", plant_port="u"
        )
        manual.connect_boundary(
            direction="plant_to_computer", computer_port="y", plant_port="y"
        )
        self.assertEqual(len(shortcut.connections), len(manual.connections))
        self.assertEqual(shortcut.connections[0], manual.connections[0])
        self.assertEqual(shortcut.connections[1], manual.connections[1])

    def test_computer_matmul_leaf_plant(self):
        computer = Computer(_build_computer_diagram(), StepSchedule(dt_base=0.01))
        hybrid = computer @ Integrator()
        self.assertEqual(len(hybrid.connections), 2)
        self.assertEqual(hybrid.connections[0].computer_port, "u")
        self.assertEqual(hybrid.connections[1].computer_port, "y")

    def test_computer_matmul_plant(self):
        computer = Computer(_build_computer_diagram(), StepSchedule(dt_base=0.01))
        hybrid = computer @ _build_plant_diagram()
        self.assertEqual(len(hybrid.connections), 2)

    def test_leaf_system_wrapping(self):
        hybrid = hybrid_closed_loop(
            ProportionalController(0.3), Integrator(), schedule=0.01
        )
        self.assertIn("ctl", hybrid.computer.diagram.subsystems)
        self.assertIn("plant", hybrid.plant.subsystems)


from minilink.core.step_rollout import StepRollout
from minilink.core.trajectory import Trajectory
from minilink.simulation.hybrid_simulator import HybridSimulator


def _build_hybrid():
    step_diagram = StepDiagramSystem()
    step_diagram.add_subsystem(ProportionalController(0.5), "ctl")
    step_diagram.add_input_port("r")
    step_diagram.add_input_port("y")
    step_diagram.connect("input", "r", "ctl", "r")
    step_diagram.connect("input", "y", "ctl", "y")
    step_diagram.connect_new_output_port("ctl", "u", "u_cmd")
    plant = DiagramSystem()
    plant.add_subsystem(Integrator(), "plant")
    plant.add_input_port("u")
    plant.connect("input", "u", "plant", "u")
    plant.connect_new_output_port("plant", "y", "y")
    return hybrid_closed_loop(step_diagram, plant, schedule=0.01, computer_out="u_cmd")


class TestHybridSimulator(unittest.TestCase):
    def test_records_both_states(self):
        hybrid = _build_hybrid()
        sim = HybridSimulator(hybrid, t0=0, tf=0.2)
        result = sim.solve_forced(np.array([1.0]), input_port_id="r")
        self.assertEqual(result.computer.x.shape[0], hybrid.computer.diagram.n)
        self.assertEqual(result.plant.x.shape[0], hybrid.plant.n)
        self.assertEqual(result.computer.n_samples, sim.n_ticks)
        self.assertGreaterEqual(result.plant.n_samples, sim.n_ticks)
        self.assertIsInstance(result.computer, StepRollout)
        self.assertIsInstance(result.plant, Trajectory)
        self.assertIn("u_cmd", result.computer.signals)

    def test_one_tick_measurement_delay(self):
        hybrid = _build_hybrid()
        sim = HybridSimulator(hybrid, t0=0, tf=0.05, n_steps=5)
        result = sim.solve_forced(np.array([1.0]), input_port_id="r")
        self.assertAlmostEqual(float(result.computer.signals["u_cmd"][0, 0]), 0.5)

    def test_plot_smoke(self):
        import matplotlib

        matplotlib.use("Agg")
        hybrid = _build_hybrid()
        hybrid.compute_forced(np.array([1.0]), t0=0, tf=0.1, input_port_id="r")
        hybrid.plot_trajectory(signals=("r", "y", "u_cmd", "x"), show=False)

    def test_plot_smoke_default_computer_out_u(self):
        """``hybrid_closed_loop`` default ``computer_out='u'`` must not clash with Trajectory.u."""
        import matplotlib

        matplotlib.use("Agg")
        from minilink.control.modelbased import SlidingModeController
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        plant = Pendulum(length=1.0, mass=1.0)
        smc = SlidingModeController(plant)
        hybrid = hybrid_closed_loop(
            smc, plant, schedule=0.02, computer_in="y", plant_out="y"
        )
        hybrid.compute_forced(
            np.array([0.0, 0.0]), t0=0.0, tf=0.1, input_port_id="r", verbose=False
        )
        hybrid.plot_trajectory(show=False)

    def test_plot_defaults_states_and_inputs_only(self):
        from minilink.control.modelbased import SlidingModeController
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
        from minilink.graphical.signals.time_signals import (
            build_signal_plot_spec,
            resolve_plot_signals,
        )

        plant = Pendulum(length=1.0, mass=1.0)
        smc = SlidingModeController(plant)
        hybrid = hybrid_closed_loop(
            smc, plant, schedule=0.02, computer_in="y", plant_out="y"
        )
        result = hybrid.compute_forced(
            np.array([0.0, 0.0]), t0=0.0, tf=0.1, input_port_id="r", verbose=False
        )
        self.assertIn("y", result.plant.signals)
        signals = resolve_plot_signals(hybrid.plant)
        if result.plant.u.shape[0]:
            signals = tuple(dict.fromkeys((*signals, "u")))
        spec = build_signal_plot_spec(hybrid.plant, result.plant, signals=signals)
        plotted = {trace.signal for trace in spec.traces}
        self.assertIn("x", plotted)
        self.assertIn("u", plotted)
        self.assertNotIn("y", plotted)
        self.assertNotIn("r", plotted)

    def test_compute_forced_caches_traj(self):
        hybrid = _build_hybrid()
        self.assertIsNone(hybrid.traj)
        self.assertIsNone(hybrid.last_result)
        result = hybrid.compute_forced(
            np.array([1.0]), t0=0, tf=0.05, input_port_id="r", verbose=False
        )
        self.assertIs(hybrid.last_result, result)
        self.assertIs(hybrid.traj, result.plant)
        self.assertIs(hybrid.rollout, result.computer)

    def test_animate_uses_plant_traj(self):
        hybrid = _build_hybrid()
        hybrid.compute_forced(
            np.array([1.0]), t0=0, tf=0.05, input_port_id="r", verbose=False
        )
        hybrid.animate(show=False)

    def test_plant_diagram_inherits_camera_scale(self):
        from minilink.core.hybrid_composition import _as_plant_diagram
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        plant = Pendulum(length=1.0, mass=1.0)
        plant_diagram = _as_plant_diagram(plant)
        self.assertEqual(plant_diagram.camera_scale, plant.camera_scale)

    def test_hybrid_animate_refreshes_plant_camera_scale(self):
        from minilink.blocks.basic import Integrator
        from minilink.control.output import ProportionalController
        from minilink.core.hybrid_composition import hybrid_closed_loop

        plant = Integrator()
        plant.camera_scale = 3.0
        hybrid = hybrid_closed_loop(ProportionalController(0.5), plant, schedule=0.01)
        self.assertEqual(hybrid.plant.camera_scale, 3.0)
        plant.camera_scale = 9.0
        hybrid.animate(show=False)
        self.assertEqual(hybrid.plant.camera_scale, 9.0)


from minilink.core.backends import array_module
from minilink.core.system import StepSystem
from minilink.graphical.diagrams.hybrid_topology import (
    build_hybrid_topology,
    export_hybrid_topology,
)


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


def _build_hybrid_hybrid_topology():
    schedule = StepSchedule.from_rates(
        dt_base=0.01, rates_hz={"filter": 100.0, "ctl": 10.0}
    )
    computer = Computer(_build_step_diagram(), schedule)
    plant = _build_plant_diagram()
    hybrid = HybridDiagram(computer=computer, plant=plant)
    hybrid.connect_boundary(
        direction="computer_to_plant", computer_port="u_cmd", plant_port="u"
    )
    hybrid.connect_boundary(
        direction="plant_to_computer", computer_port="y_meas", plant_port="y"
    )
    return hybrid


class TestHybridTopology(unittest.TestCase):
    def test_schedule_label_and_prefixed_ids(self):
        topology = build_hybrid_topology(_build_hybrid_hybrid_topology())
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
        topology = build_hybrid_topology(_build_hybrid_hybrid_topology())
        zoh = next((edge for edge in topology.boundary_edges if edge.label == "ZOH"))
        sample = next(
            (edge for edge in topology.boundary_edges if edge.label == "sample")
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
            topology.plant, direction="computer_to_plant", plant_port=zoh.plant_port
        )
        self.assertEqual((p_in, p_in_port), ("plant__plant", "u"))
        p_out, p_out_port = resolve_plant_boundary_anchor(
            topology.plant, direction="plant_to_computer", plant_port=sample.plant_port
        )
        self.assertEqual((p_out, p_out_port), ("plant__plant", "y"))

    def test_external_nodes_kept_when_disabled(self):
        topology = build_hybrid_topology(
            _build_hybrid_hybrid_topology(), abstract_boundary=False
        )
        step_ids = {node.id for node in topology.step_diagram.nodes}
        self.assertIn("computer__input", step_ids)
        self.assertIn("computer__output", step_ids)

    def test_boundary_edges(self):
        topology = build_hybrid_topology(_build_hybrid_hybrid_topology())
        labels = {edge.label for edge in topology.boundary_edges}
        self.assertEqual(labels, {"ZOH", "sample"})
        zoh = next((edge for edge in topology.boundary_edges if edge.label == "ZOH"))
        self.assertEqual(zoh.computer_port, "u_cmd")
        self.assertEqual(zoh.plant_port, "u")
        sample = next(
            (edge for edge in topology.boundary_edges if edge.label == "sample")
        )
        self.assertEqual(sample.computer_port, "y_meas")
        self.assertEqual(sample.plant_port, "y")

    def test_graphviz_source(self):
        try:
            import graphviz
        except ImportError:
            self.skipTest("graphviz package not installed")
        graph = export_hybrid_topology(
            _build_hybrid_hybrid_topology(), backend="graphviz"
        )
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
            import graphviz
        except ImportError:
            self.skipTest("graphviz package not installed")
        import os
        import shutil
        import subprocess
        import tempfile

        if shutil.which("dot") is None:
            self.skipTest("graphviz dot binary not installed")
        graph = export_hybrid_topology(
            _build_hybrid_hybrid_topology(), backend="graphviz"
        )
        with tempfile.NamedTemporaryFile(suffix=".gv", delete=False) as tmp:
            path = tmp.name
        try:
            graph.save(path)
            result = subprocess.run(
                ["dot", "-Tsvg", "-O", path], capture_output=True, check=False
            )
            self.assertEqual(result.returncode, 0, result.stderr.decode())
        finally:
            os.unlink(path)
            svg_path = path + ".svg"
            if os.path.exists(svg_path):
                os.unlink(svg_path)

    def test_mermaid_source(self):
        source = export_hybrid_topology(
            _build_hybrid_hybrid_topology(), backend="mermaid"
        )
        self.assertIn('subgraph Plant["Plant"]', source)
        self.assertIn('subgraph Computer["', source)
        self.assertIn('subgraph StepDiagram["StepDiagram"]', source)
        self.assertIn("plant__plant", source)
        self.assertIn("computer__filter", source)
        self.assertIn("ZOH", source)
        self.assertIn("sample", source)
        self.assertIn("dt_base=0.01", source)


def _reference(k):
    return np.array([1.0 if k < 60 else 0.0])


class TestHybridMultiRate(unittest.TestCase):
    def test_scheduled_hybrid_runs(self):
        hybrid = _build_hybrid_hybrid_topology()
        sim = HybridSimulator(hybrid, t0=0, tf=1.6)
        result = sim.solve_forced(_reference, input_port_id="r")
        self.assertEqual(result.computer.n_samples, 160)
        self.assertIn("y_f", result.computer.signals)
        self.assertIn("u_cmd", result.computer.signals)
        self.assertTrue(np.all(np.isfinite(result.plant.x)))

    def test_hand_loop_coarse_parity(self):
        hybrid = _build_hybrid_hybrid_topology()
        dt_base = hybrid.computer.schedule.dt_base
        plant_eval = hybrid.plant.compile()
        computer = hybrid.computer
        computer.compile()
        computer.reset()
        x_plant = np.zeros(1)
        u_nom = hybrid.plant.get_u_from_input_ports()
        sample = plant_eval.outputs(x_plant, u_nom, 0.0)
        y_sample = float(sample["y"][0])
        n_steps = 20
        y_hand = []
        for _ in range(n_steps):
            k = computer.k
            u_computer = hybrid.computer.diagram.get_u_from_input_ports().copy()
            u_computer[hybrid.computer.diagram.get_input_port_slice("r")] = _reference(
                k
            )
            u_computer[hybrid.computer.diagram.get_input_port_slice("y_meas")] = (
                y_sample
            )
            outs = computer.tick(u_computer)
            u_plant = np.array([float(outs["u_cmd"][0])])
            x_plant = plant_eval.integrate_zoh(x_plant, u_plant, k * dt_base, dt_base)
            y_sample = float(
                plant_eval.outputs(x_plant, u_plant, (k + 1) * dt_base)["y"][0]
            )
            y_hand.append(y_sample)
        sim = HybridSimulator(hybrid, t0=0, n_steps=n_steps)
        result = sim.solve_forced(_reference, input_port_id="r")
        np.testing.assert_allclose(
            result.computer.signals["y_meas"][0, :n_steps], y_hand, rtol=1e-06
        )


from minilink.control.modelbased import SlidingModeController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestHybridFineRecording(unittest.TestCase):
    def test_plant_records_finer_than_computer(self):
        plant = Pendulum(length=1.0, mass=1.0)
        plant.x0 = np.array([0.5, 0.0])
        ctl = SlidingModeController(plant, lam=2.0, gain=4.0, nab=0.1)
        hybrid = hybrid_closed_loop(
            ctl, plant, schedule=0.1, computer_in="y", plant_out="y"
        )
        sim = HybridSimulator(hybrid, t0=0.0, tf=0.5, plant_dt_inner=0.01)
        result = sim.solve_forced(np.array([0.0, 0.0]), input_port_id="r")
        self.assertIsInstance(result.computer, StepRollout)
        self.assertIsInstance(result.plant, Trajectory)
        self.assertGreater(result.plant.n_samples, result.computer.n_samples)
        self.assertEqual(result.computer.n_samples, sim.n_ticks)
        dt_median = np.median(np.diff(result.plant.t))
        self.assertAlmostEqual(dt_median, 0.01, places=5)
        tick_end = hybrid.computer.schedule.dt_base * sim.n_ticks
        np.testing.assert_allclose(result.plant.t[0], 0.0)
        np.testing.assert_allclose(result.plant.t[-1], tick_end, rtol=1e-09)


def _build_hybrid_smc_hybrid(*, ts=0.05):
    plant = Pendulum(length=1.0, mass=1.0)
    plant.x0 = np.array([2.5, -0.2])
    ctl = SlidingModeController(plant, lam=2.0, gain=6.0, nab=0.12)
    return hybrid_closed_loop(ctl, plant, schedule=ts, computer_in="y", plant_out="y")


class TestSmcHybrid(unittest.TestCase):
    def test_hand_loop_matches_hybrid_simulator(self):
        hybrid = _build_hybrid_smc_hybrid(ts=0.05)
        dt_base = hybrid.computer.schedule.dt_base
        plant_eval = hybrid.plant.compile()
        computer = hybrid.computer
        computer.compile()
        computer.reset()
        diagram = hybrid.computer.diagram
        slice_r = diagram.get_input_port_slice("r")
        slice_y = diagram.get_input_port_slice("y")
        ref = np.array([0.0, 0.0])
        x_plant = hybrid.plant.x0.copy()
        u_nom = hybrid.plant.get_u_from_input_ports()
        y_sample = plant_eval.outputs(x_plant, u_nom, 0.0)["y"]
        n_steps = 12
        q_hand = []
        for _ in range(n_steps):
            k = computer.k
            u_computer = diagram.get_u_from_input_ports().copy()
            u_computer[slice_r] = ref
            u_computer[slice_y] = y_sample
            outs = computer.tick(u_computer)
            u_plant = np.asarray(outs["u"], dtype=float).reshape(-1)
            x_plant = plant_eval.integrate_zoh(x_plant, u_plant, k * dt_base, dt_base)
            y_sample = plant_eval.outputs(x_plant, u_plant, (k + 1) * dt_base)["y"]
            q_hand.append(float(y_sample[0]))
        sim = HybridSimulator(hybrid, t0=0.0, n_steps=n_steps, plant_dt_inner=0.002)
        result = sim.solve_forced(ref, input_port_id="r")
        np.testing.assert_allclose(
            result.computer.signals["y"][0, :n_steps], q_hand, rtol=1e-05, atol=1e-05
        )
