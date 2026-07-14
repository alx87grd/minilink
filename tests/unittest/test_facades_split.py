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
