"""Tests for StepDiagramSystem wiring and interpreted step."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.blocks.routing import Gain
from minilink.core.backends import array_module
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.step_diagram_evaluator import (
    NumpyStepDiagramEvaluator,
)
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem, System


class KDependentPort(System):
    """Static block whose output depends on step index ``k`` in the third slot."""

    def __init__(self):
        super().__init__()
        self.add_input_port("u", dim=1)
        self.add_output_port("y", dim=1, function=self.compute, dependencies=("u",))

    def compute(self, x, u, t=0, params=None):
        xp = array_module(u)
        k = int(t)
        return xp.array([u[0] + float(k)])


class Accumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def _build_gain_plant_loop():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(Gain(1.0, dim=1), "gain")
    diagram.add_subsystem(Accumulator(), "plant")
    diagram.add_input_port("u")
    diagram.connect("input", "u", "gain", "u")
    diagram.connect("gain", "y", "plant", "u")
    return diagram


class TestStepDiagram(unittest.TestCase):
    def test_is_step_system(self):
        diagram = StepDiagramSystem()
        self.assertIsInstance(diagram, StepSystem)

    def test_interpreted_step_matches_manual(self):
        diagram = _build_gain_plant_loop()
        x = np.zeros(1)
        u_boundary = np.array([1.0])
        x_manual = 0.0
        for k in range(5):
            u_eff = 1.0 * u_boundary[0]
            x_manual = x_manual + u_eff
            x = diagram.step(x, u_boundary, k=k)
        self.assertAlmostEqual(float(x[0]), x_manual)

    def test_k_passed_to_static_port(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(KDependentPort(), "blk")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "blk", "u")
        diagram.connect_new_output_port("blk", "y", "y")

        y = diagram.compute_subsys_output_port(
            np.array([]), np.array([2.0]), 3, "blk", "y"
        )
        self.assertAlmostEqual(float(y[0]), 5.0)

    def test_reject_dynamic_subsystem_at_compile(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Integrator(), "plant")
        with self.assertRaises(TypeError):
            diagram.compile()

    def test_compile_returns_step_diagram_evaluator(self):
        diagram = _build_gain_plant_loop()
        ev = compile(diagram)
        self.assertIsInstance(ev, NumpyStepDiagramEvaluator)

    def test_compiled_step_matches_interpreted(self):
        diagram = _build_gain_plant_loop()
        ev = diagram.compile()
        x = np.zeros(1)
        u = np.array([0.5])
        for k in range(8):
            x_ref = diagram.step(x, u, k=k)
            x_ev = ev.step(x, u, k=k)
            np.testing.assert_allclose(x_ev, x_ref)
            x = x_ev
