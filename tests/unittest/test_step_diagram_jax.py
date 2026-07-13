"""JAX compile smoke for step diagrams."""

import unittest

import numpy as np
import pytest

from minilink.blocks.routing import Gain
from minilink.core.backends import array_module
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.jax_evaluators import (
    JaxStepDiagramEvaluator,
)
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem

try:
    import jax  # noqa: F401

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class Accumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1)


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX not installed")
class TestStepDiagramJax(unittest.TestCase):
    def test_jax_step_diagram_rollout(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Gain(1.0, dim=1), "gain")
        diagram.add_subsystem(Accumulator(), "plant")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "gain", "u")
        diagram.connect("gain", "y", "plant", "u")

        ev = compile(diagram, backend="jax")
        self.assertIsInstance(ev, JaxStepDiagramEvaluator)

        rollout = ev.rollout(diagram.x0, n_steps=5, u=np.array([1.0]))
        np_ev = compile(diagram, backend="numpy")
        ref = np_ev.rollout(diagram.x0, n_steps=5, u=np.array([1.0]))
        np.testing.assert_allclose(rollout.x, ref.x, rtol=1e-10, atol=1e-10)
