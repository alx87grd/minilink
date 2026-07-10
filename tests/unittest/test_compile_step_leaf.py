"""compile() dispatch for StepSystem leaves."""

import unittest

import numpy as np

from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.step_evaluator import NumpyStepEvaluator
from minilink.core.system import StepSystem


class LinearStep(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1)
        self.params = {"a": 0.5}
        self.x0 = np.array([1.0])

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        return np.array([params["a"] * x[0] + u[0]])


class TestCompileStepLeaf(unittest.TestCase):
    def test_compile_returns_numpy_step_evaluator(self):
        ev = compile(LinearStep())
        self.assertIsInstance(ev, NumpyStepEvaluator)

    def test_frozen_params_step_parity(self):
        plant = LinearStep()
        ev = compile(plant)
        x = np.array([2.0])
        u = np.array([1.0])
        np.testing.assert_allclose(ev.step(x, u, 0), plant.step(x, u, 0))
        np.testing.assert_allclose(
            ev.step_p(x, u, 0, {"a": 0.25}), plant.step(x, u, 0, {"a": 0.25})
        )


if __name__ == "__main__":
    unittest.main()
