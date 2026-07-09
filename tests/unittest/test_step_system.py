"""StepSystem leaf contract tests."""

import unittest

import numpy as np

from minilink.blocks.step import ZOHHold
from minilink.core.system import StepSystem


class Accumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        return np.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        return np.array([x[0]])


class TestStepSystem(unittest.TestCase):
    def test_step_returns_next_state(self):
        plant = Accumulator()
        x1 = plant.step(np.array([1.0]), np.array([2.0]), k=3)
        np.testing.assert_allclose(x1, [3.0])

    def test_h_uses_k_slot(self):
        plant = Accumulator()
        y = plant.h(np.array([4.0]), np.array([0.0]), k=5)
        np.testing.assert_allclose(y, [4.0])

    def test_no_f_on_stepsystem(self):
        from minilink.core.compile.compiler import compile

        plant = Accumulator()
        self.assertFalse(hasattr(plant, "f"))
        ev = compile(plant)
        self.assertFalse(hasattr(ev, "f"))

    def test_rollout_attr_initialized(self):
        plant = Accumulator()
        self.assertIsNone(plant.rollout)

    def test_zoh_hold_latches_command(self):
        hold = ZOHHold()
        x1 = hold.step(np.array([0.0]), np.array([2.5]), k=0)
        np.testing.assert_allclose(x1, [2.5])
        np.testing.assert_allclose(hold.h(np.array([1.0]), np.array([0.0]), k=1), [1.0])

    def test_n_must_be_positive(self):
        with self.assertRaises(ValueError):
            StepSystem(0)


if __name__ == "__main__":
    unittest.main()
