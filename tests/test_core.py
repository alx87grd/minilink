import unittest
import numpy as np
from minilink.core.framework import (
    VectorSignal,
    InputPort,
    OutputPort,
    System,
    StaticSystem,
    DynamicSystem,
)


class TestCoreComponents(unittest.TestCase):

    def test_vector_signal(self):
        v = VectorSignal(2, "test", nominal_value=[1.0, 2.0])
        self.assertEqual(v.dim, 2)
        np.testing.assert_array_equal(v.nominal_value, np.array([1.0, 2.0]))

    def test_system_initialization(self):
        sys = System(n=2, m=1, p=3)
        self.assertEqual(sys.n, 2)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.p, 3)
        self.assertIn("u", sys.inputs)
        self.assertIn("y", sys.outputs)
        # Verify default dependencies for base System
        self.assertEqual(sys.outputs["y"].dependencies, "all")

    def test_static_system(self):
        sys = StaticSystem(m=2, p=2)
        self.assertEqual(sys.n, 0)
        self.assertEqual(sys.outputs["y"].dependencies, "all")

    def test_dynamic_system(self):
        sys = DynamicSystem(n=2, m=1, p=1)
        self.assertEqual(sys.n, 2)
        # Verify y has no dependencies by default
        self.assertEqual(sys.outputs["y"].dependencies, ())
        self.assertIn("x", sys.outputs)


if __name__ == "__main__":
    unittest.main()
