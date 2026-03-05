import unittest
import numpy as np
from minilink.blocks.sources import Source, Step, WhiteNoise


class TestBlocks(unittest.TestCase):

    def test_source(self):
        s = Source(p=2)
        s.params["value"] = np.array([3.0, 4.0])
        y = s.h(x=[], u=[], t=0)
        np.testing.assert_array_equal(y, np.array([3.0, 4.0]))

    def test_step_source(self):
        step = Step(
            initial_value=np.array([0.0]), final_value=np.array([1.0]), step_time=2.0
        )
        y_before = step.h(x=[], u=[], t=1.0)
        y_after = step.h(x=[], u=[], t=3.0)
        np.testing.assert_array_equal(y_before, np.array([0.0]))
        np.testing.assert_array_equal(y_after, np.array([1.0]))


if __name__ == "__main__":
    unittest.main()
