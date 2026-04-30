import unittest

import numpy as np

from minilink.core.blocks.sources import WhiteNoise


class TestWhiteNoiseSource(unittest.TestCase):
    def test_same_seed_same_refresh_same_output(self):
        times = np.linspace(0.0, 2.0, 200)

        n1 = WhiteNoise(1)
        n1.params["seed"] = 123
        n1.params["sample_period"] = 0.01
        n1.params["t0"] = -1.0
        n1.params["tf"] = 3.0
        n1.refresh()
        y1 = np.array([n1.h(np.array([]), np.array([]), t)[0] for t in times])

        n2 = WhiteNoise(1)
        n2.params["seed"] = 123
        n2.params["sample_period"] = 0.01
        n2.params["t0"] = -1.0
        n2.params["tf"] = 3.0
        n2.refresh()
        y2 = np.array([n2.h(np.array([]), np.array([]), t)[0] for t in times])

        self.assertTrue(np.allclose(y1, y2))

    def test_different_seed_changes_output(self):
        times = np.linspace(0.0, 2.0, 200)

        n1 = WhiteNoise(1)
        n1.params["seed"] = 1
        n1.refresh()
        y1 = np.array([n1.h(np.array([]), np.array([]), t)[0] for t in times])

        n2 = WhiteNoise(1)
        n2.params["seed"] = 2
        n2.refresh()
        y2 = np.array([n2.h(np.array([]), np.array([]), t)[0] for t in times])

        self.assertFalse(np.allclose(y1, y2))

    def test_continuity_with_interpolation(self):
        n = WhiteNoise(1)
        n.params["seed"] = 77
        n.params["sample_period"] = 0.05
        n.params["t0"] = 0.0
        n.params["tf"] = 1.0
        n.refresh()

        t_left = 0.5 - 1e-6
        t_right = 0.5 + 1e-6
        y_left = n.h(np.array([]), np.array([]), t_left)[0]
        y_right = n.h(np.array([]), np.array([]), t_right)[0]

        self.assertLess(abs(y_right - y_left), 1e-2)

    def test_refresh_horizon_changes_edge_values(self):
        n = WhiteNoise(1)
        n.params["seed"] = 10
        n.params["t0"] = -100.0
        n.params["tf"] = 100.0
        n.refresh()
        y_at_minus_five = n.h(np.array([]), np.array([]), -5.0)[0]

        n.params["t0"] = 0.0
        n.params["tf"] = 1.0
        n.refresh()
        y_left_clamped = n.h(np.array([]), np.array([]), -5.0)[0]

        self.assertNotEqual(y_at_minus_five, y_left_clamped)


if __name__ == "__main__":
    unittest.main()
