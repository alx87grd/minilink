import unittest

import numpy as np

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.animation.primitives import Arrow

from tests.unittest.graphics_contract_helpers import resolve_draw_frame


class TestDynamicBicycle(unittest.TestCase):
    def test_dimensions_and_labels(self):
        sys = DynamicBicycle()

        self.assertEqual(sys.n, 6)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.p, 6)
        self.assertEqual(sys.state.labels, ["x", "y", "theta", "vx", "vy", "yaw_rate"])
        self.assertEqual(sys.inputs["w_rear"].labels, ["w_rear"])
        self.assertEqual(sys.inputs["delta"].labels, ["delta"])

    def test_camera_follows_vehicle_position_by_default(self):
        sys = DynamicBicycle()
        sys.camera_target[:] = (1.0, -2.0, 0.5)
        sys.camera_scale = 7.0
        x = np.array([10.0, 3.0, 0.25, 4.0, 0.0, 0.0])
        u = np.zeros(sys.m)

        camera = sys.get_camera_transform(x, u, 0.0)

        np.testing.assert_allclose(camera[:3, 3], np.array([11.0, 1.0, 0.5]))
        self.assertEqual(camera[3, 3], 7.0)

    def test_dynamics_reference_value(self):
        sys = DynamicBicycle()
        x = np.array([0.1, -0.2, 0.3, 5.0, 0.4, 0.05])
        u = np.array([20.0, 0.1])

        dx = sys.f(x, u)

        np.testing.assert_allclose(
            dx[:3],
            sys.N(x[:3]) @ x[3:],
        )
        self.assertEqual(dx.shape, (6,))
        self.assertGreater(dx[3], -20.0)

    def test_graphics_geometry_and_transform_counts_match(self):
        sys = DynamicBicycle()
        x = np.zeros(sys.n)
        x[3] = 1.0
        u = np.array([10.0, 0.1])

        frame = resolve_draw_frame(sys, x, u, 0.0)
        primitives = frame["primitives"]
        transforms = frame["transforms"]

        self.assertEqual(len(primitives), 7)
        self.assertEqual(len(primitives), len(transforms))
        self.assertEqual(sum(isinstance(item, Arrow) for item in primitives), 4)
        for T in transforms:
            self.assertEqual(T.shape, (4, 4))
            self.assertTrue(np.all(np.isfinite(T)))


if __name__ == "__main__":
    unittest.main()
