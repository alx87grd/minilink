import unittest

import numpy as np

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.primitives import Arrow, CustomLine


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

        camera = resolve_camera_from_hints(sys, sys.tf(x, u, 0.0), 0.0)

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

        geometry = sys.get_kinematic_geometry()
        dynamic = sys.get_dynamic_geometry(x, u, 0.0)
        frames = sys.tf(x, u, 0.0)

        self.assertEqual(len(geometry), 3)
        self.assertEqual(len(dynamic), 1)
        self.assertIn("world", dynamic)
        for key in geometry:
            self.assertIn(key, frames)
        self.assertIn("world", frames)
        arrow_count = sum(
            isinstance(item, Arrow)
            for items in dynamic.values()
            for item in items
        )
        self.assertEqual(arrow_count, 4)
        for T in frames.values():
            self.assertEqual(T.shape, (4, 4))
            self.assertTrue(np.all(np.isfinite(T)))


if __name__ == "__main__":
    unittest.main()
