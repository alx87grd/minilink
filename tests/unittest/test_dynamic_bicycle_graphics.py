import unittest

import numpy as np

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycle,
    DynamicBicycleCar3D,
    DynamicBicycleCar3DRealistic,
)
from minilink.graphical.animation.primitives import ExtrudedPolygon


class TestDynamicBicycleCamera(unittest.TestCase):
    def test_camera_follows_vehicle_position_by_default(self):
        sys = DynamicBicycle()
        sys.camera_target[:] = (1.0, -2.0, 0.5)
        sys.camera_scale = 7.0
        x = np.array([10.0, 3.0, 0.25, 4.0, 0.0, 0.0])
        u = np.zeros(sys.m)

        camera = sys.get_camera_transform(x, u, 0.0)

        np.testing.assert_allclose(camera[:3, 3], np.array([11.0, 1.0, 0.5]))
        self.assertEqual(camera[3, 3], 7.0)

    def test_camera_follow_can_be_disabled(self):
        sys = DynamicBicycle()
        sys.camera_follow_vehicle = False
        sys.camera_target[:] = (1.0, -2.0, 0.5)
        x = np.array([10.0, 3.0, 0.25, 4.0, 0.0, 0.0])
        u = np.zeros(sys.m)

        camera = sys.get_camera_transform(x, u, 0.0)

        np.testing.assert_allclose(camera[:3, 3], np.array([1.0, -2.0, 0.5]))


class TestDynamicBicycleCar3DRealisticGraphics(unittest.TestCase):
    def test_realistic_car_adds_richer_geometry_without_replacing_old_class(self):
        old_sys = DynamicBicycleCar3D()
        new_sys = DynamicBicycleCar3DRealistic()

        old_primitives = old_sys.get_kinematic_geometry()
        new_primitives = new_sys.get_kinematic_geometry()

        self.assertGreater(len(new_primitives), len(old_primitives))
        self.assertTrue(any(isinstance(p, ExtrudedPolygon) for p in new_primitives))

    def test_realistic_car_geometry_and_transform_counts_match(self):
        sys = DynamicBicycleCar3DRealistic()
        x = np.zeros(sys.n)
        u = np.zeros(sys.m)

        primitives = sys.get_kinematic_geometry()
        transforms = sys.get_kinematic_transforms(x, u, 0.0)

        self.assertEqual(len(primitives), len(transforms))
        for T in transforms:
            self.assertEqual(T.shape, (4, 4))


if __name__ == "__main__":
    unittest.main()
