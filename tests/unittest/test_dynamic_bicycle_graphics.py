import unittest

import numpy as np

from minilink.dynamics.vehicles.dynamic_bicycle import (
    DynamicBicycleCar3D,
    DynamicBicycleCar3DRealistic,
)
from minilink.graphical.primitives import ExtrudedPolygon


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
