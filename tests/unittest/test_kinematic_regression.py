"""Kinematic visual-regression tests (geometry-level + PNG hash)."""

from __future__ import annotations

import unittest

import numpy as np

from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycle,
    DynamicBicycleCar3D,
)
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from tests.fixtures.generate_kinematic_baselines import (
    REFERENCE_SYSTEMS,
    _MpcPlanBicycleRateBaseline,
)
from tests.fixtures.kinematic_capture import (
    capture_draw_list,
    capture_draw_list_dict,
    capture_frame_png_hash,
    compare_draw_lists,
    load_baseline,
)


def _has_upgraded_api(sys) -> bool:
    return hasattr(sys, "tf") and not hasattr(sys, "get_kinematic_transforms") or callable(
        getattr(sys, "tf", None)
    )


class TestKinematicRegression(unittest.TestCase):
    def _capture(self, sys, x, u, t):
        if hasattr(sys, "tf"):
            geom = sys.get_kinematic_geometry()
            if isinstance(geom, dict):
                return capture_draw_list_dict(sys, x, u, t)
        return capture_draw_list(sys, x, u, t)

    def _assert_matches_baseline(self, name, factory):
        baseline = load_baseline(name)
        sys = factory()
        for case in baseline:
            x = np.asarray(case["x"], dtype=float)
            u = np.asarray(case["u"], dtype=float)
            t = case["t"]
            actual = self._capture(sys, x, u, t)
            compare_draw_lists(actual, case["draw_list"])
            png_hash = capture_frame_png_hash(
                sys, x, u, t, is_3d=(name == "dynamic_bicycle_car3d")
            )
            self.assertEqual(png_hash, case["png_hash"], f"{name} PNG hash at t={t}")

    def test_dynamic_bicycle(self):
        self._assert_matches_baseline("dynamic_bicycle", DynamicBicycle)

    def test_dynamic_bicycle_car3d(self):
        self._assert_matches_baseline("dynamic_bicycle_car3d", DynamicBicycleCar3D)

    def test_pendulum(self):
        self._assert_matches_baseline("pendulum", lambda: Pendulum(length=1.0))

    def test_two_link_manipulator(self):
        self._assert_matches_baseline("two_link_manipulator", TwoLinkManipulator)

    def test_holonomic_mobile_robot(self):
        self._assert_matches_baseline(
            "holonomic_mobile_robot", HolonomicMobileRobot
        )

    def test_mpc_plan_bicycle(self):
        self._assert_matches_baseline("mpc_plan_bicycle", _MpcPlanBicycleRateBaseline)


if __name__ == "__main__":
    unittest.main()
