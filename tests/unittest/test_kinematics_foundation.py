"""Unit tests for kinematic foundation modules (pre-contract-upgrade)."""

from __future__ import annotations

import unittest

import numpy as np

from minilink.core.kinematics import (
    apply_transform,
    identity_matrix,
    invert_transform,
    pose2d_matrix,
    translation_matrix,
)
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.animation.builders import arrow_pts, torque_arc_pts
from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.primitives import CustomLine, Point
from minilink.graphical.animation.visualization import flatten_draw_list


class TestKinematicsFoundation(unittest.TestCase):
    def test_pose2d_round_trip(self):
        T = pose2d_matrix(1.0, 2.0, 0.5)
        T_inv = invert_transform(T)
        p = np.array([0.5, -0.3, 0.0])
        world = apply_transform(T, p)
        back = apply_transform(T_inv, world)
        np.testing.assert_allclose(back, p, atol=1e-12)

    def test_flatten_draw_list_local_transform(self):
        prim = Point(pt=(1.0, 0.0, 0.0))
        prim.local_transform = translation_matrix(0.0, 2.0, 0.0)
        frames = {"body": pose2d_matrix(3.0, 4.0, 0.0)}
        skin = {"body": [prim]}
        draw_list = flatten_draw_list(frames, skin, {})
        self.assertEqual(len(draw_list), 1)
        _, world_T = draw_list[0]
        local = np.array([1.0, 0.0, 0.0, 1.0])
        world = world_T @ local
        np.testing.assert_allclose(world[:3], [4.0, 6.0, 0.0], atol=1e-12)

    def test_flatten_missing_frame_raises(self):
        with self.assertRaises(KeyError):
            flatten_draw_list({"body": identity_matrix()}, {"wheel": [Point()]}, {})

    def test_resolve_camera_matches_legacy_follow(self):
        sys = DynamicBicycle()
        sys.camera_target[:] = (1.0, -2.0, 0.5)
        sys.camera_scale = 7.0
        sys.camera_follow_frame = "body"
        x = np.array([10.0, 3.0, 0.25, 4.0, 0.0, 0.0])
        u = np.zeros(sys.m)
        frames = sys.tf(x, u, 0.0)
        resolved = resolve_camera_from_hints(sys, frames, 0.0)
        np.testing.assert_allclose(resolved[:3, 3], np.array([11.0, 1.0, 0.5]))
        self.assertEqual(resolved[3, 3], 7.0)

    def test_arrow_pts_nonzero_length(self):
        pts = arrow_pts(vector=(3.0, 4.0), scale=1.0)
        self.assertGreater(pts.shape[0], 1)
        np.testing.assert_allclose(pts[0], [0.0, 0.0, 0.0])

    def test_torque_arc_pts_sweep(self):
        pts = torque_arc_pts(radius=1.0, sweep=0.5)
        self.assertGreater(pts.shape[0], 3)


if __name__ == "__main__":
    unittest.main()
