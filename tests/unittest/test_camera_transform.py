"""Tests for the standard camera transform contract (DESIGN.md §3.2 / §4.7a)."""

from __future__ import annotations

import unittest

import matplotlib

matplotlib.use("Agg")  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from minilink.core.system import DynamicSystem
from minilink.graphical.animation import Animator
from minilink.graphical.environment import override_env
from minilink.graphical.primitives import (
    attach_standard_camera,
    camera_matrix,
    world_to_camera,
)
from minilink.graphical.renderers.matplotlib_renderer import (
    _axis_label_from_column,
    _camera_3d_view_init,
)


class TestCameraMatrix(unittest.TestCase):
    def test_default_matches_legacy_box(self):
        T = camera_matrix()
        np.testing.assert_array_equal(T[:3, :3], np.eye(3))
        np.testing.assert_array_equal(T[:3, 3], np.zeros(3))
        self.assertEqual(T[3, 3], 10.0)

    def test_plot_axes_xz_swaps_columns(self):
        T = camera_matrix(plot_axes=(0, 2), scale=5.0)
        np.testing.assert_array_equal(T[:3, 0], [1.0, 0.0, 0.0])
        np.testing.assert_array_equal(T[:3, 1], [0.0, 0.0, 1.0])
        np.testing.assert_array_equal(T[:3, 2], [0.0, -1.0, 0.0])
        self.assertEqual(T[3, 3], 5.0)

    def test_plot_axes_yz_is_right_handed(self):
        T = camera_matrix(plot_axes=(1, 2))
        np.testing.assert_array_equal(T[:3, 0], [0.0, 1.0, 0.0])
        np.testing.assert_array_equal(T[:3, 1], [0.0, 0.0, 1.0])
        np.testing.assert_array_equal(T[:3, 2], [1.0, 0.0, 0.0])

    def test_target_translates(self):
        T = camera_matrix(target=(2.0, -1.0, 3.0), scale=4.0)
        np.testing.assert_array_equal(T[:3, 3], [2.0, -1.0, 3.0])
        self.assertEqual(T[3, 3], 4.0)

    def test_explicit_R_overrides_plot_axes(self):
        R = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        T = camera_matrix(plot_axes=(0, 2), R=R)
        np.testing.assert_array_equal(T[:3, :3], R)

    def test_invalid_plot_axes_raises(self):
        with self.assertRaises(ValueError):
            camera_matrix(plot_axes=(0, 0))
        with self.assertRaises(ValueError):
            camera_matrix(plot_axes=(0, 5))

    def test_world_to_camera_is_inverse_of_pose(self):
        T = camera_matrix(target=(2.0, -1.0, 3.0), plot_axes=(0, 2), scale=7.0)
        W = world_to_camera(T)
        # Stripping the amplitude channel, T_pose @ W must give identity rigid xform.
        T_pose = T.copy()
        T_pose[3, 3] = 1.0
        np.testing.assert_allclose(T_pose @ W, np.eye(4), atol=1e-12)

    def test_world_to_camera_centers_target_at_origin(self):
        target = np.array([2.0, -1.0, 3.0])
        T = camera_matrix(target=target, scale=4.0)
        W = world_to_camera(T)
        p = np.append(target, 1.0)
        np.testing.assert_allclose(W @ p, [0.0, 0.0, 0.0, 1.0], atol=1e-12)


class TestSystemDefaultCamera(unittest.TestCase):
    def test_default_camera_matches_factory(self):
        s = DynamicSystem(2, 1, 1)
        T = s.get_camera_transform(np.zeros(2), np.zeros(1), 0.0)
        np.testing.assert_array_equal(T, camera_matrix())

    def test_attach_standard_camera_forwards_kwargs(self):
        s = DynamicSystem(2, 1, 1)
        attach_standard_camera(s, scale=4.5, plot_axes=(0, 2))
        T = s.get_camera_transform(np.zeros(2), np.zeros(1), 0.0)
        np.testing.assert_array_equal(T, camera_matrix(scale=4.5, plot_axes=(0, 2)))
        self.assertEqual(s.camera_scale, 4.5)
        self.assertEqual(s.camera_plot_axes, (0, 2))

    def test_camera_attributes_override_without_attach(self):
        s = DynamicSystem(2, 1, 1)
        s.camera_scale = 2.0
        s.camera_plot_axes = (1, 2)
        s.camera_target[:] = (1.0, -1.0, 0.5)
        T = s.get_camera_transform(np.zeros(2), np.zeros(1), 0.0)
        np.testing.assert_array_equal(
            T,
            camera_matrix(target=(1.0, -1.0, 0.5), plot_axes=(1, 2), scale=2.0),
        )


class TestAnimatorPipesCameraToRenderer(unittest.TestCase):
    def setUp(self):
        override_env("jupyter")
        self.addCleanup(override_env, None)

    def test_default_2d_view_uses_target_plus_minus_scale(self):
        s = DynamicSystem(2, 1, 1)
        a = Animator(s)
        a.show(np.zeros(2), np.zeros(1), 0.0, is_3d=False, renderer="matplotlib")
        # The animator closes its figure in show(); inspect via a fresh open_scene.
        from minilink.graphical.renderers.matplotlib_renderer import MatplotlibRenderer

        backend = MatplotlibRenderer(a)
        backend.open_scene(
            is_3d=False,
            show=False,
            camera=s.get_camera_transform(np.zeros(2), np.zeros(1), 0.0),
        )
        self.assertEqual(backend.ax.get_xlim(), (-10.0, 10.0))
        self.assertEqual(backend.ax.get_ylim(), (-10.0, 10.0))
        self.assertEqual(backend.ax.get_xlabel(), "X")
        self.assertEqual(backend.ax.get_ylabel(), "Y")
        plt.close(backend.fig)

    def test_xz_camera_sets_z_as_vertical_axis(self):
        s = DynamicSystem(2, 1, 1)
        s.get_camera_transform = lambda x, u, t: camera_matrix(
            plot_axes=(0, 2), scale=3.0
        )
        a = Animator(s)
        from minilink.graphical.renderers.matplotlib_renderer import MatplotlibRenderer

        backend = MatplotlibRenderer(a)
        backend.open_scene(
            is_3d=False,
            show=False,
            camera=s.get_camera_transform(np.zeros(2), np.zeros(1), 0.0),
        )
        self.assertEqual(backend.ax.get_xlim(), (-3.0, 3.0))
        self.assertEqual(backend.ax.get_ylim(), (-3.0, 3.0))
        self.assertEqual(backend.ax.get_xlabel(), "X")
        self.assertEqual(backend.ax.get_ylabel(), "Z")
        plt.close(backend.fig)

    def test_follow_target_shifts_3d_view_box(self):
        s = DynamicSystem(2, 1, 1)
        s.get_camera_transform = lambda x, u, t: camera_matrix(
            target=(5.0, -2.0, 1.0), scale=4.0
        )
        a = Animator(s)
        from minilink.graphical.renderers.matplotlib_renderer import MatplotlibRenderer

        backend = MatplotlibRenderer(a)
        backend.open_scene(
            is_3d=True,
            show=False,
            camera=s.get_camera_transform(np.zeros(2), np.zeros(1), 0.0),
        )
        self.assertEqual(backend.ax.get_xlim3d(), (1.0, 9.0))
        self.assertEqual(backend.ax.get_ylim3d(), (-6.0, 2.0))
        self.assertEqual(backend.ax.get_zlim3d(), (-3.0, 5.0))
        plt.close(backend.fig)


class TestRendererHelpers(unittest.TestCase):
    def test_axis_label_axis_aligned(self):
        self.assertEqual(_axis_label_from_column([1.0, 0.0, 0.0]), "X")
        self.assertEqual(_axis_label_from_column([0.0, 1.0, 0.0]), "Y")
        self.assertEqual(_axis_label_from_column([0.0, 0.0, 1.0]), "Z")
        self.assertEqual(_axis_label_from_column([-1.0, 0.0, 0.0]), "-X")

    def test_axis_label_blank_for_oblique(self):
        v = np.array([1.0, 1.0, 0.0]) / np.sqrt(2)
        self.assertEqual(_axis_label_from_column(v), "")

    def test_view_init_decode_default_top_down(self):
        T = camera_matrix()  # R = I → view-out = +Z
        elev, azim = _camera_3d_view_init(T)
        self.assertAlmostEqual(elev, 90.0)
        self.assertAlmostEqual(azim, 0.0)

    def test_view_init_decode_xz_view(self):
        T = camera_matrix(plot_axes=(0, 2))  # view-out = -Y
        elev, azim = _camera_3d_view_init(T)
        self.assertAlmostEqual(elev, 0.0, places=5)
        self.assertAlmostEqual(azim, -90.0, places=5)


if __name__ == "__main__":
    unittest.main()
