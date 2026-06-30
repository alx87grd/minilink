"""Tests for overlay drawables and ``animate(overlays=[...])``."""

from __future__ import annotations

import unittest

import matplotlib

matplotlib.use("Agg")

import numpy as np

from minilink.core.geometry import Box as GeomBox
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.graphical.animation import Animator
from minilink.graphical.animation.drawables import (
    Replay,
    SceneHistory,
    validate_overlay,
)
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.catalog import SceneHistory as SceneHistoryExport
from minilink.planning.spatial.scene import Scene


class TestOverlayValidation(unittest.TestCase):
    def test_rejects_raw_scene(self):
        scene = Scene(obstacles=[GeomBox(np.array([0.0, 0.0]), np.array([1.0, 1.0]))])
        with self.assertRaisesRegex(TypeError, "as_visualizer"):
            validate_overlay(scene)

    def test_rejects_bare_system(self):
        sys = DynamicSystem(1, output_dim=1, expose_state=True)
        with self.assertRaisesRegex(TypeError, "Replay"):
            validate_overlay(sys)


class TestSceneHistory(unittest.TestCase):
    def test_dynamic_layers_rebuild_with_time(self):
        plans = [
            (
                0.0,
                Trajectory(
                    t=np.array([0.0, 1.0]),
                    x=np.array([[0.0, 1.0], [0.0, 0.0]]),
                    u=np.zeros((0, 2)),
                ),
            )
        ]
        history = SceneHistory(
            reference=CustomLine([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]], color="k"),
            horizon=HorizonPolyline(plans, color="tab:orange"),
        )
        kin = history.get_kinematic_geometry()
        self.assertEqual(len(kin["world"]), 1)

        early_pts = history.get_dynamic_geometry(t=0.0)["world"][0].pts
        late_pts = history.get_dynamic_geometry(t=0.5)["world"][0].pts
        self.assertFalse(np.allclose(early_pts, late_pts))

    def test_exported_from_catalog(self):
        self.assertIs(SceneHistoryExport, SceneHistory)


class TestReplay(unittest.TestCase):
    def test_forwards_drawable_at_trajectory_time(self):
        class MarkerPlant(DynamicSystem):
            def __init__(self):
                super().__init__(0)
                self.last_t = None

            def tf(self, x, u, t=0, params=None):
                self.last_t = t
                return {}

        plant = MarkerPlant()
        traj = Trajectory(
            t=np.array([0.0, 0.5, 1.0]),
            x=np.zeros((0, 3)),
            u=np.zeros((0, 3)),
        )
        ghost = Replay(plant, traj)
        ghost.tf(t=0.5)
        self.assertAlmostEqual(plant.last_t, 0.5)


class TestTrajectoryPolyline(unittest.TestCase):
    def test_prefix_window_grows_with_playback_time(self):
        traj = Trajectory(
            t=np.array([0.0, 1.0, 2.0, 3.0]),
            x=np.array(
                [
                    [0.0, 1.0, 2.0, 3.0],
                    [0.0, 0.2, 0.1, 0.0],
                    np.zeros(4),
                    np.full(4, 5.0),
                    np.zeros(4),
                    np.zeros(4),
                ]
            ),
            u=np.zeros((2, 4)),
        )
        prim = TrajectoryPolyline(traj, window="prefix")
        pts_early = prim.compute_pts(1.0)
        pts_late = prim.compute_pts(2.5)
        self.assertEqual(pts_early.shape[0], 2)
        self.assertEqual(pts_late.shape[0], 3)


class TestAnimatorOverlays(unittest.TestCase):
    def test_merges_overlay_primitives(self):
        sys = DynamicSystem(0)
        history = SceneHistory(
            reference=CustomLine([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], color="k")
        )
        frame = Animator(sys)._resolve_frame(
            np.array([]),
            np.array([]),
            0.0,
            kinematic=sys.get_kinematic_geometry(),
            overlays=[history],
        )
        self.assertEqual(len(frame["primitives"]), 1)

    def test_scene_visualizer_draws_obstacle(self):
        scene = Scene(obstacles=[GeomBox(np.array([1.0, 2.0]), np.array([3.0, 4.0]))])
        sys = DynamicSystem(0)
        frame = Animator(sys)._resolve_frame(
            np.array([]),
            np.array([]),
            0.0,
            kinematic=sys.get_kinematic_geometry(),
            overlays=[scene.as_visualizer()],
        )
        self.assertEqual(len(frame["primitives"]), 1)


if __name__ == "__main__":
    unittest.main()
