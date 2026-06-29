"""Implicit ``world`` frame contract: tf omits identity; pipeline injects it."""

from __future__ import annotations

import unittest

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.kinematics import SE2
from minilink.core.system import DynamicSystem, StaticSystem
from minilink.dynamics.catalog.equations.integrators import SimpleIntegrator
from minilink.graphical.animation.drawables import SceneHistory
from minilink.graphical.animation.primitives import CustomLine, Point
from minilink.graphical.animation.visualization import (
    WORLD,
    ensure_world_frame,
    flatten_draw_list,
)
from tests.unittest.graphics_contract_helpers import geometry_smoke, resolve_draw_frame


class WorldOnlyPlant(StaticSystem):
    """World-fixed geometry with an empty ``tf`` (implicit world)."""

    def get_kinematic_geometry(self):
        return {
            "world": [
                CustomLine(
                    np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]),
                    color="k",
                )
            ]
        }

    def tf(self, x, u, t=0, params=None):
        return {}


class TestEnsureWorldFrame(unittest.TestCase):
    def test_injects_identity_when_absent(self):
        frames = ensure_world_frame({})
        self.assertIn(WORLD, frames)
        np.testing.assert_allclose(frames[WORLD], np.eye(4))

    def test_preserves_existing_frames(self):
        body = SE2(1.0, 2.0, 0.5)
        frames = ensure_world_frame({"body": body})
        np.testing.assert_allclose(frames["body"], body)
        np.testing.assert_allclose(frames[WORLD], np.eye(4))


class TestEmptyTfWorldGeometry(unittest.TestCase):
    def test_flatten_draw_list_resolves_world_keyed_primitives(self):
        plant = WorldOnlyPlant()
        draw_list = flatten_draw_list(
            plant.tf(np.array([]), np.array([])), plant.get_kinematic_geometry()
        )
        self.assertEqual(len(draw_list), 1)
        _, transform = draw_list[0]
        np.testing.assert_allclose(transform, np.eye(4))

    def test_animator_resolve_frame_smoke(self):
        geometry_smoke(WorldOnlyPlant())


class TestIntegratorLocalTransform(unittest.TestCase):
    def test_marker_tracks_state_via_local_transform(self):
        sys = SimpleIntegrator()
        x = np.array([3.0])
        frame = resolve_draw_frame(sys, x, np.zeros(sys.m))
        self.assertEqual(len(frame["primitives"]), 1)
        transform = np.asarray(frame["transforms"][0], dtype=float)
        self.assertAlmostEqual(transform[0, 3], 3.0)
        self.assertAlmostEqual(transform[1, 3], 0.0)


class TestDiagramSharedWorld(unittest.TestCase):
    def test_subsystem_world_geometry_merges_under_shared_world(self):
        diagram = DiagramSystem()
        diagram.add_subsystem(WorldOnlyPlant(), "marker")
        geom = diagram.get_dynamic_geometry(
            np.zeros(diagram.n), np.zeros(diagram.m)
        )
        self.assertIn("world", geom)
        self.assertNotIn("marker:world", geom)
        self.assertEqual(len(geom["world"]), 1)

    def test_diagram_tf_omits_namespaced_world(self):
        diagram = DiagramSystem()
        diagram.add_subsystem(WorldOnlyPlant(), "marker")
        frames = diagram.tf(np.zeros(diagram.n), np.zeros(diagram.m))
        self.assertNotIn("marker:world", frames)

    def test_diagram_resolves_shared_world_geometry(self):
        diagram = DiagramSystem()
        diagram.add_subsystem(WorldOnlyPlant(), "marker")
        frame = resolve_draw_frame(diagram, np.zeros(diagram.n), np.zeros(diagram.m))
        self.assertEqual(len(frame["primitives"]), 1)

    def test_multiple_subsystems_merge_world_geometry(self):
        class WorldLineA(WorldOnlyPlant):
            def get_kinematic_geometry(self):
                return {
                    "world": [
                        CustomLine(
                            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                            color="k",
                        )
                    ]
                }

        class WorldLineB(WorldOnlyPlant):
            def get_kinematic_geometry(self):
                return {
                    "world": [
                        CustomLine(
                            np.array([[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]),
                            color="r",
                        )
                    ]
                }

        diagram = DiagramSystem()
        diagram.add_subsystem(WorldLineA(), "a")
        diagram.add_subsystem(WorldLineB(), "b")
        geom = diagram.get_dynamic_geometry(
            np.zeros(diagram.n), np.zeros(diagram.m)
        )
        self.assertEqual(len(geom["world"]), 2)
        frame = resolve_draw_frame(diagram, np.zeros(diagram.n), np.zeros(diagram.m))
        self.assertEqual(len(frame["primitives"]), 2)

    def test_articulated_frames_still_namespaced(self):
        class BodyPlant(DynamicSystem):
            def __init__(self):
                super().__init__(1, input_dim=1, output_dim=1, expose_state=True)

            def f(self, x, u, t=0.0, params=None):
                return np.array([u[0]])

            def h(self, x, u, t=0.0, params=None):
                return np.array([x[0]])

            def tf(self, x, u, t=0, params=None):
                return {"body": SE2(float(x[0]), 0.0, 0.0)}

            def get_kinematic_geometry(self):
                return {}

        diagram = DiagramSystem()
        diagram.add_subsystem(BodyPlant(), "vehicle")
        frames = diagram.tf(np.array([2.0]), np.zeros(diagram.m))
        self.assertIn("vehicle:body", frames)
        self.assertNotIn("vehicle:world", frames)


class TestOverlayImplicitWorld(unittest.TestCase):
    def test_scene_history_tf_is_empty(self):
        history = SceneHistory(
            reference=CustomLine([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], color="k")
        )
        self.assertEqual(history.tf(), {})
        draw_list = flatten_draw_list(
            history.tf(), history.get_kinematic_geometry()
        )
        self.assertEqual(len(draw_list), 1)


class TestCameraFollowWorld(unittest.TestCase):
    def test_empty_tf_plant_with_camera_follow_world(self):
        class FollowWorldPlant(DynamicSystem):
            def __init__(self):
                super().__init__(1, input_dim=1, output_dim=1, expose_state=True)
                self.camera_follow_frame = "world"

            def f(self, x, u, t=0.0, params=None):
                return np.array([u[0]])

            def h(self, x, u, t=0.0, params=None):
                return np.array([x[0]])

            def tf(self, x, u, t=0, params=None):
                return {}

            def get_kinematic_geometry(self):
                marker = Point(color="blue", marker="o", size=8)
                marker.local_transform = SE2(0.0, 0.0, 0.0)
                return {"world": [marker]}

        frame = resolve_draw_frame(FollowWorldPlant(), np.zeros(1), np.zeros(1))
        self.assertIsNotNone(frame["camera"])


if __name__ == "__main__":
    unittest.main()
