"""Tests for the standard camera transform contract (DESIGN.md §3.2 / §4.7a)."""

import contextlib
import importlib
import io
import unittest
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from minilink.core.composition import _propagate_animation_camera
from minilink.core.diagram import DiagramSystem
from minilink.core.kinematics import translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation import Animator
from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.primitives import (
    Point,
    camera_matrix,
    world_to_camera,
)
from minilink.graphical.animation.renderers.matplotlib_renderer import (
    _axis_label_from_column,
    _camera_3d_view_init,
)
from minilink.graphical.common.environment import override_env


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

    def test_invalid_plot_axes_raises(self):
        with self.assertRaises(ValueError):
            camera_matrix(plot_axes=(0, 0))
        with self.assertRaises(ValueError):
            camera_matrix(plot_axes=(0, 5))

    def test_world_to_camera_is_inverse_of_pose(self):
        T = camera_matrix(target=(2.0, -1.0, 3.0), plot_axes=(0, 2), scale=7.0)
        W = world_to_camera(T)
        T_pose = T.copy()
        T_pose[3, 3] = 1.0
        np.testing.assert_allclose(T_pose @ W, np.eye(4), atol=1e-12)

    def test_world_to_camera_centers_target_at_origin(self):
        target = np.array([2.0, -1.0, 3.0])
        T = camera_matrix(target=target, scale=4.0)
        W = world_to_camera(T)
        p = np.append(target, 1.0)
        np.testing.assert_allclose(W @ p, [0.0, 0.0, 0.0, 1.0], atol=1e-12)


class TestResolveCameraFromHints(unittest.TestCase):
    def test_default_camera_matches_factory(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        T = resolve_camera_from_hints(s, {}, np.zeros(2), np.zeros(1), 0.0)
        np.testing.assert_array_equal(T, camera_matrix())

    def test_camera_attributes_match_camera_matrix(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        s.camera_scale = 2.0
        s.camera_plot_axes = (1, 2)
        s.camera_target[:] = (1.0, -1.0, 0.5)
        T = resolve_camera_from_hints(s, {}, np.zeros(2), np.zeros(1), 0.0)
        np.testing.assert_array_equal(
            T, camera_matrix(target=(1.0, -1.0, 0.5), plot_axes=(1, 2), scale=2.0)
        )

    def test_follow_frame_adds_frame_origin_to_target(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        s.camera_follow_frame = "body"
        s.camera_scale = 4.0
        frames = {"body": translation(10.0, 3.0, 0.0)}
        T = resolve_camera_from_hints(s, frames, np.zeros(2), np.zeros(1), 0.0)
        np.testing.assert_allclose(T[:3, 3], [10.0, 3.0, 0.0])
        self.assertEqual(T[3, 3], 4.0)

    def test_override_constant_matrix(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        override = camera_matrix(target=(5.0, -2.0, 1.0), scale=4.0)
        T = resolve_camera_from_hints(
            s, {}, np.zeros(2), np.zeros(1), 0.0, override=override
        )
        np.testing.assert_array_equal(T, override)

    def test_override_callable(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)

        def custom(frames, x, u, t):
            return camera_matrix(plot_axes=(0, 2), scale=3.0)

        T = resolve_camera_from_hints(
            s, {}, np.zeros(2), np.zeros(1), 0.0, override=custom
        )
        np.testing.assert_array_equal(T, camera_matrix(plot_axes=(0, 2), scale=3.0))


class TestPropagateAnimationCamera(unittest.TestCase):
    def test_namespaces_follow_frame_on_diagram(self):
        plant = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        plant.skin = lambda _self: {"body": []}
        plant.camera_follow_frame = "body"
        plant.camera_scale = 6.0
        diagram = DiagramSystem()
        diagram.add_subsystem(plant, "bike")
        _propagate_animation_camera(diagram, plant)
        self.assertEqual(diagram.camera_follow_frame, "bike:body")
        self.assertEqual(diagram.camera_scale, 6.0)


class TestAnimatorPipesCameraToRenderer(unittest.TestCase):
    def setUp(self):
        override_env("jupyter")
        self.addCleanup(override_env, None)

    def test_default_2d_view_uses_target_plus_minus_scale(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        a = Animator(s)
        a.show(np.zeros(2), np.zeros(1), 0.0, is_3d=False, renderer="matplotlib")
        from minilink.graphical.animation.renderers.matplotlib_renderer import (
            MatplotlibRenderer,
        )

        backend = MatplotlibRenderer(a)
        camera = resolve_camera_from_hints(s, {}, np.zeros(2), np.zeros(1), 0.0)
        backend.open_scene(is_3d=False, show=False, camera=camera)
        self.assertEqual(backend.ax.get_xlim(), (-10.0, 10.0))
        self.assertEqual(backend.ax.get_ylim(), (-10.0, 10.0))
        self.assertEqual(backend.ax.get_xlabel(), "X")
        self.assertEqual(backend.ax.get_ylabel(), "Y")
        plt.close(backend.fig)

    def test_follow_frame_keeps_2d_geometry_in_world_coordinates(self):
        from minilink.graphical.animation.renderers.matplotlib_renderer import (
            MatplotlibRenderer,
        )

        s = DynamicSystem(0)
        s.camera_follow_frame = "body"
        s.camera_scale = 4.0
        a = Animator(s)
        backend = MatplotlibRenderer(a)
        frames = {"body": translation(10.0, 3.0, 0.0)}
        camera = resolve_camera_from_hints(s, frames, np.zeros(0), np.zeros(0), 0.0)
        backend.open_scene(is_3d=False, show=False, camera=camera)
        backend.draw_frame([Point()], [translation(10.0, 3.0, 0.0)], 0.0, camera)
        self.assertEqual(backend.ax.get_xlim(), (6.0, 14.0))
        self.assertEqual(backend.ax.get_ylim(), (-1.0, 7.0))
        point_line = backend.canvas.drawn_objects[0]
        np.testing.assert_allclose(point_line.get_xdata(), np.array([10.0]))
        np.testing.assert_allclose(point_line.get_ydata(), np.array([3.0]))
        plt.close(backend.fig)

    def test_xz_camera_sets_z_as_vertical_axis(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        a = Animator(s)
        from minilink.graphical.animation.renderers.matplotlib_renderer import (
            MatplotlibRenderer,
        )

        backend = MatplotlibRenderer(a)
        camera = camera_matrix(plot_axes=(0, 2), scale=3.0)
        backend.open_scene(is_3d=False, show=False, camera=camera)
        self.assertEqual(backend.ax.get_xlim(), (-3.0, 3.0))
        self.assertEqual(backend.ax.get_ylim(), (-3.0, 3.0))
        self.assertEqual(backend.ax.get_xlabel(), "X")
        self.assertEqual(backend.ax.get_ylabel(), "Z")
        plt.close(backend.fig)

    def test_follow_target_shifts_3d_view_box(self):
        s = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
        a = Animator(s)
        from minilink.graphical.animation.renderers.matplotlib_renderer import (
            MatplotlibRenderer,
        )

        backend = MatplotlibRenderer(a)
        camera = camera_matrix(target=(5.0, -2.0, 1.0), scale=4.0)
        backend.open_scene(is_3d=True, show=False, camera=camera)
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
        T = camera_matrix()
        elev, azim = _camera_3d_view_init(T)
        self.assertAlmostEqual(elev, 90.0)
        self.assertAlmostEqual(azim, 0.0)

    def test_view_init_decode_xz_view(self):
        T = camera_matrix(plot_axes=(0, 2))
        elev, azim = _camera_3d_view_init(T)
        self.assertAlmostEqual(elev, 0.0, places=5)
        self.assertAlmostEqual(azim, -90.0, places=5)


from minilink.core.kinematics import SE2
from minilink.core.system import DynamicSystem, System
from minilink.dynamics.catalog.equations.integrators import SimpleIntegrator
from minilink.graphical.animation.drawables import SceneHistory
from minilink.graphical.animation.primitives import CustomLine, Point
from minilink.graphical.animation.visualization import (
    WORLD,
    ensure_world_frame,
    flatten_draw_list,
)
from tests.unittest.graphics_contract_helpers import resolve_draw_frame


class WorldOnlyPlant(System):
    """World-fixed geometry with an empty ``tf`` (implicit world)."""

    def get_kinematic_geometry(self):
        return {
            "world": [
                CustomLine(np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), color="k")
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
        geom = diagram.get_dynamic_geometry(np.zeros(diagram.n), np.zeros(diagram.m))
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
                            np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]), color="k"
                        )
                    ]
                }

        class WorldLineB(WorldOnlyPlant):
            def get_kinematic_geometry(self):
                return {
                    "world": [
                        CustomLine(
                            np.array([[0.0, 1.0, 0.0], [1.0, 1.0, 0.0]]), color="r"
                        )
                    ]
                }

        diagram = DiagramSystem()
        diagram.add_subsystem(WorldLineA(), "a")
        diagram.add_subsystem(WorldLineB(), "b")
        geom = diagram.get_dynamic_geometry(np.zeros(diagram.n), np.zeros(diagram.m))
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
        draw_list = flatten_draw_list(history.tf(), history.get_kinematic_geometry())
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


from minilink.core.geometry import Box as GeomBox
from minilink.core.trajectory import Trajectory
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
            t=np.array([0.0, 0.5, 1.0]), x=np.zeros((0, 3)), u=np.zeros((0, 3))
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


import pytest
from minilink.graphical.animation.primitives import Point
from minilink.graphical.animation.renderers.meshcat_renderer import (
    MeshcatCanvas,
    _import_meshcat,
)
from minilink.graphical.animation.renderers.pygame_renderer import (
    PygameCanvas,
    _import_pygame,
)


def _has_meshcat():
    try:
        _import_meshcat()
    except ImportError:
        return False
    return True


def _has_pygame():
    try:
        _import_pygame()
    except ImportError:
        return False
    return True


class _FakeMeshcatNode:
    """Minimal meshcat path tree for canvas smoke tests (no ZMQ server)."""

    def __init__(self):
        self.children = {}
        self.object = None
        self.transform = None

    def __getitem__(self, key):
        child = self.children.get(key)
        if child is None:
            child = _FakeMeshcatNode()
            self.children[key] = child
        return child

    def set_object(self, obj):
        self.object = obj

    def set_transform(self, transform):
        self.transform = transform

    def delete(self):
        self.children.clear()
        self.object = None
        self.transform = None


@pytest.mark.optional
@pytest.mark.visualization
class TestVisualizationOptionalImports(unittest.TestCase):
    def test_import_meshcat_reports_extra_when_missing(self):
        try:
            meshcat = _import_meshcat()
        except ImportError as exc:
            self.assertIn("minilink[visualization]", str(exc))
        else:
            self.assertTrue(hasattr(meshcat, "Visualizer"))

    def test_import_pygame_reports_extra_when_missing(self):
        try:
            pygame = _import_pygame()
        except ImportError as exc:
            self.assertIn("minilink[visualization]", str(exc))
        else:
            self.assertTrue(hasattr(pygame, "init"))


@pytest.mark.optional
@pytest.mark.visualization
class TestMeshcatOptionalSmoke(unittest.TestCase):
    @pytest.mark.skipif(not _has_meshcat(), reason="meshcat not installed")
    def test_canvas_can_create_point_geometry_without_opening_browser(self):
        canvas = MeshcatCanvas(_FakeMeshcatNode(), is_3d=True)
        canvas.ensure_objects([Point([0.0, 0.0, 0.0])])
        canvas.update_primitive(0, Point([0.0, 0.0, 0.0]), np.eye(4))
        self.assertEqual(canvas._n_slots, 1)
        slot = canvas.scene["p0"]
        self.assertIsNotNone(slot.object)
        self.assertIsNotNone(slot.transform)
        canvas.clear()


@pytest.mark.optional
@pytest.mark.visualization
class TestPygameOptionalSmoke(unittest.TestCase):
    @pytest.mark.skipif(not _has_pygame(), reason="pygame not installed")
    def test_canvas_maps_world_coordinates_to_screen(self):
        pygame = _import_pygame()
        pygame.init()
        try:
            surface = pygame.Surface((100, 100))
            canvas = PygameCanvas(surface, scale=1.0, is_3d=False)
            self.assertEqual(canvas._to_screen(0.0, 0.0), (50, 50))
        finally:
            pygame.quit()

    @pytest.mark.skipif(not _has_pygame(), reason="pygame not installed")
    def test_canvas_draws_point_to_surface(self):
        pygame = _import_pygame()
        pygame.init()
        try:
            surface = pygame.Surface((100, 100))
            canvas = PygameCanvas(surface, scale=1.0, is_3d=False)
            canvas.draw_primitive(Point([0.0, 0.0, 0.0]), np.eye(4), pygame)
        finally:
            pygame.quit()


import pytest
from minilink.graphical.signals import (
    build_signal_plot_spec,
    open_time_signal_plot,
    plot_time_signals,
    resolve_plot_signals,
)
from minilink.simulation.simulator import Simulator


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])


class PController(System):
    def __init__(self):
        super().__init__()
        self.params = {"Kp": 5.0}
        self.add_input_port("r", nominal_value=1.0)
        self.add_input_port("y", nominal_value=0.0)
        self.add_output_port("u", function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        Kp = params["Kp"] if params else self.params["Kp"]
        r, y = self.get_port_values_from_u(u, "r", "y")
        return np.array([Kp * (r[0] - y[0])])


class Step(System):
    def __init__(self):
        super().__init__()
        self.add_output_port("y", function=self.compute)

    def compute(self, x, u, t=0, params=None):
        return np.array([1.0])


class TestAdvancedPlotting(unittest.TestCase):
    def setUp(self):
        self.sys = Integrator()
        self.ctl = PController()
        self.step = Step()
        self.diagram = DiagramSystem()
        self.diagram.add_subsystem(self.step, "step")
        self.diagram.add_subsystem(self.ctl, "ctl")
        self.diagram.add_subsystem(self.sys, "plant")
        self.diagram.connect("step", "y", "ctl", "r")
        self.diagram.connect("ctl", "u", "plant", "u")
        self.diagram.connect("plant", "y", "ctl", "y")
        self.sim = Simulator(self.diagram, t0=0, tf=2.0, dt=0.1, verbose=False)
        self.traj = self.sim.solve()

    def test_plotting_import_is_quiet(self):
        import minilink.graphical.signals as signals

        interactive = plt.isinteractive()
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            importlib.reload(signals)
        self.assertEqual(stream.getvalue(), "")
        self.assertEqual(plt.isinteractive(), interactive)

    def test_compute_internal_signals(self):
        self.assertFalse(self.traj.has_signal("step:y"))
        traj_plus = self.diagram.reconstruct_internal_signals(self.traj)
        self.assertTrue(traj_plus.has_signal("step:y"))
        self.assertTrue(traj_plus.has_signal("ctl:u"))
        self.assertTrue(traj_plus.has_signal("plant:y"))
        n_pts = len(self.traj.t)
        self.assertEqual(traj_plus.get_signal("ctl:u").shape, (1, n_pts))

    def test_plot_time_signals_does_not_crash(self):
        from minilink.graphical.common.environment import override_env

        override_env("jupyter")
        self.addCleanup(override_env, None)
        result = plot_time_signals(
            self.diagram, self.traj, signals=("x", "ctl:u", "plant:y"), show=False
        )
        self.assertIsNotNone(result.figure)
        plt.close(result.figure)

    def test_plot_time_signals_accepts_subsystem_port_tuples(self):
        from minilink.graphical.common.environment import override_env

        override_env("jupyter")
        self.addCleanup(override_env, None)
        result = plot_time_signals(
            self.diagram,
            self.traj,
            signals=("x", (self.ctl, "u"), (self.sys, "y")),
            show=False,
        )
        self.assertIsNotNone(result.figure)
        plt.close(result.figure)

    def test_plot_time_signals_accepts_single_subsystem_port_pair(self):
        from minilink.graphical.common.environment import override_env

        override_env("jupyter")
        self.addCleanup(override_env, None)
        result = plot_time_signals(
            self.diagram, self.traj, signals=(self.sys, "y"), show=False
        )
        self.assertIsNotNone(result.figure)
        plt.close(result.figure)

    def test_subsystem_signal_resolves_diagram_id(self):
        self.assertEqual(self.diagram.subsystem_signal(self.sys, "y"), "plant:y")
        self.assertEqual(self.diagram.subsystem_signal(self.ctl, "u"), "ctl:u")

    def test_plot_trajectory_computes_missing_trajectory_then_plots(self):
        sys = Integrator()
        result = sys.plot_trajectory(show=False)
        self.assertEqual(result.backend, "matplotlib")
        self.assertIsNotNone(result.figure)
        self.assertIsNotNone(sys.traj)
        plt.close(result.figure)

    def test_resolve_plot_signals_leaf_integrator(self):
        self.assertEqual(resolve_plot_signals(self.sys), ("x", "u"))

    def test_resolve_plot_signals_closed_loop_diagram(self):
        self.assertEqual(resolve_plot_signals(self.diagram), ("x", "step:y", "ctl:u"))

    def test_resolve_plot_signals_lqr_at_cartpole(self):
        from minilink.control.lqr import lqr_at_operating_point
        from minilink.dynamics.catalog.pendulum.cartpole import CartPole

        plant = CartPole()
        controller = lqr_at_operating_point(
            plant,
            [0.0, np.pi, 0.0, 0.0],
            np.diag([1.0, 10.0, 1.0, 1.0]),
            np.array([[0.1]]),
        )
        diagram = controller @ plant
        signals = resolve_plot_signals(diagram)
        self.assertEqual(signals[0], "x")
        self.assertTrue(signals[1].endswith(":u"))

    def test_resolve_plot_signals_series_into_closed_loop(self):
        from minilink.blocks.sources import Step
        from minilink.control.impedance import ImpedanceController
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        diagram = (
            Step(final_value=[1.0], step_time=0.0) >> ImpedanceController() @ Pendulum()
        )
        self.assertEqual(resolve_plot_signals(diagram), ("x", "ref:y", "ctl:u"))

    def test_resolve_plot_signals_dynamic_controller(self):
        from minilink.blocks.sources import Step
        from minilink.control.siso import FilteredController
        from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator

        diagram = (
            Step(final_value=[1.0], step_time=0.0)
            >> FilteredController() @ DoubleIntegrator()
        )
        self.assertEqual(resolve_plot_signals(diagram), ("x", "ref:y", "ctl:u"))

    def test_plot_trajectory_uses_default_signals_for_closed_loop(self):
        from minilink.graphical.common.environment import override_env

        override_env("jupyter")
        self.addCleanup(override_env, None)
        result = self.diagram.plot_trajectory(self.traj, show=False)
        self.assertIsNotNone(result.figure)
        plt.close(result.figure)

    def test_signal_spec_resolves_core_and_extra_signals(self):
        traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
            signals={"extra": np.ones((2, 2))},
        )
        spec = build_signal_plot_spec(self.sys, traj, signals=("x", "u", "extra"))
        labels = [trace.label for trace in spec.traces]
        self.assertIn("x[0]", labels)
        self.assertIn("u[0]", labels)
        self.assertIn("extra[0]", labels)
        self.assertIn("extra[1]", labels)

    def test_signal_spec_assigns_distinct_tier_colors(self):
        traj_plus = self.diagram.reconstruct_internal_signals(self.traj)
        spec = build_signal_plot_spec(
            self.diagram, traj_plus, signals=("x", "step:y", "ctl:u")
        )
        colors = {trace.signal: trace.color for trace in spec.traces}
        self.assertNotEqual(colors["x"], colors["step:y"])
        self.assertNotEqual(colors["step:y"], colors["ctl:u"])
        self.assertNotEqual(colors["x"], colors["ctl:u"])

    def test_unknown_signal_reports_available_names(self):
        with self.assertRaisesRegex(ValueError, "ctl:u"):
            build_signal_plot_spec(self.diagram, self.traj, signals=("missing",))

    def test_live_time_signal_plot_reuses_artists(self):
        traj0 = Trajectory(
            t=np.array([0.0, 1.0]), x=np.array([[0.0, 1.0]]), u=np.array([[1.0, 1.0]])
        )
        traj1 = Trajectory(
            t=np.array([0.0, 1.0]), x=np.array([[0.0, 0.5]]), u=np.array([[0.5, 0.5]])
        )
        handle = open_time_signal_plot(self.sys, traj0, signals=("x", "u"), show=False)
        try:
            fig_id = id(handle.fig)
            line_ids = [id(line) for line in handle.lines]
            handle.update(traj1)
            self.assertEqual(id(handle.fig), fig_id)
            self.assertEqual([id(line) for line in handle.lines], line_ids)
            np.testing.assert_allclose(
                handle.lines[0].get_ydata(), np.array([0.0, 0.5])
            )
        finally:
            handle.close()


from types import SimpleNamespace
from minilink.core.kinematics import identity, translation
from minilink.graphical.animation import Animator, make_renderer
from minilink.graphical.animation.primitives import Point, camera_matrix
from minilink.graphical.animation.renderers.plotly_renderer import (
    PlotlyRenderer,
    _import_plotly,
)
from minilink.graphical.catalog.skins import debug_state_skin
from minilink.graphical.common.plotly_style import (
    PLOTLY_ANIMATION_2D_MARGIN,
    PLOTLY_ANIMATION_HEIGHT,
    PLOTLY_FIG_WIDTH,
)
from minilink.graphical.signals import open_time_signal_plot, plot_time_signals


class Integrator_plotly_renderer(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


@pytest.mark.plotting
class TestPlotlyRendererOptionalImport(unittest.TestCase):
    def test_import_plotly_reports_extra_when_missing(self):
        try:
            go = _import_plotly()
        except ImportError as exc:
            self.assertIn("minilink[plotting]", str(exc))
        else:
            self.assertTrue(hasattr(go, "Figure"))

    def testmake_renderer_accepts_plotly(self):
        animator = Animator(DynamicSystem(1, output_dim=1, expose_state=True))
        backend = make_renderer("plotly", animator)
        self.assertIsInstance(backend, PlotlyRenderer)

    def test_plotly_is_not_interactive_loop_backend(self):
        sys = DynamicSystem(1, output_dim=1, expose_state=True)
        animator = Animator(sys)

        def update_callback(x, u, t, step_idx, events):
            return (x, u, True)

        with self.assertRaisesRegex(ValueError, "interactive loops"):
            animator.run_interactive(
                update_callback,
                x0=np.array([0.0]),
                renderer="plotly",
                show=False,
                max_steps=1,
            )
        with self.assertRaisesRegex(ValueError, "interactive loops"):
            animator.game(renderer="plotly", max_steps=1)


@pytest.mark.plotting
class TestPlotlyRenderer(unittest.TestCase):
    def setUp(self):
        pytest.importorskip("plotly")

    def test_static_2d_frame_builds_figure_without_showing(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        sys.skin = debug_state_skin
        animator = Animator(sys)
        backend = PlotlyRenderer(animator)
        x = np.array([0.5])
        u = np.array([1.0])
        frame = animator._resolve_frame(
            x, u, 0.0, kinematic=sys.get_kinematic_geometry()
        )
        backend.open_scene(
            is_3d=False, show=False, camera=frame["camera"], title="Plotly smoke"
        )
        backend.draw_frame(
            frame["primitives"], frame["transforms"], 0.0, frame["camera"]
        )
        fig = backend.present(block=False)
        self.assertEqual(len(fig.data), 2)
        self.assertEqual(fig.data[0].type, "scatter")
        self.assertEqual(fig.layout.width, PLOTLY_FIG_WIDTH)
        self.assertEqual(fig.layout.height, PLOTLY_ANIMATION_HEIGHT)
        self.assertEqual(tuple(fig.layout.xaxis.range), (-10.0, 10.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-10.0, 10.0))

    def test_static_xy_camera_keeps_geometry_in_world_coordinates(self):
        sys = DynamicSystem(0)
        animator = Animator(sys)
        backend = PlotlyRenderer(animator)
        camera = camera_matrix(target=(10.0, 3.0, 0.0), scale=4.0)
        backend.open_scene(
            is_3d=False, show=False, camera=camera, title="Plotly camera"
        )
        backend.draw_frame([Point()], [translation(10.0, 3.0, 0.0)], 0.0, camera)
        fig = backend.present(block=False)
        self.assertEqual(tuple(fig.layout.xaxis.range), (6.0, 14.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-1.0, 7.0))
        np.testing.assert_allclose(np.asarray(fig.data[0].x, dtype=float), [10.0])
        np.testing.assert_allclose(np.asarray(fig.data[0].y, dtype=float), [3.0])

    def test_static_3d_frame_builds_scatter3d(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        sys.skin = debug_state_skin
        animator = Animator(sys)
        backend = PlotlyRenderer(animator)
        frame = animator._resolve_frame(
            np.array([0.5]),
            np.array([1.0]),
            0.0,
            kinematic=sys.get_kinematic_geometry(),
        )
        backend.open_scene(
            is_3d=True, show=False, camera=frame["camera"], title="Plotly 3D smoke"
        )
        backend.draw_frame(
            frame["primitives"], frame["transforms"], 0.0, frame["camera"]
        )
        fig = backend.present(block=False)
        self.assertEqual(len(fig.data), 2)
        self.assertEqual(fig.data[0].type, "scatter3d")
        self.assertEqual(fig.layout.scene.aspectmode, "cube")

    def test_inline_animation_has_expected_frames(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        sys.skin = debug_state_skin
        traj = Trajectory(
            t=np.array([0.0, 0.1, 0.2]),
            x=np.array([[0.0, 0.1, 0.2]]),
            u=np.array([[1.0, 1.0, 1.0]]),
        )
        fig = Animator(sys).animate_simulation(
            traj, renderer="plotly", html=True, show=False
        )
        self.assertEqual(len(fig.data), 2)
        self.assertEqual(len(fig.frames), 3)
        self.assertEqual(len(fig.frames[0].data), 2)
        self.assertEqual(fig.layout.width, PLOTLY_FIG_WIDTH)
        self.assertEqual(fig.layout.height, PLOTLY_ANIMATION_HEIGHT)
        self.assertEqual(fig.layout.margin.b, PLOTLY_ANIMATION_2D_MARGIN["b"])

    def test_inline_animation_uses_fixed_camera_2d_axes(self):
        sys = DynamicSystem(1, output_dim=1, expose_state=True)
        sys.skin = debug_state_skin
        traj = Trajectory(
            t=np.array([0.0, 0.1, 0.2]),
            x=np.array([[0.0, 25.0, -5.0]]),
            u=np.zeros((0, 3)),
        )
        fig = Animator(sys).animate_simulation(
            traj, renderer="plotly", html=True, show=False
        )
        self.assertEqual(tuple(fig.layout.xaxis.range), (-10.0, 10.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-10.0, 10.0))
        self.assertFalse(fig.layout.xaxis.autorange)
        self.assertFalse(fig.layout.yaxis.autorange)
        self.assertIsNone(fig.frames[0].layout.xaxis.range)

    def test_inline_animation_updates_xy_camera_axis_ranges(self):
        sys = DynamicSystem(0)
        backend = PlotlyRenderer(Animator(sys))
        point = Point()
        frames = [
            {
                "t": 0.0,
                "primitives": [point],
                "transforms": [identity()],
                "camera": camera_matrix(target=(0.0, 0.0, 0.0), scale=2.0),
            },
            {
                "t": 0.1,
                "primitives": [point],
                "transforms": [translation(10.0, 3.0, 0.0)],
                "camera": camera_matrix(target=(10.0, 3.0, 0.0), scale=2.0),
            },
        ]
        fig = backend.render_inline_animation(
            [point], frames, SimpleNamespace(interval_ms=50), is_3d=False
        )
        self.assertEqual(tuple(fig.layout.xaxis.range), (-2.0, 2.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-2.0, 2.0))
        self.assertEqual(tuple(fig.frames[1].layout.xaxis.range), (8.0, 12.0))
        self.assertEqual(tuple(fig.frames[1].layout.yaxis.range), (1.0, 5.0))
        np.testing.assert_allclose(
            np.asarray(fig.frames[1].data[0].x, dtype=float), [10.0]
        )
        np.testing.assert_allclose(
            np.asarray(fig.frames[1].data[0].y, dtype=float), [3.0]
        )

    def test_plotly_native_false_html_false_raises(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        traj = Trajectory(
            t=np.array([0.0, 0.1]), x=np.array([[0.0, 0.1]]), u=np.array([[1.0, 1.0]])
        )
        with self.assertRaisesRegex(ValueError, "renderer='plotly'.*native=False"):
            Animator(sys).animate_simulation(
                traj, renderer="plotly", html=False, native=False, show=True
            )

    def test_plotly_native_false_html_true_allowed(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        sys.skin = debug_state_skin
        traj = Trajectory(
            t=np.array([0.0, 0.1]), x=np.array([[0.0, 0.1]]), u=np.array([[1.0, 1.0]])
        )
        fig = Animator(sys).animate_simulation(
            traj, renderer="plotly", html=True, native=False, show=False
        )
        self.assertEqual(len(fig.frames), 2)


@pytest.mark.plotting
class TestPlotlySignalPlot(unittest.TestCase):
    def test_plotly_static_and_live_update(self):
        pytest.importorskip("plotly")
        sys = Integrator_plotly_renderer()
        traj0 = Trajectory(
            t=np.array([0.0, 1.0]), x=np.array([[0.0, 1.0]]), u=np.array([[1.0, 1.0]])
        )
        traj1 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 0.25]]),
            u=np.array([[0.25, 0.25]]),
        )
        result = plot_time_signals(
            sys, traj0, signals=("x", "u"), backend="plotly", show=False
        )
        self.assertEqual(len(result.figure.data), 2)
        self.assertEqual(result.figure.layout.width, PLOTLY_FIG_WIDTH)
        handle = open_time_signal_plot(
            sys, traj0, signals=("x", "u"), backend="plotly", show=False
        )
        handle.update(traj1)
        np.testing.assert_allclose(handle.fig.data[0].y, np.array([0.0, 0.25]))

    def test_stacked_figsize_caps_height_for_popup_layout(self):
        from minilink.graphical.common.matplotlib_style import (
            SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP,
            SIGNAL_PLOT_ROW_HEIGHT,
            TRAJECTORY_MAX_FIG_HEIGHT_POPUP,
            TRAJECTORY_ROW_HEIGHT,
            signal_stack_figsize,
            trajectory_stack_figsize,
        )

        n = 20
        _, h_tall = trajectory_stack_figsize(n, allow_tall=True)
        self.assertEqual(h_tall, TRAJECTORY_ROW_HEIGHT * n)
        _, h_cap = trajectory_stack_figsize(n, allow_tall=False)
        self.assertEqual(h_cap, TRAJECTORY_MAX_FIG_HEIGHT_POPUP)
        _, h_sig = signal_stack_figsize(n, allow_tall=True)
        self.assertEqual(h_sig, SIGNAL_PLOT_ROW_HEIGHT * n)
        _, h_sig_cap = signal_stack_figsize(n, allow_tall=False)
        self.assertEqual(h_sig_cap, SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP)
