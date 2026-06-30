import unittest
from types import SimpleNamespace

import numpy as np
import pytest

from minilink.core.kinematics import identity, translation
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.graphical.animation import Animator, make_renderer
from minilink.graphical.animation.primitives import (
    Point,
    camera_matrix,
)
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
from minilink.graphical.signals import (
    open_time_signal_plot,
    plot_time_signals,
)


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


@pytest.mark.optional
@pytest.mark.visualization
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
            return x, u, True

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

    def test_game_requires_interactive_renderer(self):
        sys = DynamicSystem(1, output_dim=1, expose_state=True)
        animator = Animator(sys)
        self.assertTrue(hasattr(animator, "game"))
        with self.assertRaisesRegex(ValueError, "interactive loops"):
            animator.game(renderer="plotly", max_steps=1)


@pytest.mark.optional
@pytest.mark.visualization
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
            is_3d=False,
            show=False,
            camera=frame["camera"],
            title="Plotly smoke",
        )
        backend.draw_frame(
            frame["primitives"],
            frame["transforms"],
            0.0,
            frame["camera"],
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
            is_3d=False,
            show=False,
            camera=camera,
            title="Plotly camera",
        )
        backend.draw_frame(
            [Point()],
            [translation(10.0, 3.0, 0.0)],
            0.0,
            camera,
        )
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
            is_3d=True,
            show=False,
            camera=frame["camera"],
            title="Plotly 3D smoke",
        )
        backend.draw_frame(
            frame["primitives"],
            frame["transforms"],
            0.0,
            frame["camera"],
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
            traj,
            renderer="plotly",
            html=True,
            show=False,
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
            traj,
            renderer="plotly",
            html=True,
            show=False,
        )

        self.assertEqual(tuple(fig.layout.xaxis.range), (-10.0, 10.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-10.0, 10.0))
        self.assertFalse(fig.layout.xaxis.autorange)
        self.assertFalse(fig.layout.yaxis.autorange)
        self.assertIsNone(fig.frames[0].layout.xaxis.range)

    def test_inline_animation_updates_xy_camera_axis_ranges(self):
        sys = DynamicSystem(0)
        backend = PlotlyRenderer(Animator(sys))
        frames = [
            {
                "t": 0.0,
                "transforms": [identity()],
                "camera": camera_matrix(target=(0.0, 0.0, 0.0), scale=2.0),
            },
            {
                "t": 0.1,
                "transforms": [translation(10.0, 3.0, 0.0)],
                "camera": camera_matrix(target=(10.0, 3.0, 0.0), scale=2.0),
            },
        ]

        fig = backend.render_inline_animation(
            [Point()],
            frames,
            SimpleNamespace(interval_ms=50),
            is_3d=False,
        )

        self.assertEqual(tuple(fig.layout.xaxis.range), (-2.0, 2.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-2.0, 2.0))
        self.assertEqual(tuple(fig.frames[1].layout.xaxis.range), (8.0, 12.0))
        self.assertEqual(tuple(fig.frames[1].layout.yaxis.range), (1.0, 5.0))
        np.testing.assert_allclose(
            np.asarray(fig.frames[1].data[0].x, dtype=float),
            [10.0],
        )
        np.testing.assert_allclose(
            np.asarray(fig.frames[1].data[0].y, dtype=float),
            [3.0],
        )

    def test_plotly_native_false_html_false_raises(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        traj = Trajectory(
            t=np.array([0.0, 0.1]),
            x=np.array([[0.0, 0.1]]),
            u=np.array([[1.0, 1.0]]),
        )
        with self.assertRaisesRegex(ValueError, "renderer='plotly'.*native=False"):
            Animator(sys).animate_simulation(
                traj,
                renderer="plotly",
                html=False,
                native=False,
                show=True,
            )

    def test_plotly_native_false_html_true_allowed(self):
        sys = DynamicSystem(1, input_dim=1, output_dim=1, expose_state=True)
        sys.skin = debug_state_skin
        traj = Trajectory(
            t=np.array([0.0, 0.1]),
            x=np.array([[0.0, 0.1]]),
            u=np.array([[1.0, 1.0]]),
        )
        fig = Animator(sys).animate_simulation(
            traj,
            renderer="plotly",
            html=True,
            native=False,
            show=False,
        )
        self.assertEqual(len(fig.frames), 2)


@pytest.mark.optional
@pytest.mark.visualization
class TestPlotlySignalPlot(unittest.TestCase):
    def test_plotly_static_and_live_update(self):
        pytest.importorskip("plotly")

        sys = Integrator()
        traj0 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
        )
        traj1 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 0.25]]),
            u=np.array([[0.25, 0.25]]),
        )

        result = plot_time_signals(
            sys,
            traj0,
            signals=("x", "u"),
            backend="plotly",
            show=False,
        )
        self.assertEqual(len(result.figure.data), 2)
        self.assertEqual(result.figure.layout.width, PLOTLY_FIG_WIDTH)

        handle = open_time_signal_plot(
            sys,
            traj0,
            signals=("x", "u"),
            backend="plotly",
            show=False,
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


if __name__ == "__main__":
    unittest.main()
