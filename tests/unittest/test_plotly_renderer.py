import unittest

import numpy as np
import pytest

from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.graphical.animation import Animator, _make_renderer
from minilink.graphical.renderers.plotly_renderer import PlotlyRenderer, _import_plotly


class TestPlotlyRendererOptionalImport(unittest.TestCase):
    def test_import_plotly_reports_extra_when_missing(self):
        try:
            go = _import_plotly()
        except ImportError as exc:
            self.assertIn("minilink[plotting]", str(exc))
        else:
            self.assertTrue(hasattr(go, "Figure"))

    def test_make_renderer_accepts_plotly(self):
        animator = Animator(DynamicSystem(1, 0, 1))
        backend = _make_renderer("plotly", animator)
        self.assertIsInstance(backend, PlotlyRenderer)

    def test_plotly_is_not_interactive_loop_backend(self):
        sys = DynamicSystem(1, 0, 1)
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


@pytest.mark.optional
class TestPlotlyRenderer(unittest.TestCase):
    def setUp(self):
        pytest.importorskip("plotly")

    def test_static_2d_frame_builds_figure_without_showing(self):
        sys = DynamicSystem(1, 1, 1)
        animator = Animator(sys)
        backend = PlotlyRenderer(animator)
        x = np.array([0.5])
        u = np.array([1.0])
        frame = animator._prepare_transforms(x, u, 0.0)

        backend.open_scene(
            is_3d=False,
            show=False,
            camera=frame["camera"],
            title="Plotly smoke",
        )
        backend.draw_frame(
            sys.get_kinematic_geometry(),
            frame["transforms"],
            0.0,
            frame["camera"],
        )
        fig = backend.present(block=False)

        self.assertEqual(len(fig.data), 2)
        self.assertEqual(fig.data[0].type, "scatter")
        self.assertEqual(tuple(fig.layout.xaxis.range), (-10.0, 10.0))
        self.assertEqual(tuple(fig.layout.yaxis.range), (-10.0, 10.0))

    def test_static_3d_frame_builds_scatter3d(self):
        sys = DynamicSystem(1, 1, 1)
        animator = Animator(sys)
        backend = PlotlyRenderer(animator)
        frame = animator._prepare_transforms(np.array([0.5]), np.array([1.0]), 0.0)

        backend.open_scene(
            is_3d=True,
            show=False,
            camera=frame["camera"],
            title="Plotly 3D smoke",
        )
        backend.draw_frame(
            sys.get_kinematic_geometry(),
            frame["transforms"],
            0.0,
            frame["camera"],
        )
        fig = backend.present(block=False)

        self.assertEqual(len(fig.data), 2)
        self.assertEqual(fig.data[0].type, "scatter3d")
        self.assertEqual(fig.layout.scene.aspectmode, "cube")

    def test_inline_animation_has_expected_frames(self):
        sys = DynamicSystem(1, 1, 1)
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

    def test_inline_animation_uses_fixed_camera_2d_axes(self):
        sys = DynamicSystem(1, 0, 1)
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


if __name__ == "__main__":
    unittest.main()
