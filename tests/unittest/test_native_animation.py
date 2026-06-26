"""Regression test: native matplotlib animation uses per-frame primitives."""

from __future__ import annotations

import unittest

import matplotlib

matplotlib.use("Agg")
import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.animation import Animator
from minilink.graphical.animation.renderers.matplotlib_renderer import MatplotlibRenderer
from minilink.graphical.animation.renderers.timing import trajectory_frame_schedule


class TestNativeAnimationDrawList(unittest.TestCase):
    def test_matplotlib_funcanimation_uses_frame_primitives(self):
        sys = DynamicBicycle()
        t = np.linspace(0.0, 1.0, 3)
        x = np.zeros((6, 3))
        x[0] = np.linspace(0.0, 2.0, 3)
        x[3] = 2.0
        traj = Trajectory(t=t, x=x, u=np.zeros((2, 3)))
        anim = Animator(sys)
        static = sys.get_kinematic_geometry()
        schedule = trajectory_frame_schedule(traj, 1.0)
        frames = [
            anim._prepare_frame(traj, i, schedule, static_geometry=static)
            for i in range(schedule.n_frames)
        ]
        backend = MatplotlibRenderer(anim)
        fig, ani = backend._build_animation(static, frames, schedule)
        artists = ani._func(0)
        self.assertGreater(len(artists), 0)
        self.assertEqual(len(artists), len(frames[0]["primitives"]))

        import matplotlib.pyplot as plt

        plt.close(fig)


if __name__ == "__main__":
    unittest.main()
