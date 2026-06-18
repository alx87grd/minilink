import unittest

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.graphical.animation.primitives import TrajectoryPolyline


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
        np.testing.assert_allclose(pts_late[-1, :2], [2.0, 0.1])


if __name__ == "__main__":
    unittest.main()
