import unittest

import numpy as np
import pytest

try:
    import gymnasium  # noqa: F401

    GYMNASIUM_AVAILABLE = True
except ImportError:
    GYMNASIUM_AVAILABLE = False

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


def make_bounded_pendulum():
    plant = Pendulum()
    plant.state.lower_bound = np.array([-2.0 * np.pi, -12.0])
    plant.state.upper_bound = np.array([+2.0 * np.pi, +12.0])
    plant.inputs["u"].lower_bound = np.array([-1.0])
    plant.inputs["u"].upper_bound = np.array([+1.0])
    plant.x0 = np.array([-np.pi, 0.0])
    return plant


class _ConstantPolicy:
    """Duck-typed stand-in for a stable-baselines3 model."""

    class _Space:
        def __init__(self, shape):
            self.shape = shape

    def __init__(self, n, m, value=0.5):
        self.observation_space = self._Space((n,))
        self.action_space = self._Space((m,))
        self.value = value

    def predict(self, obs, deterministic=True):
        return np.full(self.action_space.shape[0], self.value), None


@pytest.mark.rl
@unittest.skipUnless(GYMNASIUM_AVAILABLE, "gymnasium not installed")
class TestSys2Gym(unittest.TestCase):
    def make_env(self, **kwargs):
        from minilink.interfaces.gymnasium import Sys2Gym

        plant = make_bounded_pendulum()
        cost = QuadraticCost.from_system(plant, xbar=np.zeros(2))
        return Sys2Gym(plant, cost, dt=0.05, tf=1.0, **kwargs)

    def test_env_checker(self):
        from gymnasium.utils.env_checker import check_env

        check_env(self.make_env(), skip_render_check=True)

    def test_spaces_match_bounds(self):
        env = self.make_env()
        np.testing.assert_allclose(env.observation_space.low, [-2.0 * np.pi, -12.0])
        np.testing.assert_allclose(env.observation_space.high, [+2.0 * np.pi, +12.0])
        np.testing.assert_allclose(env.action_space.low, [-1.0])
        np.testing.assert_allclose(env.action_space.high, [+1.0])

    def test_reward_is_negative_cost_increment(self):
        env = self.make_env(reset_mode="determinist")
        y, _ = env.reset()
        x, t, dt = env.x.copy(), env.t, env.dt
        u = np.array([0.3])
        _, r, _, _, _ = env.step(u)
        expected = -float(env.cost.g(x, u, t)) * dt
        self.assertAlmostEqual(r, expected)

    def test_step_integrates_euler(self):
        env = self.make_env(reset_mode="determinist")
        env.reset()
        x, t = env.x.copy(), env.t
        u = np.array([0.0])
        expected = x + env.sys.f(x, u, t) * env.dt
        env.step(u)
        np.testing.assert_allclose(env.x, expected)

    def test_truncates_on_horizon(self):
        env = self.make_env(reset_mode="determinist")
        env.reset()
        truncated = False
        for _ in range(30):  # tf=1.0, dt=0.05 -> truncation within 21 steps
            _, _, _, truncated, _ = env.step(np.array([0.0]))
            if truncated:
                break
        self.assertTrue(truncated)

    def test_input_clipping(self):
        env = self.make_env(reset_mode="determinist")
        env.reset()
        env.step(np.array([100.0]))
        self.assertLessEqual(float(env.u[0]), 1.0)

    def test_reset_modes(self):
        for mode in ("uniform", "gaussian", "determinist"):
            env = self.make_env(reset_mode=mode)
            y, info = env.reset(seed=1)
            self.assertEqual(y.shape, (2,))
            self.assertIn("state", info)


@pytest.mark.rl
@unittest.skipUnless(GYMNASIUM_AVAILABLE, "gymnasium not installed")
class TestSB3Controller(unittest.TestCase):
    def test_closed_loop_composition(self):
        from minilink.interfaces.gymnasium import SB3Controller

        plant = make_bounded_pendulum()
        ctl = SB3Controller(_ConstantPolicy(n=2, m=1, value=0.2))

        diagram = ctl @ plant
        self.assertEqual(diagram.connections["ctl"]["x"], ("sys", "x"))
        self.assertEqual(diagram.connections["sys"]["u"], ("ctl", "u"))

        traj = diagram.compute_trajectory(tf=0.5, n_steps=51, verbose=False)
        self.assertEqual(traj.x.shape[0], 2)
        np.testing.assert_allclose(ctl.action(np.zeros(2)), [0.2])

    def test_plot_control_law(self):
        import matplotlib.pyplot as plt

        from minilink.interfaces.gymnasium import SB3Controller

        plant = make_bounded_pendulum()
        ctl = SB3Controller(_ConstantPolicy(n=2, m=1, value=0.2), sys=plant)
        res = ctl.plot_control_law(grid_shape=(5, 5), show=False)
        mesh = res.axes.collections[0]
        np.testing.assert_allclose(mesh.get_array(), 0.2)
        self.assertIn(plant.state.labels[0], res.axes.get_xlabel())
        plt.close(res.figure)


if __name__ == "__main__":
    unittest.main()
