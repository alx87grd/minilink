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

    def test_euler_option_reproduces_the_explicit_formula(self):
        env = self.make_env(reset_mode="determinist", integrator="euler")
        env.reset()
        x, t = env.x.copy(), env.t
        u = np.array([0.0])
        expected = x + env.sys.f(x, u, t) * env.dt
        env.step(u)
        np.testing.assert_allclose(env.x, expected)

    def test_default_step_is_rk4_on_the_compiled_plant(self):
        env = self.make_env(reset_mode="determinist", compile_backend="numpy")
        self.assertEqual(env.integrator, "rk4")
        self.assertEqual(env.compile_backend, "numpy")
        env.reset()
        x, t, dt = env.x.copy(), env.t, env.dt
        u = np.array([0.4])
        f = env.sys.f
        k1 = f(x, u, t)
        k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = f(x + dt * k3, u, t + dt)
        env.step(u)
        np.testing.assert_allclose(env.x, x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4))

    def test_rk4_and_euler_agree_at_small_dt(self):
        from minilink.interfaces.gymnasium import Sys2Gym

        plant = make_bounded_pendulum()
        cost = QuadraticCost.from_system(plant, xbar=np.zeros(2))
        envs = [
            Sys2Gym(
                plant,
                cost,
                dt=1e-3,
                tf=1.0,
                reset_mode="determinist",
                integrator=name,
                compile_backend="numpy",
            )
            for name in ("rk4", "euler")
        ]
        for env in envs:
            env.reset()
        rng = np.random.default_rng(0)
        for _ in range(200):
            u = rng.uniform(-1.0, 1.0, size=1)
            for env in envs:
                env.step(u)
        np.testing.assert_allclose(envs[0].x, envs[1].x, atol=1e-4)
        self.assertGreater(np.abs(envs[0].x - envs[1].x).max(), 0.0)

    def test_unknown_integrator_is_rejected(self):
        with self.assertRaises(ValueError):
            self.make_env(integrator="rk45")

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
@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(GYMNASIUM_AVAILABLE, "gymnasium not installed")
class TestSys2GymJax(unittest.TestCase):
    def setUp(self):
        pytest.importorskip("jax")

    def _envs(self, **kwargs):
        from minilink.interfaces.gymnasium import Sys2Gym

        plant = make_bounded_pendulum()
        cost = QuadraticCost.from_system(plant, xbar=np.zeros(2))
        return Sys2Gym(plant, cost, dt=0.05, tf=1.0, reset_mode="determinist", **kwargs)

    def test_jax_is_picked_when_the_plant_traces(self):
        env = self._envs()
        self.assertEqual(env.compile_backend, "jax")

    def test_jax_step_matches_numpy_step(self):
        env_jax, env_np = (
            self._envs(compile_backend="jax"),
            self._envs(compile_backend="numpy"),
        )
        env_jax.reset()
        env_np.reset()
        for u in (np.array([0.3]), np.array([-0.7]), np.array([0.0])):
            env_jax.step(u)
            env_np.step(u)
        np.testing.assert_allclose(env_jax.x, env_np.x, rtol=1e-10, atol=1e-12)
        self.assertEqual(env_jax.x.dtype, np.float64)

    def test_untraceable_plant_falls_back_to_numpy(self):
        from minilink.core.system import DynamicSystem
        from minilink.interfaces.gymnasium import Sys2Gym

        class Branching(DynamicSystem):
            """A Python branch on the state value: NumPy only."""

            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1)
                self.state.lower_bound = np.array([-5.0])
                self.state.upper_bound = np.array([5.0])
                self.inputs["u"].lower_bound = np.array([-1.0])
                self.inputs["u"].upper_bound = np.array([1.0])

            def f(self, x, u, t=0, params=None):
                gain = 2.0 if x[0] > 0.0 else 1.0
                return np.array([-gain * x[0] + u[0]])

        plant = Branching()
        cost = QuadraticCost.from_system(plant, xbar=np.zeros(plant.n))
        env = Sys2Gym(plant, cost, dt=0.05, tf=1.0)
        self.assertEqual(env.compile_backend, "numpy")
        env.reset()
        env.step(np.zeros(plant.m))
        with self.assertRaises(RuntimeError):
            Sys2Gym(plant, cost, dt=0.05, tf=1.0, compile_backend="jax")


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
