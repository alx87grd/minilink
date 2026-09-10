"""Smoke tests for the pure-JAX PPO prototype (research lane)."""

import numpy as np
import pytest

from minilink import CostFunction, Drone2D

pytest.importorskip("jax")
from minilink.experimental.ppo_jax import PPO  # noqa: E402


class HoverCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        return x @ x + 0.001 * (u @ u)

    def h(self, x, t=0.0, params=None):
        return 0.0


def bounded_drone():
    plant = Drone2D()
    plant.state.upper_bound = np.array([10, 10, 2 * np.pi, 10, 10, 10.0])
    plant.state.lower_bound = -plant.state.upper_bound
    plant.inputs["u"].lower_bound = np.array([0.0, 0.0])
    plant.inputs["u"].upper_bound = np.array([10.0, 10.0])
    return plant


@pytest.mark.optional
@pytest.mark.jax
def test_learn_runs_and_controller_closes_the_loop():
    plant = bounded_drone()
    ppo = PPO(plant, HoverCost(), n_steps=128, batch_size=32, n_envs=2, verbose=0)
    ppo.learn(256)

    assert ppo.num_timesteps == 256
    assert len(ppo.history) == 1
    assert np.isfinite(ppo.history[0]["policy_loss"])

    # Policy samples respect the input bounds (clipped deterministic action)
    u = ppo.controller.action(np.zeros(6))
    assert u.shape == (2,)
    assert np.all(u >= 0.0) and np.all(u <= 10.0)

    # Batch predict matches the single-state action
    u_batch, _ = ppo.predict(np.zeros((3, 6)))
    assert u_batch.shape == (3, 2)
    np.testing.assert_allclose(u_batch[0], u, rtol=1e-6)

    # The learned policy composes as a state-feedback block on both backends
    cl_sys = ppo.controller @ plant
    traj = cl_sys.compute_trajectory(tf=0.2, dt=0.1, verbose=False)
    assert np.all(np.isfinite(traj.x))
    evaluator = cl_sys.compile(backend="jax", verbose=False)
    dx = np.asarray(evaluator.f(np.zeros(6), np.zeros(0), 0.0))
    assert dx.shape == (6,)


@pytest.mark.optional
@pytest.mark.jax
def test_batch_size_must_divide_the_rollout():
    with pytest.raises(ValueError):
        PPO(bounded_drone(), HoverCost(), n_steps=100, batch_size=64, verbose=0)
