"""The seeded stochastic policy: a law plus its own counter-based noise, a pure block."""

import numpy as np
import pytest
from scipy import stats

from minilink import Gain, Pendulum, PlanningProblem, QuadraticCost, Uniform
from minilink.control import GaussianPolicy, SquashedGaussianPolicy, StateFeedback

DT = 0.05


def gain():
    """``μ(x) = K x``, one input, one output."""
    return Gain(np.array([[2.0, 1.0]]))


def policy(seed=3):
    return GaussianPolicy(gain(), np.log([0.5]), sample_period=DT, seed=seed)


def test_no_seed_is_the_mean_law():
    law = gain()
    pi = GaussianPolicy(law, np.log([0.5]), sample_period=DT)
    x = np.array([0.3, -0.2])
    np.testing.assert_array_equal(
        pi.h(None, x, 0.12), law.outputs["y"].compute(None, x, 0.12)
    )
    assert set(pi.params) == {"mean", "log_std", "seed", "sample_period"}
    assert list(pi.inputs) == list(law.inputs)


def test_a_draw_is_held_over_the_period_and_scaled_by_the_spread():
    pi = policy()
    x = np.array([0.3, -0.2])
    mu = gain().outputs["y"].compute(None, x, 0.0)
    np.testing.assert_array_equal(pi.h(None, x, 0.051), pi.h(None, x, 0.099))
    assert not np.array_equal(pi.h(None, x, 0.049), pi.h(None, x, 0.051))

    # u − μ = σ ε: doubling σ doubles the step from the mean, the same ε
    wide = dict(pi.params, log_std=np.log([1.0]))
    np.testing.assert_allclose(
        pi.h(None, x, 0.07, wide) - mu, 2.0 * (pi.h(None, x, 0.07) - mu)
    )


def test_the_density_and_the_entropy_are_the_gaussians():
    pi = policy()
    x = np.array([0.3, -0.2])
    mu = gain().outputs["y"].compute(None, x, 0.0)
    a = np.array([0.4])
    expected = stats.norm(mu, 0.5).logpdf(a).sum()
    np.testing.assert_allclose(pi.log_prob(x, a), expected, rtol=1e-12)
    np.testing.assert_allclose(
        pi.entropy(x), stats.norm(0.0, 0.5).entropy(), rtol=1e-12
    )


def test_a_squashed_law_stays_in_its_box_and_its_density_integrates_to_one():
    pi = SquashedGaussianPolicy(
        gain(), np.log([0.8]), lower=[-2.0], upper=[1.0], sample_period=DT, seed=5
    )
    x = np.array([0.3, -0.2])
    draws = np.array([pi.h(None, x, DT * k + 0.01) for k in range(200)])
    assert np.all((draws > -2.0) & (draws < 1.0))

    grid = np.linspace(-2.0 + 1e-4, 1.0 - 1e-4, 20001)
    density = np.exp([pi.log_prob(x, np.array([a])) for a in grid])
    np.testing.assert_allclose(np.trapezoid(density, grid), 1.0, atol=2e-3)
    with pytest.raises(NotImplementedError):
        pi.entropy(x)


@pytest.mark.jax
def test_a_policy_vmaps_over_seeds_on_both_backends():
    pytest.importorskip("jax")
    from minilink.core.backends import ensure_jax_x64

    jax = ensure_jax_x64()
    jnp = jax.numpy
    pi = policy()
    x = np.array([0.3, -0.2])
    seeds = np.arange(8)

    on_numpy = np.array([pi.h(None, x, 0.07, dict(pi.params, seed=s)) for s in seeds])
    on_jax = jax.vmap(
        lambda s: pi.h(jnp.zeros(0), jnp.asarray(x), 0.07, dict(pi.params, seed=s))
    )(jnp.asarray(seeds))
    np.testing.assert_array_equal(np.asarray(on_jax), on_numpy)
    assert len(set(on_numpy.ravel())) == 8

    # The pathwise gradient through the fixed ε reaches the mean's gain
    def u_of(K):
        params = dict(pi.params, mean=dict(pi.params["mean"], K=K))
        return jnp.sum(pi.h(jnp.zeros(0), jnp.asarray(x), 0.07, params))

    grad = jax.grad(u_of)(jnp.asarray(pi.params["mean"]["K"]))
    np.testing.assert_allclose(np.asarray(grad), np.atleast_2d(x), rtol=1e-12)


def test_an_evaluation_draws_the_policys_seed_per_trial():
    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        x_start=Uniform([0.1, -0.1], [0.3, 0.1]),
        tf=1.0,
    )
    feedback = StateFeedback(np.array([[2.0, 1.0]]))
    pi = GaussianPolicy(feedback, np.log([0.5]), sample_period=DT, seed=3)
    runs = problem.compile(pi, dt=DT, backend="numpy").evaluate(n_trials=3, key=0)
    mean_only = problem.compile(feedback, dt=DT, backend="numpy")
    assert pi.realize(1)["seed"] != pi.realize(2)["seed"]
    assert not np.allclose(runs.J, mean_only.evaluate(n_trials=3, key=0).J)
