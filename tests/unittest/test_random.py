"""The counter-based draws: published cipher vectors, the JAX primitive, the normal law, the index traps."""

import numpy as np
import pytest

from minilink.core.distributions import (
    child_seed,
    random_bits,
    sample_index,
    standard_normal,
    threefry_2x32,
)


def test_threefry_matches_the_random123_vectors():
    key = np.array(
        [[0, 0xFFFFFFFF, 0x13198A2E], [0, 0xFFFFFFFF, 0x03707344]], dtype=np.uint32
    )
    count = np.array(
        [[0, 0xFFFFFFFF, 0x243F6A88], [0, 0xFFFFFFFF, 0x85A308D3]], dtype=np.uint32
    )
    x0, x1 = threefry_2x32((key[0], key[1]), (count[0], count[1]))
    assert x0.tolist() == [0x6B200159, 0x1CB996FC, 0xC4923A9C]
    assert x1.tolist() == [0x99BA4EFE, 0xBB002BE7, 0x483DF7A0]


@pytest.mark.optional
@pytest.mark.jax
def test_threefry_matches_the_jax_primitive():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp
    from jax.extend.random import threefry_2x32 as jax_threefry

    rng = np.random.default_rng(0)
    n = 64
    key = rng.integers(0, 2**32, size=2, dtype=np.uint32)
    count = rng.integers(0, 2**32, size=(2, n), dtype=np.uint32)
    ours_0, ours_1 = threefry_2x32((int(key[0]), int(key[1])), (count[0], count[1]))
    theirs = np.asarray(jax_threefry(jnp.asarray(key), jnp.concatenate(list(count))))
    np.testing.assert_array_equal(ours_0, theirs[:n])
    np.testing.assert_array_equal(ours_1, theirs[n:])
    assert jax is not None


@pytest.mark.optional
@pytest.mark.jax
def test_bits_and_normals_agree_on_numpy_and_jax():
    pytest.importorskip("jax")
    from minilink.core.backends import require_jax_numpy

    jnp = (
        require_jax_numpy()
    )  # the library's 64-bit default, as every evaluator binds it

    for k in (0, 123, -5):
        bits_np = random_bits(7, np.asarray(k), 5)
        bits_jx = random_bits(jnp.asarray(7), jnp.asarray(k), 5)
        np.testing.assert_array_equal(np.asarray(bits_jx[0]), bits_np[0])
        np.testing.assert_array_equal(np.asarray(bits_jx[1]), bits_np[1])
        np.testing.assert_array_equal(
            np.asarray(standard_normal(jnp.asarray(7), jnp.asarray(k), 5)),
            standard_normal(7, np.asarray(k), 5),
        )


def test_standard_normal_is_a_standard_normal():
    n = 100_000
    draws = np.array([standard_normal(3, np.asarray(k), 1)[0] for k in range(n)])
    assert abs(draws.mean()) < 3.0 / np.sqrt(n)
    assert abs(draws.var() - 1.0) < 3.0 * np.sqrt(2.0 / n)

    # consecutive samples, and channels of one sample, are uncorrelated
    lagged = np.corrcoef(draws[:-1], draws[1:])[0, 1]
    assert abs(lagged) < 3.0 / np.sqrt(n)
    channels = np.array([standard_normal(3, np.asarray(k), 2) for k in range(n)])
    assert abs(np.corrcoef(channels[:, 0], channels[:, 1])[0, 1]) < 3.0 / np.sqrt(n)


def test_a_different_seed_or_channel_count_is_a_different_stream():
    a = standard_normal(1, np.asarray(10), 3)
    b = standard_normal(2, np.asarray(10), 3)
    assert not np.allclose(a, b)
    np.testing.assert_array_equal(standard_normal(1, np.asarray(10), 2), a[:2])


@pytest.mark.parametrize("n", [1, 10])
def test_sample_index_survives_accumulated_time(n):
    period = 0.01
    dt = period / n
    steps = 2_000_000
    t = np.cumsum(np.full(steps, dt))  # t_k = t_{k-1} + dt, as the fixed-step loops do
    k = sample_index(t, period)
    expected = np.arange(1, steps + 1) // n
    np.testing.assert_array_equal(k, expected)


def test_sample_index_at_a_millisecond_period_and_at_negative_time():
    t = np.cumsum(np.full(200_000, 0.001))
    np.testing.assert_array_equal(sample_index(t, 0.001), np.arange(1, 200_001))
    assert int(sample_index(-0.5, 0.1)) == -5
    assert int(sample_index(0.0, 0.1)) == 0

    # a negative index wraps to the same block on both backends: the stream is periodic
    np.testing.assert_array_equal(
        standard_normal(4, np.asarray(-5), 2),
        standard_normal(4, np.asarray(2**32 - 5), 2),
    )


@pytest.mark.optional
@pytest.mark.jax
def test_sample_index_traces_under_jit():
    jax = pytest.importorskip("jax")
    from minilink.core.backends import require_jax

    require_jax()
    k = jax.jit(lambda t: sample_index(t, 0.01))(0.3)
    assert int(k) == 30


def test_child_seed_is_stable_and_distinct_by_name():
    assert child_seed(1, "plant") == child_seed(1, "plant")
    assert child_seed(1, "plant") != child_seed(1, "noise")
    assert child_seed(1, "plant") != child_seed(2, "plant")
    assert 0 <= child_seed(5, "seed") < 2**31


def test_the_held_train_reproduces_the_lyapunov_variance():
    # ẋ = −a x + w, w white of intensity W: the stationary variance solves 0 = −2 a P + W
    from minilink import WhiteNoise
    from minilink.dynamics.abstraction.state_space import LTISystem

    a, W, period = 1.0, 1.0, 0.01
    P = W / (2.0 * a)
    plant = LTISystem(np.array([[-a]]), np.array([[1.0]]))
    noise = WhiteNoise(1, psd=W, sample_period=period)
    loop = noise >> plant
    samples = []
    for seed in range(12):
        noise.params["seed"] = seed
        traj = loop.compute_trajectory(
            tf=40.0, dt=period, solver="rk4_fixedsteps", verbose=False
        )
        samples.append(
            traj.x[0, 500:]
        )  # after five time constants, at the sample instants
    variance = np.var(np.concatenate(samples))
    assert abs(variance / P - 1.0) < 0.15
