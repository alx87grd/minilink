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


# --- realizations: one key per experiment, the mean for analysis ---


def noisy_loop(seed_w=1, seed_v=2):
    from minilink import (
        DiagramSystem,
        Impedance,
        NoisyPendulum,
        Step,
        WhiteNoise,
    )

    loop = DiagramSystem()
    loop.add_subsystem(Step(final_value=1.0, step_time=10.0), "step")
    loop.add_subsystem(Impedance(Kp=100.0, Kd=50.0), "controller")
    loop.add_subsystem(NoisyPendulum(), "plant")
    loop.add_subsystem(WhiteNoise(seed=seed_w), "process_noise")
    loop.add_subsystem(WhiteNoise(seed=seed_v), "measurement_noise")
    loop.connect("step", "y", "controller", "r")
    loop.connect("controller", "u", "plant", "u")
    loop.connect("plant", "y", "controller", "y")
    loop.connect("process_noise", "y", "plant", "d")
    loop.connect("measurement_noise", "y", "plant", "n")
    return loop


def noisy_plant():
    """A pendulum with white noise on its sensor and its action port exported."""
    from minilink import DiagramSystem, NoisyPendulum, WhiteNoise

    plant = DiagramSystem()
    plant.add_subsystem(NoisyPendulum(), "plant")
    plant.add_subsystem(WhiteNoise(seed=5), "sensor_noise")
    plant.connect("sensor_noise", "y", "plant", "n")
    plant.add_input_port("u", dim=1)
    plant.connect("input", "u", "plant", "u")
    return plant


def test_realize_none_is_the_mean_nested_like_params():
    loop = noisy_loop()
    nominal = loop.realize(None)
    assert loop.is_random and not loop.subsystems["plant"].is_random
    assert set(nominal) == set(loop.params)
    assert nominal["process_noise"]["seed"] is None
    assert nominal["measurement_noise"]["seed"] is None
    assert (
        nominal["plant"] is loop.subsystems["plant"].params
    )  # untouched, by reference
    assert loop.params["process_noise"]["seed"] == 1  # the block's own seed stays


def test_realize_key_names_each_stream_so_an_added_block_moves_no_other():
    loop = noisy_loop()
    drawn = loop.realize(3)
    seeds = {drawn["process_noise"]["seed"], drawn["measurement_noise"]["seed"]}
    assert len(seeds) == 2 and None not in seeds
    assert drawn == loop.realize(3)
    assert drawn["process_noise"]["seed"] != loop.realize(4)["process_noise"]["seed"]

    from minilink import WhiteNoise

    loop.add_subsystem(WhiteNoise(seed=9), "another")
    again = loop.realize(3)
    assert again["process_noise"] == drawn["process_noise"]
    assert again["measurement_noise"] == drawn["measurement_noise"]

    loop.params = again  # a realization is assignable, like params
    assert (
        loop.subsystems["process_noise"].params["seed"]
        == again["process_noise"]["seed"]
    )


def test_analysis_verbs_see_the_noise_at_its_mean():
    from minilink.analysis import find_equilibrium, jacobian
    from minilink.control.design import lqr

    noisy, quiet = noisy_loop(), noisy_loop(None, None)
    A = jacobian(noisy, "f", "x", method="fd")
    np.testing.assert_allclose(A, jacobian(quiet, "f", "x", method="fd"))
    assert np.all(np.isfinite(A))

    x_eq = find_equilibrium(noisy, noisy.x0)
    np.testing.assert_allclose(x_eq, find_equilibrium(quiet, quiet.x0))

    plant = noisy_plant()
    assert plant.is_random
    K = lqr(plant, np.eye(2), np.eye(1), x_bar=np.zeros(2), method="fd")
    assert np.all(np.isfinite(K.params["K"]))


def test_a_problem_on_a_random_system_plans_against_the_mean():
    from minilink import Pendulum
    from minilink.planning.problems import PlanningProblem, nominal_plant_params

    noisy = noisy_loop()
    problem = PlanningProblem(noisy, x_start=noisy.x0, U=noisy.input_box)
    nominal = nominal_plant_params(problem)
    assert nominal["process_noise"]["seed"] is None
    assert nominal["plant"] is noisy.subsystems["plant"].params

    quiet = PlanningProblem(Pendulum(), x_start=np.zeros(2))
    assert nominal_plant_params(quiet) is None


# --- the two warnings: a shared seed, noise the stepped tools never see ---


def test_compile_warns_when_two_random_blocks_share_a_seed():
    import warnings

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        noisy_loop(1, 1).compile(backend="numpy", verbose=False)
    assert any("share seed 1" in str(w.message) for w in caught)

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        noisy_loop(1, 2).compile(backend="numpy", verbose=False)
    assert not any("share seed" in str(w.message) for w in caught)


def test_the_rollout_environment_warns_about_noise_it_never_sees():
    import warnings

    from minilink import NoisyPendulum
    from minilink.blocks.sources import WhiteNoise
    from minilink.core.costs import QuadraticCost
    from minilink.core.distributions import Uniform
    from minilink.learning.reinforcement_learning.environment import RolloutEnvironment
    from minilink.planning.problems import PlanningProblem

    def problem(sys, **disturbances):
        return PlanningProblem(
            sys,
            cost=QuadraticCost.from_system(sys),
            tf=1.0,
            x_start=Uniform([-0.5, -0.5], [0.5, 0.5]),
            disturbances=disturbances or None,
            U=sys.input_box,
        )

    cases = [
        (problem(NoisyPendulum(), d=WhiteNoise(1, psd=0.01, sample_period=0.1)), None),
        (
            problem(NoisyPendulum(), n=WhiteNoise(1, psd=0.01, sample_period=0.1)),
            "never calls h",
        ),
        (problem(noisy_plant()), "fixed seed"),
    ]
    for prob, expected in cases:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            RolloutEnvironment(prob, dt=0.1, backend="numpy")
        messages = [str(w.message) for w in caught]
        if expected is None:
            assert not any("never calls h" in m or "fixed seed" in m for m in messages)
        else:
            assert any(expected in m for m in messages), messages


def test_a_renamed_noise_source_keeps_its_dimension():
    from minilink.blocks.sources import WhiteNoise

    noise = WhiteNoise(4, psd=1e-4, port="n")
    assert list(noise.outputs) == ["n"] and noise.dim == 4
    assert noise.h(np.array([]), np.array([]), 0.3).shape == (4,)
    assert noise.compile().outputs(np.array([]), np.array([]), 0.3)["n"].shape == (4,)


def test_a_distribution_is_dim_mean_and_sample_and_a_user_law_drives_a_problem():
    from minilink import Gaussian, PlanningProblem, QuadraticCost, Uniform
    from minilink.core import Distribution
    from minilink.dynamics.catalog.equations import DoubleIntegrator

    # The library laws are Distributions: their draws have the shape (n, dim)
    for law in (Gaussian([0.0, 1.0], 0.1), Uniform([-1.0, -1.0], [1.0, 1.0])):
        assert isinstance(law, Distribution) and law.dim == 2
        assert np.shape(law.sample(0, 5)) == (5, 2) and np.shape(law.mean) == (2,)

    # A user law: the two-method surface, nothing else
    class Corners(Distribution):
        dim = 2
        mean = np.zeros(2)
        support = None

        def sample(self, key, n=None):
            rng = np.random.default_rng(key)
            draws = rng.choice([-1.0, 1.0], size=(1 if n is None else n, 2))
            return draws[0] if n is None else draws

    plant = DoubleIntegrator()
    problem = PlanningProblem(
        plant, cost=QuadraticCost.from_system(plant), x_start=Corners()
    )
    x0s = problem.realize(0, 4)["x_start"]
    assert x0s.shape == (4, 2) and set(np.abs(x0s).ravel()) == {1.0}
    assert isinstance(problem.nominal(), PlanningProblem)


def test_a_law_reads_its_numbers_from_params():
    from minilink import Gaussian, Uniform
    from minilink.core.distributions import Particles

    law = Gaussian([0.0, 1.0], [1.0, 0.5])
    assert set(law.params) == {"mean", "std"}
    np.testing.assert_array_equal(law.cov, np.diag([1.0, 0.25]))
    wider = {"mean": law.mean, "std": 2.0 * law.std}
    np.testing.assert_allclose(
        law.sample(3, 4, wider) - law.mean, 2.0 * (law.sample(3, 4) - law.mean)
    )

    np.testing.assert_allclose(Uniform([0.0], [1.0]).cov, [[1.0 / 12.0]])
    points = np.array([[0.0, 0.0], [2.0, 4.0]])
    np.testing.assert_allclose(Particles(points).cov, [[1.0, 2.0], [2.0, 4.0]])


def test_a_full_covariance_is_drawn_through_its_cholesky_factor():
    from minilink import Gaussian

    cov = np.array([[1.0, 0.8], [0.8, 1.0]])
    law = Gaussian(np.zeros(2), cov=cov)
    assert set(law.params) == {"mean", "cov"}
    draws = law.sample(0, 20000)
    np.testing.assert_allclose(np.cov(draws, rowvar=False), cov, atol=0.03)
    with pytest.raises(ValueError, match="one spread"):
        Gaussian(np.zeros(2), std=1.0, cov=cov)


@pytest.mark.optional
@pytest.mark.jax
def test_a_family_of_noise_levels_vmaps_like_a_family_of_masses():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    from minilink import Gaussian

    law = Gaussian(np.zeros(2), np.ones(2))
    key = jax.random.PRNGKey(0)
    stds = jnp.array([[0.1, 0.1], [1.0, 1.0], [3.0, 3.0]])

    family = jax.vmap(
        lambda std: law.sample(key, params={"mean": law.mean, "std": std})
    )(stds)

    one = law.sample(key)
    np.testing.assert_allclose(family, np.asarray(stds) * np.asarray(one), rtol=1e-12)


def test_a_set_reads_its_numbers_from_params_and_moves_by_them():
    from minilink.core.sets import BallSet, BoxSet, SingletonSet

    box = BoxSet([0.0, 0.0], [1.0, 2.0])
    z = np.array([0.5, 0.5])
    assert set(box.params) == {"lower", "upper"}
    np.testing.assert_array_equal(
        box.margin(z, params={"lower": np.zeros(2), "upper": np.ones(2)}),
        box.with_params({"lower": np.zeros(2), "upper": np.ones(2)}).margin(z),
    )
    both = box & BallSet([0.0, 0.0], 1.0)
    assert set(both.params) == {"0", "1"}
    moved = both.with_params(
        {**both.params, "1": {"center": np.ones(2), "radius": 0.1}}
    )
    assert moved.margin(z)[-1] < 0.0 <= both.margin(z)[-1]
    assert SingletonSet([1.0]).params["point"].tolist() == [1.0]


def test_a_sampled_sensor_holds_draws_of_its_datasheet_law():
    from minilink import Gaussian
    from minilink.blocks import NoiseSource

    R_d = np.array([[0.04, 0.01], [0.01, 0.09]])
    sensor = NoiseSource(Gaussian(np.zeros(2), cov=R_d), sample_period=0.02, seed=4)
    assert set(sensor.params) == {"seed", "sample_period", "law"} and sensor.dim == 2

    # The twin: the samples' covariance is the datasheet's, each sample held over Δ
    t = 0.02 * np.arange(4000) + 0.005
    w = np.array([sensor.h(None, None, tk) for tk in t])
    np.testing.assert_allclose(np.cov(w, rowvar=False), R_d, atol=0.006)
    np.testing.assert_array_equal(
        sensor.h(None, None, 0.021), sensor.h(None, None, 0.039)
    )

    # The law's numbers are parameters; no seed is the mean; a period is a hint
    quiet = dict(sensor.params, law={"mean": np.zeros(2), "cov": 0.25 * R_d})
    np.testing.assert_allclose(
        sensor.h(None, None, 0.1, quiet), 0.5 * sensor.h(None, None, 0.1)
    )
    np.testing.assert_array_equal(
        sensor.h(None, None, 0.1, dict(sensor.params, seed=None)), [0.0, 0.0]
    )
    assert sensor.solver_info["sample_period"] == 0.02


@pytest.mark.jax
def test_a_noise_source_is_the_same_signal_on_both_backends():
    pytest.importorskip("jax")
    from minilink import Uniform
    from minilink.blocks import NoiseSource
    from minilink.core.backends import ensure_jax_x64

    jax = ensure_jax_x64()
    noise = NoiseSource(Uniform([-0.005], [0.005]), sample_period=0.02, seed=3)
    t = 0.02 * np.arange(20) + 0.01
    on_numpy = np.array([noise.h(None, None, tk) for tk in t])
    on_jax = jax.vmap(lambda tk: noise.h(jax.numpy.zeros(0), jax.numpy.zeros(0), tk))(t)
    np.testing.assert_array_equal(np.asarray(on_jax), on_numpy)
    assert np.all(np.abs(on_numpy) <= 0.005)


def test_a_source_without_a_seed_is_its_mean_and_not_random():
    from minilink import Gain
    from minilink.blocks.sources import NoiseSource
    from minilink.control.stochastic import GaussianPolicy
    from minilink.core.distributions import Gaussian

    law = Gaussian([1.0], [0.5])
    source = NoiseSource(law, sample_period=0.1, seed=None)
    assert not source.is_random and NoiseSource(law, sample_period=0.1).is_random

    # The mean is the one the params give
    moved = dict(source.params, law={"mean": np.array([2.0]), "std": law.std})
    np.testing.assert_array_equal(source.h(None, None, 0.3, moved), [2.0])

    policy = GaussianPolicy(Gain(np.array([[1.0]])), -1.0, sample_period=0.1)
    assert not policy.is_random
    policy.params["seed"] = 3
    assert policy.is_random


def test_a_full_covariance_law_prints_its_covariance():
    from minilink.core.distributions import Gaussian

    text = str(Gaussian([0.0, 0.0], cov=[[1.0, 0.2], [0.2, 2.0]]))
    assert "cov=" in text and "None" not in text


def test_the_rollout_refuses_a_source_it_cannot_feed():
    from minilink import Pendulum
    from minilink.blocks.sources import WhiteNoise
    from minilink.core.costs import QuadraticCost
    from minilink.learning.reinforcement_learning.environment import (
        RolloutEnvironment,
        episode_seed,
    )
    from minilink.planning.problems import PlanningProblem

    sys = Pendulum()
    problem = PlanningProblem(
        sys,
        cost=QuadraticCost.from_system(sys),
        tf=1.0,
        disturbances={"n": WhiteNoise(2, psd=0.1)},
    )
    with pytest.raises(ValueError, match="no port on"):
        RolloutEnvironment(problem, dt=0.1, backend="numpy")

    # An integer key gives each port its own seed
    assert episode_seed(5, "d") != episode_seed(5, "n")
    assert episode_seed(5, "d") == episode_seed(5, "d")
