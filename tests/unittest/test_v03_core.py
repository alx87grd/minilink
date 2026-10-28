"""The v0.3 core's definition of done: one test per acceptance script, named by its number and lane."""

import numpy as np
import pytest

from minilink import (
    PID,
    DiagramSystem,
    Integrator,
    P,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Sine,
    Step,
    StepSystem,
    WhiteNoise,
    closed_loop,
)
from minilink.analysis import discretize
from minilink.core.backends import array_module
from minilink.core.discrete.sampled_diagram import SampledDiagramSystem


class LowPass(StepSystem):
    """``z_{k+1} = α z_k + (1 − α) u_k``, ``y = z``."""

    def __init__(self, alpha=0.6):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        alpha = params["alpha"]

        # z_{k+1} = α z_k + (1 − α) u_k
        z_next = xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

        return z_next

    def h(self, x, u, k=0, params=None):
        return x


def digital_pid_problem():
    """A digital PID on the pendulum, its setpoint 1 rad, scored over 2 s."""
    plant = Pendulum()
    plant.solver_info["smallest_time_constant"] = 0.005
    cost = QuadraticCost.from_system(plant, xbar=np.array([1.0, 0.0]))
    problem = PlanningProblem(plant, cost=cost, x_start=np.zeros(2), tf=2.0)
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    pid.inputs["e"].nominal_value = np.zeros(1)
    return problem, pid


# Script 1, the objects lane

LQG_DT, LQG_TF = 0.01, 5.0
PROCESS_PSD = 0.04  # W, the torque's white-noise intensity
SENSOR_STD = 1.0  # the rate sensor's per-sample standard deviation over LQG_DT


def lqg_problem():
    """An LQG compensator on the noisy pendulum: white noise on the torque, a sampled sensor on the rate."""
    from minilink import Gaussian, NoisyPendulum
    from minilink.blocks import NoiseSource
    from minilink.control.design import lqg

    plant = NoisyPendulum()
    Q, r = np.diag([10.0, 1.0]), 0.1
    V = SENSOR_STD**2 * LQG_DT  # the sensor's intensity: its variance over its period
    box = lqg(
        plant,
        Q,
        np.array([[r]]),
        Q_n=np.diag([0.0, PROCESS_PSD]),
        R_n=np.diag([1e-4, V]),
    )
    cost = QuadraticCost.from_system(plant, Q=Q, R=np.diag([r, 0.0, 0.0]))
    problem = PlanningProblem(
        plant,
        cost=cost,
        x_start=np.zeros(2),
        tf=LQG_TF,
        disturbances={
            "d": WhiteNoise(1, psd=PROCESS_PSD, sample_period=LQG_DT, seed=1),
            "n": NoiseSource(
                Gaussian([0.0], [SENSOR_STD]), sample_period=LQG_DT, seed=2
            ),
        },
    )
    return problem, box, Q, r, V


def predicted_cost(plant, box, Q, r, V):
    """``E[J] = ∫₀ᵀ tr(M P(t)) dt``, ``Ṗ = A P + P Aᵀ + N``: the loop's covariance from a zero start."""
    from minilink.analysis import linearize

    lin = linearize(plant)
    K = box.subsystems["gain"].params["K"]
    L = box.subsystems["observer"].params["L"]
    A, C = lin.A(), lin.C()
    B_u, B_d, D_n = lin.B()[:, :1], lin.B()[:, 1:2], lin.D()[:, 2:3]

    # X = [x; x̂]: ẋ = A x − B K x̂ + B_d d, the estimate driven by y = C x + D_n n
    A_cl = np.block([[A, -B_u @ K], [L @ C, A - B_u @ K - L @ C]])
    B_w = np.vstack([B_d, np.zeros((2, 1))])
    B_v = np.vstack([np.zeros((2, 1)), L @ D_n])
    N = PROCESS_PSD * B_w @ B_w.T + V * B_v @ B_v.T
    M = np.zeros((4, 4))
    M[:2, :2] = Q
    M[2:, 2:] = r * K.T @ K

    # The Lyapunov differential equation by RK4, E[J] by the trapezoid
    h = 1e-3
    P = np.zeros((4, 4))
    expected = 0.0
    for _ in range(round(LQG_TF / h)):

        def rate(P):
            return A_cl @ P + P @ A_cl.T + N

        k1 = rate(P)
        k2 = rate(P + 0.5 * h * k1)
        k3 = rate(P + 0.5 * h * k2)
        k4 = rate(P + h * k3)
        P_next = P + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        expected += 0.5 * h * (np.trace(M @ P) + np.trace(M @ P_next))
        P = P_next

    return expected


@pytest.mark.optional
@pytest.mark.jax
def test_script_01_objects_an_lqg_loop_scores_its_covariance_prediction():
    pytest.importorskip("jax")
    problem, box, Q, r, V = lqg_problem()
    expected = predicted_cost(problem.sys, box, Q, r, V)

    # The Monte Carlo mean within the sampling error of the prediction (3 standard errors)
    runs = problem.compile(box, dt=LQG_DT, backend="jax").evaluate(n_trials=400, key=1)
    J = np.asarray(runs.J)
    assert abs(J.mean() - expected) < 3.0 * J.std(ddof=1) / np.sqrt(J.size)

    # One key, the same draws on NumPy and JAX
    on_numpy = problem.compile(box, dt=LQG_DT, backend="numpy").evaluate(
        n_trials=4, key=1
    )
    np.testing.assert_allclose(np.asarray(on_numpy.J), J[:4], rtol=1e-9)

    # The computer sampled: its noise blocks realized per trial, the same prediction
    sampled = problem.compile(box % LQG_DT, backend="jax").evaluate(n_trials=400, key=1)
    J_s = np.asarray(sampled.J)
    assert len(set(J_s[:10])) == 10
    assert abs(J_s.mean() - expected) < 3.0 * J_s.std(ddof=1) / np.sqrt(J_s.size)


# Script 2, the loop lane


@pytest.mark.optional
@pytest.mark.jax
def test_script_02_loop_a_digital_pid_has_a_gradient():
    jax = pytest.importorskip("jax")
    problem, pid = digital_pid_problem()
    pe = problem.compile(pid % 0.01)
    assert pe.dt == 0.01 and pe.backend == "jax"
    params = pe.params
    params["r"] = np.array([1.0])

    grad = jax.grad(lambda p: pe.evaluate(p).objective)(params)

    eps = 1e-6
    for gain in ("Kp", "Ki", "Kd"):
        up = {**params, "controller": {**params["controller"]}}
        down = {**params, "controller": {**params["controller"]}}
        up["controller"][gain] = params["controller"][gain] + eps
        down["controller"][gain] = params["controller"][gain] - eps
        central = (pe.evaluate(up).objective - pe.evaluate(down).objective) / (2 * eps)
        np.testing.assert_allclose(grad["controller"][gain], central, rtol=1e-6)

    with pytest.raises(ValueError, match="own clock"):
        problem.compile(pid % 0.01, dt=0.02)


# Script 3, the loop lane


def test_script_03_loop_the_run_verbs_and_the_score_give_one_run():
    problem, pid = digital_pid_problem()
    loop = (pid % 0.01) @ problem.sys
    loop.inputs["r"].nominal_value = np.array([1.0])
    pe = problem.compile(loop, backend="numpy")
    S = loop.subsystems["plant"].substeps

    scored = pe.trajectory()
    ticks = loop.compute_rollout(200)
    dense = loop.compute_trajectory(tf=2.0)
    host = loop.compute_trajectory(tf=2.0, solver="scipy")

    np.testing.assert_array_equal(scored.x, ticks.x)
    np.testing.assert_array_equal(dense.x[:, ::S], ticks.x)
    np.testing.assert_allclose(host.x, dense.x, atol=1e-7)


# Script 10, the loop lane


def multi_rate_loop():
    """A 100 Hz filter and a 10 Hz gain on a continuous integrator, the reference a step."""
    return closed_loop(
        P(10.0, ports="reference") % 0.1,
        discretize(Integrator(), 0.01, substeps=5),
        reference=Step(np.array([1.0]), np.array([0.0]), step_time=1.0),
        filter=LowPass(alpha=0.6) % 0.01,
    )


@pytest.mark.parametrize("backend", ["numpy", "jax"])
def test_script_10_loop_a_multi_rate_computer_on_a_continuous_plant(backend):
    if backend == "jax":
        pytest.importorskip("jax")
    loop = multi_rate_loop()
    assert isinstance(loop, SampledDiagramSystem)
    assert loop.sample_period == 0.01
    held = loop.hold_index[(("controller",), "u")]

    run = loop.compute_rollout(200, backend=backend)

    # The same loop by hand, the filter read at the same tick (R125), the gain
    # firing on k mod 10 = 0 and its command held in between
    x_f, x_p, u_held = 0.0, 0.0, 0.0
    states = []
    for k in range(200):
        r = 1.0 if 0.01 * k < 1.0 else 0.0
        if k % 10 == 0:
            u_held = 10.0 * (r - x_f)
        x_f = 0.6 * x_f + 0.4 * x_p
        x_p = x_p + 0.01 * u_held
        states.append((x_f, x_p, u_held))
    a, b = loop.state_index["plant"]
    np.testing.assert_allclose(
        run.x[a, 1:], [s[1] for s in states], rtol=1e-12, atol=1e-14
    )
    np.testing.assert_allclose(
        run.x[held[0], 1:], [s[2] for s in states], rtol=1e-12, atol=1e-14
    )


# Script 11, the loop lane


def test_script_11_loop_a_bare_reference_reads_seconds():
    dt = 0.01
    plant = Pendulum()
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    bare = Sine(amplitude=0.5, omega=2.0) >> (pid % dt) @ plant
    clocked = (Sine(amplitude=0.5, omega=2.0) % dt) >> (pid % dt) @ plant
    np.testing.assert_array_equal(
        bare.compute_rollout(100).x, clocked.compute_rollout(100).x
    )

    # A white noise on its own period reads seconds: bare in the loop, the draws
    # of its continuous diagram at the ticks, from the same seed
    noise = WhiteNoise(1, psd=0.1, sample_period=0.05, seed=3)
    sampled = SampledDiagramSystem()
    sampled.add_subsystem(noise, "noise")
    sampled.add_subsystem(discretize(Integrator(), dt), "plant")
    sampled.connect("noise", "y", "plant", "u")
    continuous = DiagramSystem()
    continuous.add_subsystem(noise, "noise")
    continuous.add_subsystem(Integrator(), "plant")
    continuous.connect("noise", "y", "plant", "u")
    ev = sampled.compile()
    ticks = sampled.compute_rollout(40)
    draws = [ev.signals(ticks.x[:, k], np.zeros(0), k)[0] for k in range(40)]
    t = dt * np.arange(40)
    np.testing.assert_array_equal(draws, [noise.h(None, None, tk)[0] for tk in t])
    traj = continuous.compute_trajectory(
        tf=0.4, dt=dt, solver="euler_fixedsteps", verbose=False
    )
    np.testing.assert_allclose(ticks.x[0], traj.x[0], rtol=1e-12, atol=1e-15)
