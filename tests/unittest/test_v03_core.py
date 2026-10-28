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
