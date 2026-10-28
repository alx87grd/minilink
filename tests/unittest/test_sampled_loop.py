"""The sampled loop: blocks on clocks composed into a step diagram, one pure map on the base tick."""

import numpy as np
import pytest

from minilink import PID, Integrator, P, Pendulum, Sine, StepSystem
from minilink.analysis import discretize
from minilink.core.backends import array_module
from minilink.core.discrete.sampled_diagram import SampledDiagramSystem
from minilink.core.discrete.sampled_system import SampledSystem
from minilink.core.discrete.step_diagram import StepDiagramSystem


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


def multi_rate_computer_on_an_integrator():
    """A 100 Hz filter and a 10 Hz gain on a continuous integrator, wired by hand."""
    loop = SampledDiagramSystem()
    loop.add_subsystem(LowPass() % 0.01, "filter")
    loop.add_subsystem(P(10.0, ports="reference") % 0.1, "controller")
    loop.add_subsystem(discretize(Integrator(), 0.01, substeps=5), "plant")
    loop.add_input_port("r")
    loop.connect("input", "r", "controller", "r")
    loop.connect("plant", "y", "filter", "u")
    loop.connect("filter", "y", "controller", "y")
    loop.connect("controller", "u", "plant", "u")
    loop.connect_new_output_port("plant", "y", "y")
    return loop


def test_a_multi_rate_loop_is_one_map_on_its_base_tick():
    loop = multi_rate_computer_on_an_integrator()
    assert loop.sample_period == 0.01
    assert loop.divisor_of(loop.subsystems["controller"]) == 10
    assert loop.hold_index == {(("controller",), "u"): (2, 3)}
    assert loop.n == 3

    traj = loop.compute_rollout(200, u=np.array([1.0]))

    # The same loop by hand: the slow gain fires on k mod 10 = 0, its command held
    x_f, x_p, u_held = 0.0, 0.0, 0.0
    for k in range(200):
        if k % 10 == 0:
            u_held = 10.0 * (1.0 - x_f)  # y_f read at the same tick
        x_f = 0.6 * x_f + 0.4 * x_p
        x_p = x_p + 0.01 * u_held
    np.testing.assert_allclose(traj.x[:, -1], [x_f, x_p, u_held], rtol=1e-12)
    np.testing.assert_allclose(traj.t, 0.01 * np.arange(201))


@pytest.mark.optional
@pytest.mark.jax
def test_the_loop_runs_alike_on_jax_with_a_branch_per_slow_block():
    jax = pytest.importorskip("jax")
    loop = multi_rate_computer_on_an_integrator()
    us = np.ones((50, 1))
    on_numpy = loop.compile().rollout(loop.x0, us)
    ev = loop.compile(backend="jax")
    np.testing.assert_allclose(ev.rollout(loop.x0, us), on_numpy, rtol=1e-14)

    # The firing is a cond on the tick alone, kept as a branch under vmap
    batched = jax.vmap(lambda x0: ev.rollout(x0, us))
    jaxpr = str(jax.make_jaxpr(batched)(np.tile(loop.x0, (4, 1))))
    assert "cond" in jaxpr


def test_pid_on_a_clock_is_its_euler_difference_equation():
    pid = PID(Kp=2.0, Ki=1.5, Kd=0.5)
    digital = pid % 0.01
    assert isinstance(digital, SampledSystem)
    assert list(digital.inputs) == list(pid.inputs)
    x = np.array(digital.x0, dtype=float)
    e = np.array([0.7])
    for k in range(5):
        x_next = digital.step(x, e, k)
        np.testing.assert_array_equal(x_next, x + 0.01 * pid.f(x, e, 0.01 * k))
        x = x_next


@pytest.mark.parametrize("backend", ["numpy", "jax"])
def test_sampled_at_continuous_is_the_hand_written_loop(backend):
    if backend == "jax":
        pytest.importorskip("jax")
    dt = 0.01
    plant = Pendulum()
    plant.x0 = np.array([0.3, 0.0])
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    loop = (pid % dt) @ discretize(plant, dt)
    assert isinstance(loop, SampledDiagramSystem)
    a, b = loop.state_index["plant"]
    c, d = loop.state_index["controller"]

    run = loop.compute_rollout(50, backend=backend)

    # e_k = r − θ_k, the PID's difference equation, the plant one RK4 period on
    sampled_plant = discretize(plant, dt)
    x_p, z = plant.x0.copy(), np.array(pid.x0, dtype=float)
    for k in range(50):
        e = np.array([0.0 - x_p[0]])
        u = pid.outputs["u"].compute(z, e, k * dt)
        z = z + dt * pid.f(z, e, k * dt)
        x_p = sampled_plant.step(x_p, u, k)
    np.testing.assert_allclose(run.x[a:b, -1], x_p, rtol=1e-12)
    np.testing.assert_allclose(run.x[c:d, -1], z, rtol=1e-12)


def test_a_continuous_and_a_tick_operand_are_refused_naming_the_clock():
    with pytest.raises(TypeError, match="%"):
        PID(Kp=1.0) @ discretize(Pendulum(), 0.01)
    with pytest.raises(TypeError, match="%"):
        LowPass() @ Integrator()


def test_a_tick_diagram_refuses_a_clocked_block():
    diagram = StepDiagramSystem()
    with pytest.raises(TypeError, match="SampledDiagramSystem"):
        diagram.add_subsystem(LowPass() % 0.01, "filter")


def test_periods_must_be_commensurate():
    loop = SampledDiagramSystem()
    loop.add_subsystem(LowPass() % 0.01, "a")
    with pytest.raises(ValueError, match="commensurate"):
        loop.add_subsystem(LowPass() % (0.01 * np.sqrt(2.0)), "b")


def test_a_bare_static_block_reads_seconds_in_a_sampled_loop():
    dt = 0.01
    plant = Pendulum()
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    bare = Sine(amplitude=0.5, omega=2.0) >> (pid % dt) @ plant
    clocked = (Sine(amplitude=0.5, omega=2.0) % dt) >> (pid % dt) @ plant
    np.testing.assert_array_equal(
        bare.compute_rollout(100).x, clocked.compute_rollout(100).x
    )


def test_a_box_of_clocked_blocks_nests_under_its_role():
    computer = (LowPass() % 0.01) + (P(10.0, ports="reference") % 0.1)
    assert isinstance(computer, SampledDiagramSystem)
    assert computer.sample_period == 0.01
    assert computer.hold_dim == 1
