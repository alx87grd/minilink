"""The sampled loop: blocks on clocks composed into a step diagram, one pure map on the base tick."""

import numpy as np
import pytest

from minilink import PID, Integrator, P, Pendulum, Sine, StepSystem
from minilink.analysis import discretize, linearize
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
    loop = (pid % dt) @ plant
    assert isinstance(loop, SampledDiagramSystem)
    a, b = loop.state_index["plant"]
    c, d = loop.state_index["controller"]

    run = loop.compute_rollout(50, dt=dt, backend=backend)

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


def a_digital_pid_on_a_pendulum(dt=0.01):
    plant = Pendulum()
    plant.x0 = np.array([0.3, 0.0])
    plant.solver_info["smallest_time_constant"] = 0.005
    return (PID(Kp=8.0, Ki=1.0, Kd=2.0) % dt) @ plant


@pytest.mark.parametrize("backend", ["numpy", "jax"])
def test_the_two_run_verbs_agree_at_the_ticks(backend):
    if backend == "jax":
        pytest.importorskip("jax")
    loop = a_digital_pid_on_a_pendulum()
    S = loop.subsystems["plant"].substeps
    assert S == 20

    ticks = loop.compute_rollout(100, backend=backend)
    dense = loop.compute_trajectory(tf=1.0, backend=backend)

    assert dense.x.shape == (4, 100 * S + 1)
    np.testing.assert_allclose(dense.t[::S], ticks.t, rtol=0, atol=1e-15)
    np.testing.assert_array_equal(dense.x[:, ::S], ticks.x)


def test_the_dense_run_is_the_loop_re_clocked_at_its_substeps():
    dt = 0.01
    loop = a_digital_pid_on_a_pendulum(dt)
    S = loop.subsystems["plant"].substeps
    plant = loop.subsystems["plant"].source
    re_clocked = (PID(Kp=8.0, Ki=1.0, Kd=2.0) % dt) @ discretize(plant, dt / S)
    assert re_clocked.sample_period == pytest.approx(dt / S)

    dense = loop.compute_trajectory(tf=0.5)
    fine = re_clocked.compute_rollout(50 * S)
    a, b = loop.state_index["plant"]
    c, d = re_clocked.state_index["plant"]
    np.testing.assert_allclose(dense.x[a:b], fine.x[c:d], rtol=1e-12, atol=1e-14)


def test_the_host_clock_runner_agrees_within_its_tolerance():
    loop = a_digital_pid_on_a_pendulum()
    dense = loop.compute_trajectory(tf=1.0)
    host = loop.compute_trajectory(tf=1.0, solver="scipy")
    np.testing.assert_allclose(host.x, dense.x, atol=1e-8)


def test_a_tick_law_has_no_continuous_time():
    with pytest.raises(TypeError, match="compute_rollout"):
        (LowPass() % 0.01).compute_trajectory(tf=1.0)


@pytest.mark.optional
@pytest.mark.jax
def test_the_problem_refuses_the_pure_mpc_naming_its_clock():
    pytest.importorskip("jax")
    from minilink import PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner
    from minilink.control import ModelPredictiveController

    plant = Pendulum()
    problem = PlanningProblem(
        plant, cost=QuadraticCost.from_system(plant), x_start=np.zeros(2), tf=1.0
    )
    planner = TrajectoryOptimizationPlanner(problem, n_steps=5, verbose=False)
    mpc = ModelPredictiveController(planner, warm_start=False)
    with pytest.raises(TypeError, match="% dt"):
        problem.compile(mpc, dt=0.1)

    # The delayed form with no clock is a tick diagram with no clock: refused alike
    with pytest.raises(TypeError, match="% dt"):
        problem.compile(ModelPredictiveController(planner, delay=True), dt=0.1)


def test_a_tick_law_with_no_clock_is_refused_by_the_problem():
    from minilink import PlanningProblem, QuadraticCost

    plant = Pendulum()
    problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant), tf=1.0)
    with pytest.raises(TypeError, match="%"):
        problem.compile(LowPass(), dt=0.01)


def test_the_seconds_face_is_the_evaluators_step():
    plant = Pendulum()
    sampled = discretize(plant, 0.05, substeps=3)
    ev = plant.compile()
    x, u = np.array([0.4, -0.2]), np.array([0.3])
    np.testing.assert_array_equal(
        sampled.update(x, u, 1.25), ev.step(x, u, 1.25, dt=0.05, substeps=3)
    )


@pytest.mark.parametrize(
    "loop, dt",
    [
        (lambda: multi_rate_computer_on_an_integrator(), None),
        (lambda: (SlidingModeOf(Pendulum()) % 0.05) @ Pendulum(), 0.005),
    ],
)
def test_the_host_runner_agrees_on_the_hybrid_demos(loop, dt):
    loop = loop()
    u = np.ones(loop.m) if loop.m else None
    dense = loop.compute_trajectory(tf=1.0, u=u, dt=dt)
    host = loop.compute_trajectory(tf=1.0, u=u, dt=dt, solver="scipy")
    np.testing.assert_allclose(host.x, dense.x, atol=1e-6)


def SlidingModeOf(plant):
    """The sampled sliding-mode demo's law on its plant, the reference at the origin."""
    from minilink import SlidingMode

    plant.x0 = np.array([np.pi + 0.25, 0.0])
    return SlidingMode(plant, lam=2.0, gain=8.0, nab=0.15)


@pytest.mark.parametrize(
    "block",
    [
        lambda: discretize(linearize(Pendulum()), 0.01, solver="exact"),
        lambda: P(2.0) % 0.1,
    ],
)
def test_a_block_with_nothing_to_integrate_has_one_point_per_period(block):
    block = block()
    h = block.sample_period
    ticks = block.compute_rollout(5, u=np.ones(block.m))
    dense = block.compute_trajectory(tf=5 * h, u=np.ones(block.m))
    host = block.compute_trajectory(tf=5 * h, u=np.ones(block.m), solver="scipy")
    np.testing.assert_array_equal(dense.x, ticks.x)
    np.testing.assert_array_equal(host.x, ticks.x)


def test_the_plant_is_found_by_kind_never_by_id():
    # A model (discretize) in a chain is a step system: ticks only, no plant
    chain = (Sine() % 0.05) >> discretize(Pendulum(), 0.05, substeps=5)
    assert chain.compile().plan.plant_op is None
    assert chain.compute_trajectory(tf=0.5).x.shape[1] == 10 + 1

    # The continuous plant a loop closes on is its plant, at any depth
    inner = (PID(Kp=8.0, Ki=1.0, Kd=2.0) % 0.01) @ Pendulum()
    cascade = (PID(Kp=1.0) % 0.05) @ inner
    plan = cascade.compile(substeps=5).plan
    assert plan.plant_op.label == "plant/plant"
    assert plan.dense_points == 5


def test_two_continuous_systems_in_a_sampled_diagram_are_refused():
    both = SampledDiagramSystem()
    both.add_subsystem((PID(Kp=1.0) % 0.01) @ Integrator(), "a")
    both.add_subsystem((PID(Kp=1.0) % 0.01) @ Integrator(), "b")
    with pytest.raises(ValueError, match="compose them into one continuous system"):
        both.compile()


def test_a_chain_of_two_euler_laws_is_a_computer_with_no_plant():
    chain = (PID(Kp=1.0, Ki=1.0) % 0.01) >> (LowPass() % 0.01)
    ev = chain.compile()
    assert ev.plan.plant_op is None
    run = chain.compute_trajectory(tf=0.1, u=np.ones(chain.m))
    np.testing.assert_array_equal(
        run.x, chain.compute_rollout(10, u=np.ones(chain.m)).x
    )


def test_the_run_step_sets_the_plants_substeps():
    loop = a_digital_pid_on_a_pendulum()
    assert loop.subsystems["plant"].substeps == 20
    run = loop.compute_trajectory(tf=0.1, dt=0.002)
    assert run.t.size == 10 * 5 + 1
    assert loop.compile(substeps=5).plan.dense_points == 5
    np.testing.assert_array_equal(loop.compute_rollout(10, dt=0.002).x, run.x[:, ::5])
    with pytest.raises(ValueError, match="no continuous plant"):
        ((PID() % 0.01) >> (LowPass() % 0.01)).compile(substeps=4)


def test_a_static_box_joins_a_sampled_loop():
    gains = P(2.0) >> P(3.0)
    loop = SampledDiagramSystem()
    loop.add_subsystem(LowPass() % 0.01, "filter")
    loop.add_subsystem(gains, "gains")
    loop.connect("filter", "y", "gains", list(gains.inputs)[0])
    loop.connect_new_output_port("gains", list(gains.outputs)[0], "y")

    ev = loop.compile()
    x = np.array([0.5])
    np.testing.assert_allclose(ev.outputs(x, np.zeros(0), 0)["y"], [3.0])


def test_a_loop_keeps_its_own_x0_held_outputs_included_on_refresh():
    loop = multi_rate_computer_on_an_integrator()
    loop.x0 = np.array([0.5, 1.0, 0.0])
    loop.refresh()
    np.testing.assert_array_equal(loop.x0, [0.5, 1.0, 0.0])


def test_a_sampled_plant_reads_its_source_x0_again_on_refresh():
    plant = Pendulum()
    loop = (PID() % 0.01) @ plant
    plant.x0 = np.array([0.3, 0.0])
    loop.refresh()
    a, b = loop.state_index["plant"]
    np.testing.assert_array_equal(loop.x0[a:b], plant.x0)

    # An x0 set on the view itself stays
    view = discretize(Pendulum(), 0.01)
    view.x0 = np.array([1.0, 0.0])
    view.refresh()
    np.testing.assert_array_equal(view.x0, [1.0, 0.0])


def test_a_block_on_a_clock_reads_seconds_in_its_outputs():
    s = Sine() % 0.1
    ev = s.compile()
    x, u = np.zeros(0), np.zeros(0)
    np.testing.assert_array_equal(ev.outputs(x, u, 3)["y"], s.h(x, u, 3))
    np.testing.assert_allclose(ev.outputs(x, u, 3)["y"], np.sin(0.3))


def test_animating_a_loop_reads_the_recorded_plant_input():
    # A pure MPC solves at each read of its output: the frames must not read it again
    plant = Pendulum()
    plant.x0 = np.array([0.3, 0.0])
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    loop = (pid % 0.05) @ plant
    run = loop.compute_trajectory(tf=1.0, dt=0.01)
    port = pid.outputs["u"]
    original = port.compute
    calls = []

    def counting(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    port.compute = counting
    loop.animate(run, show=False, html=False)

    # The run recorded the plant's input: the frames read it, evaluating nothing
    assert len(calls) == 0


# The time table: what each block reads as time on a clock (the night run's step 1)


class TickCount(StepSystem):
    """A tick law whose output is its own count: ``y_k = k``."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

    def step(self, x, u, k=0, params=None):
        return x

    def h(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        return xp.array([float(k)])


def box_of(block):
    """A tick diagram holding one block, its output on the edge."""
    box = StepDiagramSystem()
    box.add_subsystem(block, "block")
    box.connect_new_output_port("block", "y", "y")
    return box


def output_at(sampled, k):
    ev = sampled.compile()
    x = np.asarray(sampled.x0, dtype=float)
    u = np.zeros(sampled.m)
    return ev.outputs(x, u, k)["y"]


def test_a_static_block_in_a_box_on_a_clock_reads_seconds():
    clocked = box_of(Sine()) % 0.1
    assert isinstance(clocked, SampledDiagramSystem)
    np.testing.assert_allclose(output_at(clocked, 3), np.sin(0.3))


def test_a_tick_law_in_a_box_on_a_clock_reads_its_count():
    clocked = box_of(TickCount()) % 0.1
    np.testing.assert_allclose(output_at(clocked, 3), 3.0)


def test_a_box_nested_in_a_box_is_clocked_block_by_block():
    outer = StepDiagramSystem()
    outer.add_subsystem(box_of(Sine()), "inner")
    outer.connect_new_output_port("inner", "y", "y")
    np.testing.assert_allclose(output_at(outer % 0.1, 3), np.sin(0.3))


def test_a_slow_box_reads_seconds_at_its_ticks_and_holds_between():
    loop = SampledDiagramSystem()
    loop.add_subsystem(box_of(Sine()) % 0.1, "slow")
    loop.add_subsystem(Sine() % 0.05, "fast")
    loop.connect_new_output_port("slow", "y", "y")
    assert loop.sample_period == 0.05
    np.testing.assert_allclose(output_at(loop, 2), np.sin(0.1))
    ev = loop.compile()
    x = ev.step(np.asarray(loop.x0, dtype=float), np.zeros(0), 2)
    np.testing.assert_allclose(ev.outputs(x, np.zeros(0), 3)["y"], np.sin(0.1))


def test_a_box_is_clocked_once():
    clocked = box_of(Sine()) % 0.1
    with pytest.raises(TypeError, match="already holds blocks on a clock"):
        clocked % 0.1


def test_a_bare_tick_law_is_refused_where_a_nested_box_would_speed_it_up():
    inner = SampledDiagramSystem()
    inner.add_subsystem(Sine() % 0.1, "sine")
    inner.add_subsystem(LowPass(), "filter")
    inner.connect("sine", "y", "filter", "u")
    inner.connect_new_output_port("filter", "y", "y")
    loop = SampledDiagramSystem()
    loop.add_subsystem(inner, "box")
    loop.add_subsystem(Sine() % 0.05, "fast")
    with pytest.raises(TypeError, match="give it its box's clock"):
        loop.compile()


def test_the_mpc_box_has_no_clock_operator_of_its_own():
    from minilink.planning.trajectory_optimization.mpc.predictive import (
        PredictiveController,
    )

    assert "__mod__" not in vars(PredictiveController)


def test_the_host_runner_lands_on_the_period_end_for_any_substep_count():
    # Δ = 0.05 with 3 substeps puts the last sample an ulp past the period's end in
    # floating point; the runner's grid ends on the period exactly
    plant = Pendulum()
    plant.x0 = np.array([0.3, 0.0])
    loop = (PID(Kp=8.0, Ki=1.0, Kd=2.0) % 0.05) @ plant
    host = loop.compute_trajectory(tf=0.2, dt=0.05 / 3, solver="scipy")
    dense = loop.compute_trajectory(tf=0.2, dt=0.05 / 3)
    assert host.t.shape == dense.t.shape
    np.testing.assert_allclose(host.x, dense.x, atol=1e-4)


def test_a_lone_sampled_plant_is_integrated_once_per_period_in_the_dense_run():
    class CountingPendulum(Pendulum):
        calls = 0

        def f(self, x, u, t=0, params=None):
            type(self).calls += 1
            return super().f(x, u, t, params)

    sampled = discretize(CountingPendulum(), 0.1, substeps=4)
    CountingPendulum.calls = 0
    sampled.compute_rollout(10)
    on_the_ticks = CountingPendulum.calls
    CountingPendulum.calls = 0
    sampled.compute_trajectory(tf=1.0)
    assert CountingPendulum.calls == on_the_ticks


def test_the_plant_run_of_a_ticks_run_reads_the_input_the_loop_fed_it():
    from minilink.core.discrete.sampled_diagram import plant_run_of

    loop = a_digital_pid_on_a_pendulum()
    S = loop.subsystems["plant"].substeps
    ticks = loop.compute_rollout(40)
    dense = loop.compute_trajectory(tf=0.4)
    _, on_ticks = plant_run_of(loop, ticks)
    _, on_dense = plant_run_of(loop, dense)
    np.testing.assert_allclose(on_ticks.u, on_dense.u[:, ::S], rtol=0, atol=1e-12)


def test_a_recorded_run_of_no_period_is_refused():
    loop = a_digital_pid_on_a_pendulum()
    with pytest.raises(ValueError, match="at least one period"):
        loop.compute_trajectory(tf=0.0)


def test_a_sampled_data_law_takes_its_period_at_construction():
    from minilink.core.discrete.sampled_system import NativeSampledSystem

    with pytest.raises(TypeError, match="at construction"):
        NativeSampledSystem(n=0)
    law = NativeSampledSystem(n=0, sample_period=0.1)
    with pytest.raises(TypeError, match="never by %"):
        law % 0.2
    with pytest.raises(TypeError, match="already on a clock"):
        discretize(law, 0.1)
