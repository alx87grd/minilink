"""The loop's grammar: the plant continuous, the computer clocked; each pipeline the plan lists, and its refusals."""

import numpy as np
import pytest

from minilink import (
    PID,
    Integrator,
    P,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Sine,
    StepSystem,
    WhiteNoise,
    closed_loop,
    discretize,
)
from minilink.analysis import linearize
from minilink.analysis.linear import poles
from minilink.blocks.step import UnitDelay
from minilink.core.backends import array_module
from minilink.core.discrete.sampled_diagram import SampledDiagramSystem
from minilink.core.discrete.sampled_system import SampledSystem
from minilink.dynamics.abstraction.state_space import DiscreteLTISystem, LTISystem


class DiscreteLowPass(StepSystem):
    """``z_{k+1} = α z_k + (1 − α) u_k``, ``y = z``: a tick law."""

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


def pendulum():
    plant = Pendulum()
    plant.x0 = np.array([0.3, 0.0])
    return plant


def pid():
    return PID(Kp=8.0, Ki=1.0, Kd=2.0)


# The pipelines, as the user writes them


def test_a_continuous_loop_runs_and_linearizes_in_s():
    loop = pid() @ pendulum()
    run = loop.compute_trajectory(tf=1.0, dt=0.01)
    assert run.t[-1] == pytest.approx(1.0)
    assert isinstance(linearize(loop, x_bar=np.zeros(loop.n)), LTISystem)


def test_a_pure_step_chain_rolls_out_and_linearizes_in_z_per_tick():
    chain = UnitDelay() >> DiscreteLowPass(0.5)
    run = chain.compute_rollout(5, u=np.ones(1))
    np.testing.assert_allclose(run.t, np.arange(6))
    model = linearize(chain)
    assert isinstance(model, DiscreteLTISystem) and model.period is None
    np.testing.assert_allclose(np.sort(np.abs(poles(model.A()))), [0.0, 0.5])


def test_a_computer_alone_reads_seconds_and_linearizes_with_its_period():
    computer = (Sine() % 0.01) >> (P(2.0) % 0.01)
    run = computer.compute_rollout(10)
    np.testing.assert_allclose(run.t, 0.01 * np.arange(11))
    model = linearize(PID(Kp=2.0, Ki=1.0, Kd=0.5) % 0.01)
    assert model.period == 0.01


def test_the_sampled_loop_runs_its_ticks_its_dense_run_and_the_host_runner():
    loop = (pid() % 0.01) @ pendulum()
    ticks = loop.compute_rollout(20, dt=0.002)
    dense = loop.compute_trajectory(tf=0.2, dt=0.002, record=["controller:u"])
    host = loop.compute_trajectory(tf=0.2, dt=0.002, solver="scipy")
    np.testing.assert_array_equal(dense.x[:, ::5], ticks.x)
    np.testing.assert_allclose(host.x, dense.x, atol=1e-6)
    assert dense.has_signal("controller:u") and "plant" in dense.roles

    # A ticks run records the plant's input and its role too
    assert ticks.has_signal("plant:u") and "plant" in ticks.roles
    euler = loop.compute_trajectory(tf=0.2, dt=0.002, solver="euler")
    assert not np.array_equal(euler.x, dense.x)


def test_the_multi_rate_loop_is_the_same_by_parentheses_or_by_roles():
    by_hand = ((DiscreteLowPass(0.6) % 0.01) >> (P(10.0) % 0.1)) @ Integrator()
    by_roles = closed_loop(
        P(10.0) % 0.1, Integrator(), filter=DiscreteLowPass(0.6) % 0.01
    )
    assert by_hand.sample_period == by_roles.sample_period == 0.01
    a = by_hand.compute_rollout(50, u=np.ones(by_hand.m), dt=0.002)
    b = by_roles.compute_rollout(50, u=np.ones(by_roles.m), dt=0.002)
    pa = by_hand.state_index["plant"]
    pb = by_roles.state_index["plant"]
    assert a.x[slice(*pa)].shape == b.x[slice(*pb)].shape


def test_a_tick_law_on_its_rate_inside_the_computer():
    computer = (DiscreteLowPass(0.5) % 0.02) >> (P(2.0) % 0.01)
    assert computer.sample_period == 0.01
    assert computer.divisor_of(computer.subsystems[list(computer.subsystems)[0]]) == 2
    computer.compute_rollout(4, u=np.ones(1))


def test_the_continuous_dynamics_the_computer_does_not_own_are_the_plants():
    plant = Integrator() >> Integrator()
    loop = (P(1.0) % 0.01) @ plant
    plan = loop.compile(substeps=4).plan
    assert plan.plant_op is not None and plan.dense_points == 4


def test_a_plants_euler_model_on_the_computer_is_never_adaptive():
    predictor = Pendulum() % 0.01
    assert predictor.kind == "law" and not predictor.resolves
    ticks = predictor.compute_rollout(5)
    host = predictor.compute_trajectory(tf=0.05, solver="scipy")
    np.testing.assert_array_equal(host.x, ticks.x)


def test_the_models_for_the_discrete_tools():
    plant = Pendulum()
    exact = discretize(linearize(plant), 0.1, solver="exact")
    assert isinstance(exact, SampledSystem) and exact.kind == "model"
    problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant), tf=1.0)
    face = problem.compile(dt=0.1)
    step_problem = problem.discretize(0.1)
    x, u = np.array([0.2, 0.0]), np.zeros(1)
    np.testing.assert_array_equal(
        face.transition(x, u, 0)[0], step_problem.compile().transition(x, u, 0)[0]
    )


def test_a_cascade_is_one_flat_plan_on_one_base_clock():
    inner = (PID(Kp=4.0) % 0.001) @ Integrator()
    cascade = (PID(Kp=1.0) % 0.01) @ inner
    assert cascade.sample_period == 0.001
    plan = cascade.compile().plan
    assert plan.plant_op.label == "plant/plant"


def test_a_sampled_problem_takes_the_plants_substeps():
    plant = pendulum()
    problem = PlanningProblem(
        plant, cost=QuadraticCost.from_system(plant), x_start=plant.x0, tf=0.2
    )
    pe = problem.compile(pid() % 0.01, substeps=5)
    run = pe.trajectory()
    loop = (pid() % 0.01) @ pendulum()
    ticks = loop.compute_rollout(20, dt=0.002)
    a, b = loop.state_index["plant"]
    np.testing.assert_allclose(run.of("plant").x[:, -1], ticks.x[a:b, -1], rtol=1e-9)


def test_a_noise_source_holds_its_period_in_its_equation():
    noise = WhiteNoise(1, psd=1.0, sample_period=0.01)
    assert noise.solver_info["sample_period"] == 0.01


# The refusals, each with its message


def test_a_clocked_plant_under_a_continuous_law_is_refused():
    with pytest.raises(TypeError, match=r"\(controller % dt\) @ plant"):
        pid() @ discretize(Pendulum(), 0.01)


def test_two_continuous_systems_in_one_sampled_diagram_are_refused():
    both = SampledDiagramSystem()
    both.add_subsystem((P(1.0) % 0.01) @ Integrator(), "a")
    both.add_subsystem((P(1.0) % 0.01) @ Integrator(), "b")
    with pytest.raises(ValueError, match="compose them into one continuous system"):
        both.compile()


def test_a_tick_law_meeting_a_continuous_plant_with_no_clock_is_refused():
    with pytest.raises(TypeError, match="a tick law needs a clock"):
        UnitDelay() @ Integrator()


def test_a_closed_loop_on_the_right_of_a_chain_is_refused():
    with pytest.raises(TypeError, match="the @ binds first"):
        (DiscreteLowPass(0.6) % 0.01) >> (P(10.0) % 0.1) @ Integrator()


def test_the_step_knobs_on_a_loop_with_no_plant_are_refused():
    computer = (PID() % 0.01) >> (DiscreteLowPass() % 0.01)
    with pytest.raises(ValueError, match="no continuous plant"):
        computer.compute_trajectory(tf=0.1, u=np.ones(1), dt=0.001)
