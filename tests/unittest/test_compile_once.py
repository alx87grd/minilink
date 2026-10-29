"""Compile once, in the form the call needs: a held tool's repeats, sweeps and type changes cost no new compile."""

import copy

import numpy as np
import pytest

pytestmark = [pytest.mark.optional, pytest.mark.jax]

jax = pytest.importorskip("jax")

from minilink import (  # noqa: E402
    CartPole,
    DiagramSystem,
    DynamicSystem,
    Integrator,
    P,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Simulator,
    StateFeedback,
    System,
    TrajectoryOptimizationPlanner,
    lqr,
)
from minilink.core.backends import configure_jax  # noqa: E402

configure_jax(enable_x64=True)

COMPILES = []
jax.monitoring.register_event_duration_secs_listener(
    lambda name, secs, **kw: (
        COMPILES.append(secs)
        if name == "/jax/core/compile/backend_compile_duration"
        else None
    )
)

SIM = dict(tf=2.0, n_steps=201, solver="rk4_fixedsteps", verbose=False, backend="jax")
X0 = np.array([0.1, 0.2, 0.0, 0.0])


def compiles(call):
    """The XLA compilations ``call`` triggers, and what it returns."""
    COMPILES.clear()
    out = call()
    jax.block_until_ready(jax.tree_util.tree_leaves(out) if out is not None else 0)
    return len(COMPILES), out


def cartpole():
    plant = CartPole()
    plant.x0 = X0.copy()
    return plant


def pendulum_problem(tf=2.0):
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
    return PlanningProblem(
        plant, cost=cost, x_start=np.array([0.5, 0.0]), tf=tf, U=plant.input_box
    )


# A held tool's repeat costs no compile, pipeline by pipeline


def test_a_held_simulation_compiles_nothing_on_a_repeat():
    plant = cartpole()
    loop = lqr(plant, np.eye(4), np.eye(1)) @ plant
    sampled = (P(2.0) % 0.05) @ Integrator()
    options = dict(tf=2.0, n_steps=201, solver="rk4_fixedsteps", backend="jax")
    held = [
        Simulator(plant, **options),
        Simulator(plant, tf=2.0, solver="scipy", backend="jax"),
        Simulator(loop, **options),
    ]
    ev = sampled.compile(backend="jax")
    u = np.ones((20, 1))
    runs = [
        *(lambda sim=sim: sim.solve().x for sim in held),
        lambda: ev.rollout_dense(np.zeros(1), u, None, n_steps=20),
        lambda: ev.rollout(np.zeros(1), u, None, n_steps=20),
    ]
    for run in runs:
        run()
        assert compiles(run)[0] == 0


def test_a_convenience_call_compiles_each_time():
    plant = cartpole()
    plant.compute_trajectory(**SIM)
    assert compiles(lambda: plant.compute_trajectory(**SIM).x)[0] > 0


def test_a_repeated_evaluation_and_its_trajectory_compile_nothing():
    pe = pendulum_problem().compile(
        StateFeedback(np.array([[2.0, 0.5]])), dt=0.01, backend="jax"
    )
    for run in (
        lambda: pe.evaluate().objective,
        lambda: pe.evaluate(pe.params).objective,
        lambda: pe.trajectory().x,
    ):
        run()
        assert compiles(run)[0] == 0


def test_a_repeated_solve_compiles_nothing():
    plant = cartpole()
    problem = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        x_start=np.array([0.0, np.pi - 0.5, 0.0, 0.0]),
        tf=2.0,
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=20,
        transcription="direct_collocation",
        backend="jax",
        method="scipy_slsqp",
        optimizer_options={"maxiter": 20},
    )
    native = TrajectoryOptimizationPlanner(
        pendulum_problem(tf=1.0),
        n_steps=10,
        transcription="single_shooting",
        backend="jax",
        method="adam",
        iterations=20,
        optimizer_options={"learning_rate": 0.05},
    )
    pv = native.compile()
    start = {"x_start": np.array([0.5, 0.0]), "t_start": 0.0}
    for run in (
        lambda: planner.solve(evaluate=False).trajectory.x,
        lambda: native.solve(evaluate=False).trajectory.x,
        lambda: pv.solve(start, z0=pv.initial())[0],
    ):
        first = run()
        n, again = compiles(run)
        assert n == 0
        np.testing.assert_array_equal(np.asarray(first), np.asarray(again))


# One trace for every value of the parameters


def test_a_leaf_given_as_any_number_type_shares_one_trace():
    ev = cartpole().compile(backend="jax")
    tree = copy.deepcopy(ev.params)
    leaf = next(iter(tree))
    values = (9.0, np.float64(9.0), 9, np.array(9.0), jax.numpy.asarray(9.0))
    counts = []
    for value in values:
        tree[leaf] = value
        n, _ = compiles(lambda: ev.rollout(X0, None, 0.0, 0.01, tree, n_steps=50))
        counts.append(n)
    # The first new JAX scalar compiles the tiny cast that makes it strong, once (a
    # JAX 9.0 equal to the NumPy 9.0 before it is a repeat); the rollout itself
    # traces once for every value
    assert counts[0] == 1 and counts[1:4] == [0, 0, 0]
    tree[leaf] = jax.numpy.asarray(8.0)
    n, _ = compiles(lambda: ev.rollout(X0, None, 0.0, 0.01, tree, n_steps=50))
    assert counts[4] + n <= 1

    # A time given as an int or a float: the same trace
    ev.rollout(X0, None, 0.0, 0.01, n_steps=50)
    assert compiles(lambda: ev.rollout(X0, None, 0, 0.01, n_steps=50))[0] == 0


def test_an_evaluation_override_of_any_number_type_shares_one_trace():
    law = StateFeedback(np.array([[2.0, 0.5]]))
    pe = pendulum_problem().compile(law, dt=0.01, backend="jax")
    pe.evaluate({"x_start": np.array([0.4, 0.0])}).objective
    for value in (0.3, np.float64(0.3), 0, [0.3, 0.0]):
        start = value if isinstance(value, list) else np.array([value, 0.0])
        assert compiles(lambda: pe.evaluate({"x_start": start}).objective)[0] == 0


def test_a_parameter_sweep_through_a_held_evaluator_compiles_once_and_reads_each_value():
    plant = cartpole()
    ev = plant.compile(backend="jax")
    tree = copy.deepcopy(ev.params)
    name = next(iter(tree))
    ev.rollout(X0, None, 0.0, 0.01, tree, n_steps=200)
    total = 0
    for scale in (1.1, 1.2, 1.3):
        tree[name] = scale * np.asarray(plant.params[name])
        n, run = compiles(lambda: ev.rollout(X0, None, 0.0, 0.01, tree, n_steps=200))
        total += n
        twin = cartpole()
        twin.params = copy.deepcopy(tree)
        fresh = twin.compute_trajectory(**{**SIM, "backend": "numpy"}).x.T
        np.testing.assert_allclose(run, fresh, rtol=1e-9, atol=1e-12)
    assert total == 0


def test_a_repeated_gradient_through_a_compiled_method_never_reuses_a_tracer():
    ev = Integrator().compile(backend="jax")
    u = np.ones((5, 1))

    def last(p):
        return ev.rollout(np.zeros(1), u, 0.0, 0.1, p)[-1, 0]

    grad = jax.grad(last)
    for _ in range(50):
        grad(dict(ev.params))
    jax.vmap(lambda k: last({**ev.params, "k": k}))(np.ones(3))
    assert ev.on_device.entry is None

    # A host tree put on the device inside an outer jit is traced too: never kept
    host = {"k": np.array(2.0)}
    batch = jax.jit(jax.vmap(lambda x0: ev.rollout(x0, u, 0.0, 0.1, host)[-1]))
    batch(np.zeros((3, 1)))
    assert ev.on_device.entry is None
    np.testing.assert_allclose(ev.rollout(np.zeros(1), u, 0.0, 0.1, host)[-1], [1.0])


class Decay(DynamicSystem):
    """``dx = −a x``, ``a`` written as an integer, as a student writes it."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.params = {"a": 1}
        self.x0 = np.array([1.0])

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        return -params["a"] * x


def test_a_float_for_an_integer_default_is_never_truncated():
    plant = Decay()
    run = dict(tf=1.0, n_steps=11, solver="rk4_fixedsteps", verbose=False)
    plant.compute_trajectory(**run, backend="jax")
    plant.params["a"] = 2.5
    jax_x = plant.compute_trajectory(**run, backend="jax").x[0, -1]
    numpy_x = plant.compute_trajectory(**run, backend="numpy").x[0, -1]
    assert jax_x == pytest.approx(numpy_x, rel=1e-12)
    assert numpy_x == pytest.approx(np.exp(-2.5), rel=1e-3)


# The frozen and the parametric forms


def test_the_frozen_and_the_parametric_forms_agree():
    ev = cartpole().compile(backend="jax")
    frozen = ev.rollout(X0, None, 0.0, 0.01, n_steps=100)
    parametric = ev.rollout(X0, None, 0.0, 0.01, ev.params, n_steps=100)
    np.testing.assert_allclose(frozen, parametric, rtol=1e-12, atol=1e-14)


class Weighted(DynamicSystem):
    """``dx = −x + mean(w) u``: a block with a large parameter, as a network's weights."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"w": np.ones(5_000)}

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = jax.numpy if not isinstance(x, np.ndarray) else np
        w = params["w"]
        return -x + xp.mean(w) * u


def test_a_large_frozen_leaf_is_an_argument_not_a_constant():
    ev = Weighted().compile(backend="jax")

    # First reached inside an outer jit: the traced tree is not kept
    jax.jit(jax.vmap(lambda x: ev.f(x, np.ones(1))))(np.zeros((3, 1)))
    assert ev.frozen_on_device is None
    assert ev.call_tree(None) is not None
    frozen = ev.rollout(np.zeros(1), np.ones((10, 1)), 0.0, 0.1)
    parametric = ev.rollout(np.zeros(1), np.ones((10, 1)), 0.0, 0.1, ev.params)
    np.testing.assert_allclose(frozen, parametric, rtol=1e-12)


# A convenience call compiles each time, so it sees every edit


class Helper:
    """A plain object a model holds: a lookup table."""

    def __init__(self, gain):
        self.gain = gain


class Leak(DynamicSystem):
    """``dx/dt = −k x + r(t)``, ``k`` on a helper and ``r`` a function stored on the block."""

    def __init__(self):
        super().__init__(1, input_dim=1)
        self.x0 = np.array([0.0])
        self.r = lambda t: 1.0
        self.helper = Helper(1.0)
        self.table = jax.numpy.array([1.0])

    def f(self, x, u, t=0, params=None):
        return -self.helper.gain * self.table[0] * x + self.r(t)


def final(sys, backend):
    run = dict(SIM, backend=backend, tf=1.0, n_steps=101)
    return float(np.asarray(sys.compute_trajectory(**run).x)[0, -1])


@pytest.mark.parametrize(
    "edit",
    [
        lambda s: setattr(s, "r", lambda t: 5.0),  # a function swapped
        lambda s: setattr(s, "r", lambda t: 2.0),  # and another: no id reuse
        lambda s: setattr(s.helper, "gain", 10.0),  # inside a held helper
        lambda s: setattr(s, "helper", Helper(0.1)),  # a helper swapped
        lambda s: setattr(s, "table", jax.numpy.array([3.0])),  # a JAX array
    ],
)
def test_an_edit_after_a_jax_run_is_seen_by_the_next(edit):
    sys = Leak()
    final(sys, "jax")
    edit(sys)
    assert final(sys, "jax") == pytest.approx(final(sys, "numpy"), rel=1e-9)


def test_a_class_method_replaced_is_seen_by_the_next_call():
    class Drift(DynamicSystem):
        def __init__(self):
            super().__init__(1, input_dim=1)
            self.x0 = np.array([0.0])

        def f(self, x, u, t=0, params=None):
            return x * 0.0 + 1.0

    sys = Drift()
    final(sys, "jax")
    Drift.f = lambda self, x, u, t=0, params=None: x * 0.0 + 2.0
    assert final(sys, "jax") == pytest.approx(final(sys, "numpy"))


def test_an_embedded_models_numbers_are_seen_by_the_next_call():
    from minilink.control.modelbased import ComputedTorque

    plant = Pendulum()
    plant.x0 = np.array([0.5, 0.0])
    model = copy.deepcopy(plant)
    loop = ComputedTorque(model, tracking_ref=False) @ plant
    run = dict(tf=1.0, n_steps=101, solver="rk4_fixedsteps", verbose=False)
    loop.compute_trajectory(**run, backend="jax")
    model.params["m"] = 3.0 * np.asarray(model.params["m"])
    np.testing.assert_allclose(
        loop.compute_trajectory(**run, backend="jax").x,
        loop.compute_trajectory(**run, backend="numpy").x,
        rtol=1e-9,
        atol=1e-12,
    )


class Sink(System):
    """A block with one input, no state and no output."""

    def __init__(self):
        super().__init__(0)
        self.add_input_port("u", dim=1, nominal_value=np.zeros(1))
        self.params = {"gain": 1.0}


def test_a_parameter_edit_runs_on_a_diagram_holding_a_sink():
    diagram = DiagramSystem()
    plant = Integrator()
    plant.x0 = np.array([1.0])
    diagram.add_subsystem(plant, "plant")
    diagram.add_subsystem(Sink(), "sink")
    diagram.connect("plant", "y", "sink", "u")
    run = dict(tf=1.0, n_steps=11, solver="rk4_fixedsteps", verbose=False)
    diagram.compute_trajectory(**run, backend="jax")
    plant.params["k"] = 2.0
    np.testing.assert_allclose(
        diagram.compute_trajectory(**run, backend="jax").x,
        diagram.compute_trajectory(**run, backend="numpy").x,
        rtol=1e-12,
    )


def test_a_new_horizon_before_a_solve_is_a_new_program():
    problem = pendulum_problem(tf=1.0)
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=10,
        transcription="direct_collocation",
        backend="jax",
        method="scipy_slsqp",
        optimizer_options={"maxiter": 10},
    )
    assert planner.solve(evaluate=False).trajectory.t[-1] == pytest.approx(1.0)
    problem.tf = 2.0
    assert planner.solve(evaluate=False).trajectory.t[-1] == pytest.approx(2.0)


def test_a_bound_tree_is_what_the_program_reads():
    from minilink.optimization import MathematicalProgram
    from minilink.optimization.evaluators.compiler import compile_program_evaluator

    program = MathematicalProgram(
        n_z=2,
        J=lambda z, p: (z[0] - p["c"]) ** 2 + z[1] ** 2,
        params={"c": np.array(1.0)},
    )
    evaluator = compile_program_evaluator(program, "jax")
    z = np.zeros(2)
    assert float(evaluator.J(z)) == 1.0
    evaluator.bind({"c": 3})
    assert float(evaluator.J(z)) == 9.0
    n, _ = compiles(lambda: (evaluator.bind({"c": 2.0}), evaluator.J(z))[1])
    assert n == 0 and float(evaluator.J(z)) == 4.0


def test_a_certificate_verifies_twice_on_one_compile():
    from minilink.analysis.lyapunov import region_of_attraction

    plant = Pendulum()
    plant.params["d"] = 0.5
    roa = region_of_attraction(plant)
    roa.verify(n=20)
    assert compiles(lambda: roa.verify(n=20).converged)[0] == 0


def test_an_edit_of_the_programs_numbers_reaches_the_frozen_form():
    from minilink.optimization import MathematicalProgram
    from minilink.optimization.evaluators.compiler import compile_program_evaluator

    program = MathematicalProgram(
        n_z=2,
        J=lambda z, p: (z[0] - p["a"]) ** 2 + (z[1] - p["b"]) ** 2,
        params={"a": np.array(1.0), "b": np.array(2.0)},
    )
    evaluator = compile_program_evaluator(program, "jax")
    z = np.zeros(2)
    evaluator.bind(None)
    assert float(evaluator.J(z)) == 5.0
    program.params["a"] = np.array(5.0)
    evaluator.bind(None)
    assert float(evaluator.J(z)) == 29.0


def test_a_list_leaf_runs_on_jax_like_an_array():
    class Decay(DynamicSystem):
        def __init__(self):
            super().__init__(n=1, input_dim=0, output_dim=1)
            self.params = {"k": [1.0]}
            self.x0 = np.array([1.0])

        def f(self, x, u, t=0.0, params=None):
            params = self.params if params is None else params
            k = params["k"]
            dx = -k[0] * x
            return dx

    on_jax = Decay().compute_trajectory(tf=1.0, dt=0.1, backend="jax").x[0, -1]
    on_numpy = Decay().compute_trajectory(tf=1.0, dt=0.1, backend="numpy").x[0, -1]
    np.testing.assert_allclose(on_jax, on_numpy, rtol=1e-12)
