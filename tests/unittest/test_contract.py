"""One contract for every core math object: its ``params``, ``params=None`` its defaults, a gradient through a float leaf, the same values twice.

Each case is a public core object and one call of its mathematics, ``call(obj, params, xp)``,
whose arguments are built on the array module ``xp``. The written exceptions are DESIGN §4's
list ("Parameters"), each with its reason; they are not walked here.
"""

import inspect
import warnings

import numpy as np
import pytest

import minilink
import minilink.core as core
from minilink.core.discrete.step_system import StepSystem
from minilink.core.params import flatten, with_leaf
from minilink.core.system import DynamicSystem, System

# The public systems a walk cannot build with no arguments, and the ones the contract excepts
SYSTEM_FACTORIES = {
    "Gain": lambda: minilink.Gain(np.array([[2.0, -1.0]])),
    "OutputReference": lambda: minilink.OutputReference(
        np.array([[1.0, 0.5]]), np.array([[2.0]])
    ),
    "Source": lambda: minilink.Source(2),
    "StateFeedback": lambda: minilink.StateFeedback(np.array([[1.0, 0.5]])),
    "StateObserver": lambda: minilink.StateObserver(
        np.array([[0.0, 1.0], [-2.0, -0.5]]),
        np.array([[0.0], [1.0]]),
        np.array([[1.0, 0.0]]),
        np.array([[3.0], [2.0]]),
    ),
    "TimeVaryingFeedback": lambda: minilink.TimeVaryingFeedback(
        np.array([0.0, 1.0]), np.array([[[1.0, 0.5]], [[2.0, 1.0]]])
    ),
    "TrajectorySource": lambda: minilink.TrajectorySource(
        np.array([0.0, 0.5, 1.0]), np.array([0.0, 1.0, 0.5])
    ),
    "TransferFunction": lambda: minilink.TransferFunction([1.0], [1.0, 2.0, 1.0]),
    "ComputedTorque": lambda: minilink.ComputedTorque(minilink.Pendulum()),
    "SlidingMode": lambda: minilink.SlidingMode(minilink.Pendulum()),
    "JointImpedance": lambda: minilink.JointImpedance(minilink.Pendulum()),
    "TaskImpedance": lambda: minilink.TaskImpedance(minilink.TwoLinkManipulator()),
    "TaskKinematic": lambda: minilink.TaskKinematic(
        minilink.KinematicManipulator.from_manipulator(minilink.TwoLinkManipulator())
    ),
    "TaskKinematicNullspace": lambda: minilink.TaskKinematicNullspace(
        minilink.KinematicManipulator(3, 2)
    ),
    "KinematicManipulator": lambda: minilink.KinematicManipulator(2, 2),
    "PurePursuit": lambda: minilink.PurePursuit(
        np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]])
    ),
    "TrajectoryFeedback": lambda: minilink.TrajectoryFeedback(
        minilink.Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0], [0.0, 0.5]]),
            u=np.array([[0.0, 0.2]]),
        ),
        np.array([[[1.0, 0.5]], [[2.0, 1.0]]]),
    ),
    "MLP": lambda: minilink.MLP(2, 1, hidden=(4,)),
    "LookupPolicy": lambda: lookup_policy(),
    "NeuralPolicy": lambda: minilink.NeuralPolicy(minilink.Pendulum(), hidden=(4,)),
}
SYSTEM_EXCEPTIONS = {
    # The abstract shells and the diagrams: their params are their blocks' (walked as blocks)
    "System",
    "DynamicSystem",
    "StepSystem",
    "DiagramSystem",
    "StepDiagramSystem",
}


def lookup_policy():
    """A dynamic-programming policy table on a small pendulum grid: its node ``i`` takes the action ``i mod 3``."""
    from minilink.planning.dynamic_programming.discretizer import StateSpaceGrid

    plant = minilink.Pendulum()
    problem = minilink.PlanningProblem(plant, x_start=np.zeros(2), U=plant.input_box)
    grid = StateSpaceGrid(problem, x_grid_shape=(5, 5), u_grid_shape=(3,), dt=0.1)

    return minilink.LookupPolicy(grid, np.arange(25) % 3)


def public_systems():
    names = sorted(set(minilink.__all__) | set(core.__all__))
    systems = []
    for name in names:
        cls = getattr(minilink, name, None) or getattr(core, name, None)
        if not (inspect.isclass(cls) and issubclass(cls, System)):
            continue
        if name in SYSTEM_EXCEPTIONS:
            continue
        systems.append(name)

    return systems


def build_system(name):
    cls = getattr(minilink, name, None) or getattr(core, name)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return SYSTEM_FACTORIES[name]() if name in SYSTEM_FACTORIES else cls()


# A system whose law is singular near the origin (a straight arm) is called at a regular point
POINT_OFFSETS = {"TaskKinematic": 0.8}


def system_call(sys, params, xp):
    """``f`` (or ``step``) and every output port at one point, stacked."""
    rng = np.random.default_rng(0)
    offset = POINT_OFFSETS.get(type(sys).__name__, 0.1)
    x = xp.asarray(offset + 0.1 * rng.standard_normal(sys.n))
    u = xp.asarray(offset + 0.1 * rng.standard_normal(sys.m))

    parts = []
    if isinstance(sys, StepSystem):
        parts.append(sys.step(x, u, 0, params))
    elif isinstance(sys, DynamicSystem) and sys.n > 0:
        parts.append(sys.f(x, u, 0.1, params))
    for port in sys.outputs.values():
        parts.append(port.compute(x, u, 0.1, params))

    return xp.concatenate(
        [xp.reshape(xp.asarray(p, dtype=float), (-1,)) for p in parts]
    )


def other_cases():
    """``(name, object, call)`` for the costs, sets, distributions, fields, shapes and paths."""
    from minilink import Gaussian, Pendulum, QuadraticCost, TimeCost, Uniform
    from minilink.core.distributions import Particles
    from minilink.core.fields import QuadraticField
    from minilink.core.geometry import Box, Sphere
    from minilink.core.sets import (
        BallSet,
        BoxInputSet,
        BoxSet,
        CallableSet,
        SingletonSet,
    )
    from minilink.planning.spatial import ReferenceTrack, from_waypoints

    plant = Pendulum()
    quadratic = QuadraticCost.from_system(plant, xbar=np.array([1.0, 0.0]))
    V = QuadraticField([0.1, -0.1], [[2.0, 0.3], [0.3, 1.0]])
    schedule = QuadraticField(
        [0.0, 0.0], np.stack([np.eye(2), 3.0 * np.eye(2)]), t=np.array([0.0, 1.0])
    )
    disc = Sphere([0.0, 1.0], 0.5)
    wall = Box([1.0, -1.0], [2.0, 1.0])
    path = from_waypoints([[0.0, 0.0], [1.0, 0.0], [2.0, 1.0]])
    box = BoxSet([-1.0, -2.0], [1.0, 2.0])
    ball = BallSet([0.0, 0.5], 1.5)

    def cost_call(cost, params, xp):
        x, u = xp.asarray([0.3, -0.2]), xp.asarray([0.4])
        return xp.stack(
            [
                xp.asarray(cost.g(x, u, 0.1, params), dtype=float),
                xp.asarray(cost.h(x, 0.1, params), dtype=float),
            ]
        )

    def set_call(set_, params, xp):
        return set_.margin(xp.asarray([0.3, -0.2]), 0.0, params)

    def input_set_call(set_, params, xp):
        return set_.margin(xp.asarray([0.4]), xp.asarray([0.3, -0.2]), 0.0, params)

    def shape_call(shape, params, xp):
        return xp.stack([shape.sdf(xp.asarray([0.3, -0.2]), 0.0, params)])

    def path_call(path, params, xp):
        return xp.stack([path.distance(xp.asarray([0.3, 0.2]), 0.0, params)])

    def corridor_call(track, params, xp):
        return xp.stack([track.corridor_margin(xp.asarray([0.3, 0.2]), 0.0, params)])

    def law_call(law, params, xp):
        if xp is np:
            return law.sample(7, 3, params)
        import jax

        return law.sample(jax.random.PRNGKey(7), 3, params)

    return [
        ("QuadraticCost", quadratic, cost_call),
        ("TimeCost", TimeCost([1.0, 0.0], eps=0.5), cost_call),
        ("SumCost", quadratic + 2.0 * QuadraticCost.from_system(plant), cost_call),
        ("Set.as_cost", box.as_cost(weight=3.0), cost_call),
        ("Field.as_cost", V.as_cost(weight=2.0), cost_call),
        ("Field.as_terminal_cost", V.as_terminal_cost(weight=2.0), cost_call),
        ("Field schedule.as_cost", schedule.as_cost(), cost_call),
        ("Field.as_set", V.as_set(lower=0.1, upper=2.0), set_call),
        ("Field.as_input_set", V.as_input_set(upper=2.0), input_set_call),
        ("Sphere", disc, shape_call),
        ("Box", wall, shape_call),
        ("Union", disc | wall, shape_call),
        ("Inflated", wall.inflate(0.2), shape_call),
        ("PolylinePath", path, path_call),
        ("ReferenceTrack", ReferenceTrack(path, half_width=0.4), corridor_call),
        ("BoxSet", box, set_call),
        ("BallSet", ball, set_call),
        ("SingletonSet", SingletonSet([0.3, 0.0]), set_call),
        ("IntersectionSet", box & ball, set_call),
        ("BoxInputSet", BoxInputSet.from_bounds([-1.0], [1.0]), input_set_call),
        (
            "CallableSet",
            CallableSet(lambda z, t, p: p["r"] - z @ z, params={"r": np.asarray(1.0)}),
            set_call,
        ),
        ("Gaussian", Gaussian([0.0, 1.0], [1.0, 0.5]), law_call),
        (
            "Gaussian(cov=)",
            Gaussian([0.0, 1.0], cov=[[1.0, 0.3], [0.3, 0.5]]),
            law_call,
        ),
        ("Uniform", Uniform([-1.0, 0.0], [1.0, 2.0]), law_call),
        ("Particles", Particles([[0.0, 0.0], [1.0, 2.0], [3.0, 1.0]]), law_call),
    ]


CASES = [(name, None, system_call) for name in public_systems()] + other_cases()


def case_of(name, obj, call):
    return (build_system(name) if obj is None else obj), call


@pytest.mark.parametrize(("name", "obj", "call"), CASES, ids=[c[0] for c in CASES])
def test_params_none_reads_the_defaults_and_twice_is_the_same(name, obj, call):
    obj, call = case_of(name, obj, call)
    assert isinstance(obj.params, dict), f"{name}.params is {type(obj.params).__name__}"

    first = np.asarray(call(obj, None, np))
    with_defaults = np.asarray(call(obj, obj.params, np))
    again = np.asarray(call(obj, None, np))

    np.testing.assert_allclose(with_defaults, first, rtol=1e-12, atol=1e-15)
    np.testing.assert_array_equal(again, first)


@pytest.mark.jax
@pytest.mark.parametrize(("name", "obj", "call"), CASES, ids=[c[0] for c in CASES])
def test_jax_agrees_and_a_float_leaf_has_a_gradient(name, obj, call):
    pytest.importorskip("jax")
    from minilink.core.backends import ensure_jax_x64

    jax = ensure_jax_x64()
    jnp = jax.numpy

    obj, call = case_of(name, obj, call)
    params = obj.params

    # The same numbers on JAX (a law's draws are the key's backend's, not compared)
    on_jax = np.asarray(call(obj, None, jnp))
    if "Gaussian" not in name and name not in ("Uniform", "Particles"):
        np.testing.assert_allclose(
            on_jax, np.asarray(call(obj, None, np)), rtol=1e-9, atol=1e-12
        )

    # A gradient through the first float leaf, finite
    leaves = [
        (keys, leaf)
        for keys, leaf in flatten(params)
        if np.issubdtype(np.asarray(leaf).dtype, np.floating)
    ]
    if not leaves:
        return
    keys, leaf = leaves[0]

    def total(value):
        return jnp.sum(call(obj, with_leaf(params, keys, value), jnp))

    grad = jax.grad(total)(jnp.asarray(leaf, dtype=float))
    assert np.all(np.isfinite(np.asarray(grad))), f"{name}: d/d{'.'.join(keys)}"


def test_a_law_at_other_numbers_is_built_from_them():
    from minilink.core.distributions import Gaussian, Particles, Uniform

    for law, moved in (
        (Gaussian(np.zeros(2), np.ones(2)), {"mean": np.ones(2), "std": np.ones(2)}),
        (Uniform([0.0], [1.0]), {"lower": np.array([2.0]), "upper": np.array([3.0])}),
        (Particles(np.zeros((3, 1))), {"points": np.ones((3, 1))}),
    ):
        other = law.with_params(moved)
        assert type(other) is type(law) and other is not law
        for key, value in moved.items():
            np.testing.assert_array_equal(other.params[key], value)
