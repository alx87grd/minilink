"""Tests for :mod:`minilink.core.backends`."""

import importlib
import pytest
from minilink.core.backends import (
    BACKEND_AUTO,
    BACKEND_DIRECT,
    BACKEND_JAX,
    BACKEND_NUMPY,
    COMPILE_BACKENDS,
    SIMULATOR_BACKENDS,
    TRANSCRIPTION_BACKENDS,
    normalize_backend,
    require_jax_numpy,
)


def test_compile_simulator_transcription_backend_sets():
    assert set(COMPILE_BACKENDS) == {BACKEND_NUMPY, BACKEND_JAX}
    assert set(SIMULATOR_BACKENDS) == {BACKEND_NUMPY, BACKEND_JAX, BACKEND_AUTO}
    assert set(TRANSCRIPTION_BACKENDS) == {BACKEND_NUMPY, BACKEND_JAX, BACKEND_DIRECT}


def test_normalize_backend_lowercases_and_validates():
    assert normalize_backend("NumPy") == BACKEND_NUMPY
    assert normalize_backend(" JAX ") == BACKEND_JAX
    assert normalize_backend(None) == BACKEND_NUMPY


def test_normalize_backend_rejects_unknown():
    with pytest.raises(ValueError):
        normalize_backend("torch")


def test_normalize_backend_auto_gated():
    with pytest.raises(ValueError):
        normalize_backend("auto")
    assert normalize_backend("auto", allow_auto=True) == BACKEND_AUTO


def test_normalize_backend_direct_gated():
    with pytest.raises(ValueError):
        normalize_backend("direct")
    assert normalize_backend("direct", allow_direct=True) == BACKEND_DIRECT


def test_require_jax_numpy_returns_module_when_jax_installed():
    pytest.importorskip("jax")
    jnp = require_jax_numpy()
    assert jnp.zeros(2).shape == (2,)


def test_simulator_re_exports_compile_backend_auto():
    sim = importlib.import_module("minilink.simulation.simulator")
    assert sim.COMPILE_BACKEND_AUTO == BACKEND_AUTO


import numpy as np
from minilink.core.costs import QuadraticCost
from minilink.core.sets import (
    BallSet,
    BoxSet,
    CallableSet,
    IntersectionSet,
    SingletonSet,
)
from minilink.core.trajectory import Trajectory


def _quadratic_cost() -> QuadraticCost:
    return QuadraticCost(
        Q=np.eye(2),
        R=np.eye(1),
        S=2.0 * np.eye(2),
        xbar=np.array([1.0, -1.0]),
        ubar=np.array([0.5]),
    )


def test_set_margins_are_numpy_arrays():
    z = np.array([0.25, -0.5])
    box = BoxSet(lower=np.array([-1.0, -2.0]), upper=np.array([1.0, 2.0]))
    singleton = SingletonSet(np.array([1.0, -1.0]))
    ball = BallSet(center=np.zeros(2), radius=1.0)
    intersection = IntersectionSet([box, ball])
    np.testing.assert_allclose(box.margin(z), [1.25, 1.5, 0.75, 2.5])
    np.testing.assert_allclose(singleton.residual(z), [-0.75, 0.5])
    np.testing.assert_allclose(singleton.margin(z), [-0.75, -0.5])
    np.testing.assert_allclose(ball.margin(z), [1.0 - np.linalg.norm(z)])
    np.testing.assert_allclose(
        intersection.margin(z), np.concatenate((box.margin(z), ball.margin(z)))
    )


def test_callable_set_margin_preserves_numpy_output():
    set_ = CallableSet(
        lambda z, t, params: np.array([z[0] + t, params["limit"] - z[1]])
    )
    margin = set_.margin(np.array([1.0, 2.0]), t=0.5, params={"limit": 3.0})
    np.testing.assert_allclose(margin, [1.5, 1.0])


def test_quadratic_cost_numpy_math_and_reporting_helpers():
    cost = _quadratic_cost()
    x = np.array([2.0, 1.0])
    u = np.array([1.5])
    assert np.isclose(cost.g(x, u), 6.0)
    assert np.isclose(cost.h(x), 10.0)
    traj = Trajectory(
        t=np.array([0.0, 1.0]), x=np.column_stack((x, x)), u=np.column_stack((u, u))
    )
    evaluated = cost.evaluate_trajectory(traj)
    assert isinstance(cost.terminal_cost(traj), float)
    assert isinstance(cost.total_cost(traj), float)
    np.testing.assert_allclose(evaluated.signals["cost_rate"], [[6.0, 6.0]])
    np.testing.assert_allclose(evaluated.signals["cost"], [[0.0, 6.0]])


def test_set_margins_are_jax_jittable():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp
    from minilink.core.backends import configure_jax

    configure_jax(enable_x64=True)
    box = BoxSet(lower=np.array([-1.0, -2.0]), upper=np.array([1.0, 2.0]))
    singleton = SingletonSet(np.array([1.0, -1.0]))
    ball = BallSet(center=np.zeros(2), radius=1.0)
    callable_set = CallableSet(lambda z, t, params: jnp.array([z[0] + t]))
    intersection = IntersectionSet([box, ball])
    z = jnp.asarray([0.25, -0.5])
    np.testing.assert_allclose(
        np.asarray(jax.jit(box.margin)(z)), box.margin(np.asarray(z))
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(singleton.residual)(z)), singleton.residual(np.asarray(z))
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(singleton.margin)(z)), singleton.margin(np.asarray(z))
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(ball.margin)(z)), ball.margin(np.asarray(z))
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(callable_set.margin)(z, 0.5, None)), [0.75]
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(intersection.margin)(z)), intersection.margin(np.asarray(z))
    )


def test_quadratic_cost_is_jax_jittable():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp
    from minilink.core.backends import configure_jax

    configure_jax(enable_x64=True)
    cost = _quadratic_cost()
    x = jnp.asarray([2.0, 1.0])
    u = jnp.asarray([1.5])

    def J(x, u):
        return cost.g(x, u) + cost.h(x)

    assert np.isclose(float(jax.jit(J)(x, u)), 16.0)


# JAX 64-bit precision policy (S05)


def test_jax_x64_policy_env_var(monkeypatch):
    from minilink.core.backends import jax_x64_policy

    monkeypatch.delenv("MINILINK_JAX_X64", raising=False)
    assert jax_x64_policy()
    for off in ("0", "false", "No", "off"):
        monkeypatch.setenv("MINILINK_JAX_X64", off)
        assert not jax_x64_policy()
    monkeypatch.setenv("MINILINK_JAX_X64", "1")
    assert jax_x64_policy()


def _run_fresh_python(code, env_overrides):
    """Run ``code`` in a fresh interpreter so module-level JAX config cannot leak in."""
    import os
    import subprocess
    import sys

    env = dict(os.environ)
    env.pop("MINILINK_JAX_X64", None)
    env.update(env_overrides)
    env["PYTHONPATH"] = os.getcwd()
    env["MPLBACKEND"] = "Agg"
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert result.returncode == 0, result.stderr[-2000:]
    return result.stdout.strip().splitlines()[-1]


_X64_PROBE = """
import warnings; warnings.filterwarnings("ignore")
import jax, numpy as np
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
ev = Pendulum().compile(backend="jax")
dx = ev.f(np.zeros(2), np.zeros(1), 0.0)
print(jax.config.jax_enable_x64, dx.dtype)
"""

_TRAJOPT_PROBE = """
import warnings; warnings.filterwarnings("ignore")
import numpy as np
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import TrajectoryOptimizationPlanner
p = Pendulum()
p.inputs["u"].lower_bound = np.array([-20.0]); p.inputs["u"].upper_bound = np.array([20.0])
goal = np.array([np.pi, 0.0])
problem = PlanningProblem(p, x_start=np.zeros(2), x_goal=goal, tf=3.0,
    cost=QuadraticCost.from_system(p, Q=np.eye(2), R=np.eye(1), xbar=goal))
plan = TrajectoryOptimizationPlanner(problem, n_steps=30, transcription="direct_collocation",
    compile_backend="jax").solve()
print(plan.metadata.success)
"""


@pytest.mark.optional
@pytest.mark.jax
def test_jax_evaluator_is_float64_by_default_in_a_fresh_process():
    pytest.importorskip("jax")
    assert _run_fresh_python(_X64_PROBE, {}) == "True float64"


@pytest.mark.optional
@pytest.mark.jax
def test_jax_x64_opt_out_env_var_keeps_float32():
    pytest.importorskip("jax")
    assert _run_fresh_python(_X64_PROBE, {"MINILINK_JAX_X64": "0"}) == "False float32"


@pytest.mark.optional
@pytest.mark.jax
def test_trajopt_succeeds_on_jax_without_caller_enabling_x64():
    pytest.importorskip("jax")
    assert _run_fresh_python(_TRAJOPT_PROBE, {}) == "True"
