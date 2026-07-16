import numpy as np
import pytest

from minilink.core.costs import (
    CostFunction,
    QuadraticCost,
    ScaledCost,
    SumCost,
    TimeCost,
)
from minilink.core.trajectory import Trajectory


def test_time_cost_is_unity_off_target_zero_on_target():
    cost = TimeCost(xbar=np.zeros(2), eps=0.1)
    assert cost.g(np.array([1.0, 0.0]), np.zeros(1)) == 1.0
    assert cost.g(np.array([0.02, 0.0]), np.zeros(1)) == 0.0
    assert cost.h(np.array([1.0, 0.0])) == 0.0


def test_time_cost_jax_matches_numpy():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    cost = TimeCost(xbar=np.zeros(2), eps=0.1)
    states = jnp.array([[1.0, 0.0], [0.02, 0.0], [0.0, 0.0]])
    u = jnp.zeros(1)

    def g_at(x):
        return cost.g(x, u)

    jax_values = jax.vmap(g_at)(states)
    numpy_values = np.array([cost.g(np.asarray(x), np.zeros(1)) for x in states])
    assert np.allclose(np.asarray(jax_values), numpy_values)


def make_quadratic(n: int = 2, m: int = 1) -> QuadraticCost:
    return QuadraticCost(
        Q=np.eye(n),
        R=np.eye(m),
        S=np.zeros((n, n)),
        xbar=np.zeros(n),
        ubar=np.zeros(m),
    )


class _ParamCost(CostFunction):
    """Running cost that reads ``params['w']`` to check forwarding."""

    def g(self, x, u, t=0.0, params=None):
        return 0.0 if params is None else float(params["w"])

    def h(self, x, t=0.0, params=None):
        return 0.0


def test_sum_cost_is_additive():
    c1 = make_quadratic()
    c2 = make_quadratic()
    s = c1 + c2
    x = np.array([1.0, 2.0])
    u = np.array([3.0])

    assert isinstance(s, SumCost)
    assert s.g(x, u) == pytest.approx(c1.g(x, u) + c2.g(x, u))
    assert s.h(x) == pytest.approx(c1.h(x) + c2.h(x))


def test_add_flattens_nested_sums():
    c = make_quadratic()
    s = c + c + c

    assert isinstance(s, SumCost)
    assert len(s.terms) == 3


def test_sum_builtin_uses_radd_zero_seed():
    costs = [make_quadratic() for _ in range(3)]
    s = sum(costs)  # 0 + c0 + c1 + c2
    x = np.array([1.0, 2.0])
    u = np.array([3.0])

    assert isinstance(s, SumCost)
    assert s.g(x, u) == pytest.approx(3.0 * costs[0].g(x, u))


def test_scaled_cost_weights():
    c = make_quadratic()
    x = np.array([1.0, 2.0])
    u = np.array([3.0])

    weighted = 2.5 * c
    assert isinstance(weighted, ScaledCost)
    assert weighted.g(x, u) == pytest.approx(2.5 * c.g(x, u))

    combo = c + 2.0 * c  # reads as base + weight * other
    assert combo.g(x, u) == pytest.approx(3.0 * c.g(x, u))


def test_params_forwarded_to_terms():
    s = make_quadratic() + _ParamCost()
    x = np.array([1.0, 2.0])
    u = np.array([3.0])
    base = make_quadratic().g(x, u)

    assert s.g(x, u, params={"w": 5.0}) == pytest.approx(base + 5.0)
    assert (3.0 * _ParamCost()).g(x, u, params={"w": 2.0}) == pytest.approx(6.0)


def test_total_cost_on_trajectory():
    c = make_quadratic()
    s = c + c
    t = np.linspace(0.0, 1.0, 5)
    traj = Trajectory(t=t, x=np.ones((2, 5)), u=np.ones((1, 5)))

    assert s.total_cost(traj) == pytest.approx(2.0 * c.total_cost(traj))


def test_jax_twin_cost_grad():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    s = make_quadratic() + make_quadratic()

    def g(x):
        return s.g(x, jnp.zeros(1))

    gradient = jax.grad(g)(jnp.array([1.0, 2.0]))
    assert np.asarray(gradient).shape == (2,)


# --- merged from test_optimizer.py ---

"""Tests for :class:`~minilink.optimization.optimizer.Optimizer` backends."""

import numpy as np
import pytest

from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizer import Optimizer


def _quadratic_program() -> MathematicalProgram:
    def J(z: np.ndarray):
        return z[0] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * z[0]])

    return MathematicalProgram(n_z=1, J=J, grad_J=grad_J)


def test_scipy_minimize_record_solve_time_default_none():
    prog = _quadratic_program()
    opt = Optimizer(
        prog,
        z0=np.array([1.0]),
        method="scipy_slsqp",
        options={"disp": False, "maxiter": 50},
    )
    out = opt.solve()
    assert out.solve_time_s is None


def test_scipy_minimize_record_solve_time_sets_duration():
    prog = _quadratic_program()
    opt = Optimizer(
        prog,
        z0=np.array([1.0]),
        method="scipy_slsqp",
        options={"disp": False, "maxiter": 50},
    )
    out = opt.solve(record_solve_time=True)
    assert out.solve_time_s is not None
    assert isinstance(out.solve_time_s, float)
    assert out.solve_time_s >= 0.0
    assert out.success


def test_scipy_minimize_disp_prints_report(capsys):
    prog = _quadratic_program()
    opt = Optimizer(
        prog,
        z0=np.array([1.0]),
        method="scipy_slsqp",
        options={"disp": False, "maxiter": 50},
    )
    out = opt.solve(disp=True)
    captured = capsys.readouterr()
    assert "Optimization Program" in captured.out
    assert "method=" in captured.out
    assert "Running solver" in captured.out
    assert "success:" in captured.out
    assert "Completed in" in captured.out
    assert out.solve_time_s is not None
    assert out.success


def test_scipy_minimize_progress_callback_z_J_t():
    samples: list[tuple[np.ndarray, float, float]] = []

    prog = _quadratic_program()
    opt = Optimizer(
        prog,
        z0=np.array([1.0]),
        method="scipy_slsqp",
        options={"disp": False, "maxiter": 50},
    )

    def prog_cb(z: np.ndarray, Jv: float, t: float) -> None:
        samples.append((z.copy(), Jv, t))

    out = opt.solve(callback=prog_cb)
    assert out.success
    assert len(samples) >= 1
    for z, Jv, t in samples:
        assert z.shape == (1,)
        assert Jv == pytest.approx(float(z[0] ** 2))
        assert t >= 0.0
    times = [s[2] for s in samples]
    assert times == sorted(times)


def test_solve_accepts_one_off_initial_guess_without_recompiling():
    prog = _quadratic_program()
    opt = Optimizer(
        prog,
        z0=np.array([1.0]),
        method="scipy_slsqp",
        options={"disp": False, "maxiter": 50},
    )

    program_evaluator = opt.program_evaluator
    out = opt.solve(z0=np.array([2.0]))
    assert opt.program_evaluator is program_evaluator
    assert out.success
    assert np.allclose(out.z, [0.0], atol=1e-8)


_IPOPT_QUIET = {"print_level": 0, "max_iter": 200}


@pytest.mark.ipopt
def test_ipopt_unconstrained_quadratic():
    pytest.importorskip("cyipopt")

    def J(z: np.ndarray):
        return 0.5 * np.sum((z - np.array([1.0, 2.0])) ** 2)

    def grad_J(z: np.ndarray) -> np.ndarray:
        return z - np.array([1.0, 2.0])

    prog = MathematicalProgram(n_z=2, J=J, grad_J=grad_J)
    out = Optimizer(prog, z0=np.zeros(2), method="ipopt", options=_IPOPT_QUIET).solve()
    assert out.success
    assert np.allclose(out.z, [1.0, 2.0], atol=1e-6)
    assert out.cost is not None and out.cost < 1e-10


@pytest.mark.ipopt
def test_ipopt_box_bounds():
    pytest.importorskip("cyipopt")

    def J(z: np.ndarray):
        return 0.5 * np.sum(z**2)

    def grad_J(z: np.ndarray) -> np.ndarray:
        return z

    prog = MathematicalProgram(
        n_z=3,
        J=J,
        grad_J=grad_J,
        lower=np.ones(3),
        upper=np.full(3, np.inf),
    )
    out = Optimizer(
        prog,
        z0=2.0 * np.ones(3),
        method="ipopt",
        options=_IPOPT_QUIET,
    ).solve()
    assert out.success
    assert np.allclose(out.z, np.ones(3), atol=1e-6)


@pytest.mark.ipopt
def test_ipopt_equality_constraint():
    pytest.importorskip("cyipopt")

    def J(z: np.ndarray):
        return z[0] + z[1]

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([1.0, 1.0])

    def h(z: np.ndarray) -> np.ndarray:
        return np.array([z[0] ** 2 + z[1] ** 2 - 1.0])

    def jac_h(z: np.ndarray) -> np.ndarray:
        return np.array([[2.0 * z[0], 2.0 * z[1]]])

    prog = MathematicalProgram(n_z=2, J=J, h=h, grad_J=grad_J, jac_h=jac_h)
    out = Optimizer(
        prog,
        z0=np.array([-0.5, -0.5]),
        method="ipopt",
        options=_IPOPT_QUIET,
    ).solve()
    assert out.success
    assert np.allclose(out.z, [-1.0 / np.sqrt(2.0), -1.0 / np.sqrt(2.0)], atol=1e-5)


@pytest.mark.ipopt
def test_ipopt_inequality_with_bounds():
    pytest.importorskip("cyipopt")

    def J(z: np.ndarray):
        return (z[0] - 2.0) ** 2 + (z[1] - 1.0) ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * (z[0] - 2.0), 2.0 * (z[1] - 1.0)])

    def g(z: np.ndarray) -> np.ndarray:
        return np.array([1.0 - z[0] - z[1]])

    def jac_g(z: np.ndarray) -> np.ndarray:
        return np.array([[-1.0, -1.0]])

    prog = MathematicalProgram(
        n_z=2,
        J=J,
        g=g,
        grad_J=grad_J,
        jac_g=jac_g,
        lower=np.zeros(2),
        upper=np.full(2, np.inf),
    )
    out = Optimizer(
        prog,
        z0=np.array([0.5, 0.5]),
        method="ipopt",
        options=_IPOPT_QUIET,
    ).solve()
    assert out.success
    assert np.allclose(out.z, [1.0, 0.0], atol=1e-4)


@pytest.mark.ipopt
def test_ipopt_record_solve_time():
    pytest.importorskip("cyipopt")

    def J(z: np.ndarray):
        return z[0] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * z[0]])

    prog = MathematicalProgram(n_z=1, J=J, grad_J=grad_J)
    opt = Optimizer(prog, z0=np.array([1.0]), method="ipopt", options=_IPOPT_QUIET)
    out = opt.solve(record_solve_time=True)
    assert out.success
    assert out.solve_time_s is not None and out.solve_time_s >= 0.0


@pytest.mark.ipopt
def test_ipopt_solve_callback_not_supported():
    pytest.importorskip("cyipopt")

    def J(z: np.ndarray):
        return z[0] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * z[0]])

    prog = MathematicalProgram(n_z=1, J=J, grad_J=grad_J)
    opt = Optimizer(prog, z0=np.array([1.0]), method="ipopt", options=_IPOPT_QUIET)

    def _cb(z: np.ndarray, _J: float, _t: float) -> None:
        del z, _J, _t

    with pytest.raises(NotImplementedError, match="does not support solve\\(callback"):
        opt.solve(callback=_cb)
