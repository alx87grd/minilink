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


@pytest.mark.optional
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


@pytest.mark.optional
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


@pytest.mark.optional
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


@pytest.mark.optional
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


@pytest.mark.optional
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


@pytest.mark.optional
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
