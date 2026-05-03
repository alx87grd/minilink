"""Unit tests for the ipopt optimizer backend.

These tests require the optional ``cyipopt`` package and are skipped when it
is not installed.
"""

import numpy as np
import pytest

from minilink.optimization.mathematical_program import MathematicalProgram

cyipopt = pytest.importorskip("cyipopt")

from minilink.optimization.optimizer import Optimizer  # noqa: E402

_QUIET = {"print_level": 0, "max_iter": 200}


def test_ipopt_unconstrained_quadratic():
    def J(z: np.ndarray):
        return 0.5 * np.sum((z - np.array([1.0, 2.0])) ** 2)

    def grad_J(z: np.ndarray) -> np.ndarray:
        return z - np.array([1.0, 2.0])

    prog = MathematicalProgram(n_z=2, J=J, grad_J=grad_J)
    out = Optimizer(prog, z0=np.zeros(2), method="ipopt", options=_QUIET).solve()
    assert out.success
    assert np.allclose(out.z, [1.0, 2.0], atol=1e-6)
    assert out.cost is not None and out.cost < 1e-10


def test_ipopt_box_bounds():
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
        options=_QUIET,
    ).solve()
    assert out.success
    assert np.allclose(out.z, np.ones(3), atol=1e-6)


def test_ipopt_equality_constraint():
    # min z0 + z1 s.t. z0^2 + z1^2 = 1  =>  z* = (-1,-1)/sqrt(2)
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
        options=_QUIET,
    ).solve()
    assert out.success
    assert np.allclose(out.z, [-1.0 / np.sqrt(2.0), -1.0 / np.sqrt(2.0)], atol=1e-5)


def test_ipopt_inequality_with_bounds():
    # min (z0-2)^2 + (z1-1)^2 s.t. z0 + z1 <= 1, z >= 0  =>  z* = (1, 0)
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
        options=_QUIET,
    ).solve()
    assert out.success
    assert np.allclose(out.z, [1.0, 0.0], atol=1e-4)


def test_ipopt_record_solve_time():
    def J(z: np.ndarray):
        return z[0] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * z[0]])

    prog = MathematicalProgram(n_z=1, J=J, grad_J=grad_J)
    opt = Optimizer(prog, z0=np.array([1.0]), method="ipopt", options=_QUIET)
    out = opt.solve(record_solve_time=True)
    assert out.success
    assert out.solve_time_s is not None and out.solve_time_s >= 0.0


def test_ipopt_solve_callback_not_supported():
    def J(z: np.ndarray):
        return z[0] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * z[0]])

    prog = MathematicalProgram(n_z=1, J=J, grad_J=grad_J)
    opt = Optimizer(prog, z0=np.array([1.0]), method="ipopt", options=_QUIET)

    def _cb(z: np.ndarray, _J: float, _t: float) -> None:
        del z, _J, _t

    with pytest.raises(NotImplementedError, match="does not support solve\\(callback"):
        opt.solve(callback=_cb)
