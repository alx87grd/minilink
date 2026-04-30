"""Tests for optional wall-clock timing on :class:`Optimizer`."""

import numpy as np

from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer


def test_scipy_minimize_record_solve_time_default_none():
    opt = ScipyMinimizeOptimizer(method="SLSQP", options={"disp": False, "maxiter": 50})

    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

    prog = MathematicalProgram(J=J, z0=np.array([1.0]), grad=grad)
    out = opt.solve(prog)
    assert out.solve_time_s is None


def test_scipy_minimize_record_solve_time_sets_duration():
    opt = ScipyMinimizeOptimizer(method="SLSQP", options={"disp": False, "maxiter": 50})

    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

    prog = MathematicalProgram(J=J, z0=np.array([1.0]), grad=grad)
    out = opt.solve(prog, record_solve_time=True)
    assert out.solve_time_s is not None
    assert isinstance(out.solve_time_s, float)
    assert out.solve_time_s >= 0.0
    assert out.success


def test_scipy_minimize_disp_prints_report(capsys):
    opt = ScipyMinimizeOptimizer(method="SLSQP", options={"disp": False, "maxiter": 50})

    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

    prog = MathematicalProgram(J=J, z0=np.array([1.0]), grad=grad)
    out = opt.solve(prog, disp=True)
    captured = capsys.readouterr()
    assert "Optimization report" in captured.out
    assert "solve_time_s" in captured.out
    assert out.solve_time_s is not None
    assert out.success