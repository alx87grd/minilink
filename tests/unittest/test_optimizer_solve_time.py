"""Tests for optional wall-clock timing on :class:`Optimizer`."""

import numpy as np
import pytest

from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizer import Optimizer


def test_scipy_minimize_record_solve_time_default_none():
    opt = Optimizer(backend="scipy", options={"disp": False, "maxiter": 50})

    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

    prog = MathematicalProgram(J=J, z0=np.array([1.0]), grad=grad)
    out = opt.solve(prog)
    assert out.solve_time_s is None


def test_scipy_minimize_record_solve_time_sets_duration():
    opt = Optimizer(backend="scipy", options={"disp": False, "maxiter": 50})

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
    opt = Optimizer(backend="scipy", options={"disp": False, "maxiter": 50})

    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

    prog = MathematicalProgram(J=J, z0=np.array([1.0]), grad=grad)
    out = opt.solve(prog, disp=True)
    captured = capsys.readouterr()
    assert "Optimization Program" in captured.out
    assert "backend=" in captured.out
    assert "Running solver" in captured.out
    assert "success:" in captured.out
    assert "completed in" in captured.out
    assert out.solve_time_s is not None
    assert out.success


def test_scipy_minimize_progress_callback_z_J_t():
    samples: list[tuple[np.ndarray, float, float]] = []

    opt = Optimizer(backend="scipy", options={"disp": False, "maxiter": 50})

    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

    def prog_cb(z: np.ndarray, Jv: float, t: float) -> None:
        samples.append((z.copy(), Jv, t))

    prog = MathematicalProgram(J=J, z0=np.array([1.0]), grad=grad)
    out = opt.solve(prog, callback=prog_cb)
    assert out.success
    assert len(samples) >= 1
    for z, Jv, t in samples:
        assert z.shape == (1,)
        assert Jv == pytest.approx(float(z[0] ** 2))
        assert t >= 0.0
    times = [s[2] for s in samples]
    assert times == sorted(times)
