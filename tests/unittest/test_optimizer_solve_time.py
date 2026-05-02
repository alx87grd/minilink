"""Tests for optional wall-clock timing on :class:`Optimizer`."""

import numpy as np
import pytest

from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizer import Optimizer


def _quadratic_program() -> MathematicalProgram:
    def J(z: np.ndarray) -> float:
        return float(z[0] ** 2)

    def grad_J(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * float(z[0])])

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
