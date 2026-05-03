"""Unit tests for pure mathematical programs and compiled evaluators."""

import numpy as np
import pytest

from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import MathematicalProgram


def test_mathematical_program_is_pure_description_without_z0():
    def J(z: np.ndarray):
        return z @ z

    program = MathematicalProgram(
        n_z=2,
        J=J,
        metadata={"source": "test"},
    )

    assert program.problem_class == "nlp"
    assert program.n_z == 2
    assert not hasattr(program, "z0")
    assert program.metadata == {"source": "test"}


def test_mathematical_program_validates_bounds_dimension():
    with pytest.raises(ValueError, match="lower bounds must have shape"):
        MathematicalProgram(
            n_z=2,
            J=lambda z: z @ z,
            lower=np.zeros(3),
        )


def test_mathematical_program_validates_bounds_order():
    with pytest.raises(ValueError, match="lower bounds must be less"):
        MathematicalProgram(
            n_z=1,
            J=lambda z: z[0] * z[0],
            lower=np.array([2.0]),
            upper=np.array([1.0]),
        )


def test_numpy_evaluator_objective_constraints_and_derivatives():
    def J(z: np.ndarray):
        return z[0] ** 2 + z[1] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return 2.0 * z

    def hess_J(z: np.ndarray) -> np.ndarray:
        return 2.0 * np.eye(2)

    def h(z: np.ndarray) -> np.ndarray:
        return np.array([z[0] + z[1] - 1.0])

    def jac_h(z: np.ndarray) -> np.ndarray:
        return np.array([[1.0, 1.0]])

    def g(z: np.ndarray) -> np.ndarray:
        return np.array([z[0], z[1]])

    def jac_g(z: np.ndarray) -> np.ndarray:
        return np.eye(2)

    program = MathematicalProgram(
        n_z=2,
        J=J,
        h=h,
        g=g,
        grad_J=grad_J,
        hess_J=hess_J,
        jac_h=jac_h,
        jac_g=jac_g,
    )
    program_evaluator = compile_program_evaluator(
        program,
        sample_z=np.array([0.5, 0.5]),
    )

    assert program_evaluator.backend == "numpy"
    assert program_evaluator.n_h == 1
    assert program_evaluator.n_g == 2
    assert program_evaluator.objective(np.array([1.0, 2.0])) == pytest.approx(5.0)
    np.testing.assert_allclose(program_evaluator.equality_residual([0.25, 0.75]), [0.0])
    np.testing.assert_allclose(
        program_evaluator.inequality_margin([0.25, 0.75]),
        [0.25, 0.75],
    )
    np.testing.assert_allclose(program_evaluator.gradient([1.0, 2.0]), [2.0, 4.0])
    np.testing.assert_allclose(
        program_evaluator.hessian([1.0, 2.0]),
        2.0 * np.eye(2),
    )
    np.testing.assert_allclose(program_evaluator.jacobian_h([1.0, 2.0]), [[1.0, 1.0]])
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0, 2.0]), np.eye(2))


def test_numpy_evaluator_empty_constraints():
    program = MathematicalProgram(n_z=1, J=lambda z: z[0] ** 2)
    program_evaluator = compile_program_evaluator(program, sample_z=np.array([1.0]))

    assert program_evaluator.n_h == 0
    assert program_evaluator.n_g == 0
    np.testing.assert_allclose(program_evaluator.equality_residual([1.0]), [])
    np.testing.assert_allclose(program_evaluator.inequality_margin([1.0]), [])
    np.testing.assert_allclose(program_evaluator.jacobian_h([1.0]), np.zeros((0, 1)))
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0]), np.zeros((0, 1)))


def test_numpy_evaluator_reports_bad_constraint_shape():
    program = MathematicalProgram(
        n_z=2,
        J=lambda z: z @ z,
        h=lambda z: np.array([z[0]]),
    )
    program_evaluator = compile_program_evaluator(
        program,
        sample_z=np.array([0.0, 0.0]),
    )

    with pytest.raises(ValueError, match="decision vector must have shape"):
        program_evaluator.objective(np.array([1.0, 2.0, 3.0]))


def test_jax_evaluator_autodiff_when_available():
    jax = pytest.importorskip("jax")
    jnp = pytest.importorskip("jax.numpy")

    def J(z):
        return z @ z

    def h(z):
        return jnp.array([z[0] + z[1] - 1.0])

    def g(z):
        return jnp.array([z[0], z[1]])

    program = MathematicalProgram(n_z=2, J=J, h=h, g=g)
    program_evaluator = compile_program_evaluator(
        program,
        backend="jax",
        sample_z=jnp.array([0.5, 0.5]),
        use_hessian=True,
    )

    assert program_evaluator.backend == "jax"
    assert program_evaluator.objective([1.0, 2.0]) == pytest.approx(5.0)
    np.testing.assert_allclose(program_evaluator.gradient([1.0, 2.0]), [2.0, 4.0])
    np.testing.assert_allclose(
        program_evaluator.hessian([1.0, 2.0]),
        2.0 * np.eye(2),
    )
    np.testing.assert_allclose(
        program_evaluator.jacobian_h([1.0, 2.0]),
        [[1.0, 1.0]],
    )
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0, 2.0]), np.eye(2))

    # Keep the imported symbol used so linters do not mark it as dead in some configs.
    assert jax is not None
