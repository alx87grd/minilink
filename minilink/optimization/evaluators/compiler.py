"""Compiler for mathematical-program evaluators."""

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    normalize_backend,
)
from minilink.optimization.mathematical_program import MathematicalProgram


def compile_program_evaluator(
    program: MathematicalProgram,
    backend: str = BACKEND_NUMPY,
    *,
    sample_z: np.ndarray | None = None,
    use_hessian: bool = False,
    verbose: bool = False,
):
    """
    Compile ``program`` into a backend-specific program evaluator.

    Parameters
    ----------
    program : MathematicalProgram
        Pure NLP description to evaluate.
    backend : {"numpy", "jax"}, optional
        Program-evaluator backend. ``"numpy"`` is always available; ``"jax"``
        requires the optional JAX dependency.
    sample_z : array_like, optional
        Representative decision vector used for shape validation and JAX
        warm-start. If omitted, evaluators use ``zeros(n_z)``.
    use_hessian : bool, optional
        For JAX evaluators, auto-generate a dense objective Hessian when no
        explicit ``hess_J`` was supplied.
    verbose : bool, optional
        If True, print backend compilation diagnostics where supported.
    """
    key = normalize_backend(backend)
    if key == BACKEND_NUMPY:
        from minilink.optimization.evaluators.numpy_evaluator import (
            NumpyMathematicalProgramEvaluator,
        )

        return NumpyMathematicalProgramEvaluator(program, sample_z=sample_z)

    if key == BACKEND_JAX:
        from minilink.optimization.evaluators.jax_evaluator import (
            JaxMathematicalProgramEvaluator,
        )

        return JaxMathematicalProgramEvaluator(
            program,
            sample_z=sample_z,
            use_hessian=use_hessian,
            verbose=verbose,
        )

    raise ValueError(f"Unknown mathematical-program backend {backend!r}.")
