"""The NumPy program evaluator: a mathematical program evaluated as given, with the derivatives it supplies."""

import numpy as np

from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import MathematicalProgram

###############################################################################
# Public API
###############################################################################


class NumpyMathematicalProgramEvaluator(MathematicalProgramEvaluator):
    """
    Solver-facing NumPy program evaluator for :class:`MathematicalProgram`.

    This program evaluator does not create finite-difference derivatives. It
    exposes only derivative callables supplied on the program.
    """

    def __init__(self, program: MathematicalProgram, sample_z=None):
        super().__init__(program, sample_z=sample_z)
        self.backend = "numpy"

        # The constraint sizes, then every available quantity once at the sample
        self.n_h = int(np.asarray(self.h(self.sample_z), dtype=float).reshape(-1).size)
        self.n_g = int(np.asarray(self.g(self.sample_z), dtype=float).reshape(-1).size)
        self.check_derivative_shapes(self.sample_z)

    @property
    def has_gradient(self) -> bool:
        return self.program.grad_J is not None

    @property
    def has_hessian(self) -> bool:
        return self.program.hess_J is not None

    @property
    def has_jacobian_h(self) -> bool:
        return self.n_h == 0 or self.program.jac_h is not None

    @property
    def has_jacobian_g(self) -> bool:
        return self.n_g == 0 or self.program.jac_g is not None

    # The program's functions, on a checked decision vector

    def J(self, z):
        z_arr = self.decision_vector(z)
        return self.program.J(z_arr)

    def h(self, z):
        z_arr = self.decision_vector(z)
        if self.program.h is None:
            return np.zeros(0, dtype=float)
        return self.program.h(z_arr)

    def g(self, z):
        z_arr = self.decision_vector(z)
        if self.program.g is None:
            return np.zeros(0, dtype=float)
        return self.program.g(z_arr)

    # The derivatives the program supplies, as flat NumPy arrays

    def gradient(self, z) -> np.ndarray:
        if self.program.grad_J is None:
            return super().gradient(z)
        value = self.program.grad_J(self.decision_vector(z))
        return self.as_vector(value, self.n_z, "objective gradient")

    def hessian(self, z) -> np.ndarray:
        if self.program.hess_J is None:
            return super().hessian(z)
        value = self.program.hess_J(self.decision_vector(z))
        return self.as_matrix(value, (self.n_z, self.n_z), "objective Hessian")

    def jacobian_h(self, z) -> np.ndarray:
        if self.n_h == 0 or self.program.jac_h is None:
            return super().jacobian_h(z)
        value = self.program.jac_h(self.decision_vector(z))
        return self.as_matrix(value, (self.n_h, self.n_z), "equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        if self.n_g == 0 or self.program.jac_g is None:
            return super().jacobian_g(z)
        value = self.program.jac_g(self.decision_vector(z))
        return self.as_matrix(value, (self.n_g, self.n_z), "inequality Jacobian")
