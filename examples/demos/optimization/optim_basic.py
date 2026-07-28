import numpy as np

from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizer import Optimizer

z_bar = np.array([1.0, -0.5, 2.0])

prog = MathematicalProgram(
    n_z=3,
    J=lambda z: 0.5 * np.dot(z - z_bar, z - z_bar),
    grad_J=lambda z: z - z_bar,
)

Optimizer(prog, z0=np.zeros(3), method="scipy_slsqp").solve(disp=True)
