"""Optimizer iterates traced on a bounded non-convex one-variable cost."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import MathematicalProgram, Optimizer

z_lo, z_hi = 0.6, 2.05  # feasible interval
z0 = np.array([2.0])


def J(z):
    """Non-convex: cubic + quadratic envelope + two sine ridges."""
    x = z[0]
    return x**3 + x * x + np.sin(5.0 * np.pi * x) + 0.12 * np.sin(15.0 * np.pi * x)


def grad_J(z):
    x = z[0]
    dJ = (
        3.0 * x * x
        + 2.0 * x
        + 5.0 * np.pi * np.cos(5.0 * np.pi * x)
        + 0.12 * 15.0 * np.pi * np.cos(15.0 * np.pi * x)
    )
    return np.array([dJ])


def g(z):
    """g(z) >= 0 keeps z inside [z_lo, z_hi]."""
    return np.array([z[0] - z_lo, z_hi - z[0]])


def jac_g(z):
    return np.array([[1.0], [-1.0]])


prog = MathematicalProgram(n_z=1, J=J, grad_J=grad_J, g=g, jac_g=jac_g)
opt = Optimizer(
    prog, z0=z0, method="scipy_slsqp", options={"maxiter": 200, "ftol": 1e-12}
)
# opt = Optimizer(prog, z0=z0, method="ipopt", options={"maxiter": 200})

iterates = [(z0[0], J(z0))]


def record(z, J_value, t):
    iterates.append((z[0], J_value))
    print(f"z = {z[0]:.4f}   J = {J_value:.4f}   t = {t:.3f} s")


out = opt.solve(callback=record, verbose=True)

# Cost landscape, feasible interval, and the SLSQP path from z0 to z*.
z_plot = np.linspace(z_lo - 0.12, z_hi + 0.12, 1200)
J_plot = [J(np.array([z])) for z in z_plot]
path_z, path_J = zip(*iterates)

fig, ax = plt.subplots(figsize=(6.0, 6.0))
ax.axvspan(z_lo, z_hi, alpha=0.2, color="C2", label="feasible (g(z) >= 0)")
ax.plot(z_plot, J_plot, color="C0", label="J(z)")
ax.plot(path_z, path_J, "r.-", label="SLSQP iterates")
ax.plot(z0[0], J(z0), "mo", ms=8, label="z0")
ax.plot(out.z[0], J(out.z), "g*", ms=16, label="z*")
ax.set_xlabel("z")
ax.set_ylabel("J(z)")
ax.legend()
plt.show()
