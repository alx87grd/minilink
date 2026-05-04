import numpy as np

from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizer import Optimizer

# Demo controls.
Z0 = 2.0
SHOW_FIGURE = True  # Show the final matplotlib figure.
TRACE_ITERATES = True  # Print one callback line for each optimizer iterate.
PRINT_SOLVE_REPORT = True  # Print the Minilink Optimizer pre/post solve report.
SCIPY_DISP = False  # Keep SciPy's own backend text off; use PRINT_SOLVE_REPORT.
FIGSIZE_SQUARE = (6.0, 6.0)
z_lo = 0.6
z_hi = 2.05


def J_1d_values(z: np.ndarray) -> np.ndarray:
    """Same cost as :func:`J_1d`, vectorized for plotting (any shape ``z``)."""
    x = np.asarray(z, dtype=float)
    return x**3 + x * x + np.sin(5.0 * np.pi * x) + 0.12 * np.sin(15.0 * np.pi * x)


def J_1d(z: np.ndarray):
    """Non-convex: cubic + quadratic envelope + two sine ridges."""
    x = z.reshape(-1)[0]
    return x**3 + x * x + np.sin(5.0 * np.pi * x) + 0.12 * np.sin(15.0 * np.pi * x)


def grad_1d(z: np.ndarray) -> np.ndarray:
    x = float(np.asarray(z, dtype=float).reshape(-1)[0])
    d = (
        3.0 * x * x
        + 2.0 * x
        + 5.0 * np.pi * np.cos(5.0 * np.pi * x)
        + 0.12 * 15.0 * np.pi * np.cos(15.0 * np.pi * x)
    )
    return np.array([d], dtype=float)


def g_box(z: np.ndarray) -> np.ndarray:
    x = float(np.asarray(z, dtype=float).reshape(-1)[0])
    return np.array([x - z_lo, z_hi - x], dtype=float)


def jac_box(z: np.ndarray) -> np.ndarray:
    return np.array([[1.0], [-1.0]], dtype=float)


prog = MathematicalProgram(
    n_z=1,
    J=J_1d,
    grad_J=grad_1d,
    g=g_box,
    jac_g=jac_box,
)

opt = Optimizer(
    prog,
    z0=np.array([Z0]),
    method="scipy_slsqp",
    options={
        "disp": SCIPY_DISP,
        "maxiter": 200,
        "ftol": 1e-12,
    },
)

# opt = Optimizer(
#     prog,
#     z0=np.array([Z0]),
#     method="ipopt",
#     options={
#         "disp": SCIPY_DISP,
#         "maxiter": 200,
#     },
# )

iterates_z: list[float] = []
iterates_J: list[float] = []


def record_iterate(z: np.ndarray, J: float, t: float) -> None:
    zz = float(np.asarray(z, dtype=float).reshape(-1)[0])
    iterates_z.append(zz)
    iterates_J.append(J)
    print(f"z: {zz}, J: {J}, t: {t}")


out = opt.solve(
    callback=record_iterate if TRACE_ITERATES else None,
    disp=PRINT_SOLVE_REPORT,
)

if SHOW_FIGURE:
    import matplotlib.pyplot as plt

    from minilink.graphical.environment import is_blocking_needed
    from minilink.graphical.matplotlib_style import (
        DPI_FIGURE,
        FONT_SIZE,
        style_trajectory_subplot,
    )

    z_plot = np.linspace(z_lo - 0.12, z_hi + 0.12, 1200)
    J_plot = J_1d_values(z_plot)

    # Square canvas (this plot is not a stacked trajectory strip).
    fig, ax = plt.subplots(
        1,
        1,
        figsize=FIGSIZE_SQUARE,
        dpi=DPI_FIGURE,
        frameon=True,
    )
    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title("Optimization demo: 1-D cost vs. z")

    ax.axvspan(z_lo, z_hi, alpha=0.2, color="C2", label="feasible (g(z)≥0)")

    ax.plot(z_plot, J_plot, color="C0", lw=1.5, alpha=0.8, label=r"$J(z)$")

    ax.axvline(
        z_lo, color="C3", ls="--", lw=1.0, label=r"$z_{\mathrm{lo}}, z_{\mathrm{hi}}$"
    )
    ax.axvline(z_hi, color="C3", ls="--", lw=1.0)

    z0 = float(opt.z0.flat[0])
    J0 = J_1d(opt.z0)
    ax.plot(z0, J0, "mo", ms=8, label=r"$z_0$")

    path_z = [z0] + iterates_z
    path_J = [J0] + iterates_J
    if len(path_z) > 1:
        ax.plot(path_z, path_J, "r.-", ms=6, lw=1.0, label="SLSQP iterates")

    zf = float(out.z.flat[0])
    ax.plot(
        zf,
        float(out.cost) if out.cost is not None else J_1d(out.z),
        "g*",
        ms=16,
        label=r"$z^\star$",
    )

    ax.set_xlabel(r"$z$ (decision, 1-D)", fontsize=FONT_SIZE)
    ax.set_ylabel(r"$J(z)$ (cost)", fontsize=FONT_SIZE)
    ax.set_title("Non-convex 1-D cost with interval constraints", fontsize=FONT_SIZE)
    style_trajectory_subplot(ax)
    ax.legend(loc="best", fontsize=FONT_SIZE)
    fig.tight_layout()
    plt.show(block=is_blocking_needed())
