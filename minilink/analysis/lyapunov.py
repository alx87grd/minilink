"""Region of attraction of a closed loop: the sublevel set ``{x : (x − x̄)ᵀ P (x − x̄) ≤ c}`` of a quadratic Lyapunov function."""

from __future__ import annotations

import textwrap

import numpy as np
from scipy.linalg import solve_continuous_lyapunov

from minilink.analysis.derivatives import jacobian
from minilink.analysis.equilibria import find_equilibrium
from minilink.core.backends import BACKEND_JAX, require_jax, require_jax_numpy
from minilink.core.fields import EllipsoidSet, QuadraticField

#: The seed of the first pass's samples, one more per pass: the level reads the same twice.
SAMPLE_SEED = 0

#: Passes of the level search, each on the region the previous one left.
MAX_PASSES = 20

#: Points per axis of the phase-plane plot.
PLOT_POINTS = 201


###############################################################################
# Public API
###############################################################################


def region_of_attraction(
    sys,
    x_bar=None,
    u_bar=None,
    t: float = 0.0,
    params=None,
    *,
    Q=None,
    n_samples: int = None,
) -> EllipsoidSet:
    """
    Estimate a region of attraction of ``sys`` around an equilibrium.

    ``V(x) = (x − x̄)ᵀ P (x − x̄)`` with ``P`` the solution of the Lyapunov
    equation of the linearization, and the region is ``{x : V(x) ≤ c}``. The
    level starts at ``V ≤ 9`` inside the state box and shrinks: each pass
    draws states uniformly inside the current region and lowers ``c`` to the
    smallest ``V`` among those where ``V̇ ≥ 0`` along the nonlinear dynamics,
    until a pass finds none. ``V̇ < 0`` is checked on samples only, so the
    region is an estimate, not a proof: a state no sample hit can break it.
    The level can only err high, and the samples thin out as the state grows:
    in four states and more, read it with ``basin=True`` on
    :func:`plot_region_of_attraction`, or a larger ``n_samples``.

    Parameters
    ----------
    sys : DynamicSystem
        Autonomous system, normally a closed loop such as ``ctl @ plant``.
    x_bar : array of shape (n,), optional
        Guess of the equilibrium, refined by
        :func:`~minilink.analysis.equilibria.find_equilibrium`; the nominal
        state by default. The region is centered on the refined point.
    u_bar, t, params : optional
        Held input, time and parameter set, as in every analysis tool.
    Q : array of shape (n, n), optional
        Right-hand side of the Lyapunov equation; the identity by default.
    n_samples : int, optional
        States drawn per pass, 50 000 by default.

    Returns
    -------
    EllipsoidSet
        The region: its center is the equilibrium, its ``params["P"]`` the
        Lyapunov matrix, its :attr:`~minilink.core.fields.EllipsoidSet.level` the level ``c`` and
        its :attr:`~minilink.core.fields.EllipsoidSet.field` the function ``V``.

    Raises
    ------
    ValueError
        If the linearization is not Hurwitz: an unstable equilibrium has no
        region of attraction.
    """
    # The equilibrium is found near the guess; the held input defaults to the nominal one
    x_bar = find_equilibrium(sys, x_bar, u_bar, t, params)
    u_bar = np.asarray(sys.u_nominal if u_bar is None else u_bar, dtype=float)

    n = int(sys.n)
    Q = np.eye(n) if Q is None else np.asarray(Q, dtype=float)
    n_samples = 50_000 if n_samples is None else int(n_samples)
    domain = sys.state.box
    f = dynamics_of(sys, u_bar, t, params)

    # A = ∂f/∂x at the equilibrium, which must be Hurwitz
    A = np.asarray(jacobian(sys, "f", "x", x_bar, u_bar, t, params), dtype=float)
    poles = np.linalg.eigvals(A)
    require_hurwitz(poles)

    # AᵀP + PA = −Q; (P + Pᵀ) / 2 drops the solver's rounding off the symmetric solution
    P = solve_continuous_lyapunov(A.T, -Q)
    P = 0.5 * (P + P.T)

    # The start: c = 9, capped so the region stays in the state box, √(c (P⁻¹)ᵢᵢ) ≤ the
    # distance from x̄ to each face
    half = np.minimum(x_bar - domain.lower, domain.upper - x_bar)
    c = min(9.0, float(np.min(half**2 / np.diag(np.linalg.inv(P)))))

    for k_pass in range(MAX_PASSES):
        # States drawn uniformly inside the current region {V ≤ c}
        X = (
            QuadraticField(x_bar, P)
            .as_set(upper=c)
            .sample(SAMPLE_SEED + k_pass, n_samples)
        )
        d = X - x_bar

        # V = dᵀ P d and V̇ = 2 dᵀ P f(x) at every sample, f the nonlinear dynamics
        V = np.einsum("ij,jk,ik->i", d, P, d)
        V_dot = 2.0 * np.einsum("ij,jk,ik->i", d, P, f(X))

        # A sample blocks the level where V̇ ≥ 0; the equilibrium itself, V̇ = 0 to rounding, does not
        blocked = (V_dot >= 0.0) & (V > 1e-10 * c)
        if not blocked.any():
            break

        # c = the smallest V among the blocked samples
        c = float(V[blocked].min())

    region = QuadraticField(x_bar, P).as_set(upper=c)

    return region


def plot_region_of_attraction(
    sys,
    region: EllipsoidSet,
    x_axis: int = 0,
    y_axis: int = 1,
    *,
    u_bar=None,
    t: float = 0.0,
    params=None,
    basin: bool = False,
    trajectories=(),
    ax=None,
    show: bool = True,
):
    """
    Draw a region of attraction of ``sys`` in the plane of two states.

    The other states are held at the region's center, so what is drawn is the
    slice of the region through the equilibrium. The layers: the level set
    ``V = c``, the curve ``V̇ = 0`` where ``V`` stops decreasing, and the
    equilibrium; ``basin=True`` shades the states of the plane that converge
    in simulation, which shows how much a quadratic ``V`` gives up;
    ``trajectories`` is a sequence of initial states to simulate and overlay.

    Returns
    -------
    (Figure, Axes)
    """
    # Lazy import: matplotlib loads on first use, not with the analysis
    import matplotlib.pyplot as plt

    # Lazy import: the simulator loads on first use, not with the analysis verbs
    from minilink.simulation.simulator import Simulator

    x_bar = region.params["center"]
    P = region.params["P"]
    c = region.level
    V = region.field
    u_bar = np.asarray(sys.u_nominal if u_bar is None else u_bar, dtype=float)
    labels = list(sys.state.labels)

    # The window: two and a half times the slice's half-widths √(c ((P_s)⁻¹)ᵢᵢ)
    axes = [x_axis, y_axis]
    P_s = P[np.ix_(axes, axes)]
    half = 2.5 * np.sqrt(c * np.diag(np.linalg.inv(P_s)))
    grid_x = np.linspace(x_bar[x_axis] - half[0], x_bar[x_axis] + half[0], PLOT_POINTS)
    grid_y = np.linspace(x_bar[y_axis] - half[1], x_bar[y_axis] + half[1], PLOT_POINTS)
    GX, GY = np.meshgrid(grid_x, grid_y)
    X = np.tile(x_bar, (GX.size, 1))
    X[:, x_axis] = GX.ravel()
    X[:, y_axis] = GY.ravel()

    # V and V̇ = 2 (x − x̄)ᵀ P f(x) on the plane
    f = dynamics_of(sys, u_bar, t, params)
    V_plane = V(X).reshape(GX.shape)
    V_dot_plane = 2.0 * np.einsum("ij,jk,ik->i", X - x_bar, P, f(X)).reshape(GX.shape)

    if ax is None:
        _, ax = plt.subplots(figsize=(7.0, 5.5))
    horizon, dt = time_scales_of(sys, x_bar, u_bar, t, params)

    # The states of the plane that converge in simulation
    if basin:
        X_final = rollout_of(sys, u_bar, t, params, horizon, dt)(X)
        reached = np.linalg.norm(X_final - x_bar, axis=1) < 0.05 * np.max(
            region.half_width
        )
        ax.contourf(
            GX, GY, reached.reshape(GX.shape), levels=[0.5, 1.5], colors=["#cfe3f7"]
        )
        ax.fill([], [], color="#cfe3f7", label="converges in simulation")

    # The level set V = c and the curve V̇ = 0
    ax.contour(GX, GY, V_plane, levels=[c], colors="tab:red", linewidths=2.0)
    ax.plot([], [], color="tab:red", linewidth=2.0, label=r"region $V \leq c$")
    if np.any(V_dot_plane > 1e-9 * np.max(np.abs(V_dot_plane))):
        ax.contour(
            GX,
            GY,
            V_dot_plane,
            levels=[0.0],
            colors="tab:orange",
            linewidths=1.0,
            linestyles="--",
        )
        ax.plot(
            [],
            [],
            color="tab:orange",
            linewidth=1.0,
            linestyle="--",
            label=r"$\dot V = 0$",
        )

    # Trajectories, each simulated from its initial state
    for x0 in trajectories:
        traj = Simulator(sys, x0=x0, tf=horizon, verbose=False).solve()
        ax.plot(traj.x[x_axis], traj.x[y_axis], color="0.35", linewidth=1.0)
        ax.plot(x0[x_axis], x0[y_axis], "o", color="0.35", markersize=4)
    if len(trajectories):
        ax.plot([], [], color="0.35", linewidth=1.0, label="trajectory")

    # The equilibrium, the axes and the legend
    ax.plot(x_bar[x_axis], x_bar[y_axis], "k*", markersize=12, label="equilibrium")
    title = textwrap.fill(f"{sys.name}: region of attraction", width=52)
    if int(sys.n) > 2:
        title += "\n(slice at the equilibrium)"
    ax.set_title(title, fontsize=10)
    ax.set_xlabel(labels[x_axis])
    ax.set_ylabel(labels[y_axis])
    ax.set_xlim(grid_x[0], grid_x[-1])
    ax.set_ylim(grid_y[0], grid_y[-1])
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.figure.tight_layout()
    if show:
        plt.show()

    return ax.figure, ax


###############################################################################
# Internal machinery
###############################################################################


def require_hurwitz(poles):
    """Refuse an equilibrium whose linearization has a pole in the closed right half-plane."""
    if poles.real.max() >= 0.0:
        worst = poles[int(np.argmax(poles.real))]
        raise ValueError(
            f"the loop is not stable at this equilibrium (pole at {worst:.4g}); "
            "there is no region of attraction"
        )


def dynamics_of(sys, u_bar, t, params):
    """``f(X) -> dX``: the dynamics of ``sys`` on a stack of states, the input held at ``u_bar``."""
    # Lazy import: the compiler loads on first use, not with the analysis verbs
    from minilink.core.compile.compiler import compile_auto

    backend, evaluator = compile_auto(sys)

    # JAX: f vectorized over the stack and jitted
    if backend == BACKEND_JAX:
        jax = require_jax()
        jnp = require_jax_numpy()
        batch = jax.jit(jax.vmap(evaluator.f, in_axes=(0, None, None, None)))
        return lambda X: np.asarray(batch(jnp.asarray(X), u_bar, t, params))

    # NumPy: f state by state
    return lambda X: np.stack([evaluator.f(x, u_bar, t, params) for x in X])


def time_scales_of(sys, x_bar, u_bar, t, params):
    """``(horizon, dt)`` of the simulations: ten time constants of the slowest linearized mode, a tenth of the fastest."""
    A = np.asarray(jacobian(sys, "f", "x", x_bar, u_bar, t, params), dtype=float)
    poles = np.linalg.eigvals(A)

    # t_f = 10 / |Re λ_slow|, and Δt = 0.1 / |λ_fast| with at least 50 steps
    horizon = 10.0 / abs(float(np.max(poles.real)))
    dt = min(0.1 / float(np.max(np.abs(poles))), horizon / 50.0)

    return horizon, dt


def rollout_of(sys, u_bar, t, params, horizon: float, dt: float):
    """``rollout(X0) -> X_final``: each state integrated over ``horizon`` seconds, the input held."""
    # Lazy import: the compiler loads on first use, not with the analysis verbs
    from minilink.core.compile.compiler import compile_auto

    backend, evaluator = compile_auto(sys)

    n_steps = max(1, int(round(horizon / dt)))
    u_seq = np.tile(u_bar, (n_steps, 1))

    # JAX: the step model's rollout, vectorized over the stack and jitted
    if backend == BACKEND_JAX:
        jax = require_jax()
        jnp = require_jax_numpy()
        finals = jax.jit(
            jax.vmap(lambda x0: evaluator.rollout(x0, u_seq, t, dt)[-1]),
        )
        return lambda X0: np.asarray(finals(jnp.asarray(X0)))

    # NumPy: state by state
    return lambda X0: np.stack([evaluator.rollout(x0, u_seq, t, dt)[-1] for x0 in X0])


if __name__ == "__main__":
    from minilink import InvertedPendulum, lqr

    plant = InvertedPendulum()
    plant.inputs["u"].lower = np.array([-2.0])  # too weak to hold it far over
    plant.inputs["u"].upper = np.array([2.0])
    ctl = lqr(plant, Q=np.eye(2), R=np.eye(1), x_bar=np.zeros(2))
    region = region_of_attraction(ctl @ plant)
    print(region.level, region.half_width)
