"""
Lyapunov certificates: where is a closed loop guaranteed to settle?

:func:`region_of_attraction` answers that for any autonomous
:class:`~minilink.core.system.DynamicSystem` — an LQR loop, a lookup table
from value iteration, an impedance law, or a neural policy — because all of
them are the same kind of object.

The default method is the textbook one. About an equilibrium ``x_bar`` the
linearization ``A = df/dx`` must be Hurwitz; the Lyapunov equation
``AᵀP + PA = -Q`` then gives a quadratic

    V(x) = (x - x_bar)ᵀ P (x - x_bar),    V̇(x) = 2 (x - x_bar)ᵀ P f(x)

whose derivative is taken along the **nonlinear** dynamics, saturation and
network and all. Every state of the largest sublevel set ``{V <= c}`` on
which ``V̇ < 0`` converges to ``x_bar``, so that set is a certified region of
attraction.

Any :class:`~minilink.core.system.DynamicSystem` works. Under JAX the
Jacobian is exact and the sweep is one vmapped call; a plant that does not
trace falls back to finite differences and a Python loop, which costs a few
times more and is still seconds on a small model.

The level ``c`` is found by sampling, which makes it a *sharp estimate*
rather than a proof: an unsampled state where ``V`` stops decreasing would
lower it. Coverage thins out quickly with the number of states, so the search
scores itself — :attr:`LyapunovCertificate.sample_limited` is ``True`` when
two halves of the samples disagree about the level, which is the honest
signal that the number is optimistic. :meth:`LyapunovCertificate.verify` is the counter-check — it draws
states inside the certified set and integrates them — and
:meth:`LyapunovCertificate.plot` shows how much of the true basin a quadratic
``V`` gives up.
"""

import textwrap
from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_continuous_lyapunov, solve_triangular

from minilink.analysis.derivatives import jacobian
from minilink.analysis.equilibria import find_equilibrium
from minilink.core.sets import BoxSet

#: Families of Lyapunov function this tool can build.
METHODS = ("quadratic", "sos")

# Public API


@dataclass
class VerificationReport:
    """Simulation's verdict on a certificate: every trial inside it must converge."""

    converged: np.ndarray
    final_distance: np.ndarray
    counterexample: np.ndarray | None = None

    @property
    def holds(self) -> bool:
        """``True`` when every sampled state converged to the equilibrium."""
        return bool(np.all(self.converged))

    def __str__(self) -> str:
        n = int(self.converged.size)
        worst = float(np.max(self.final_distance)) if n else float("nan")
        verdict = "holds" if self.holds else "FAILS"
        return (
            f"{int(np.sum(self.converged))}/{n} trials converged, "
            f"worst final distance {worst:.2e} — certificate {verdict}"
        )


@dataclass
class LyapunovCertificate:
    """
    Lyapunov function and the level that certifies a region of attraction.

    Returned by :func:`region_of_attraction`. The certified region is
    ``{x : V(x) <= level}``: every state in it converges to :attr:`x_bar`.

    Attributes
    ----------
    sys : DynamicSystem
        The closed loop the certificate belongs to.
    x_bar : array of shape (n,)
        Equilibrium the region is centered on. It is *found*, not assumed, so
        it may differ from the operating point that was asked for.
    P : array of shape (n, n)
        Lyapunov matrix of the quadratic family,
        ``V(x) = (x - x_bar)ᵀ P (x - x_bar)``.
    level : float
        Certified value ``c`` of ``V``.
    Q : array of shape (n, n)
        Right-hand side of ``AᵀP + PA = -Q``.
    poles : array of shape (n,)
        Eigenvalues of the linearization, kept as evidence.
    window : BoxSet
        Box the sublevel search covered.
    limiting_state : array of shape (n,)
        Sampled state that pinned the level: the nearest place, in ``V``,
        where the loop stops making progress. Reading the control input
        there usually explains the certificate.
    sample_spread : float
        Relative disagreement between the levels two halves of the samples
        give. Near zero means the sweep had enough samples to decide; see
        :attr:`sample_limited`.
    method : str
        Family the Lyapunov function came from (see :data:`METHODS`).
    """

    sys: object
    x_bar: np.ndarray
    P: np.ndarray
    level: float
    Q: np.ndarray
    poles: np.ndarray
    window: BoxSet
    limiting_state: np.ndarray
    sample_spread: float = 0.0
    method: str = "quadratic"
    u_bar: np.ndarray = field(default=None, repr=False)
    t: float = field(default=0.0, repr=False)
    params: object = field(default=None, repr=False)
    backend: str = field(default=None, repr=False)
    evaluator: object = field(default=None, repr=False)

    # --- the certificate itself ---

    def V(self, x):
        """Lyapunov function at one state ``(n,)`` or a stack of states ``(N, n)``."""
        d = np.atleast_2d(np.asarray(x, dtype=float)) - self.x_bar
        v = np.einsum("ij,jk,ik->i", d, self.P, d)
        return v if np.ndim(x) > 1 else float(v[0])

    def V_dot(self, x):
        """Derivative of ``V`` along the nonlinear dynamics, same shapes as :meth:`V`."""
        states = np.atleast_2d(np.asarray(x, dtype=float))
        d = states - self.x_bar
        f = self.dynamics()(states)
        v_dot = 2.0 * np.einsum("ij,jk,ik->i", d, self.P, f)
        return v_dot if np.ndim(x) > 1 else float(v_dot[0])

    def contains(self, x):
        """``True`` where the state lies in the certified region."""
        v = self.V(x)
        return v <= self.level if np.ndim(x) > 1 else bool(v <= self.level)

    @property
    def extent(self) -> np.ndarray:
        """Half-width of the certified region along each state, ``sqrt(c (P⁻¹)ᵢᵢ)``."""
        return np.sqrt(self.level * np.diag(np.linalg.inv(self.P)))

    def slice_extent(self, x_axis: int = 0, y_axis: int = 1) -> np.ndarray:
        """
        Half-widths of the region's *slice* through the equilibrium in two states.

        With the other states pinned at ``x_bar`` the certified set cuts a
        smaller ellipse than :attr:`extent`, which is its shadow on each axis.
        The slice is what a phase-plane plot draws.
        """
        block = self.P[np.ix_([x_axis, y_axis], [x_axis, y_axis])]
        return np.sqrt(self.level * np.diag(np.linalg.inv(block)))

    @property
    def sample_limited(self) -> bool:
        """
        ``True`` when the level moved between two halves of the samples.

        The sublevel search is only as good as its coverage, and coverage
        thins out fast with the number of states. When this is ``True`` the
        level is an optimistic guess rather than a sharp estimate: raise
        ``samples``, narrow ``window``, or read it as indicative only.
        """
        return self.sample_spread > 0.1

    @property
    def rate(self) -> float:
        """Slowest linearized decay rate: the natural time scale of the loop."""
        return float(np.max(self.poles.real))

    def __str__(self) -> str:
        extent = np.array2string(self.extent, precision=3, suppress_small=True)
        caveat = (
            f" — sample-limited, halves disagree by {100 * self.sample_spread:.0f}%"
            if self.sample_limited
            else ""
        )
        return (
            f"certified V <= {self.level:.4g} about "
            f"{np.array2string(self.x_bar, precision=3, suppress_small=True)}: "
            f"reaches {extent} per state, slowest pole {self.rate:.3g}{caveat}"
        )

    # --- the two questions asked of it ---

    def verify(self, n: int = 200, tf: float = None, tol: float = None):
        """
        Integrate ``n`` states drawn inside the certified region: all must converge.

        States are drawn uniformly in the ellipsoid ``{V <= level}``. ``tf``
        defaults to ten time constants of the slowest linearized mode and
        ``tol`` to a hundredth of the certified extent. A failure means the
        sublevel search was too coarse, or the model is not what it seems,
        and the report carries the offending state.
        """
        tf = 10.0 / abs(self.rate) if tf is None else float(tf)
        tol = 0.01 * float(np.max(self.extent)) if tol is None else float(tol)

        x0 = self.x_bar + sample_in_ellipsoid(self.P, self.level, n)
        x_final = self.rollout(tf)(x0)

        final_distance = np.linalg.norm(x_final - self.x_bar, axis=1)
        converged = final_distance < tol
        counterexample = None if np.all(converged) else x0[int(np.argmin(converged))]
        return VerificationReport(converged, final_distance, counterexample)

    def plot(
        self,
        x_axis: int = 0,
        y_axis: int = 1,
        *,
        basin: bool = False,
        detail: bool = True,
        verified: int = 0,
        trajectories=(),
        n: int = 201,
        limits=None,
        ax=None,
        show: bool = True,
    ):
        """Phase-plane slice of the certified region. See :func:`plot_region_of_attraction`."""
        return plot_region_of_attraction(
            self,
            x_axis,
            y_axis,
            basin=basin,
            detail=detail,
            verified=verified,
            trajectories=trajectories,
            n=n,
            limits=limits,
            ax=ax,
            show=show,
        )

    # --- compiled dynamics, shared by the questions above ---

    def dynamics(self):
        """``f_many(X) -> dX``: the closed-loop dynamics on a stack of states."""
        return dynamics_on(
            self.backend, self.evaluator, self.u_bar, self.t, self.params
        )

    def rollout(self, tf: float):
        """``rollout(X0) -> X_final``: integrate a stack of states for ``tf``."""
        dt = min(0.1 / float(np.max(np.abs(self.poles))), tf / 50.0)
        return rollout_on(self.backend, self.evaluator, self.u_bar, self.t, dt, tf)


def region_of_attraction(
    sys,
    x_bar=None,
    u_bar=None,
    t: float = 0.0,
    params=None,
    *,
    method: str = "quadratic",
    Q=None,
    window=None,
    samples: int = None,
    search: str = "auto",
) -> LyapunovCertificate:
    """
    Certify a region of attraction of ``sys`` around an equilibrium.

    Parameters
    ----------
    sys : DynamicSystem
        Autonomous system, normally a closed loop such as ``ctl @ plant``.
    x_bar : array of shape (n,), optional
        Guess of the equilibrium; refined by
        :func:`~minilink.analysis.equilibria.find_equilibrium`. Defaults to
        ``sys.x0``. The refined point is what the certificate reports: a law
        tuned by learning often settles beside the state it was aimed at.
    u_bar, t, params : optional
        Held input, time, and parameter set, as in every analysis tool.
    method : {"quadratic", "sos"}
        Family the Lyapunov function comes from. ``"quadratic"`` solves the
        Lyapunov equation of the linearization, the textbook construction and
        the only one implemented; ``"sos"`` is reserved for a sum-of-squares
        certificate, which would also make the level a proof rather than an
        estimate.
    Q : array of shape (n, n), optional
        Right-hand side of the Lyapunov equation; the identity by default. A
        larger ``Q`` in one direction weighs that direction more in ``V``.
    window : BoxSet or (lower, upper), optional
        Box the sublevel search covers. The default searches twice, first on
        the natural scale of ``V`` and then on a box sized to that answer, so
        the samples land where the level is decided.
    samples : int, optional
        Sampling budget per pass (default ``201**2`` in one or two states,
        50 000 above).
    search : {"auto", "grid", "random"}
        How the sublevel set is sampled. ``"grid"`` sweeps a regular grid,
        the default in one or two states and what the plots draw;
        ``"random"`` uses a low-discrepancy sequence, which scales better.

    Returns
    -------
    LyapunovCertificate

    Raises
    ------
    ValueError
        If the linearization is not Hurwitz: an unstable equilibrium has no
        region of attraction to certify.
    NotImplementedError
        For ``method="sos"``.
    """
    if method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}, got {method!r}")
    if method == "sos":
        raise NotImplementedError(
            "sum-of-squares certificates are not implemented; "
            "see docs/plans/lyapunov-certificates.md"
        )

    x_bar = find_equilibrium(sys, sys.x0 if x_bar is None else x_bar, u_bar, t, params)
    u_bar = np.asarray(
        sys.get_u_from_input_ports() if u_bar is None else u_bar, dtype=float
    )
    P, Q, poles = lyapunov_matrix(sys, x_bar, u_bar, t, params, Q)

    backend, evaluator = compiled(sys)
    f_many = dynamics_on(backend, evaluator, u_bar, t, params)
    domain = state_box(sys)
    n = int(sys.n)
    search = ("grid" if n <= 2 else "random") if search == "auto" else search
    samples = (201**2 if n <= 2 else 50_000) if samples is None else int(samples)

    # One pass on the natural scale of V, then a second on a box sized to its
    # answer: the level is decided by the samples nearest its own boundary.
    box = natural_window(x_bar, P, domain) if window is None else as_box(window)
    level, limiting, spread = search_level(
        box, x_bar, P, f_many, domain, samples, search
    )
    if window is None:
        box = level_window(x_bar, P, level, domain)
        refined, refined_limiting, refined_spread = search_level(
            box, x_bar, P, f_many, domain, samples, search
        )
        if refined < level:  # a finer sweep only ever tightens the claim
            level, limiting, spread = refined, refined_limiting, refined_spread

    return LyapunovCertificate(
        sys=sys,
        x_bar=x_bar,
        P=P,
        level=float(level),
        Q=Q,
        poles=poles,
        window=box,
        limiting_state=limiting,
        sample_spread=spread,
        method=method,
        u_bar=u_bar,
        t=t,
        params=params,
        backend=backend,
        evaluator=evaluator,
    )


def plot_region_of_attraction(
    certificate: LyapunovCertificate,
    x_axis: int = 0,
    y_axis: int = 1,
    *,
    basin: bool = False,
    detail: bool = True,
    verified: int = 0,
    trajectories=(),
    n: int = 201,
    limits=None,
    ax=None,
    show: bool = True,
):
    """
    Draw a slice of a certified region: the level set, and what pins it.

    The two swept states are ``x_axis`` and ``y_axis``; the others are pinned
    at the equilibrium, so what is drawn is the *slice* of the certified set
    through ``x_bar`` (smaller than its shadow on the two axes whenever the
    states are coupled).

    Layers, each in the legend: ``basin=True`` shades the states of the slice
    that really converge, by simulation, which shows how much a quadratic
    ``V`` gives up. ``detail=True`` adds the curve where ``V`` stops
    decreasing, and the sample that set the level when it lies on this slice
    — on a slice the two need not touch, since the binding state usually sits
    off the plane. ``verified=200`` draws the states
    :meth:`LyapunovCertificate.verify` would test, marked by whether they
    converged. They lie *inside* the certified set, because the claim is what
    is on trial and a state there that fails is a counterexample; whether
    states outside converge is what ``basin`` answers. Two-state loops only,
    where the draw lies on the plane.
    ``trajectories`` is a sequence of initial states to overlay.

    ``limits`` is an explicit ``((x_min, x_max), (y_min, y_max))`` window;
    the default frames the slices being drawn.
    """
    import matplotlib.pyplot as plt

    if limits is None:
        center = certificate.x_bar[[x_axis, y_axis]]
        half = 2.5 * certificate.slice_extent(
            x_axis, y_axis
        )  # the slice, not its shadow
        limits = (
            (center[0] - half[0], center[0] + half[0]),
            (center[1] - half[1], center[1] + half[1]),
        )
    if ax is None:
        _, ax = plt.subplots(figsize=(7.0, 5.5))

    draw_region(
        certificate,
        ax,
        x_axis,
        y_axis,
        limits,
        basin=basin,
        detail=detail,
        verified=verified,
        trajectories=trajectories,
        n=n,
    )
    ax.figure.tight_layout()
    if show:
        plt.show()
    return ax.figure, ax


# Internal machinery


def draw_region(
    cert,
    ax,
    x_axis,
    y_axis,
    limits,
    *,
    basin,
    detail,
    verified,
    trajectories,
    n,
):
    """Draw one certificate on one axis, and label every layer it drew."""
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    grid_x = np.linspace(limits[0][0], limits[0][1], n)
    grid_y = np.linspace(limits[1][0], limits[1][1], n)
    GX, GY = np.meshgrid(grid_x, grid_y)
    states = np.tile(cert.x_bar, (GX.size, 1))
    states[:, x_axis] = GX.ravel()
    states[:, y_axis] = GY.ravel()
    horizon = 10.0 / abs(cert.rate)
    settled = 0.05 * np.max(cert.extent)
    keys = []

    if basin:
        final = cert.rollout(horizon)(states)
        reached = np.linalg.norm(final - cert.x_bar, axis=1) < settled
        ax.contourf(
            GX, GY, reached.reshape(GX.shape), levels=[0.5, 1.5], colors=["#cfe3f7"]
        )
        keys.append(
            Patch(facecolor="#cfe3f7", label="empirical: converged in simulation")
        )

    ax.contour(
        GX,
        GY,
        cert.V(states).reshape(GX.shape),
        levels=[cert.level],
        colors="tab:red",
        linewidths=2.0,
    )
    keys.append(
        Line2D(
            [],
            [],
            color="tab:red",
            linewidth=2.0,
            label=r"theory: Lyapunov guarantee $V \leq c$",
        )
    )

    if detail:
        # Draw where V stops decreasing only if it actually does on this slice.
        # V̇ vanishes at the equilibrium as well, to within rounding, and that
        # speck is not a curve — so compare against the scale of V̇ itself.
        v_dot = cert.V_dot(states)
        if (v_dot > 1e-9 * np.max(np.abs(v_dot))).any():
            ax.contour(
                GX,
                GY,
                v_dot.reshape(GX.shape),
                levels=[0.0],
                colors="tab:orange",
                linewidths=1.0,
                linestyles="--",
            )
            keys.append(
                Line2D(
                    [],
                    [],
                    color="tab:orange",
                    linewidth=1.0,
                    linestyle="--",
                    label=r"$\dot V = 0$",
                )
            )
        off_slice = np.delete(cert.limiting_state - cert.x_bar, [x_axis, y_axis])
        if off_slice.size == 0 or np.allclose(off_slice, 0.0, atol=1e-9):
            ax.plot(
                cert.limiting_state[x_axis],
                cert.limiting_state[y_axis],
                "s",
                color="tab:orange",
                markersize=6,
            )
            keys.append(
                Line2D(
                    [],
                    [],
                    color="tab:orange",
                    marker="s",
                    linestyle="",
                    label="level pinned here",
                )
            )

    if verified and int(cert.sys.n) == 2:  # the draw lies on the plane only then
        x0 = cert.x_bar + sample_in_ellipsoid(cert.P, cert.level, int(verified))
        final = cert.rollout(horizon)(x0)
        converged = np.linalg.norm(final - cert.x_bar, axis=1) < 0.01 * np.max(
            cert.extent
        )
        ax.plot(
            x0[converged, x_axis],
            x0[converged, y_axis],
            ".",
            color="tab:green",
            markersize=3,
        )
        keys.append(
            Line2D(
                [],
                [],
                color="tab:green",
                marker=".",
                linestyle="",
                label=f"verify(): {int(converged.sum())}/{int(verified)} inside converged",
            )
        )
        if not converged.all():
            ax.plot(
                x0[~converged, x_axis],
                x0[~converged, y_axis],
                "x",
                color="k",
                markersize=5,
            )
            keys.append(
                Line2D(
                    [],
                    [],
                    color="k",
                    marker="x",
                    linestyle="",
                    label="counterexample: did not converge",
                )
            )

    x0_saved = np.asarray(cert.sys.x0, dtype=float).copy()
    try:
        for x0 in trajectories:
            cert.sys.x0 = np.asarray(x0, dtype=float)
            traj = cert.sys.compute_trajectory(tf=horizon, verbose=False)
            ax.plot(traj.x[x_axis], traj.x[y_axis], color="0.35", linewidth=1.0)
            ax.plot(x0[x_axis], x0[y_axis], "o", color="0.35", markersize=4)
    finally:
        cert.sys.x0 = x0_saved
    if len(trajectories):
        keys.append(Line2D([], [], color="0.35", linewidth=1.0, label="trajectory"))

    ax.plot(cert.x_bar[x_axis], cert.x_bar[y_axis], "k*", markersize=12)
    keys.append(
        Line2D([], [], color="k", marker="*", linestyle="", label="equilibrium")
    )

    labels = list(cert.sys.state.labels)
    ax.set_xlabel(labels[x_axis])
    ax.set_ylabel(labels[y_axis])
    notes = [] if int(cert.sys.n) == 2 else ["slice at the equilibrium"]
    if cert.sample_limited:
        notes.append("sample-limited")
    title = textwrap.fill(f"{cert.sys.name}: region of attraction", width=52)
    if notes:
        title += "\n(" + ", ".join(notes) + ")"
    ax.set_title(title, fontsize=10)
    ax.set_xlim(grid_x[0], grid_x[-1])
    ax.set_ylim(grid_y[0], grid_y[-1])
    ax.grid(True, alpha=0.3)
    ax.legend(handles=keys, loc="upper right", fontsize=8, framealpha=0.9)


def lyapunov_matrix(sys, x_bar, u_bar, t, params, Q):
    """Return ``(P, Q, poles)`` of the quadratic family at a stable equilibrium."""
    A = np.asarray(jacobian(sys, "f", "x", x_bar, u_bar, t, params), dtype=float)
    poles = np.linalg.eigvals(A)
    if poles.real.max() >= 0.0:
        worst = poles[int(np.argmax(poles.real))]
        raise ValueError(
            f"the loop is not stable at this equilibrium (pole at {worst:.4g}); "
            "there is no region of attraction to certify"
        )
    Q = np.eye(int(sys.n)) if Q is None else np.asarray(Q, dtype=float)
    return solve_continuous_lyapunov(A.T, -Q), Q, poles


def state_box(sys) -> BoxSet:
    """The model's own domain: the state-port bounds, infinite where unset."""
    n = int(sys.n)
    lower = sys.state.lower_bound
    upper = sys.state.upper_bound
    lower = np.full(n, -np.inf) if lower is None else np.asarray(lower, dtype=float)
    upper = np.full(n, np.inf) if upper is None else np.asarray(upper, dtype=float)
    return BoxSet(lower, upper)


def as_box(window) -> BoxSet:
    """Coerce a user window: a ``BoxSet`` or a ``(lower, upper)`` pair."""
    if isinstance(window, BoxSet):
        return window
    lower, upper = window
    return BoxSet(np.asarray(lower, dtype=float), np.asarray(upper, dtype=float))


def natural_window(x_bar, P, domain: BoxSet) -> BoxSet:
    """First-pass box: the extent of ``{V <= 9}``, trimmed to the model's domain."""
    return clip_box(x_bar, 3.0 * np.sqrt(np.diag(np.linalg.inv(P))), domain)


def level_window(x_bar, P, level, domain: BoxSet) -> BoxSet:
    """Second-pass box: half again the extent of the level set just found."""
    return clip_box(x_bar, 1.5 * np.sqrt(level * np.diag(np.linalg.inv(P))), domain)


def clip_box(x_bar, half_width, domain: BoxSet) -> BoxSet:
    """Box of the given half-width about ``x_bar``, trimmed to ``domain``."""
    return BoxSet(
        np.maximum(x_bar - half_width, domain.lower),
        np.minimum(x_bar + half_width, domain.upper),
    )


def search_level(window, x_bar, P, f_many, domain, samples, search):
    """
    Largest sampled level ``c`` whose sublevel set only holds decreasing states.

    Returns ``(level, limiting_state, spread)``. A sample blocks the level when
    ``V̇ >= 0`` there, when it lies outside the model's domain, or when it sits
    on the window's face — the last so the certified set never reaches past
    what was actually checked.

    ``spread`` is the relative disagreement between the levels two disjoint
    halves of the same samples would give. It costs nothing extra and it is
    how the search says whether it has enough samples: a sharp answer has a
    spread near zero, while in many states the samples thin out and the
    halves disagree (see :attr:`LyapunovCertificate.sample_limited`).
    """
    states, on_face = sample_box(window, samples, search)
    d = states - x_bar
    v = np.einsum("ij,jk,ik->i", d, P, d)
    v_dot = 2.0 * np.einsum("ij,jk,ik->i", d, P, f_many(states))

    outside = np.any(states < domain.lower, axis=1) | np.any(
        states > domain.upper, axis=1
    )
    away = v > 1e-10 * v.max()  # the equilibrium itself has V̇ = 0
    blocked = ((v_dot >= 0.0) | outside | on_face) & away

    halves = [blocked_level(blocked[half], v[half]) for half in (EVEN, ODD)]
    spread = abs(halves[0] - halves[1]) / max(halves[0], halves[1], 1e-300)

    if not blocked.any():  # nothing stops V inside the window
        return float(v.max()), states[int(np.argmax(v))], spread
    limiting = int(np.argmin(np.where(blocked, v, np.inf)))
    return float(v[limiting]), states[limiting], spread


#: The two disjoint halves the level search scores itself on.
EVEN = slice(0, None, 2)
ODD = slice(1, None, 2)


def blocked_level(blocked, v) -> float:
    """Level implied by one subset of samples: the smallest blocked ``V``."""
    return float(v[blocked].min()) if blocked.any() else float(v.max())


def sample_box(window: BoxSet, samples: int, search: str):
    """Return ``(states, on_face)``: points filling the box, flagged on its faces."""
    lower, upper = window.lower, window.upper
    n = int(lower.size)
    if search == "grid":
        per_axis = max(3, int(round(samples ** (1.0 / n))))
        axes = [np.linspace(lower[i], upper[i], per_axis) for i in range(n)]
        mesh = np.meshgrid(*axes, indexing="ij")
        states = np.stack([m.ravel() for m in mesh], axis=1)
        on_face = np.any(states <= lower, axis=1) | np.any(states >= upper, axis=1)
        return states, on_face
    if search != "random":
        raise ValueError(f"search must be 'auto', 'grid' or 'random', got {search!r}")

    from scipy.stats import qmc

    unit = qmc.Halton(d=n, scramble=False).random(int(samples) + 1)[1:]
    interior = lower + unit * (upper - lower)

    # A tenth as many points on the faces: snap one coordinate to a bound.
    faces = interior[: max(2 * n, int(samples) // 10)].copy()
    which = np.arange(faces.shape[0]) % n
    side = (np.arange(faces.shape[0]) // n) % 2
    faces[np.arange(faces.shape[0]), which] = np.where(
        side == 0, lower[which], upper[which]
    )
    states = np.vstack([interior, faces])
    on_face = np.zeros(states.shape[0], dtype=bool)
    on_face[interior.shape[0] :] = True
    return states, on_face


def sample_in_ellipsoid(P, level, n: int) -> np.ndarray:
    """``n`` offsets drawn uniformly inside ``{d : dᵀ P d <= level}``."""
    rng = np.random.default_rng(0)  # a certificate must read the same twice
    dim = int(P.shape[0])
    z = rng.standard_normal((n, dim))
    z *= (rng.random((n, 1)) ** (1.0 / dim)) / np.linalg.norm(z, axis=1, keepdims=True)
    L = np.linalg.cholesky(P)  # P = L Lᵀ, so d = sqrt(c) L⁻ᵀ z satisfies dᵀPd <= c
    return np.sqrt(level) * solve_triangular(L.T, z.T, lower=False).T


def compiled(sys):
    """Compile once for the whole certificate: ``(backend, evaluator)``."""
    from minilink.core.compile.compiler import compile_auto

    return compile_auto(sys)


def dynamics_on(backend, evaluator, u_bar, t, params):
    """Return ``f_many(X) -> dX`` evaluating ``f`` on a stack of states."""
    from minilink.core.backends import BACKEND_JAX

    if backend == BACKEND_JAX:
        import jax
        import jax.numpy as jnp

        if params is None:
            batch = jax.jit(jax.vmap(lambda x: evaluator.f_trace(x, u_bar, t)))
        else:
            batch = jax.jit(
                jax.vmap(lambda x: evaluator.f_trace_p(x, u_bar, t, params))
            )
        return lambda states: np.asarray(batch(jnp.asarray(states)))

    if params is None:
        return lambda states: np.stack([evaluator.f(x, u_bar, t) for x in states])
    return lambda states: np.stack([evaluator.f_p(x, u_bar, t, params) for x in states])


def rollout_on(backend, evaluator, u_bar, t, dt: float, tf: float):
    """Return ``rollout(X0) -> X_final`` integrating each state for ``tf``."""
    from minilink.core.backends import BACKEND_JAX

    n_steps = max(1, int(round(tf / dt)))

    if backend == BACKEND_JAX:
        import jax
        import jax.numpy as jnp

        def one(x0):
            def step(x, _):
                return evaluator.rk4_step_trace(x, u_bar, t, dt), None

            return jax.lax.scan(step, x0, None, length=n_steps)[0]

        batch = jax.jit(jax.vmap(one))
        return lambda states: np.asarray(batch(jnp.asarray(states)))

    def rollout_numpy(states):
        final = np.empty_like(states)
        for i, x in enumerate(states):
            for _ in range(n_steps):
                x = evaluator.rk4_step(x, u_bar, t, dt)
            final[i] = x
        return final

    return rollout_numpy


if __name__ == "__main__":
    from minilink import InvertedPendulum
    from minilink.control.lqr import lqr_at_operating_point

    plant = InvertedPendulum()
    plant.inputs["u"].lower_bound = np.array([-2.0])  # too weak to hold it far over
    plant.inputs["u"].upper_bound = np.array([2.0])
    ctl = lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1))
    certificate = region_of_attraction(ctl @ plant)
    print(certificate)
    print(certificate.verify())
