"""Linear analysis on the matrices ``(A, B, C, D)`` of one channel: poles, zeros, gain, minimal realization, frequency response, margins, root locus, step response."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import linalg

from minilink.analysis.structural import RANK_TOL, controllability, observability

# =============================================================================
# Public API — poles, zeros, response
# =============================================================================


def poles(A):
    """Eigenvalues of ``A``."""
    A = np.asarray(A, dtype=float)
    if A.size == 0:
        return np.array([], dtype=complex)

    # p = eig(A)
    p = np.linalg.eigvals(A)

    return p


def zeros(A, B, C, D):
    """Transmission zeros of the square channel ``(A, B, C, D)``: the finite eigenvalues of its Rosenbrock pencil."""
    A, B, C, D = as_matrices(A, B, C, D)
    n, m = B.shape
    if n == 0 or m != C.shape[0]:
        return np.array([], dtype=complex)

    # z = α / β over the eigenpairs of  [[A, B], [C, D]] − s [[I, 0], [0, 0]]  with β ≠ 0
    P = np.block([[A, B], [C, D]])
    E = np.zeros_like(P)
    E[:n, :n] = np.eye(n)
    alpha, beta = linalg.eigvals(P, E, homogeneous_eigvals=True)
    finite = np.abs(beta) > _INFINITE_ZERO_RATIO * np.abs(alpha)
    z = alpha[finite] / beta[finite]

    return z


def gain(A, B, C, D):
    """Leading coefficient ``k`` of the factored transfer function of a SISO channel.

    ``d`` when the channel has feedthrough, otherwise the first nonzero Markov
    parameter; read off the response at one real point beyond every pole and zero.
    """
    A, B, C, D = as_siso_matrices(A, B, C, D)
    n = A.shape[0]

    # The roots of G(s) = k ∏(s − z) / ∏(s − p)
    z = zeros(A, B, C, D)
    p = poles(A)

    # One real point beyond every root, where the factored and state-space forms agree
    # (a Markov-parameter scan would need a tolerance that fails on a badly scaled channel)
    s0 = 2.0 * open_loop_radius(p, z)

    # G(s0) = C (s0 I − A)⁻¹ B + D
    G_s0 = (C @ np.linalg.solve(s0 * np.eye(n) - A, B) + D)[0, 0]

    # k = G(s0) ∏(s0 − p) / ∏(s0 − z), the products as sums of logarithms so
    # that hundreds of roots do not overflow
    log_ratio = np.sum(np.log(s0 - p)) - np.sum(np.log(s0 - z))
    k = G_s0 * np.exp(log_ratio)

    return float(np.real(k))


def frequency_response(A, B, C, D, w):
    """Frequency response of a SISO channel, one complex value per frequency in ``w``."""
    A, B, C, D = as_siso_matrices(A, B, C, D)
    w = np.asarray(w, dtype=float).reshape(-1)
    I = np.eye(A.shape[0])

    # G(jω) = C (jω I − A)⁻¹ B + D
    G = np.empty(w.size, dtype=complex)
    for k, omega in enumerate(w):
        G[k] = (C @ np.linalg.solve(1j * omega * I - A, B) + D)[0, 0]

    return G


def frequency_range(A, B, C, D):
    """``(w_min, w_max)`` covering the dynamics *and* any 0 dB crossing.

    One decade beyond the slowest and fastest nonzero pole or zero, then
    widened until the band brackets ``|G| = 1``. The widening matters: roots
    at the origin carry no rate, so an integrator's crossover sits below the
    pole-derived band, and a large static gain pushes it above — in both
    cases a band read from the roots alone misses the crossover and
    :func:`margins` would report an infinite margin.
    """
    rates = np.abs(np.concatenate([poles(A), zeros(A, B, C, D)]))
    rates = rates[rates > 1e-9]
    if rates.size == 0:
        w_min, w_max = 1e-2, 1e2
    else:
        # One decade past min |λ| and max |λ|
        w_min = 10.0 ** np.floor(np.log10(rates.min()) - 1.0)
        w_max = 10.0 ** np.ceil(np.log10(rates.max()) + 1.0)

    # Widen until the band brackets |G| = 1
    w_min, w_max = bracket_unit_gain(A, B, C, D, w_min, w_max)

    return w_min, w_max


# =============================================================================
# Public API — minimal realization
# =============================================================================


def minreal(A, B, C, D, *, tol=RANK_TOL):
    """Minimal realization: drop the modes that are uncontrollable or unobservable.

    The pole/zero pairs of those modes cancel; ``pzmap``, ``root_locus`` and
    ``transfer_function`` run it by default.
    ``tol`` is the relative singular-value threshold below which a direction
    counts as missing, the one :func:`~minilink.analysis.structural.controllability`
    and :func:`~minilink.analysis.structural.observability` use. A realization that is
    already minimal comes back in its own coordinates.
    """
    A, B, C, D = as_matrices(A, B, C, D)
    n = A.shape[0]

    # Controllable subspace: 𝒞 = U Σ Vᵀ, keep the r directions with σᵢ > tol σ₁
    U, sigma, _ = np.linalg.svd(controllability(A, B).matrix)
    r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))
    T_c = U[:, :r] if r < n else np.eye(n)

    # Restrict to it: x = T_c z  ⇒  A_c = T_cᵀ A T_c,  B_c = T_cᵀ B,  C_c = C T_c
    A_c = T_c.T @ A @ T_c
    B_c = T_c.T @ B
    C_c = C @ T_c

    # Observable part of it: 𝒪ᵀ = U Σ Vᵀ, keep the q directions with σᵢ > tol σ₁
    U, sigma, _ = np.linalg.svd(observability(A_c, C_c).matrix.T)
    q = int(np.sum(sigma > tol * sigma.max(initial=0.0)))
    T_o = U[:, :q] if q < r else np.eye(r)

    # Restrict again: z = T_o w  ⇒  the minimal realization, D unchanged
    A_min = T_o.T @ A_c @ T_o
    B_min = T_o.T @ B_c
    C_min = C_c @ T_o

    return A_min, B_min, C_min, D


# =============================================================================
# Public API — margins
# =============================================================================


@dataclass(frozen=True)
class Margins:
    """Gain and phase margins of a loop transfer function ``L(jw)``."""

    gain_margin_db: float  # inf when the phase never crosses -180 deg
    phase_margin_deg: float  # inf when the magnitude never crosses 0 dB
    w_gain_crossover: float  # rad/s where |L| = 1 (phase margin is read here)
    w_phase_crossover: float  # rad/s where arg L = -180 deg (gain margin is read here)


def margins(w, G):
    """Margins from a sampled response: crossings found by interpolation."""
    w = np.asarray(w, dtype=float).reshape(-1)

    # Bode coordinates: |L| in dB and the unwrapped ∠L in degrees
    magnitude_db = 20.0 * np.log10(np.abs(np.asarray(G)))
    phase_deg = np.degrees(np.unwrap(np.angle(np.asarray(G))))

    # Gain crossovers |L(jω_gc)| = 1, with the phase there
    w_gc, phase_gc = gain_crossovers(w, magnitude_db, phase_deg)

    # PM = 180° + ∠L(jω_gc), folded into [−180°, 180°)
    pm = (phase_gc + 180.0 + 180.0) % 360.0 - 180.0

    # Phase crossovers ∠L(jω_pc) = −180° (mod 360°), with the magnitude there
    w_pc, magnitude_pc = phase_crossovers(w, magnitude_db, phase_deg)

    # GM = −|L(jω_pc)| in dB
    gm = -magnitude_pc

    # The loop is as robust as its smallest margin of each kind
    pm, w_gc = smallest_margin(pm, w_gc)
    gm, w_pc = smallest_margin(gm, w_pc)

    return Margins(float(gm), float(pm), float(w_gc), float(w_pc))


# =============================================================================
# Public API — root locus
# =============================================================================


def closed_loop_poles(A, B, C, D, K):
    """Poles of the SISO loop closed with ``u = -K y``."""
    A, B, C, D = as_siso_matrices(A, B, C, D)

    # The algebraic loop 1 + K d; at K = −1/d it is singular and no pole is finite
    loop = 1.0 + K * D[0, 0]
    if loop == 0.0:
        return np.full(A.shape[0], np.inf, dtype=complex)

    # p = eig(A − B K (1 + K d)⁻¹ C)
    p = np.linalg.eigvals(A - (K / loop) * (B @ C))

    return p


def root_locus(A, B, C, D, gains=None, *, n_gains=400):
    """Closed-loop poles over a gain sweep, one continuous branch per column.

    ``gains`` defaults to ``0`` followed by six log-spaced decades ending
    where the far branches reach ten times the radius of the open-loop
    poles and zeros (``n_gains`` points); consecutive gain points are refined
    while any branch moves by more than two percent of that radius. Returns
    ``(gains, roots)`` with ``roots`` of shape ``(len(gains), n_states)``.
    """
    A, B, C, D = as_siso_matrices(A, B, C, D)
    if gains is None:
        gains = default_gains(A, B, C, D, n_gains)
    gains = np.asarray(gains, dtype=float).reshape(-1)

    # p(K) = eig(A − B K (1 + K d)⁻¹ C), each branch continued from the previous gain
    roots = [closed_loop_poles(A, B, C, D, gains[0])]
    for K in gains[1:]:
        roots.append(matched_branches(roots[-1], closed_loop_poles(A, B, C, D, K)))

    # Refine the gains wherever a branch jumps
    gains, roots = refine_jumps(A, B, C, D, list(gains), roots)

    return np.asarray(gains), np.asarray(roots)


# =============================================================================
# Public API — time response
# =============================================================================


def step_response(A, B, C, D, t):
    """Unit-step response from rest, exact on a uniform time grid ``t``.

    One matrix exponential gives the zero-order-hold pair ``(A_d, B_d)``; the
    state is then marched exactly.
    """
    A, B, C, D = as_siso_matrices(A, B, C, D)
    t = np.asarray(t, dtype=float).reshape(-1)
    dt = uniform_step(t)
    n, m = B.shape

    # expm([[A, B], [0, 0]] Δt) = [[A_d, B_d], [0, I]]
    hold = linalg.expm(np.block([[A, B], [np.zeros((m, n + m))]]) * dt)
    A_d = hold[:n, :n]
    B_d = hold[:n, n:]

    # y_k = C x_k + D u,   x_{k+1} = A_d x_k + B_d u,   from x_0 = 0 under u = 1
    u = np.ones(m)
    x = np.zeros(n)
    y = np.empty(t.size)
    for k in range(t.size):
        y[k] = (C @ x + D @ u)[0]
        x = A_d @ x + B_d @ u

    return y


def settling_horizon(A):
    """Default step-response horizon: eight times the slowest stable time constant."""
    decay = -np.real(poles(A))
    decay = decay[decay > 1e-9]
    if decay.size == 0:
        return 10.0

    # Eight times the slowest time constant 1/σ, σ = −Re p of the stable poles
    horizon = float(8.0 / decay.min())

    return horizon


# =============================================================================
# Internal machinery
# =============================================================================

# A pencil eigenvalue with |β| below this fraction of |α| is infinite, not a zero:
# rounding turns the pencil's infinite eigenvalues into finite ones near 1e15
_INFINITE_ZERO_RATIO = 1e-8


def bracket_unit_gain(A, B, C, D, w_min, w_max, *, decades=8):
    """Widen ``(w_min, w_max)`` until the band brackets ``|G| = 1``.

    Downward while the magnitude is below one *and still climbing steeply*
    (an integrator gains a decade per decade; a static gain approaches its
    plateau and stops the walk), upward while it is above one. At most
    ``decades`` steps each way.
    """

    def magnitude(w):
        return abs(frequency_response(A, B, C, D, [w])[0])

    low = magnitude(w_min)
    for _ in range(decades):
        if low >= 1.0:
            break
        lower = magnitude(0.1 * w_min)
        if lower <= 2.0 * low:  # approaching the DC plateau, not integrating
            break
        w_min, low = 0.1 * w_min, lower

    for _ in range(decades):
        if magnitude(w_max) <= 1.0:
            break
        w_max = 10.0 * w_max

    return w_min, w_max


def as_matrices(A, B, C, D):
    A = np.atleast_2d(np.asarray(A, dtype=float))
    B = np.atleast_2d(np.asarray(B, dtype=float))
    C = np.atleast_2d(np.asarray(C, dtype=float))
    D = np.atleast_2d(np.asarray(D, dtype=float))
    if A.size == 0:
        A = np.zeros((0, 0))
        B = np.zeros((0, D.shape[1]))
        C = np.zeros((D.shape[0], 0))
    return A, B, C, D


def as_siso_matrices(A, B, C, D):
    """``as_matrices`` for a channel with one input and one output, which it checks."""
    A, B, C, D = as_matrices(A, B, C, D)
    if B.shape[1] != 1 or C.shape[0] != 1:
        raise ValueError(
            "this function takes one input and one output (a SISO channel); got "
            f"{B.shape[1]} inputs and {C.shape[0]} outputs"
        )
    return A, B, C, D


def sign_changes(values):
    """Indices ``k`` where ``values[k]`` and ``values[k + 1]`` have opposite signs."""
    signs = np.sign(values)
    return np.flatnonzero(signs[:-1] * signs[1:] < 0)


def interpolate_crossing(w_pair, values_pair, target):
    """Frequency where a linearly interpolated segment crosses ``target``."""
    (w0, w1), (v0, v1) = w_pair, values_pair
    return w0 + (target - v0) * (w1 - w0) / (v1 - v0)


def gain_crossovers(w, magnitude_db, phase_deg):
    """Frequencies where ``|L|`` crosses 1 (interpolated), with the phase there."""
    w_c, phase_c = [], []
    for k in sign_changes(magnitude_db):
        w_k = interpolate_crossing(w[k : k + 2], magnitude_db[k : k + 2], 0.0)
        w_c.append(w_k)
        phase_c.append(np.interp(w_k, w[k : k + 2], phase_deg[k : k + 2]))
    return np.array(w_c, dtype=float), np.array(phase_c, dtype=float)


def phase_crossovers(w, magnitude_db, phase_deg):
    """Frequencies where ``arg L`` crosses -180 deg (mod 360, interpolated), with the magnitude there."""
    w_c, magnitude_c = [], []
    lowest = int(np.floor((phase_deg.min() + 180.0) / 360.0))
    highest = int(np.ceil((phase_deg.max() + 180.0) / 360.0))
    for k_wrap in range(lowest, highest + 1):
        target = -180.0 + 360.0 * k_wrap
        for k in sign_changes(phase_deg - target):
            w_k = interpolate_crossing(w[k : k + 2], phase_deg[k : k + 2], target)
            w_c.append(w_k)
            magnitude_c.append(np.interp(w_k, w[k : k + 2], magnitude_db[k : k + 2]))
    return np.array(w_c, dtype=float), np.array(magnitude_c, dtype=float)


def smallest_margin(margins, frequencies):
    """The margin of least magnitude and its frequency; ``inf`` when there is no crossover."""
    best, w_best = np.inf, np.inf
    for margin, w_c in zip(margins, frequencies):
        if abs(margin) < abs(best):
            best, w_best = margin, w_c
    return best, w_best


def uniform_step(t):
    """The step ``dt`` of a uniform time grid of at least two samples."""
    if t.size < 2:
        raise ValueError("step_response needs at least two time samples")
    dt = t[1] - t[0]
    if not np.allclose(np.diff(t), dt):
        raise ValueError("step_response needs a uniform time grid")
    return dt


def open_loop_radius(p, z):
    """Radius of the open-loop poles and zeros, at least 1."""
    return max(np.max(np.abs(np.concatenate([p, z])), initial=0.0), 1.0)


def default_gains(A, B, C, D, n_gains):
    """``0`` then six log decades ending where the far branches leave the picture."""
    radius = open_loop_radius(poles(A), zeros(A, B, C, D))
    k_max = gain_reaching(A, B, C, D, 10.0 * radius)
    return np.concatenate(
        [[0.0], np.logspace(np.log10(k_max) - 6.0, np.log10(k_max), n_gains)]
    )


def refine_jumps(A, B, C, D, gains, roots):
    """Insert the midpoint gain wherever a branch jumps more than two percent of the radius.

    An interval that reaches or crosses the singular gain ``K = -1/d`` is left as
    is: the branch passes through infinity there, and no midpoint shrinks the jump.
    """
    step = 0.02 * open_loop_radius(poles(A), zeros(A, B, C, D))
    d = D[0, 0]
    k = 0
    while k < len(gains) - 1:
        regular = (1.0 + gains[k] * d) * (1.0 + gains[k + 1] * d) > 0.0
        if (
            regular
            and np.max(np.abs(roots[k + 1] - roots[k])) > step
            and gains[k + 1] - gains[k] > 1e-12 * abs(gains[k + 1])
        ):
            K_mid = 0.5 * (gains[k] + gains[k + 1])
            gains.insert(k + 1, K_mid)
            roots.insert(
                k + 1, matched_branches(roots[k], closed_loop_poles(A, B, C, D, K_mid))
            )
            roots[k + 2] = matched_branches(roots[k + 1], roots[k + 2])
        else:
            k += 1
    return gains, roots


def gain_reaching(A, B, C, D, radius):
    """Gain at which the farthest closed-loop pole reaches ``radius``.

    Stops early when doubling the gain no longer moves the poles (every
    branch has arrived at a finite zero).
    """
    K = 1.0
    previous = closed_loop_poles(A, B, C, D, K)
    for _ in range(60):
        if np.max(np.abs(previous), initial=0.0) >= radius:
            return K
        current = matched_branches(previous, closed_loop_poles(A, B, C, D, 2.0 * K))
        if np.max(np.abs(current - previous), initial=0.0) < 1e-4 * radius:
            return 2.0 * K
        K, previous = 2.0 * K, current
    return K


def matched_branches(previous, current):
    """Reorder ``current`` so each entry continues the nearest ``previous`` branch.

    Across the singular gain ``K = -1/d`` no pole is finite and there is nothing to
    match: ``current`` comes back in its own order.
    """
    from scipy.optimize import linear_sum_assignment

    if not (np.all(np.isfinite(previous)) and np.all(np.isfinite(current))):
        return current
    cost = np.abs(previous[:, None] - current[None, :])
    rows, cols = linear_sum_assignment(cost)
    ordered = np.empty_like(current)
    ordered[rows] = current[cols]
    return ordered
