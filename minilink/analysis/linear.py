"""Linear-algebra core of the frequency and time-response tools.

Every tool in :mod:`minilink.analysis.frequency` and
:mod:`minilink.analysis.time_response` reduces a system to one state-space
channel ``(A, b, c, d)`` — the linearization at the operating point, or the
model itself for an ``LTISystem`` — and calls the functions below. Nothing
here knows about ports, operating points or plotting backends: the inputs are
matrices, the outputs are arrays.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import linalg

# =============================================================================
# Public API — poles, zeros, response
# =============================================================================


def poles(A):
    """Eigenvalues of ``A``."""
    A = np.asarray(A, dtype=float)
    if A.size == 0:
        return np.array([], dtype=complex)

    # λ = eig(A)
    return np.linalg.eigvals(A)


def zeros(A, B, C, D):
    """Transmission zeros of the square channel ``(A, B, C, D)``.

    The finite generalized eigenvalues of the Rosenbrock pencil
    ``[[A, B], [C, D]] - s [[I, 0], [0, 0]]``; ``D`` must be square.
    """
    A, B, C, D = _matrices(A, B, C, D)
    n, m = B.shape
    if n == 0 or m != C.shape[0]:
        return np.array([], dtype=complex)

    # finite eig of  [[A, B], [C, D]] − s [[I, 0], [0, 0]]
    P = np.block([[A, B], [C, D]])
    E = np.zeros_like(P)
    E[:n, :n] = np.eye(n)
    values = linalg.eigvals(P, E)
    return values[np.isfinite(values)]


def gain(A, B, C, D):
    """Leading coefficient ``k`` of ``G(s) = k prod(s - z) / prod(s - p)``.

    ``d`` when the channel has feedthrough, otherwise the first nonzero
    Markov parameter ``c A^(r-1) b`` (``r`` the relative degree).
    """
    A, B, C, D = _matrices(A, B, C, D)

    # k = d  if the channel has feedthrough
    if abs(D[0, 0]) > 0.0:
        return float(D[0, 0])

    # else k = first nonzero Markov parameter  c A^{r-1} b
    markov = C @ B
    power = np.eye(A.shape[0]) if A.size else np.zeros((0, 0))
    scale = max(np.max(np.abs(A), initial=0.0), 1.0)
    for _ in range(A.shape[0]):
        if abs(markov[0, 0]) > 1e-12 * scale ** (A.shape[0]):
            return float(markov[0, 0])
        power = power @ A
        markov = C @ power @ B
    return 0.0


def frequency_response(A, B, C, D, w):
    """``G(jw) = C (jw I - A)^-1 B + D`` of a SISO channel, one complex value per ``w``."""
    A, B, C, D = _matrices(A, B, C, D)
    w = np.asarray(w, dtype=float).reshape(-1)
    if A.size == 0:
        return np.full(w.shape, complex(D[0, 0]))

    # G(jw) = C (jw I − A)^{-1} B + D
    I = np.eye(A.shape[0])
    G = np.empty(w.size, dtype=complex)
    for k, omega in enumerate(w):
        G[k] = (C @ np.linalg.solve(1j * omega * I - A, B) + D)[0, 0]
    return G


def frequency_range(A, B, C, D):
    """``(w_min, w_max)`` one decade beyond the slowest and fastest pole or zero."""
    rates = np.abs(np.concatenate([poles(A), zeros(A, B, C, D)]))
    rates = rates[rates > 1e-9]
    if rates.size == 0:
        return 1e-2, 1e2

    # one decade past min |λ| and max |λ|
    return 10.0 ** np.floor(np.log10(rates.min()) - 1.0), 10.0 ** np.ceil(
        np.log10(rates.max()) + 1.0
    )


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

    magnitude_db = 20.0 * np.log10(np.abs(np.asarray(G)))
    phase_deg = np.degrees(np.unwrap(np.angle(np.asarray(G))))

    # PM = 180° + ∠L at |L| = 1, wrapped to (−180, 180]
    # GM = −|L|_dB at ∠L = −180°
    pm, w_gc = _phase_margin_from_samples(w, magnitude_db, phase_deg)
    gm, w_pc = _gain_margin_from_samples(w, magnitude_db, phase_deg)
    return Margins(float(gm), float(pm), float(w_gc), float(w_pc))


# =============================================================================
# Public API — root locus
# =============================================================================


def closed_loop_poles(A, B, C, D, K):
    """Poles of the loop closed with ``u = -K y``: ``eig(A - B K (1 + K d)^-1 C)``."""
    A, B, C, D = _matrices(A, B, C, D)

    # λ = eig(A − B K (1 + K d)^{-1} C)
    feedback = K / (1.0 + K * D[0, 0])
    return np.linalg.eigvals(A - feedback * (B @ C))


def root_locus(A, B, C, D, gains=None, *, n=400):
    """Closed-loop poles over a gain sweep, one continuous branch per column.

    ``gains`` defaults to ``0`` followed by six log-spaced decades ending
    where the far branches reach ten times the radius of the open-loop
    poles and zeros; consecutive gain points are refined while any branch
    moves by more than two percent of that radius. Returns ``(gains, roots)``
    with ``roots`` of shape ``(len(gains), n_states)``.
    """
    A, B, C, D = _matrices(A, B, C, D)
    if gains is None:
        gains = _default_gains(A, B, C, D, n)
    gains = np.asarray(gains, dtype=float).reshape(-1)

    # λ(K) = eig(A − B K (1 + K d)^{-1} C)
    roots = [closed_loop_poles(A, B, C, D, gains[0])]
    for K in gains[1:]:
        roots.append(_matched(roots[-1], closed_loop_poles(A, B, C, D, K)))

    gains, roots = _refine_jumps(A, B, C, D, list(gains), roots)
    return np.asarray(gains), np.asarray(roots)


# =============================================================================
# Public API — time response
# =============================================================================


def step_response(A, B, C, D, t):
    """Unit-step response ``y(t)`` from rest, exact on a uniform time grid.

    One matrix exponential of the augmented ``[[A, B], [0, 0]]`` gives the
    zero-order-hold pair ``(A_d, B_d)``; the state is then marched exactly.
    """
    A, B, C, D = _matrices(A, B, C, D)
    t = np.asarray(t, dtype=float).reshape(-1)
    n, m = B.shape
    if t.size < 2:
        raise ValueError("step_response needs at least two time samples")
    dt = t[1] - t[0]
    if not np.allclose(np.diff(t), dt):
        raise ValueError("step_response needs a uniform time grid")

    # expm([[A, B], [0, 0]] dt) = [[A_d, B_d], [0, I]]
    # y_k = C x_k + D u,   x_{k+1} = A_d x_k + B_d u,   x_0 = 0, u = 1
    hold = linalg.expm(np.block([[A, B], [np.zeros((m, n + m))]]) * dt)
    A_d, B_d = hold[:n, :n], hold[:n, n:]
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

    # 8 / min σ   (σ = −Re λ of the stable poles)
    return float(8.0 / decay.min())


# =============================================================================
# Internal machinery
# =============================================================================


def _matrices(A, B, C, D):
    A = np.atleast_2d(np.asarray(A, dtype=float))
    B = np.atleast_2d(np.asarray(B, dtype=float))
    C = np.atleast_2d(np.asarray(C, dtype=float))
    D = np.atleast_2d(np.asarray(D, dtype=float))
    if A.size == 0:
        A = np.zeros((0, 0))
        B = np.zeros((0, D.shape[1]))
        C = np.zeros((D.shape[0], 0))
    return A, B, C, D


def _sign_changes(values):
    """Indices ``k`` where ``values[k]`` and ``values[k + 1]`` have opposite signs."""
    signs = np.sign(values)
    return np.flatnonzero(signs[:-1] * signs[1:] < 0)


def _interpolate(w_pair, values_pair, target):
    """Frequency where a linearly interpolated segment crosses ``target``."""
    (w0, w1), (v0, v1) = w_pair, values_pair
    return w0 + (target - v0) * (w1 - w0) / (v1 - v0)


def _phase_margin_from_samples(w, magnitude_db, phase_deg):
    """Phase margin at the gain crossover ``|L| = 1``: the smallest margin wins."""
    pm, w_gc = np.inf, np.inf
    for k in _sign_changes(magnitude_db):
        w_c = _interpolate(w[k : k + 2], magnitude_db[k : k + 2], 0.0)
        phase_c = np.interp(w_c, w[k : k + 2], phase_deg[k : k + 2])
        # 180 + ∠L, folded into (−180, 180]
        margin = (phase_c + 180.0 + 180.0) % 360.0 - 180.0
        if abs(margin) < abs(pm):
            pm, w_gc = margin, w_c
    return pm, w_gc


def _gain_margin_from_samples(w, magnitude_db, phase_deg):
    """Gain margin at the phase crossovers ``arg L = -180`` deg (mod 360)."""
    gm, w_pc = np.inf, np.inf
    lowest = int(np.floor((phase_deg.min() + 180.0) / 360.0))
    highest = int(np.ceil((phase_deg.max() + 180.0) / 360.0))
    for k_wrap in range(lowest, highest + 1):
        target = -180.0 + 360.0 * k_wrap
        for k in _sign_changes(phase_deg - target):
            w_c = _interpolate(w[k : k + 2], phase_deg[k : k + 2], target)
            gain_c = -np.interp(w_c, w[k : k + 2], magnitude_db[k : k + 2])
            if abs(gain_c) < abs(gm):
                gm, w_pc = gain_c, w_c
    return gm, w_pc


def _open_loop_radius(A, B, C, D):
    return max(
        np.max(np.abs(np.concatenate([poles(A), zeros(A, B, C, D)])), initial=0.0),
        1.0,
    )


def _default_gains(A, B, C, D, n):
    """``0`` then six log decades ending where the far branches leave the picture."""
    k_max = _gain_reaching(A, B, C, D, 10.0 * _open_loop_radius(A, B, C, D))
    return np.concatenate(
        [[0.0], np.logspace(np.log10(k_max) - 6.0, np.log10(k_max), n)]
    )


def _refine_jumps(A, B, C, D, gains, roots):
    """Insert the midpoint gain wherever a branch jumps more than two percent of the radius."""
    step = 0.02 * _open_loop_radius(A, B, C, D)
    k = 0
    while k < len(gains) - 1:
        if (
            np.max(np.abs(roots[k + 1] - roots[k])) > step
            and gains[k + 1] - gains[k] > 1e-12 * gains[k + 1]
        ):
            K_mid = 0.5 * (gains[k] + gains[k + 1])
            gains.insert(k + 1, K_mid)
            roots.insert(
                k + 1, _matched(roots[k], closed_loop_poles(A, B, C, D, K_mid))
            )
            roots[k + 2] = _matched(roots[k + 1], roots[k + 2])
        else:
            k += 1
    return gains, roots


def _gain_reaching(A, B, C, D, radius):
    """Gain at which the farthest closed-loop pole reaches ``radius``.

    Stops early when doubling the gain no longer moves the poles (every
    branch has arrived at a finite zero).
    """
    K = 1.0
    previous = closed_loop_poles(A, B, C, D, K)
    for _ in range(60):
        if np.max(np.abs(previous), initial=0.0) >= radius:
            return K
        current = _matched(previous, closed_loop_poles(A, B, C, D, 2.0 * K))
        if np.max(np.abs(current - previous), initial=0.0) < 1e-4 * radius:
            return 2.0 * K
        K, previous = 2.0 * K, current
    return K


def _matched(previous, current):
    """Reorder ``current`` so each entry continues the nearest ``previous`` branch."""
    from scipy.optimize import linear_sum_assignment

    cost = np.abs(previous[:, None] - current[None, :])
    rows, cols = linear_sum_assignment(cost)
    ordered = np.empty_like(current)
    ordered[rows] = current[cols]
    return ordered
