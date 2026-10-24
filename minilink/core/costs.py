"""Cost functions: the running cost ``g(x, u, t)`` and the terminal cost ``h(x, t)`` of ``J``."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module, require_jax
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.trajectory import Trajectory

###############################################################################
# Public API
###############################################################################


class CostFunction(ABC):
    """
    Mother class for deterministic planning cost functions.

    Subclasses define a running cost ``g(x, u, t)`` and a terminal cost
    ``h(x, t)`` of the objective ``J = ∫ exp(-rho t) g dt + h(x(tf), tf)``.
    The horizon is the planning problem's: a finite ``tf`` ends the integral
    and charges ``h`` there, an infinite one never does. Leaving the allowed
    set is priced by the problem (``infeasible_cost``), not by the cost.

    ``g`` and ``h`` are native-array equation paths: NumPy in, NumPy out;
    JAX in, JAX out. Reporting helpers such as :meth:`total_cost` convert to
    Python floats at the boundary. A cost lives in :mod:`minilink.core`, not
    on a :class:`~minilink.core.system.System`, so one model serves many
    planning problems.

    The discount rate ``rho`` is the problem's (``PlanningProblem.discount_rate``):
    the cost is the integrand only, and :meth:`integrate` takes the rate.
    """

    @abstractmethod
    def g(self, x, u, t=0.0, params=None):
        """Return the native scalar running cost density ``g(x, u, t)``."""
        ...

    @abstractmethod
    def h(self, x, t=0.0, params=None):
        """Return the native scalar terminal cost ``h(x, t)``."""
        ...

    def g_max(self, X, U, params=None):
        """
        An upper bound of ``g`` over the boxes ``X × U`` (two :class:`~minilink.core.sets.BoxSet`).

        The largest ``g`` on a quasi-random sample of the boxes at ``t = 0``: a
        sampled estimate, which a subclass replaces by its exact maximum.
        """
        xs, us = box_samples(X, U)
        g = self.g

        # g_max ≈ max over the samples of g(x, u, 0)
        values = [float(g(x, u, 0.0, params)) for x, u in zip(xs, us)]
        g_max = max(values)

        return g_max

    def h_max(self, X, params=None):
        """An upper bound of ``h`` over the box ``X``: the largest ``h`` on a quasi-random sample."""
        xs, _ = box_samples(X, None)
        h = self.h

        # h_max ≈ max over the samples of h(x, 0)
        values = [float(h(x, 0.0, params)) for x in xs]
        h_max = max(values)

        return h_max

    def integrate(self, traj: Trajectory, *, discount_rate=0.0, params=None):
        """
        The run's discounted running cost: ``(cost_rate, cost)``, both shape ``(N,)``.

        ``cost_rate`` is the integrand ``e^{−ρ t_k} g(x_k, u_k, t_k)`` at each
        sample and ``cost`` its integral ``∫_{t_0}^{t_k}`` by the trapezoid,
        ``cost[0] = 0``; the terminal cost ``h`` is not included. On the
        trajectory's array module, so it traces under JAX.
        """
        xp = array_module(traj.x)

        g = self.g
        rho = discount_rate
        t = traj.t
        x = traj.x
        u = traj.u

        # g_k = g(x_k, u_k, t_k), every sample (a vmap on JAX, a loop on NumPy)
        g_k = per_sample(g, x.T, u.T, t, params)

        # dJ_k = e^{−ρ t_k} g_k
        cost_rate = xp.exp(-rho * t) * g_k

        # J(t_k) = Σ_{i<k} ½ (dJ_i + dJ_{i+1}) (t_{i+1} − t_i), J(t_0) = 0
        increments = 0.5 * (cost_rate[:-1] + cost_rate[1:]) * (t[1:] - t[:-1])
        cost = xp.concatenate([xp.zeros(1), xp.cumsum(increments)])

        return cost_rate, cost

    def evaluate_trajectory(
        self,
        traj: Trajectory,
        params=None,
        *,
        discount_rate=0.0,
    ) -> Trajectory:
        """
        Return ``traj`` with sampled running and cumulative costs.

        The added signals are ``"cost_rate"`` and ``"cost"`` with shape
        ``(1, N)``. ``cost_rate`` is the discounted integrand
        ``exp(-rho t) g(x, u, t)``; the cumulative integral uses the
        trapezoidal rule and excludes the terminal cost; callers can add
        ``h(x(tf), tf)`` when they need the full objective scalar.
        """
        g = self.g
        rho = float(discount_rate)
        N = traj.n_samples

        # dJ_k = exp(−ρ t_k) g(x_k, u_k, t_k), each sample's discounted running cost
        dJ = np.zeros(N, dtype=float)
        for i, t in enumerate(traj.t):
            g_i = float(g(traj.x[:, i], traj.u[:, i], float(t), params=params))
            dJ[i] = np.exp(-rho * float(t)) * g_i

        # J(t_k) = ∫ dJ dt from t_0 to t_k by the trapezoidal rule, J(t_0) = 0
        J = np.zeros(N, dtype=float)
        if N > 1:
            dt = np.diff(traj.t)
            increments = 0.5 * (dJ[:-1] + dJ[1:]) * dt
            J[1:] = np.cumsum(increments)

        return traj.with_signals(
            {
                "cost_rate": dJ.reshape(1, -1),
                "cost": J.reshape(1, -1),
            }
        )

    def terminal_cost(
        self,
        traj: Trajectory,
        params=None,
    ) -> float:
        """Return ``h`` evaluated at the final state of ``traj``."""
        return float(self.h(traj.x[:, -1], traj.tf, params=params))

    def total_cost(
        self,
        traj: Trajectory,
        params=None,
        *,
        discount_rate=0.0,
    ) -> float:
        """Return the trapezoidal running cost plus terminal cost."""
        evaluated = self.evaluate_trajectory(
            traj, params=params, discount_rate=discount_rate
        )
        running = evaluated.signals["cost"][0, -1]
        terminal = self.terminal_cost(traj, params)

        # J = ∫ exp(−ρ t) g dt + h(x(t_f), t_f)
        J = running + terminal

        return float(J)

    # Cost algebra

    def __add__(self, other: "CostFunction") -> "CostFunction":
        """Return the sum cost ``self + other``."""
        return SumCost.of(self, other)

    def __radd__(self, other) -> "CostFunction":
        """Support ``sum([...])``, whose start value is the integer ``0``."""
        if other == 0:
            return self

        return SumCost.of(other, self)

    def __mul__(self, weight: float) -> "CostFunction":
        """Return this cost scaled by a weight, ``weight * self``."""
        return ScaledCost(self, float(weight))

    __rmul__ = __mul__

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


@dataclass(frozen=True)
class QuadraticCost(CostFunction):
    """
    Quadratic running and terminal cost.

    The running cost is
    ``(x - xbar).T @ Q @ (x - xbar) + (u - ubar).T @ R @ (u - ubar)``.
    The terminal cost is ``(x - xbar).T @ S @ (x - xbar)``.
    """

    Q: np.ndarray
    R: np.ndarray
    S: np.ndarray
    xbar: np.ndarray
    ubar: np.ndarray

    def __post_init__(self) -> None:
        # The weights and the targets as float copies, the targets flat
        Q = np.asarray(self.Q, dtype=float).copy()
        R = np.asarray(self.R, dtype=float).copy()
        S = np.asarray(self.S, dtype=float).copy()
        xbar = np.asarray(self.xbar, dtype=float).reshape(-1).copy()
        ubar = np.asarray(self.ubar, dtype=float).reshape(-1).copy()

        # The weights sized by the targets: Q and S are n × n, R is m × m
        n = xbar.size
        m = ubar.size
        check_weights(Q, R, S, n, m)

        object.__setattr__(self, "Q", Q)
        object.__setattr__(self, "R", R)
        object.__setattr__(self, "S", S)
        object.__setattr__(self, "xbar", xbar)
        object.__setattr__(self, "ubar", ubar)

    @classmethod
    def from_system(
        cls,
        sys,
        *,
        Q: np.ndarray | None = None,
        R: np.ndarray | None = None,
        S: np.ndarray | None = None,
        xbar: np.ndarray | None = None,
        ubar: np.ndarray | None = None,
    ):
        """
        Create a default quadratic cost from a Minilink system.

        Parameters not provided default to identity running weights, zero
        terminal weight, and the system's operating point as the target: its
        nominal state and nominal input.
        """
        n = int(sys.n)
        m = int(sys.m)

        # The system's operating point as the default target
        if xbar is None:
            xbar = sys.state.nominal_value
        if ubar is None:
            ubar = sys.get_u_from_input_ports()

        return cls(
            Q=np.eye(n) if Q is None else Q,
            R=np.eye(m) if R is None else R,
            S=np.zeros((n, n)) if S is None else S,
            xbar=xbar,
            ubar=ubar,
        )

    def g(self, x, u, t=0.0, params=None):
        """Return the quadratic running cost."""
        Q = self.Q
        R = self.R
        xbar = self.xbar
        ubar = self.ubar

        dx = x - xbar
        du = u - ubar

        # g = (x − x̄)ᵀ Q (x − x̄) + (u − ū)ᵀ R (u − ū)
        g = dx.T @ Q @ dx + du.T @ R @ du

        return g

    def h(self, x, t=0.0, params=None):
        """Return the quadratic terminal cost."""
        S = self.S
        xbar = self.xbar

        dx = x - xbar

        # h = (x − x̄)ᵀ S (x − x̄)
        h = dx.T @ S @ dx

        return h

    def g_max(self, X, U, params=None):
        """The exact maximum of ``g`` over the boxes: each quadratic at its farthest vertex."""
        # max g = max (x − x̄)ᵀ Q (x − x̄) + max (u − ū)ᵀ R (u − ū), the two independent
        g_max = quadratic_max(self.Q, self.xbar, X) + quadratic_max(
            self.R, self.ubar, U
        )

        return g_max

    def h_max(self, X, params=None):
        """The exact maximum of ``h`` over the box ``X``."""
        return quadratic_max(self.S, self.xbar, X)


@dataclass(frozen=True)
class TimeCost(CostFunction):
    """
    Minimum-time cost ``J = T``.

    The running cost is ``g = 1`` everywhere except within ``eps`` of the target
    ``xbar``, where it is ``0`` (the absorbing goal); the terminal cost is ``0``.
    Under dynamic programming the cost-to-go becomes the time-to-go and the
    optimal policy is bang-bang.
    """

    xbar: np.ndarray
    eps: float = 1e-3

    def __post_init__(self) -> None:
        xbar = np.asarray(self.xbar, dtype=float).reshape(-1).copy()
        object.__setattr__(self, "xbar", xbar)
        object.__setattr__(self, "eps", float(self.eps))

    @classmethod
    def from_system(cls, sys, *, xbar: np.ndarray | None = None, eps: float = 1e-3):
        """Create a time cost targeting ``xbar`` (default the system's nominal state)."""
        xbar = sys.state.nominal_value if xbar is None else xbar

        return cls(xbar=xbar, eps=eps)

    def g(self, x, u, t=0.0, params=None):
        """Return unit running cost away from the target, zero on it."""
        xp = array_module(x)

        xbar = xp.asarray(self.xbar)
        eps = self.eps

        on_target = xp.linalg.norm(x - xbar) < eps

        # g = 0 on the target (‖x − x̄‖ < ε), 1 elsewhere
        g = xp.where(on_target, xp.asarray(0.0), xp.asarray(1.0))

        return g

    def h(self, x, t=0.0, params=None):
        """Return zero terminal cost."""
        return 0.0

    def g_max(self, X, U, params=None):
        """``g`` is at most 1."""
        return 1.0

    def h_max(self, X, params=None):
        """``h`` is 0."""
        return 0.0


@dataclass(frozen=True)
class SumCost(CostFunction):
    """
    Additive cost ``J = sum_i J_i`` over several cost functions.

    Built by the ``+`` operator on :class:`CostFunction`; use it to add an
    obstacle or traversability term to a base objective.
    """

    terms: tuple

    @classmethod
    def of(cls, *costs: CostFunction) -> "SumCost":
        """Build a sum cost, flattening nested sums into a single term list."""
        terms: list[CostFunction] = []
        for cost in costs:
            if isinstance(cost, SumCost):
                terms.extend(cost.terms)
            else:
                terms.append(cost)

        return cls(tuple(terms))

    def g(self, x, u, t=0.0, params=None):
        """Return the summed running cost."""
        terms = self.terms

        # g = Σᵢ gᵢ(x, u, t)
        g = sum(cost.g(x, u, t, params) for cost in terms)

        return g

    def h(self, x, t=0.0, params=None):
        """Return the summed terminal cost."""
        terms = self.terms

        # h = Σᵢ hᵢ(x, t)
        h = sum(cost.h(x, t, params) for cost in terms)

        return h

    def g_max(self, X, U, params=None):
        """The sum of the terms' bounds: ``max Σ gᵢ ≤ Σ max gᵢ``."""
        return sum(cost.g_max(X, U, params) for cost in self.terms)

    def h_max(self, X, params=None):
        """The sum of the terms' bounds."""
        return sum(cost.h_max(X, params) for cost in self.terms)


@dataclass(frozen=True)
class ScaledCost(CostFunction):
    """
    Cost scaled by a weight, ``J = weight * J0``.

    Built by the ``*`` operator on :class:`CostFunction`, so a weighted sum
    reads as ``base + weight * obstacle_cost``.
    """

    cost: CostFunction
    weight: float

    def g(self, x, u, t=0.0, params=None):
        """Return the weighted running cost."""
        cost = self.cost
        weight = self.weight

        # g = w g₀(x, u, t)
        g = weight * cost.g(x, u, t, params)

        return g

    def h(self, x, t=0.0, params=None):
        """Return the weighted terminal cost."""
        cost = self.cost
        weight = self.weight

        # h = w h₀(x, t)
        h = weight * cost.h(x, t, params)

        return h

    def g_max(self, X, U, params=None):
        """``w · max g₀`` for a nonnegative weight; sampled for a negative one."""
        if self.weight < 0.0:
            return super().g_max(X, U, params)

        return self.weight * self.cost.g_max(X, U, params)

    def h_max(self, X, params=None):
        """``w · max h₀`` for a nonnegative weight; sampled for a negative one."""
        if self.weight < 0.0:
            return super().h_max(X, params)

        return self.weight * self.cost.h_max(X, params)


###############################################################################
# Internal machinery
###############################################################################


def check_weights(Q, R, S, n, m):
    """Refuse weights the targets do not size: ``Q``, ``S`` ``(n, n)`` and ``R`` ``(m, m)``."""
    if Q.shape != (n, n):
        raise ValueError("Q must have shape (n, n)")
    if S.shape != (n, n):
        raise ValueError("S must have shape (n, n)")
    if R.shape != (m, m):
        raise ValueError("R must have shape (m, m)")


def quadratic_max(W, center, box):
    """
    ``max (z − c)ᵀ W (z − c)`` over a box, for ``W ⪰ 0``: at a vertex (a convex function).

    Exact for a diagonal ``W`` and for up to 12 coupled entries (every vertex);
    beyond, the bound ``λ_max(W) Σ max(dᵢ²)``. Entries ``W`` does not weigh may be
    unbounded; a weighed unbounded entry gives ``+inf``.
    """
    W = np.asarray(W, dtype=float)

    # The entries W weighs, and their farthest offsets from the center
    weighed = np.flatnonzero(np.any(W != 0.0, axis=0) | np.any(W != 0.0, axis=1))
    if weighed.size == 0:
        return 0.0
    lo = box.lower[weighed] - center[weighed]
    hi = box.upper[weighed] - center[weighed]
    W = W[np.ix_(weighed, weighed)]
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))):
        return float(np.inf)

    # Diagonal: Σ wᵢᵢ max(loᵢ², hiᵢ²)
    if np.all(W == np.diag(np.diag(W))):
        return float(np.sum(np.diag(W) * np.maximum(lo**2, hi**2)))

    # Coupled, small: the largest value at the 2^k vertices
    if weighed.size <= 12:
        vertices = np.array(np.meshgrid(*zip(lo, hi), indexing="ij")).reshape(
            weighed.size, -1
        )
        return float(np.max(np.einsum("ik,ij,jk->k", vertices, W, vertices)))

    # Coupled, large: λ_max(W) ‖d‖², d the farthest offset of each entry
    lam_max = float(np.max(np.linalg.eigvalsh(0.5 * (W + W.T))))

    return lam_max * float(np.sum(np.maximum(lo**2, hi**2)))


def box_samples(X, U, n_samples=4096):
    """A Halton set of ``(x, u)`` over the finite boxes ``X × U`` (``U=None``: ``x`` only)."""
    # Lazy import: SciPy's quasi-Monte Carlo loads with the first sampled bound
    from scipy.stats import qmc

    boxes = [X] if U is None else [X, U]
    lower = np.concatenate([box.lower for box in boxes])
    upper = np.concatenate([box.upper for box in boxes])
    require_finite_bounds(lower, upper)

    # The Halton points mapped on the box, the origin of the sequence skipped
    unit = qmc.Halton(d=lower.size, scramble=False).random(int(n_samples) + 1)[1:]
    points = lower + unit * (upper - lower)
    n = X.dim

    return points[:, :n], (None if U is None else points[:, n:])


def require_finite_bounds(lower, upper):
    """Refuse a sampled bound over an unbounded box."""
    if not (np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))):
        raise ValueError(
            "a sampled bound of the cost needs finite boxes: declare X and U on the "
            "problem (X=plant.state.box, U=plant.input_box), or give the problem "
            "infeasible_cost as a number"
        )


def per_sample(g, xs, us, ts, params):
    """``g(x_k, u_k, t_k, params)`` for every sample, as an ``(N,)`` array (vmap on JAX)."""
    xp = array_module(xs)
    if xp is np:
        return xp.array(
            [float(g(x, u, t, params=params)) for x, u, t in zip(xs, us, ts)]
        )

    jax = require_jax()

    return jax.vmap(lambda x, u, t: g(x, u, t, params=params))(xs, us, ts)
