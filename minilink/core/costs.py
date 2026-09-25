"""Cost functions: the running cost ``g(x, u, t)`` and the terminal cost ``h(x, t)`` of ``J``."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.trajectory import Trajectory


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

    Class attribute (override it in the body of a subclass):

    - ``discount_rate``: continuous rate ``rho >= 0``; ``0`` is undiscounted.
      Planners convert it with :meth:`discount_factor` (value iteration's
      ``alpha`` and reinforcement learning's ``gamma`` are both ``exp(-rho dt)``).
      The library costs are frozen dataclasses, so assigning it on one of their
      instances raises. ``weight * cost`` keeps the cost's rate and ``a + b``
      requires equal rates. Setting it per instance (a constructor field or a
      ``params`` entry) is workboard step A2 in docs/plans/TODO.md.
    """

    discount_rate = 0.0

    @abstractmethod
    def g(self, x, u, t=0.0, params=None):
        """Return the native scalar running cost density ``g(x, u, t)``."""
        ...

    @abstractmethod
    def h(self, x, t=0.0, params=None):
        """Return the native scalar terminal cost ``h(x, t)``."""
        ...

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    def discount_factor(self, dt) -> float:
        """Per-step factor ``exp(-rho dt)`` for a planner with time step ``dt``."""
        return float(np.exp(-float(self.discount_rate) * float(dt)))

    def evaluate_trajectory(
        self,
        traj: Trajectory,
        params=None,
    ) -> Trajectory:
        """
        Return ``traj`` with sampled running and cumulative costs.

        The added signals are ``"cost_rate"`` and ``"cost"`` with shape
        ``(1, N)``. ``cost_rate`` is the discounted integrand
        ``exp(-rho t) g(x, u, t)``; the cumulative integral uses the
        trapezoidal rule and excludes the terminal cost; callers can add
        ``h(x(tf), tf)`` when they need the full objective scalar.
        """
        dJ = np.zeros(traj.n_samples, dtype=float)
        rho = float(self.discount_rate)
        for i, t in enumerate(traj.t):
            g = float(self.g(traj.x[:, i], traj.u[:, i], float(t), params=params))
            dJ[i] = np.exp(-rho * float(t)) * g

        J = np.zeros(traj.n_samples, dtype=float)
        if traj.n_samples > 1:
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
    ) -> float:
        """Return the trapezoidal running cost plus terminal cost."""
        evaluated = self.evaluate_trajectory(traj, params=params)
        return float(
            evaluated.signals["cost"][0, -1] + self.terminal_cost(traj, params)
        )

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
        Q = np.asarray(self.Q, dtype=float).copy()
        R = np.asarray(self.R, dtype=float).copy()
        S = np.asarray(self.S, dtype=float).copy()
        xbar = np.asarray(self.xbar, dtype=float).reshape(-1).copy()
        ubar = np.asarray(self.ubar, dtype=float).reshape(-1).copy()

        n = xbar.size
        m = ubar.size
        if Q.shape != (n, n):
            raise ValueError("Q must have shape (n, n)")
        if S.shape != (n, n):
            raise ValueError("S must have shape (n, n)")
        if R.shape != (m, m):
            raise ValueError("R must have shape (m, m)")

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
        terminal weight, zero state target, and the system's nominal input.
        """
        n = int(sys.n)
        m = int(sys.m)
        if xbar is None:
            xbar = np.zeros(n)
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
        Q, R = self.Q, self.R
        xbar, ubar = self.xbar, self.ubar
        dx = x - xbar
        du = u - ubar

        g = dx.T @ Q @ dx + du.T @ R @ du

        return g

    def h(self, x, t=0.0, params=None):
        """Return the quadratic terminal cost."""
        S, xbar = self.S, self.xbar
        dx = x - xbar

        h = dx.T @ S @ dx

        return h


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
        """Create a time cost targeting ``xbar`` (default the state origin)."""
        xbar = np.zeros(int(sys.n)) if xbar is None else xbar
        return cls(xbar=xbar, eps=eps)

    def g(self, x, u, t=0.0, params=None):
        """Return unit running cost away from the target, zero on it."""
        xp = array_module(x)
        xbar = xp.asarray(self.xbar)
        eps = self.eps
        on_target = xp.linalg.norm(x - xbar) < eps

        # g = 0 on the target, 1 elsewhere
        g = xp.where(on_target, xp.asarray(0.0), xp.asarray(1.0))

        return g

    def h(self, x, t=0.0, params=None):
        """Return zero terminal cost."""
        return 0.0


@dataclass(frozen=True)
class SumCost(CostFunction):
    """
    Additive cost ``J = sum_i J_i`` over several cost functions.

    Built by the ``+`` operator on :class:`CostFunction`; use it to add an
    obstacle or traversability term to a base objective. The terms share one
    ``discount_rate``; terms whose rates differ raise ``ValueError``.
    """

    terms: tuple

    def __post_init__(self) -> None:
        rates = sorted({float(cost.discount_rate) for cost in self.terms})
        if len(rates) > 1:
            raise ValueError(
                f"SumCost terms have different discount rates {rates}; "
                "one objective has one rate rho, so give every term the same "
                "discount_rate"
            )

    @property
    def discount_rate(self) -> float:
        """The terms' common rate ``rho`` (``0`` for an empty sum)."""
        terms = self.terms
        return terms[0].discount_rate if terms else 0.0

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

        g = sum(cost.g(x, u, t, params) for cost in terms)

        return g

    def h(self, x, t=0.0, params=None):
        """Return the summed terminal cost."""
        terms = self.terms

        h = sum(cost.h(x, t, params) for cost in terms)

        return h


@dataclass(frozen=True)
class ScaledCost(CostFunction):
    """
    Cost scaled by a weight, ``J = weight * J0``.

    Built by the ``*`` operator on :class:`CostFunction`, so a weighted sum
    reads as ``base + weight * obstacle_cost``. The weight leaves the
    ``discount_rate`` of ``J0`` unchanged.
    """

    cost: CostFunction
    weight: float

    @property
    def discount_rate(self) -> float:
        """The scaled cost's rate ``rho``."""
        return self.cost.discount_rate

    def g(self, x, u, t=0.0, params=None):
        """Return the weighted running cost."""
        cost, weight = self.cost, self.weight

        g = weight * cost.g(x, u, t, params)

        return g

    def h(self, x, t=0.0, params=None):
        """Return the weighted terminal cost."""
        cost, weight = self.cost, self.weight

        h = weight * cost.h(x, t, params)

        return h
