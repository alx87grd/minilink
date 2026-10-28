"""Cost functions: the running cost ``g(x, u, t)`` and the terminal cost ``h(x, t)`` of ``J``."""

from abc import ABC, abstractmethod

import numpy as np

from minilink.core.backends import array_module
from minilink.core.inspect import inspect_text, latex_inspect, repr_pretty
from minilink.core.trajectory import Trajectory


def param_view(key):
    """An attribute that reads and writes ``self.params[key]``: a view, so in-place writes reach the cost."""

    def read(self):
        return self.params[key]

    def write(self, value):
        self.params[key] = value

    return property(read, write, doc=f"View of ``params[{key!r}]``.")


###############################################################################
# Public API
###############################################################################


class CostFunction(ABC):
    """
    Mother class for deterministic planning cost functions.

    Subclasses define a running cost ``g(x, u, t)`` and a terminal cost
    ``h(x, t)`` of the objective ``J = ∫ e^{−ρt} g dt + e^{−ρ tf} h(x(tf), tf)``.
    The horizon is the planning problem's: a finite ``tf`` ends the integral
    and charges ``h`` there, an infinite one never does. Leaving the allowed
    set is priced by the problem (``infeasible_cost``), not by the cost.

    ``g`` and ``h`` are native-array equation paths: NumPy in, NumPy out;
    JAX in, JAX out. :meth:`integrate` gives the running cost along a
    trajectory on its array module. A cost lives in :mod:`minilink.core`, not
    on a :class:`~minilink.core.system.System`, so one model serves many
    planning problems.

    The discount rate ``rho`` is the problem's (``PlanningProblem.discount_rate``):
    the cost is the integrand only, and :meth:`integrate` takes the rate.

    Class attributes: ``params``, the cost's parameters (``{}`` for a cost
    that hard-codes its constants; the library costs hold a dictionary, the
    System pattern: ``params=None`` in ``g`` and ``h`` reads them), and ``id``,
    the cost's key in a sum (``None``: its class name).
    """

    params = {}
    id = None

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
        The largest ``g`` over the boxes ``X × U`` (two :class:`~minilink.core.sets.BoxSet`).

        The largest ``g`` on a quasi-random sample of the boxes at ``t = 0``: a
        sampled estimate, which may fall short of the maximum between the samples;
        a subclass replaces it by its exact maximum or a bound.
        """
        xs, us = box_samples(X, U)
        g = self.g

        # g_max ≈ max over the samples of g(x, u, 0)
        values = [float(g(x, u, 0.0, params)) for x, u in zip(xs, us)]
        g_max = max(values)

        return g_max

    def h_max(self, X, params=None):
        """The largest ``h`` over the box ``X``: a sampled estimate, as :meth:`g_max`."""
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


class QuadraticCost(CostFunction):
    """
    Quadratic running and terminal cost.

    The running cost is
    ``(x - xbar).T @ Q @ (x - xbar) + (u - ubar).T @ R @ (u - ubar)``.
    The terminal cost is ``(x - xbar).T @ S @ (x - xbar)``.

    The weights and the targets are :attr:`params` (``"Q"``, ``"R"``, ``"S"``,
    ``"xbar"``, ``"ubar"``), the System pattern: ``params=None`` in :meth:`g`
    and :meth:`h` reads them, a dictionary replaces them, so ``jax.grad``
    reaches ``Q``. The attributes ``cost.Q`` … are views of the dictionary.
    """

    def __init__(self, Q, R, S, xbar, ubar):
        # The weights and the targets as float copies, the targets flat
        Q = np.asarray(Q, dtype=float).copy()
        R = np.asarray(R, dtype=float).copy()
        S = np.asarray(S, dtype=float).copy()
        xbar = np.asarray(xbar, dtype=float).reshape(-1).copy()
        ubar = np.asarray(ubar, dtype=float).reshape(-1).copy()

        # The weights sized by the targets: Q and S are n × n, R is m × m
        check_weights(Q, R, S, xbar.size, ubar.size)

        self.params = {"Q": Q, "R": R, "S": S, "xbar": xbar, "ubar": ubar}

    Q = param_view("Q")
    R = param_view("R")
    S = param_view("S")
    xbar = param_view("xbar")
    ubar = param_view("ubar")

    def _repr_latex_(self):
        return latex_inspect(self)

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
        params = self.params if params is None else params
        Q = params["Q"]
        R = params["R"]
        xbar = params["xbar"]
        ubar = params["ubar"]

        dx = x - xbar
        du = u - ubar

        # g = (x − x̄)ᵀ Q (x − x̄) + (u − ū)ᵀ R (u − ū)
        g = dx.T @ Q @ dx + du.T @ R @ du

        return g

    def h(self, x, t=0.0, params=None):
        """Return the quadratic terminal cost."""
        params = self.params if params is None else params
        S = params["S"]
        xbar = params["xbar"]

        dx = x - xbar

        # h = (x − x̄)ᵀ S (x − x̄)
        h = dx.T @ S @ dx

        return h

    def g_max(self, X, U, params=None):
        """The exact maximum of ``g`` over the boxes: each quadratic at its farthest vertex."""
        params = self.params if params is None else params
        Q = params["Q"]
        R = params["R"]
        xbar = params["xbar"]
        ubar = params["ubar"]

        # max g = max (x − x̄)ᵀ Q (x − x̄) + max (u − ū)ᵀ R (u − ū), the two independent
        g_max = quadratic_max(Q, xbar, X) + quadratic_max(R, ubar, U)

        return g_max

    def h_max(self, X, params=None):
        """The exact maximum of ``h`` over the box ``X``."""
        params = self.params if params is None else params

        return quadratic_max(params["S"], params["xbar"], X)


class TimeCost(CostFunction):
    """
    Minimum-time cost ``J = T``.

    The running cost is ``g = 1`` everywhere except within ``eps`` of the target
    ``xbar``, where it is ``0`` (the absorbing goal); the terminal cost is ``0``.
    Under dynamic programming the cost-to-go becomes the time-to-go and the
    optimal policy is bang-bang. The target is :attr:`params` (``"xbar"``);
    ``eps``, a threshold with no useful gradient, stays an attribute.
    """

    def __init__(self, xbar, eps=1e-3):
        self.params = {"xbar": np.asarray(xbar, dtype=float).reshape(-1).copy()}
        self.eps = float(eps)

    xbar = param_view("xbar")

    @classmethod
    def from_system(cls, sys, *, xbar: np.ndarray | None = None, eps: float = 1e-3):
        """Create a time cost targeting ``xbar`` (default the system's nominal state)."""
        xbar = sys.state.nominal_value if xbar is None else xbar

        return cls(xbar=xbar, eps=eps)

    def g(self, x, u, t=0.0, params=None):
        """Return unit running cost away from the target, zero on it."""
        xp = array_module(x)
        params = self.params if params is None else params

        xbar = xp.asarray(params["xbar"])
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


class SumCost(CostFunction):
    """
    Additive cost ``J = sum_i J_i`` over several cost functions.

    Built by the ``+`` operator on :class:`CostFunction`; use it to add an
    obstacle or traversability term to a base objective. The terms are keyed
    by id, as a diagram keys its blocks (:meth:`of`): ``terms`` is
    ``{term_id: cost}`` and :attr:`params` the live view
    ``{term_id: cost.params}``, its setter routing each entry to its term, so
    a partial dictionary changes only the terms it names. A sum holds each cost
    object once: ``q + 10 * q`` is refused, ``11 * q`` is the same cost.
    """

    def __init__(self, terms):
        self.terms = dict(terms)
        require_distinct_terms(self.terms)

    @classmethod
    def of(cls, *costs: CostFunction, **named: CostFunction) -> "SumCost":
        """
        Build a sum cost: anonymous terms keyed by their id, named ones by their name.

        An anonymous sum (``a + b + c``, ``sum([...])``) is flattened into one level,
        so the dictionary does not depend on the order Python adds; a term's id is
        its ``id``, else its class name verbatim, with ``2``, ``3``, … for repeats
        (a scaled cost takes the id of the cost it scales). A sum with an ``id``,
        and every term passed by name, stays nested.
        """
        terms = {}
        for cost in costs:
            if isinstance(cost, SumCost) and cost.id is None:
                for term_id, term in cost.terms.items():
                    base = term_id if is_named_term(term_id, term) else term_id_of(term)
                    terms[unique_term_id(base, terms)] = term
            else:
                terms[unique_term_id(term_id_of(cost), terms)] = cost
        for name, cost in named.items():
            require_new_term(name, terms)
            terms[name] = cost

        return cls(terms)

    @property
    def params(self):
        """Nested live view ``{term_id: cost.params}``."""
        return {term_id: cost.params for term_id, cost in self.terms.items()}

    @params.setter
    def params(self, value):
        validate_cost_params(value, self.terms)
        for term_id, term_params in (value or {}).items():
            self.terms[term_id].params = term_params

    def g(self, x, u, t=0.0, params=None):
        """Return the summed running cost, each term on its own entry of ``params``."""
        validate_cost_params(params, self.terms)
        params = params or {}

        # g = Σᵢ gᵢ(x, u, t)
        g = sum(
            cost.g(x, u, t, params.get(term_id)) for term_id, cost in self.terms.items()
        )

        return g

    def h(self, x, t=0.0, params=None):
        """Return the summed terminal cost, each term on its own entry of ``params``."""
        validate_cost_params(params, self.terms)
        params = params or {}

        # h = Σᵢ hᵢ(x, t)
        h = sum(
            cost.h(x, t, params.get(term_id)) for term_id, cost in self.terms.items()
        )

        return h

    def g_max(self, X, U, params=None):
        """The sum of the terms' bounds: ``max Σ gᵢ ≤ Σ max gᵢ``."""
        params = params or {}

        return sum(
            cost.g_max(X, U, params.get(term_id))
            for term_id, cost in self.terms.items()
        )

    def h_max(self, X, params=None):
        """The sum of the terms' bounds."""
        params = params or {}

        return sum(
            cost.h_max(X, params.get(term_id)) for term_id, cost in self.terms.items()
        )


class ScaledCost(CostFunction):
    """
    Cost scaled by a weight, ``J = weight * J0``.

    Built by the ``*`` operator on :class:`CostFunction`, so a weighted sum
    reads as ``base + weight * obstacle_cost``. Its :attr:`params` nest the
    scaled cost's, ``{"weight": w, "cost": cost.params}``; in a sum it takes
    the id of the cost it scales.
    """

    def __init__(self, cost, weight):
        self.cost = cost
        self.weight = float(weight)

    @property
    def params(self):
        """Live view ``{"weight": w, "cost": cost.params}``."""
        return {"weight": self.weight, "cost": self.cost.params}

    @params.setter
    def params(self, value):
        require_known_keys(value, ("weight", "cost"), "ScaledCost")
        if value is None:
            return
        if "weight" in value:
            self.weight = value["weight"]
        if "cost" in value:
            self.cost.params = value["cost"]

    def g(self, x, u, t=0.0, params=None):
        """Return the weighted running cost."""
        weight, inner = self.split(params)
        cost = self.cost

        # g = w g₀(x, u, t)
        g = weight * cost.g(x, u, t, inner)

        return g

    def h(self, x, t=0.0, params=None):
        """Return the weighted terminal cost."""
        weight, inner = self.split(params)
        cost = self.cost

        # h = w h₀(x, t)
        h = weight * cost.h(x, t, inner)

        return h

    def g_max(self, X, U, params=None):
        """``w · max g₀`` for a nonnegative weight; sampled for a negative one."""
        weight, inner = self.split(params)
        if weight < 0.0:
            return super().g_max(X, U, params)

        return weight * self.cost.g_max(X, U, inner)

    def h_max(self, X, params=None):
        """``w · max h₀`` for a nonnegative weight; sampled for a negative one."""
        weight, inner = self.split(params)
        if weight < 0.0:
            return super().h_max(X, params)

        return weight * self.cost.h_max(X, inner)

    def split(self, params):
        """``(weight, the scaled cost's params)``: the entries given, else the cost's own."""
        require_known_keys(params, ("weight", "cost"), "ScaledCost")
        params = params or {}

        return params.get("weight", self.weight), params.get("cost")


###############################################################################
# Internal machinery
###############################################################################


def term_id_of(cost) -> str:
    """A term's id in a sum: its ``id``, else (a scaled cost) its scaled cost's, else its class name verbatim."""
    if cost.id is not None:
        return str(cost.id)
    if isinstance(cost, ScaledCost):
        return term_id_of(cost.cost)

    return type(cost).__name__


def require_distinct_terms(terms):
    """Refuse a sum holding one cost object twice: its parameters would sit twice in the tree."""
    seen = set()
    for cost in leaf_costs_of(terms.values()):
        if id(cost) in seen:
            name = term_id_of(cost)
            raise ValueError(
                f"the sum holds the cost {name!r} twice, so its parameters would sit "
                "twice in the parameter tree; write one weighted term (11 * q for "
                "q + 10 * q, 2 * cost for cost + cost), or add a copy "
                "(copy.deepcopy(cost))"
            )
        seen.add(id(cost))


def leaf_costs_of(costs):
    """The cost objects under ``costs``, through nested sums and scalings, in order."""
    for cost in costs:
        if isinstance(cost, SumCost):
            yield from leaf_costs_of(cost.terms.values())
        elif isinstance(cost, ScaledCost):
            yield from leaf_costs_of([cost.cost])
        else:
            yield cost


def is_named_term(term_id, term) -> bool:
    """Whether a sum's key was given by name: not the term's own id, nor that id with a repeat count."""
    base = term_id_of(term)
    count = term_id[len(base) :]

    return not (term_id.startswith(base) and (count == "" or count.isdigit()))


def unique_term_id(base, existing) -> str:
    """``base``, else the first of ``base2``, ``base3``, … not in ``existing``."""
    if base not in existing:
        return base

    i = 2
    while f"{base}{i}" in existing:
        i += 1

    return f"{base}{i}"


def require_new_term(name, terms):
    """Refuse a named term whose name another term already holds."""
    if name in terms:
        raise ValueError(f"the sum already has a term {name!r}; name the terms apart")


def validate_cost_params(params, terms):
    """Refuse a sum's params with an id that names no term."""
    if params is None:
        return
    unknown = sorted(set(params) - set(terms))
    if unknown:
        raise ValueError(
            f"unknown cost term id(s) {unknown}; the sum's terms are {list(terms)}"
        )


def require_known_keys(params, keys, owner):
    """Refuse a params dictionary with a key the owner does not have."""
    if params is None:
        return
    unknown = sorted(set(params) - set(keys))
    if unknown:
        raise ValueError(f"{owner} params take {list(keys)}, got {unknown}")


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
    beyond, the bound ``λ_max(W) Σ max(dᵢ²)``. An indefinite ``W`` takes the bound
    ``dᵀ |W| d``. Entries ``W`` does not weigh may be unbounded; a weighed
    unbounded entry gives ``+inf``.
    """
    W = np.asarray(W, dtype=float)
    center = np.asarray(center, dtype=float)

    # The entries W weighs, and their farthest offsets from the center
    weighed = np.flatnonzero(np.any(W != 0.0, axis=0) | np.any(W != 0.0, axis=1))
    if weighed.size == 0:
        return 0.0
    lo = box.lower[weighed] - center[weighed]
    hi = box.upper[weighed] - center[weighed]
    W = W[np.ix_(weighed, weighed)]
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))):
        return float(np.inf)

    # Indefinite: |zᵢ − cᵢ| ≤ dᵢ bounds every term, (z − c)ᵀ W (z − c) ≤ dᵀ |W| d
    eigenvalues = np.linalg.eigvalsh(0.5 * (W + W.T))
    if eigenvalues[0] < -1e-12 * max(1.0, abs(eigenvalues[-1])):
        d = np.maximum(np.abs(lo), np.abs(hi))
        bound = d @ np.abs(W) @ d
        return float(bound)

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
    lam_max = float(eigenvalues[-1])

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

    # The float width is the caller's trace: no change to the JAX config here
    g_vectorized = xp.vectorize(
        lambda x, u, t: g(x, u, t, params=params), signature="(n),(m),()->()"
    )

    return g_vectorized(xs, us, ts)
