"""The scores of a run: the problem's price, its margins, and the Evaluation record of a batch."""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from minilink.core.backends import (
    array_module,
    ensure_jax_x64,
)
from minilink.core.trajectory import Trajectory

PROBLEM_PRICE = "problem"  # score_run's price read from the problem


# The run's discounted running cost, integrated with the state: the evaluator's recorded signal
RUNNING_COST = "plant:running_cost"

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class Evaluation:
    """
    The scores of the runs of one evaluation: one row per run, and their statistics.

    A pytree record (traceable inside a trace, plain arrays outside), each field
    of shape ``(n_runs,)``:

    - ``cost``: ``∫₀^{tf} e^{−ρt} g dt + e^{−ρ tf} h(x(tf))``, ``+inf`` for a run
      that is not finite;
    - ``J``: the cost, plus the problem's price when the run failed or missed the
      goal (with the price switch on);
    - ``feasibility_margin``: the least margin of ``x`` to ``X`` and of ``u`` to
      ``U`` along the run, and ``failed`` (not finite, or a negative margin);
    - ``reachability_margin``: the margin of ``x(tf)`` to ``Xf`` (``+inf`` without
      ``Xf``), and ``reached``;
    - ``x0``: the starts, ``(n_runs, n)``.

    :attr:`objective` is the problem's criterion over ``J``: the one number a
    program minimizes.
    """

    cost: object
    J: object
    feasibility_margin: object
    failed: object
    reachability_margin: object
    reached: object
    x0: object
    trajectories: list | None = None
    criterion: object = "expectation"

    @property
    def n_trials(self) -> int:
        """Number of runs."""
        return int(np.shape(self.J)[0])

    @property
    def objective(self):
        """The criterion over the runs' ``J``: the mean, a CVaR or the worst."""
        return reduce_criterion(self.J, self.criterion)

    @property
    def mean(self):
        """Mean of ``J`` over the runs."""
        return array_module(self.J).mean(self.J)

    @property
    def std(self):
        """Standard deviation of ``J`` over the runs."""
        return array_module(self.J).std(self.J)

    @property
    def worst(self):
        """Largest ``J`` over the runs."""
        return array_module(self.J).max(self.J)

    def cvar(self, alpha):
        """Mean of the worst ``ceil(alpha · n)`` values of ``J``."""
        return reduce_criterion(self.J, ("cvar", alpha))

    @property
    def feasibility_rate(self):
        """Fraction of the runs that stayed finite, in ``X`` and in ``U``."""
        return array_module(self.J).mean(~self.failed)

    @property
    def reachability_rate(self):
        """Fraction of the runs that ended in ``Xf``."""
        return array_module(self.J).mean(self.reached)

    @property
    def success_rate(self):
        """Fraction of the runs that stayed feasible and ended in ``Xf``."""
        return array_module(self.J).mean(self.reached & ~self.failed)

    # Display

    def __str__(self) -> str:
        if self.n_trials == 1:
            notes = []
            if bool(self.failed[0]):
                notes.append("failed")
            if not bool(self.reached[0]):
                notes.append("goal missed")
            note = f" ({', '.join(notes)})" if notes else ""
            return (
                f"J = {float(self.J[0]):.4g}{note}, cost = {float(self.cost[0]):.4g}, "
                f"feasibility margin = {float(self.feasibility_margin[0]):.3g}"
            )

        return (
            f"J over {self.n_trials} runs: objective {float(self.objective):.4g}, "
            f"mean {float(self.mean):.4g}, std {float(self.std):.4g}, "
            f"worst {float(self.worst):.4g}; feasible {100 * float(self.feasibility_rate):.0f}%, "
            f"reached {100 * float(self.reachability_rate):.0f}%"
        )


def score_run(
    problem,
    traj: Trajectory,
    *,
    tf=None,
    params=None,
    price=PROBLEM_PRICE,
    cost_bound=None,
    running_cost=None,
) -> dict:
    """
    The scores of one plant run under the problem: the formula every evaluation reports.

    ``traj`` is the plant's run (its state and its input on a time grid). On the
    trajectory's array module, so it traces under JAX::

        finite              = every x_k and u_k finite
        feasibility_margin  = min over the run of X.margin(x_k, t_k), U.margin(u_k, x_k, t_k)
        failed              = not finite, or feasibility_margin < 0
        reachability_margin = Xf.margin(x(t_end), t_end)   reached = margin ≥ 0  (no Xf: reached)
        cost                = ∫ e^{−ρ(t − t_start)} g dt + e^{−ρ tf} h(x(t_end), t_end)
                              (+inf when not finite), t_end = t_start + tf
        J                   = cost, or with the price on
                              cost + price · (failed or not reached)   finite
                              cost_bound + price                       not finite

    A run that is not finite is scored on zeros (a safe ``where``), and with the
    price on its cost term is the problem's cost bound, the largest cost a finite
    run can reach: its ``J`` stays finite and above that of any finite run. Its
    gradient is not defined, since the rollout itself overflowed before the
    ``where``. ``running_cost`` is the run's integral of
    ``e^{−ρ(t − t_start)} g`` carried with the state by the step model (the
    evaluator's), ``(N + 1,)``; without it, a logged run, the integral is
    the trapezoid on the samples. ``tf`` is the horizon's length (by default the run's,
    ``t[-1] − t_start``) and ``t_start`` its start. ``params`` overrides
    the problem's own leaves (``"cost"``, the sets' by role, ``"t_start"``,
    ``"discount_rate"``); ``None`` reads the problem's. ``price``
    is the failure price as :func:`price_of` resolves it (``None`` the switch
    off); by default the problem's. ``cost_bound`` is the cost bound as
    :func:`diverged_cost_of` resolves it; by default the problem's
    ``cost_bound(tf)``, its ``1e6`` fallback when the boxes bound no cost.
    """
    params = problem_leaves(problem) if params is None else params
    xp = array_module(traj.x)

    cost_fn = problem.require_cost()
    rho = params.get("discount_rate", problem.discount_rate)
    t_start = params.get("t_start", problem.t_start)
    finite_horizon = problem.horizon_kind() == "finite"
    X = problem.X
    U = problem.U
    cost_params = params.get("cost")
    X_params = params.get("X")
    U_params = params.get("U")
    Xf_params = params.get("Xf")

    t = traj.t
    x = traj.x.T
    u = traj.u.T
    tf = t[-1] - t_start if tf is None else tf
    t_end = t_start + tf
    price = price_of(problem, tf) if price is PROBLEM_PRICE else price
    if cost_bound is None:
        cost_bound = diverged_cost_of(problem, tf, price)

    # Finite: every sample, then the run on zeros where it is not (the safe where)
    finite = xp.all(xp.isfinite(x)) & xp.all(xp.isfinite(u))
    x = xp.where(finite, x, 0.0)
    u = xp.where(finite, u, 0.0)

    # The least margin to X and U along the run, the margin of x(tf) to Xf
    feasibility_margin = xp.minimum(
        least_margin(lambda xk, uk, tk: X.margin(xk, tk, X_params), x, u, t),
        least_margin(lambda xk, uk, tk: U.margin(uk, xk, tk, U_params), x, u, t),
    )
    failed = ~finite | (feasibility_margin < 0.0)
    if problem.Xf is None:
        reachability_margin = xp.asarray(xp.inf)
    else:
        reachability_margin = xp.min(problem.Xf.margin(x[-1], t_end, Xf_params))
    reached = reachability_margin >= 0.0

    # cost = ∫ e^{−ρ(t − t_start)} g dt + e^{−ρ tf} h(x(t_end), t_end): the
    # integral the run carried, or the trapezoid on a logged run's samples
    if running_cost is None:
        safe_run = Trajectory(t=t, x=x.T, u=u.T)
        _, running = cost_fn.integrate(
            safe_run, discount_rate=rho, params=cost_params, t_start=t_start
        )
    else:
        running = xp.where(finite, running_cost, 0.0)
    cost = running[-1]
    if finite_horizon:
        cost = cost + xp.exp(-rho * tf) * cost_fn.h(x[-1], t_end, cost_params)
    cost = xp.where(finite, cost, xp.inf)  # A run not finite costs +inf

    # J = cost, or with the price on cost + price · (failed or not reached),
    # and cost_bound + price for a run not finite
    if price is None:
        J = cost
    else:
        J = xp.where(finite, cost, cost_bound) + xp.where(failed | ~reached, price, 0.0)

    return {
        "cost": cost,
        "J": J,
        "feasibility_margin": feasibility_margin,
        "failed": failed,
        "reachability_margin": reachability_margin,
        "reached": reached,
    }


def evaluate_trajectory(problem, traj: Trajectory) -> Evaluation:
    """
    The :class:`Evaluation` of a trajectory you already hold: one row, by :func:`score_run`.

    A loop's run is projected to the plant's by its ``plant`` role
    (``traj.of("plant")``); a trajectory without roles is the plant's own only
    when its ``x`` and ``u`` have the plant's sizes and it records no signals
    (a logged run). A loop's run from the evaluator carries its running cost,
    the stage cost integrated with the state, so its score is the
    evaluator's; a logged run, which has none, is scored by the trapezoid on
    its samples. On the trajectory's array module.
    """
    run = plant_run_of(problem, traj)
    xp = array_module(run.x)

    # A loop's run carries its running cost: its score is the evaluator's;
    # a logged run is scored on its samples by the trapezoid
    running = traj.signals.get(RUNNING_COST)
    row = score_run(problem, run, running_cost=None if running is None else running[0])

    return Evaluation(
        **{name: xp.reshape(value, (1,)) for name, value in row.items()},
        x0=xp.reshape(run.x[:, 0], (1, -1)),
        trajectories=[traj],
        criterion=criterion_of(problem),
    )


###############################################################################
# Internal machinery
###############################################################################


def plant_run_of(problem, traj):
    """The plant's run in ``traj``: its ``plant`` role, else ``traj`` when it is a logged plant run."""
    if "plant" in traj.roles:
        return traj.of("plant")

    plant = problem.sys
    sizes = (int(traj.x.shape[0]), int(traj.u.shape[0]))
    if traj.signals:
        raise ValueError(
            "a trajectory with recorded signals and no 'plant' role: score the loop's "
            "run from problem.compile(...).trajectory(), or pass traj.of('plant')"
        )
    if sizes != (int(plant.n), int(plant.m)):
        raise ValueError(
            f"the trajectory's (n, m) = {sizes} is not the plant's "
            f"({int(plant.n)}, {int(plant.m)}): pass the plant's run"
        )

    return traj


def least_margin(margin, x, u, t):
    """``min_k min margin(x_k, u_k, t_k)`` over the run (``+inf`` for no rows)."""
    xp = array_module(x)
    if xp is np:
        rows = [xp.reshape(margin(xk, uk, tk), (-1,)) for xk, uk, tk in zip(x, u, t)]
        values = xp.concatenate(rows) if rows else xp.zeros(0)
    else:
        jax = ensure_jax_x64()
        values = jax.vmap(lambda xk, uk, tk: xp.reshape(margin(xk, uk, tk), (-1,)))(
            x, u, t
        )

    if values.size == 0:
        return xp.asarray(xp.inf)

    least = xp.min(values)

    return least


def problem_leaves(problem) -> dict:
    """The problem's own leaves of a run's tree: ``"cost"``, the sets' parameters by role, ``"x_start"``, ``"t_start"``, ``"discount_rate"``."""
    leaves = {"cost": problem.cost.params if problem.cost is not None else None}
    for role in ("X", "U", "Xf"):
        set_params = getattr(getattr(problem, role), "params", None)
        if set_params:
            leaves[role] = set_params
    leaves["x_start"] = problem.x_start
    leaves["t_start"] = problem.t_start
    leaves["discount_rate"] = problem.discount_rate

    return leaves


def price_of(problem, tf, cost_params=None, discount_rate=None):
    """The price of a failed or missed run of length ``tf``: ``None`` (switch off), the bound (``True``), or the number.

    ``cost_params`` are the cost's numbers the bound is taken at (an
    evaluation's overridden weights); ``None`` reads the cost's own.
    ``discount_rate`` is the evaluation's rate (``None``: the problem's).
    """
    price = problem.infeasible_cost
    if price is None:
        return None
    if price is True:
        return problem.cost_bound(tf, discount_rate=discount_rate, params=cost_params)

    return price


def diverged_cost_of(problem, tf, price, cost_params=None, discount_rate=None):
    """
    The cost a run that is not finite is charged with the price on: the problem's cost bound.

    ``None`` with the price off. ``price`` is the one :func:`price_of` resolved
    for the problem: when the problem prices a failure at its bound
    (``infeasible_cost=True``) it is the bound itself; else the bound is computed
    here, once, and its ``1e6`` fallback is not announced, the price being the
    problem's own number. ``cost_params`` and ``discount_rate`` as :func:`price_of`.
    """
    if price is None:
        return None
    if problem.infeasible_cost is True:
        return price

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return problem.cost_bound(tf, discount_rate=discount_rate, params=cost_params)


def criterion_of(problem):
    """The criterion over the runs: the problem's, or the expectation of one run."""
    return getattr(problem, "criterion", "expectation")


def reduce_criterion(J, criterion):
    """The criterion over ``J``: ``"expectation"``, ``"worst_case"`` or ``("cvar", α)``."""
    xp = array_module(J)

    # E[J], max J, or CVaR_α: the mean of the worst ⌈α n⌉ runs
    if criterion == "expectation":
        objective = xp.mean(J)
    elif criterion == "worst_case":
        objective = xp.max(J)
    else:
        _, alpha = criterion
        n = J.shape[0]
        k = max(
            1, math.ceil(round(float(alpha) * n, 9))
        )  # 0.07 · 100 is 7.000000000000001
        worst = xp.sort(J)[n - k :]
        objective = xp.mean(worst)

    return objective


@lru_cache(maxsize=None)
def register_evaluation_pytree():
    """Register :class:`Evaluation` as a JAX pytree: the arrays as children, the criterion static."""
    jax = ensure_jax_x64()

    fields = (
        "cost",
        "J",
        "feasibility_margin",
        "failed",
        "reachability_margin",
        "reached",
        "x0",
        "trajectories",
    )

    def flatten(record):
        return [getattr(record, name) for name in fields], record.criterion

    def unflatten(criterion, children):
        return Evaluation(**dict(zip(fields, children)), criterion=criterion)

    jax.tree_util.register_pytree_node(Evaluation, flatten, unflatten)
