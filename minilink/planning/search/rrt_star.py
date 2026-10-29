"""RRT*: the RRT with a near-neighbour parent search and cost-based rewiring."""

from __future__ import annotations

from dataclasses import dataclass, fields, replace

import numpy as np

from minilink.planning.results import PlanningSolution
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.metric import euclidean
from minilink.planning.search.rrt import (
    _RRT_OPTION_KEYS,
    _UNSET,
    RRTOptions,
    RRTPlanner,
    rrt_options_of,
)
from minilink.planning.search.tree import Node, Tree

###############################################################################
# Public API
###############################################################################


class RRTStarPlanner(RRTPlanner):
    """
    RRT* over a `PlanningProblem`.

    Extends :class:`~minilink.planning.search.rrt.RRTPlanner` with a
    near-neighbour parent search and cost-based rewiring; it reuses the same
    ``TrajectoryExtender`` and ``metric`` seams, and only the attach step
    changes. Same constructor as :class:`RRTPlanner`, plus RRT* flat kwargs /
    :class:`RRTStarOptions`. A plain :class:`RRTOptions` still works with
    rewiring defaults.

    Attributes
    ----------
    converged : bool
        ``True`` when post-goal optimization stopped on the patience criterion.
    iterations : int
        Number of successful tree extensions performed.
    best_goal_cost : float or None
        Best cost-to-come among nodes in the goal region.
    history : list of RRTStarSnapshot
        Recorded frames when ``record_history`` is enabled.
    """

    def __init__(
        self,
        problem,
        extender=None,
        *,
        metric=euclidean,
        options: RRTOptions | None = None,
        max_nodes=_UNSET,
        goal_bias=_UNSET,
        seed=_UNSET,
        edge_resolution=_UNSET,
        max_sample_attempts=_UNSET,
        return_best_effort=_UNSET,
        callback=_UNSET,
        live_plot=_UNSET,
        live_plot_every=_UNSET,
        live_plot_pause=_UNSET,
        live_plot_after_goal_only=_UNSET,
        live_plot_ax=_UNSET,
        nearest_backend=_UNSET,
        gamma=_UNSET,
        rewire_eta=_UNSET,
        rewire=_UNSET,
        optimize_after_goal=_UNSET,
        cost_tol=_UNSET,
        convergence_patience=_UNSET,
        record_history=_UNSET,
        history_stride=_UNSET,
    ) -> None:
        # Workflow options: the flat keywords over the bag, a plain RRTOptions
        # completed with the RRT* defaults
        star_options = rrt_options_of(
            rrt_star_options_of(options),
            default_factory=RRTStarOptions,
            allowed_keys=_RRT_STAR_OPTION_KEYS,
            max_nodes=max_nodes,
            goal_bias=goal_bias,
            seed=seed,
            edge_resolution=edge_resolution,
            max_sample_attempts=max_sample_attempts,
            return_best_effort=return_best_effort,
            callback=callback,
            live_plot=live_plot,
            live_plot_every=live_plot_every,
            live_plot_pause=live_plot_pause,
            live_plot_after_goal_only=live_plot_after_goal_only,
            live_plot_ax=live_plot_ax,
            nearest_backend=nearest_backend,
            gamma=gamma,
            rewire_eta=rewire_eta,
            rewire=rewire,
            optimize_after_goal=optimize_after_goal,
            cost_tol=cost_tol,
            convergence_patience=convergence_patience,
            record_history=record_history,
            history_stride=history_stride,
        )
        super().__init__(problem, extender, metric=metric, options=star_options)

        # The search's convergence, refreshed by every solve
        self.converged: bool = False
        self.iterations: int = 0
        self.best_goal_cost: float | None = None
        self.history: list[RRTStarSnapshot] = []

    def solve(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Offline traj-family entry (see :meth:`RRTPlanner.solve`)."""
        return self.solve_trajectory(evaluate=evaluate, n_trials=n_trials)

    def solve_trajectory(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Grow an RRT* until goal convergence, patience, or ``max_nodes``."""
        options = self.options
        metric = self.metric
        self.require_nearest_backend()

        rng = np.random.default_rng(options.seed)
        goal_region = self.goal_region()

        # The RRT* options, or their defaults when the planner holds a plain RRTOptions
        star_options = options if isinstance(options, RRTStarOptions) else None
        optimize_after_goal = (
            star_options.optimize_after_goal if star_options else False
        )
        cost_tol = star_options.cost_tol if star_options else 0.05
        patience = star_options.convergence_patience if star_options else 500
        record_history = star_options.record_history if star_options else False
        history_stride = max(1, star_options.history_stride if star_options else 1)
        x_start = np.asarray(self.problem.x_start, dtype=float)

        # The tree, rooted at the start
        self.tree = Tree(
            Node(x=x_start, parent=None, edge=None, cost=0.0),
            nearest_backend=options.nearest_backend,
        )
        self.reached_goal = False
        self.solution_node = None
        self.converged = False
        self.iterations = 0
        self.best_goal_cost = None
        self.history = []
        self.search_callback = self.resolve_search_callback()

        best_goal_node = None
        stale_extensions = 0

        for _ in range(options.max_nodes):
            # Extend: steer toward a sample, attach through the cheapest near
            # parent, rewire the near nodes
            if not self.extend_once(rng):
                continue
            self.iterations += 1

            # The cheapest node in the goal region, and whether it improved
            best_goal_node, improved = self.refresh_best_goal(
                goal_region, best_goal_node, cost_tol
            )
            if best_goal_node is not None:
                self.reached_goal = True
                self.solution_node = best_goal_node
                self.best_goal_cost = float(best_goal_node.cost)

            # The step's callback, and its snapshot when the history is recorded
            phase = (
                "optimize" if self.reached_goal and optimize_after_goal else "explore"
            )
            self.record_search_step(
                best_goal_node, phase, record_history, history_stride
            )

            # Stop at the first goal hit, or once the goal cost has gone patience
            # extensions without improving by cost_tol
            if self.reached_goal and not optimize_after_goal:
                break

            if self.reached_goal and optimize_after_goal:
                if improved:
                    stale_extensions = 0
                else:
                    stale_extensions += 1
                if stale_extensions >= patience:
                    self.converged = True
                    break

        # The last extension's snapshot, when the stride skipped it
        if record_history and (
            not self.history or self.history[-1].iteration != self.iterations
        ):
            self.record_snapshot(best_goal_node)

        # No goal node: the node closest to the goal, when best effort is allowed
        if self.solution_node is None:
            goal_point = self.goal_point(goal_region)
            closest = self.tree.nearest(goal_point, metric)
            self.solution_node = closest
            if not options.return_best_effort:
                raise RuntimeError("RRT* failed to reach goal within max_nodes")

        return self.finish_trajectory(
            self.tree.extract_trajectory(self.solution_node), evaluate, n_trials
        )

    def search_record(self):
        """The RRT* record: the best goal cost, and whether patience declared convergence."""
        record = super().search_record()
        cost = (
            record.cost if self.best_goal_cost is None else float(self.best_goal_cost)
        )

        return replace(
            record,
            cost=cost,
            converged=bool(self.converged),
            history=list(self.history),
        )

    def animate_convergence(self, **kwargs):
        """Animate recorded RRT* history (requires ``record_history=True``)."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.plotting import animate_convergence

        return animate_convergence(self, **kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def extend_once(self, rng) -> bool:
        """One RRT* extension; ``False`` when no collision-free edge was found."""
        tree = self.tree
        metric = self.metric

        # Steer from the nearest node toward a random sample
        x_rand = self.sample_state(rng)
        near = tree.nearest(x_rand, metric)
        edge = self.try_extend(near, x_rand, rng)
        if edge is None:
            return False

        # The near set: the nodes within the rewiring radius of the new state
        x_new = edge.x_end
        radius = self.rewire_radius(len(tree.nodes))
        near_nodes = tree.near(x_new, radius, metric)

        # The new state joins the tree through its cheapest near parent
        best_parent, best_edge, best_cost = self.choose_parent(near_nodes, x_new, rng)
        if best_edge is None:
            return False

        new_node = tree.add(
            Node(
                x=x_new,
                parent=best_parent,
                edge=best_edge,
                cost=best_cost,
            )
        )

        # The near nodes that are cheaper to reach through it are rewired
        if getattr(self.options, "rewire", True):
            self.rewire_near(new_node, near_nodes, rng)

        return True

    def rewire_radius(self, n_nodes: int) -> float:
        """The rewiring radius of a tree of ``n_nodes`` nodes."""
        d = int(self.problem.sys.n)
        n = max(int(n_nodes), 2)
        gamma = self.radius_scale()
        eta = self.max_connection()

        # r = min(γ (log n / n)^(1/d), η)
        log_term = (np.log(n) / n) ** (1.0 / d)
        r = min(gamma * log_term, eta)

        return r

    def radius_scale(self) -> float:
        """The radius scale γ: the ``gamma`` option, or the default from the volume of the sampling box."""
        gamma = getattr(self.options, "gamma", None)
        if gamma is not None:
            return float(gamma)

        d = int(self.problem.sys.n)
        lower = self.sample_box.lower
        upper = self.sample_box.upper

        # γ = 2 (1 + 1/d)^(1/d) vol(X)^(1/d), each side of the box at least 10⁻¹²
        span = upper - lower
        volume = float(np.prod(np.maximum(span, 1e-12)))
        gamma = 2.0 * (1.0 + 1.0 / d) ** (1.0 / d) * (volume ** (1.0 / d))

        return gamma

    def max_connection(self) -> float:
        """The longest connection η: the ``rewire_eta`` option, or the extender's step."""
        eta = getattr(self.options, "rewire_eta", None)
        if eta is not None:
            return float(eta)

        extender = self.extender
        if isinstance(extender, SteeringExtender):
            return float(extender.max_distance)
        if isinstance(extender, KinodynamicExtender):
            return float(extender.horizon)

        raise ValueError(
            "RRTStarOptions.rewire_eta is required when the extender has no "
            "max_distance or horizon attribute"
        )

    def choose_parent(self, near_nodes, x_new, rng):
        """The near node, edge and cost-to-come that reach ``x_new`` cheapest."""
        best_parent = None
        best_edge = None
        best_cost = np.inf

        for node in near_nodes:
            edge = self.try_extend(node, x_new, rng)
            if edge is None:
                continue
            cost = node.cost + edge.cost
            if cost < best_cost:
                best_cost = cost
                best_parent = node
                best_edge = edge

        return best_parent, best_edge, best_cost

    def rewire_near(self, new_node, near_nodes, rng):
        """Reparent each near node through ``new_node`` when that lowers its cost-to-come."""
        tree = self.tree

        for candidate in near_nodes:
            if candidate is new_node.parent:
                continue
            edge = self.try_extend(new_node, candidate.x, rng)
            if edge is None:
                continue
            new_cost = new_node.cost + edge.cost
            if new_cost < candidate.cost:
                tree.rewire(candidate, new_node, edge)
                tree.propagate_cost(candidate)

    def refresh_best_goal(self, goal_region, best_goal_node, cost_tol):
        """The cheapest node in the goal region, and whether it beats the previous best by ``cost_tol``."""
        improved = False
        previous_cost = None if best_goal_node is None else float(best_goal_node.cost)

        for node in self.tree.nodes:
            if not goal_region.contains(node.x):
                continue
            if best_goal_node is None or node.cost < best_goal_node.cost:
                best_goal_node = node

        if best_goal_node is None:
            return None, False

        if previous_cost is None or best_goal_node.cost < previous_cost - cost_tol:
            improved = True

        return best_goal_node, improved

    def record_search_step(self, best_goal_node, phase, record_history, history_stride):
        """Hand the step to the solve's callback, then snapshot it on the first and every ``history_stride``-th extension."""
        self.on_search_step(phase=phase)

        if record_history and (
            self.iterations == 1 or self.iterations % history_stride == 0
        ):
            self.record_snapshot(best_goal_node)

    def record_snapshot(self, best_goal_node):
        """Append the tree and the best goal path of this extension to the history."""
        tree_edges = [
            np.asarray(node.edge.states, dtype=float)
            for node in self.tree.nodes
            if node.parent is not None
        ]
        path_states = None
        if best_goal_node is not None:
            path_states = self.path_states(best_goal_node)

        self.history.append(
            RRTStarSnapshot(
                iteration=self.iterations,
                best_cost=(
                    None if best_goal_node is None else float(best_goal_node.cost)
                ),
                reached_goal=best_goal_node is not None,
                tree_edges=tree_edges,
                path_states=path_states,
            )
        )


@dataclass
class RRTStarOptions(RRTOptions):
    """
    Workflow options for :class:`RRTStarPlanner`.

    When ``optimize_after_goal`` is set, the search keeps running after the
    first feasible goal connection and stops once the best goal cost-to-come
    has not improved by at least ``cost_tol`` for ``convergence_patience``
    consecutive successful extensions.

    Parameters
    ----------
    gamma : float, optional
        Scale in the rewiring radius ``r_n = min(gamma * (log(n)/n)^(1/d), eta)``.
        When omitted, ``gamma = 2 (1 + 1/d)^(1/d) volume(X)^(1/d)`` from the
        problem state bounding box.
    rewire_eta : float, optional
        Maximum connection length ``eta``. Defaults to the extender step
        (``SteeringExtender.max_distance`` or ``KinodynamicExtender.horizon``).
    rewire : bool
        When ``False``, skip the rewiring pass (parent selection only).
    optimize_after_goal : bool
        When ``True``, continue searching after the first goal connection until
        the best goal cost stops improving (see ``cost_tol`` and
        ``convergence_patience``). When ``False``, return at the first goal hit.
    cost_tol : float
        Minimum goal cost improvement that resets the convergence counter.
    convergence_patience : int
        Stop after this many successful extensions without a goal-cost
        improvement of at least ``cost_tol`` once a goal has been reached.
    record_history : bool
        Record :class:`RRTStarSnapshot` frames for animation (every
        ``history_stride`` extensions).
    history_stride : int
        Snapshot interval in successful extension steps.
    """

    gamma: float | None = None
    rewire_eta: float | None = None
    rewire: bool = True
    optimize_after_goal: bool = False
    cost_tol: float = 0.05
    convergence_patience: int = 500
    record_history: bool = False
    history_stride: int = 1


#: The flat keywords :class:`RRTStarPlanner` overlays on :class:`RRTStarOptions`.
_RRT_STAR_OPTION_KEYS = _RRT_OPTION_KEYS + (
    "gamma",
    "rewire_eta",
    "rewire",
    "optimize_after_goal",
    "cost_tol",
    "convergence_patience",
    "record_history",
    "history_stride",
)


@dataclass
class RRTStarSnapshot:
    """
    One recorded frame of an RRT* search for convergence animation.

    Parameters
    ----------
    iteration : int
        Extension index when the snapshot was taken.
    best_cost : float or None
        Best goal cost-to-come at this frame (``None`` before any goal node).
    reached_goal : bool
        Whether a goal-region node exists at this frame.
    tree_edges : list of np.ndarray
        Edge state samples ``(k+1, n)`` for every branch in the tree.
    path_states : np.ndarray or None
        State samples ``(k+1, n)`` along the best goal path at this frame.
    """

    iteration: int
    best_cost: float | None
    reached_goal: bool
    tree_edges: list
    path_states: np.ndarray | None = None


###############################################################################
# Internal machinery
###############################################################################


def rrt_star_options_of(options: RRTOptions | None) -> RRTStarOptions:
    """The options as an :class:`RRTStarOptions`: a plain bag's fields under the RRT* defaults."""
    if options is None:
        return RRTStarOptions()
    if isinstance(options, RRTStarOptions):
        return options
    return RRTStarOptions(
        **{f.name: getattr(options, f.name) for f in fields(RRTOptions)}
    )
