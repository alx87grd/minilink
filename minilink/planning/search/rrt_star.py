"""RRT*: the RRT with a near-neighbour parent search and cost-based rewiring."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace

import numpy as np

from minilink.planning.planner import planner_at
from minilink.planning.results import PlanningSolution
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.metric import euclidean
from minilink.planning.search.rrt import (
    RRTPlanner,
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
    changes. Same constructor as :class:`RRTPlanner`, plus the RRT* keywords:

    - ``radius_constant``: the scale ``γ`` of the rewiring radius
      ``r_n = min(γ (log n / n)^(1/d), η)``; unset, ``2 (1 + 1/d)^(1/d)
      vol(X)^(1/d)`` from the sampling box;
    - ``rewire_eta``: the longest connection ``η``; unset, the extender's step
      (``SteeringExtender.max_distance`` or ``KinodynamicExtender.horizon``);
    - ``rewire``: ``False`` keeps the parent selection and skips the rewiring;
    - ``optimize_after_goal``, ``cost_tol``, ``convergence_patience``: keep
      searching after the first goal connection until the best goal cost has
      not improved by ``cost_tol`` for ``convergence_patience`` extensions.

    Its :meth:`solve` takes RRT's workflow, plus ``record_history`` and
    ``history_stride``: an :class:`RRTStarSnapshot` every ``history_stride``
    extensions, for the animation.

    Attributes
    ----------
    converged : bool
        ``True`` when post-goal optimization stopped on the patience criterion.
    iterations : int
        Number of successful tree extensions performed.
    best_goal_cost : float or None
        Best cost-to-come among nodes in the goal region.
    history : list of RRTStarSnapshot
        Recorded frames of a ``solve(record_history=True)``.
    """

    number_names = RRTPlanner.number_names + ("radius_constant", "rewire_eta")

    def __init__(
        self,
        problem,
        extender=None,
        *,
        metric=euclidean,
        max_nodes: int = 5000,
        goal_bias: float = 0.1,
        seed: int = 0,
        edge_resolution: float | None = None,
        max_sample_attempts: int = 100,
        return_best_effort: bool = True,
        nearest_method: str = "brute_force",
        radius_constant: float | None = None,
        rewire_eta: float | None = None,
        rewire: bool = True,
        optimize_after_goal: bool = False,
        cost_tol: float = 0.05,
        convergence_patience: int = 500,
    ) -> None:
        super().__init__(
            problem,
            extender,
            metric=metric,
            max_nodes=max_nodes,
            goal_bias=goal_bias,
            seed=seed,
            edge_resolution=edge_resolution,
            max_sample_attempts=max_sample_attempts,
            return_best_effort=return_best_effort,
            nearest_method=nearest_method,
        )

        # The rewiring: its radius constant and longest connection, and its switch
        self.radius_constant = radius_constant
        self.rewire_eta = rewire_eta
        self.rewire = bool(rewire)

        # The post-goal refinement and its stopping rule
        self.optimize_after_goal = bool(optimize_after_goal)
        self.cost_tol = float(cost_tol)
        self.convergence_patience = int(convergence_patience)

        # The search's convergence, refreshed by every solve
        self.converged: bool = False
        self.best_goal_cost: float | None = None
        self.history: list[RRTStarSnapshot] = []

    def solve(
        self,
        *,
        params=None,
        evaluate=False,
        n_trials=50,
        callback: Callable | list[Callable] | None = None,
        record_history: bool = False,
        history_stride: int = 1,
    ) -> PlanningSolution:
        """
        Grow an RRT* until goal convergence, patience, or ``max_nodes`` (the solution as :meth:`RRTPlanner.solve`'s).

        ``params`` (the complete tree of :attr:`params`, a copy edited) solves
        the planner rebuilt there (:func:`~minilink.planning.planner.planner_at`), this one unchanged.
        The workflow is :meth:`RRTPlanner.solve`'s, plus ``record_history``: an
        :class:`RRTStarSnapshot` in :attr:`history` on the first and every
        ``history_stride``-th extension, and on the last.
        """
        # At another tree: the planner there, its solution kept here
        if params is not None:
            solution = planner_at(self, params).solve(
                evaluate=evaluate,
                n_trials=n_trials,
                record_history=record_history,
                history_stride=history_stride,
                callback=callback,
            )
            return self.store_solution(solution)

        metric = self.metric
        self.require_nearest_method()

        rng = np.random.default_rng(self.seed)
        goal_region = self.goal_region()

        # The post-goal refinement and the snapshots
        optimize_after_goal = self.optimize_after_goal
        cost_tol = self.cost_tol
        patience = self.convergence_patience
        history_stride = max(1, int(history_stride))
        x0 = np.asarray(self.problem.x0, dtype=float)

        # The tree, rooted at the start
        self.tree = Tree(
            Node(x=x0, parent=None, edge=None, cost=0.0),
            nearest_method=self.nearest_method,
        )
        self.solution_node = None
        self.converged = False
        self.best_goal_cost = None
        self.history = []
        report = self.build_search_report(callback)

        # The extensions so far, and whether one reached the goal: the record's counts
        n_iterations = 0
        reached_goal = False
        best_goal_node = None
        stale_extensions = 0

        for _ in range(self.max_nodes):
            # Extend: steer toward a sample, attach through the cheapest near
            # parent, rewire the near nodes
            if not self.extend_once(rng):
                continue
            n_iterations += 1

            # The cheapest node in the goal region, and whether it improved
            best_goal_node, improved = self.refresh_best_goal(
                goal_region, best_goal_node, cost_tol
            )
            if best_goal_node is not None:
                reached_goal = True
                self.solution_node = best_goal_node
                self.best_goal_cost = float(best_goal_node.cost)

            # The step's callback, and its snapshot when the history is recorded
            phase = "optimize" if reached_goal and optimize_after_goal else "explore"
            report(phase, n_iterations, reached_goal)
            if record_history and (
                n_iterations == 1 or n_iterations % history_stride == 0
            ):
                self.record_snapshot(best_goal_node, n_iterations)

            # Stop at the first goal hit, or once the goal cost has gone patience
            # extensions without improving by cost_tol
            if reached_goal and not optimize_after_goal:
                break

            if reached_goal and optimize_after_goal:
                if improved:
                    stale_extensions = 0
                else:
                    stale_extensions += 1
                if stale_extensions >= patience:
                    self.converged = True
                    break

        # The last extension's snapshot, when the stride skipped it
        if record_history and (
            not self.history or self.history[-1].iteration != n_iterations
        ):
            self.record_snapshot(best_goal_node, n_iterations)

        # No goal node: the node closest to the goal, when best effort is allowed
        if self.solution_node is None:
            goal_point = self.goal_point(goal_region)
            closest = self.tree.nearest(goal_point, metric)
            self.solution_node = closest
            if not self.return_best_effort:
                raise RuntimeError("RRT* failed to reach goal within max_nodes")

        return self.finish_trajectory(
            self.tree.extract_trajectory(self.solution_node, self.problem.t0),
            evaluate,
            n_trials,
            self.search_record(reached_goal, n_iterations),
        )

    def build_kwargs(self) -> dict:
        """The keywords that rebuild this planner on another problem: RRT's, and the rewiring's."""
        kwargs = super().build_kwargs()
        kwargs["radius_constant"] = self.radius_constant
        kwargs["rewire_eta"] = self.rewire_eta
        kwargs["rewire"] = self.rewire
        kwargs["optimize_after_goal"] = self.optimize_after_goal
        kwargs["cost_tol"] = self.cost_tol
        kwargs["convergence_patience"] = self.convergence_patience

        return kwargs

    def search_record(self, reached_goal, n_iterations):
        """The RRT* record: the best goal cost, and whether patience declared convergence."""
        record = super().search_record(reached_goal, n_iterations)
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
        """Animate recorded RRT* history (a ``solve(record_history=True)``)."""
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
        if self.rewire:
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
        """The radius scale γ: :attr:`radius_constant`, or the default from the volume of the sampling box."""
        if self.radius_constant is not None:
            return float(self.radius_constant)

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
        eta = self.rewire_eta
        if eta is not None:
            return float(eta)

        extender = self.extender
        if isinstance(extender, SteeringExtender):
            return float(extender.max_distance)
        if isinstance(extender, KinodynamicExtender):
            return float(extender.horizon)

        raise ValueError(
            "rewire_eta= is required when the extender has no "
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

    def record_snapshot(self, best_goal_node, n_iterations):
        """Append the tree and the best goal path of extension ``n_iterations`` to the history."""
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
                iteration=n_iterations,
                best_cost=(
                    None if best_goal_node is None else float(best_goal_node.cost)
                ),
                reached_goal=best_goal_node is not None,
                tree_edges=tree_edges,
                path_states=path_states,
            )
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
