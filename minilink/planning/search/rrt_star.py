"""
RRT* — asymptotically optimal sampling-based motion planning.

:class:`RRTStarPlanner` extends :class:`~minilink.planning.search.rrt.RRTPlanner`
with a near-neighbour parent search and cost-based rewiring. It reuses the same
``TrajectoryExtender`` and ``metric`` seams; only the attach step changes.

When ``optimize_after_goal`` is set, the search keeps running after the first
feasible goal connection and stops once the best goal cost-to-come has not
improved by at least ``cost_tol`` for ``convergence_patience`` consecutive
successful extensions.
"""

from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.tree import Node, Tree

# Public API


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


@dataclass
class RRTStarOptions(RRTOptions):
    """
    Workflow options for :class:`RRTStarPlanner`.

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


class RRTStarPlanner(RRTPlanner):
    """
    RRT* over a `PlanningProblem`.

    Same constructor as :class:`RRTPlanner`, but ``options`` should be an
    :class:`RRTStarOptions` instance (a plain :class:`RRTOptions` still works
    with rewiring defaults).

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

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.converged: bool = False
        self.iterations: int = 0
        self.best_goal_cost: float | None = None
        self.history: list[RRTStarSnapshot] = []

    def compute_solution(self) -> Trajectory:
        """Grow an RRT* until goal convergence, patience, or ``max_nodes``."""
        options = self.options
        self._validate_nearest_backend()
        rng = np.random.default_rng(options.seed)
        goal_region = self._goal_region()
        star_options = options if isinstance(options, RRTStarOptions) else None
        optimize_after_goal = (
            star_options.optimize_after_goal if star_options else False
        )
        cost_tol = star_options.cost_tol if star_options else 0.05
        patience = star_options.convergence_patience if star_options else 500
        record_history = star_options.record_history if star_options else False
        history_stride = max(
            1, star_options.history_stride if star_options else 1
        )

        x_start = np.asarray(self.problem.x_start, dtype=float)
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
        self._search_callback = self._resolve_search_callback()

        best_goal_node = None
        stale_extensions = 0

        for _ in range(options.max_nodes):
            if not self._extend_once(rng):
                continue

            self.iterations += 1
            best_goal_node, improved = self._refresh_best_goal(
                goal_region, best_goal_node, cost_tol
            )

            if best_goal_node is not None:
                self.reached_goal = True
                self.solution_node = best_goal_node
                self.best_goal_cost = float(best_goal_node.cost)

            phase = (
                "optimize"
                if self.reached_goal and optimize_after_goal
                else "explore"
            )
            self._on_search_step(phase=phase)

            if record_history and (
                self.iterations == 1 or self.iterations % history_stride == 0
            ):
                self._record_snapshot(best_goal_node)

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

        if record_history and (
            not self.history or self.history[-1].iteration != self.iterations
        ):
            self._record_snapshot(best_goal_node)

        if self.solution_node is None:
            goal_point = self._goal_point(goal_region)
            closest = self.tree.nearest(goal_point, self.metric)
            self.solution_node = closest
            if not options.return_best_effort:
                raise RuntimeError("RRT* failed to reach goal within max_nodes")

        return self._store_result(
            self.tree.extract_trajectory(self.solution_node)
        )

    def animate_convergence(self, **kwargs):
        """Animate recorded RRT* history (requires ``record_history=True``)."""
        from minilink.planning.search.plotting import animate_convergence

        return animate_convergence(self, **kwargs)

    # Internal machinery

    def _extend_once(self, rng) -> bool:
        x_rand = self._sample(rng)
        near = self.tree.nearest(x_rand, self.metric)
        edge = self._try_extend(near, x_rand, rng)
        if edge is None:
            return False

        x_new = edge.x_end
        radius = self._rewire_radius(len(self.tree.nodes))
        near_nodes = self.tree.near(x_new, radius, self.metric)

        best_parent, best_edge, best_cost = self._choose_parent(
            near_nodes, x_new, rng
        )
        if best_edge is None:
            return False

        new_node = self.tree.add(
            Node(
                x=x_new,
                parent=best_parent,
                edge=best_edge,
                cost=best_cost,
            )
        )

        if getattr(self.options, "rewire", True):
            self._rewire_near(new_node, near_nodes, rng)

        return True

    def _refresh_best_goal(self, goal_region, best_goal_node, cost_tol):
        improved = False
        previous_cost = (
            None if best_goal_node is None else float(best_goal_node.cost)
        )

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

    def _record_snapshot(self, best_goal_node):
        tree_edges = [
            np.asarray(node.edge.states, dtype=float)
            for node in self.tree.nodes
            if node.parent is not None
        ]
        path_states = None
        if best_goal_node is not None:
            path_states = self._path_states(best_goal_node)

        self.history.append(
            RRTStarSnapshot(
                iteration=self.iterations,
                best_cost=(
                    None
                    if best_goal_node is None
                    else float(best_goal_node.cost)
                ),
                reached_goal=best_goal_node is not None,
                tree_edges=tree_edges,
                path_states=path_states,
            )
        )

    def _choose_parent(self, near_nodes, x_new, rng):
        best_parent = None
        best_edge = None
        best_cost = np.inf

        for node in near_nodes:
            edge = self._try_extend(node, x_new, rng)
            if edge is None:
                continue
            cost = node.cost + edge.cost
            if cost < best_cost:
                best_cost = cost
                best_parent = node
                best_edge = edge

        return best_parent, best_edge, best_cost

    def _rewire_near(self, new_node, near_nodes, rng):
        for candidate in near_nodes:
            if candidate is new_node.parent:
                continue
            edge = self._try_extend(new_node, candidate.x, rng)
            if edge is None:
                continue
            new_cost = new_node.cost + edge.cost
            if new_cost < candidate.cost:
                self.tree.rewire(candidate, new_node, edge)
                self.tree.propagate_cost(candidate)

    def _rewire_radius(self, n_nodes: int) -> float:
        d = int(self.problem.sys.n)
        n = max(int(n_nodes), 2)
        gamma = self._gamma()
        eta = self._rewire_eta()
        log_term = (np.log(n) / n) ** (1.0 / d)
        return min(gamma * log_term, eta)

    def _gamma(self) -> float:
        gamma = getattr(self.options, "gamma", None)
        if gamma is not None:
            return float(gamma)

        d = int(self.problem.sys.n)
        span = self._sample_box.upper - self._sample_box.lower
        volume = float(np.prod(np.maximum(span, 1e-12)))
        return 2.0 * (1.0 + 1.0 / d) ** (1.0 / d) * (volume ** (1.0 / d))

    def _rewire_eta(self) -> float:
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
