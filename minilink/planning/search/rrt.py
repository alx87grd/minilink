"""
The RRT orchestrator.

`RRTPlanner` owns the invariant loop — sample, nearest, propose, select, add,
goal-test, extract — and sources every concern from the `PlanningProblem`:
collision from ``problem.X``, the goal region from ``problem.Xf``/``x_goal``,
sampling from the state bounds and ``problem.U``, dynamics from ``problem.sys``.
The only swappable pieces are the injected ``extender`` (how two states connect)
and ``metric`` (the nearest-neighbour distance).
"""

from dataclasses import dataclass

import numpy as np

from minilink.core.sets import BallSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.metric import euclidean
from minilink.planning.search.tree import Node, Tree

# Public API


@dataclass
class RRTOptions:
    """Workflow options for :class:`RRTPlanner` (step/resolution are extender-owned)."""

    max_nodes: int = 5000
    goal_bias: float = 0.1
    goal_tolerance: float = 0.5
    seed: int | None = None


class RRTPlanner(Planner):
    """
    Rapidly-exploring random tree over a `PlanningProblem`.

    Parameters
    ----------
    problem : PlanningProblem
        Supplies dynamics, free space (``X``), goal (``Xf``/``x_goal``), and bounds.
    extender : TrajectoryExtender
        Proposes candidate edges; the orchestrator selects the best collision-free one.
    metric : callable
        Pairwise nearest-neighbour distance ``metric(a, b) -> float``.
    options : RRTOptions
        Loop options.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        extender,
        metric=euclidean,
        options: RRTOptions | None = None,
    ) -> None:
        super().__init__(problem)
        self.extender = extender
        self.metric = metric
        self.options = RRTOptions() if options is None else options
        self.tree: Tree | None = None
        self._sample_box = BoxSet.from_system_state(problem.sys)

    def compute_solution(self) -> Trajectory:
        """Grow the tree until the goal region is reached or the budget is spent."""
        problem = self.problem
        options = self.options
        rng = np.random.default_rng(options.seed)
        goal_region = self._goal_region()

        x_start = np.asarray(problem.x_start, dtype=float)
        self.tree = Tree(Node(x=x_start, parent=None, edge=None, cost=0.0))

        for _ in range(options.max_nodes):
            x_rand = self._sample(rng)
            near = self.tree.nearest(x_rand, self.metric)
            edge = self._select(
                self.extender.propose(near.x, x_rand, problem, rng), x_rand
            )
            if edge is None:
                continue

            node = self.tree.add(
                Node(x=edge.x_end, parent=near, edge=edge, cost=near.cost + edge.cost)
            )
            if goal_region.contains(node.x):
                return self._store_result(self.tree.extract_trajectory(node))

        # budget spent: return the path to the node closest to the goal point
        goal_point = self._goal_point(goal_region)
        closest = self.tree.nearest(goal_point, self.metric)
        return self._store_result(self.tree.extract_trajectory(closest))

    def plot_tree(self, **kwargs):
        """Plot the final tree and solution path (lazy matplotlib import).

        ``x_axis`` / ``y_axis`` choose the state projection; pass ``ax`` to
        overlay on a ``scene.plot`` heatmap.
        """
        from minilink.planning.search.plotting import plot_tree

        return plot_tree(self, **kwargs)

    def animate_search(self, **kwargs):
        """Animate the tree growth in a state projection (lazy matplotlib import)."""
        from minilink.planning.search.plotting import animate_search

        return animate_search(self, **kwargs)

    # Internal machinery

    def _sample(self, rng):
        if self.problem.x_goal is not None and rng.random() < self.options.goal_bias:
            return np.asarray(self.problem.x_goal, dtype=float)
        return self._sample_box.sample(rng)[0]

    def _select(self, candidates, x_rand):
        best, best_distance = None, np.inf
        for edge in candidates:
            if not self._edge_is_free(edge):
                continue
            distance = self.metric(edge.x_end, x_rand)
            if distance < best_distance:
                best, best_distance = edge, distance
        return best

    def _edge_is_free(self, edge) -> bool:
        problem = self.problem
        if problem.X is not None:
            if not all(problem.X.contains(state) for state in edge.states):
                return False
        if problem.U is not None:
            if not all(problem.U.contains(u) for u in edge.inputs):
                return False
        return True

    def _goal_region(self):
        Xf = self.problem.Xf
        if Xf is not None and not isinstance(Xf, SingletonSet):
            return Xf
        if self.problem.x_goal is None:
            raise ValueError("RRT requires a goal: set x_goal or a region Xf")
        return BallSet(self.problem.x_goal, self.options.goal_tolerance)

    def _goal_point(self, goal_region):
        if self.problem.x_goal is not None:
            return np.asarray(self.problem.x_goal, dtype=float)
        center = getattr(goal_region, "center", None)
        if center is None:
            raise ValueError(
                "RRT goal has no representative point for best-effort fallback"
            )
        return np.asarray(center, dtype=float)
