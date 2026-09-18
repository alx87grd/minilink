"""
The RRT orchestrator.

`RRTPlanner` owns the invariant loop — sample, nearest, propose, select, add,
goal-test, extract — and sources every concern from the `PlanningProblem`:
collision from ``problem.X``, the goal region from ``problem.Xf``/``x_goal``,
sampling from ``problem.X`` (rejection when needed), dynamics from ``problem.sys``.
The only swappable pieces are the injected ``extender`` (how two states connect)
and ``metric`` (the nearest-neighbour distance).

Note
----
Provisional tool — maintainer review before assigning for coursework.
"""

from dataclasses import dataclass, replace
from typing import Callable

import numpy as np

from minilink.core.sets import BallSet, SingletonSet, is_finite_box
from minilink.core.trajectory import Trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.metric import euclidean
from minilink.planning.search.tree import (
    NEAREST_BACKENDS,
    NEAREST_KD_TREE,
    Node,
    Tree,
)

# Public API

_UNSET = object()

_RRT_OPTION_KEYS = (
    "max_nodes",
    "goal_bias",
    "goal_tolerance",
    "seed",
    "edge_resolution",
    "max_sample_attempts",
    "return_best_effort",
    "callback",
    "live_plot",
    "live_plot_every",
    "live_plot_pause",
    "live_plot_after_goal_only",
    "live_plot_ax",
    "nearest_backend",
)


@dataclass
class RRTOptions:
    """Workflow options for :class:`RRTPlanner` (step/resolution are extender-owned)."""

    max_nodes: int = 5000
    goal_bias: float = 0.1
    goal_tolerance: float = 0.5
    seed: int | None = None
    edge_resolution: float | None = None
    max_sample_attempts: int = 100
    return_best_effort: bool = True
    callback: Callable | None = None
    live_plot: bool = False
    live_plot_every: int = 5
    live_plot_pause: float = 0.001
    live_plot_after_goal_only: bool = False
    live_plot_ax: object | None = None
    nearest_backend: str = "brute_force"


def _merge_rrt_options(
    options: RRTOptions | None,
    *,
    default_factory=RRTOptions,
    allowed_keys=_RRT_OPTION_KEYS,
    **flat,
) -> RRTOptions:
    base = default_factory() if options is None else options
    updates = {key: value for key, value in flat.items() if value is not _UNSET}
    unknown = sorted(k for k in updates if k not in allowed_keys)
    if unknown:
        raise ValueError(f"Unknown RRT planner kwargs: {unknown}.")
    if not updates:
        return base
    return replace(base, **updates)


@dataclass(frozen=True)
class TreeSearchRecord:
    """Whether the goal was reached, how much of the budget the search took, and the path's cost-to-come."""

    reached_goal: bool
    iterations: int
    nodes: int
    cost: float
    converged: bool
    history: list

    @property
    def success(self) -> bool:
        return bool(self.reached_goal)

    def __str__(self) -> str:
        outcome = "goal reached" if self.reached_goal else "goal not reached"
        text = (
            f"{outcome} after {self.iterations} extensions ({self.nodes} nodes), "
            f"path cost {self.cost:.4g}"
        )
        if self.converged:
            text += ", converged"
        return text


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
        Tier-2 workflow bag. Flat kwargs below overlay matching fields.

    Notes
    -----
    Flat keyword arguments overlay :class:`RRTOptions`: ``max_nodes``,
    ``goal_bias``, ``goal_tolerance``, ``seed``, ``edge_resolution``,
    ``max_sample_attempts``, ``return_best_effort``, ``callback``,
    ``live_plot``, ``live_plot_every``, ``live_plot_pause``,
    ``live_plot_after_goal_only``, ``live_plot_ax``, ``nearest_backend``.

    Attributes
    ----------
    reached_goal : bool
        Set by :meth:`solve`; ``True`` only when the goal region is reached.
    solution_node : Node or None
        Tree node whose parent chain defines the returned trajectory.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        extender=None,
        *,
        metric=euclidean,
        options: RRTOptions | None = None,
        max_nodes=_UNSET,
        goal_bias=_UNSET,
        goal_tolerance=_UNSET,
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
    ) -> None:
        super().__init__(problem)
        if extender is None:
            # Bang-bang motion primitives from the input bounds, 0.3 s edges.
            extender = KinodynamicExtender()
        self.extender = extender
        self.metric = metric
        self.options = _merge_rrt_options(
            options,
            max_nodes=max_nodes,
            goal_bias=goal_bias,
            goal_tolerance=goal_tolerance,
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
        )
        self.tree: Tree | None = None
        self.reached_goal: bool = False
        self.solution_node: Node | None = None
        self.iterations: int = 0
        box = problem.X.bounding_box()
        self._sample_box = problem.sys.state.box if not is_finite_box(box) else box

    def solve(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """
        Offline traj-family entry.

        The solution's policy is the path's input held by a source block,
        ``policy >> plant``; ``evaluate=True`` replays it over the problem's
        draws when the problem declares a cost.
        """
        return self.solve_trajectory(evaluate=evaluate, n_trials=n_trials)

    def solve_trajectory(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Grow the tree until the goal region is reached or the budget is spent."""
        options = self.options
        self._validate_nearest_backend()
        rng = np.random.default_rng(options.seed)
        goal_region = self._goal_region()

        x_start = np.asarray(self.problem.x_start, dtype=float)
        self.tree = Tree(
            Node(x=x_start, parent=None, edge=None, cost=0.0),
            nearest_backend=options.nearest_backend,
        )
        self.reached_goal = False
        self.solution_node = None
        self.iterations = 0
        self._search_callback = self._resolve_search_callback()

        for _ in range(options.max_nodes):
            x_rand = self._sample(rng)
            near = self.tree.nearest(x_rand, self.metric)
            edge = self._try_extend(near, x_rand, rng)
            if edge is None:
                continue

            node = self.tree.add(
                Node(x=edge.x_end, parent=near, edge=edge, cost=near.cost + edge.cost)
            )
            self.iterations += 1
            if goal_region.contains(node.x):
                self.reached_goal = True
                self.solution_node = node
                self._on_search_step(phase="explore")
                return self._finish_trajectory(
                    self.tree.extract_trajectory(node), evaluate, n_trials
                )

            self._on_search_step(phase="explore")

        goal_point = self._goal_point(goal_region)
        closest = self.tree.nearest(goal_point, self.metric)
        self.solution_node = closest
        if not options.return_best_effort:
            raise RuntimeError("RRT failed to reach goal within max_nodes")
        return self._finish_trajectory(
            self.tree.extract_trajectory(closest), evaluate, n_trials
        )

    def _finish_trajectory(
        self, trajectory: Trajectory, evaluate, n_trials
    ) -> PlanningSolution:
        """The path as a solution: its input held by a source block, the search's record, scored when asked."""
        policy = self.open_loop_policy(trajectory, interpolation="previous")
        evaluation = None
        if evaluate:
            dt = float(np.median(np.diff(trajectory.t)))
            evaluation = self.evaluate(
                policy, dt=dt, n_trials=n_trials, tf=float(trajectory.tf)
            )
        return self.store_solution(
            PlanningSolution(
                self.problem, policy, self.search_record(), trajectory, evaluation
            )
        )

    def search_record(self) -> "TreeSearchRecord":
        """The search's account: goal reached or not, the budget it took, the path's cost-to-come."""
        node = self.solution_node
        return TreeSearchRecord(
            reached_goal=bool(self.reached_goal),
            iterations=int(self.iterations),
            nodes=len(self.tree.nodes),
            cost=float("nan") if node is None else float(node.cost),
            converged=False,
            history=[],
        )

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

    def show_live_search(self, **kwargs):
        """Build a :class:`~minilink.planning.search.live_plot.LiveSearchPlotCallback`."""
        from minilink.planning.search.live_plot import LiveSearchPlotCallback

        return LiveSearchPlotCallback(self, **kwargs)

    # Internal machinery

    def _validate_nearest_backend(self) -> None:
        backend = self.options.nearest_backend
        if backend not in NEAREST_BACKENDS:
            raise ValueError(
                f"nearest_backend must be one of {NEAREST_BACKENDS}, got {backend!r}"
            )
        if backend == NEAREST_KD_TREE and self.metric is not euclidean:
            raise ValueError(
                "nearest_backend='kd_tree' requires metric=euclidean "
                "(Euclidean L2 only; use nearest_backend='brute_force' otherwise)"
            )

    def _resolve_search_callback(self):
        if self.options.callback is not None:
            return self.options.callback
        if not self.options.live_plot:
            return None

        from minilink.planning.search.live_plot import LiveSearchPlotCallback

        return LiveSearchPlotCallback(
            self,
            ax=self.options.live_plot_ax,
            every=self.options.live_plot_every,
            pause=self.options.live_plot_pause,
            after_goal_only=self.options.live_plot_after_goal_only,
        )

    def _on_search_step(self, *, phase: str) -> None:
        callback = getattr(self, "_search_callback", None)
        if callback is None:
            return
        if self.options.live_plot_after_goal_only and not self.reached_goal:
            return
        if self.iterations % max(1, self.options.live_plot_every) != 0:
            return

        from minilink.planning.search.live_plot import RRSearchIteration

        best_cost = None
        if self.reached_goal and self.solution_node is not None:
            best_cost = float(self.solution_node.cost)

        callback(
            RRSearchIteration(
                iteration=self.iterations,
                phase=phase,
                reached_goal=self.reached_goal,
                best_cost=best_cost,
                tree_nodes=len(self.tree.nodes),
                planner=self,
            )
        )

    def _path_states(self, node: Node) -> np.ndarray:
        chain = []
        current = node
        while current.parent is not None:
            chain.append(current)
            current = current.parent
        chain.reverse()

        states = [np.asarray(self.tree.root.x, dtype=float)]
        for nd in chain:
            edge = nd.edge
            for k in range(1, len(edge.states)):
                states.append(np.asarray(edge.states[k], dtype=float))
        return np.asarray(states)

    def _try_extend(self, from_node, toward, rng):
        return self._select(
            self.extender.propose(from_node.x, toward, self.problem, rng), toward
        )

    def _sample(self, rng):
        if self.problem.x_goal is not None and rng.random() < self.options.goal_bias:
            return np.asarray(self.problem.x_goal, dtype=float)
        return self._sample_free_state(rng)

    def _sample_free_state(self, rng):
        problem = self.problem
        set_params = problem.params.sets
        X = problem.X

        try:
            sample = X.sample(rng, params=set_params)
            if X.contains(sample, params=set_params):
                return np.asarray(sample, dtype=float)
        except NotImplementedError:
            pass

        for _ in range(self.options.max_sample_attempts):
            candidate = self._sample_box.sample(rng)
            if X.contains(candidate, params=set_params):
                return np.asarray(candidate, dtype=float)

        raise RuntimeError(
            f"Could not sample a collision-free state from X after "
            f"{self.options.max_sample_attempts} attempts"
        )

    def _select(self, candidates, x_rand):
        best, best_distance = None, np.inf
        for edge in candidates:
            if not self._edge_is_free(edge):
                continue
            distance = self.metric(edge.x_end, x_rand)
            if distance < best_distance:
                best, best_distance = edge, distance
        return best

    def _edge_states(self, edge) -> np.ndarray:
        resolution = self.options.edge_resolution
        if resolution is None:
            return edge.states

        states = np.asarray(edge.states, dtype=float)
        if states.shape[0] < 2:
            return states

        dense = [states[0]]
        for k in range(states.shape[0] - 1):
            x0 = states[k]
            x1 = states[k + 1]
            segment_length = float(np.linalg.norm(x1 - x0))
            if segment_length <= 1e-12:
                continue
            n_sub = max(1, int(np.ceil(segment_length / resolution)))
            for j in range(1, n_sub + 1):
                alpha = j / n_sub
                dense.append(x0 + alpha * (x1 - x0))
        return np.asarray(dense)

    def _edge_is_free(self, edge) -> bool:
        problem = self.problem
        set_params = problem.params.sets
        if problem.X is not None:
            if not all(
                problem.X.contains(state, params=set_params)
                for state in self._edge_states(edge)
            ):
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
