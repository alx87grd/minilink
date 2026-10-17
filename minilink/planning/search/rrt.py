"""The RRT planner: a tree grown from the start until it reaches the goal region."""

from __future__ import annotations

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

_UNSET = object()  # Marks a keyword the caller did not pass


###############################################################################
# Public API
###############################################################################


class RRTPlanner(Planner):
    """
    Rapidly-exploring random tree over a `PlanningProblem`.

    The planner owns the loop (sample, nearest, propose, select, add, goal
    test, extract) and reads every concern from the problem: collision from
    ``problem.X``, the goal region from ``problem.Xf`` / ``x_goal``, sampling
    from ``problem.X`` (by rejection when needed), dynamics from
    ``problem.sys``. Only the ``extender`` (how two states connect) and the
    ``metric`` (the nearest-neighbour distance) are swappable.

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

    Provisional tool: maintainer review before assigning for coursework.

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

        # The connector (by default bang-bang motion primitives from the input
        # bounds, 0.3 s edges) and the nearest-neighbour distance
        if extender is None:
            extender = KinodynamicExtender()
        self.extender = extender
        self.metric = metric

        # Workflow options: the flat keywords over the bag
        self.options = rrt_options_of(
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

        # The search, refreshed by every solve
        self.tree: Tree | None = None
        self.reached_goal: bool = False
        self.solution_node: Node | None = None
        self.iterations: int = 0
        self.search_callback = None  # The solve's step callback, when it has one

        # The box free states are drawn from by rejection: the box of X, or the
        # state bounds when X has no finite one
        box = problem.X.bounding_box()
        self.sample_box = problem.sys.state.box if not is_finite_box(box) else box

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
        metric = self.metric
        self.require_nearest_backend()

        rng = np.random.default_rng(options.seed)
        goal_region = self.goal_region()
        x_start = np.asarray(self.problem.x_start, dtype=float)

        # The tree, rooted at the start
        self.tree = Tree(
            Node(x=x_start, parent=None, edge=None, cost=0.0),
            nearest_backend=options.nearest_backend,
        )
        self.reached_goal = False
        self.solution_node = None
        self.iterations = 0
        self.search_callback = self.resolve_search_callback()

        for _ in range(options.max_nodes):
            # Sample a state, and extend the nearest node toward it
            x_rand = self.sample_state(rng)
            near = self.tree.nearest(x_rand, metric)
            edge = self.try_extend(near, x_rand, rng)
            if edge is None:
                continue

            # The new node, with its cost-to-come
            cost = near.cost + edge.cost
            node = self.tree.add(Node(x=edge.x_end, parent=near, edge=edge, cost=cost))
            self.iterations += 1

            # Stop at the first node inside the goal region
            if goal_region.contains(node.x):
                self.reached_goal = True
                self.solution_node = node
                self.on_search_step(phase="explore")
                return self.finish_trajectory(
                    self.tree.extract_trajectory(node), evaluate, n_trials
                )

            self.on_search_step(phase="explore")

        # The budget is spent: the node closest to the goal, when best effort is allowed
        goal_point = self.goal_point(goal_region)
        closest = self.tree.nearest(goal_point, metric)
        self.solution_node = closest
        if not options.return_best_effort:
            raise RuntimeError("RRT failed to reach goal within max_nodes")

        return self.finish_trajectory(
            self.tree.extract_trajectory(closest), evaluate, n_trials
        )

    def search_record(self) -> TreeSearchRecord:
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
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.plotting import plot_tree

        return plot_tree(self, **kwargs)

    def animate_search(self, **kwargs):
        """Animate the tree growth in a state projection (lazy matplotlib import)."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.plotting import animate_search

        return animate_search(self, **kwargs)

    def show_live_search(self, **kwargs):
        """Build a :class:`~minilink.planning.search.live_plot.LiveSearchPlotCallback`."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.live_plot import LiveSearchPlotCallback

        return LiveSearchPlotCallback(self, **kwargs)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_nearest_backend(self) -> None:
        """Refuse an unknown backend, and the k-d tree under a non-Euclidean metric."""
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

    def goal_region(self):
        """The goal region: ``problem.Xf``, or a ball of ``goal_tolerance`` about ``x_goal``."""
        Xf = self.problem.Xf
        if Xf is not None and not isinstance(Xf, SingletonSet):
            return Xf
        if self.problem.x_goal is None:
            raise ValueError("RRT requires a goal: set x_goal or a region Xf")
        return BallSet(self.problem.x_goal, self.options.goal_tolerance)

    def resolve_search_callback(self):
        """The solve's step callback: the option's, a live plot, or ``None``."""
        if self.options.callback is not None:
            return self.options.callback
        if not self.options.live_plot:
            return None

        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.live_plot import LiveSearchPlotCallback

        return LiveSearchPlotCallback(
            self,
            ax=self.options.live_plot_ax,
            every=self.options.live_plot_every,
            pause=self.options.live_plot_pause,
            after_goal_only=self.options.live_plot_after_goal_only,
        )

    def sample_state(self, rng):
        """The goal with probability ``goal_bias``, otherwise a free state."""
        x_goal = self.problem.x_goal
        goal_bias = self.options.goal_bias

        if x_goal is not None and rng.random() < goal_bias:
            return np.asarray(x_goal, dtype=float)

        return self.sample_free_state(rng)

    def sample_free_state(self, rng):
        """A state of ``X``: drawn from ``X`` itself, or by rejection from the sampling box."""
        problem = self.problem
        set_params = problem.params.sets
        X = problem.X

        # A set that samples itself, when its draw lands in it
        try:
            sample = X.sample(rng, params=set_params)
            if X.contains(sample, params=set_params):
                return np.asarray(sample, dtype=float)
        except NotImplementedError:
            pass

        # Otherwise by rejection: at most max_sample_attempts draws from the box
        for _ in range(self.options.max_sample_attempts):
            candidate = self.sample_box.sample(rng)
            if X.contains(candidate, params=set_params):
                return np.asarray(candidate, dtype=float)

        raise RuntimeError(
            f"Could not sample a collision-free state from X after "
            f"{self.options.max_sample_attempts} attempts"
        )

    def try_extend(self, from_node, toward, rng):
        """The best collision-free edge the extender proposes from ``from_node`` toward ``toward``."""
        candidates = self.extender.propose(from_node.x, toward, self.problem, rng)
        edge = self.nearest_free_edge(candidates, toward)

        return edge

    def nearest_free_edge(self, candidates, x_rand):
        """The collision-free candidate whose end is nearest ``x_rand`` (``None`` when all collide)."""
        metric = self.metric

        best = None
        best_distance = np.inf
        for edge in candidates:
            if not self.edge_is_free(edge):
                continue
            distance = metric(edge.x_end, x_rand)
            if distance < best_distance:
                best = edge
                best_distance = distance

        return best

    def edge_is_free(self, edge) -> bool:
        """Every state of the edge in ``X`` and every input in ``U``."""
        problem = self.problem
        set_params = problem.params.sets

        if problem.X is not None:
            if not all(
                problem.X.contains(state, params=set_params)
                for state in self.edge_states(edge)
            ):
                return False

        if problem.U is not None:
            if not all(problem.U.contains(u) for u in edge.inputs):
                return False

        return True

    def edge_states(self, edge) -> np.ndarray:
        """The edge's states, densified to ``edge_resolution`` when it is set."""
        resolution = self.options.edge_resolution
        if resolution is None:
            return edge.states

        states = np.asarray(edge.states, dtype=float)
        if states.shape[0] < 2:
            return states

        # Each segment cut into steps no longer than the resolution
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

    def on_search_step(self, *, phase: str) -> None:
        """Hand the step to the solve's callback, every ``live_plot_every`` extensions."""
        callback = self.search_callback
        if callback is None:
            return
        if self.options.live_plot_after_goal_only and not self.reached_goal:
            return
        if self.iterations % max(1, self.options.live_plot_every) != 0:
            return

        # Lazy import: the graphics load on first use, not with the core
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

    def finish_trajectory(
        self, trajectory: Trajectory, evaluate, n_trials
    ) -> PlanningSolution:
        """The path as a solution: its input held by a source block, the search's record, scored when asked."""
        policy = self.open_loop_policy(trajectory, interpolation="previous")

        # Scored over the problem's draws, on the path's median time step
        evaluation = None
        if evaluate:
            dt = float(np.median(np.diff(trajectory.t)))
            evaluation = self.evaluate(
                policy, dt=dt, n_trials=n_trials, tf=float(trajectory.tf)
            )

        solution = PlanningSolution(
            self.problem, policy, self.search_record(), trajectory, evaluation
        )

        return self.store_solution(solution)

    def goal_point(self, goal_region):
        """The goal state the best effort aims at: ``x_goal``, or the region's center."""
        if self.problem.x_goal is not None:
            return np.asarray(self.problem.x_goal, dtype=float)
        center = getattr(goal_region, "center", None)
        if center is None:
            raise ValueError(
                "RRT goal has no representative point for best-effort fallback"
            )
        return np.asarray(center, dtype=float)

    def path_states(self, node: Node) -> np.ndarray:
        """The state samples of every edge from the root to ``node``, shape ``(k+1, n)``."""
        # The nodes from the root's child down to node
        chain = []
        current = node
        while current.parent is not None:
            chain.append(current)
            current = current.parent
        chain.reverse()

        # The root's state, then each edge's samples after its first (its parent's state)
        states = [np.asarray(self.tree.root.x, dtype=float)]
        for nd in chain:
            edge = nd.edge
            for k in range(1, len(edge.states)):
                states.append(np.asarray(edge.states[k], dtype=float))

        return np.asarray(states)


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


#: The flat keywords :class:`RRTPlanner` overlays on :class:`RRTOptions`.
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
        """The search reached the goal region."""
        return bool(self.reached_goal)

    # Display

    def __str__(self) -> str:
        outcome = "goal reached" if self.reached_goal else "goal not reached"
        text = (
            f"{outcome} after {self.iterations} extensions ({self.nodes} nodes), "
            f"path cost {self.cost:.4g}"
        )
        if self.converged:
            text += ", converged"
        return text


###############################################################################
# Internal machinery
###############################################################################


def rrt_options_of(
    options: RRTOptions | None,
    *,
    default_factory=RRTOptions,
    allowed_keys=_RRT_OPTION_KEYS,
    **flat,
) -> RRTOptions:
    """The options bag (or a default one) with the passed flat keywords laid over it."""
    base = default_factory() if options is None else options
    updates = {key: value for key, value in flat.items() if value is not _UNSET}
    unknown = sorted(k for k in updates if k not in allowed_keys)
    if unknown:
        raise ValueError(f"Unknown RRT planner kwargs: {unknown}.")
    if not updates:
        return base
    return replace(base, **updates)
