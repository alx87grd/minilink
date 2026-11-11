"""The RRT planner: a tree grown from the start until it reaches the goal region."""

from __future__ import annotations

import time
from copy import deepcopy
from dataclasses import dataclass
from typing import Callable

import numpy as np

from minilink.core.sets import SingletonSet, is_finite_box
from minilink.core.trajectory import Trajectory
from minilink.planning.planner import (
    Planner,
    open_loop_controller_of,
    planner_at,
    planner_evidence,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution, solve_time_text
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.metric import euclidean
from minilink.planning.search.tree import (
    NEAREST_KD_TREE,
    NEAREST_METHODS,
    Node,
    Tree,
)

###############################################################################
# Public API
###############################################################################


class RRTPlanner(Planner):
    """
    Rapidly-exploring random tree over a `PlanningProblem`.

    The planner owns the loop (sample, nearest, propose, select, add, goal
    test, extract) and reads every concern from the problem: collision from
    ``problem.X``, the goal region from ``problem.Xf`` (a region: a ball written as one, ``Xf=BallSet(goal, tol)``), sampling
    from ``problem.X`` (by rejection when needed), dynamics from
    ``problem.sys``. Only the ``extender`` (how two states connect) and the
    ``metric`` (the nearest-neighbour distance) are swappable.

    Parameters
    ----------
    problem : PlanningProblem
        Supplies dynamics, free space (``X``), the goal region (``Xf``), and bounds.
    extender : TrajectoryExtender
        Proposes candidate edges; the orchestrator selects the best collision-free one.
    metric : callable
        Pairwise nearest-neighbour distance ``metric(a, b) -> float``.
    Notes
    -----
    The search's settings are keywords, read back as attributes: ``max_iterations``,
    ``goal_bias``, ``seed``, ``edge_resolution``,
    ``max_sample_attempts``, ``return_best_effort``, ``nearest_method``.
    The workflow, ``callback`` (the live figure one of its hooks), is a keyword of :meth:`solve`.

    Provisional tool: maintainer review before assigning for coursework.

    Attributes
    ----------
    solution_node : Node or None
        Tree node whose parent chain defines the returned trajectory.
    """

    number_names = ("goal_bias", "edge_resolution")

    def __init__(
        self,
        problem: PlanningProblem,
        extender=None,
        *,
        metric=euclidean,
        max_iterations: int = 5000,
        goal_bias: float = 0.1,
        seed: int = 0,
        edge_resolution: float | None = None,
        max_sample_attempts: int = 100,
        return_best_effort: bool = True,
        nearest_method: str = "brute_force",
    ) -> None:
        super().__init__(problem)

        # The connector (by default bang-bang motion primitives from the input
        # bounds, 0.3 s edges) and the nearest-neighbour distance
        if extender is None:
            extender = KinodynamicExtender()
        self.extender = extender
        self.metric = metric

        # The search's structure and numbers
        self.max_iterations = int(max_iterations)
        self.goal_bias = float(goal_bias)
        self.seed = int(seed)
        self.edge_resolution = edge_resolution
        self.max_sample_attempts = int(max_sample_attempts)
        self.return_best_effort = bool(return_best_effort)
        self.nearest_method = nearest_method

        # The search, refreshed by every solve
        self.tree: Tree | None = None
        self.solution_node: Node | None = None

        # The box free states are drawn from by rejection: the box of X, or the
        # state bounds when X has no finite one
        box = problem.X.bounding_box()
        self.sample_box = problem.sys.state.box if not is_finite_box(box) else box

    def solve(
        self,
        *,
        params=None,
        evaluate=False,
        n_trials=50,
        verbose=False,
        callback: Callable | list[Callable] | None = None,
    ) -> PlanningSolution:
        """
        Grow the tree until the goal region is reached or the budget is spent.

        The solution's controller is the path's input held by a source block,
        ``controller >> plant``; ``evaluate=True`` replays it over the problem's
        draws and horizon (the path's last input held after it ends) when the
        problem declares a cost; the record's "goal reached" is the search's.
        ``params`` (the complete tree of :attr:`params`, a copy edited) solves
        the planner rebuilt there (:func:`~minilink.planning.planner.planner_at`), this one unchanged.

        The workflow: ``callback`` (a callable or a list of callables) sees
        every successful extension as an
        :class:`~minilink.planning.search.live_plot.TreeSearchIteration`; each
        hook filters for itself. The live figure is one such hook,
        ``callback=LiveSearchPlot(every=10)``
        (:class:`~minilink.planning.search.live_plot.LiveSearchPlot`);
        ``verbose`` prints the record at the end.
        """
        # At another tree: the planner there, its solution kept here
        if params is not None:
            solution = planner_at(self, params).solve(
                evaluate=evaluate, n_trials=n_trials, verbose=verbose, callback=callback
            )
            return self.store_solution(solution)

        metric = self.metric
        self.require_nearest_method()
        started = time.perf_counter()

        rng = np.random.default_rng(self.seed)
        goal_region = self.goal_region()
        x0 = np.asarray(self.problem.x0, dtype=float)

        # The tree, rooted at the start
        self.tree = Tree(
            Node(x=x0, parent=None, edge=None, cost_to_come=0.0),
            nearest_method=self.nearest_method,
        )
        self.solution_node = None
        report = self.build_search_report(callback)

        # The extensions so far, and whether one reached the goal: the record's counts
        n_iterations = 0
        reached_goal = False
        for _ in range(self.max_iterations):
            # Sample a state, and extend the nearest node toward it
            x_rand = self.sample_state(rng)
            near = self.tree.nearest(x_rand, metric)
            edge = self.try_extend(near, x_rand, rng)
            if edge is None:
                continue

            # The new node, with its cost-to-come
            cost = near.cost_to_come + edge.cost
            node = self.tree.add(
                Node(x=edge.x_end, parent=near, edge=edge, cost_to_come=cost)
            )
            n_iterations += 1

            # Stop at the first node inside the goal region
            if goal_region.contains(node.x):
                reached_goal = True
                self.solution_node = node
                report("explore", n_iterations, reached_goal)
                return self.finish_trajectory(
                    self.tree.extract_trajectory(node, self.problem.t0),
                    evaluate,
                    n_trials,
                    self.search_record(
                        reached_goal, n_iterations, time.perf_counter() - started
                    ),
                    verbose=verbose,
                )

            report("explore", n_iterations, reached_goal)

        # The budget is spent: the node closest to the goal, when best effort is allowed
        goal_point = self.goal_point(goal_region)
        closest = self.tree.nearest(goal_point, metric)
        self.solution_node = closest
        if not self.return_best_effort:
            raise RuntimeError("RRT failed to reach goal within max_iterations")

        return self.finish_trajectory(
            self.tree.extract_trajectory(closest, self.problem.t0),
            evaluate,
            n_trials,
            self.search_record(
                reached_goal, n_iterations, time.perf_counter() - started
            ),
            verbose=verbose,
        )

    def build_kwargs(self) -> dict:
        """The keywords that rebuild this planner on another problem: the extender and metric, the search's settings and numbers."""
        kwargs = dict(extender=self.extender, metric=self.metric)
        kwargs["max_iterations"] = self.max_iterations
        kwargs["goal_bias"] = self.goal_bias
        kwargs["seed"] = self.seed
        kwargs["edge_resolution"] = self.edge_resolution
        kwargs["max_sample_attempts"] = self.max_sample_attempts
        kwargs["return_best_effort"] = self.return_best_effort
        kwargs["nearest_method"] = self.nearest_method

        return kwargs

    def search_record(
        self, reached_goal, n_iterations, solve_time_s=None
    ) -> TreeSearchRecord:
        """The search's account: goal reached or not, the budget it took (``n_iterations`` extensions, ``solve_time_s`` seconds), the path's cost-to-come."""
        node = self.solution_node

        return TreeSearchRecord(
            reached_goal=bool(reached_goal),
            n_iterations=int(n_iterations),
            n_nodes=len(self.tree.nodes),
            path_cost=float("nan") if node is None else float(node.cost_to_come),
            converged=False,
            history=[],
            solve_time_s=solve_time_s,
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

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_nearest_method(self) -> None:
        """Refuse an unknown nearest-neighbour method, and the k-d tree under a non-Euclidean metric."""
        method = self.nearest_method
        if method not in NEAREST_METHODS:
            raise ValueError(
                f"nearest_method must be one of {NEAREST_METHODS}, got {method!r}"
            )
        if method == NEAREST_KD_TREE and self.metric is not euclidean:
            raise ValueError(
                "nearest_method='kd_tree' requires metric=euclidean "
                "(Euclidean L2 only; use nearest_method='brute_force' otherwise)"
            )

    def goal_region(self):
        """The goal region: ``problem.Xf``, refused unless a region."""
        Xf = self.problem.Xf
        if Xf is None or isinstance(Xf, SingletonSet):
            raise ValueError(
                "RRT reaches a goal region: give the problem Xf as a region, a ball "
                "written as one (Xf=BallSet(goal, tolerance))"
            )

        return Xf

    def build_search_report(self, callback=None):
        """
        The solve's report of one extension, ``report(phase, n_iterations, reached_goal)``: the search as it stands, handed to every hook.

        ``callback`` is a callable or a list of callables; each filters for
        itself. A no-op when the solve asks for none.
        """
        if callback is None:
            hooks = []
        elif isinstance(callback, (list, tuple)):
            hooks = list(callback)
        else:
            hooks = [callback]

        def report(phase, n_iterations, reached_goal):
            if not hooks:
                return
            step = self.search_step(phase, n_iterations, reached_goal)
            for hook in hooks:
                hook(step)

        return report

    def sample_state(self, rng):
        """The goal region's point with probability ``goal_bias``, otherwise a free state."""
        x_goal = self.problem.goal_point()
        goal_bias = self.goal_bias

        if rng.random() < goal_bias:
            return x_goal

        return self.sample_free_state(rng)

    def sample_free_state(self, rng):
        """A state of ``X``: drawn from ``X`` itself, or by rejection from the sampling box."""
        problem = self.problem
        X = problem.X

        # A set that samples itself, when its draw lands in it
        try:
            sample = X.sample(rng)
            if X.contains(sample):
                return np.asarray(sample, dtype=float)
        except NotImplementedError:
            pass

        # Otherwise by rejection: at most max_sample_attempts draws from the box
        for _ in range(self.max_sample_attempts):
            candidate = self.sample_box.sample(rng)
            if X.contains(candidate):
                return np.asarray(candidate, dtype=float)

        raise RuntimeError(
            f"Could not sample a collision-free state from X after "
            f"{self.max_sample_attempts} attempts"
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

        if problem.X is not None:
            if not all(problem.X.contains(state) for state in self.edge_states(edge)):
                return False

        # Each input at the state it is applied from: u_k ∈ U(x_k)
        if problem.U is not None:
            if not all(
                problem.U.contains(u, x) for x, u in zip(edge.states[:-1], edge.inputs)
            ):
                return False

        return True

    def edge_states(self, edge) -> np.ndarray:
        """The edge's states, densified to ``edge_resolution`` when it is set."""
        resolution = self.edge_resolution
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

    def search_step(self, phase: str, n_iterations: int, reached_goal: bool):
        """The search as it stands after ``n_iterations`` extensions, one :class:`~minilink.planning.search.live_plot.TreeSearchIteration` for the solve's hooks."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.search.live_plot import TreeSearchIteration

        best_cost_to_come = None
        if reached_goal and self.solution_node is not None:
            best_cost_to_come = float(self.solution_node.cost_to_come)

        return TreeSearchIteration(
            iteration=n_iterations,
            phase=phase,
            reached_goal=reached_goal,
            best_cost_to_come=best_cost_to_come,
            n_nodes=len(self.tree.nodes),
            planner=self,
        )

    def finish_trajectory(
        self, trajectory: Trajectory, evaluate, n_trials, record, *, verbose=False
    ) -> PlanningSolution:
        """The path as a solution: its input held by a source block, the search's ``record`` (printed when ``verbose``), scored when asked."""
        controller = open_loop_controller_of(trajectory, self.problem.sys, hold="zoh")

        # Scored over the problem's draws and its horizon (the path's last input
        # held after it ends), on the path's median time step
        evaluation = None
        if evaluate:
            dt = float(np.median(np.diff(trajectory.t)))
            evaluation = planner_evidence(self, controller, dt=dt, n_trials=n_trials)

        solution = PlanningSolution(
            problem=self.problem,
            controller=controller,
            record=record,
            trajectory=trajectory,
            evaluation=evaluation,
            params=deepcopy(self.params),
        )
        if verbose:
            print(record)

        return self.store_solution(solution)

    def goal_point(self, goal_region):
        """The goal state the best effort aims at: the problem's goal point (``Xf``'s)."""
        return self.problem.goal_point()

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


@dataclass(frozen=True)
class TreeSearchRecord:
    """Whether the goal was reached, how much of the budget and of the wall clock (``solve_time_s``) the search took, and the path's cost-to-come, ``path_cost``."""

    reached_goal: bool
    n_iterations: int
    n_nodes: int
    path_cost: float
    converged: bool
    history: list
    solve_time_s: float | None = None

    @property
    def success(self) -> bool:
        """The search reached the goal region."""
        return bool(self.reached_goal)

    # Display

    def __str__(self) -> str:
        outcome = "goal reached" if self.reached_goal else "goal not reached"
        text = (
            f"{outcome} after {self.n_iterations} extensions ({self.n_nodes} nodes), "
            f"path cost {self.path_cost:.4g}"
        )
        if self.converged:
            text += ", converged"
        return text + solve_time_text(self.solve_time_s)


###############################################################################
# Internal machinery
###############################################################################
