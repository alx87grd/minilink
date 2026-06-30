"""Pyro vs Minilink parity benchmarks for DP and RRT.

Each case mirrors a pyro demo (grid sizes, costs, seeds) and reports success,
precision (goal error, cost-to-go at start), and wall time split into build /
solve / closed-loop simulation.

Pyro requires numpy < 2 (``dev`` conda env). Minilink JAX runs in ``dev-h26``.
"""

from __future__ import annotations

import contextlib
import io
import json
import time
from dataclasses import asdict, dataclass, field

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.core.diagram import DiagramSystem
from minilink.core.sets import BallSet
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.edge import Edge
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner

INF_PENDULUM = 500.0
INF_DOUBLE = 1000.0
UPRIGHT_PYRO = np.array([-np.pi, 0.0])
GOAL_DOUBLE = np.zeros(4)
HANGING = np.array([-np.pi, 1.0, 0.0, 0.0])

CLOSED_LOOP_STARTS = {
    "pendulum": [np.array([0.0, 0.0])],
    "double_pendulum": [
        HANGING.copy(),
        np.array([-1.0, -1.0, 0.0, 0.0]),
        np.array([-1.0, 1.0, 0.0, 0.0]),
    ],
}


@dataclass
class ParityRow:
    """One timed parity run."""

    case: str
    framework: str
    backend: str
    success: bool
    build_s: float
    solve_s: float
    sim_s: float
    total_s: float
    j_at_start: float | None = None
    goal_error: float | None = None
    max_dj_vs_pyro: float | None = None
    nodes: int | None = None
    iterations: int | None = None
    path_time_s: float | None = None
    notes: str = ""
    extras: dict = field(default_factory=dict)


def _goal_error(x_final: np.ndarray, x_goal: np.ndarray) -> float:
    err = x_final - x_goal
    if err.size >= 1:
        err[0] = _wrap_angle(err[0])
    return float(np.linalg.norm(err))


def _wrap_angle(theta: float) -> float:
    return float((theta + np.pi) % (2.0 * np.pi) - np.pi)


def _quiet():
    return contextlib.redirect_stdout(io.StringIO())


class EulerKinodynamicExtender(KinodynamicExtender):
    """Kinodynamic extender using Euler steps (pyro ``x_next`` convention)."""

    def _rollout(self, evaluator, from_state, u) -> Edge:
        x = np.asarray(from_state, dtype=float)
        states = [x]
        for _ in range(self.n_substeps):
            x = np.asarray(evaluator.euler_step(x, u, 0.0, self.dt), dtype=float)
            states.append(x)
        return Edge(
            states=np.asarray(states),
            inputs=np.tile(u, (self.n_substeps, 1)),
            times=np.arange(self.n_substeps + 1) * self.dt,
            cost=self.horizon,
        )


def _minilink_closed_loop(plant, controller, x0, *, tf, n_steps, solver="euler"):
    plant.x0 = np.asarray(x0, dtype=float)
    diagram = DiagramSystem()
    diagram.add_subsystem(controller, "controller")
    diagram.add_subsystem(plant, "plant")
    diagram.connect("plant", "y", "controller", "x")
    diagram.connect("controller", "u", "plant", "u")
    return diagram.compute_trajectory(tf=tf, n_steps=n_steps, solver=solver)


def _pendulum_problem(*, bounds=10.0):
    sys = Pendulum()
    sys.state.lower_bound = np.array([-bounds, -bounds])
    sys.state.upper_bound = np.array([bounds, bounds])
    sys.inputs["u"].lower_bound = np.array([-5.0])
    sys.inputs["u"].upper_bound = np.array([5.0])
    cost = QuadraticCost.from_system(
        sys,
        xbar=UPRIGHT_PYRO,
        Q=np.eye(2),
        R=np.array([[1.0]]),
        S=np.diag([10.0, 10.0]),
    )
    return PlanningProblem(sys, x_goal=UPRIGHT_PYRO, cost=cost)


def _double_pendulum_problem():
    plant = DoublePendulum()
    plant.state.lower_bound = np.array([-5.0, -1.5, -4.0, -4.0])
    plant.state.upper_bound = np.array([0.5, 4.0, 5.5, 7.0])
    plant.inputs["u"].lower_bound = np.array([-12.0, -12.0])
    plant.inputs["u"].upper_bound = np.array([12.0, 12.0])
    Q = np.diag([1.0, 0.5, 0.1, 0.05])
    R = np.diag([0.05, 0.05])
    cost = QuadraticCost.from_system(plant, xbar=GOAL_DOUBLE, Q=Q, R=R, S=Q)
    return PlanningProblem(plant, x_goal=GOAL_DOUBLE, cost=cost)


def _sample_j_on_grid(grid, J: np.ndarray, n_sample: int = 200, rng=None):
    """Sample feasible node indices and return (states, values)."""
    rng = np.random.default_rng(0) if rng is None else rng
    feasible = np.flatnonzero(J < 0.9 * INF_PENDULUM)
    if feasible.size == 0:
        feasible = np.arange(min(n_sample, J.size))
    picks = rng.choice(feasible, size=min(n_sample, feasible.size), replace=False)
    states = grid.states[picks]
    values = J.ravel()[picks]
    return states, values


def compare_j_fields(grid, J_ref, J_other, *, inf=INF_PENDULUM, n_sample=300):
    """Max |J_ref - J_other| on a random feasible sample (same grid layout)."""
    states, j_ref = _sample_j_on_grid(grid, J_ref, n_sample=n_sample)
    j_other = np.array(
        [float(grid.interpolate(J_other, s[None, :])[0]) for s in states]
    )
    mask = (j_ref < 0.9 * inf) & (j_other < 0.9 * inf)
    if not np.any(mask):
        return float("nan")
    return float(np.max(np.abs(j_ref[mask] - j_other[mask])))


def run_minilink_pendulum_dp(
    *,
    x_grid=(201, 201),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    n_steps=2000,
    backend="numpy",
    fast_solve=False,
) -> ParityRow:
    """Pendulum VI — pyro ``pendulum_optimal_swingup_demo`` settings."""
    problem = _pendulum_problem()
    x0 = CLOSED_LOOP_STARTS["pendulum"][0]

    t0 = time.perf_counter()
    grid = StateSpaceGrid(
        problem,
        x_grid_shape=x_grid,
        u_grid_shape=u_grid,
        dt=dt,
        precompute=(backend != "loop"),
        verbose=False,
    )
    planner = DynamicProgrammingPlanner(
        problem,
        grid=grid,
        options=DynamicProgrammingOptions(
            backend=backend,
            alpha=1.0,
            tol=tol,
            max_iterations=n_steps,
            out_of_bound_cost=INF_PENDULUM,
            verbose=False,
        ),
    )
    t_build = time.perf_counter()
    with _quiet():
        if fast_solve:
            result = planner.solve_steps(min(n_steps, 50))
        else:
            result = planner.compute_solution()
        planner.clean_infeasible_set()
    t_solve = time.perf_counter()

    controller = result.controller()
    t_sim0 = time.perf_counter()
    traj = _minilink_closed_loop(
        problem.sys, controller, x0, tf=10.0, n_steps=10001, solver="euler"
    )
    t_sim = time.perf_counter()
    goal_err = _goal_error(traj.x[:, -1], UPRIGHT_PYRO)
    j_start = result.value_at(x0)

    return ParityRow(
        case="pendulum_dp",
        framework="minilink",
        backend=backend,
        success=goal_err < 0.5,
        build_s=t_build - t0,
        solve_s=t_solve - t_build,
        sim_s=t_sim - t_sim0,
        total_s=t_sim - t0,
        j_at_start=j_start,
        goal_error=goal_err,
        iterations=result.iterations,
        notes=f"grid {x_grid}x{u_grid} tol={tol}",
    )


def run_minilink_double_pendulum_dp(
    *,
    x_grid=(51, 41, 51, 41),
    u_grid=(5, 5),
    dt=0.1,
    n_steps=50,
    backend="jax",
) -> ParityRow:
    """Double-pendulum VI — pyro ``double_pendulum_optimal_swingup`` settings."""
    problem = _double_pendulum_problem()
    x0 = HANGING

    t0 = time.perf_counter()
    grid = StateSpaceGrid(
        problem,
        x_grid_shape=x_grid,
        u_grid_shape=u_grid,
        dt=dt,
        precompute=False,
        verbose=False,
    )
    planner = DynamicProgrammingPlanner(
        problem,
        grid=grid,
        options=DynamicProgrammingOptions(
            backend=backend,
            alpha=1.0,
            max_iterations=n_steps,
            out_of_bound_cost=INF_DOUBLE,
            verbose=False,
        ),
    )
    t_build = time.perf_counter()
    with _quiet():
        result = planner.solve_steps(n_steps)
        planner.clean_infeasible_set()
    t_solve = time.perf_counter()

    controller = result.controller()
    t_sim0 = time.perf_counter()
    traj = _minilink_closed_loop(
        problem.sys, controller, x0, tf=8.0, n_steps=4001, solver="euler"
    )
    t_sim = time.perf_counter()
    goal_err = _goal_error(traj.x[:, -1], GOAL_DOUBLE)

    return ParityRow(
        case="double_pendulum_dp",
        framework="minilink",
        backend=backend,
        success=goal_err < 0.5,
        build_s=t_build - t0,
        solve_s=t_solve - t_build,
        sim_s=t_sim - t_sim0,
        total_s=t_sim - t0,
        j_at_start=result.value_at(x0),
        goal_error=goal_err,
        iterations=result.iterations,
        notes=f"grid {x_grid}x{u_grid} steps={n_steps}",
    )


def run_minilink_pendulum_rrt(*, seed=0, max_nodes=20000) -> ParityRow:
    """Pendulum kinodynamic RRT — aligned with pyro ``simple_pendulum_with_rrt``."""
    sys = Pendulum()
    sys.state.lower_bound = np.array([-2.0 * np.pi, -12.0])
    sys.state.upper_bound = np.array([2.0 * np.pi, 12.0])
    sys.inputs["u"].lower_bound = np.array([-5.0])
    sys.inputs["u"].upper_bound = np.array([5.0])

    x_start = np.array([0.1, 0.0])
    x_goal = UPRIGHT_PYRO.copy()
    torques = [np.array([tau]) for tau in (-5.0, -3.0, -1.0, 0.0, 1.0, 3.0, 5.0)]
    problem = PlanningProblem(
        sys=sys,
        x_start=x_start,
        x_goal=x_goal,
        Xf=BallSet(x_goal, 0.2),
    )
    extender = EulerKinodynamicExtender(controls=torques, horizon=0.1, n_substeps=1)

    t0 = time.perf_counter()
    planner = RRTPlanner(
        problem,
        extender=extender,
        options=RRTOptions(
            seed=seed,
            goal_bias=0.1,
            max_nodes=max_nodes,
            goal_tolerance=0.2,
            nearest_backend="kd_tree",
        ),
    )
    traj = planner.compute_solution()
    elapsed = time.perf_counter() - t0
    goal_err = _goal_error(traj.x[:, -1], x_goal)

    return ParityRow(
        case="pendulum_rrt",
        framework="minilink",
        backend="kd_tree+euler",
        success=planner.reached_goal,
        build_s=0.0,
        solve_s=elapsed,
        sim_s=0.0,
        total_s=elapsed,
        goal_error=goal_err,
        nodes=len(planner.tree.nodes),
        path_time_s=float(traj.t[-1]),
        notes=f"seed={seed} max_nodes={max_nodes}",
    )


# ---------------------------------------------------------------------------
# Pyro runners (import pyro — requires numpy < 2)
# ---------------------------------------------------------------------------


def _pyro_available() -> bool:
    try:
        import pyro  # noqa: F401

        return True
    except Exception:
        return False


def _pyro_local_ok() -> bool:
    """Pyro grid precompute breaks on numpy 2.x; use subprocess otherwise."""
    if not _pyro_available():
        return False
    import numpy as np

    return int(np.__version__.split(".")[0]) < 2


def run_pyro_pendulum_dp(
    *,
    x_grid=(201, 201),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    n_steps=50,
    use_fixed_steps=False,
) -> ParityRow:
    from pyro.analysis import costfunction
    from pyro.dynamic import pendulum
    from pyro.planning import discretizer, dynamicprogramming

    sys_p = pendulum.SinglePendulum()
    sys_p.x_ub = np.array([10.0, 10.0])
    sys_p.x_lb = np.array([-10.0, -10.0])
    x0 = CLOSED_LOOP_STARTS["pendulum"][0]

    t0 = time.perf_counter()
    with _quiet():
        grid = discretizer.GridDynamicSystem(sys_p, list(x_grid), list(u_grid), dt=dt)
    t_build = time.perf_counter()
    qcf = costfunction.QuadraticCostFunction.from_sys(sys_p)
    qcf.xbar = UPRIGHT_PYRO.copy()
    qcf.INF = INF_PENDULUM
    qcf.R[0, 0] = 1.0
    qcf.S[0, 0] = 10.0
    qcf.S[1, 1] = 10.0

    with _quiet():
        dp = dynamicprogramming.DynamicProgrammingWithLookUpTable(grid, qcf)
        if use_fixed_steps:
            dp.compute_steps(n_steps)
        else:
            dp.solve_bellman_equation(tol=tol)
        dp.clean_infeasible_set()
    t_solve = time.perf_counter()

    ctl = dp.get_lookup_table_controller()
    cl = ctl + sys_p
    cl.x0 = x0.copy()
    t_sim0 = time.perf_counter()
    with _quiet():
        cl.compute_trajectory(10, 10001, "euler")
    t_sim = time.perf_counter()
    final = cl.traj.x[-1, :]
    goal_err = _goal_error(final, UPRIGHT_PYRO)
    j_start = float(dp.J_interpol(x0))

    return ParityRow(
        case="pendulum_dp",
        framework="pyro",
        backend="lookup",
        success=goal_err < 0.5,
        build_s=t_build - t0,
        solve_s=t_solve - t_build,
        sim_s=t_sim - t_sim0,
        total_s=t_sim - t0,
        j_at_start=j_start,
        goal_error=goal_err,
        iterations=dp.k,
        notes=f"grid {x_grid}x{u_grid} tol={tol}",
    )


def run_pyro_double_pendulum_dp(
    *,
    x_grid=(51, 41, 51, 41),
    u_grid=(5, 5),
    dt=0.1,
    n_steps=50,
) -> ParityRow:
    from pyro.analysis import costfunction
    from pyro.dynamic import pendulum
    from pyro.planning import discretizer, dynamicprogramming

    sys_p = pendulum.DoublePendulum()
    sys_p.x_ub = np.array([0.5, 4.0, 5.5, 7.0])
    sys_p.x_lb = np.array([-5.0, -1.5, -4.0, -4.0])
    sys_p.u_ub = np.array([12.0, 12.0])
    sys_p.u_lb = np.array([-12.0, -12.0])
    x0 = HANGING

    t0 = time.perf_counter()
    with _quiet():
        grid = discretizer.GridDynamicSystem(sys_p, list(x_grid), list(u_grid), dt=dt)
    t_build = time.perf_counter()
    qcf = costfunction.QuadraticCostFunction.from_sys(sys_p)
    qcf.xbar = GOAL_DOUBLE.copy()
    qcf.Q[0, 0] = 1.0
    qcf.Q[1, 1] = 0.5
    qcf.Q[2, 2] = 0.1
    qcf.Q[3, 3] = 0.05
    qcf.R[0, 0] = 0.05
    qcf.R[1, 1] = 0.05
    qcf.INF = INF_DOUBLE
    qcf.EPS = 1.0

    with _quiet():
        dp = dynamicprogramming.DynamicProgrammingWithLookUpTable(grid, qcf)
        dp.compute_steps(n_steps)
        dp.clean_infeasible_set()
    t_solve = time.perf_counter()

    ctl = dp.get_lookup_table_controller()
    cl = ctl + sys_p
    cl.x0 = x0.copy()
    t_sim0 = time.perf_counter()
    with _quiet():
        cl.compute_trajectory(8, 4001, "euler")
    t_sim = time.perf_counter()
    final = cl.traj.x[-1, :]
    goal_err = _goal_error(final, GOAL_DOUBLE)
    j_start = float(dp.J_interpol(x0))

    return ParityRow(
        case="double_pendulum_dp",
        framework="pyro",
        backend="lookup",
        success=goal_err < 0.5,
        build_s=t_build - t0,
        solve_s=t_solve - t_build,
        sim_s=t_sim - t_sim0,
        total_s=t_sim - t0,
        j_at_start=j_start,
        goal_error=goal_err,
        iterations=n_steps,
        notes=f"grid {x_grid}x{u_grid} steps={n_steps}",
    )


def run_pyro_pendulum_rrt(*, seed=0, max_nodes=20000) -> ParityRow:
    from pyro.dynamic import pendulum
    from pyro.planning import randomtree

    np.random.seed(seed)
    sys_p = pendulum.SinglePendulum()
    x_start = np.array([0.1, 0.0])
    x_goal = UPRIGHT_PYRO.copy()

    planner = randomtree.RRT(sys_p, x_start)
    planner.u_options = [
        np.array([tau]) for tau in (-5.0, -3.0, -1.0, 0.0, 1.0, 3.0, 5.0)
    ]
    planner.goal_radius = 0.2
    planner.alpha = 0.9
    planner.beta = 0.0
    planner.max_nodes = max_nodes
    planner.dyna_plot = False
    planner.dt = 0.1
    planner.steps = 1

    t0 = time.perf_counter()
    with _quiet():
        planner.find_path_to_goal(x_goal)
    elapsed = time.perf_counter() - t0
    final = planner.traj.x[-1, :]
    goal_err = _goal_error(final, x_goal)

    return ParityRow(
        case="pendulum_rrt",
        framework="pyro",
        backend="brute_nn+euler",
        success=True,
        build_s=0.0,
        solve_s=elapsed,
        sim_s=0.0,
        total_s=elapsed,
        goal_error=goal_err,
        nodes=len(planner.nodes),
        path_time_s=float(planner.traj.t[-1]),
        notes=f"seed={seed} max_nodes={max_nodes}",
    )


PYRO_RUNNERS = {
    "pendulum_dp": run_pyro_pendulum_dp,
    "double_pendulum_dp": run_pyro_double_pendulum_dp,
    "pendulum_rrt": run_pyro_pendulum_rrt,
}

MINILINK_RUNNERS = {
    "pendulum_dp_numpy": lambda **kw: run_minilink_pendulum_dp(backend="numpy", **kw),
    "pendulum_dp_jax": lambda **kw: run_minilink_pendulum_dp(backend="jax", **kw),
    "double_pendulum_dp": run_minilink_double_pendulum_dp,
    "pendulum_rrt": run_minilink_pendulum_rrt,
}


def row_to_json(row: ParityRow) -> str:
    return json.dumps(asdict(row))


def row_from_json(text: str) -> ParityRow:
    data = json.loads(text)
    return ParityRow(**data)
