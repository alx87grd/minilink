"""RRT nearest-neighbour backend timing on a holonomic obstacle scene.

Times ``RRTOptions.nearest_backend`` / ``RRTStarOptions.nearest_backend`` for
``brute_force`` vs SciPy ``cKDTree`` on the same problem fixture.
"""

import time
from dataclasses import dataclass

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import SteeringExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.rrt_star import RRTStarOptions, RRTStarPlanner
from minilink.planning.search.steering import StraightLineSteering
from minilink.planning.spatial.robot import sphere
from minilink.planning.spatial.scene import Scene

GOAL_TOLERANCE = 0.4
GOAL_BIAS = 0.0
MAX_NODES = 30000
STEERING_MAX_DISTANCE = 0.3
STEERING_RESOLUTION = 0.05
STAR_OPTIMIZE_AFTER_GOAL = True
STAR_CONVERGENCE_PATIENCE = 2000
STAR_COST_TOL = 0.01


@dataclass
class RRTNearestBenchmarkRow:
    """One timed RRT run."""

    planner: str
    backend: str
    seed: int
    elapsed_s: float
    nodes: int
    goal_error: float
    success: bool
    reached_goal: bool


def holonomic_problem():
    """18-sphere holonomic obstacle fixture (duplicated here to keep package boundary)."""
    sys = HolonomicMobileRobot()
    sys.state.lower_bound = np.array([-6.0, -6.0])
    sys.state.upper_bound = np.array([6.0, 6.0])
    sys.inputs["u"].lower_bound = np.array([-1.0, -1.0])
    sys.inputs["u"].upper_bound = np.array([1.0, 1.0])

    scene = Scene(
        obstacles=(
            Sphere([0.0, 0.0], 1.0),
            Sphere([2.0, -1.5], 0.8),
            Sphere([-2.0, -1.0], 0.75),
            Sphere([-1.0, 1.5], 0.7),
            Sphere([1.0, 2.0], 0.65),
            Sphere([3.0, 0.5], 0.7),
            Sphere([0.5, -2.5], 0.75),
            Sphere([-3.0, 1.0], 0.6),
            Sphere([-0.5, -3.5], 0.65),
            Sphere([2.5, 2.5], 0.6),
            Sphere([-2.5, 2.0], 0.65),
            Sphere([1.5, -0.5], 0.55),
            Sphere([-1.5, -2.5], 0.7),
            Sphere([4.0, -1.0], 0.65),
            Sphere([-4.0, 0.0], 0.6),
            Sphere([0.0, 3.5], 0.55),
            Sphere([3.5, -2.5], 0.6),
            Sphere([-3.5, -2.0], 0.55),
        )
    )
    robot = sphere(radius=0.25, position=(0, 1))
    X = BoxSet.from_system_state(sys) & scene.clearance_field(robot).as_constraint()
    x_goal = np.array([4.0, 4.0])
    problem = PlanningProblem(
        sys=sys,
        x_start=np.array([-4.0, -4.0]),
        x_goal=x_goal,
        X=X,
    )
    extender = SteeringExtender(
        StraightLineSteering(speed=1.0),
        max_distance=STEERING_MAX_DISTANCE,
        resolution=STEERING_RESOLUTION,
    )
    return problem, extender, x_goal


def _make_options(planner_cls, seed, nearest_backend):
    if planner_cls is RRTStarPlanner:
        return RRTStarOptions(
            seed=seed,
            goal_tolerance=GOAL_TOLERANCE,
            goal_bias=GOAL_BIAS,
            max_nodes=MAX_NODES,
            nearest_backend=nearest_backend,
            optimize_after_goal=STAR_OPTIMIZE_AFTER_GOAL,
            convergence_patience=STAR_CONVERGENCE_PATIENCE,
            cost_tol=STAR_COST_TOL,
        )
    return RRTOptions(
        seed=seed,
        goal_tolerance=GOAL_TOLERANCE,
        goal_bias=GOAL_BIAS,
        max_nodes=MAX_NODES,
        nearest_backend=nearest_backend,
    )


def benchmark_nearest_backend(planner_cls, backend, seed):
    """Time one planner/backend pair on the holonomic fixture."""
    problem, extender, x_goal = holonomic_problem()
    planner = planner_cls(
        problem,
        extender=extender,
        options=_make_options(planner_cls, seed, backend),
    )
    t0 = time.perf_counter()
    traj = planner.compute_solution()
    elapsed = time.perf_counter() - t0
    goal_error = float(np.linalg.norm(traj.x[:, -1] - x_goal))
    label = "rrt*" if planner_cls is RRTStarPlanner else "rrt"
    return RRTNearestBenchmarkRow(
        planner=label,
        backend=backend,
        seed=seed,
        elapsed_s=elapsed,
        nodes=len(planner.tree.nodes),
        goal_error=goal_error,
        success=goal_error <= GOAL_TOLERANCE,
        reached_goal=planner.reached_goal,
    )


def speedup(brute_s, kd_s):
    if kd_s <= 0.0:
        return float("inf")
    return brute_s / kd_s


def print_rrt_nearest_benchmark(rows, *, seeds):
    """Print per-seed and aggregate nearest-backend timing."""
    planner_pairs = (("rrt", RRTPlanner), ("rrt*", RRTStarPlanner))
    backends = ("brute_force", "kd_tree")
    runs = {name: {backend: [] for backend in backends} for name, _ in planner_pairs}

    print(
        f"RRT nearest-backend comparison ({len(seeds)} seeds, max_nodes={MAX_NODES}, "
        f"rrt* optimize_after_goal={STAR_OPTIMIZE_AFTER_GOAL})"
    )
    print(
        f"{'seed':>4s}  {'rrt n':>6s}  {'rrt brute t':>11s}  {'rrt kd t':>8s}  "
        f"{'rrt spdup':>8s}  {'rrt* n':>7s}  {'rrt* brute t':>12s}  "
        f"{'rrt* kd t':>9s}  {'rrt* spdup':>10s}"
    )
    print("-" * 96)

    by_seed = {}
    for row in rows:
        by_seed.setdefault(row.seed, {})[(row.planner, row.backend)] = row

    for seed in seeds:
        row = by_seed[seed]
        for label, _ in planner_pairs:
            for backend in backends:
                runs[label][backend].append(row[(label, backend)])

        rrt_brute = row[("rrt", "brute_force")]
        rrt_kd = row[("rrt", "kd_tree")]
        star_brute = row[("rrt*", "brute_force")]
        star_kd = row[("rrt*", "kd_tree")]

        print(
            f"{seed:4d}  "
            f"{rrt_brute.nodes:6d}  "
            f"{rrt_brute.elapsed_s:11.2f}  {rrt_kd.elapsed_s:8.2f}  "
            f"{speedup(rrt_brute.elapsed_s, rrt_kd.elapsed_s):8.2f}x  "
            f"{star_brute.nodes:7d}  "
            f"{star_brute.elapsed_s:12.2f}  {star_kd.elapsed_s:9.2f}  "
            f"{speedup(star_brute.elapsed_s, star_kd.elapsed_s):10.2f}x"
        )

    print("-" * 96)
    for label, _ in planner_pairs:
        speedups = []
        for brute_run, kd_run in zip(runs[label]["brute_force"], runs[label]["kd_tree"]):
            if brute_run.success and kd_run.success:
                speedups.append(speedup(brute_run.elapsed_s, kd_run.elapsed_s))
        avg_nodes = np.mean([run.nodes for run in runs[label]["brute_force"]])
        avg_speedup = np.mean(speedups) if speedups else float("nan")
        successes = sum(run.success for run in runs[label]["brute_force"])
        print(
            f"{label:5s} avg: {avg_nodes:5.0f} nodes, "
            f"{successes}/{len(seeds)} reached goal, "
            f"avg speedup: {avg_speedup:.2f}x"
        )
