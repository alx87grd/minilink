"""Benchmark-study preset implementations (machine exploration, not CI gates)."""

from __future__ import annotations

import importlib
import importlib.util
from collections.abc import Sequence

import numpy as np

from benchmarks.common import ipopt_available
from benchmarks.dynamic_programming import (
    benchmark_backend,
    check_agreement,
    print_dp_benchmark,
)
from benchmarks.f_evaluators import benchmark_f_evaluators, print_f_benchmark
from benchmarks.optimization import (
    STANDARD_OPTIMIZATION_CASES,
    benchmark_optimizer_backends,
    default_optimizer_variants,
    print_optimizer_benchmark,
)
from benchmarks.planning_rrt import (
    benchmark_nearest_backend,
    print_rrt_nearest_benchmark,
)
from benchmarks.simulation import (
    DEFAULT_SIMULATION_VARIANTS,
    SimulationBenchmarkVariant,
    benchmark_simulation_matrix,
    benchmark_standard_simulation_suite,
    print_simulation_matrix_benchmark,
    print_standard_simulation_benchmark,
)
from benchmarks.step_evaluators import benchmark_step_evaluators, print_step_benchmark
from benchmarks.systems.basic import JaxPendulum, make_pendulum
from benchmarks.systems.engine import make_physics_many_spheres
from benchmarks.systems.network import make_dense_network
from benchmarks.systems.step import LogisticMap, make_step_chain
from benchmarks.trajopt import (
    TrajectoryOptimizationBenchmarkConfig,
    benchmark_trajectory_optimization,
    default_trajectory_optimization_solver_variants,
    default_trajectory_optimization_variants,
    print_trajectory_optimization_benchmark,
)
from minilink.planning.search.rrt import RRTPlanner
from minilink.planning.search.rrt_star import RRTStarPlanner


def run_f_eval(*, plant: str, n_calls: int = 1000) -> None:
    if plant == "pendulum":
        sys = JaxPendulum(damping=0.5)
        sys.x0[0] = 1.0
        x_np = np.array([1.0, 0.0])
        u_np = np.array([0.0])
    elif plant == "diagram_dense":
        sys = make_dense_network(num_nodes=50, connections_per_node=5)
        x_np = np.ones(sys.n)
        u_np = np.array([])
    else:
        raise ValueError(
            f"unknown f_eval plant {plant!r}; use pendulum or diagram_dense"
        )
    result = benchmark_f_evaluators(sys, x_np, u_np, t=0.0, n_calls=n_calls)
    print_f_benchmark(result)


def run_step_eval(*, plant: str, n_calls: int | None = None) -> None:
    if plant == "leaf":
        sys = LogisticMap()
        x_np = np.array([0.4])
        u_np = np.array([])
        calls = 100_000 if n_calls is None else n_calls
    elif plant == "step_diagram":
        sys = make_step_chain(depth=50)
        x_np = np.ones(sys.n)
        u_np = np.array([1.0])
        calls = 10_000 if n_calls is None else n_calls
    else:
        raise ValueError(f"unknown step_eval plant {plant!r}; use leaf or step_diagram")
    result = benchmark_step_evaluators(sys, x_np, u_np, k=0, n_calls=calls)
    print_step_benchmark(result)


def run_sim_standard(
    *, solver: str = "rk4_fixedsteps", backend: str = "jax", n_runs: int = 1
) -> None:
    _configure_jax_x64(False)
    candidate = SimulationBenchmarkVariant(solver, backend)
    results = benchmark_standard_simulation_suite(candidate, n_runs=n_runs)
    print_standard_simulation_benchmark(results)


def run_sim_matrix(*, n_runs: int = 5) -> None:
    _configure_jax_x64(False)
    cases = (
        (make_pendulum(), "Pendulum", 0.0, 100.0, 0.01),
        (
            make_dense_network(num_nodes=50, connections_per_node=5),
            "Dense network (50 nodes, 5 conn/node)",
            0.0,
            5.0,
            0.1,
        ),
        (
            make_physics_many_spheres(nx=6, ny=4),
            "PhysicsManySpheres (24 bodies)",
            0.0,
            1.0,
            0.01,
        ),
    )
    for sys, case_name, t0, tf, dt in cases:
        result = benchmark_simulation_matrix(
            sys,
            case_name=case_name,
            variants=DEFAULT_SIMULATION_VARIANTS,
            t0=t0,
            tf=tf,
            dt=dt,
            n_runs=n_runs,
        )
        print_simulation_matrix_benchmark(result)


def run_sim_manual(*, n_runs: int = 1) -> None:
    _configure_jax_x64(True)
    variants = (
        SimulationBenchmarkVariant("euler", "numpy"),
        SimulationBenchmarkVariant("euler", "jax"),
        SimulationBenchmarkVariant("rk4_fixedsteps", "numpy"),
        SimulationBenchmarkVariant("rk4_fixedsteps", "jax"),
        SimulationBenchmarkVariant("scipy", "numpy"),
        SimulationBenchmarkVariant("scipy", "jax"),
        SimulationBenchmarkVariant("scipy_stiff", "numpy"),
        SimulationBenchmarkVariant("scipy_stiff", "jax"),
        SimulationBenchmarkVariant("scipy_lsoda", "numpy"),
        SimulationBenchmarkVariant("scipy_lsoda", "jax"),
        SimulationBenchmarkVariant("scipy_max", "numpy"),
        SimulationBenchmarkVariant("scipy_max", "jax"),
        SimulationBenchmarkVariant("scipy_ultra", "numpy"),
        SimulationBenchmarkVariant("scipy_ultra", "jax"),
    )
    sys = make_pendulum()
    result = benchmark_simulation_matrix(
        sys,
        case_name="Pendulum (manual variants)",
        variants=variants,
        t0=0.0,
        tf=10.0,
        dt=0.01,
        n_runs=n_runs,
    )
    print_simulation_matrix_benchmark(result)


def run_optimizer(*, maxiter: int = 200, runs: int = 1) -> None:
    if not ipopt_available():
        print("note: cyipopt is not installed; running SciPy variants only.")
    variants = default_optimizer_variants(maxiter=maxiter, ftol=1e-2, tol=1e-2)
    result = benchmark_optimizer_backends(
        STANDARD_OPTIMIZATION_CASES, variants, n_runs=runs
    )
    print_optimizer_benchmark(result)


def run_trajopt_backends(
    *,
    case: str = "cartpole",
    tf: float = 5.0,
    steps: int = 20,
    maxiter: int = 200,
    runs: int = 1,
    starts: str = "both",
    include_ipopt: bool = True,
    print_details: bool = True,
) -> None:
    if include_ipopt and not ipopt_available():
        print("note: cyipopt is not installed; skipping Ipopt trajopt variants.")
    start_map = {"cold": ("cold",), "warm": ("warm",), "both": ("cold", "warm")}
    if starts not in start_map:
        raise ValueError(f"starts must be one of {sorted(start_map)}")
    config = TrajectoryOptimizationBenchmarkConfig(
        case=case,
        tf=tf,
        n_steps=steps,
        maxiter=maxiter,
        ftol=1e-2,
        n_runs=runs,
        print_mathematical_program_details=print_details,
    )
    variants = default_trajectory_optimization_variants(
        starts=start_map[starts],
        ftol=1e-2,
        include_ipopt=include_ipopt,
    )
    result = benchmark_trajectory_optimization(config, variants)
    print_trajectory_optimization_benchmark(result)


def run_trajopt_solvers(
    *,
    case: str = "cartpole",
    tf: float = 5.0,
    steps: int = 100,
    maxiter: int = 200,
    runs: int = 1,
    starts: str = "cold",
    include_trust_constr: bool = False,
    include_ipopt: bool = True,
) -> None:
    if include_ipopt and not ipopt_available():
        print("note: cyipopt is not installed; skipping Ipopt trajopt variants.")
    start_map = {"cold": ("cold",), "warm": ("warm",), "both": ("cold", "warm")}
    if starts not in start_map:
        raise ValueError(f"starts must be one of {sorted(start_map)}")
    config = TrajectoryOptimizationBenchmarkConfig(
        case=case,
        tf=tf,
        n_steps=steps,
        maxiter=maxiter,
        ftol=1e-2,
        n_runs=runs,
    )
    variants = default_trajectory_optimization_solver_variants(
        starts=start_map[starts],
        ftol=1e-2,
        include_trust_constr=include_trust_constr,
        include_ipopt=include_ipopt,
    )
    result = benchmark_trajectory_optimization(config, variants)
    print_trajectory_optimization_benchmark(result)


def run_dp(*, runs_loop: int = 1, runs_numpy: int = 3, runs_jax: int = 3) -> None:
    jax_available = importlib.util.find_spec("jax") is not None
    small_grid = (21, 21)
    small_u = (5,)
    large_grid = (151, 151)
    large_u = (15,)
    runs = {"loop": runs_loop, "numpy": runs_numpy, "jax": runs_jax}

    small_backends = ["loop", "numpy"] + (["jax"] if jax_available else [])
    small_rows = [
        benchmark_backend(be, small_grid, small_u, runs=runs[be])
        for be in small_backends
    ]
    print(f"\n=== Small grid {small_grid}x{small_u} (loop included) ===")
    print_dp_benchmark(small_rows, reference="loop")
    check_agreement(small_rows)

    large_backends = ["numpy"] + (["jax"] if jax_available else [])
    large_rows = [
        benchmark_backend(be, large_grid, large_u, runs=runs[be])
        for be in large_backends
    ]
    print(f"\n=== Large grid {large_grid}x{large_u} (numpy vs jax) ===")
    print_dp_benchmark(large_rows, reference="numpy")
    check_agreement(large_rows)


def run_rrt_nearest(*, seeds: Sequence[int] = (0,)) -> None:
    backends = ("brute_force", "kd_tree")
    planner_pairs = (("rrt", RRTPlanner), ("rrt*", RRTStarPlanner))
    rows = []
    for seed in seeds:
        for label, planner_cls in planner_pairs:
            for backend in backends:
                rows.append(benchmark_nearest_backend(planner_cls, backend, seed))
    print_rrt_nearest_benchmark(rows, seeds=tuple(seeds))


def run_pyro_parity(*, argv: list[str] | None = None) -> int:
    """Delegate to pyro parity CLI (dual-interpreter story unchanged)."""
    import sys

    from benchmarks.run_pyro_minilink_parity import main as pyro_main

    if argv is not None:
        saved = sys.argv
        sys.argv = ["run_pyro_minilink_parity", *argv]
        try:
            pyro_main()
        finally:
            sys.argv = saved
    else:
        pyro_main()
    return 0


def _configure_jax_x64(enable: bool) -> None:
    if importlib.util.find_spec("jax") is None:
        return
    import jax

    jax.config.update("jax_enable_x64", enable)
