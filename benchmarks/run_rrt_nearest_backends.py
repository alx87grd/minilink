"""Manual RRT nearest-neighbour backend benchmark runner.

Compares ``brute_force`` vs ``kd_tree`` for RRT and RRT* on the holonomic
18-sphere obstacle scene. RRT* uses ``optimize_after_goal=True`` with
``convergence_patience=2000`` — each extra seed adds significant runtime.

Modest speedups (~1.2–1.7×) are normal here: scene collision checks and the
post-goal ``_refresh_best_goal`` full-tree scan dominate wall time (~75%
combined on ~7k-node runs), and 2D brute-force nearest queries are already
cheap. For clearer ``kd_tree`` wins, run the 4D double-pendulum example::

    python examples/scripts/planning/demo_rrt_kdtree_speed_double_pendulum.py

Run from the repository root::

    python benchmarks/run_rrt_nearest_backends.py
"""

from __future__ import annotations

from minilink.planning.search.rrt import RRTPlanner
from minilink.planning.search.rrt_star import RRTStarPlanner

from benchmarks.planning_rrt import benchmark_nearest_backend, print_rrt_nearest_benchmark

# Edit these, then re-run.
seeds = (0,)
backends = ("brute_force", "kd_tree")
planner_pairs = (("rrt", RRTPlanner), ("rrt*", RRTStarPlanner))

rows = []
for seed in seeds:
    for label, planner_cls in planner_pairs:
        for backend in backends:
            rows.append(benchmark_nearest_backend(planner_cls, backend, seed))

print_rrt_nearest_benchmark(rows, seeds=seeds)
