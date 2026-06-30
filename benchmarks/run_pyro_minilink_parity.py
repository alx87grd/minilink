"""Pyro vs Minilink parity benchmark runner.

Compares dynamic programming and RRT on problems matched to pyro demos:
success (goal reached), precision (cost-to-go, goal error), and wall time.

Run from the repository root::

    python benchmarks/run_pyro_minilink_parity.py
    python benchmarks/run_pyro_minilink_parity.py --fast
    python benchmarks/run_pyro_minilink_parity.py --cases pendulum_dp pendulum_rrt

Pyro runs in a separate Python with numpy < 2 (default: ``dev`` conda env).
Minilink runs in the current interpreter (use ``dev-h26`` for JAX).

Environment variables:

- ``PYRO_PYTHON`` — interpreter for pyro (default ``/opt/anaconda3/envs/dev/bin/python``)
- ``MPLBACKEND=Agg`` — set automatically
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

os.environ.setdefault("MPLBACKEND", "Agg")

from benchmarks.common import format_benchmark_backend_label  # noqa: E402
from benchmarks.pyro_parity import (  # noqa: E402
    MINILINK_RUNNERS,
    PYRO_RUNNERS,
    ParityRow,
    _pyro_local_ok,
    row_from_json,
    run_minilink_double_pendulum_dp,
    run_minilink_pendulum_dp,
)

DEFAULT_PYRO_PYTHON = os.environ.get(
    "PYRO_PYTHON", "/opt/anaconda3/envs/dev/bin/python"
)


def _run_pyro_subprocess(case: str, *, fast: bool, seed: int = 0) -> ParityRow:
    worker = os.path.join(_REPO, "benchmarks", "pyro_parity_worker.py")
    cmd = [DEFAULT_PYRO_PYTHON, worker, case]
    if fast:
        cmd.append("--fast")
    if case == "pendulum_rrt":
        cmd.extend(["--seed", str(seed)])
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    env["PYTHONPATH"] = _REPO + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"pyro worker failed ({case}):\n{proc.stderr[-2000:]}\n{proc.stdout[-500:]}"
        )
    line = proc.stdout.strip().splitlines()[-1]
    return row_from_json(line)


def _run_pyro(case: str, *, fast: bool, seed: int = 0) -> ParityRow:
    if _pyro_local_ok():
        runner = PYRO_RUNNERS[case]
        if case == "pendulum_dp":
            if fast:
                return runner(
                    x_grid=(51, 51),
                    u_grid=(11,),
                    use_fixed_steps=True,
                    n_steps=30,
                )
            return runner()
        if case == "double_pendulum_dp":
            if fast:
                return runner(x_grid=(31, 25, 31, 25), n_steps=30)
            return runner()
        return runner(seed=seed)
    return _run_pyro_subprocess(case, fast=fast, seed=seed)


def _print_table(rows: list[ParityRow]) -> None:
    print(
        f"\n{'case':22} {'fw':8} {'backend':14} {'ok':>3} "
        f"{'build':>7} {'solve':>7} {'sim':>6} {'total':>7} "
        f"{'J0':>8} {'goal_e':>7} {'nodes':>7} {'notes'}"
    )
    print("-" * 120)
    for r in rows:
        ok = "Y" if r.success else "N"
        j0 = f"{r.j_at_start:8.2f}" if r.j_at_start is not None else "     n/a"
        ge = f"{r.goal_error:7.3f}" if r.goal_error is not None else "    n/a"
        nodes = f"{r.nodes:7d}" if r.nodes is not None else "    n/a"
        backend = (
            format_benchmark_backend_label(r.backend)
            if r.backend == "jax"
            else r.backend
        )
        print(
            f"{r.case:22} {r.framework:8} {backend:14} {ok:>3} "
            f"{r.build_s:7.2f} {r.solve_s:7.2f} {r.sim_s:6.2f} {r.total_s:7.2f} "
            f"{j0} {ge} {nodes}  {r.notes}"
        )
    print("-" * 120)
    print(
        "times [s]: build = grid/x_next; solve = Bellman or RRT; sim = closed-loop roll-out"
    )


def _parity_verdict(pyro: ParityRow, mini: ParityRow) -> list[str]:
    lines = []
    if mini.success and not pyro.success:
        lines.append(
            f"  {mini.case}/{mini.backend}: minilink succeeds where pyro failed"
        )
    elif not mini.success and pyro.success:
        lines.append(f"  FAIL {mini.case}/{mini.backend}: minilink did not reach goal")
    else:
        lines.append(f"  {mini.case}/{mini.backend}: success parity ({mini.success})")

    if mini.goal_error is not None and pyro.goal_error is not None:
        if mini.goal_error <= pyro.goal_error * 1.1 + 0.05:
            lines.append(
                f"    goal error {mini.goal_error:.3f} vs pyro {pyro.goal_error:.3f} — ok"
            )
        else:
            lines.append(
                f"    FAIL goal error {mini.goal_error:.3f} vs pyro {pyro.goal_error:.3f}"
            )

    if mini.j_at_start is not None and pyro.j_at_start is not None:
        dj = abs(mini.j_at_start - pyro.j_at_start)
        if dj < max(2.0, 0.05 * abs(pyro.j_at_start)):
            lines.append(
                f"    J(x0) {mini.j_at_start:.2f} vs pyro {pyro.j_at_start:.2f} — ok"
            )
        else:
            lines.append(
                f"    WARN J(x0) gap {dj:.2f} (mini {mini.j_at_start:.2f}, pyro {pyro.j_at_start:.2f})"
            )

    if mini.total_s > 0 and pyro.total_s > 0:
        speedup = pyro.total_s / mini.total_s
        tag = (
            "faster" if speedup > 1.05 else ("slower" if speedup < 0.95 else "similar")
        )
        lines.append(
            f"    total time {mini.total_s:.2f}s vs pyro {pyro.total_s:.2f}s ({speedup:.2f}x, {tag})"
        )
    return lines


def run_pendulum_dp(*, fast: bool) -> list[ParityRow]:
    rows = []
    print("\n=== pendulum DP (pyro pendulum_optimal_swingup_demo) ===")
    pyro = _run_pyro("pendulum_dp", fast=fast)
    rows.append(pyro)
    print(f"  pyro done in {pyro.total_s:.2f}s (success={pyro.success})")

    for label, backend in (("numpy", "numpy"), ("jax", "jax")):
        if backend == "jax":
            try:
                import jax  # noqa: F401
            except ImportError:
                print("  skipping minilink/jax (jax not installed)")
                continue
        t0 = time.perf_counter()
        if fast:
            mini = run_minilink_pendulum_dp(
                x_grid=(51, 51),
                u_grid=(11,),
                backend=backend,
                fast_solve=True,
                n_steps=30,
            )
        else:
            mini = run_minilink_pendulum_dp(backend=backend)
        rows.append(mini)
        print(
            f"  minilink/{backend} done in {mini.total_s:.2f}s "
            f"(success={mini.success}, elapsed {time.perf_counter() - t0:.1f}s)"
        )
    return rows


def run_double_pendulum_dp(*, fast: bool) -> list[ParityRow]:
    """Minilink-only — pyro full grid (~109M pairs) takes 15+ min in Python."""
    rows = []
    print("\n=== double pendulum DP (minilink only; pyro skipped — too slow) ===")
    try:
        import jax  # noqa: F401
    except ImportError:
        print("  skipping minilink/jax")
        return rows

    if fast:
        mini = run_minilink_double_pendulum_dp(
            x_grid=(31, 25, 31, 25), n_steps=30, backend="jax"
        )
    else:
        mini = run_minilink_double_pendulum_dp(backend="jax")
    rows.append(mini)
    print(f"  minilink/jax done in {mini.total_s:.2f}s (success={mini.success})")
    return rows


def run_pendulum_rrt(*, seeds: list[int]) -> list[ParityRow]:
    rows = []
    print("\n=== pendulum RRT (pyro simple_pendulum_with_rrt) ===")
    for seed in seeds:
        print(f"  seed {seed}...")
        pyro = _run_pyro("pendulum_rrt", fast=False, seed=seed)
        mini = MINILINK_RUNNERS["pendulum_rrt"](seed=seed)
        pyro.notes = f"seed={seed}"
        mini.notes = f"seed={seed}"
        rows.extend([pyro, mini])
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Pyro vs Minilink parity benchmarks.")
    parser.add_argument(
        "--fast",
        action="store_true",
        help="Smaller grids and fewer Bellman steps (recommended first run).",
    )
    parser.add_argument(
        "--cases",
        nargs="+",
        default=["pendulum_dp", "double_pendulum_dp", "pendulum_rrt"],
        choices=["pendulum_dp", "double_pendulum_dp", "pendulum_rrt"],
    )
    parser.add_argument("--rrt-seeds", type=int, nargs="+", default=[0, 1, 2])
    args = parser.parse_args()

    print("Pyro interpreter:", DEFAULT_PYRO_PYTHON)
    print("Minilink interpreter:", sys.executable)
    print("Mode:", "fast" if args.fast else "pyro-matched")

    all_rows: list[ParityRow] = []
    if "pendulum_dp" in args.cases:
        all_rows.extend(run_pendulum_dp(fast=args.fast))
    if "double_pendulum_dp" in args.cases:
        all_rows.extend(run_double_pendulum_dp(fast=args.fast))
    if "pendulum_rrt" in args.cases:
        all_rows.extend(run_pendulum_rrt(seeds=args.rrt_seeds))

    _print_table(all_rows)

    print("\n=== parity verdict ===")
    by_key: dict[tuple[str, str], ParityRow] = {}
    for r in all_rows:
        by_key[(r.case, r.framework if r.framework == "pyro" else r.backend)] = r

    pyro_dp = by_key.get(("pendulum_dp", "pyro"))
    for backend in ("numpy", "jax"):
        mini = by_key.get(("pendulum_dp", backend))
        if pyro_dp and mini:
            for line in _parity_verdict(pyro_dp, mini):
                print(line)

    _ = by_key.get(("double_pendulum_dp", "pyro"))
    mini_ddp = by_key.get(("double_pendulum_dp", "jax"))
    if mini_ddp:
        print(
            f"  double_pendulum/jax: success={mini_ddp.success}, "
            f"goal_err={mini_ddp.goal_error:.3f}, total={mini_ddp.total_s:.1f}s "
            f"(pyro reference not timed — full grid too slow in pyro Python loops)"
        )

    pyro_rrt = [
        r for r in all_rows if r.case == "pendulum_rrt" and r.framework == "pyro"
    ]
    mini_rrt = [
        r for r in all_rows if r.case == "pendulum_rrt" and r.framework == "minilink"
    ]
    if pyro_rrt and mini_rrt:
        pyro_ok = sum(r.success for r in pyro_rrt)
        mini_ok = sum(r.success for r in mini_rrt)
        pyro_t = sum(r.solve_s for r in pyro_rrt) / len(pyro_rrt)
        mini_t = sum(r.solve_s for r in mini_rrt) / len(mini_rrt)
        print(
            f"  pendulum_rrt: success {mini_ok}/{len(mini_rrt)} minilink vs "
            f"{pyro_ok}/{len(pyro_rrt)} pyro; avg solve {mini_t:.2f}s vs {pyro_t:.2f}s"
        )


if __name__ == "__main__":
    main()
