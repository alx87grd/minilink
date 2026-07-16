"""Unified Layer-C benchmark entry point (machine exploration, not CI gates).

Usage (from repo root)::

    python benchmarks/run_study.py --list
    python benchmarks/run_study.py --preset f_eval --plant pendulum
    python benchmarks/run_study.py --preset sim --mode matrix
    python benchmarks/run_study.py --preset trajopt --mode backends
    python benchmarks/run_study.py --preset pyro_parity -- --fast
"""

from __future__ import annotations

import argparse

from benchmarks.studies import presets

PRESET_HELP = {
    "f_eval": "Native vs NumPy vs JAX ``f()`` micro-benchmark (--plant pendulum|diagram_dense)",
    "step_eval": "Leaf/step-diagram ``step()`` micro-benchmark (--plant leaf|step_diagram)",
    "sim": "Simulator solver×backend study (--mode standard|matrix|manual)",
    "optimizer": "NLP solver presets on textbook problems",
    "trajopt": "Trajopt sweep (--mode backends|solvers)",
    "dp": "Dynamic programming loop/numpy/jax backends",
    "rrt_nearest": "RRT nearest brute_force vs kd_tree on dense holonomic scene",
    "pyro_parity": "Pyro vs Minilink DP/RRT parity (forwards extra args after ``--``)",
}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Layer-C benchmark studies (manual machine exploration)"
    )
    parser.add_argument(
        "--preset",
        choices=sorted(PRESET_HELP),
        help="Benchmark study preset",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List presets and exit",
    )
    parser.add_argument(
        "--plant",
        help="Plant fixture for f_eval / step_eval presets",
    )
    parser.add_argument(
        "--mode",
        help="Sub-mode for sim (standard|matrix|manual) or trajopt (backends|solvers)",
    )
    parser.add_argument(
        "--n-calls",
        type=int,
        default=None,
        help="Repeat count for f_eval / step_eval",
    )
    parser.add_argument(
        "--n-runs",
        type=int,
        default=None,
        help="Repeat count for sim / optimizer / trajopt presets",
    )
    parser.add_argument(
        "--case",
        default="cartpole",
        help="Trajopt problem case name",
    )
    parser.add_argument(
        "remainder",
        nargs=argparse.REMAINDER,
        help="Extra args for pyro_parity (prefix with --)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.list or args.preset is None:
        print("Layer-C benchmark presets (not CI gates):\n")
        for name in sorted(PRESET_HELP):
            print(f"  {name:12}  {PRESET_HELP[name]}")
        return 0

    preset = args.preset

    if preset == "f_eval":
        plant = args.plant or "pendulum"
        presets.run_f_eval(plant=plant, n_calls=args.n_calls or 1000)
        return 0

    if preset == "step_eval":
        plant = args.plant or "leaf"
        presets.run_step_eval(plant=plant, n_calls=args.n_calls)
        return 0

    if preset == "sim":
        mode = args.mode or "matrix"
        n_runs = args.n_runs or 1
        if mode == "standard":
            presets.run_sim_standard(n_runs=n_runs)
        elif mode == "matrix":
            presets.run_sim_matrix(n_runs=max(n_runs, 1))
        elif mode == "manual":
            presets.run_sim_manual(n_runs=n_runs)
        else:
            raise SystemExit(
                f"unknown sim mode {mode!r}; use standard, matrix, or manual"
            )
        return 0

    if preset == "optimizer":
        presets.run_optimizer(runs=args.n_runs or 1)
        return 0

    if preset == "trajopt":
        mode = args.mode or "backends"
        runs = args.n_runs or 1
        if mode == "backends":
            presets.run_trajopt_backends(case=args.case, runs=runs)
        elif mode == "solvers":
            presets.run_trajopt_solvers(case=args.case, runs=runs)
        else:
            raise SystemExit(f"unknown trajopt mode {mode!r}; use backends or solvers")
        return 0

    if preset == "dp":
        presets.run_dp()
        return 0

    if preset == "rrt_nearest":
        presets.run_rrt_nearest()
        return 0

    if preset == "pyro_parity":
        extra = [a for a in args.remainder if a != "--"]
        return presets.run_pyro_parity(extra or None)

    raise SystemExit(f"unhandled preset {preset!r}")


if __name__ == "__main__":
    raise SystemExit(main())
