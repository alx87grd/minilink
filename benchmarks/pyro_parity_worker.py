"""Subprocess entry point for pyro-only parity cases (numpy < 2 env).

Usage::

    PYRO_PYTHON=/opt/anaconda3/envs/dev/bin/python \\
        python benchmarks/pyro_parity_worker.py pendulum_dp --fast
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# Ensure repo root is importable when invoked as a script.
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

os.environ.setdefault("MPLBACKEND", "Agg")

from benchmarks.pyro_parity import PYRO_RUNNERS, row_to_json  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one pyro parity benchmark case.")
    parser.add_argument("case", choices=tuple(PYRO_RUNNERS))
    parser.add_argument("--fast", action="store_true", help="Smaller grids / fewer steps.")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.case == "pendulum_dp":
        if args.fast:
            row = PYRO_RUNNERS[args.case](
                x_grid=(51, 51),
                u_grid=(11,),
                tol=0.1,
                use_fixed_steps=True,
                n_steps=30,
            )
        else:
            row = PYRO_RUNNERS[args.case]()
    elif args.case == "double_pendulum_dp":
        if args.fast:
            row = PYRO_RUNNERS[args.case](
                x_grid=(31, 25, 31, 25),
                u_grid=(5, 5),
                n_steps=30,
            )
        else:
            row = PYRO_RUNNERS[args.case]()
    else:
        row = PYRO_RUNNERS[args.case](seed=args.seed)

    print(row_to_json(row))


if __name__ == "__main__":
    main()
