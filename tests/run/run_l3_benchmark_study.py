"""L3 — backend/computer performance exploration (tables, not CI gates).

**Human (IDE):** open this file, set ``PRESET`` below, click **Run**.

For pass/fail performance gates use ``run_l2_regression.py`` instead.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_CHECKS = Path(__file__).resolve().parent
for path in (_ROOT, _CHECKS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import _common  # noqa: E402

# Set LIST_PRESETS = True to print available studies; otherwise run one preset.
LIST_PRESETS = False

# Presets: f_eval | step_eval | sim | optimizer | trajopt | dp | rrt_nearest | pyro_parity
PRESET = "f_eval"
PLANT = "pendulum"  # f_eval / step_eval
MODE = None  # sim: standard|matrix|manual; trajopt: backends|solvers


def main() -> int:
    if LIST_PRESETS:
        return _common.run_study_script(["--list"])
    args = ["--preset", PRESET]
    if PLANT:
        args.extend(["--plant", PLANT])
    if MODE:
        args.extend(["--mode", MODE])
    return _common.run_study_script(args)


if __name__ == "__main__":
    raise SystemExit(main())
