"""L2 — regression (accuracy goldens + backend solve-speed gates).

**Human (IDE):** open this file and click **Run**.

Pass/fail vs committed baselines in ``benchmarks/baselines/``. Run after compile /
Simulator / trajopt / MPC changes.
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

# False → full local pre-handoff; True → same flags as GitHub CI ``regression`` job
CI_MODE = False


def main() -> int:
    return _common.run_regression(ci_mode=CI_MODE)


if __name__ == "__main__":
    raise SystemExit(main())
