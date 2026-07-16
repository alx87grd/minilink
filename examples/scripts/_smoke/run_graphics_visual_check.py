"""Interactive graphics visual check (L5) — run locally, confirm with your eyes.

This script is **not** for CI. It launches or prints commands for the main
graphics output channels so you can verify Meshcat, Matplotlib, and Plotly.

Usage (from repo root)::

    python examples/scripts/_smoke/run_graphics_visual_check.py
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]

CHECKS = (
    (
        "Matplotlib kinematic (catalog manifest PNGs)",
        [
            sys.executable,
            "examples/scripts/_smoke/run_flagship_graphics.py",
            "--out",
            "artifacts/gfx-smoke",
        ],
    ),
    (
        "Matplotlib animate (computed torque pendulum)",
        [sys.executable, "examples/scripts/control/demo_computed_torque_pendulum.py"],
    ),
    (
        "Matplotlib hybrid MPC animate",
        [sys.executable, "examples/scripts/mpc/demo_mpc_minimal.py"],
    ),
    (
        "Meshcat 3D (physics in diagram — optional meshcat extra)",
        [sys.executable, "examples/scripts/engine/demo_physics_in_diagram.py"],
    ),
    (
        "Plotly live trajopt (optional plotting extra)",
        [
            sys.executable,
            "examples/scripts/trajectory_optimization/demo_cartpole_direct_collocation_live_plot.py",
        ],
    ),
)


def main() -> int:
    print("Minilink L5 graphics visual check")
    print("Run each item below and confirm output looks correct.\n")
    for index, (label, cmd) in enumerate(CHECKS, start=1):
        rel = [str(Path(part)) if part.endswith(".py") else part for part in cmd]
        full_cmd = []
        for part in rel:
            if part.endswith(".py"):
                full_cmd.append(str(REPO_ROOT / part))
            else:
                full_cmd.append(part)
        print(f"{index}. {label}")
        print(f"   {' '.join(full_cmd)}\n")

    answer = input("Run Matplotlib kinematic smoke now? [y/N] ").strip().lower()
    if answer == "y":
        cmd = [
            sys.executable,
            str(REPO_ROOT / "examples/scripts/_smoke/run_flagship_graphics.py"),
            "--out",
            str(REPO_ROOT / "artifacts/gfx-smoke"),
        ]
        subprocess.run(cmd, cwd=REPO_ROOT, check=False)
        print(f"Inspect PNGs under {REPO_ROOT / 'artifacts/gfx-smoke'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
