"""Interactive graphics visual check — run locally, confirm with your eyes.

This script is **not** for CI. It launches or prints commands for the main
graphics output channels so you can verify Meshcat, Matplotlib, and Plotly.

Usage (from repo root)::

    python tests/demo_checks/run_graphics_visual_check.py
    python tests/demo_checks/run_graphics_visual_check.py --run-headless
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

CHECKS = (
    (
        "Matplotlib kinematic (catalog manifest PNGs)",
        [
            sys.executable,
            "tests/demo_checks/run_flagship_graphics.py",
            "--out",
            "artifacts/gfx-check",
        ],
    ),
    (
        "Matplotlib animate (computed torque pendulum)",
        [sys.executable, "examples/scripts/control/computed_torque_pendulum.py"],
    ),
    (
        "Matplotlib hybrid MPC animate",
        [sys.executable, "examples/scripts/mpc/mpc_car_minimal.py"],
    ),
    (
        "Meshcat 3D (physics in diagram — optional meshcat extra)",
        [sys.executable, "examples/experimental/engine/engine_physics_in_diagram.py"],
    ),
    (
        "Cartpole trajopt animate (set LIVE_PLOT=True for Plotly iterates)",
        [
            sys.executable,
            "examples/scripts/trajopt/trajopt_cartpole_collocation_jax.py",
        ],
    ),
)


def _resolve_cmd(cmd: list[str]) -> list[str]:
    resolved: list[str] = []
    for part in cmd:
        if part.endswith(".py"):
            resolved.append(str(REPO_ROOT / part))
        else:
            resolved.append(part)
    return resolved


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Graphics visual checklist (local)")
    parser.add_argument(
        "--run-headless",
        action="store_true",
        help="Run headless kinematic PNG check only (no interactive prompt)",
    )
    args = parser.parse_args(argv)

    print("Minilink graphics visual check")
    print("Run each item below and confirm output looks correct.\n")
    for index, (label, cmd) in enumerate(CHECKS, start=1):
        full_cmd = _resolve_cmd(cmd)
        print(f"{index}. {label}")
        print(f"   {' '.join(full_cmd)}\n")

    if args.run_headless:
        cmd = _resolve_cmd(
            [
                sys.executable,
                "tests/demo_checks/run_flagship_graphics.py",
                "--out",
                "artifacts/gfx-check",
            ]
        )
        subprocess.run(cmd, cwd=REPO_ROOT, check=False)
        print(f"Inspect PNGs under {REPO_ROOT / 'artifacts/gfx-check'}")
        return 0

    answer = input("Run Matplotlib kinematic PNG check now? [y/N] ").strip().lower()
    if answer == "y":
        cmd = _resolve_cmd(
            [
                sys.executable,
                "tests/demo_checks/run_flagship_graphics.py",
                "--out",
                "artifacts/gfx-check",
            ]
        )
        subprocess.run(cmd, cwd=REPO_ROOT, check=False)
        print(f"Inspect PNGs under {REPO_ROOT / 'artifacts/gfx-check'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
