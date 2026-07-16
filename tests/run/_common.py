"""Shared plumbing for ``tests/run/`` IDE-click-run launchers."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def repo_env(*, headless_pygame: bool = False) -> dict[str, str]:
    env = {**os.environ, "PYTHONPATH": str(REPO_ROOT)}
    if headless_pygame:
        env["SDL_VIDEODRIVER"] = "dummy"
    return env


def run_command(cmd: list[str], *, headless_pygame: bool = False) -> int:
    print(f"[minilink] cwd={REPO_ROOT}")
    print(f"[minilink] {' '.join(cmd)}")
    return subprocess.call(
        cmd, cwd=REPO_ROOT, env=repo_env(headless_pygame=headless_pygame)
    )


def run_pytest(
    extra_args: list[str] | None = None,
    *,
    marker: str | None = None,
    headless_pygame: bool = False,
) -> int:
    cmd = [sys.executable, "-m", "pytest"]
    if marker:
        cmd.extend(["-m", marker])
    if extra_args:
        cmd.extend(extra_args)
    return run_command(cmd, headless_pygame=headless_pygame)


def run_ruff_check() -> int:
    return run_command([sys.executable, "-m", "ruff", "check", "."])


def run_ruff_format_check() -> int:
    return run_command([sys.executable, "-m", "ruff", "format", "--check", "."])


def run_regression(*, ci_mode: bool) -> int:
    cmd = [
        sys.executable,
        str(REPO_ROOT / "benchmarks" / "run_regression_check.py"),
        "--suite",
        "all",
    ]
    if ci_mode:
        cmd.extend(
            [
                "--tiny",
                "--factor",
                "6",
                "--speed-gate-suffixes",
                "solve_s,nlp_s,speedup",
            ]
        )
    return run_command(cmd)


def run_study_script(args: list[str]) -> int:
    cmd = [sys.executable, str(REPO_ROOT / "benchmarks" / "run_study.py"), *args]
    return run_command(cmd)


def run_demo_check_script(relative_path: str, args: list[str] | None = None) -> int:
    cmd = [sys.executable, str(REPO_ROOT / relative_path), *(args or [])]
    return run_command(cmd)
