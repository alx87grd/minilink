"""Shared helpers for integration regression scenarios."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from benchmarks.simulation import (
    TRUTH_SIMULATION_VARIANT,
    SimulationBenchmarkVariant,
    benchmark_simulation_backend,
)
from minilink.simulation.simulator import Simulator


@dataclass(frozen=True)
class SimulationScenarioResult:
    """One timed simulation with trajectory checkpoints."""

    checkpoint_times: tuple[float, ...]
    checkpoint_x: dict[float, list[float]]
    x_tf: list[float]
    solve_s: float
    rel_err_l2: float
    truth_x_tf: list[float]


@dataclass(frozen=True)
class TrajoptScenarioResult:
    """One timed trajectory-optimization solve with checkpoints."""

    checkpoint_times: tuple[float, ...]
    checkpoint_x: dict[float, list[float]]
    x_tf: list[float]
    cost: float
    eq_inf: float
    success: bool
    total_s: float
    transcribe_s: float
    solve_s: float


def sample_trajectory_at_times(
    traj: Any, times: Sequence[float]
) -> dict[float, np.ndarray]:
    """Linearly interpolate ``traj.x`` at the requested times."""
    t = np.asarray(traj.t, dtype=float)
    x = np.asarray(traj.x, dtype=float)
    if x.ndim != 2:
        raise ValueError("expected traj.x with shape (n, n_t)")
    out: dict[float, np.ndarray] = {}
    for target in times:
        if target <= t[0]:
            out[target] = x[:, 0].copy()
            continue
        if target >= t[-1]:
            out[target] = x[:, -1].copy()
            continue
        idx = int(np.searchsorted(t, target, side="right") - 1)
        idx = max(0, min(idx, t.size - 2))
        t0, t1 = t[idx], t[idx + 1]
        alpha = float((target - t0) / (t1 - t0)) if t1 > t0 else 0.0
        out[target] = (1.0 - alpha) * x[:, idx] + alpha * x[:, idx + 1]
    return out


def assert_simulation_finite(traj: Any) -> None:
    """Raise when the simulated state trajectory is non-finite."""
    x = np.asarray(traj.x, dtype=float)
    if not np.all(np.isfinite(x)):
        raise ValueError("simulation produced non-finite state values")


def run_simulation_scenario(
    system: Any,
    *,
    scenario_id: str,
    candidate: SimulationBenchmarkVariant,
    t0: float,
    tf: float,
    dt: float,
    checkpoint_times: tuple[float, ...],
    n_runs: int = 2,
    truth: SimulationBenchmarkVariant = TRUTH_SIMULATION_VARIANT,
) -> SimulationScenarioResult:
    """Run a candidate simulation, sample checkpoints, and compare to truth."""
    del scenario_id  # used by callers for metric id prefixes
    bench = benchmark_simulation_backend(
        system,
        candidate=candidate,
        truth=truth,
        t0=t0,
        tf=tf,
        dt=dt,
        n_runs=n_runs,
        compile_once=True,
    )
    traj = _simulate_once(
        system,
        solver=candidate.solver,
        compile_backend=candidate.compile_backend,
        t0=t0,
        tf=tf,
        dt=dt,
    )
    assert_simulation_finite(traj)
    samples = sample_trajectory_at_times(traj, checkpoint_times)
    return SimulationScenarioResult(
        checkpoint_times=checkpoint_times,
        checkpoint_x={t: [float(v) for v in samples[t]] for t in checkpoint_times},
        x_tf=[float(v) for v in traj.x[:, -1].reshape(-1)],
        solve_s=float(bench.mean_solve_time),
        rel_err_l2=float(bench.rel_err_l2),
        truth_x_tf=[float(v) for v in bench.truth_x_final.reshape(-1)],
    )


def _simulate_once(
    system: Any,
    *,
    solver: str,
    compile_backend: str,
    t0: float,
    tf: float,
    dt: float,
):
    sim = Simulator(
        system,
        t0=t0,
        tf=tf,
        dt=dt,
        solver=solver,
        compile_backend=compile_backend,
        verbose=False,
    )
    t_start = time.perf_counter()
    traj = sim.solve()
    _ = time.perf_counter() - t_start
    return traj
