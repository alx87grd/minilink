"""Simulation scenarios for integration regression."""

from __future__ import annotations

import numpy as np

from benchmarks.scenarios.common import run_simulation_scenario
from benchmarks.simulation import SimulationBenchmarkVariant
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

DOUBLE_PENDULUM_CHECKPOINTS = (0.0, 1.0, 2.5, 5.0, 7.5, 10.0)
SHOWCASE_PENDULUM_CHECKPOINTS = (0.0, 2.5, 5.0, 7.5, 10.0)

DOUBLE_PENDULUM_CANDIDATE = SimulationBenchmarkVariant("rk4_fixedsteps", "numpy")
SHOWCASE_PENDULUM_CANDIDATE = SimulationBenchmarkVariant("rk4_fixedsteps", "numpy")


def build_double_pendulum():
    """Double pendulum with a deterministic non-trivial initial state."""
    sys = DoublePendulum()
    sys.x0 = np.array([-np.pi / 2, 0.5, 0.0, 0.0])
    return sys


def build_showcase_pendulum():
    """Catalog pendulum from the showcase notebook quick-start path."""
    sys = Pendulum()
    sys.x0[0] = 2.0
    return sys


def run_double_pendulum_sim(*, n_runs: int = 2):
    """Long double-pendulum integration with trajectory checkpoints."""
    return run_simulation_scenario(
        build_double_pendulum(),
        scenario_id="sim.double_pendulum",
        candidate=DOUBLE_PENDULUM_CANDIDATE,
        t0=0.0,
        tf=10.0,
        dt=0.01,
        checkpoint_times=DOUBLE_PENDULUM_CHECKPOINTS,
        n_runs=n_runs,
    )


def run_showcase_pendulum_sim(*, n_runs: int = 2):
    """Showcase catalog pendulum integration with trajectory checkpoints."""
    return run_simulation_scenario(
        build_showcase_pendulum(),
        scenario_id="sim.showcase_pendulum",
        candidate=SHOWCASE_PENDULUM_CANDIDATE,
        t0=0.0,
        tf=10.0,
        dt=0.01,
        checkpoint_times=SHOWCASE_PENDULUM_CHECKPOINTS,
        n_runs=n_runs,
    )
