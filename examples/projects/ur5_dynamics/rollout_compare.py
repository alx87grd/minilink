"""Rollout simulation comparison for UR5 EoM pipelines."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator
from minilink.simulation.simulator import Simulator

DEFAULT_ROLLOUT_TF = 1.0
DEFAULT_ROLLOUT_DT = 0.002
DEFAULT_ROLLOUT_SOLVER = "rk4_fixedsteps"
DEFAULT_ROLLOUT_BACKEND = "jax"


class UR5ManipulatorABA(UR5Manipulator):
    """Catalog UR5 with :meth:`forward_dynamics` routed through ABA."""

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        return self.forward_dynamics_aba(q, v, u, t, params)


@dataclass
class RolloutTiming:
    """Wall times for one simulated rollout [ms]."""

    method: str
    compile_ms: float
    jit_warmup_ms: float
    rollout_ms: float
    backend: str = DEFAULT_ROLLOUT_BACKEND


@dataclass
class RolloutComparisonResult:
    """Rollout trajectories and errors vs RNEA–H reference."""

    backend: str
    tf: float
    dt: float
    n_steps: int
    reference_method: str = "RNEA-H"
    timings: list[RolloutTiming] = field(default_factory=list)
    max_state_error: dict[str, float] = field(default_factory=dict)


def default_rollout_state(arm):
    """Initial pose with modest joint rates for a nontrivial rollout."""
    q0 = np.array([0.0, -np.pi / 2 + 0.2, 0.0, -np.pi / 2, 0.0, 0.0])
    v0 = np.array([0.2, -0.1, 0.15, -0.05, 0.08, -0.03])
    return arm.q2x(q0, v0)


def configure_rollout_plant(plant, params):
    """Apply shared rollout parameters and zero nominal torque."""
    plant.params = dict(params)
    plant.x0 = default_rollout_state(plant)
    plant.inputs["u"].nominal_value = np.zeros(plant.m)
    return plant


def _sim_kwargs(*, tf, dt, solver, x0):
    return dict(
        x0=x0,
        t0=0.0,
        tf=tf,
        dt=dt,
        solver=solver,
        verbose=False,
    )


def timed_rollout(
    sys,
    *,
    method,
    compile_backend=DEFAULT_ROLLOUT_BACKEND,
    tf=DEFAULT_ROLLOUT_TF,
    dt=DEFAULT_ROLLOUT_DT,
    solver=DEFAULT_ROLLOUT_SOLVER,
):
    """
    Time compile, first solve (JIT warm-up), and second solve (integration).

    Returns ``(trajectory, RolloutTiming)``.
    """
    if compile_backend == "jax":
        from minilink.core.backends import configure_jax

        configure_jax(enable_x64=True)

    x0 = sys.x0 if sys.x0 is not None else default_rollout_state(sys)
    kwargs = _sim_kwargs(tf=tf, dt=dt, solver=solver, x0=x0)

    t0 = time.perf_counter()
    sim = Simulator(sys, compile_backend=compile_backend, **kwargs)
    compile_ms = 1e3 * (time.perf_counter() - t0)

    t0 = time.perf_counter()
    sim.solve()
    jit_warmup_ms = 1e3 * (time.perf_counter() - t0)

    t0 = time.perf_counter()
    traj = sim.solve()
    rollout_ms = 1e3 * (time.perf_counter() - t0)

    timing = RolloutTiming(
        method=method,
        compile_ms=compile_ms,
        jit_warmup_ms=jit_warmup_ms,
        rollout_ms=rollout_ms,
        backend=compile_backend,
    )
    return traj, timing


def _max_state_gap(reference, candidate):
    return float(np.max(np.abs(reference - candidate)))


def rollout_comparison(
    params,
    *,
    symbolic_plant=None,
    compile_backend=DEFAULT_ROLLOUT_BACKEND,
    tf=DEFAULT_ROLLOUT_TF,
    dt=DEFAULT_ROLLOUT_DT,
    solver=DEFAULT_ROLLOUT_SOLVER,
):
    """
    Simulate RNEA–H, ABA, and optional symbolic plants on the same grid.

    Reference trajectory: RNEA–H. Reports state errors and separate compile /
    JIT warm-up / rollout timings.
    """
    rnea = configure_rollout_plant(UR5Manipulator(), params)
    aba = configure_rollout_plant(UR5ManipulatorABA(), params)

    n_steps = int(round(tf / dt)) + 1
    result = RolloutComparisonResult(
        backend=compile_backend,
        tf=tf,
        dt=dt,
        n_steps=n_steps,
    )

    ref_traj, ref_timing = timed_rollout(
        rnea,
        method="RNEA-H",
        compile_backend=compile_backend,
        tf=tf,
        dt=dt,
        solver=solver,
    )
    result.timings.append(ref_timing)

    aba_traj, aba_timing = timed_rollout(
        aba,
        method="ABA",
        compile_backend=compile_backend,
        tf=tf,
        dt=dt,
        solver=solver,
    )
    result.timings.append(aba_timing)
    result.max_state_error["ABA"] = _max_state_gap(ref_traj.x, aba_traj.x)

    if symbolic_plant is not None:
        sym = configure_rollout_plant(symbolic_plant, params)
        sym_traj, sym_timing = timed_rollout(
            sym,
            method="Symbolic",
            compile_backend=compile_backend,
            tf=tf,
            dt=dt,
            solver=solver,
        )
        result.timings.append(sym_timing)
        result.max_state_error["Symbolic"] = _max_state_gap(ref_traj.x, sym_traj.x)

    return result, ref_traj


def rollout_timing_rows(
    result: RolloutComparisonResult,
) -> list[dict[str, float | str]]:
    """Rows for rollout timing tables."""
    rows = []
    for timing in result.timings:
        rows.append(
            {
                "method": timing.method,
                "compile_ms": timing.compile_ms,
                "jit_warmup_ms": timing.jit_warmup_ms,
                "rollout_ms": timing.rollout_ms,
                "backend": timing.backend,
            }
        )
    return rows


def rollout_error_rows(result: RolloutComparisonResult) -> list[dict[str, float | str]]:
    """State error vs RNEA–H reference trajectory."""
    rows = []
    for method, err in result.max_state_error.items():
        rows.append(
            {
                "method": method,
                "max_abs_dx": err,
            }
        )
    return rows


def print_rollout_report(result: RolloutComparisonResult):
    """Human-readable rollout summary (CLI smoke)."""
    print(
        f"Rollout: tf={result.tf}s, dt={result.dt}s, "
        f"n={result.n_steps}, backend={result.backend}"
    )
    print(f"Reference trajectory: {result.reference_method}\n")
    header = (
        f"{'method':<12}  {'compile ms':>11}  {'JIT warm ms':>12}  {'rollout ms':>11}"
    )
    print(header)
    print("-" * len(header))
    for row in rollout_timing_rows(result):
        print(
            f"{row['method']:<12}  {row['compile_ms']:11.3f}  "
            f"{row['jit_warmup_ms']:12.3f}  {row['rollout_ms']:11.3f}"
        )

    if result.max_state_error:
        print("\nMax |Δx| vs RNEA–H trajectory:")
        for row in rollout_error_rows(result):
            print(f"  {row['method']:<12}  {row['max_abs_dx']:.3e}")


def plot_rollout_timings(result: RolloutComparisonResult, *, figsize=(8, 3.5)):
    """Grouped bar chart of compile / JIT warm-up / rollout times."""
    import matplotlib.pyplot as plt

    methods = [t.method for t in result.timings]
    compile_ms = [t.compile_ms for t in result.timings]
    warmup_ms = [t.jit_warmup_ms for t in result.timings]
    rollout_ms = [t.rollout_ms for t in result.timings]

    x = np.arange(len(methods))
    width = 0.25
    fig, ax = plt.subplots(figsize=figsize, constrained_layout=True)
    ax.bar(x - width, compile_ms, width, label="compile")
    ax.bar(x, warmup_ms, width, label="JIT warm-up")
    ax.bar(x + width, rollout_ms, width, label="rollout")
    ax.set_xticks(x)
    ax.set_xticklabels(methods)
    ax.set_ylabel("Time [ms]")
    ax.set_title(f"Rollout timing ({result.backend}, tf={result.tf}s, dt={result.dt}s)")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    return fig, ax
