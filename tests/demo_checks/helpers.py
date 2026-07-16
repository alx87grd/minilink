"""Shared helpers for demo-check runners (catalog, demos, graphics)."""

from __future__ import annotations

import os

import numpy as np

from minilink.graphical.animation import Animator


def configure_headless() -> None:
    os.environ.setdefault("MPLBACKEND", "Agg")


def check_dynamics_f(system) -> None:
    """One ``f`` evaluation at ``x0`` with finite output."""
    x = np.asarray(system.x0, dtype=float)
    if x.shape != (system.n,):
        x = np.zeros(system.n)
    u = system.get_u_from_input_ports()
    dx = np.asarray(system.f(x, u, 0.0), dtype=float)
    if dx.shape != (system.n,):
        raise ValueError(
            f"{system.name}: f returned shape {dx.shape}, expected ({system.n},)"
        )
    if not np.all(np.isfinite(dx)):
        raise ValueError(f"{system.name}: f returned non-finite values")


def check_geometry_frame(system, x=None, u=None, t=0.0) -> None:
    """Resolve one kinematic frame; transforms must be finite 4×4."""
    x = np.asarray(system.x0 if x is None else x, dtype=float)
    if x.shape != (system.n,):
        x = np.zeros(system.n)
    u = system.get_u_from_input_ports() if u is None else np.asarray(u, dtype=float)
    animator = Animator(system)
    kinematic = system.get_kinematic_geometry()
    frame = animator._resolve_frame(x, u, t, kinematic=kinematic)
    if len(frame["primitives"]) != len(frame["transforms"]):
        raise ValueError(f"{system.name}: primitive/transform count mismatch")
    for transform in frame["transforms"]:
        mat = np.asarray(transform, dtype=float)
        if mat.shape != (4, 4) or not np.all(np.isfinite(mat)):
            raise ValueError(f"{system.name}: invalid transform matrix")


def check_short_simulation(system, *, tf: float = 0.03, dt: float = 0.01) -> None:
    """Three-step Euler integration (continuous plants only)."""
    from minilink.simulation.simulator import Simulator

    x0 = np.asarray(system.x0, dtype=float)
    if x0.shape != (system.n,):
        x0 = np.zeros(system.n)
    sim = Simulator(system, solver="euler", dt=dt)
    result = sim.run(tf=tf, x0=x0)
    if not np.all(np.isfinite(result.x)):
        raise ValueError(f"{system.name}: simulation state non-finite")


def check_catalog_plant(system, *, fast: bool) -> None:
    check_dynamics_f(system)
    check_geometry_frame(system)
    if not fast:
        check_short_simulation(system)
