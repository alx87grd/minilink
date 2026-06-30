"""Shared helpers for graphics contract tests (tf + Animator frame resolution)."""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation import Animator


def resolve_draw_frame(system, x=None, u=None, t=0.0):
    x = np.asarray(system.x0 if x is None else x, dtype=float)
    if x.shape != (system.n,):
        x = np.zeros(system.n)
    u = system.get_u_from_input_ports() if u is None else np.asarray(u, dtype=float)
    animator = Animator(system)
    kinematic = system.get_kinematic_geometry()
    return animator._resolve_frame(x, u, t, kinematic=kinematic)


def geometry_smoke(system, x=None, u=None, t=0.0):
    frame = resolve_draw_frame(system, x, u, t)
    assert len(frame["primitives"]) == len(frame["transforms"])
    for transform in frame["transforms"]:
        transform = np.asarray(transform, dtype=float)
        assert transform.shape == (4, 4)
        assert np.all(np.isfinite(transform))


def resolved_primitive_count(system, primitive_type, x=None, u=None, t=0.0):
    frame = resolve_draw_frame(system, x, u, t)
    return sum(
        isinstance(primitive, primitive_type) for primitive in frame["primitives"]
    )
