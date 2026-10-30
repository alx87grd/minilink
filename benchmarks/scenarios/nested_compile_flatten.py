"""Nested diagram compile: nested vs flatten=True vs hand-flat (informational timings)."""

from __future__ import annotations

import time

import numpy as np

from minilink.blocks.routing import Gain
from minilink.control.siso import PID
from minilink.core.compile.compiler import compile_diagram
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.mass_spring_damper.linear import SingleMass


def _nested_and_flat_outer_loops():
    """Same nested vs hand-flat closed loop as the unit-test fixture."""
    pid = PID(Kp=5.0, Ki=2.0, Kd=1.0, ports="reference")
    mass = SingleMass()
    outer = Gain([-0.5])

    inner = DiagramSystem()
    inner.add_subsystem(pid, "controller")
    inner.add_subsystem(mass, "plant")
    inner.add_input_port("r")
    inner.connect("input", "r", "controller", "r")
    inner.connect("plant", "y", "controller", "y")
    inner.connect("controller", "u", "plant", "u")
    inner.connect_new_output_port("plant", "y", "y")

    nested = DiagramSystem()
    nested.add_subsystem(outer, "outer")
    nested.add_subsystem(inner, "inner")
    nested.connect("inner", "y", "outer", "u")
    nested.connect("outer", "y", "inner", "r")

    flat = DiagramSystem()
    flat.add_subsystem(outer, "outer")
    flat.add_subsystem(pid, "controller")
    flat.add_subsystem(mass, "plant")
    flat.connect("plant", "y", "outer", "u")
    flat.connect("outer", "y", "controller", "r")
    flat.connect("plant", "y", "controller", "y")
    flat.connect("controller", "u", "plant", "u")

    mass.x0 = np.array([1.0, 0.0])
    return nested, flat


def run_nested_compile_flatten_benchmark(
    *, n_calls: int = 5_000
) -> dict[str, float | str]:
    """Compile and ``f`` micro-benchmark for nested vs compile-time flatten."""
    nested, flat = _nested_and_flat_outer_loops()
    x = np.array([0.3, -0.2, 0.5, 0.1])
    u = np.zeros(0)
    rows: dict[str, float | str] = {}

    for label, diagram, flatten in (
        ("nested", nested, False),
        ("compile_flatten", nested, True),
        ("hand_flat", flat, False),
    ):
        t0 = time.perf_counter()
        ev = compile_diagram(diagram, backend="numpy", flatten=flatten)
        rows[f"{label}.compile_s"] = time.perf_counter() - t0
        warm = ev.f(x, u, 0.0)
        t1 = time.perf_counter()
        for _ in range(n_calls):
            ev.f(x, u, 0.0)
        rows[f"{label}.f_us_per_call"] = (time.perf_counter() - t1) / n_calls * 1e6
        rows[f"{label}.f_sample"] = float(np.asarray(warm).reshape(-1)[0])

    return rows


def print_nested_compile_flatten_benchmark(**kwargs) -> None:
    rows = run_nested_compile_flatten_benchmark(**kwargs)
    for key in sorted(rows):
        print(f"{key}: {rows[key]}")
