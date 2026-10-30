"""Parity and speed study: nested compile vs compile(flatten=True) vs hand-flat.

Run from repo root:
  python benchmarks/run_nested_flatten_study.py
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from minilink.blocks.routing import Gain
from minilink.control.siso import PID
from minilink.core.assembly import flatten_diagram, remap_params_nested_to_flat
from minilink.core.compile.compiler import compile_diagram
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.mass_spring_damper.linear import SingleMass


@dataclass(frozen=True)
class Scenario:
    name: str
    nested: DiagramSystem
    hand_flat: DiagramSystem | None
    x: np.ndarray
    u: np.ndarray


def build_simple_scenario() -> Scenario:
    """One nested loop under an outer gain (unit-test fixture)."""
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
    x = np.array([0.3, -0.2, 0.5, 0.1])
    return Scenario("simple_one_level", nested, flat, x, np.zeros(0))


def _three_level_nested_outer() -> DiagramSystem:
    """Three-level nest from ``test_deep_nest_flatten_compile`` (native ``f`` verified)."""
    pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, ports="reference")
    mass = SingleMass()
    inner = DiagramSystem()
    inner.add_subsystem(pid, "controller")
    inner.add_subsystem(mass, "plant")
    inner.add_input_port("r")
    inner.connect("input", "r", "controller", "r")
    inner.connect("plant", "y", "controller", "y")
    inner.connect("controller", "u", "plant", "u")
    inner.connect_new_output_port("plant", "y", "y")

    mid = DiagramSystem()
    mid.add_subsystem(Gain([1.0]), "gain")
    mid.add_subsystem(inner, "loop")
    mid.connect("loop", "y", "gain", "u")
    mid.connect_new_output_port("gain", "y", "y")
    mid.add_input_port("r")
    mid.connect("input", "r", "loop", "r")

    outer = DiagramSystem()
    outer.add_subsystem(Gain([0.5]), "scale")
    outer.add_subsystem(mid, "box")
    outer.connect("box", "y", "scale", "u")
    outer.connect("scale", "y", "box", "r")
    return outer


def _three_level_with_open_boundary() -> DiagramSystem:
    """Three-level mid/inner core with boundary ``r``/``y`` (for a fourth wrap)."""
    pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, ports="reference")
    mass = SingleMass()
    inner = DiagramSystem()
    inner.add_subsystem(pid, "controller")
    inner.add_subsystem(mass, "plant")
    inner.add_input_port("r")
    inner.connect("input", "r", "controller", "r")
    inner.connect("plant", "y", "controller", "y")
    inner.connect("controller", "u", "plant", "u")
    inner.connect_new_output_port("plant", "y", "y")

    mid = DiagramSystem()
    mid.add_subsystem(Gain([1.0]), "gain")
    mid.add_subsystem(inner, "loop")
    mid.connect("loop", "y", "gain", "u")
    mid.connect_new_output_port("gain", "y", "y")
    mid.add_input_port("r")
    mid.connect("input", "r", "loop", "r")

    shell = DiagramSystem()
    shell.add_subsystem(Gain([0.5]), "scale")
    shell.add_subsystem(mid, "box")
    shell.connect("box", "y", "scale", "u")
    shell.add_input_port("r")
    shell.connect("input", "r", "box", "r")
    shell.connect_new_output_port("scale", "y", "y")
    return shell


def _wrap_diagram(
    diagram: DiagramSystem, *, gain_value: float, gain_id: str
) -> DiagramSystem:
    """Outer gain shell around a diagram that exposes ``r`` and ``y``."""
    shell = DiagramSystem()
    shell.add_subsystem(Gain([gain_value]), gain_id)
    shell.add_subsystem(diagram, "box")
    shell.connect("box", "y", gain_id, "u")
    shell.connect_new_output_port(gain_id, "y", "y")
    shell.add_input_port("r")
    shell.connect("input", "r", "box", "r")
    return shell


def build_deep_scenario(*, levels: int = 3) -> Scenario:
    """Multi-level nesting: 3 = closed test outer; 4–5 = extra gain shells."""
    if levels < 3 or levels > 5:
        raise ValueError("levels must be 3, 4, or 5")
    if levels == 3:
        diagram = _three_level_nested_outer()
        x = np.array([0.2, 0.1, 0.0, 0.0])
        u = np.zeros(0)
    else:
        diagram = _three_level_with_open_boundary()
        for idx, gain in enumerate([0.75, 0.6][: levels - 3], start=4):
            diagram = _wrap_diagram(diagram, gain_value=gain, gain_id=f"g{idx}")
        x = np.array([0.2, 0.1, 0.0, 0.0])
        u = np.array([0.0])
    return Scenario(f"deep_{levels}_levels", diagram, None, x, u)


def _max_abs_diff(a, b) -> float:
    return float(
        np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))
    )


def _time_compile(diagram, *, flatten: bool, backend: str, repeats: int) -> float:
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        compile_diagram(diagram, backend=backend, flatten=flatten)
        times.append(time.perf_counter() - t0)
    return float(np.median(times))


def _time_f(ev, x, u, t: float, n_calls: int) -> float:
    ev.f(x, u, t)
    t0 = time.perf_counter()
    for _ in range(n_calls):
        ev.f(x, u, t)
    return (time.perf_counter() - t0) / n_calls * 1e6


def run_parity(scenario: Scenario) -> dict[str, Any]:
    nested = scenario.nested
    x, u = scenario.x, scenario.u
    out: dict[str, Any] = {"scenario": scenario.name, "n_state": nested.n}

    dx_native = nested.f(x, u, 0.0)
    flat_copy, report = flatten_diagram(nested)
    out["flatten_id_map"] = report.id_map
    out["top_level_blocks_before"] = list(nested.subsystems)
    out["top_level_blocks_after"] = list(flat_copy.subsystems)

    out["parity"] = {
        "flatten_diagram_f_vs_native": _max_abs_diff(flat_copy.f(x, u, 0.0), dx_native),
        "compile_nested_vs_native": None,
        "compile_flatten_vs_native": None,
        "compile_flatten_vs_compile_nested": None,
        "compile_flatten_vs_hand_flat": None,
        "compile_nested_vs_hand_flat": None,
        "params_flatten_vs_native": None,
        "rollout_max_x_diff_flatten_vs_native": None,
        "signals_y_flatten_vs_native": None,
    }

    ev_nested = compile_diagram(nested, backend="numpy", flatten=False)
    ev_flatten = compile_diagram(nested, backend="numpy", flatten=True)
    dx_nested = ev_nested.f(x, u, 0.0)
    dx_flatten = ev_flatten.f(x, u, 0.0)
    p = out["parity"]
    p["compile_nested_vs_native"] = _max_abs_diff(dx_nested, dx_native)
    p["compile_flatten_vs_native"] = _max_abs_diff(dx_flatten, dx_native)
    p["compile_flatten_vs_compile_nested"] = _max_abs_diff(dx_flatten, dx_nested)

    if scenario.hand_flat is not None:
        ev_hand = compile_diagram(scenario.hand_flat, backend="numpy", flatten=False)
        dx_hand = ev_hand.f(x, u, 0.0)
        p["compile_flatten_vs_hand_flat"] = _max_abs_diff(dx_flatten, dx_hand)
        p["compile_nested_vs_hand_flat"] = _max_abs_diff(dx_nested, dx_hand)

    nested_params = nested.params
    flat_params = remap_params_nested_to_flat(nested, nested_params)
    p["params_flatten_vs_native"] = _max_abs_diff(
        ev_flatten.f(x, u, 0.0, params=flat_params),
        nested.f(x, u, 0.0, params=nested_params),
    )

    traj_native = nested.compute_trajectory(tf=0.5, dt=0.01, verbose=False)
    traj_flatten = compile_diagram(nested, flatten=True).rollout(
        x, None, 0.0, 0.01, n_steps=len(traj_native.t) - 1
    )
    x_roll = np.asarray(traj_flatten, dtype=float)
    x_ref = np.asarray(traj_native.x, dtype=float)
    if x_ref.shape != x_roll.shape and x_ref.shape == x_roll.T.shape:
        x_ref = x_ref.T
    p["rollout_max_x_diff_flatten_vs_native"] = _max_abs_diff(x_roll, x_ref)

    sig_native = ev_nested.signals(x, u, 0.0)
    sig_flatten = ev_flatten.signals(x, u, 0.0)
    if "inner:controller:u" in sig_native and "inner__controller:u" in sig_flatten:
        p["signals_y_flatten_vs_native"] = _max_abs_diff(
            sig_flatten["inner__controller:u"], sig_native["inner:controller:u"]
        )

    try:
        import jax  # noqa: F401

        ev_j_nested = compile_diagram(nested, backend="jax", flatten=False)
        ev_j_flat = compile_diagram(nested, backend="jax", flatten=True)
        p["jax_compile_flatten_vs_nested"] = _max_abs_diff(
            ev_j_flat.f(x, u, 0.0), ev_j_nested.f(x, u, 0.0)
        )
    except ImportError:
        p["jax_compile_flatten_vs_nested"] = "skipped_no_jax"

    return out


def run_speed(
    scenario: Scenario,
    *,
    n_calls: int,
    compile_repeats: int,
) -> dict[str, Any]:
    nested = scenario.nested
    x, u = scenario.x, scenario.u
    speed: dict[str, Any] = {"scenario": scenario.name, "n_calls": n_calls}

    t0 = time.perf_counter()
    flatten_diagram(nested)
    speed["flatten_diagram_s"] = time.perf_counter() - t0

    speed["compile_s"] = {
        "nested": _time_compile(
            nested, flatten=False, backend="numpy", repeats=compile_repeats
        ),
        "compile_flatten": _time_compile(
            nested, flatten=True, backend="numpy", repeats=compile_repeats
        ),
    }
    if scenario.hand_flat is not None:
        speed["compile_s"]["hand_flat"] = _time_compile(
            scenario.hand_flat, flatten=False, backend="numpy", repeats=compile_repeats
        )

    ev_native = nested
    ev_nested = compile_diagram(nested, flatten=False)
    ev_flatten = compile_diagram(nested, flatten=True)
    speed["f_us_per_call_numpy"] = {
        "native_recursive": _time_f(ev_native, x, u, 0.0, n_calls),
        "compile_nested": _time_f(ev_nested, x, u, 0.0, n_calls),
        "compile_flatten": _time_f(ev_flatten, x, u, 0.0, n_calls),
    }
    if scenario.hand_flat is not None:
        ev_hand = compile_diagram(scenario.hand_flat, flatten=False)
        speed["f_us_per_call_numpy"]["compile_hand_flat"] = _time_f(
            ev_hand, x, u, 0.0, n_calls
        )

    try:
        import jax  # noqa: F401

        ev_j_nested = compile_diagram(nested, backend="jax", flatten=False)
        ev_j_flat = compile_diagram(nested, backend="jax", flatten=True)
        for _ in range(20):
            ev_j_nested.f(x, u, 0.0).block_until_ready()
            ev_j_flat.f(x, u, 0.0).block_until_ready()
        speed["f_us_per_call_jax"] = {
            "compile_nested": _time_jax_f(ev_j_nested, x, u, 0.0, n_calls),
            "compile_flatten": _time_jax_f(ev_j_flat, x, u, 0.0, n_calls),
        }
        speed["compile_s"]["jax_nested"] = _time_compile(
            nested, flatten=False, backend="jax", repeats=max(2, compile_repeats // 2)
        )
        speed["compile_s"]["jax_compile_flatten"] = _time_compile(
            nested, flatten=True, backend="jax", repeats=max(2, compile_repeats // 2)
        )
    except ImportError:
        speed["f_us_per_call_jax"] = "skipped_no_jax"

    return speed


def _time_jax_f(ev, x, u, t, n_calls: int) -> float:
    ev.f(x, u, t).block_until_ready()
    t0 = time.perf_counter()
    for _ in range(n_calls):
        ev.f(x, u, t).block_until_ready()
    return (time.perf_counter() - t0) / n_calls * 1e6


def main() -> int:
    n_calls = 50_000 if "--quick" not in sys.argv else 5_000
    compile_repeats = 7 if "--quick" not in sys.argv else 3
    scenarios = [
        build_simple_scenario(),
        build_deep_scenario(levels=3),
        build_deep_scenario(levels=4),
        build_deep_scenario(levels=5),
    ]
    report: dict[str, Any] = {"n_calls": n_calls, "compile_repeats": compile_repeats}
    report["parity"] = [run_parity(s) for s in scenarios]
    report["speed"] = [
        run_speed(s, n_calls=n_calls, compile_repeats=compile_repeats)
        for s in scenarios
    ]
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
