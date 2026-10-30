"""Advanced nested-diagram flatten study (dense / deep-chain subnets).

Hypotheses under test:
- NumPy: ``compile(flatten=True)`` wins when nested boxes hide many blocks behind
  ``DiagramSystem.f``.
- JAX: nested vs flatten ``f()`` may be similar after XLA; plan size and compile
  time still differ.

Run from repo root:
  python benchmarks/run_nested_dense_flatten_study.py
  python benchmarks/run_nested_dense_flatten_study.py --quick --nodes 30
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

import numpy as np

from benchmarks.scenarios.nested_dense_network import (
    make_twin_dense_network,
    make_wrapped_deep_chain,
    make_wrapped_dense_network,
    scenario_state,
)
from minilink.core.assembly import flatten_diagram
from minilink.core.compile.compiler import compile_diagram
from minilink.core.compile.evaluators.compiled import body_of
from minilink.core.compile.plan import build_execution_plan
from minilink.core.topology import evaluation_order_of


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


def _time_jax_f(ev, x, u, t: float, n_calls: int) -> float:
    ev.f(x, u, t).block_until_ready()
    t0 = time.perf_counter()
    for _ in range(n_calls):
        ev.f(x, u, t).block_until_ready()
    return (time.perf_counter() - t0) / n_calls * 1e6


def _plan_metrics(diagram, *, flatten: bool) -> dict[str, int]:
    if flatten:
        diagram, _ = flatten_diagram(diagram)
    order = evaluation_order_of(diagram)
    plan = build_execution_plan(diagram)
    n_stateful = sum(
        1 for sys_id in diagram.subsystems if diagram.subsystems[sys_id].n > 0
    )
    return {
        "top_level_blocks": len(diagram.subsystems),
        "port_order_len": len(order),
        "port_ops": len(plan.port_ops),
        "state_ops": len(plan.state_ops),
        "stateful_blocks": n_stateful,
        "signal_dim": plan.signal_dim,
        "state_dim": plan.state_dim,
    }


def _jax_eqn_count(ev, x, u, t: float) -> int | str:
    try:
        import jax
    except ImportError:
        return "no_jax"
    if ev.backend != "jax":
        return "n/a"
    body = body_of(ev, "f")
    params = ev.params
    pr = jax.make_jaxpr(lambda x, u, t: body(x, u, t, params))(x, u, t)
    return len(pr.eqns)


def analyze_diagram(
    name: str, diagram, *, n_calls: int, compile_repeats: int
) -> dict[str, Any]:
    x, u = scenario_state(diagram)
    out: dict[str, Any] = {"name": name, "n_state": diagram.n, "n_input": diagram.m}

    dx_native = diagram.f(x, u, 0.0)
    flat_diag, _ = flatten_diagram(diagram)
    out["parity"] = {
        "flatten_diagram_f_vs_native": _max_abs_diff(flat_diag.f(x, u, 0.0), dx_native),
        "compile_nested_vs_native": None,
        "compile_flatten_vs_native": None,
        "compile_flatten_vs_nested": None,
        "compile_pre_flatten_vs_compile_flag": None,
    }

    ev_nested = compile_diagram(diagram, backend="numpy", flatten=False)
    ev_flatten = compile_diagram(diagram, backend="numpy", flatten=True)
    ev_pre_flat = compile_diagram(flat_diag, backend="numpy", flatten=False)
    p = out["parity"]
    p["compile_nested_vs_native"] = _max_abs_diff(ev_nested.f(x, u, 0.0), dx_native)
    p["compile_flatten_vs_native"] = _max_abs_diff(ev_flatten.f(x, u, 0.0), dx_native)
    p["compile_flatten_vs_nested"] = _max_abs_diff(
        ev_flatten.f(x, u, 0.0), ev_nested.f(x, u, 0.0)
    )
    p["compile_pre_flatten_vs_compile_flag"] = _max_abs_diff(
        ev_pre_flat.f(x, u, 0.0), ev_flatten.f(x, u, 0.0)
    )
    if p["flatten_diagram_f_vs_native"] > 1e-9:
        p["flatten_diagram_warning"] = (
            "flatten_diagram.f differs from native — often multiple nested "
            "boxes with overlapping block ids / state order; treat twin scenario "
            "as a stress test until fixed."
        )

    t0 = time.perf_counter()
    flatten_diagram(diagram)
    out["flatten_diagram_s"] = time.perf_counter() - t0

    out["plan_nested"] = _plan_metrics(diagram, flatten=False)
    out["plan_compile_flatten"] = _plan_metrics(diagram, flatten=True)

    out["compile_s_numpy"] = {
        "nested": _time_compile(
            diagram, flatten=False, backend="numpy", repeats=compile_repeats
        ),
        "flatten_flag": _time_compile(
            diagram, flatten=True, backend="numpy", repeats=compile_repeats
        ),
        "pre_flattened_diagram": _time_compile(
            flat_diag, flatten=False, backend="numpy", repeats=compile_repeats
        ),
    }

    out["f_us_per_call_numpy"] = {
        "native_diagram_f": _time_f(diagram, x, u, 0.0, n_calls),
        "compile_nested": _time_f(ev_nested, x, u, 0.0, n_calls),
        "compile_flatten": _time_f(ev_flatten, x, u, 0.0, n_calls),
        "compile_pre_flattened": _time_f(ev_pre_flat, x, u, 0.0, n_calls),
    }

    for label, num, den in (
        ("nested_over_native", "compile_nested", "native_diagram_f"),
        ("flatten_over_native", "compile_flatten", "native_diagram_f"),
        ("flatten_over_nested", "compile_flatten", "compile_nested"),
    ):
        out["f_us_per_call_numpy"][label] = (
            out["f_us_per_call_numpy"][num] / out["f_us_per_call_numpy"][den]
        )

    try:
        import jax  # noqa: F401

        out["compile_s_jax"] = {
            "nested": _time_compile(
                diagram, flatten=False, backend="jax", repeats=compile_repeats
            ),
            "flatten_flag": _time_compile(
                diagram, flatten=True, backend="jax", repeats=compile_repeats
            ),
        }
        ev_j_nested = compile_diagram(diagram, backend="jax", flatten=False)
        ev_j_flat = compile_diagram(diagram, backend="jax", flatten=True)
        for _ in range(10):
            ev_j_nested.f(x, u, 0.0).block_until_ready()
            ev_j_flat.f(x, u, 0.0).block_until_ready()
        out["f_us_per_call_jax"] = {
            "compile_nested": _time_jax_f(ev_j_nested, x, u, 0.0, n_calls),
            "compile_flatten": _time_jax_f(ev_j_flat, x, u, 0.0, n_calls),
        }
        out["f_us_per_call_jax"]["flatten_over_nested"] = (
            out["f_us_per_call_jax"]["compile_flatten"]
            / out["f_us_per_call_jax"]["compile_nested"]
        )
        out["jax_eqn_count"] = {
            "compile_nested": _jax_eqn_count(ev_j_nested, x, u, 0.0),
            "compile_flatten": _jax_eqn_count(ev_j_flat, x, u, 0.0),
        }
        out["parity"]["jax_flatten_vs_nested"] = _max_abs_diff(
            ev_j_flat.f(x, u, 0.0), ev_j_nested.f(x, u, 0.0)
        )
    except ImportError:
        out["compile_s_jax"] = "no_jax"
        out["f_us_per_call_jax"] = "no_jax"
        out["jax_eqn_count"] = "no_jax"

    return out


def build_scenarios(
    *, nodes: int, connections: int, depth: int
) -> list[tuple[str, Any]]:
    twin_n = max(nodes // 2, 5)
    return [
        (
            f"dense_wrapped_n{nodes}_c{connections}",
            make_wrapped_dense_network(nodes, connections),
        ),
        (
            f"dense_twin_n{twin_n}_c{connections}",
            make_twin_dense_network(twin_n, connections),
        ),
        (f"deep_chain_wrapped_d{depth}", make_wrapped_deep_chain(depth=depth)),
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="Fewer iterations")
    parser.add_argument(
        "--nodes", type=int, default=50, help="Dense network node count"
    )
    parser.add_argument(
        "--connections", type=int, default=5, help="Fan-in per dense node"
    )
    parser.add_argument("--depth", type=int, default=50, help="Deep chain depth")
    args = parser.parse_args(argv)

    n_calls = 5_000 if args.quick else 30_000
    compile_repeats = 3 if args.quick else 5

    report: dict[str, Any] = {
        "n_calls": n_calls,
        "compile_repeats": compile_repeats,
        "nodes": args.nodes,
        "connections": args.connections,
        "depth": args.depth,
        "scenarios": [],
    }

    for name, diagram in build_scenarios(
        nodes=args.nodes, connections=args.connections, depth=args.depth
    ):
        report["scenarios"].append(
            analyze_diagram(
                name, diagram, n_calls=n_calls, compile_repeats=compile_repeats
            )
        )

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
