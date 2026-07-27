"""Compare UR5 forward-dynamics paths: RNEA–H, ABA, and symbolic Lagrange."""

from __future__ import annotations

import time

import numpy as np


def sample_configurations(rng, n_samples):
    """Draw random ``(q, v, u)`` triples within a modest joint range."""
    configs = []
    for _ in range(n_samples):
        q = rng.uniform(-1.0, 1.0, 6)
        v = rng.uniform(-1.0, 1.0, 6)
        u = rng.uniform(-5.0, 5.0, 6)
        configs.append((q, v, u))
    return configs


def forward_dynamics_rnea_h(arm, q, v, u, params):
    """Explicit ``H`` assembly + linear solve (catalog default)."""
    return arm.forward_dynamics(q, v, u, params=params)


def forward_dynamics_aba(arm, q, v, u, params):
    """Articulated-body algorithm (no explicit ``H``)."""
    tau = arm.generalized_force(q, v, u, 0.0, params)
    d = arm.d(q, v, params=params)
    return arm._aba(q, v, tau - d, params, gravity=True)


def forward_dynamics_symbolic(plant, q, v, u):
    """Lagrange ``H`` / ``C`` / ``g`` exported plant (no catalog damping)."""
    return plant.forward_dynamics(q, v, u)


def max_acceleration_error(reference, candidate):
    return float(np.max(np.abs(reference - candidate)))


def time_callable(func, n_repeat, *args, **kwargs):
    """Return median wall time per call in seconds."""
    times = []
    for _ in range(n_repeat):
        t0 = time.perf_counter()
        func(*args, **kwargs)
        times.append(time.perf_counter() - t0)
    return float(np.median(times))


def run_parity_table(
    arm,
    symbolic_plant,
    configs,
    *,
    params,
    label_width=14,
):
    """
    Print max |qdd_ref - qdd| for each technique vs RNEA–H reference.

    Returns summary dict with per-method errors and timing medians.
    """
    ref_label = "RNEA-H"
    methods = [
        ("ABA", lambda q, v, u: forward_dynamics_aba(arm, q, v, u, params)),
    ]
    if symbolic_plant is not None:
        methods.append(
            (
                "Symbolic",
                lambda q, v, u: forward_dynamics_symbolic(symbolic_plant, q, v, u),
            )
        )

    errors = {name: [] for name, _ in methods}
    for q, v, u in configs:
        qdd_ref = forward_dynamics_rnea_h(arm, q, v, u, params)
        for name, fd in methods:
            errors[name].append(max_acceleration_error(qdd_ref, fd(q, v, u)))

    timing = {}
    q, v, u = configs[0]
    timing[ref_label] = time_callable(forward_dynamics_rnea_h, 20, arm, q, v, u, params)
    timing["ABA"] = time_callable(forward_dynamics_aba, 20, arm, q, v, u, params)
    if symbolic_plant is not None:
        timing["Symbolic"] = time_callable(
            forward_dynamics_symbolic, 20, symbolic_plant, q, v, u
        )

    print(f"\n{'method':<{label_width}}  {'max |dqdd|':>12}  {'median ms':>10}")
    print("-" * (label_width + 28))
    print(
        f"{ref_label + ' (ref)':<{label_width}}  {'—':>12}  {1e3 * timing[ref_label]:10.3f}"
    )
    for name, _ in methods:
        err = max(errors[name])
        print(f"{name:<{label_width}}  {err:12.3e}  {1e3 * timing[name]:10.3f}")

    return {"errors": errors, "timing": timing}
