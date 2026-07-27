"""Compare UR5 forward-dynamics paths: RNEA–H, ABA, and symbolic Lagrange."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

DEFAULT_SEED = 0
DEFAULT_N_SAMPLES = 64
DEFAULT_N_TIMING = 200
DEFAULT_Q_RANGE = (-1.0, 1.0)
DEFAULT_V_RANGE = (-1.0, 1.0)
DEFAULT_U_RANGE = (-5.0, 5.0)


@dataclass
class MethodStats:
    """Aggregate acceleration error statistics vs RNEA–H."""

    name: str
    per_sample_max: np.ndarray
    per_joint_max: np.ndarray | None = None

    @property
    def max(self) -> float:
        return float(np.max(self.per_sample_max))

    @property
    def mean(self) -> float:
        return float(np.mean(self.per_sample_max))

    @property
    def rms(self) -> float:
        return float(np.sqrt(np.mean(self.per_sample_max**2)))

    @property
    def p95(self) -> float:
        return float(np.percentile(self.per_sample_max, 95))


@dataclass
class ComparisonResult:
    """Structured output from :func:`comprehensive_comparison`."""

    seed: int
    n_samples: int
    configs: list[tuple[np.ndarray, np.ndarray, np.ndarray]]
    case_labels: list[str]
    methods: dict[str, MethodStats] = field(default_factory=dict)
    timing_ms: dict[str, float] = field(default_factory=dict)
    timing_per_config_ms: dict[str, np.ndarray] = field(default_factory=dict)


def sample_configurations(
    rng,
    n_samples,
    *,
    q_range=DEFAULT_Q_RANGE,
    v_range=DEFAULT_V_RANGE,
    u_range=DEFAULT_U_RANGE,
):
    """Draw random ``(q, v, u)`` triples within the given ranges."""
    configs = []
    q_lo, q_hi = q_range
    v_lo, v_hi = v_range
    u_lo, u_hi = u_range
    for _ in range(n_samples):
        q = rng.uniform(q_lo, q_hi, 6)
        v = rng.uniform(v_lo, v_hi, 6)
        u = rng.uniform(u_lo, u_hi, 6)
        configs.append((q, v, u))
    return configs


def named_case_studies():
    """Fixed UR5 configurations for interpretable spot checks."""
    return [
        (
            "gravity hold (v=0)",
            np.array([0.0, -np.pi / 2 + 0.2, 0.0, -np.pi / 2, 0.0, 0.0]),
            np.zeros(6),
            np.zeros(6),
        ),
        (
            "mid-range motion",
            np.linspace(-0.4, 0.5, 6),
            np.linspace(0.3, -0.2, 6),
            np.linspace(1.0, -2.0, 6),
        ),
        (
            "elbow extended",
            np.array([0.2, -0.3, 0.5, -1.2, 0.4, -0.6]),
            np.array([0.4, -0.5, 0.1, 0.2, -0.3, 0.15]),
            np.array([2.0, -3.0, 1.0, 0.5, -1.0, 0.25]),
        ),
    ]


def build_evaluation_batch(seed=DEFAULT_SEED, n_random=DEFAULT_N_SAMPLES):
    """
    Named case studies followed by a reproducible random batch.

    Returns ``(configs, labels)`` where ``labels[i]`` describes sample *i*.
    """
    rng = np.random.default_rng(seed)
    cases = named_case_studies()
    configs = [(q, v, u) for _, q, v, u in cases]
    labels = [name for name, _, _, _ in cases]
    random_configs = sample_configurations(rng, n_random)
    configs.extend(random_configs)
    labels.extend([f"random {i:03d}" for i in range(n_random)])
    return configs, labels


def forward_dynamics_rnea_h(arm, q, v, u, params):
    """Explicit ``H`` assembly + linear solve (catalog default)."""
    return arm.forward_dynamics(q, v, u, params=params)


def forward_dynamics_aba(arm, q, v, u, params):
    """Articulated-body algorithm (no explicit ``H``)."""
    return arm.forward_dynamics_aba(q, v, u, params=params)


def forward_dynamics_symbolic(plant, q, v, u):
    """Lagrange ``H`` / ``C`` / ``g`` exported plant (no catalog damping)."""
    return plant.forward_dynamics(q, v, u)


def max_acceleration_error(reference, candidate):
    return float(np.max(np.abs(reference - candidate)))


def per_joint_error(reference, candidate):
    return np.abs(reference - candidate)


def _warmup(arm, symbolic_plant, q, v, u, params):
    forward_dynamics_rnea_h(arm, q, v, u, params)
    forward_dynamics_aba(arm, q, v, u, params)
    if symbolic_plant is not None:
        forward_dynamics_symbolic(symbolic_plant, q, v, u)


def benchmark_method(func, configs, n_repeat, *args):
    """
    Median wall time per call [s] cycling through *configs*.

    Each repeat evaluates one configuration from the list.
    """
    n_cfg = len(configs)
    times = []
    for k in range(n_repeat):
        q, v, u = configs[k % n_cfg]
        t0 = time.perf_counter()
        func(q, v, u, *args)
        times.append(time.perf_counter() - t0)
    return float(np.median(times))


def benchmark_method_per_config(func, configs, n_repeat_per_config, *args):
    """Median wall time [s] for each configuration."""
    medians = []
    for q, v, u in configs:
        times = []
        for _ in range(n_repeat_per_config):
            t0 = time.perf_counter()
            func(q, v, u, *args)
            times.append(time.perf_counter() - t0)
        medians.append(float(np.median(times)))
    return np.asarray(medians)


def comprehensive_comparison(
    arm,
    symbolic_plant,
    configs,
    *,
    params,
    seed=DEFAULT_SEED,
    n_timing=DEFAULT_N_TIMING,
    case_labels=None,
):
    """
    Compare ABA and symbolic Lagrange against RNEA–H on many ``(q, v, u)`` samples.

    Returns a :class:`ComparisonResult` with per-sample and aggregate statistics.
    """
    if case_labels is None:
        case_labels = [f"sample {i:03d}" for i in range(len(configs))]

    method_errors = {}
    if symbolic_plant is not None:
        method_errors["Symbolic"] = {"max": [], "per_joint": []}
    method_errors["ABA"] = {"max": [], "per_joint": []}

    for q, v, u in configs:
        qdd_ref = forward_dynamics_rnea_h(arm, q, v, u, params)
        qdd_aba = forward_dynamics_aba(arm, q, v, u, params)
        method_errors["ABA"]["max"].append(max_acceleration_error(qdd_ref, qdd_aba))
        method_errors["ABA"]["per_joint"].append(per_joint_error(qdd_ref, qdd_aba))

        if symbolic_plant is not None:
            qdd_sym = forward_dynamics_symbolic(symbolic_plant, q, v, u)
            method_errors["Symbolic"]["max"].append(
                max_acceleration_error(qdd_ref, qdd_sym)
            )
            method_errors["Symbolic"]["per_joint"].append(
                per_joint_error(qdd_ref, qdd_sym)
            )

    q0, v0, u0 = configs[0]
    _warmup(arm, symbolic_plant, q0, v0, u0, params)

    timing_ms = {
        "RNEA-H": 1e3
        * benchmark_method(
            lambda q, v, u, p: forward_dynamics_rnea_h(arm, q, v, u, p),
            configs,
            n_timing,
            params,
        ),
        "ABA": 1e3
        * benchmark_method(
            lambda q, v, u, p: forward_dynamics_aba(arm, q, v, u, p),
            configs,
            n_timing,
            params,
        ),
    }
    if symbolic_plant is not None:
        timing_ms["Symbolic"] = 1e3 * benchmark_method(
            lambda q, v, u: forward_dynamics_symbolic(symbolic_plant, q, v, u),
            configs,
            n_timing,
        )

    timing_per_config = {
        "RNEA-H": 1e3
        * benchmark_method_per_config(
            lambda q, v, u, p: forward_dynamics_rnea_h(arm, q, v, u, p),
            configs,
            max(3, n_timing // len(configs)),
            params,
        ),
        "ABA": 1e3
        * benchmark_method_per_config(
            lambda q, v, u, p: forward_dynamics_aba(arm, q, v, u, p),
            configs,
            max(3, n_timing // len(configs)),
            params,
        ),
    }
    if symbolic_plant is not None:
        timing_per_config["Symbolic"] = 1e3 * benchmark_method_per_config(
            lambda q, v, u: forward_dynamics_symbolic(symbolic_plant, q, v, u),
            configs,
            max(3, n_timing // len(configs)),
        )

    methods = {}
    for name, data in method_errors.items():
        per_joint = np.stack(data["per_joint"], axis=0)
        methods[name] = MethodStats(
            name=name,
            per_sample_max=np.asarray(data["max"]),
            per_joint_max=np.max(per_joint, axis=0),
        )

    return ComparisonResult(
        seed=seed,
        n_samples=len(configs),
        configs=configs,
        case_labels=case_labels,
        methods=methods,
        timing_ms=timing_ms,
        timing_per_config_ms=timing_per_config,
    )


def format_stats_row(stats: MethodStats, timing_ms: float, label_width=12) -> str:
    return (
        f"{stats.name:<{label_width}}  "
        f"{stats.max:12.3e}  {stats.mean:12.3e}  {stats.rms:12.3e}  "
        f"{stats.p95:12.3e}  {timing_ms:10.3f}"
    )


def print_comprehensive_report(result: ComparisonResult, *, label_width=12):
    """Human-readable accuracy and timing summary."""
    print(f"Seed = {result.seed}, samples = {result.n_samples}")
    print("Reference: RNEA–H (catalog forward_dynamics)\n")
    header = (
        f"{'method':<{label_width}}  {'max':>12}  {'mean':>12}  "
        f"{'rms':>12}  {'p95':>12}  {'median ms':>10}"
    )
    print(header)
    print("-" * len(header))
    print(
        f"{'RNEA-H (ref)':<{label_width}}  "
        f"{'0':>12}  {'0':>12}  {'0':>12}  {'0':>12}  "
        f"{result.timing_ms['RNEA-H']:10.3f}"
    )
    for name, stats in result.methods.items():
        print(format_stats_row(stats, result.timing_ms[name], label_width))

    print("\nNamed case studies (max |Δqdd| vs RNEA–H):")
    n_named = min(3, result.n_samples)
    for i in range(n_named):
        label = result.case_labels[i]
        aba_err = result.methods["ABA"].per_sample_max[i]
        line = f"  {label:<28}  ABA {aba_err:.3e}"
        if "Symbolic" in result.methods:
            sym_err = result.methods["Symbolic"].per_sample_max[i]
            line += f"  Symbolic {sym_err:.3e}"
        print(line)

    speedup = result.timing_ms["RNEA-H"] / max(result.timing_ms["ABA"], 1e-12)
    print(f"\nABA median speedup vs RNEA–H: {speedup:.2f}×")


def plot_comparison(result: ComparisonResult, *, figsize=(10, 4)):
    """
    Matplotlib figures: per-sample errors and median timings.

    Returns ``(fig_err, fig_time)``.
    """
    import matplotlib.pyplot as plt

    n_random = result.n_samples - min(3, result.n_samples)
    random_start = result.n_samples - n_random

    fig_err, axes = plt.subplots(1, 2, figsize=figsize, constrained_layout=True)

    x = np.arange(n_random)
    axes[0].semilogy(x, result.methods["ABA"].per_sample_max[random_start:], label="ABA")
    if "Symbolic" in result.methods:
        axes[0].semilogy(
            x,
            result.methods["Symbolic"].per_sample_max[random_start:],
            label="Symbolic",
            alpha=0.85,
        )
    axes[0].set_xlabel("Random sample index")
    axes[0].set_ylabel("max |Δqdd| vs RNEA–H")
    axes[0].set_title(f"Random batch (seed={result.seed}, n={n_random})")
    axes[0].legend()
    axes[0].grid(True, which="both", alpha=0.3)

    names = ["RNEA–H", "ABA"]
    times = [result.timing_ms["RNEA–H"], result.timing_ms["ABA"]]
    if "Symbolic" in result.methods:
        names.append("Symbolic")
        times.append(result.timing_ms["Symbolic"])

    axes[1].bar(names, times, color=["#4c72b0", "#55a868", "#c44e52"][: len(names)])
    axes[1].set_ylabel("Median time per FD call [ms]")
    axes[1].set_title("Forward dynamics timing")
    axes[1].grid(True, axis="y", alpha=0.3)

    return fig_err, axes


# Backward-compatible alias used by early run_demo.
run_parity_table = print_comprehensive_report
