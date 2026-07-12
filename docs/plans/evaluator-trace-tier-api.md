# Evaluator execution tiers (JAX)

Architecture reference for compiled evaluators: **fast tier** (default) vs
**trace tier** (JAX-only, pre-JIT). Contracts in [DESIGN.md](../../DESIGN.md) §5;
code under `minilink/core/compile/evaluators/`.

---

## Overview

| Tier | JAX | NumPy |
| --- | --- | --- |
| **Fast (default)** | JIT-compiled (`.f`, `.rk4_step`, …) | Eager Python |
| **Trace** | Pre-JIT flat callables (`.f_trace`, …) | Not exposed |

Trace tier callables compose inside outer `jax.jit` / `grad` / `vmap` (parameter
identification losses, C export). Fast tier stays the simulation default.

NumPy evaluators reject `*_trace` / `*_jit` via `NoTraceTierMixin`.
`has_trace_tier` is `True` on JAX evaluators.

---

## Naming grammar

### Param tier (all backends)

| Suffix | Meaning |
| --- | --- |
| *(none)* | **Bound** — params fixed at compile |
| `_p` | **Parametric** — runtime nested params pytree |

### Execution tier (JAX)

| Suffix | Meaning |
| --- | --- |
| *(none)* | Fast tier (JIT) |
| `_trace` | Trace tier (pre-JIT) |
| `_jit` | Alias of fast tier (optional symmetry) |

### Combination rule

```
{name}              → fast, bound
{name}_p            → fast, parametric
{name}_trace        → trace, bound        (JAX only)
{name}_trace_p      → trace, parametric   (JAX only)
{name}_jit          → alias of {name}     (JAX only)
{name}_jit_p        → alias of {name}_p   (JAX only)
```

Do not use `_jax` (clashes with `compile(backend="jax")`) or `_raw` on evaluators
(reserve for reference `sys.f` and `compile_backend="direct"`).

---

## Public API matrix

### Core dynamics / outputs / step

| Bound fast | Param fast | Bound trace | Param trace |
| --- | --- | --- | --- |
| `f` | `f_p` | `f_trace` | `f_trace_p` |
| `outputs` | `outputs_p` | `outputs_trace` | `outputs_trace_p` |
| `step` | `step_p` | `step_trace` | `step_trace_p` |

Special: `jacobian_f_params` uses JIT `jacfwd` on the trace core.
`f_ivp` / `f_ivp_p` / `f_ivp_scipy` are fast tier (no public `f_ivp_trace`).

### Integration (JAX dynamics evaluators)

See [evaluator-integration-api.md](evaluator-integration-api.md) for the full
`rk4` / `euler` × `zoh` / `linear` / `ivp` × bound / `_p` / `_trace` grid.

| Bound fast | Param fast | Bound trace | Param trace |
| --- | --- | --- | --- |
| `rk4_step` | `rk4_step_p` | `rk4_step_trace` | `rk4_step_trace_p` |
| `euler_step` | `euler_step_p` | `euler_step_trace` | `euler_step_trace_p` |
| `rk4_step_ivp` | `rk4_step_ivp_p` | `rk4_step_ivp_trace` | `rk4_step_ivp_trace_p` |
| `euler_step_ivp` | `euler_step_ivp_p` | `euler_step_ivp_trace` | `euler_step_ivp_trace_p` |
| `rk4_integrate_zoh` | `rk4_integrate_zoh_p` | `rk4_integrate_zoh_trace` | `rk4_integrate_zoh_trace_p` |
| `rk4_integrate_linear` | `rk4_integrate_linear_p` | `rk4_integrate_linear_trace` | `rk4_integrate_linear_trace_p` |
| `rk4_integrate_ivp` | `rk4_integrate_ivp_p` | `rk4_integrate_ivp_trace` | `rk4_integrate_ivp_trace_p` |
| `euler_integrate_zoh` | `euler_integrate_zoh_p` | `euler_integrate_zoh_trace` | `euler_integrate_zoh_trace_p` |
| `euler_integrate_ivp` | `euler_integrate_ivp_p` | `euler_integrate_ivp_trace` | `euler_integrate_ivp_trace_p` |

NumPy: `IntegrationMixin` Python loops on `self.f` / `self.f_p`.

JAX fast tier: scanned rollouts (lazy JIT on first call); single-step helpers call JIT `f`.
JAX trace tier: RK4/Euler call `f_trace` / `f_trace_p`; rollout trace helpers fuse
one scan for AD.

### Not exposed on JAX (gaps)

| Surface | Notes |
| --- | --- |
| `integrate_zoh_trace` | Sugar over RK4 ZOH; deferred |
| `rollout`, `rollout_p` (step) | Fast `_rollout_jit_fn` scan; state-only — log via :class:`~minilink.simulation.computer.Computer` / hybrid sim |
| `compute_internal_signals` (diagram) | JIT fast tier; no `compute_internal_signals_trace` |
| `f_scipy`, `as_scipy_*` | Fast tier bridges only |

---

## Code organization

```
minilink/core/compile/evaluators/
  evaluators.py         # ABCs: Output / Dynamics / Step / Static
  numpy_evaluators.py   # all Numpy* + IntegrationMixin
  jax_evaluators.py     # all Jax* + JaxIntegrationMixin + helpers
  step_rollout.py       # gather_u + StepRolloutMixin (shared NumPy/JAX)
  tiers.py              # TraceTierMixin, NoTraceTierMixin, register_jit_aliases
```

### Internal naming

**Leaf and diagram evaluators** share one convention:

- Fast: `_f_jit_fn`, `_step_jit_fn`, `_outputs_jit_fn`, `_rollout_jit_fn`, …
- Trace: `_f_trace_fn`, `_step_trace_fn`, `_outputs_trace_fn`, …

Leaf tiers are built via ``build_dynamic_leaf_tiers`` / ``build_step_leaf_tiers``.
Diagram evaluators define trace closures as methods, then ``jax.jit`` them at compile time.

### Pattern (leaf and diagram)

```python
# compile time
self._f_jit_fn = jax.jit(...)
self._f_trace_fn = _trace_core

def f(self, x, u, t=0.0):
    return self._f_jit_fn(x, u, t)

def f_trace(self, x, u, t=0.0):
    return self._f_trace_fn(x, u, t)
```

Integration: trace closures at setup; fast rollouts `jax.jit` lazily on first call.
No compile-time dummy warm-start.

---

## Consumers

| Module | Usage |
| --- | --- |
| `interfaces/c_export.py` | `f_trace` / `step_trace`; `disable_jit` only around `make_jaxpr` when subsystem `f()` embeds `jax.jit` |
| `planning/.../transcription.py` | `dynamics_function` → `f_trace` / `f_trace_p` when `has_trace_tier` |
| Examples / MPC / benchmarks | Fast tier (`f`, `rk4_step`, …) — unchanged |

**Trajopt:** JAX NLP transcriptions embed `problem.sys.f` directly (not evaluator
`f` / `f_trace`). `dynamics_function` trace tier applies to NumPy transcribe and
`reconstruct_result` only.

---

## Testing

`tests/unittest/test_evaluator_tiers.py`:

- Fast vs trace numeric parity
- `f_jit` class alias ≡ `f`
- NumPy `f_trace` / `rk4_step_trace` → `AttributeError`
- `jax.jit(jax.grad(loss))` with `f_trace_p`
- Integration trace parity

C-export path: manual / `demo_c_export`.

---

## Compatibility

- No breaking changes to existing fast-tier signatures for `f` / `outputs` / `step`.
- Integration rollouts renamed: `integrate` → `rk4_integrate_zoh`,
  `rk4_integrate_forced` → `rk4_integrate_linear` (clean rename).
- `get_f_jit` removed; use `f` or `f_jit`.
- Missing grid cells: no attribute or explicit `NotImplementedError` with reason.

---

## Future work

| Item | Notes |
| --- | --- |
| `rollout_trace`, `rollout_trace_p` | Scan over `step_trace` |
| `compute_internal_signals_trace` | Thin wiring to `_internal_signals_trace_fn` |
| `integrate_zoh_trace` | JAX ZOH sugar scan |
| `benchmarks/jax_evaluator_tiers.py` | Fast vs trace AD benchmarks |

---

## Decision log

| Decision | Rationale |
| --- | --- |
| Default `.f` = fast tier | Simulation-first UX |
| `_trace` suffix | Short; scales to long integration names |
| `_jit` aliases optional | Document fast tier without breaking `f` |
| No `_trace` on NumPy | Clear backend guard |
| Param suffix always last | Matches `f_p`, `outputs_p` |
| Unified `_trace_fn` / `_jit_fn` internals | Same naming for leaf and diagram evaluators |
| No compile-time warm-start | First real call pays XLA; lazy rollout JIT |
