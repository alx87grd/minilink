# Evaluator integration API — naming and compile plan

Architecture plan for dynamics evaluator **integration** methods (RK4 / Euler steps
and rollouts). Complements the trace-tier contract in
[evaluator-trace-tier-api.md](evaluator-trace-tier-api.md). Contracts in code:
[DESIGN.md](../../DESIGN.md) §5.

Status: **approved plan** (not yet implemented in code).

---

## Problem

Today’s names mix three axes:

| Axis | Example confusion |
| --- | --- |
| Integrator | `integrate` implies generic; it is RK4 |
| Input over Δt | `integrate` = ZOH; `rk4_integrate_forced` = linear between knots |
| Scope | `rk4_step` vs rollout methods |

Both `integrate` and `rk4_integrate_forced` use **RK4**; the difference is **ZOH vs
linear `u`**, not integrator order.

---

## Naming grammar

```
{rk4|euler}_{step|integrate}_{zoh|linear|ivp}[_{trace|trace_p|p}]
```

| Token | Values |
| --- | --- |
| Integrator | `rk4`, `euler` |
| Scope | `step` (one Δt), `integrate` (rollout) |
| Input model | `zoh` (hold `u_k`), `linear` (knots `u_k→u_{k+1}`), `ivp` (nominal `u` at compile) |
| Param | `_p` — runtime params pytree (always last) |
| Trace | `_trace`, `_trace_p` — **JAX only** (pre-JIT for outer `jit` / `grad`) |

**Aliases:** `rk4_step` ≡ `rk4_step_zoh`, `euler_step` ≡ `euler_step_zoh`.

**Deprecated (one release):** `integrate` → `rk4_integrate_zoh`,
`rk4_integrate_forced` → `rk4_integrate_linear` (+ trace/`_p` siblings).

### Intentionally absent

| Surface | Why |
| --- | --- |
| `*_step_linear`, `euler_integrate_linear` | Linear `u` needs `u_{k+1}` — rollout-only, RK4 only |
| NumPy `*_trace` | `NoTraceTierMixin` rejects trace tier |

IVP `_p` means **parametric plant params** with **nominal `u` fixed at compile**:
`f_ivp_p(x, t, params) ≡ f_p(x, u_nom, t, params)`.

---

## Full API (target)

### Single-step

| Bound (fast) | Param (`_p`) | Trace (JAX) | Trace param (JAX) |
| --- | --- | --- | --- |
| `rk4_step_zoh` | `rk4_step_zoh_p` | `rk4_step_zoh_trace` | `rk4_step_zoh_trace_p` |
| `euler_step_zoh` | `euler_step_zoh_p` | `euler_step_zoh_trace` | `euler_step_zoh_trace_p` |
| `rk4_step_ivp` | `rk4_step_ivp_p` | `rk4_step_ivp_trace` | `rk4_step_ivp_trace_p` |
| `euler_step_ivp` | `euler_step_ivp_p` | `euler_step_ivp_trace` | `euler_step_ivp_trace_p` |

```python
rk4_step_zoh(x, u, t, dt)
rk4_step_zoh_p(x, u, t, dt, params)
rk4_step_ivp(x, t, dt)
rk4_step_ivp_p(x, t, dt, params)
```

Steps compose over `f` / `f_p` / `f_ivp_p` — **no separate JIT program per step**.

### Rollouts

| Bound (fast) | Param (`_p`) | Trace (JAX) | Trace param (JAX) |
| --- | --- | --- | --- |
| `rk4_integrate_zoh` | `rk4_integrate_zoh_p` | `rk4_integrate_zoh_trace` | `rk4_integrate_zoh_trace_p` |
| `rk4_integrate_linear` | `rk4_integrate_linear_p` | `rk4_integrate_linear_trace` | `rk4_integrate_linear_trace_p` |
| `rk4_integrate_ivp` | `rk4_integrate_ivp_p` | `rk4_integrate_ivp_trace` | `rk4_integrate_ivp_trace_p` |
| `euler_integrate_zoh` | `euler_integrate_zoh_p` | `euler_integrate_zoh_trace` | `euler_integrate_zoh_trace_p` |
| `euler_integrate_ivp` | `euler_integrate_ivp_p` | `euler_integrate_ivp_trace` | `euler_integrate_ivp_trace_p` |

```python
# ZOH — u_sequence (N, m) → states (N+1, n)
rk4_integrate_zoh(x0, u_sequence, t0, dt)

# Linear — u_knots (K, m) → states (K, n)
rk4_integrate_linear(x0, u_knots, t0, dt)

# IVP — n_steps → states (n_steps+1, n)
rk4_integrate_ivp(x0, t0, dt, n_steps)
```

Euler rollouts mirror RK4 shapes and semantics.

### Sugar (RK4 ZOH hold window)

| Method | Role |
| --- | --- |
| `integrate_zoh` | One hold; optional `dt_inner`; calls `rk4_integrate_zoh` |
| `integrate_zoh_p` | Parametric twin |
| `integrate_zoh_rollout` | Returns `(t_samples, x_samples)` |

JAX trace for sugar: optional — delegate to expanded `rk4_integrate_zoh_trace`.

### Old → new map

| Old | New |
| --- | --- |
| `integrate` | `rk4_integrate_zoh` |
| `integrate_p` | `rk4_integrate_zoh_p` |
| `integrate_trace` | `rk4_integrate_zoh_trace` |
| `integrate_trace_p` | `rk4_integrate_zoh_trace_p` |
| `rk4_integrate_forced` | `rk4_integrate_linear` |
| `rk4_integrate_forced_p` | `rk4_integrate_linear_p` |
| `rk4_integrate_forced_trace` | `rk4_integrate_linear_trace` |
| `rk4_integrate_forced_trace_p` | `rk4_integrate_linear_trace_p` |
| `rk4_step` | `rk4_step_zoh` (alias) |
| `rk4_step_p` | `rk4_step_zoh_p` (alias) |
| `rk4_step_trace` | `rk4_step_zoh_trace` (alias) |
| `rk4_step_trace_p` | `rk4_step_zoh_trace_p` (alias) |
| `euler_step` | `euler_step_zoh` (alias) |

---

## Policy

| Suffix | Rule |
| --- | --- |
| **`_p`** | Every integrator that evaluates dynamics — NumPy and JAX |
| **`_trace` / `_trace_p`** | JAX only; mirror fast / `_p` one-for-one |

Deferred (separate from this plan): `rollout_trace` on step diagrams,
`compute_internal_signals_trace`, `integrate_zoh_trace` on JAX.

---

## Backend matrix

| | NumPy | JAX |
| --- | --- | --- |
| fast | ✓ | ✓ (JIT for rollouts) |
| `_p` | ✓ | ✓ |
| `_trace` | `AttributeError` | ✓ |
| `_trace_p` | `AttributeError` | ✓ |

---

## JAX implementation

### Single source of truth (fast vs trace)

```text
build_trace_{integrator}_{input}(..., f_trace or f_trace_p)
build_jit_* = jax.jit(build_trace_*)
```

Steps call `f` / `f_trace` inline — no duplicate RK4/Euler formulas between tiers.

NumPy and JAX still duplicate rollout **loop vs scan** bodies today; optional
follow-up: shared formula helpers in one module.

### Compile cost — lazy by default

| At `compile(backend="jax")` | Cost |
| --- | --- |
| `jax.jit(...)` wrappers | Cheap (no XLA until first call) |
| `build_trace_*` closures | Cheap |
| Warm-start (default) | **`f`, `f_p`, `f_ivp`, `outputs` only** |

**API surface ≠ compile cost.** Each distinct **rollout JIT** compiles on **first use**
unless warm-started.

| Do | Don't |
| --- | --- |
| Lazy `jax.jit` bind on first rollout call | Eager warm-start all integrators |
| Steps as `f` composition (no step JIT) | Separate XLA per step variant |
| Optional `compile(..., warm_start=...)` | Block full API out of compile fear |

**`warm_start` profiles** (future `compile` flag):

| Profile | Warm dummy calls |
| --- | --- |
| `False` / `"core"` (default) | `f`, `f_p`, `outputs` |
| `"sim"` | + `rk4_integrate_ivp` |
| `"trajopt"` | + `rk4_integrate_linear` |
| `"ad"` | trace smoke / consumer-driven |
| `"all"` | every rollout (debug) |

---

## Call-site impact (rename)

| Rename | ~Files |
| --- | --- |
| `integrate` → `rk4_integrate_zoh` | 5 (mostly tests + `integrate_zoh`) |
| `rk4_integrate_forced` → `rk4_integrate_linear` | 8 (solver, shooting, demos, tests) |
| Step ZOH aliases | 0 (alias-only) |

---

## Implementation order

1. **Plumbing:** `f_ivp_p` / `_f_ivp_trace_p_fn` on leaf + diagram evaluators; fix NumPy
   `euler_step_ivp` → `x + dt * f_ivp(x, t)`.
2. **Euler rollouts:** `euler_integrate_zoh` / `_p` / `_trace` / `_trace_p`,
   `euler_integrate_ivp` (+ `_p`, trace siblings) — NumPy mixin + JAX builders.
3. **Fill `_p` gaps:** `euler_step_zoh_p`, `*_step_ivp_p`, `*_integrate_ivp_p`,
   NumPy `rk4_integrate_linear_p`.
4. **RK4 renames** + deprecation aliases + call-site updates.
5. **Step ZOH aliases** on evaluator classes.
6. **Lazy rollout JIT** in `JaxIntegrationMixin` (defer `jax.jit` until first call).
7. **Solvers:** `EulerSolverBackend` → `euler_integrate_ivp` / `euler_integrate_zoh`.
8. **Tests:** fast ≈ trace parity; `f_p(frozen) ≈ f`; NumPy `_p` guards.
9. **Docs:** [DESIGN.md](../../DESIGN.md) §5 integration table; deprecations in
   [evaluator-trace-tier-api.md](evaluator-trace-tier-api.md).

---

## Relation to trace tier (landed separately)

PR [#65](https://github.com/alx87grd/minilink/pull/65) adds JAX fast vs trace on
`f`, `outputs`, `step`, and current integration names (`integrate`, `rk4_integrate_forced`, …).
This plan **renames** those integration surfaces and **extends** Euler + full `_p` /
`_trace` grid. Implement after or merge with trace-tier branch before release.

---

## Decision log

| Decision | Rationale |
| --- | --- |
| `zoh` / `linear` / `ivp` in names | States input model, not integrator |
| No Euler linear | Linear `u` is rollout-only; RK4 only |
| Full `_p` grid | Parametric ID / MPC need runtime params everywhere |
| Full JAX `_trace` grid | AD composition; no compile until used |
| Lazy rollout JIT | Large API, small default compile tax |
| Steps alias `*_zoh` | ZOH is the only sensible single-step input model |
