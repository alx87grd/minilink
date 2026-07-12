# Evaluator integration API — naming, layout, and compile plan

Architecture plan for dynamics evaluator **integration** methods (RK4 / Euler steps
and rollouts), plus by-backend file layout. Complements the trace-tier contract in
[evaluator-trace-tier-api.md](evaluator-trace-tier-api.md). Contracts in code:
[DESIGN.md](../../DESIGN.md) §5.

Status: **implemented**.

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

## Locked decisions

| Decision | Choice |
| --- | --- |
| Layout | By backend: `evaluators.py` (ABCs), `numpy_evaluators.py`, `jax_evaluators.py` |
| Step / Static | Same backend files |
| Rename policy | **Clean rename** — AGENTS pre-1.0 no-alias; fix all call sites in the same change |
| Step names | Short canonical `rk4_step` / `euler_step` (ZOH implied). No `*_step_zoh` aliases |
| Rollout names | Full grammar: `rk4_integrate_{zoh,linear,ivp}`, `euler_integrate_{zoh,ivp}` |
| Warm-start | **No** dummy JIT invocations at evaluator `__init__` / compile |
| Compatibility check | Keep `make_jaxpr` / `check_jax_compatible` (traceability, not XLA warm) |
| Rollout JIT | Bind `jax.jit` lazily on first call |
| Math style | RK4/Euler equations **inline** in the method body |
| WIP files | Delete untracked `dynamic_evaluator.py` / `dynamic_diagram_evaluator.py` |

---

## Target package layout

```
minilink/core/compile/evaluators/
  evaluators.py          # ABCs only
  numpy_evaluators.py    # all Numpy* + NumPy IntegrationMixin
  jax_evaluators.py      # all Jax* + JaxIntegrationMixin + JAX helpers
  step_rollout.py        # gather_u + StepRolloutMixin (shared NumPy/JAX)
  tiers.py               # TraceTierMixin / NoTraceTierMixin / register_jit_aliases
  __init__.py            # public re-exports
```

Delete after cutover: `dynamics_evaluator.py`, `evaluator.py`, `numpy_evaluator.py`,
`jax_evaluator.py`, `jax_utils.py`, `integration.py`, `static_evaluator.py`,
`step_evaluator.py`, `step_diagram_evaluator.py`, `step_rollout_mixin.py`,
`output_evaluator.py`, WIP `dynamic_*.py`.

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

**Steps:** public names omit `_zoh` — `rk4_step` / `euler_step` are the ZOH single-step
APIs. Rollouts always include the input model token.

### Intentionally absent

| Surface | Why |
| --- | --- |
| `*_step_linear`, `euler_integrate_linear` | Linear `u` needs `u_{k+1}` — rollout-only, RK4 only |
| NumPy `*_trace` | `NoTraceTierMixin` rejects trace tier |
| `*_step_zoh` aliases | Clean rename; ZOH is implied for steps |
| Compile-time warm-start / `warm_start=` profiles | First real call pays XLA cost |

IVP `_p` means **parametric plant params** with **nominal `u` fixed at compile**:
`f_ivp_p(x, t, params) ≡ f_p(x, u_nom, t, params)`.

---

## Full API (target)

### Single-step

| Bound (fast) | Param (`_p`) | Trace (JAX) | Trace param (JAX) |
| --- | --- | --- | --- |
| `rk4_step` | `rk4_step_p` | `rk4_step_trace` | `rk4_step_trace_p` |
| `euler_step` | `euler_step_p` | `euler_step_trace` | `euler_step_trace_p` |
| `rk4_step_ivp` | `rk4_step_ivp_p` | `rk4_step_ivp_trace` | `rk4_step_ivp_trace_p` |
| `euler_step_ivp` | `euler_step_ivp_p` | `euler_step_ivp_trace` | `euler_step_ivp_trace_p` |

```python
rk4_step(x, u, t, dt)
rk4_step_p(x, u, t, dt, params)
rk4_step_ivp(x, t, dt)
rk4_step_ivp_p(x, t, dt, params)
```

Steps compose over `f` / `f_p` / `f_ivp` / `f_ivp_p` — **no separate JIT program per step**.

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

### Old → new map (hard rename)

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
| fast | ✓ | ✓ (JIT for rollouts, lazy) |
| `_p` | ✓ | ✓ |
| `_trace` | `AttributeError` | ✓ |
| `_trace_p` | `AttributeError` | ✓ |

---

## JAX implementation

### Textbook math, inline

Single-step and scan bodies show RK4/Euler equations at the method (no distant
`_rk4_step_from_f` indirection on the public path). NumPy and JAX may duplicate
formulas for readability. Trace-tier siblings sit next to fast methods in the
same section.

### Compile cost — no warm-start

| At `compile(backend="jax")` | Cost |
| --- | --- |
| `jax.jit(...)` wrappers | Cheap (no XLA until first call) |
| Trace closures | Cheap |
| Dummy JIT warm-start | **None** |

**API surface ≠ compile cost.** Each distinct **rollout JIT** compiles on **first use**.

| Do | Don't |
| --- | --- |
| Lazy `jax.jit` bind on first rollout call | Eager warm-start at `__init__` |
| Steps as `f` composition (no step JIT) | Separate XLA per step variant |
| `make_jaxpr` compatibility check | `compile(..., warm_start=...)` profiles |

---

## Call-site impact (rename)

| Rename | ~Files |
| --- | --- |
| `integrate` → `rk4_integrate_zoh` | tests + `integrate_zoh` sugar |
| `rk4_integrate_forced` → `rk4_integrate_linear` | solver, shooting, demos, tests |
| `rk4_step` | unchanged |

---

## Implementation order

1. **Docs:** this file (status + locked decisions + layout).
2. **File reorg:** `evaluators.py` / `numpy_evaluators.py` / `jax_evaluators.py`;
   remove warm-start; rewire imports; delete old + WIP modules.
3. **Plumbing:** `f_ivp_p` / `_f_ivp_trace_p_fn`; fix NumPy
   `euler_step_ivp` → `x + dt * f_ivp(x, t)`.
4. **API grid + renames:** Euler rollouts, full `_p` / `_trace` grid, clean rename
   of `integrate` / `rk4_integrate_forced`, lazy rollout JIT.
5. **Solvers:** `EulerSolverBackend` → `euler_integrate_ivp` / `euler_integrate_zoh`.
6. **Tests + contracts:** DESIGN §5, ROADMAP §5.0, trace-tier plan paths.

---

## Relation to trace tier (landed)

PR [#65](https://github.com/alx87grd/minilink/pull/65) adds JAX fast vs trace on
`f`, `outputs`, `step`, and (pre-rename) integration names. This plan **renames**
those integration surfaces, **extends** Euler + full `_p` / `_trace` grid, and
**reorganizes** evaluator files by backend.

---

## Decision log

| Decision | Rationale |
| --- | --- |
| `zoh` / `linear` / `ivp` in rollout names | States input model, not integrator |
| Short step names (no `*_zoh`) | ZOH is the only sensible single-step input model |
| No Euler linear | Linear `u` is rollout-only; RK4 only |
| Full `_p` grid | Parametric ID / MPC need runtime params everywhere |
| Full JAX `_trace` grid | AD composition; no compile until used |
| Lazy rollout JIT; no warm-start | Large API, small default compile tax |
| Clean rename (no aliases) | AGENTS pre-1.0 no-alias rule |
| By-backend file layout | Crystal-clear for human readers |
