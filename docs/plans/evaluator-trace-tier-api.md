# Evaluator trace tier — naming scheme and implementation plan

Status: **phases 1–2 landed** (July 2026); phases 3–5 deferred. PR #65.

Follow-up to JAX evaluator export review: separate the **fast default tier**
(JIT on JAX, eager NumPy) from an optional **trace tier** (pre-JIT, composable
in outer `jit` / `grad` / `vmap` / C export). Contracts in code today:
[DESIGN.md](../../DESIGN.md) §5 (Compilation), evaluator modules under
`minilink/core/compile/evaluators/`.

---

## Implementation status (July 2026)

| Phase | Status | Notes |
| --- | --- | --- |
| **0** Docs / plan | Done | This document, [DESIGN.md](../../DESIGN.md) §5, [README.md](../../README.md) |
| **1** Core trace (`f`, `outputs`, `step`) | **Landed** | `tiers.py`; all JAX leaf + diagram evaluators; `test_evaluator_tiers.py` |
| **2** Integration trace | **Landed** | `JaxIntegrationMixin` in `jax_utils.py`; RK4/Euler/integrate/forced rollouts |
| **3** Rollout / ZOH / diagram internals | **Deferred** | New APIs only; comments at extension points — see [Phase 3](#phase-3--step-rollouts-and-diagram-internals-deferred) |
| **4** `warm_start` compile flag | Not started | Orthogonal cleanup |
| **5** Tier benchmarks | Not started | `benchmarks/jax_evaluator_tiers.py` |

**Consumers updated:** `c_export.py` (trace + scoped `disable_jit`), `transcription.dynamics_function`
(NumPy + reconstruct path), stale notebook `get_f_jit` reference.

**Not on JAX trajopt hot path:** direct collocation / shooting / multiple shooting JAX
transcriptions call `problem.sys.f` in the NLP — unchanged. `dynamics_function` trace
tier affects NumPy transcribe and post-solve `reconstruct_result` only.

**Gaps (intentional):** `f_ivp_trace`; `integrate_zoh*` JAX/trace; `rollout_trace`;
`compute_internal_signals_trace`. See tier tables below.

Diagnostic: `benchmarks/run_trajopt_timing_breakdown.py` (end-to-end phase timing).

---

## Goals

1. **`.f` stays the default** — fastest evaluation path on every backend.
2. **Trace tier is JAX-only** — NumPy evaluators expose no `_trace` / `_jit`
   siblings; missing combinations raise `AttributeError` with a clear message.
3. **Two orthogonal suffix axes** — execution tier, then param tier (`_p` last).
4. **Code organization mirrors the naming grid** — gaps in the matrix are visible
   as `NotImplementedError` or missing methods during review.
5. **Backward compatible** — existing `f`, `f_p`, `rk4_integrate_forced`, …
   behavior unchanged; add siblings and aliases only.

Non-goals for v1: JAX trajopt NLP hot path (already uses raw `sys.f`); removing
compile-time warm-start; renaming `compile(backend="jax")`.

---

## Naming grammar

### Param tier (all backends, unchanged)

| Suffix | Meaning |
| --- | --- |
| *(none)* | **Bound** — params fixed at compile (`bind_params` / `_frozen_params`) |
| `_p` | **Parametric** — caller supplies nested params pytree at call time |

### Execution tier

| Surface | JAX | NumPy |
| --- | --- | --- |
| Default method / property | JIT-compiled (`_jit_*`) | Eager flat eval |
| `_trace` sibling | Pre-JIT flat callable (`_trace_*`) | **Not exposed** |
| `_jit` alias | Alias of default (symmetry / docs) | **Not exposed** |

### Combination rule

```
{name}              → fast, bound
{name}_p            → fast, parametric
{name}_trace        → composable, bound        (JAX only)
{name}_trace_p      → composable, parametric   (JAX only)

{name}_jit          → alias of {name}          (JAX only)
{name}_jit_p        → alias of {name}_p        (JAX only)
```

**Do not** use `_jax` as the primary suffix (clashes with `compile(backend="jax")`).
Optional undocumented alias `f_jax = f_trace` only if grep demand appears later.

**Do not** use `_raw` on evaluators (reserve for reference `sys.f` and
`compile_backend="direct"`).

### Glossary (DESIGN.md / README)

| Term | Meaning |
| --- | --- |
| **bound** | Params snapshot at compile; call has no `params` argument |
| **parametric** | Runtime params pytree; `_p` suffix |
| **fast tier** | Default public methods — JIT on JAX |
| **trace tier** | Pre-JIT flat callable for JAX composition |
| **reference** | Uncompiled `sys.f` / recursive diagram path |

---

## API matrix (JAX dynamics evaluator)

|  | Bound | Parametric |
| --- | --- | --- |
| **Fast (default)** | `f` ≡ `f_jit` | `f_p` ≡ `f_jit_p` |
| **Trace** | `f_trace` | `f_trace_p` |

Same 2×2 for: `outputs`, `step` (step evaluators), and integration helpers
listed below.

NumPy: first row, first two columns only.

---

## Methods in scope

### Tier 1 — core dynamics / outputs / step (phase 1)

| Bound fast | Param fast | Bound trace | Param trace |
| --- | --- | --- | --- |
| `f` | `f_p` | `f_trace` | `f_trace_p` |
| `outputs` | `outputs_p` | `outputs_trace` | `outputs_trace_p` |
| `step` | `step_p` | `step_trace` | `step_trace_p` |

**Special (keep names, reimplement internals):**

- `jacobian_f_params` — JIT `jacfwd` on `_f_trace_p_fn` (trace core).
- `f_ivp` / `f_ivp_scipy` — fast tier only; **`f_ivp_trace` not landed** (optional).

### Tier 2 — single-step integration (phase 2)

| Bound fast | Param fast | Bound trace | Param trace |
| --- | --- | --- | --- |
| `rk4_step` | `rk4_step_p` | `rk4_step_trace` | `rk4_step_trace_p` |
| `rk4_step_ivp` | — | `rk4_step_ivp_trace` | — |
| `euler_step` | — | `euler_step_trace` | — |
| `euler_step_ivp` | — | `euler_step_ivp_trace` | — |

NumPy: `IntegrationMixin` keeps Python loops on `self.f` / `self.f_p` (unchanged).

JAX fast tier: keep current behavior (may call JIT `f` inside JIT scan wrappers).

JAX trace tier: RK4/Euler formulas call `f_trace` / `f_trace_p`; rollout-level
trace helpers JIT **once** over the scan (single fused graph for AD).

### Tier 3 — trajectory / rollout (phase 2 partial, phase 3 deferred)

| Bound fast | Param fast | Bound trace | Param trace |
| --- | --- | --- | --- |
| `rk4_integrate_forced` | `rk4_integrate_forced_p` | `rk4_integrate_forced_trace` | `rk4_integrate_forced_trace_p` |
| `rk4_integrate_ivp` | — | `rk4_integrate_ivp_trace` | — |
| `integrate` | `integrate_p` | `integrate_trace` | `integrate_trace_p` |
| `integrate_zoh` | `integrate_zoh_p` * | `integrate_zoh_trace` * | … |
| `integrate_zoh_rollout` | … * | … * | … |
| `rollout` | `rollout_p` | `rollout_trace` * | `rollout_trace_p` * |

\* **Phase 3 / gaps:** ZOH and step `rollout_trace` not landed; `integrate_zoh_p` NumPy-only today.

### Tier 4 — diagram / bridges (phase 3, as needed)

| Method | Trace variant |
| --- | --- |
| `compute_internal_signals` | `compute_internal_signals_trace` (if AD through internals needed) |
| `compute_internal_signals_dict` | wraps above |
| `f_scipy`, `as_scipy_rhs*`, `as_scipy_jac` | **none** — NumPy bridge from fast tier only |

---

## Code organization

### Module layout

Introduce a small shared module for the naming contract and helpers:

```
minilink/core/compile/evaluators/
  tiers.py              # TraceTierMixin, NoTraceTierMixin, register_jit_aliases
  jax_utils.py          # tier builders + JaxIntegrationMixin (trace + JIT integration)
  jax_evaluator.py      # JaxDynamicEvaluator, JaxDiagramEvaluator
  step_evaluator.py
  step_diagram_evaluator.py
  static_evaluator.py
  integration.py        # NumPy IntegrationMixin
```

(`jax_integration.py` split deferred — integration overrides live in `jax_utils.py`.)

### Internal private names (rename for clarity)

| Old | New |
| --- | --- |
| `_f_eager` | `_f_trace_fn` |
| `_f_p_eager` | `_f_trace_p_fn` |
| `_jit_f` | `_f_jit_fn` |
| `f_raw` in builders | `f_bound_fn` or `trace_f_fn` local |

### `TraceTierMixin` (JAX evaluators only)

Responsibilities:

1. Expose `_f_trace_fn` / `_f_trace_p_fn` callables built at compile time.
2. Register `_jit` aliases pointing at existing fast methods.
3. `NoTraceTierMixin.__getattr__` rejects `*_trace` / `*_jit` suffixes on NumPy backends.
4. `has_trace_tier: bool` property (`True` on JAX, absent on NumPy).

Pattern for each quantity `q` (e.g. dynamics `f`):

```python
# compile time
self._f_jit_fn = jax.jit(...)
self._f_trace_fn = _f_eager   # or closure over system.f + frozen_p

# public
def f(self, x, u, t=0.0):
    """Evaluates the dynamics (fast tier / JIT)."""
    return self._f_jit_fn(x, u, t)

# We use a private backing callable (_f_trace_fn) and a method wrapper (f_trace) 
# to maintain exact symmetry with _f_jit_fn and f, which guarantees perfect 
# IDE auto-complete and docstring support without leaking dynamic callables.
def f_trace(self, x, u, t=0.0):
    """Pre-JIT flat callable for JAX composition."""
    return self._f_trace_fn(x, u, t)

f_jit = f  # alias at class level or property
```

Integration methods follow the same builder pattern:

```python
build_jit_rk4_integrate_forced(jax, jnp, f_jit)
build_trace_rk4_integrate_forced(jax, jnp, f_trace)
```

### Coverage grid / stub policy

Maintain a **checklist table** in this doc and in `tests/unittest/test_evaluator_tiers.py`:

- Implemented combination → test numeric parity: `jit(x) ≈ trace(x)` and
  `f_p(..., frozen) ≈ f(...)`.
- Intentionally missing (e.g. `rk4_step_ivp_p`) → no attribute or explicit
  `NotImplementedError` with reason in docstring.
- NumPy evaluator accessing `f_trace` → `AttributeError`:
  `"Trace tier requires compile(backend='jax')."`.

This makes “missing cell” visible in code review.

---

## Implementation phases

### Phase 0 — Docs and plan (this document)

- [x] Approve naming scheme (Scheme 1 + `_jit` aliases).
- [x] Add row to [docs/plans/README.md](README.md).
- [x] Short pointer in [DESIGN.md](../../DESIGN.md) §5 (after phase 1 lands).

### Phase 0.5 — Codebase Audit for API Consumers

This section catalogs exactly where the evaluator API is currently consumed across tools, examples, and demos, and what updates will be required. 

Because this refactor **preserves existing method signatures** (like `.f`, `.f_p`, `.rk4_step`), most consuming scripts will continue to work exactly as they do today. However, scripts that hacked into internal structures or rely on stale API methods must be updated.

**Requires Update (done in PR #65):**
1. **`minilink/interfaces/c_export.py`** — uses `f_trace` / `step_trace`; `disable_jit` only around `make_jaxpr` when subsystem `f()` embeds `jax.jit`.
2. **`examples/notebooks/demo_stateless_functional_jax.ipynb`** — `evaluator.f_jit` instead of removed `get_f_jit()`.
3. **`minilink/planning/trajectory_optimization/transcription.py`** — `dynamics_function` uses `f_trace` / `f_trace_p` when `has_trace_tier` (NumPy transcribe + reconstruct; not JAX NLP hot path).

**No Update Required (Standard API consumers):**
The following examples heavily utilize the `evaluator`, but rely strictly on the standard public fast-tier methods (`f`, `f_p`, `rk4_step`, `rk4_step_p`, `jacobian_f_params`, `objective`, `constraint_violations`), which remain **fully supported**:
- `examples/scripts/control/demo_neural_controller_jax.py` (uses `f_p`)
- `examples/scripts/control/demo_pid_autotuning_jax.py` (uses `rk4_step_p`, `rk4_integrate_forced`)
- `examples/scripts/identification/demo_params_gradient.py` (uses `jacobian_f_params`, `f_p`, `f`)
- `examples/scripts/engine/demo_physics_many_spheres.py` (uses `f`)
- All MPC demos in `examples/scripts/mpc/` (use `rk4_step`, `objective`, `constraint_violations` natively)
- All benchmarks in `benchmarks/` (use `f`, `step`, `euler_step`)

### Phase 1 — Core trace callables

- [x] `f_trace`, `f_trace_p`, `outputs_trace`, `outputs_trace_p`
- [x] `step_trace`, `step_trace_p` on step evaluators
- [x] `f_jit` / `f_jit_p` aliases (`register_jit_aliases`)
- [x] Internal rename `_f_eager` → `_f_trace_fn` (diagram); tier builders for leaves
- [x] Tests: `test_evaluator_tiers.py`
- [x] Consumers: `c_export.py`, notebook, `dynamics_function`

### Phase 2 — Integration trace tier

- [x] `rk4_step_trace`, `rk4_step_trace_p`, `rk4_step_ivp_trace`, `euler_step_trace`, `euler_step_ivp_trace`
- [x] `rk4_integrate_forced_trace`, `rk4_integrate_ivp_trace`, `integrate_trace`, `integrate_trace_p`
- [x] Fast-tier gap: `rk4_integrate_forced_p`
- [x] Tests in `test_evaluator_tiers.py`

### Phase 3 — Step rollouts and diagram internals (deferred)

**Not rename-only.** Phase 1–2 added trace siblings next to existing fast-tier methods
(`f` / `f_trace`). Phase 3 items below are **new public methods** (or new JAX paths)
that do not exist yet — do not land stubs until a consumer needs them.

| Planned surface | Exists today? | Phase 3 work |
| --- | --- | --- |
| `rollout` / `rollout_p` | Yes (`StepRolloutMixin`, JAX `_jit_rollout`) | **New:** `build_trace_step_rollout` + `rollout_trace` / `rollout_trace_p` scan over `step_trace` |
| `compute_internal_signals` | Yes (diagram JIT + `_internal_signals_eager`) | **Thin wiring:** `compute_internal_signals_trace` → eager fn (like `f_trace`) |
| `integrate_zoh` / `integrate_zoh_rollout` | Yes (NumPy loop on `IntegrationMixin` only) | **New:** JAX ZOH scan + `integrate_zoh_trace` (no JAX ZOH path today) |
| `integrate_zoh_p`, `integrate_zoh_trace_p` | No | **New** (optional; grid gaps in plan table) |

**Policy:** add `# Phase 3 (deferred): …` comments at extension points; implement only
when a test or demo requires AD through that path.

- [ ] `rollout_trace`, `rollout_trace_p`
- [ ] `compute_internal_signals_trace` (diagram) if required by tests/demos
- [ ] `integrate_zoh*` trace/param siblings as needed

### Phase 4 — Warm-start and compile flags (orthogonal cleanup)

- `compile(..., warm_start=True)` on JAX evaluators
- Centralize warm-start in `jax_utils._warm_start_callables`
- Step evaluator warm-start parity with dynamics
- Document which grid cells are warmed at compile time

### Phase 5 — Benchmarks

- `benchmarks/jax_evaluator_tiers.py`: fast vs trace inside `jit(grad(loss))`;
  nested RK4 JIT vs single trace scan.

---

## Testing policy

| Test | Status |
| --- | --- |
| `f` vs `f_trace` numeric parity | [x] |
| `f_jit` class alias ≡ `f` | [x] |
| NumPy `f_trace` / `rk4_step_trace` raises | [x] |
| `jax.jit(jax.grad(loss))` with `f_trace_p` | [x] |
| Integration trace parity (`rk4_step_trace`, `integrate_trace`) | [x] |
| `make_jaxpr(f_trace)` (C-export) | manual / `demo_c_export` |
| Grid coverage parametrized | [ ] future |

Follow [tests/README.md](../../tests/README.md): stable public API → tests justified.

---

## Documentation updates (when landing code)

| Doc | Status |
| --- | --- |
| [DESIGN.md](../../DESIGN.md) §5 | [x] tier table + integration note |
| [README.md](../../README.md) §Compiled execution | [x] `f_trace` example |
| [AGENTS.md](../../AGENTS.md) | unchanged (`warm_start` not landed) |
| [docs/plans/README.md](README.md) | [x] |

Example README snippet (after phase 1):

```python
ev = diagram.compile(backend="jax")

dx = ev.f(x, u, t)                    # fast tier (JIT)
loss_and_grad = jax.jit(jax.value_and_grad(
    lambda theta: jnp.mean((ev.f_trace_p(x, u, t, {"plant": theta}) - dx_ref) ** 2)
))
```

---

## Migration and compatibility

- **No breaking changes** to existing method signatures.
- **`get_f_jit`**: remains removed; use `f` or `f_jit`.
- **Trajopt/MPC:** JAX NLP transcriptions still use `problem.sys.f` directly.
  `dynamics_function` uses trace tier when `has_trace_tier` (NumPy transcribe,
  `reconstruct_result`).
- **Deprecations**: none planned for v1.

---

## Decision log

| Decision | Rationale |
| --- | --- |
| Default `.f` = fast tier | Simulation-first UX; NumPy unchanged |
| `_trace` not `_traceable` | Shorter; scales to `rk4_integrate_forced_trace_p` |
| `_jit` aliases optional | Scheme 2 symmetry for docs/tests without breaking `f` |
| No `_trace` on NumPy | Avoid duplicate API; clear backend guard |
| Param suffix always last | Consistent with existing `f_p`, `outputs_p` |
| Organize code by tier builders | Missing combinations obvious in matrix |

---

## Tracking

Phases 1–2 landed in PR #65. Phase 3+ deferred until a consumer needs AD through
rollout / ZOH / diagram internals. Optional follow-up: `benchmarks/jax_evaluator_tiers.py`,
`compile(..., warm_start=True)`.
