# Evaluator trace tier — naming scheme and implementation plan

Status: **approved design** (July 2026). Implementation not yet landed.

Follow-up to JAX evaluator export review: separate the **fast default tier**
(JIT on JAX, eager NumPy) from an optional **trace tier** (pre-JIT, composable
in outer `jit` / `grad` / `vmap` / C export). Contracts in code today:
[DESIGN.md](../../DESIGN.md) §5 (Compilation), evaluator modules under
`minilink/core/compile/evaluators/`.

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

Non-goals for v1: changing trajopt/MPC transcription (they already use raw
`sys.f` in the NLP hot path); removing compile-time warm-start; renaming
`compile(backend="jax")`.

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

- `jacobian_f_params` — implement via `f_trace_p` + `jacfwd` (already AD on
  trace core today; align implementation explicitly).
- `f_ivp` / `f_ivp_scipy` — fast tier only; add `f_ivp_trace` as thin wrapper
  over `f_trace` with nominal `u` (phase 2).

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

### Tier 3 — trajectory / rollout (phase 2–3)

| Bound fast | Param fast | Bound trace | Param trace |
| --- | --- | --- | --- |
| `rk4_integrate_forced` | `rk4_integrate_forced_p` * | `rk4_integrate_forced_trace` | `rk4_integrate_forced_trace_p` * |
| `rk4_integrate_ivp` | — | `rk4_integrate_ivp_trace` | — |
| `integrate` | `integrate_p` | `integrate_trace` | `integrate_trace_p` |
| `integrate_zoh` | `integrate_zoh_p` * | `integrate_zoh_trace` | … |
| `integrate_zoh_rollout` | … | … | … |
| `rollout` | `rollout_p` | `rollout_trace` | `rollout_trace_p` |

\* **Gap today:** forced RK4 / ZOH rollouts lack parametric fast-tier siblings;
add when implementing trace rollouts for API symmetry.

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
  tiers.py              # TraceTierMixin, alias helpers, AttributeError messages
  jax_utils.py          # build_jit_*, build_trace_*, build_jit_rk4_*, build_trace_rk4_*
  jax_evaluator.py      # JaxDynamicEvaluator, JaxDiagramEvaluator
  step_evaluator.py
  step_diagram_evaluator.py
  integration.py        # NumPy IntegrationMixin (unchanged public surface)
  jax_integration.py    # JAX-only integration overrides (optional split from jax_utils)
```

### Internal private names (rename for clarity)

| Old | New |
| --- | --- |
| `_f_eager` | `_trace_f` |
| `_f_p_eager` | `_trace_f_p` |
| `_jit_f` | `_jit_f` (keep) |
| `f_raw` in builders | `f_bound` or `trace_f` local |

### `TraceTierMixin` (JAX evaluators only)

Responsibilities:

1. Expose `_trace_*` callables built at compile time.
2. Register `_jit` aliases pointing at existing fast methods.
3. Provide `_require_jax_trace(name)` for uniform errors on NumPy backends.
4. Optional: `has_trace_tier: bool` property (`True` on JAX, absent on NumPy).

Pattern for each quantity `q` (e.g. dynamics `f`):

```python
# compile time
self._jit_f = jax.jit(...)
self._trace_f = _f_eager   # or closure over system.f + frozen_p

# public
def f(self, x, u, t=0.0):
    return self._jit_f(x, u, t)

@property
def f_trace(self):
    return self._trace_f

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
- [ ] Add row to [docs/plans/README.md](README.md).
- [ ] Short pointer in [DESIGN.md](../../DESIGN.md) §5 (after phase 1 lands).

### Phase 1 — Core trace callables (MVP)

**Files:** `tiers.py`, `jax_utils.py`, `jax_evaluator.py`, `static_evaluator.py`,
`step_evaluator.py`, `step_diagram_evaluator.py`.

**Deliver:**

- `f_trace`, `f_trace_p`, `outputs_trace`, `outputs_trace_p`
- `step_trace`, `step_trace_p`, `outputs_trace` on step evaluators
- `f_jit` / `f_jit_p` aliases (properties or class aliases)
- Rename internal `_f_eager` → `_trace_f`
- Tests: `test_evaluator_tiers.py` — parity, NumPy rejection, alias identity
- Update `test_evaluator_api.py` if needed

**Consumers (optional in same PR or immediate follow-up):**

- `interfaces/c_export.py` → use `f_trace` instead of `evaluator.f` + `disable_jit`
- Stale notebook reference to removed `get_f_jit()`

### Phase 2 — Integration trace tier

**Files:** `jax_utils.py`, `JaxIntegrationMixin`, `integration.py` (docstrings only).

**Deliver:**

- `rk4_step_trace`, `rk4_step_trace_p`
- `rk4_integrate_forced_trace`, `rk4_integrate_ivp_trace`
- `integrate_trace`, `integrate_trace_p`
- Fill fast-tier gaps: `rk4_integrate_forced_p` (if low cost)

**Consumers:**

- `examples/scripts/identification/demo_params_gradient.py` → `f_trace_p` in loss
- `examples/scripts/control/demo_pid_autotuning_jax.py` → `rk4_step_trace_p`

### Phase 3 — Step rollouts and diagram internals

**Deliver:**

- `rollout_trace`, `rollout_trace_p`
- `compute_internal_signals_trace` (diagram) if required by tests/demos
- `integrate_zoh*` trace/param siblings as needed

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

| Test | Purpose |
| --- | --- |
| `f` vs `f_trace` numeric parity | Same math, different dispatch |
| `f_jit is f` | Alias contract |
| NumPy `f_trace` raises | Backend guard |
| `jax.jit(jax.grad(loss))` with `f_trace_p` | Smoke AD (optional marker `@pytest.mark.jax`) |
| `make_jaxpr(f_trace)(...)` | C-export path |
| Grid coverage parametrized | `@pytest.mark.parametrize("method", TIER_MATRIX)` |

Follow [tests/README.md](../../tests/README.md): stable public API → tests justified.

---

## Documentation updates (when landing code)

| Doc | Update |
| --- | --- |
| [DESIGN.md](../../DESIGN.md) §5 | 2×2 tier table, JAX-only trace note |
| [README.md](../../README.md) §Compiled execution | Replace “traceable for autodiff” wording; show `f_trace` example |
| [AGENTS.md](../../AGENTS.md) | Only if compile API changes (`warm_start` flag) |

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
- **Trajopt/MPC**: no change required (keep raw `sys.f` in NLP).
- **`dynamics_function`**: optional later `tier="trace"` for reconstruction; not phase 1.
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

Implementation PRs should reference phase numbers above and tick checkboxes in
follow-up edits to this file. When phase 1 lands, add a ROADMAP checkbox under
compile/JAX maturity if appropriate.
