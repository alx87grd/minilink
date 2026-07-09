# Plan: Discontinuous-aware solver selection and warnings

**Status:** Implemented (July 2026). Follow-up to [DESIGN.md §5 — Discontinuous closed loops](../../DESIGN.md#discontinuous-closed-loops--known-issues).

**Goal:** Use `solver_info` metadata so `compute_trajectory` auto-picks safer solvers for discontinuous closed loops (SMC first), emits clear warnings when the user forces a solver whose **logged** torques do not reflect **internal** sub-step behavior, and drops the legacy `continuous_time_equation` key (evolution kind is class-type routing).

---

## Problem (confirmed)

| What the user sees | What integration does |
| --- | --- |
| Constant `ctl:u` on the output grid | `u` flips sign at RK4 k2–k4 sub-steps |
| Large `ddq` from `f(x,t)` at grid points | Net `Δdq/Δt` ≈ 0 (sub-step cancellation) |
| Pendulum “frozen” in animation | Numerical fixed point of algebraic `f_ivp` map |

`reconstruct_internal_signals` is **correct** for grid-state algebraic evaluation; it is **misleading** as a picture of sub-step physics under `rk4_fixedsteps`.

---

## Implemented machinery

### `System.solver_info` (defaults in `core/system.py`)

| Key | Default | Role |
| --- | --- | --- |
| `smallest_time_constant` | `0.001` | default `dt` scale in `select_time_vector` |
| `discontinuous_behavior` | `False` | if `True` → auto `euler` + finer auto `dt` |

Evolution kind (`DynamicSystem` / `StepSystem` / static `n=0`) is **not** encoded in `solver_info`; see hybrid-discrete plans.

### `Simulator.select_solver` (priority order)

1. User `solver=` argument → use as-is (with warnings when discontinuous)
2. `discontinuous_behavior` → **`euler`**
3. JAX + long uniform grid → `rk4_fixedsteps`
4. else → `scipy`

### Auto `dt` when `n_steps` and `dt` are both omitted

| `discontinuous_behavior` | Scale × `smallest_time_constant` |
| --- | --- |
| `True` | `0.1` (`DISCONTINUOUS_AUTO_DT_SCALE`) |
| `False` | `0.1` (`SMOOTH_AUTO_DT_SCALE`) |

### Warnings (`minilink/simulation/solver_warnings.py`)

- **Always** `UserWarning` when `discontinuous_behavior` is true (unless `solver_warnings="ignore"`); notes Euler, RK4/SciPy risks, and **HybridSimulator** with sampled computer + explicit sample time
- **Extra** message when user forces `rk4_fixedsteps` or `scipy_*`
- **Extra** message when user forces `euler` with `dt` coarser than `0.1 × smallest_time_constant`
- Threaded through `Simulator` and `DynamicSystemFacades.compute_trajectory` / `compute_forced` via `solver_warnings=`

### Leaf + diagram metadata

- `SlidingModeController` sets `discontinuous_behavior=True`
- `WiredDiagramMixin._refresh_solver_info()` aggregates from subsystems on `add_subsystem`

### Removed

- `solver_info["continuous_time_equation"]` — redundant with `Simulator` requiring `DynamicSystem`

---

## Demo and regression

- [`examples/scripts/control/demo_sliding_mode_pendulum.py`](../../examples/scripts/control/demo_sliding_mode_pendulum.py) — simple continuous SMC run (auto Euler).
- [`tests/unittest/test_discontinuous_solvers.py`](../../tests/unittest/test_discontinuous_solvers.py) — Euler vs RK4 consistency on coarse `dt`; SciPy cases skipped in smoke (can hang).

---

## Out of scope (unchanged)

- Sample-hold inside continuous `f_ivp` / RK4
- Event detection at switching surfaces
- Auto-redirect to `HybridSimulator`
- Changing `reconstruct_internal_signals` semantics
