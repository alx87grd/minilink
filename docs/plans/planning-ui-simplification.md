# Planning UI simplification

Status: **draft plan** (July 2026). User architectural decisions **locked** for v1
shape; implementation not started.

**Scope:** Trajectory-optimization and MPC **construction ergonomics** — not a
change to the locked planning contracts in
[mpc-rh-refactor/vision.md](mpc-rh-refactor/vision.md).

**Related plans:**

| Doc | Relationship |
| --- | --- |
| [mpc-rh-refactor/vision.md](mpc-rh-refactor/vision.md) | Contracts this plan must preserve |
| [mpc-rh-refactor/](mpc-rh-refactor/) | MPC refactor phases; demo dedup is orthogonal |
| [planning-pipeline-architecture.md](planning-pipeline-architecture.md) | Parametric NLP / result wrappers (orthogonal) |

Implemented contracts today: [DESIGN.md](../../DESIGN.md) §6,
[ROADMAP.md](../../ROADMAP.md) §5.5.

---

## Problem statement

After the MPC refactor (E0–E5), the **product pipeline is correct** but the
**student ceremony is heavy**. A minimal closed-loop demo forces imports of
option dataclasses the user does not yet need:

```python
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
```

That is unlike the rest of minilink, where beginners specify **strings and
scalars** on familiar objects:

```python
Simulator(sys, tf=10, dt=0.01, solver="euler")
Optimizer(prog, method="scipy_slsqp", maxiter=50)
ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
```

The fix is **not** recipes that hide physics or merge pipeline stages. It is
**Simulator-style defaults** on `TrajectoryOptimizationPlanner`: flatten
constructor kwargs, default transcription, keep advanced types available but
off the simple path.

---

## Design principle

> **Simplify ceremony, not structure.**

| Simplify (details not meaningful yet) | Keep explicit (fundamentals) |
| --- | --- |
| `DirectCollocationOptions` import | `PlanningProblem` |
| `TrajectoryOptimizationOptions` import | `TrajectoryOptimizationPlanner` |
| Mandatory `DirectCollocationTranscription(...)` wrapper | `ModelPredictiveController` |
| Nested construction for the default path | `tf` on problem, `n_steps` on planner, `dt_mpc` on controller |

---

## Locked decisions (user review, July 2026)

| # | Question | Decision | Rationale |
| --- | --- | --- | --- |
| 1 | Where does `n_steps` live? | **On `TrajectoryOptimizationPlanner`** | Knot count is solver discretization precision, not part of the continuous-time problem definition |
| 2 | Default transcription? | **Direct collocation when omitted** | Same spirit as `Simulator(solver=None)` — sensible default, advanced override available |
| 3 | Can `ModelPredictiveController` take `problem` directly? | **No (v1)** | Teaching clarity: problem → planner → controller stays visible |

---

## Target UX

### Simple pipeline (student / README / flagship demos)

Three named building blocks, flat planner kwargs, zero options-dataclass imports:

```python
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import TrajectoryOptimizationPlanner
from minilink.planning.mpc import ModelPredictiveController

problem = PlanningProblem(sys=sys, x_start=x0, cost=cost, tf=2.0)

planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=20,
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 50, "ftol": 1.0},
    record_solve_time=True,
)

mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
hybrid = mpc @ plant
hybrid.compute_trajectory(tf=10.0)
```

Hand-loop deploy (same planner construction):

```python
cmd = mpc.compute_command(y, t=t)
```

### Quantity homes (unchanged from vision)

| Quantity | Home | Notes |
| --- | --- | --- |
| Continuous horizon \(T\) | `PlanningProblem.tf` | Math task |
| Knot count \(N\) | `TrajectoryOptimizationPlanner` (`n_steps` kwarg) | Solver precision |
| Transcription method | Planner; default `"direct_collocation"` | Advanced: string or object |
| Replan period | `ModelPredictiveController.dt_mpc` | Deploy |
| Sim length | `hybrid.compute_trajectory(tf=...)` | Different object |

---

## Simulator analogy (reference pattern)

`TrajectoryOptimizationPlanner` should mirror `Simulator` layering:

| Simulator | TrajectoryOptimizationPlanner (proposed) |
| --- | --- |
| `solver=None` → auto / default | `transcription=None` → `"direct_collocation"` |
| `solver="euler"` string preset | `transcription="multiple_shooting"` string preset |
| `dt` / `n_steps` flat kwargs | `n_steps` flat kwarg |
| Internal `solver_backend_options` dict | `optimizer_options` dict (user-facing, like `Optimizer`) |
| `compile_backend` on simulator | `compile_backend` on planner |
| Advanced: inject solver backend directly | Advanced: pass `Transcription` instance |

**Future (out of v1):** auto transcription selection when multiple methods are
mature — analogous to `Simulator.select_solver()`.

---

## Two-tier API

### Tier 1 — Default path (public, documented in README)

Flat `TrajectoryOptimizationPlanner.__init__` kwargs:

| Kwarg | Default | Notes |
| --- | --- | --- |
| `problem` | (required) | `PlanningProblem` |
| `n_steps` | (required) | No default — user must pick grid resolution |
| `transcription` | `"direct_collocation"` | String preset or `Transcription` instance |
| `compile_backend` | `"numpy"` | `"jax"` for MPC parametric path |
| `optimizer_method` | `"scipy_slsqp"` | Same preset table as `Optimizer` |
| `optimizer_options` | `{}` | Merged into method preset; user keys win |
| `initial_guess` | `None` | Per vision optional-kwarg convention |
| `warm_start` | `False` | Planner-level initial-guess policy |
| `record_solve_time` | `False` | |
| `solve_disp` | `False` | |
| `step_disp` | `False` | MPC tick timing |
| `record_history` | `False` | Teaching / live plot |
| `use_hessian` | `False` | |
| `callback` | `None` | |

Implementation note: kwargs populate an internal `TrajectoryOptimizationOptions`
dataclass — **users never need to import it** on the default path.

### Tier 2 — Advanced path (library developers, custom transcriptions)

Unchanged escape hatches:

```python
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=MyCustomTranscription(...),
    options=TrajectoryOptimizationOptions(
        compile_backend="jax",
        callback=my_callback,
        record_history=True,
    ),
)
```

Optional convenience (no new concepts):

```python
from minilink.planning.trajectory_optimization.direct_collocation import direct_collocation

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=direct_collocation(n_steps=40),
)
```

Where `direct_collocation(n_steps)` is a **function** returning a
`Transcription` — replaces `DirectCollocationTranscription(DirectCollocationOptions(...))`
for users who want an explicit transcription without the Options type.

---

## Explicit non-goals

| Approach | Why not |
| --- | --- |
| Recipes that preset `Q`/`R`/scenes | Hides physics; contradicts “ceremony not structure” |
| `ModelPredictiveController(problem, ...)` in v1 | Hides planner building block |
| `n_steps` on `PlanningProblem` | Confuses continuous problem with discretization |
| Global `configure_trajopt()` session defaults | Surprises across notebooks |
| Mutable `planner.n_steps = ...` without policy | Compile-once MPC + AGENTS “no shadow state” |
| Mandatory nested dict config schema | Worse discoverability than flat kwargs |
| Removing `Transcription` ABC or options dataclasses internally | Library developers still need them |

---

## Implementation phases

Independent of mpc-rh-refactor E6–E8; can land on `dev-mpc-v2` when approved.

### UI-1 — Constructor flattening

| Work | Detail |
| --- | --- |
| Extend `TrajectoryOptimizationPlanner.__init__` | Accept flat kwargs; build internal `options` + default transcription |
| Default transcription | When `transcription is None`, use direct collocation |
| String transcription presets | `"direct_collocation"`, `"multiple_shooting"` (as supported) |
| Validation | Reject unknown kwargs; clear errors for transcription-specific keys on wrong method |
| Backward compat | Existing `transcription=` + `options=` call sites keep working |

**Gate:** `pytest tests/unittest/test_mpc_planner.py tests/unittest/test_jax_direct_collocation.py tests/unittest/test_planning_architecture.py -q`

### UI-2 — Documentation & flagship demo

| Work | Detail |
| --- | --- |
| README planning snippet | Replace Options/transcription nesting with flat kwargs |
| `demo_mpc_hybrid_minimal.py` | Optional: shorten imports only; preserve tuned constants |
| DESIGN §6 one paragraph | Document two-tier TOP constructor |
| `docs/plans/mpc-rh-refactor/vision.md` “Aim UX” | Add flat-constructor example alongside existing |

**Gate:** `ruff check . && ruff format --check .`

### UI-3 — Optional sugar

| Work | Detail |
| --- | --- |
| `direct_collocation(n_steps)` helper | Function wrapper; deprecate user-facing `DirectCollocationOptions` import in docs |
| `planner.with_options(**kw)` | Frozen copy for grid changes before `compile_parametric_program()` |
| Barrel re-export | `from minilink.planning.trajectory_optimization import TrajectoryOptimizationPlanner` only |

**Gate:** full `pytest` proportionate to touched modules.

### UI-4 — Broader demo migration (low priority)

Migrate remaining MPC/trajopt scripts to flat kwargs when touched — **do not**
bulk-rewrite user-tuned demo constants or commented sections (AGENTS.md).

---

## Migration map

| Call site | Action |
| --- | --- |
| `TrajectoryOptimizationPlanner(problem, transcription=DC(...), options=TOP(...))` | Keep working (tier 2) |
| README + hybrid minimal | Switch to tier 1 (UI-2) |
| Teaching notebooks (spatial guide) | Keep explicit step-by-step; optional tier-1 summary cell |
| `DirectCollocationOptions` class | Remain for internal/tier-2; remove from README imports |
| `TrajectoryOptimizationOptions` class | Internal + tier-2 only |

---

## Success criteria

1. **Hello-world MPC** uses ≤3 planning imports and **zero** options dataclasses.
2. **Vision pipeline** unchanged: `PlanningProblem` → `TrajectoryOptimizationPlanner` → `ModelPredictiveController`.
3. **Advanced users** retain `transcription=` objects and `options=` without behavior regression.
4. **Parity** with `Simulator` / `Optimizer` mental model documented in DESIGN.
5. All existing MPC + trajopt unit tests green after UI-1.

---

## Open items (before UI-1 coding)

1. **Required `n_steps`:** keep required on tier 1, or default from `problem.tf` and `dt_mpc` hint? (Locked: required — user must choose precision.)
2. **Top-level aliases:** allow `maxiter=50` as sugar merging into `optimizer_options`, or dict-only? (Lean: dict-only to match `Optimizer`; revisit if demos feel noisy.)
3. **Shooting string preset:** ship in UI-1 or only direct collocation default until shooting MPC path is tested?

---

## Suggested PR slicing

| PR | Contents |
| --- | --- |
| PR-UI-A | UI-1 constructor + tests |
| PR-UI-B | UI-2 README / DESIGN / vision snippet |
| PR-UI-C | UI-3 helpers + optional demo touch-ups |

This document is the **source of truth** for planning UI simplification.
Constructor flattening takes precedence over ad-hoc recipe-layer sketches in
other cleanup notes.
