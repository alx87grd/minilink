# Planning UI simplification

Status: **draft plan** (July 2026). User architectural decisions **locked** for v1
shape; implementation not started.

**Scope:** **Planner-family** construction ergonomics (trajopt, RRT, DP) aligned
with Simulation / Optimizer patterns — not a change to the locked planning
contracts in [mpc-rh-refactor/vision.md](mpc-rh-refactor/vision.md).

**Related plans:**

| Doc | Relationship |
| --- | --- |
| [mpc-rh-refactor/vision.md](mpc-rh-refactor/vision.md) | Contracts this plan must preserve |
| [mpc-rh-refactor/phase-F-cleanup.md](mpc-rh-refactor/phase-F-cleanup.md) | Land `control/mpc` **before** this plan so demos teach final import paths |
| [mpc-rh-refactor/](mpc-rh-refactor/) | MPC refactor E0–E8 done; closing sequence F → this UI plan |
| [planning-pipeline-architecture.md](planning-pipeline-architecture.md) | Pipeline B / scene bind — later feature, orthogonal |

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
**Simulator-style defaults** across the planner family: flatten constructor
kwargs where Options dataclasses are mandatory on the default path, keep
advanced types available but off the simple path.

**Reference modules (already aligned — do not change):** `Simulator`,
`Optimizer`, `linearize(..., method=)`, `ModelPredictiveController`,
`HybridDiagram.compute_trajectory`, `sys.compile(backend=)`.

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
from minilink.control.mpc import ModelPredictiveController

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

## Planner family alignment

All deterministic planners share the same textbook stack:

```text
PlanningProblem  →  discretization / method seam  →  Planner  →  typed result
```

| Planner family | Discretization / method seam (keep explicit) | Ceremony to flatten |
| --- | --- | --- |
| Trajopt / MPC | `transcription` (default `"direct_collocation"`) | `*Options` dataclass imports on default path |
| RRT / RRT* | `extender` (+ `metric`) | `RRTOptions` / `RRTStarOptions` imports for common knobs |
| DP | `StateSpaceGrid` (`x_grid_shape`, `u_grid_shape`, `dt`) | `DynamicProgrammingOptions` imports for common knobs |

**Rule:** the middle column is a **fundamental building block** (like
`Simulator`'s time grid — user must choose resolution or connection law).
Options dataclasses are **workflow** detail — flatten onto the planner ctor
when beginners tune them in every demo.

### Family consistency map

| Layer | Trajopt (TOP) | RRT | DP |
| --- | --- | --- | --- |
| Continuous problem | `PlanningProblem.tf` | same | same (+ cost) |
| Discretization precision | `n_steps` on planner | `extender` params | `StateSpaceGrid` shapes, `dt` |
| Method preset | `transcription="direct_collocation"` | extender type (steering / kinodynamic) | `backend="numpy"\|"jax"\|"loop"` |
| Workflow knobs | `optimizer_method`, `optimizer_options` | `max_nodes`, `goal_bias`, `seed` | `alpha`, `tol`, `max_iterations` |
| Result | `TrajectoryPlan` | `TrajectoryPlan` | `PolicyPlan` |

### RRT / RRT* — target UX (UI-5)

**Today:** `options=None` already defaults internally — better than TOP — but
demos import `RRTOptions` / `RRTStarOptions` for routine tuning.

**Tier 1 (proposed):**

```python
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.rrt import RRTPlanner
from minilink.planning.search.extenders import SteeringExtender

problem = PlanningProblem(sys=sys, x_start=x0, x_goal=x_goal, X=X)

planner = RRTPlanner(
    problem,
    SteeringExtender(max_distance=1.0, resolution=0.05),
    max_nodes=5000,
    goal_bias=0.1,
    seed=0,
)
plan = planner.solve()
```

**RRT*:**

```python
planner = RRTStarPlanner(
    problem,
    extender,
    optimize_after_goal=True,
    cost_tol=0.05,
    convergence_patience=500,
    max_nodes=8000,
)
```

**Tier 2:** `options=RRTStarOptions(live_plot=True, callback=..., nearest_backend="cKDTree")`.

**Keep explicit:** `extender`, `metric` override — not ceremony.

### Dynamic programming — target UX (UI-6)

**Today:** `DynamicProgrammingOptions` optional; `StateSpaceGrid` already uses
flat kwargs (good).

**Tier 1 (proposed):**

```python
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner

grid = StateSpaceGrid(problem, x_grid_shape=(101, 101), u_grid_shape=(41,), dt=0.05)

planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    backend="numpy",
    alpha=1.0,
    tol=0.1,
    max_iterations=1000,
)
policy_plan = planner.solve()
```

**Tier 2:** `options=DynamicProgrammingOptions(record_history=True, verbose=True, interpolation="cubic")`.

**Keep explicit:** `StateSpaceGrid` construction — discretization precision,
analogous to trajopt `n_steps`.

### Out of scope for planner-family flattening

| Area | Why |
| --- | --- |
| `planning/spatial/` scene composition | Domain teaching API; many types are intentional |
| `DiagramSystem` wiring | Structural, not solver-preset |
| `PlanningProblem` / `ProblemParameters` | Math task — stay dataclasses |
| Simulation / control / analysis | Already Simulator-style |

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

### UI-5 — RRT / RRT* constructor flattening

| Work | Detail |
| --- | --- |
| Flat kwargs on `RRTPlanner.__init__` | `max_nodes`, `goal_bias`, `seed`, `goal_tolerance`, `edge_resolution`, … |
| Flat kwargs on `RRTStarPlanner.__init__` | `optimize_after_goal`, `cost_tol`, `convergence_patience`, `gamma`, `rewire`, … |
| Internal | kwargs → `RRTOptions` / `RRTStarOptions`; tier-2 `options=` unchanged |
| Defaults | Same as today's `RRTOptions()` / `RRTStarOptions()` |
| Docs | One RRT demo + README row uses tier 1 |

**Gate:** `pytest tests/unittest/test_rrt.py -q`

### UI-6 — Dynamic programming constructor flattening

| Work | Detail |
| --- | --- |
| Flat kwargs on `DynamicProgrammingPlanner.__init__` | `backend`, `alpha`, `tol`, `max_iterations`, `interpolation`, `out_of_bound_cost`, `final_time`, `verbose` |
| Internal | kwargs → `DynamicProgrammingOptions`; tier-2 `options=` unchanged |
| `StateSpaceGrid` | No change — stays explicit flat ctor |
| Docs | `demo_basics.py` tier-1 example optional |

**Gate:** `pytest tests/unittest/test_dynamic_programming.py -q`

**Sequencing:** UI-5 / UI-6 after UI-1 lands — reuse the same ctor-flattening
machinery and documentation pattern; do not block MPC/trajopt on RRT/DP.

---

## Migration map

| Call site | Action |
| --- | --- |
| `TrajectoryOptimizationPlanner(problem, transcription=DC(...), options=TOP(...))` | Keep working (tier 2) |
| README + hybrid minimal | Switch to tier 1 (UI-2) |
| Teaching notebooks (spatial guide) | Keep explicit step-by-step; optional tier-1 summary cell |
| `DirectCollocationOptions` class | Remain for internal/tier-2; remove from README imports |
| `TrajectoryOptimizationOptions` class | Internal + tier-2 only |
| `RRTOptions` / `RRTStarOptions` | Internal + tier-2 after UI-5 |
| `DynamicProgrammingOptions` | Internal + tier-2 after UI-6 |

---

## Success criteria

1. **Hello-world MPC** uses ≤3 planning imports and **zero** options dataclasses.
2. **Vision pipeline** unchanged: `PlanningProblem` → `TrajectoryOptimizationPlanner` → `ModelPredictiveController`.
3. **Advanced users** retain `transcription=` / `options=` (and RRT/DP tier-2) without behavior regression.
4. **Parity** with `Simulator` / `Optimizer` mental model documented in DESIGN.
5. All existing MPC + trajopt unit tests green after UI-1.
6. **Planner family:** RRT and DP tier-1 paths need zero `*Options` imports for routine demos (UI-5 / UI-6).
7. **Fundamental seams** stay visible: `extender`, `StateSpaceGrid`, `transcription` (when non-default).

---

## Open items (before UI-1 coding)

1. **Required `n_steps`:** keep required on tier 1, or default from `problem.tf` and `dt_mpc` hint? (Locked: required — user must choose precision.)
2. **Top-level aliases:** allow `maxiter=50` as sugar merging into `optimizer_options`, or dict-only? (Lean: dict-only to match `Optimizer`; revisit if demos feel noisy.)
3. **Shooting string preset:** ship in UI-1 or only direct collocation default until shooting MPC path is tested?

---

## Suggested PR slicing

| PR | Contents |
| --- | --- |
| PR-UI-A | UI-1 TOP constructor + tests |
| PR-UI-B | UI-2 README / DESIGN / vision snippet |
| PR-UI-C | UI-3 TOP helpers + optional demo touch-ups |
| PR-UI-D | UI-5 RRT / RRT* flattening + tests |
| PR-UI-E | UI-6 DP flattening + tests |

This document is the **source of truth** for planning UI simplification
across the **planner family**. Constructor flattening takes precedence over
ad-hoc recipe-layer sketches in other cleanup notes.
