# Planning pipeline architecture

Status: **partial** (July 2026). Result / metadata wrappers and compile-once
parametric `x0` are landed. Open work is **pipeline B** — runtime scene /
obstacle bind without re-JIT.

Contracts in code: [DESIGN.md](../../DESIGN.md) §6,
[ROADMAP.md](../../ROADMAP.md) §5.5.

Related future taxonomy: [standard-planning-problems.md](standard-planning-problems.md).

---

## Landed baseline

| Piece | Status |
| --- | --- |
| `PlanningProblem` + `Planner` ABC | Done |
| `TrajectoryPlan` + `SolveMetadata` | Done |
| Trajopt / RRT return `TrajectoryPlan` | Done |
| `TrajectoryOptimizationPlanner.compile_parametric_program` + `solve_trajectory_from(x0)` | Done (`x0` only) |
| `ModelPredictiveController` in `control/mpc/` | Done |
| Online `params` façade | Slim only — `None`/`{}` ok; `{"scene": …}` → `NotImplementedError` |
| Soft obstacle costs via `Scene` at problem-build time | Done (centers baked into JIT) |

---

## Still open — pipeline B (parametric scene)

Do **not** rebuild `Scene` + re-`prepare()` every MPC tick for moving or newly
detected obstacles. Use a **fixed-capacity obstacle bank** (`K` slots, dynamic
centers, `active` mask). Weight-only scaling does **not** remove an obstacle
from min-based clearance fields — inactive slots must yield effectively
infinite SDF.

| ID | Improvement | Notes |
| --- | --- | --- |
| **B1** | Generalize `ParametricMathematicalProgram` runtime pytree `p` | Today only `x0`; target `{"x0", "scene", ...}` |
| **B2** | Extend parametric evaluator `bind(p)` | `J(z, p_scene)`, `h(z, p_x0)`, optional `g(z, p_scene)` |
| **B3** | `ProblemParameters.scene` + `SceneParameters` | centers `(K, dim)`, radii, `active` mask |
| **B4** | `ObstacleBank(K, dim)` with JAX-traceable `sdf` | Inactive slots → large SDF |
| **B5** | `solve_trajectory_from(x, params=…)` scene path | Per-tick perception without re-JIT |
| **B6** | Perception-update MPC demo + compile-once timing test | |
| **B7** | Document bank semantics in DESIGN §6 | Contract sync |
| **B8** | Trajopt scenario sweeps via same parametric evaluator | Vary scene arrays without re-JIT |
| **B9** | Online tick parametric feed from MPC loop | `p` each tick without recompile |

**Obstacle update policy:** nearest-free-slot for new detections; deactivate far
slots; if detections exceed `K`, keep closest `K` or rare re-`prepare()` with
larger capacity.

**Start soft costs only** for moving obstacles; hard `FieldSet` margins need
parametric `g(z, p)` (heavier follow-up).

---

## Deferred / optional result polish

| ID | Improvement | Notes |
| --- | --- | --- |
| **A3** | `PolicyPlan` wrapper around DP results | Optional; DP still returns `DynamicProgrammingResult` |
| **A5** | `Planner.result_kind` marker | Typing aid only |

---

## Decision summary

| Question | Decision |
| --- | --- |
| Unified `PlanningProblem` input? | **Yes — keep** |
| Split planner ABC hierarchies? | **No** |
| Result families? | **`TrajectoryPlan` vs policy results** (DP wrapper optional) |
| Recompile when obstacle moves? | **No** — fixed `K` bank + dynamic arrays in JIT args |
| Variable obstacle count? | **Pad to `K` + `active` mask** |
| Weight-only to hide obstacles? | **Not for min/union clearance**; mask or `+∞` SDF |

---

## Target mental model

> **PlanningProblem** — what to achieve.
> **Planner** — how to compute it.
> **TrajectoryPlan** — open-loop schedule.
> **Parametric NLP** — compile structure once; bind numeric scenario data each tick.
> **Scene bank** — fixed `K`; centers / `active` in `p`, not in the JIT closure.

---

## Non-goals (this plan)

- Shooting / multiple-shooting MPC transcription
- Mandatory LQR wrapper under `planning/`
- Forcing policy fields into MPC or trajopt results
- Hard moving-obstacle constraints in the first parametric scene pass
