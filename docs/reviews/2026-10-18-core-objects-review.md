# The core objects from 10,000 feet: one differentiable path, no twins (2026-10-18)

**Date:** 2026-10-18
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** After the v0.2.0 tag and the differentiable-problem design
([the plan](../plans/differentiable-cost.md), [the options](../plans/differentiable-cost-options.md)),
the maintainer asked whether the core object system as a whole is sound for a compilable,
differentiable path from parameters to trajectory, cost, averaged cost and success rate, with
no code duplication, or whether the big picture needs adjusting. This is that review. Facts
were counted on `dev` at ac9a230b.
**Consolidated:** the plan, the options and the target-architecture ideas this file links to
were merged into [differentiable-problem.md](../plans/differentiable-problem.md) on
2026-10-18 ([the record](2026-10-18-differentiable-problem-consolidation.md)); the links below
are history and this file stays as it was.

## Verdict

The **nouns are right**; the **verbs are duplicated**. Every law a user writes (`f`, `h`,
`g`, `margin`) is written once and traces under JAX through the `xp = array_module(...)`
idiom. The duplication sits one layer up, in the orchestration: integrating, scoring, cutting
at exit, drawing samples and reconstructing signals each exist in two to five copies, one per
consumer, and about half of those copies are NumPy-only. The differentiable path needs no new
noun beyond the compiled evaluator and its result object already planned. It needs one traced
kernel per verb, with the NumPy consumers calling that kernel instead of keeping a twin.

Two consequences for the plan of record: the cost-parameters work and the differentiable
problem should be read as one job (make every noun read its parameters, then run them through
one kernel), and the November consolidation should target the evaluator's method grid and the
scoring copies rather than the nouns.

## 1. The nouns, one line each

| Noun | Role | Reads its params? | Traces under JAX? | Array-native? | Needs for the differentiable path |
| --- | --- | --- | --- | --- | --- |
| `System` / `DynamicSystem` | the law: `f`, `h`, `tf`, ports, `params` | yes | yes (laws); `realize` casts to `int` | yes | nothing in the law; draws through `sample_*(key)`, not `realize` |
| `DiagramSystem` | composition: nested `params`, roles, signals | yes | yes | yes | a `rollout_p` that emits the internal signals |
| `JaxDiagramEvaluator` | the compiled tiers (`f_trace_p`, `rk4_integrate_zoh_trace_p`, `rollout_batch`) | yes | yes | yes | one rollout kernel; the grid trimmed (see §2.1) |
| `CostFunction` | `g`, `h`, `discount_rate`, `evaluate_trajectory` | **no**: 10 methods take `params`, 0 read it; `evaluate_trajectory` casts to `float` | laws yes, evaluator no | laws yes | read `params.cost`; drop the NumPy evaluator for the kernel |
| `Set` / `InputSet` | `margin` (traced), `contains` (NumPy) | **no**: 12 methods take `params`, 0 read it | `margin` yes, `contains` no | half | `contains` derived from `margin`; read `params.sets` |
| `Shape` (geometry) | `sdf(p, t, params)` | **no**: 6 methods, 0 read | yes | yes | read params when the fields plan lands |
| `Distribution` | `sample(key)`, moments | no params by design | yes | yes | nothing |
| `Trajectory` / `StepRollout` | the result of a run: `t, x, u`, signals, reporting verbs | n/a | **no** (`np.asarray(..., float).copy()`) | no | native pytree (ruled, step 2b); one noun for both (see §2.4) |
| `PlanningProblem` / `ProblemParameters` | the frozen statement: system, cost, sets, scene and their params as one pytree | yes (the pytree) | yes | yes | nothing; it is the root of the path |
| `StochasticPlanningProblem` | the draws: `sample_x0/params/disturbances(key)`, `nominal()` | yes | yes | yes | nothing; the evaluator takes its keys |
| `PlanningSolution` | planner output: a controller or a trajectory, the problem, reporting | n/a | no (reporting) | no | built from the kernel's result, never recomputing cost its own way |
| `MonteCarloEvaluator` | batched closed-loop scoring, three backends | partly | `evaluate_jax` yes | yes | absorbed by the compiled evaluator (ruled) |
| `RolloutEnvironment` | the RL step function, reward `−g·dt` | yes | yes | yes | its reward from the same `g` and the same exit rule |
| `MathematicalProgram` / `ParametricMathematicalProgram` | decision variables, cost, constraints; `bind(x0)` | `bind(x0)` only | yes | yes | `bind(params)` (the optimizer-wiring plan) and the `program(wrt=)` bridge |
| `Field` (planned) | a scalar over `(x, u, t)` | planned | planned | planned | absorb the six field classes in `planning/spatial` (see §2.5) |

What is missing: only the result pytree of the compiled evaluator (`Evaluation` in the
options doc). What is surplus: `MonteCarloEvaluator` as a separate noun, and `StepRollout`
as a second trajectory noun. Nothing else needs to appear or disappear.

## 2. The duplications, counted

### 2.1 The evaluator grid

`jax_evaluators.py` is 1763 lines. Its public surface is a product: five families (`f`,
`step`, `outputs`, `rk4_*`, `euler_*`) times up to six suffixes (plain, `_p`, `_trace`,
`_trace_p`, `_ivp`, `_scipy`); the `rk4_*` and `euler_*` families alone are 36 methods. Each
plain method is its `_p` twin with `self.params` bound; each `_trace` method is the jitted
`_trace_p` with the same binding. The NumPy evaluator (646 lines) repeats the grid for the
plain and `_p` columns.

The fix is mechanical and removes no feature: the `_p` methods are the only bodies; the plain
ones are generated by binding the current params (one small helper applied in `__init__` or a
descriptor), and the `_trace` ones by `jax.jit` over the `_p` body. Half the file goes.
This is a November consolidation row, not a prerequisite for the differentiable path, but the
rollout kernel should be written in the trimmed style so it does not add a seventh column.

### 2.2 Five scoring rules

The running cost is integrated in five places with four quadratures:

| Where | Rule | Discount | Exit | Backend |
| --- | --- | --- | --- | --- |
| `CostFunction.evaluate_trajectory` | loop over samples, `float(g(...))` | yes | no | NumPy only |
| `score_trajectory` (planning) | trapezoid, cut at `argmin(inside)`, `h` at finite `tf` | yes | yes | NumPy only |
| transcription `trajectory_cost` | trapezoid + `h` | **no** | no | both |
| `MonteCarloEvaluator.evaluate_jax` | `J += alive·½(g_k+g_{k+1})·dt`, frozen law | yes | yes | JAX only |
| `RolloutEnvironment` reward | left rectangle, `−g·dt` per step | by the learner | by `X.margin` | both |

Four of them can be one traced kernel, `score_rollout(xs, us, ts, params)`, with the
NumPy consumers (`PlanningSolution.cost`, `score_trajectory`, the transcriptions' reporting)
calling it. The transcriptions' *decision-variable* cost stays theirs (it is the objective
the NLP differentiates, built on knots, not on a rollout), but its quadrature and its
handling of `h` and discount should be the same rule, stated once in DESIGN. The RL reward
is the documented exception (a per-step reward is the learner's contract), derived from the
same `g` so the two cannot drift.

### 2.3 Two signal reconstructions

`_compute_port_signals_p` (traced, used by `outputs_trace_p` at the boundary) and
`reconstruct_internal_signals` (NumPy, post hoc, used by the plots) compute the same internal
signals. The plan's `rollout_p` emitting signals from the traced function is the fix; the
NumPy twin becomes a call to it with NumPy arrays, or goes.

### 2.4 Two trajectory nouns and two set predicates

`Trajectory` (continuous, `t`) and `StepRollout` (discrete, `k`) carry the same arrays and
have parallel verbs. Once `Trajectory` is a pytree, one noun with `t = k·dt` serves both;
`StepRollout` becomes a constructor, not a class. `Set.contains` is a NumPy `bool` of what
`margin ≥ 0` already says in traced form; derive it and the twelve `contains` bodies go.

### 2.5 Fields: a seventh copy in waiting

Six field-like classes live in `planning/spatial` today (`FieldGrid`, `WorkspaceField`,
`GaussianField`, `StateField` and its four subclasses, plus `FieldSet` and `FieldCost`).
The fields plan adds a core `Field` with `QuadraticField`, `GridField`, `CallableField`.
It must absorb the six, not sit beside them; `GridField` with a `jnp` interpolation also
makes the lookup-table policy traceable (today `RegularGridInterpolator` from SciPy stops
any closed loop that holds a DP policy from compiling).

### 2.6 Where the path still breaks

- `System.realize(key)` casts the key to `int`: the stochastic problem's `sample_*(key)`
  methods are the traced draws, and the evaluator must use them, never `realize`.
- The MPC controller's inner solve has no implicit gradient: the compiled evaluator refuses
  it, as the Monte Carlo evaluator refuses stateful controllers today, with the message
  saying so.
- The `Simulator` drives the evaluator through `euler_step`, `euler_integrate_*` and the
  SciPy adapters, a separate path from the compiled rollout. That is right (events, adaptive
  steps, the reference path), but a test should pin that fixed-step RK4 from both paths
  agrees to round-off on one diagram, so the compiled path cannot drift from what the plots
  show.

## 3. The end state in one picture

![The planned pipeline](../plans/differentiable-cost-pipeline.png)

Every NumPy consumer sits below the kernels and calls them with NumPy arrays. No consumer
keeps a NumPy twin of a kernel.

## 4. What to change in the plan of record

1. **Read the two jobs as one.** Making costs, sets and shapes read their parameters
   (the cost-parameters plan, scheduled before the robotics-course release) is the first
   half of the differentiable problem; the kernels are the second half. Schedule them
   back to back, and let the kernels' tests be the "done when" of the parameters work.
2. **Add one row to the November consolidation**: trim the evaluator grid to `_p` bodies plus
   generated bindings (§2.1), and fold `StepRollout` into `Trajectory` once it is a pytree
   (§2.4). Both remove code and no feature.
3. **Amend the fields plan** with "absorbs the six `planning/spatial` classes" as a done-when
   (§2.5).
4. **State the quadrature once in DESIGN** (trapezoid, `h` at the final sample, discount
   `e^{−ρt}`, cut at the first exit) and make the transcriptions cite it (§2.2).
5. **Add the agreement test** between the simulator's fixed-step RK4 and the compiled rollout
   (§2.6).

Nothing here changes a user-facing name or removes a user-importable feature; the catalog
and the teaching surface are untouched.

The ideas this review and the same morning's discussion produced are consolidated, as ideas
to review, in [target-architecture.md](../plans/target-architecture.md).
