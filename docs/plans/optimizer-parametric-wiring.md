# Optimizer wiring for parametric trajopt / MPC

Status: **draft plan** (July 2026). Analysis + proposed fix for swapping
`optimizer_method` (e.g. SciPy SLSQP → IPOPT) on the parametric MPC path.

Contracts in code: [DESIGN.md](../../DESIGN.md) §6 (Planning / NLP),
[ROADMAP.md](../../ROADMAP.md) Later (`SolverFactory` / optimizer wiring).

Related: [planning-pipeline-architecture.md](planning-pipeline-architecture.md)
(parametric `x0` bind; pipeline B extends to scene).

---

## Problem statement

Swapping `optimizer_method="ipopt"` on a
`TrajectoryOptimizationPlanner` used for JAX MPC is expected to be an orthogonal
change: the solver layer should only see a compiled program evaluator and
decision vector `z0`. In practice, **offline** trajopt supports IPOPT via the
generic `Optimizer`, but **parametric compile-once MPC** rejects or ignores
non-SciPy methods.

---

## Intended pipeline (layered independence)

```text
PlanningProblem
    → Transcription
        → MathematicalProgram              [offline / rebuild each tick]
        → ParametricMathematicalProgram    [compile-once MPC; h(z, x0)]

Program (either kind)
    → compile → evaluator
        → MathematicalProgramEvaluator
        → JaxParametricProgramEvaluator (+ bind(x0))

Evaluator + z0
    → OptimizerBackend.solve() → OptimizationResult

OptimizationResult
    → Transcription.reconstruct_result() → Trajectory / TrajectoryPlan
```

**Design law:** transcription and parametric bind are upstream; solver choice is
downstream. `OptimizerBackend` adapters (`ScipyMinimizeOptimizer`,
`IpoptOptimizer`) must not know whether the evaluator came from a one-shot
transcribe or a parametric compile.

---

## What works today

| Path | Entry | Solver wiring | IPOPT (`cyipopt`) |
| --- | --- | --- | --- |
| Offline / rebuild | `solve()` / `solve_trajectory_from` without parametric compile | `Optimizer(program, method=…)` | **Yes** — see `examples/scripts/trajopt/trajopt_cartpole_collocation_jax.py` |
| NumPy MPC rebuild | `compile_backend='numpy'`, no parametric compile | Same `Optimizer` via `_make_optimizer` | **Yes** (if optional dep installed) |
| JAX MPC fast path | `compile_parametric_program()` + `solve_trajectory_from` | **Custom** `_make_parametric_optimizer_backend` | **No** — SciPy only |

The generic `Optimizer` registry (`optimization/optimizer.py`) already lists
`scipy_slsqp`, `scipy_trust_constr`, and `ipopt` and constructs the matching
`OptimizerBackend`.

---

## Why parametric MPC breaks on solver swap

`TrajectoryOptimizationPlanner` parametric solve **bypasses** `Optimizer` and
reimplements a narrow slice:

1. **Duplicate method registry** — planner-local `_USER_OPTIMIZER_METHODS`
   contains only `scipy_slsqp` (subset of `Optimizer`'s full registry).

2. **Hardcoded backend** — `_make_parametric_optimizer_backend()` always
   returns `ScipyMinimizeOptimizer`, regardless of `optimizer_method`.

3. **Direct backend call** — `_solve_trajectory_from_parametric()` calls
   `self.optimizer_backend.solve(evaluator, z0)` instead of sharing the same
   factory used by `_make_optimizer()`.

Symptom when setting `optimizer_method="ipopt"` on JAX MPC:

```text
ValueError: Unknown optimizer method 'ipopt' for parametric compile.
Expected one of: scipy_slsqp
```

This is a **wiring** failure, not an NLP or IPOPT incompatibility.

---

## Parametric evaluator is already solver-ready

`JaxParametricProgramEvaluator` exposes the same solver-facing surface as
`MathematicalProgramEvaluator`:

- `objective`, `gradient`, `equality_residual`, `inequality_margin`
- `jacobian_h`, `jacobian_g`
- `scipy_bounds()`

`IpoptOptimizer` duck-types on these methods (same as SciPy). No transcription
change is required to run IPOPT on the parametric NLP — only backend selection
must be unified.

### Known solver limitations (orthogonal to parametric)

| Feature | IPOPT |
| --- | --- |
| Per-iterate `callback` / `record_history` | Not supported (`IpoptOptimizer` raises) |
| `use_hessian=True` on parametric evaluator | Evaluator reports `has_hessian=False`; IPOPT uses its own quasi-Newton |
| Optional dependency | Requires `cyipopt` (`minilink[ipopt]` / conda extra) |

---

## Root cause (architecture)

| Layer | Should know | Today |
| --- | --- | --- |
| `MathematicalProgram` | Pure `J, h, g, bounds` | Clean |
| `ParametricMathematicalProgram` | `h(z, x0)` runtime param | Localized ✓ |
| Evaluator | Compile + `bind(x0)` | Localized ✓ |
| `OptimizerBackend` | Evaluator + `z0` → result | Clean |
| **Planner parametric branch** | Pick backend from method preset | **Leaks: duplicate registry + fixed SciPy** |

Offline path is fine. Parametric path should differ only at **evaluator +
bind**, not at **solver catalog**.

---

## Proposed fix (minimal, keeps layers independent)

### P1 — Shared optimizer backend factory

Extract from `optimization/optimizer.py` (or new `optimization/presets.py`):

```python
def make_optimizer_backend(
    method: str,
    *,
    options: dict | None = None,
    **method_kwargs,
) -> OptimizerBackend:
    """Map user-facing optimizer_method to a concrete OptimizerBackend."""
```

- Single registry: same `_USER_OPTIMIZER_METHODS` as `Optimizer`.
- `TrajectoryOptimizationPlanner._make_parametric_optimizer_backend` calls this
  factory instead of constructing `ScipyMinimizeOptimizer` directly.
- **Delete** planner-local `_USER_OPTIMIZER_METHODS`.

### P2 — Unified solve shape (planner internal)

Both offline and parametric paths should read:

```python
backend = make_optimizer_backend(method, options=...)
result = backend.solve(evaluator, z0, callback=maybe_cb)
plan = transcription.reconstruct_result(result, ...)
```

Offline keeps `Optimizer` as orchestrator (compile program + disp + timing).
Parametric keeps pre-built evaluator + `bind(x0)` — **evaluator differs,
solver API does not**.

### P3 — Evaluator contract note (optional)

Document the solver-facing evaluator surface in
`optimization/evaluators/program_evaluator.py`. `JaxParametricProgramEvaluator`
implements the same contract by duck typing; no need to spread parametric logic
into MPC, hybrid, or transcription.

### P4 — Tests

| Test | Asserts |
| --- | --- |
| `test_parametric_backend_factory_ipopt` | `make_optimizer_backend("ipopt")` returns `IpoptOptimizer` |
| `test_planner_parametric_ipopt_smoke` | `compile_parametric_program` + one `solve_trajectory_from` with IPOPT (skip if no `cyipopt`) |
| `test_planner_parametric_rejects_bad_method` | Unknown method still raises clearly |

### P5 — Docs sync

- `DESIGN.md` §6: path × solver matrix (offline vs parametric vs callback).
- Delete stale implication that parametric path is SciPy-only by design.

---

## Path × solver matrix (target contract)

| Planner mode | `optimizer_method` | IPOPT | Callback / `record_history` |
| --- | --- | --- | --- |
| `solve()` / rebuild `solve_trajectory_from` | `ipopt` | Yes | Callback: SciPy only |
| `compile_parametric_program` + MPC | `ipopt` | **Yes after P1** | Callback: SciPy only |
| NumPy rebuild MPC | `ipopt` | Yes | Callback: SciPy only |

---

## Non-goals (this plan)

- NumPy parametric compile (rebuild-each-tick is the NumPy MPC model).
- IPOPT per-iterate callbacks (SciPy-only until cyipopt exposes a hook).
- Pipeline B scene bind (`J(z, p)`) — separate plan; same `bind()` idea on
  evaluator, same solver factory downstream.
- Forcing `ParametricMathematicalProgram` into `MathematicalProgram` — keep
  parallel types; unify at evaluator + backend boundary only.

---

## Decision summary

| Question | Decision |
| --- | --- |
| Should solver choice be orthogonal to parametric vs rebuild? | **Yes** |
| Is IPOPT incompatible with parametric NLP math? | **No** — wiring only |
| Where to fix? | Shared `make_optimizer_backend`; delete planner duplicate |
| Keep parametric localized? | **Yes** — only `bind(x0)` + `ParametricMathematicalProgram` differ upstream |

---

## Target usage (after P1)

```python
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=40,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="ipopt",
    optimizer_options={"max_iter": 500, "tol": 1e-6},
)
# Offline
plan = planner.solve()

# MPC fast path — same optimizer_method
planner.compile_parametric_program()
mpc = ModelPredictiveController(planner, dt_mpc=0.2)
```

Each MPC tick: `evaluator.bind(x0)` → `backend.solve(evaluator, z0)` →
`reconstruct_result` — no custom solver path in `control/mpc/`.
