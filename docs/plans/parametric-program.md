# The parametric program: every problem parameter bound at solve time

Status: **draft plan**. The optimizer table (OPW-1–OPW-5, §6) dates from July 2026; the
parameter binding (PP-0–PP-5) and the recommendation of §2 were added 2026-10-22 and wait
on the asks of ROADMAP §6. Rung: OPW-1–OPW-5 in v0.3 wave V with S74 (the differentiable
problem's step 5); the binding (row S76) in v0.4 on S74's evaluator and A2's cost leaves,
its names (PP-0) ruled in V1's step 0 session in v0.3 (moved from wave V on 2026-10-22: the
MPC band is research lane and nothing GMC714 teaches in December needs it). Lane: research
(the MPC band is provisional, ROADMAP §2). Scope: direct collocation and the MPC block
first; the other transcriptions later (§5).

Contracts in code: [DESIGN.md](../dev/DESIGN.md) §4 (Parameters, the planning tiers) and §6
(Planning / NLP).

---

## 1. The problem

A model predictive controller solves one trajectory optimization per tick. The program is
compiled once and solved many times, so whatever changes between ticks must be a runtime
argument of the compiled program. Today only the measured state is:

- `ParametricMathematicalProgram` is `J(z)`, `h(z, x0)`, `g(z)`: "only `x0` may vary between
  solves; all other problem data is frozen at compile time".
- Direct collocation captures `problem.params.system`, `.cost` and `.sets` in its closures
  when it transcribes, so the plant, the cost and the sets are constants of the compiled
  program.
- `solve_trajectory_from(x0, params=…)` and the MPC block's `compute_command` refuse every
  key but the reserved `scene` (`require_online_params`), and the MPC block's port
  functions drop the `params` they receive.
- `JaxParametricProgramEvaluator` repeats about half of the plain JAX program evaluator to
  carry its one extra argument.

What waits on it: an MPC that plans with the latest friction estimate of an online
estimator (the plant's `mu` as a live value), a tracking target that moves without a
rebuild (a cost parameter), cost weights tuned between runs, obstacles that move (the scene,
after A3), and the sensitivity of the plan to any of them, later.

## 2. The recommendation: one family, the frozen program a binding

Three ways to give a program parameters:

| Option | What changes | Cost |
| --- | --- | --- |
| A. Two parallel types (today's, widened from `x0` to every parameter) | nothing structural | two evaluators to keep in step; the duplicate grows with each parameter family |
| B. One class whose functions always take `(z, p)` | every producer and every hand-written program changes signature | tutorial 08 and the optimization demos break; a student writes `J(z, p)` for a problem with no parameter |
| **C. One family: the parametric form, the frozen program its binding** (recommended) | the library builds `J(z, p)`, `h(z, p)`, `g(z, p)`; `bind(p)` gives the plain `MathematicalProgram` `J(z)`; one evaluator body compiles functions of `(z, p)`, a hand-written program the case of an empty `p` | one verb, `bind` |

```python
# The library side: transcribed and compiled once
parametric = transcription.transcribe_parametric(problem)      # J(z, p), h(z, p), g(z, p)
p = parametric.params                                          # {"x0": …, "system": {…}, "cost": {…}, "sets": {…}}

# Each MPC tick: new values, no recompile
evaluator.bind({"x0": y, "system": {"mu": mu_hat}})            # a partial override, merged onto p
result = backend.solve(evaluator, z0)

# The frozen program, for one-off use
program = parametric.bind(p)                                   # a MathematicalProgram, J(z)
Optimizer(program, method="ipopt").solve()

# The student side, unchanged
MathematicalProgram(n_z=3, J=J, grad_J=grad)                   # J(z): an empty p underneath
```

Why C:

1. It is the rule the other plans already set. The step model "takes `params` always;
   freezing is a binding", and the parameterless call students use stays as a binding
   ([sampled-path.md](sampled-path.md) §3); the system evaluators have a parametric tier and a
   frozen one bound over it (DESIGN §4). The program follows the plant and the step model.
2. One evaluator. A plain program compiles as `(z, p)` with `p = {}`, so the duplicate of §1
   goes instead of growing.
3. The optimizer's contract does not move: backends see a flat `z` (§6's design law), and
   minilink stays out of optimization modelling (CONSTITUTION §1.4).
4. It is the case the differentiable problem's bridge names: the compiled MPC program is "no
   decision leaves, parameters bound" ([differentiable-problem.md](differentiable-problem.md)
   §4.6), which is `bind`.

Open with C (the asks, ROADMAP §6): the family's name (keep `ParametricMathematicalProgram`,
or a shorter noun, in the naming session of V1's step 0); whether a hand-written program may
declare its own `p` (free, and tutorial 08 could solve one NLP for several values of a
parameter); `bind` on both the evaluator (the fast path, compiled functions kept) and the
program (the plain noun returned).

## 3. The design

### 3.1 The parameter tree

`p` is a pytree with the planning tiers of `ProblemParameters` as keys: `x0`, `system`,
`cost`, `sets`, and `scene` once A3 gives workspace geometry its home. Its sample is the
problem's nominal values, so its structure is fixed at compile time and its values vary
without retracing (JAX takes numeric leaves; the diagram parameters already work this way,
DESIGN §4). A bind takes a partial tree, merged onto the bound values and checked against
the structure: an unknown path raises and names the known ones; a leaf of a new shape
raises; `scene` raises until A3. The key for the plant is the naming session's (V1's step
0): the planning tiers say `system`, the differentiable problem's evaluator says `plant`, and
one word serves both.

### 3.2 The program and its evaluator

`ParametricMathematicalProgram` (or its new name) holds `n_z`, the sample `params`,
`J(z, p)`, `h(z, p)`, `g(z, p)`, the box and the metadata; `bind(p)` returns a
`MathematicalProgram` whose functions close over `p`. One JAX evaluator jits the functions
of `(z, p)` and their derivatives with respect to `z`; `bind(p)` on it replaces the bound
tree; the plain program's evaluator is the same body with an empty tree. The solver-facing
surface (`objective`, `gradient`, `jacobian_h`, `jacobian_g`, `scipy_bounds`) is unchanged.

### 3.3 Direct collocation

`transcribe_parametric` builds every closure on `p`: the dynamics
`f(x_k, u_k, t_k, p["system"])`, the cost integral with `p["cost"]`, the set constraints with
`p["sets"]`, the initial residual `x_0 − p["x0"]`. The plant leaves work at once (`f` takes
its parameters already); the cost leaves need A2, the set leaves A5's sets reading
`params`. The rebuild path (the NumPy MPC) takes the same tree and retranscribes with it,
so the two paths answer one API at two speeds.

### 3.4 The planner's online call

`solve_trajectory_from(x0, params=…)` takes the partial tree of §3.1 and binds it with `x0`;
`require_online_params` and its list of deferred keys go, replaced by the tree's check.
`compute_command(y, params=…)` forwards it.

### 3.5 The MPC block's live parameters

The block declares, at construction, optional input ports that feed leaves of the tree (a
port `mu` feeding `system.mu`, a port `x_ref` feeding a cost target). An unwired port reads
the nominal value (RULES 4.5), so the diagram's wiring does not depend on whether a live
value is connected; the port functions forward the values to the tick's bind instead of
dropping them. The argument's name and form are the naming session's.

## 4. The steps (one commit each; the AGENTS recipe where behaviour must hold)

- **PP-0 The names**, in V1's step 0 session: the family's noun, the plant's key, the MPC
  argument, `bind`.
- **PP-1 The program with parameters**: the family of §3.2, one evaluator, `bind` on both.
  Done when the MPC baselines (`test_mpc.py`, the trajopt parity scenario) are byte-identical
  with `p = {"x0": …}` and the plain programs of tutorial 08 and the optimization demos solve
  unchanged.
- **PP-2 Collocation reads the plant and the start from `p`**. Done when one compiled
  program, bound with two values of a plant parameter, gives the two solutions a rebuild
  gives, and the friction-aware MPC example (the dynamic racecar's planner `mu` from a port)
  runs on the research lane.
- **PP-3 The cost and set leaves**, after A2 and A5's sets. Done when a moving tracking
  target binds without a rebuild and matches the rebuilt program.
- **PP-4 The planner's online call and the MPC ports** (§3.4, §3.5), the rebuild path on the
  same API. Done when an unknown path raises naming the known ones, an unwired port reads its
  nominal, and the NumPy and JAX MPC paths agree on a bound parameter.
- **PP-5 The contracts**: DESIGN §4 (the online call binds the tree) and §6 (the family, the
  evaluator), once the code lands; this plan's landed parts leave it.

**Order**: PP-0 with V1's step 0; PP-1, PP-2 and PP-4 with V1's step 5 (S74 and OPW-1–OPW-5
touch the same evaluator and backend factory); PP-3 right after A2 (V1's step 6); the scene
leaf with A3 in v0.4. No dependency on S75 or S31: collocation uses the continuous `f`, not
the step model. Before wave G, so the MPC topic G1 weighs for GMC714 meets one API.

## 5. Later, not scheduled

- The scene leaf (moving obstacles), with A3.
- Single shooting with parameters; multiple shooting has no parametric form today.
- The sensitivity of the solved plan to `p` (implicit differentiation of the optimality
  conditions): a differentiable MPC, beside the traceable inner solver
  [sampled-path.md](sampled-path.md) §8 anticipates.

---

## 6. The optimizer table for both paths (OPW-1–OPW-5)

Swapping `optimizer_method` (SciPy SLSQP → IPOPT) on the compiled MPC path should be an
orthogonal change: the solver layer sees an evaluator and `z0` only. Offline trajectory
optimization supports IPOPT through the generic `Optimizer`; the compiled path rejects every
method but SciPy's.

### The pipeline

```text
PlanningProblem
    → Transcription
        → MathematicalProgram              [offline / rebuild each tick]
        → ParametricMathematicalProgram    [compile-once MPC; bound parameters, §2]

Program (either kind)
    → compile → evaluator (+ bind(p) on the parametric one)

Evaluator + z0
    → OptimizerBackend.solve() → OptimizationResult

OptimizationResult
    → Transcription.reconstruct_result() → Trajectory / PlanningSolution
```

**Design law:** transcription and the bind are upstream; solver choice is downstream.
`OptimizerBackend` adapters (`ScipyMinimizeOptimizer`, `IpoptOptimizer`) do not know whether
the evaluator came from a one-shot transcription or a compiled parametric program.

### What works today

| Path | Entry | Solver wiring | IPOPT (`cyipopt`) |
| --- | --- | --- | --- |
| Offline / rebuild | `solve()` / `solve_trajectory_from` without parametric compile | `Optimizer(program, method=…)` | **Yes** (`examples/demos/trajopt/trajopt_cartpole_collocation_jax.py`) |
| NumPy MPC rebuild | `compile_backend='numpy'`, no parametric compile | the same `Optimizer` via `build_optimizer` | **Yes** (with the optional dependency) |
| JAX MPC fast path | `compile_parametric_program()` + `solve_trajectory_from` | **custom** `build_parametric_optimizer_backend` | **No**, SciPy only |

### Why the compiled path breaks on a solver swap

`TrajectoryOptimizationPlanner`'s parametric solve bypasses `Optimizer` and reimplements a
narrow slice: a planner-local method registry with `scipy_slsqp` only,
`build_parametric_optimizer_backend()` returning `ScipyMinimizeOptimizer` whatever the method,
and `solve_trajectory_from_parametric()` calling that backend directly. Setting
`optimizer_method="ipopt"` raises `Unknown optimizer method 'ipopt' for parametric compile`:
a wiring failure, not an NLP or IPOPT incompatibility. The parametric evaluator already
exposes the solver-facing surface IPOPT duck-types on.

Known solver limits, orthogonal to the path: IPOPT has no per-iterate `callback` /
`record_history`; the parametric evaluator reports no Hessian, so IPOPT uses its own
quasi-Newton; `cyipopt` is optional.

### The steps

- **OPW-1 One backend factory.** `make_optimizer_backend(method, *, options=None, **kwargs)`
  in `optimization/` maps a method to its `OptimizerBackend` from `Optimizer`'s one registry;
  the planner's compiled path calls it and its local registry is deleted. S74's new presets
  register here.
- **OPW-2 One solve shape.** Both paths read `backend = make_optimizer_backend(…)`,
  `result = backend.solve(evaluator, z0, callback=…)`, `reconstruct_result(…)`; the offline
  path keeps `Optimizer` as its orchestrator, the compiled path its evaluator and `bind`.
- **OPW-3 The evaluator contract**, documented in
  `optimization/evaluators/program_evaluator.py` (with PP-1, one evaluator body).
- **OPW-4 Tests**: the factory returns `IpoptOptimizer` for `"ipopt"`; one compiled MPC
  solve with IPOPT (skipped without `cyipopt`); an unknown method still raises clearly.
- **OPW-5 Docs**: DESIGN §6's path × solver matrix; the implication that the compiled path is
  SciPy-only by design removed.

### Target contract

| Planner mode | `optimizer_method` | IPOPT | Callback / `record_history` |
| --- | --- | --- | --- |
| `solve()` / rebuild `solve_trajectory_from` | `ipopt` | Yes | SciPy only |
| `compile_parametric_program` + MPC | `ipopt` | Yes after OPW-1 | SciPy only |
| NumPy rebuild MPC | `ipopt` | Yes | SciPy only |

```python
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=40,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="ipopt",
    optimizer_options={"max_iter": 500, "tol": 1e-6},
)
plan = planner.solve()                      # offline
planner.compile_parametric_program()        # MPC fast path, the same optimizer_method
mpc = ModelPredictiveController(planner, dt_mpc=0.2)
```

Non-goals of this part: a NumPy parametric compile (rebuild-each-tick is the NumPy MPC);
IPOPT per-iterate callbacks.
