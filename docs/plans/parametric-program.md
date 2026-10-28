# The parametric program: every problem parameter bound at solve time

Status: **draft plan**, its shape decided (2026-10-27, preliminary, open to readjustment with
the final architecture).
- **Decided:**
  - one noun, `MathematicalProgram(..., params=None)`, mirroring `f(x, u, t; params)`; no
    `ParametricProgram` and no `bind` on programs (R130,
    [record](../reviews/2026-10-27-design-questions-rulings.md));
  - the convention for parameters and seeds (R131, same record; §3.1);
  - the planner's compiled face, whose `pv.program` is this program, and the MPC binding leaves
    through `inputs=` (R148, [record](../reviews/2026-10-27-core-vision-decisions.md); the design
    home is [planner-and-mpc.md](planner-and-mpc.md));
  - the problem's `t_start` leaf, the start pinned from `x_start` (R148; §3.3);
  - the fixed-count solve, `Optimizer(program, method=, iterations=K)` (R149);
  - the plant's key in every tree is `plant` (R26, 2026-10-23).
- **Still open:**
  - R130's performance gate (§2);
  - the name of `program.params_at(z)`, the full tuned tree from an exported program (the
    naming question).
- **Not started:** PP-1–PP-5 and OPW-1–OPW-5. They build on the optimizer method table and the
  first-order backend that S74 landed (R74, 2026-10-24), on the compiled problem's program
  export, and on A2's cost leaves.
- **Scope:** direct collocation and the MPC first; the other transcriptions later (§5).

**Where it sits:** the core's parametric program, with the stochastic program as its seed case
(the maintainer's direction of 2026-10-26: every program deterministic given its parameters, the
seed one of them; solving a stochastic program is solving over seeds,
[native-optimizers.md](native-optimizers.md) §7).
- [core-architecture.md](core-architecture.md) §7;
- rows in [TODO.md](TODO.md) §1.7.

Contracts in code: [DESIGN.md](../dev/DESIGN.md) §4 (Parameters, the planning tiers) and §6
(Planning / NLP).

---

## 1. The problem

A model predictive controller solves one trajectory optimization per tick. The program is
compiled once and solved many times, so whatever changes between ticks must be a runtime
argument of the compiled program. Today only the measured state is:

- `ParametricMathematicalProgram` is `J(z)`, `h(z, x0)`, `g(z)`: "only `x0` may vary between
  solves; all other problem data is frozen at compile time".
- Direct collocation captures the plant's nominal parameters (`nominal_plant_params(problem)`),
  the cost's (`problem.cost.params`) and the sets in its closures when it transcribes, so the
  plant, the cost and the sets are constants of the compiled program.
- `solve_trajectory_from(x0, params=…)` and the MPC block's `compute_command` refuse every
  key but the reserved `scene` (`require_online_params`), and the MPC block's port
  functions drop the `params` they receive.
- `JaxParametricProgramEvaluator` repeats about half of the plain JAX program evaluator to
  carry its one extra argument.

What waits on it: an MPC that plans with the latest friction estimate of an online
estimator (the plant's `mu` as a live value), a tracking target that moves without a
rebuild (a cost parameter), cost weights tuned between runs, obstacles that move (the scene,
after A3), and the sensitivity of the plan to any of them, later.

## 2. The decided shape: one noun with optional parameters (R130)

```python
# The library side: transcribed and compiled once
program = transcription.transcribe(problem)          # J(z, params=None), h(z, params=None), g(z, params=None)
p = program.params                                   # the problem's tree: x_start, t_start, plant, cost, sets, …

# Each solve: new values, no recompile; a partial tree merged once onto the defaults
opt = Optimizer(program, method="ipopt")
result = opt.solve(params={"x_start": y, "plant": {"mu": mu_hat}}, z0=z_warm)

# A fixed count: K steps, one lax.scan, pure and differentiable (R149)
Optimizer(program, method="adam", iterations=200).solve(params=p)

# The student side, unchanged
MathematicalProgram(n_z=3, J=J, grad_J=grad)         # J(z): no parameters declared
```

- **The functions take `(z, params=None)`**; `params=None` reads the frozen default, as on a
  `System` and the step model (DESIGN §5).
- **One evaluator body.** A plain program is the case with no declared parameters, so the
  duplicate of §1 goes instead of growing.
- **The optimizer's contract does not move:** backends see a flat `z` (§6's design law), and
  minilink stays out of optimization modelling (CONSTITUTION §1.4).
- **It is the case the differentiable problem's bridge names:** the compiled MPC program is "no
  decision leaves, parameters bound" (the program export, `pe.program`: DESIGN §6).
- **The performance gate (R156).** Before the change lands, benchmark the compiled program
  carrying `params` against the same program with its parameters frozen (compile time and
  per-solve time, on the MPC and the PID examples); the bar is 2×, one of the five numbers of the
  budget script in core-architecture §11. If carrying them costs more, an export option compiles
  a program with its parameters baked in as constants.

### 2.1 D25, the two asks (moved from end-state-api.md, 2026-10-26)

Both are answered (2026-10-27, preliminary):
- **A hand-written program may declare its own parameters:** `MathematicalProgram(n_z, J,
  params=…)` with `J(z, params=None)` (R130), so tutorial 08 can solve one NLP for several
  parameter values.
- **The MPC's live parameters are bindings,** `inputs=` from leaves of the planner's tree to the
  block's ports, an unwired port reading the leaf's value (R148; replaces the `live=` and
  `parameter_ports=` candidates):

```python
mpc  = ModelPredictiveController(planner, inputs={"x_start": "y", "t_start": "t", "plant.mu": "mu_hat"})
loop = (mpc % 0.1) @ plant      # with an estimator's mu_hat wired to the port, the estimate drives the plan
```

## 3. The design

### 3.1 The parameter tree (R131)

- `p` is the problem's tree, assembled from the objects' own `params` (R41: one home per
  number): `x_start`, `t_start`, `plant`, `cost`, the sets, `discount_rate`, and `scene` once A3
  gives workspace geometry its home. A planner adds `"method"` and `seed` (R148).
- Its structure is fixed at compile; its values vary without retracing (JAX takes numeric
  leaves; the diagram parameters already work this way, DESIGN §4).
- **Complete trees inside, one merge at the boundary.** A program's functions always see a
  complete tree; at `solve(params=...)` a partial tree is merged once onto the defaults and
  checked against the structure: an unknown path raises and names the known ones; a leaf of a
  new shape raises; `scene` raises until A3.
- **The seed** is found by the `seed` key, an integer leaf; integer leaves (the seed, a step
  problem's `t_start`) never enter `wrt`, and asking for one is refused.
- **Sizes are structural:** the horizon `N`, the iterations `K`, batch sizes are constructor
  arguments; only values live in `params`.
- **The warm start is an argument of the solve,** `z0=`, not of the program.
- `J(z, key)` is dropped: the seed lives in `params`. How a solver uses the seed (fixed,
  `vmap`ped, advanced per iteration) is an `Optimizer` strategy for later (v0.4); the core keeps
  the sample-average export `pe.program(wrt, key=, n_trials=)`.

### 3.2 The program and its evaluator

`MathematicalProgram` holds `n_z`, the default `params`, `J`, `h`, `g`, the box and the
metadata (today's `ParametricMathematicalProgram` folds in with PP-1). One JAX evaluator jits
the functions of `(z, params)` and their derivatives with respect to `z`; the plain program's
evaluator is the same body. The solver-facing surface (`objective`, `gradient`, `jacobian_h`,
`jacobian_g`, `scipy_bounds`) is unchanged.

### 3.3 Direct collocation

The transcription builds every closure on `p`:
- the dynamics `f(x_k, u_k, t_k, p["plant"])` on the knots `t_k = p["t_start"] + k·h`;
- the cost integral with `p["cost"]`, the discount counted from `t_start`,
  `e^{−ρ(t − t_start)}` (R148; costs and sets read absolute time);
- the set constraints with the sets' leaves;
- the initial residual `x_0 − p["x_start"]`: the start pinned from `x_start`, `X0` derived from
  it (R148; today shooting and collocation read `X0` first).

The plant leaves work at once (`f` takes its parameters already); the cost leaves come with A2
(landed in v0.3), the set leaves with A5's sets reading `params`. The rebuild path (the NumPy
MPC) takes the same tree and retranscribes with it, so the two paths answer one API at two
speeds. Collocation uses the continuous `f`, not the step model, so this work does not depend on
the hybrid loop.

### 3.4 The planner's online call

R148 replaces it: the compiled planner's `pv.solve(params=None, z0=None) -> (z, status)` takes
the tree, and `pv.program` is this program ([planner-and-mpc.md](planner-and-mpc.md) §3).
`require_online_params` and its list of deferred keys go, replaced by the tree's check (§3.1).

### 3.5 The MPC block's live parameters

R148's `inputs=` bindings (§2.1); the design is [planner-and-mpc.md](planner-and-mpc.md) §6.1.

## 4. The steps (one commit each; the AGENTS recipe where behaviour must hold)

- **PP-0 The names:** the key `plant` (R26); the noun (R130); the MPC's `inputs=` (R148). Left:
  `params_at` (the naming question).
- **PP-1 The program with parameters:** §2 and §3.2, one evaluator, the performance gate first.
  Done when the MPC baselines (`test_mpc.py`, the trajopt parity scenario) are byte-identical
  with `params = {"x_start": …}` and the plain programs of tutorial 08 and the optimization demos
  solve unchanged.
- **PP-2 Collocation reads the plant, the start and `t_start` from `p`** (§3.3). Done when one
  compiled program, solved with two values of a plant parameter, gives the two solutions a
  rebuild gives, `t_start = 0` reproduces today's numbers, and the friction-aware MPC example
  (the dynamic racecar's planner `mu` from a port) runs on the research lane.
- **PP-3 The cost and set leaves,** after A5's sets (A2 landed in v0.3). Done when a moving
  tracking target binds without a rebuild and matches the rebuilt program.
- **PP-4 The planner's compiled face and the MPC's bindings** (§3.4, §3.5), the rebuild path on
  the same API. Done when an unknown path raises naming the known ones, an unwired port reads
  its leaf, and the NumPy and JAX MPC paths agree on a bound parameter.
- **PP-5 The contracts:** DESIGN §4 (the solve binds the tree) and §6 (the noun, the
  evaluator), once the code lands; this plan's landed parts leave it.

MPC is not a week of the GMC714 syllabus, but its teaching name is frozen (R49, G3; pinned when
v0.3 merges).

## 5. Later, not scheduled

- The scene leaf (moving obstacles), with A3.
- Single shooting with parameters is in the core proof of concept's row (TODO §1.8); multiple
  shooting has no parametric form today.
- The sensitivity of the solved plan to `p` (implicit differentiation of the optimality
  conditions, R131): a differentiable MPC. The native solvers that run this program inside a
  fixed-iteration MPC tick are [native-optimizers.md](native-optimizers.md) (§4.3, §4.6).

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
        → MathematicalProgram(..., params)   [one noun; the plain program has no declared params]

MathematicalProgram
    → compile → evaluator (functions of z and params)

Evaluator + z0 (+ params)
    → OptimizerBackend.solve() → OptimizationResult

OptimizationResult
    → Transcription.reconstruct_result() → Trajectory / PlanningSolution
```

**Design law:** transcription and the parameter values are upstream; solver choice is
downstream. `OptimizerBackend` adapters (`ScipyMinimizeOptimizer`, `IpoptOptimizer`) do not know
whether the evaluator came from a one-shot transcription or a compiled program with parameters.

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
  path keeps `Optimizer` as its orchestrator, the compiled path its evaluator and the bound
  values.
- **OPW-3 The evaluator contract**, documented in
  `optimization/evaluators/program_evaluator.py` (with PP-1, one evaluator body).
- **OPW-4 Tests:** the factory returns `IpoptOptimizer` for `"ipopt"`; one compiled MPC
  solve with IPOPT (skipped without `cyipopt`); an unknown method still raises clearly.
- **OPW-5 Docs:** DESIGN §6's path × solver matrix; the implication that the compiled path is
  SciPy-only by design removed.

### Target contract

| Planner mode | `optimizer_method` | IPOPT | Callback / `record_history` |
| --- | --- | --- | --- |
| `planner.solve()` (offline or rebuild) | `ipopt` | Yes | SciPy only |
| `planner.compile()` and the MPC on it | `ipopt` | Yes after OPW-1 | SciPy only |
| NumPy rebuild MPC | `ipopt` | Yes | SciPy only |

```python
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=40,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="ipopt",          # today's keywords; the constructor's names go with the sweep (R134)
    optimizer_options={"max_iter": 500, "tol": 1e-6},
)
plan = planner.solve()                 # offline
mpc  = ModelPredictiveController(planner)   # compiles the planner, the same optimizer_method
loop = (mpc % 0.2) @ plant
```

Non-goals of this part: a NumPy parametric compile (rebuild-each-tick is the NumPy MPC);
IPOPT per-iterate callbacks.
