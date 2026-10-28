# Compile once, in the form the call needs

Status: proposed 2026-10-28 (the maintainer's ask after the v0.3 run: "frozen parameters, or
parameters as arguments for a parametric computation, each as fast as if hard-coded, behind the
one method entry; `jit` at the right place on every pipeline"). Two independent reviews, then
implementation on `dev-v0.3-compile`.
Where it sits: a hardening row of the v0.3 core (TODO §1.10), beside the wiring-time refusals.

## 1. What the audit measured (CartPole / pendulum, CPU, float64, `dev-v0.3` at `1e9d2d6e`)

A second identical call, the XLA compilations it triggers:

| Pipeline | Compiles on a repeat | Repeat call | The compiled work alone |
| --- | --- | --- | --- |
| `plant.compute_trajectory(backend="jax")`, RK4, 5 001 points | 1 (0.35 s) | 465 ms | 36 ms (`ev.rollout`) |
| the same, Euler / a closed loop `lqr @ plant` / a sampled loop | 1 each | 300–480 ms | — |
| `compute_trajectory(solver="scipy", backend="jax")` | 2 | 315 ms | — |
| `pe.evaluate(params)`, `pe.evaluate()`, `jax.grad` of it | 0 | 3.7 ms | 3.3 ms |
| `problem.compile(law, dt).evaluate()`, a fresh evaluator | 1 (0.58 s) | 800 ms | — |
| `planner.solve()`, collocation + SLSQP | 6 (1.06 s) | 1.41 s | 0.11 s (`pv.solve`) |
| `planner.solve()`, native single shooting + Adam | 7 (1.48 s) | 2.07 s | — |
| `pv.solve(P)`, native Adam, called from Python | 1 (0.52 s) | 0.64 s | 25 ms under `jit` |
| the native MPC's closed loop, `pe.evaluate` | 0 | — | — |
| `(mpc % dt) @ plant).compute_trajectory` | 2 (1.51 s) | 1.97 s | — |
| value iteration `planner.solve()` (JAX), RL training | 0 | — | — |
| RL `solve_trajectory_from` | 1 | 374 ms | — |

Frozen against parameters as arguments, the compiled work alone: the rollout 72 against 76 ms,
one evaluation 2.7 against 3.3 ms, the native MPC's loop 117 against 128 ms, a native solve 25
against 25 ms; the program's evaluator call 1.51× (DESIGN §6, the bicycle MPC). On `main`
(v0.2) `compute_trajectory` and `planner.solve()` recompile the same way (395 and 1 180 ms a
repeat): not a regression, the cost of never reusing a compiled object.

## 2. The contract

- **C1, two forms, one entry.** Every compiled computation has a frozen form (`params=None`:
  the values compiled in are constants of the trace, so XLA folds them, the code a hand-written
  `jit` with literals would give) and a parametric form (a tree: traced arguments, one trace
  for every value of the same tree structure, shapes and dtypes). Each is traced once per
  holder and cached; neither retraces on a repeat, and the parametric one never on a new value.
- **C2, one dispatch per call.** `jit` sits at the outermost pure computation of a user call: the
  rollout (one `scan`), the evaluation (a `vmap` of the scan), a fixed-count solve (one `scan`),
  each callback of a host solver (`J`, `∇J`, `h`, `∂h/∂z`, … one jitted function each, the
  solver's loop the only Python). No `jax.*` primitive runs eagerly on a fresh closure.
- **C3, compiled objects are reused.** A facade (`compute_trajectory`, `compute_rollout`, the
  simulator, `planner.solve`, the RL rollouts) looks its compiled object up instead of building
  it: keyed by what compile reads, rebuilt when that changes. Correctness first: a changed
  number or wiring always gives the new result.
- **C4, what compile reads.** A system's structure (its class, its ports' sizes, nominal values
  and dependencies; a diagram's blocks and wiring; a clocked block's period) and its `params`'
  values. By the constitution's "every number is a parameter" and its build-versus-run law, any
  other attribute a block's equations read is structure, set before the first compile;
  `clear_compiled(system)` drops a system's compiled objects for the rare script that edits
  one afterwards.

## 3. The design, layer by layer

### 3.1 The evaluator cache (`core/compile/cache.py`)

- `CompiledCache`, an attribute of every `System` set in `System.__init__`: a small LRU (4
  entries) of evaluators by key. It copies and pickles empty (`__deepcopy__`, `__copy__`,
  `__getstate__`), so a copied block never carries another's evaluators, and it lives on the
  system, so the cycle system → evaluator → system is collectable.
- The key: `(backend, flatten, fingerprint(system))`. `fingerprint` is a tuple built
  recursively: for a leaf its class, `n`, its ports (id, size, nominal value bytes, an output's
  dependencies), its clock (a `SampledSystem`'s period, substeps and solver) and a digest of its
  `params` (blake2b over each leaf's path, dtype, shape and bytes); for a diagram the same of
  its edge ports, its connections and its blocks by id (the block's identity and fingerprint).
  A hit costs the fingerprint (measured target: under 0.2 ms for CartPole, under 2 ms for the
  UR5 loop).
- `compile(system, backend, verbose, flatten)` (the one compile entry, `compiler.py`) consults
  it; every caller of `sys.compile` and of `compile` then reuses: `compute_trajectory`,
  `compute_rollout`, the simulator, the sampled loops, the RL rollouts, `problem.compile`'s loop
  compile, `linearize`.

### 3.2 The evaluator methods

`@compiled` already gives C1: `jax.jit(bound)` with `params` an argument; `None` is a tree with
no leaves, so the body reads `self._frozen_params` and the trace takes them as constants; a
tree is traced. Two cache entries, one per form. Kept; documented in DESIGN §5 and pinned by
tests (a repeat: 0 compiles; a new value of the tree: 0; frozen and parametric equal to 1e-12).

### 3.3 The problem evaluator (`planning/evaluator.py`)

- `evaluate()` and `trajectory()` with `params=None` take a frozen form: the trial function
  jitted with the run's default tree closed over (constants), the drawn leaves (the trial axis
  of a stochastic problem) still arguments. Cached by the layout and a digest of the defaults
  (`pe.params` is read fresh from the objects, so a changed block or cost recompiles).
- A tree given keeps today's parametric form (one trace per layout).

### 3.4 The program evaluator (`optimization/evaluators/jax_evaluator.py`)

- The functions of `(z, params)` keep their parametric jitted forms; each gains a frozen form,
  `jit(lambda z: f_raw(z, P))` over the program's own `params`, built on first use. `J(z)`,
  `grad_J(z)`, `h(z)`, `jac_h(z)`, … with no params use it (the offline solve); the parametric
  path (`Optimizer.solve(params=)`, the compiled face) passes the tree.

### 3.5 The fixed-count solve (`optimization/optimizers/first_order.py`)

`run(program_evaluator, z0, params, iterations)` jits the whole count (init, `scan`, the final
`J`) once per `(program evaluator, K)`, with `z0` and `params` arguments, cached on the backend;
the frozen form when `params` is the program's own. A Python caller of `pv.solve` then pays one
dispatch, as under an outer `jit`.

### 3.6 `planner.solve()` on trajectory optimization

The transcribed program and its optimizer are cached on the planner, keyed by the problem's
fingerprint (its parts' identities and a digest of `problem.params`) and the transcription's
options; a repeat runs the host solver only (`z0` from the guess is data). `params=` on solve
gives a new problem copy, so a new key: a sweep of values belongs on `planner.compile()`, the
parametric face (one trace for every value), as DESIGN already says.

### 3.7 Out of scope here

- A persistent XLA cache across processes (`jax_compilation_cache_dir`): measured 370 against
  850 ms for a script's first simulation; it writes to the user's disk, so it is an opt-in
  knob for later (`configure_jax(cache_dir=)`), not part of this change.
- `problem.compile(...)` building a new `ProblemEvaluator` each call: its loop's evaluator is
  reused (3.1); its own trial function is per evaluator, as any held object (hold the `pe`).

## 4. Tests (`tests/unittest/test_compile_once.py`)

A compile counter (the `/jax/core/compile/backend_compile_duration` monitoring event). Per
pipeline of §1: a repeat costs 0 compiles; a parametric call with new values of the same
shapes costs 0; a frozen call after a changed parameter recompiles and gives the new number;
a rewired diagram gets a new evaluator; a deep copy's cache is empty; the frozen and parametric
forms agree to 1e-12; the NumPy paths are byte-identical to today (the v03 baseline).

## 5. Benchmarks

- `benchmarks/compile_once.py`: first call, repeat, frozen and parametric per pipeline, and the
  compile counts, to JSON.
- The sanity check against `main`: the classical demos' core calls (a plant's and a closed
  loop's simulation on NumPy and JAX, a trajectory optimization, value iteration, a sampled
  PID), first call and repeat, in fresh processes on `main` and on the branch.

## 6. Risks

- An attribute outside `params` edited after a compile is not seen (C4): the same holds today
  for any held evaluator; the cache widens it to the facades. Mitigation: `clear_compiled`, the
  doc line, and the fingerprint's nominal values and clocks.
- Memory: four evaluators per system at most.
- The fingerprint's cost on a large diagram, and the digest of `pe.params` per frozen call:
  measured in the benchmark, under the budgets above.
