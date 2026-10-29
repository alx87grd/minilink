# Compile once, in the form the call needs

Status: landed 2026-10-28 on `dev-v0.3` (from `dev-v0.3-compile`), after two independent
reviews of the plan (A: JAX and performance; B: architecture and correctness; their findings
and the resolution of each in §7) and one of the implementation (its five defects fixed, §7).
The frozen forms of §3.3 and §3.4 are not built: the benchmark (§5) measured none faster. The maintainer's ask: "frozen parameters, or parameters as arguments for a
parametric computation, each as fast as if hard-coded, behind the one method entry; `jit` at
the right place on every pipeline".
Where it sits: a hardening row of the v0.3 core (TODO §1.10), beside the wiring-time refusals.

## 1. What the audit measured (CartPole / pendulum, CPU, float64, `dev-v0.3` at `1e9d2d6e`)

A second identical call, the XLA compilations it triggers:

| Pipeline | Compiles on a repeat | Repeat call | The compiled work alone |
| --- | --- | --- | --- |
| `plant.compute_trajectory(backend="jax")`, RK4, 5 001 points | 1 (0.35 s) | 465 ms | 36 ms (`ev.rollout`) |
| the same, Euler / a closed loop `lqr @ plant` / a sampled loop | 1 each | 300–480 ms | — |
| `compute_trajectory(solver="scipy", backend="jax")` | 2 (`t` weak, then strong) | 315 ms | — |
| `pe.evaluate(params)`, `pe.evaluate()`, `jax.grad` of it | 0 | 3.7 ms | 3.3 ms |
| `pe.evaluate(p)` with a leaf as `9.0`, `np.float64(9.0)` or `10` | 1 each (4–5 s) | — | — |
| `pe.trajectory()` | 1 (≈ 3 s, eager `scan`) | — | — |
| `problem.compile(law, dt).evaluate()`, a fresh evaluator | 1 (0.58 s) | 800 ms | — |
| `planner.solve()`, collocation + SLSQP | 6 (1.06 s) | 1.41 s | 0.11 s (`pv.solve`) |
| `planner.solve()`, native single shooting + Adam | 7 (1.48 s) | 2.07 s | — |
| `pv.solve(P)`, native Adam, called from Python | 1 (0.52 s, eager `scan`) | 0.64 s | 25 ms under `jit` |
| `(mpc % dt) @ plant).compute_trajectory` | 2 (1.51 s) | 1.97 s | — |
| RL `solve_trajectory_from` | 1 (eager `scan`) | 374 ms | — |
| value iteration `planner.solve()` (JAX), RL training, the native MPC's `pe.evaluate` | 0 | — | — |

Frozen against parameters as arguments, the compiled work alone: the rollout 72 against 76 ms,
one evaluation 2.7 against 3.3 ms, the native MPC's loop 117 against 128 ms, a native solve 25
against 25 ms. A tree of 36 small NumPy leaves costs 166 µs a call to transfer, 34 µs already
on the device, 7 µs frozen (review A). On `main` (v0.2) `compute_trajectory` and
`planner.solve()` recompile the same way: not a regression, the cost of never reusing a
compiled object.

## 2. The contract

- **C1, two forms, one entry, at the evaluator.** An evaluator method's `params=None` is the
  frozen form: the compiled values are constants of the trace (XLA folds them, the code a
  hand-written `jit` with literals gives), unless a leaf is large (over 1 000 elements: network
  weights), where the frozen tree is instead an argument already on the device. A tree given is
  the parametric form: traced arguments, one trace for every value.
- **C2, one trace for every value.** Every parametric boundary canonicalizes what it is given:
  each leaf of a tree to the default's dtype (a strong type) and shape, a Python scalar
  argument (`t`, `dt`, `t_start`) to `np.float64`. A Python `9.0`, `np.float64(9.0)`, `9` or a
  0-d array then hit the same trace.
- **C3, one dispatch per call.** `jit` sits at the outermost pure computation of a call: the
  rollout, the evaluation and its recorded trajectory, a fixed-count solve, each callback of a
  host solver, one fused step of a first-order solver. No `jax` primitive (`scan`, `vmap`)
  runs eagerly on a fresh closure; no `jit` is built per call.
- **C4, compiled objects reused at the convenience calls, JAX only.** `compute_trajectory`,
  `compute_rollout`, the simulator, the RL rollouts, a problem evaluator's loop and
  `planner.solve()` look their compiled object up. `compile()` stays the build verb (a fresh
  evaluator, as DESIGN states); NumPy evaluators are not cached (a NumPy compile costs 0.1 ms,
  and a NumPy evaluator reads its blocks' attributes live).
- **C5, what the key is.** Structure by Python identity, never parameter values: a changed value
  takes the parametric form with the live tree (one compile for a whole sweep), an unchanged one
  the frozen form. The structure: each block's identity and class, its ports (size, nominal
  value, dependencies, the compute function's identity), instance overrides of its equations,
  a digest of its plain numeric and string attributes outside `params` and `x0` (the values an
  equation reads at trace time: `Sum.signs`, `MLP.activation`, `TrajectorySource.sample_times`,
  the `PID` flags, `NoiseSource.law`), a diagram's edge ports and wiring, and the x64 flag.

## 3. The design, layer by layer

### 3.1 The evaluator methods (`core/compile/evaluators/compiled.py`, `base.py`)

- `CompiledMethod.__get__` returns a small wrapper over the two jitted forms: it canonicalizes
  the scalar arguments (C2), and for `params` routes `None` to the frozen form (or, for a large
  tree, to the parametric form with the frozen tree put on the device once) and a tree to the
  parametric form after canonicalizing it against `evaluator.params`.
- `Evaluator.live_params()` reads the tree from the system now (the same reader as the
  constructor); `Evaluator.call_params()` is `None` when its digest equals the frozen one, else
  the live tree: the one rule the facades use.

### 3.2 The reuse (`core/compile/cache.py`)

- `compiled_of(system, backend, *, flatten=False)`: a module-level LRU (16 entries) of JAX
  evaluators keyed by each block's identity and edit count (`System.edits`, bumped by every
  attribute assignment that is not a value), its ports, the wiring, the numbers it holds by
  their bytes, a held model and a plain-data helper by value, and the x64 flag (the night run,
  N-2, which replaced the attribute fingerprint of C5); NumPy calls `compile`. Block identities
  in the key are safe because the cached evaluator keeps its blocks alive, so an `id()` cannot
  be reused while its entry exists; a loop rebuilt over the same blocks (`problem.compile`, an
  inline `ctl @ plant` held across calls) hits. `clear_compiled()` empties it, for a script that
  edits a block's structure after a convenience call.
- Callers: the simulator and its solvers (which pass `call_params()` through), the step
  systems' `compute_rollout` / `compute_trajectory`, `compile_loop` of the problem evaluator, the
  RL rollouts.

### 3.3 The problem evaluator (`planning/evaluator.py`)

- No second executable (reviews A5, B3): `params=None` runs the parametric trial function with
  the default tree put on the device once, cached by its digest (one entry; a changed default
  replaces it); a tree given is canonicalized in `merged` (C2).
- `trajectory()` runs under a cached jitted trial (record on, one trial), as `evaluate()` does,
  and the recorded signals (`signals_along`, `signals_at_ticks`) inside it.
- A frozen form at this layer is built only if the benchmark (§5) shows a steady gain above
  15 % over the device-resident parametric form. Measured: the parametric `pe.evaluate()` 3.1 ms,
  a frozen `jit(lambda: pe.evaluate(P))` 4.4 ms; not built.

### 3.4 The program evaluator (`optimization/evaluators/jax_evaluator.py`)

- The parametric jitted functions of `(z, params)` stay the one form; `bind(params)`
  canonicalizes the tree and puts it on the device once (keyed by its digest), so a host solver's
  thousands of callbacks transfer only `z`. The frozen question is the same as §3.3, decided by
  the benchmark (review B4: a frozen form must not survive a `bind`). Measured: a native
  200-step solve 24.4 ms parametric against 24.7 ms frozen; not built.

### 3.5 The first-order solves (`optimization/optimizers/first_order.py`)

- `run` (the fixed count): one jitted function of `(z0, params)` per `(K, algorithm, options,
  box)`, stored in the program evaluator's `compiled_forms` (review B11: not on the optimizer).
- `solve` (to tolerance): one jitted fused step `z ← S(z, ∇J(z; p))`, cached the same way, so a
  solve is one dispatch per iteration and no per-solve `jit`.

### 3.6 `planner.solve()` on trajectory optimization (`planning/trajectory_optimization/planner.py`)

- The transcribed program and its optimizer are cached on the planner (one entry), keyed by
  `fingerprint(problem.sys)`, the problem parts' identities, `tf` and the horizon kind, a digest
  of the program's parameter tree (laws by identity), and a snapshot of the planner's and the
  transcription's options. A repeat runs the host solver from the new `z0`.
- `solve(params=...)` keeps today's path (a problem copy, no caching: review B5); a sweep of
  values belongs on `planner.compile()`.

### 3.7 The other eager sites

- RL `rollout_law`: one jitted rollout per `(n_steps, key given)` on the planner.
- The Lyapunov helpers that re-jit per call (`analysis/lyapunov.py`): jitted once per
  certificate.

### 3.8 Out of scope

- A persistent XLA cache across processes (`jax_compilation_cache_dir`): a complement (a repeat
  206 → 85 ms in-process, a script's first simulation 850 → 370 ms) that writes to disk; an
  opt-in knob for later.
- A fresh `ProblemEvaluator` per `problem.compile` traces its own trial function again (its
  loop evaluator is reused, §3.2): hold the `pe`.
- A value sweep through `compute_trajectory` is one compile for the whole sweep (the
  parametric form), not zero; `ev.rollout(params=)`, `rollout_batch` and `planner.compile()` are
  the sweep tools.

## 4. Tests (`tests/unittest/test_compile_once.py`)

A compile counter (the `/jax/core/compile/backend_compile_duration` event, the persistent cache
off). For every row of §1: a repeat costs 0 compiles. Then: a parametric call with a leaf as
`9.0`, `np.float64(9.0)`, `9` and a 0-d `jnp` array costs 0 after the first; a parameter edit
followed by `compute_trajectory` or `pe.evaluate()` gives the new number at most one compile
for any number of values; a structural attribute edit (`Sum.signs`) followed by
`clear_compiled()` gives the new number; a rebuilt loop over the same blocks hits; a `%` copy
and a deep copy do not share an evaluator with their source; `problem.tf` edited before
`planner.solve()` gives the new grid; `bind(P)` then `J(z)` reads `P`; frozen and parametric
agree to 1e-12 (not bit for bit: the fusion order differs); NumPy paths byte-identical (the v03
baseline).

## 5. Benchmarks

- `benchmarks/compile_once.py`: for every row of §1, the first call, a repeat, the parametric
  and frozen forms and the compile counts, to JSON; it decides §3.3 and §3.4's frozen forms.
- The sanity check against `main`: the classical demos' core calls (a plant's and a closed
  loop's simulation on NumPy and JAX, a sampled PID, a trajectory optimization, value
  iteration), first call and repeat, in fresh processes, on `main`, on `dev-v0.3` and on the
  branch, the machine otherwise idle.

## 6. Risks

- A structural attribute edited after a convenience call outside the digested kinds (an object
  attribute such as a lookup object, a module global): not seen until `clear_compiled()`; a
  KNOWN_ISSUES entry, the DESIGN line, the same as any held JAX evaluator today.
- Memory: 16 evaluators, each keeping its blocks alive until evicted.
- The fingerprint's cost per convenience call (target under 0.2 ms for CartPole, 2 ms for the
  UR5 loop; JAX leaves digested by identity, NumPy ones by bytes).

## 7. The reviews and their resolution

| Finding | Resolution |
| --- | --- |
| A1 the parametric form retraces on weak types, int against float, a `(1,)` default given a scalar; SciPy's `t` | C2, §3.1, §3.3, §3.4 |
| A2 `pe.trajectory()` eager, a compile per call | §3.3 |
| A3, B8 a rebuilt loop is a new system: a per-system cache never hits | the module-level cache keyed by block identities, §3.2 |
| A4 per-solve `jit` of the first-order update; RL's eager rollout | §3.5, §3.7 |
| A5, B3 a frozen executable at the problem and program layers costs a second compile, and keyed by value a compile per value | device-resident defaults; frozen only if measured worth it, §3.3, §3.4 |
| A6, B14 large frozen leaves bloat the executable | the 1 000-element threshold, C1 |
| A7 values in the key turn a sweep into rebuilds | C5: structure only, the parametric form for a changed value |
| A8, B7 a trace reads more than `params` | C5's attribute digest and function identities; KNOWN_ISSUES |
| A9 the persistent cache | out of scope, §3.8; the test counter turns it off |
| A10 the frozen evaluator form folds constants as claimed; frozen and parametric differ in the last bits | 1e-12, §4 |
| B1 do not cache NumPy evaluators | C4 |
| B2 `compile()` stays the build verb | C4, `compiled_of` |
| B4 a frozen program form conflicts with `bind` | §3.4 |
| B5 the transcription reads `tf`, the plant's structure, the options and the laws | §3.6's key; no caching of `params=` |
| B6 a shallow `%` copy shares an attribute cache | the module-level cache by identity |
| B9 identity, not `System.id` | C5, §3.2 |
| B10 digest JAX leaves by identity | §6 |
| B11 the first-order forms on the program evaluator, keyed by the options | §3.5 |
| B12 every `System` reaches `System.__init__` | (no change needed with a module-level cache) |
| B13 one rule in one place | `call_params()`, §3.1 |
| B15 docs and tests | §4; DESIGN §4–§6, the `compile_diagram` docstring, the definition of done, KNOWN_ISSUES |

### The implementation's review (2026-10-28)

| Finding | Resolution |
| --- | --- |
| A repeated `jax.grad` or `vmap` through a compiled method met a dead tracer from the device cache | a tree holding a tracer is prepared and never kept (`DeviceTree`) |
| A freed JAX leaf's identity reused by a new array returned the old values | the kept entry holds its leaves, so their identities stay theirs |
| `2.5` given for an integer default was truncated to `2` | a value its default's dtype cannot hold promotes (one more trace) |
| A model a block holds (a computed-torque law's plant) kept its old numbers | a held system is keyed by its identity, its edit count and its parameters' digest (N-2) |
| A parameter edit crashed a diagram holding a sink | the compiled tree names every block |
| Two blocks naming each other made the fingerprint recurse forever; a large attribute was copied into the key | a block seen again ends the walk; above 512 bytes an attribute is keyed by its digest |
| The first solve traced the objective again inside its timer (the gates' trajopt rows) | the program's defaults are canonical from construction |

