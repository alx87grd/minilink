# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the design question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-11-18 (NumPy 2.5, JAX 0.12, a cloud machine).

## Composition and control

### `ComputedTorque` cannot drive an underactuated plant
- **Symptom:** `ComputedTorque(CartPole()) @ CartPole()` fails at wiring: "Port dimension
  mismatch: controller:u has dim 2, plant:u has dim 1". The law outputs one torque per
  coordinate, which fits only when `B = I`; the boat and the 3-D plane (body-frame velocities)
  are refused as not mechanical.
- **Workaround:** a fully actuated plant, or the allocation `u = B⁺ τ` in a custom block.
- **Fix:** the one-mechanical-family work (TODO §2, S32), postponed until the maintainer
  reviews the mechanical hierarchy: a model-based law then refuses an underactuated plant at
  construction, with a clear message.

### A misspelled key in a block's `params` is ignored
- **Symptom:** `plant.params["mass_typo"] = 3.0` is accepted; the block keeps reading its own
  keys, so the change has no effect and nothing says so.
- **Workaround:** a checked spelling: every tree a tool or a composite takes is checked
  against the structure it enters, so `problem.params = p`, `planner.solve(params=p)` or
  `sim.solve(params=p)`, with `p` a copy of the tool's own tree edited, raises on a misspelled
  key and names the known ones. Or read the key back (`plant.params["m"]`) after a change.
- **Fix:** none: the bare item write on a leaf stays a plain dict write, by the maintainer's
  ruling of 2026-11-05 (DESIGN §4, "Parameters", rule 5).

## The README's examples

Found by running the README's snippets as written (2026-11-09; run again 2026-11-18, a cloud
machine, JAX 0.12, NumPy backend unless the snippet says otherwise).

### The "One problem, every planner" example shows two planners missing the goal
- **Symptom:** README §7 as written: value iteration ends with `success = no`
  ("max_iterations=1000 reached before tol=0.1 (delta=5.48)") after a warning that an infinite
  undiscounted horizon has no cost bound (a failure costs 1e6), and `compare` scores it
  `J = 88.25 (goal missed)`; RRT, whose record says "goal reached", is scored
  `J = 133.6 (goal missed)`; only trajectory optimization succeeds (`J = 89.82`). Times:
  value iteration 7.4 s, RRT 0.4 s, trajectory optimization 9.2 s.
- **Workaround:** none in the snippet; a discount on the problem (`discount_rate=`) or a
  `failure_cost` gives value iteration a cost bound.
- **Fix:** **[ask]** the README is the maintainer's: the example's numbers chosen so each
  planner reaches the goal, decided with the kept demo set (TODO §1.12).

### The co-design example returns a negative gain and a 3.5 cm pendulum
- **Symptom:** README §5: `Optimizer(pe.program(wrt=["controller.Kp", "plant.l"])).solve()`
  converges to `z* = [-1.162, 0.0349]` (`J* = 5.506`): nothing bounds the gain or the rod
  length, so the optimum is not a design one would build.
- **Workaround:** optimize the gains only, or a cost that prices the parameters.
- **Fix:** **[ask]** with the entry above: bounds on the decision variables, or another pair.

## Planning

### A tree search that never leaves its root cannot be scored
- **Symptom:** `RRTStarPlanner(problem, seed=0, max_iterations=300).solve(evaluate=True)` on
  the pendulum swing-up (`Xf = BallSet([π, 0], 0.3)`, horizon 10 s) raises "dt must be
  positive, got nan" after NumPy's "Mean of empty slice" warning: no extension improved on the
  root, so the best-effort path is the root alone, a plan of one sample with no step to read,
  and `Tree.extract_trajectory` builds its input with no channel (`u` of shape `(0, 1)`),
  which the controller's `TrajectorySource` carries; `evaluate(problem, solution.controller)`
  then raises "u_seq must have shape (N, 1), got (222, 0)" (found 2026-11-11; the messages as
  of 2026-11-18).
- **Workaround:** more iterations (`max_iterations=`), or `solve()` without `evaluate` and a
  check of `solution.record.success` first.
- **Fix:** TODO, D3's hardening rows: the root-only path holds the plant's nominal input.

## Messages and small surprises

### A linear model's matrices are methods
- **Symptom:** `plant.linearize().A` is a bound method; the matrix is `plant.linearize().A()`.
  A student who writes `np.linalg.eigvals(lin.A)` gets NumPy's "0-dimensional array given";
  the library's own verbs (`lqr_gain(lin.A, …)`, `controllability_matrix`, the linear
  analysis on arrays) refuse it with the call to write, `sys.A()`.
- **Workaround:** call it: `lin.A()`, `lin.B()`.
- **Fix:** none: the matrices stay methods `A(t, params)` on every linear model (the
  maintainer's ruling 11 of 2026-11-10), and NumPy's own message cannot name the call.

## The compiled evaluators

### A compiled evaluator's `jacobian` reads the system's live numbers, its `f` the compiled ones
- **Symptom:** after `ev = plant.compile()` and `plant.params["m"] = 5.0`, `ev.f(x, u, t)` is
  unchanged (the numbers frozen at the compile) but `ev.jacobian("f", "x", x, u)` changes: with
  `params=None` it reads the system's current numbers. Its docstring says it reads the compiled
  ones. Reproduced on `Pendulum`, NumPy, 2026-11-12.
- **Workaround:** pass `params=ev.params` to `jacobian`, or compile again after changing a number.
- **Fix:** none planned yet; found by the check of the core notebooks.

### `rollout_batch` on a NumPy evaluator fails with an `AttributeError`
- **Symptom:** `plant.compile(backend="numpy").rollout_batch(starts, n_steps=5, dt=0.01)` raises
  `AttributeError: 'NoneType' object has no attribute 'asarray'`, not a message saying the
  batch needs the JAX backend. Reproduced 2026-11-12.
- **Workaround:** `plant.compile(backend="jax")` for a batch.
- **Fix:** none planned yet; a refusal that names the backend to use.

### An evaluation's rates are float32 beside float64 costs
- **Symptom:** on the JAX backend `evaluation.feasibility_rate`, `reachability_rate` and
  `success_rate` are float32 (a mean of booleans), while `mean`, `std` and `worst` are
  float64. Reproduced 2026-11-12.
- **Workaround:** `float(evaluation.feasibility_rate)`.
- **Fix:** none planned yet.

## Tooling

### Three NumPy-against-JAX tests fail in the last digit on NumPy 2.5 with JAX 0.12
- **Symptom:** a normal draw is the same on both backends in its bits, not in its last
  digit: `standard_normal` runs Box–Muller, `√(−2 ln u₁) cos(2π u₂)`, through each
  library's own `log` and `cos`, which round differently on this pair of versions (one unit
  in the last place: a fifth of the `log` values, one `cos` in a thousand, about one draw in
  fifteen). Three tests compare with exact
  equality and fail: `test_random.py::test_bits_and_normals_agree_on_numpy_and_jax`,
  `test_stochastic_policy.py::test_a_policy_vmaps_over_seeds_on_both_backends` and
  `test_blocks.py::TestWhiteNoiseSource::test_the_same_signal_on_numpy_and_jax_and_a_traced_params_family`.
  The cipher's bits are identical; a seeded noise signal or a stochastic policy's draw
  differs between `backend="numpy"` and `backend="jax"` by that last digit, where the
  docstrings say "identical". No version is pinned (`numpy` and `jax>=0.4.28` in
  `pyproject.toml`), so a fresh install or a CI run on these versions meets it. Reproduced
  2026-11-18 (NumPy 2.5.3, JAX 0.12.1, Python 3.13).
- **Workaround:** compare the two backends' draws to a tolerance (`rtol=1e-14`); deselect
  the three tests on such an install.
- **Fix:** the hardening row of TODO §3 ("The NumPy-against-JAX draw tests compare to the
  last digit"): the three tests compare the normals to a tolerance, the bits staying exact,
  and the docstrings say so.
