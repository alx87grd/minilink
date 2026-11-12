# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the design question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-11-10.

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

Found by running the README's snippets as written (2026-11-09, a 4-core cloud machine, JAX
0.12, NumPy backend unless the snippet says otherwise).

### The "One problem, every planner" example shows two planners missing the goal
- **Symptom:** README §7 as written: value iteration ends with `success = no`
  ("max_iterations=1000 reached before tol=0.1 (delta=5.48)") after a warning that an infinite
  undiscounted horizon has no cost bound (a failure costs 1e6), and `compare` scores it
  `J = 88.25 (goal missed)`; RRT is scored `J = 133.3 (goal missed)`; only trajectory
  optimization succeeds (`J = 89.82`). Times: value iteration 10.1 s, RRT 0.4 s, trajectory
  optimization 10.8 s.
- **Workaround:** none in the snippet; a discount on the problem (`discount_rate=`) or a
  `failure_cost` gives value iteration a cost bound.
- **Fix:** **[ask]** the README is the maintainer's: the example's numbers chosen so each
  planner reaches the goal (TODO, "Rows waiting for the maintainer's go").

### The co-design example returns a negative gain and a 3.5 cm pendulum
- **Symptom:** README §5: `Optimizer(pe.program(wrt=["controller.Kp", "plant.l"])).solve()`
  converges to `z* = [-1.162, 0.0349]` (`J* = 5.506`): nothing bounds the gain or the rod
  length, so the optimum is not a design one would build.
- **Workaround:** optimize the gains only, or a cost that prices the parameters.
- **Fix:** **[ask]** with the entry above: bounds on the decision variables, or another pair.

## Planning

### A tree search that never leaves its root cannot be scored
- **Symptom:** `RRTStarPlanner(problem, seed=0, max_iterations=300).solve(evaluate=True)` on
  the pendulum swing-up (`Xf = BallSet([π, 0], 0.3)`, horizon 10 s) raises "Port dimension
  mismatch: controller:y has dim 0, plant:u has dim 1": no extension improved on the root, so
  the best-effort path is the root alone, and `Tree.extract_trajectory` builds its input with
  no channel (`u` of shape `(0, 1)`), which the controller's `TrajectorySource` carries (found
  2026-11-11).
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

### Two racecar demos hang when run headless
- **Symptom:** `examples/demos/udes_racecar/pure_pursuit_racecar.py` and `los_racecar.py`
  compute their lap, print the meshcat viewer's URL and then wait: with no browser open on the
  viewer, the script never ends (both still running after 110 s on a cloud machine, 2026-11-08).
- **Workaround:** run them interactively, with a browser open on the URL they print.
- **Fix:** the maintainer's end-of-v0.4 demo rebuild (TODO §1.12, the demo set).
