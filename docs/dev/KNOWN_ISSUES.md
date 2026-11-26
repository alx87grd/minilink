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

## Planning

### A tree search that never leaves its root cannot be scored
- **Symptom:** `RRTStarPlanner(problem, seed=0, max_iterations=300).solve(evaluate=True)` on
  the pendulum swing-up (`Xf = BallSet([π, 0], 0.3)`, horizon 10 s) raises "dt must be
  positive, got nan" after NumPy's "Mean of empty slice" warning: no extension improved on the
  root, so the best-effort path is the root alone, a plan of one sample with no step to read,
  and `Tree.extract_trajectory` builds its input with no channel (`u` of shape `(0, 1)`),
  which the controller's `TrajectorySource` carries; `problem.evaluate(solution.controller)`
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

### An evaluation's rates are float32 beside float64 costs
- **Symptom:** on the JAX backend `evaluation.feasibility_rate`, `reachability_rate` and
  `success_rate` are float32 (a mean of booleans), while `mean`, `std` and `worst` are
  float64. Reproduced 2026-11-12.
- **Workaround:** `float(evaluation.feasibility_rate)`.
- **Fix:** none planned yet.

## Tooling

### The basin of a region of attraction takes minutes without JAX
- **Symptom:** `plot_region_of_attraction(sys, region, basin=True)` simulates one run per
  point of a 201 × 201 plane. With JAX the runs are one batched, compiled call; on NumPy
  they run state by state, about ten minutes for the saturated pendulum loop (a cloud
  machine, 2026-11-25), with nothing printed meanwhile.
- **Workaround:** install JAX (`pip install "minilink[full]"`), or draw without `basin=True`.
- **Fix:** the hardening row of TODO §3 ("The basin plot on NumPy"): a coarser plane on
  NumPy, or one vectorized NumPy rollout over the plane.
