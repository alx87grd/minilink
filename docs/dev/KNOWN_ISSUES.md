# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the design question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-11-05.

## Composition and control

### `ComputedTorque` cannot drive an underactuated plant
- **Symptom:** `ComputedTorque(CartPole()) @ CartPole()` fails at wiring: "Port dimension
  mismatch: controller:u has dim 2, plant:u has dim 1". The law outputs one torque per
  coordinate, which fits only when `B = I`; the boat and the 3-D plane (body-frame velocities)
  are refused as not mechanical.
- **Workaround:** a fully actuated plant, or the allocation `u = B⁺ τ` in a custom block.
- **Fix:** the one-mechanical-family work (S32, postponed): a model-based law will refuse an
  underactuated plant at construction, with a clear message.

### A misspelled key in a block's `params` is ignored
- **Symptom:** `plant.params["mass_typo"] = 3.0` is accepted; the block keeps reading its own
  keys, so the change has no effect and nothing says so.
- **Workaround:** a checked spelling: every tree a tool or a composite takes is checked
  against the structure it enters, so `problem.params = p`, `planner.solve(params=p)` or
  `sim.solve(params=p)`, with `p` a copy of the tool's own tree edited, raises on a misspelled
  key and names the known ones. Or read the key back (`plant.params["m"]`) after a change.
- **Fix:** none: the bare item write on a leaf stays a plain dict write, by the maintainer's
  ruling of 2026-11-05 (DESIGN §4, "Parameters", rule 5).

## Planning and learning

### A value-iteration law cannot run in a JAX closed loop
- **Symptom:** `(solution.policy @ plant).compute_trajectory(..., backend="jax")` with a
  value-iteration solution's `LookupPolicy` raises "Block ... is not JAX-traceable": the law
  indexes its NumPy command table with a traced cell. `backend="auto"` (the default of
  `evaluate` and `compare(problem, ...)`) falls back to NumPy, which is the faster backend at
  the grid sizes the law serves.
- **Workaround:** run the loop on NumPy (the default); the value-iteration solve itself runs on
  JAX with `DynamicProgrammingPlanner(..., backend="jax")`.
- **Fix:** none planned yet: taking the table into the point's array module makes the law trace
  (one line), but `backend="auto"` would then move it to JAX; how `"auto"` should choose for it
  waits for the maintainer ([plans-as-policies.md](../plans/plans-as-policies.md) §13).

### Direct collocation fails on a ball-shaped goal set
- **Symptom:** `TrajectoryOptimizationPlanner(..., transcription="direct_collocation")` on a
  problem whose `Xf` is a `BallSet` stops with SLSQP's "Inequality constraints
  incompatible"; a point `Xf` solves. A failed solve still returns its initial guess as an
  open-loop policy, and `compare` scores it like a solution.
- **Workaround:** `problem.replace(Xf=SingletonSet(x_goal))` for the trajectory optimizer;
  check `solution.success` before comparing.
- **Fix:** none planned yet.

## Tooling

### The RL-to-Bode showcase notebook runs close to the notebook checker's time limit
- **Symptom:** `showcase_from_rl_to_bode` takes about 8 minutes on a cloud machine (465 s
  alone, the same before and after the 2026-10-24 restructure); run beside the test suite it
  hits the checker's 300 s limit.
- **Workaround:** run the notebook checks alone, or with `--timeout 1000`.
- **Fix:** none planned; raise its timeout in the notebook checker if CI starts to time out.
