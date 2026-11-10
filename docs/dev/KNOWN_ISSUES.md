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

### A tree-search plan replayed by its own controller drifts from the plan
- **Symptom:** `RRTPlanner(problem, seed=0).solve(evaluate=True)` on the pendulum swing-up
  (`Xf = BallSet([π, 0], 0.3)`): the record says "goal reached", the evaluation "goal missed".
  The plan ends at `[3.31, −0.15]`; the handed-out controller, a `TrajectorySource` holding
  each edge's input, replayed on the plan's own `dt = 0.05`, misses the ball, and still at
  `dt = 0.01`; at `dt = 0.002` it reaches it. The same samples given as a held input,
  `plant.compute_trajectory(u=plan.u, dt=0.05, solver="rk4")`, reproduce the plan
  exactly. The source is read at each RK4 stage like any block, so the last stage of a step
  reads the next edge's input (first order at each switch, large for a bang-bang plan).
- **Workaround:** score the plan itself, `evaluate_trajectory(problem, solution.trajectory)`,
  or the controller on a finer step, `evaluate(problem, solution.controller, dt=0.002)`.
- **Fix:** **[ask]** how a held open-loop plan is replayed, in
  [api-stabilization.md](../plans/api-stabilization.md), section 4.

## Messages and small surprises

### A linear model's matrices are methods
- **Symptom:** `plant.linearize().A` is a bound method; the matrix is `plant.linearize().A()`.
  A student who writes `np.linalg.eigvals(lin.A)` gets "0-dimensional array given".
- **Workaround:** call it: `lin.A()`, `lin.B()`.
- **Fix:** **[ask]** a public-API choice, not a defect by itself (the matrices are functions
  of `params`): keep the call, or read `A` as an attribute at the block's own `params`.

## Tooling

### Two racecar demos hang when run headless
- **Symptom:** `examples/demos/udes_racecar/pure_pursuit_racecar.py` and `los_racecar.py`
  compute their lap, print the meshcat viewer's URL and then wait: with no browser open on the
  viewer, the script never ends (both still running after 110 s on a cloud machine, 2026-11-08).
- **Workaround:** run them interactively, with a browser open on the URL they print.
- **Fix:** the maintainer's end-of-v0.4 demo rebuild (TODO §1.12, the demo set).

### The RL-to-Bode showcase notebook runs close to the notebook checker's time limit
- **Symptom:** `showcase_from_rl_to_bode` takes about 8 minutes on a cloud machine (465 s
  alone, the same before and after the 2026-10-24 restructure); run beside the test suite it
  hits the checker's 300 s limit.
- **Workaround:** run the notebook checks alone, or with `--timeout 1000`.
- **Fix:** none planned; raise its timeout in the notebook checker if CI starts to time out.
