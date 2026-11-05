# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the design question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-11-04.

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
  ruling of 2026-11-05 ([one-params-model.md](../plans/one-params-model.md), ruling 4).

## Planning and learning

### `ReinforcementLearningPlanner` sizes its default `NeuralPolicy` from the plant's port bounds instead of `problem.U`
- **Symptom:** a user subclass of `MechanicalSystem` has unbounded port bounds (`[-inf, inf]`); declaring finite bounds on the problem (`U=BoxInputSet.from_bounds(...)`) and calling `ReinforcementLearningPlanner(problem)` raises `ValueError: NeuralPolicy needs finite bounds on the plant's action port` because `NeuralPolicy` in `control/neural.py` reads `sys.inputs[command_port(sys)].box` instead of `problem.U`. Conversely, omitting `U` on a catalog plant lets `ReinforcementLearningPlanner` scale actions to the plant's port bounds while `problem.U` is unconstrained, and omitting `X` lets `StateSpaceGrid.state_bounds` in `planning/dynamic_programming/discretizer.py` silently fall back to `sys.state.box` (against the rule that `X` and `U` are unconstrained unless declared).
- **Workaround:** set `plant.inputs["u"].lower_bound` and `upper_bound` on custom plants before calling `ReinforcementLearningPlanner`, and pass `X=plant.state.box, U=plant.input_box` explicitly on `PlanningProblem`.
- **Fix:** V1.2's rest in [TODO.md](../plans/TODO.md) §2 (`ReinforcementLearningPlanner` reads `problem.U.bounding_box()` and refuses an unbounded `problem.U`; `StateSpaceGrid.state_bounds` aligns with `input_bounds`).

### A value-iteration policy overshoots the input bound by rounding
- **Symptom:** a `DynamicProgrammingPlanner` policy (`LookupPolicy`) interpolates grid
  corners that all sit at the torque limit and returns `u = u_max + 1e-15`; scored on a
  problem with `U`, the run reports "failed" with a feasibility margin of about -9e-16.
- **Workaround:** read the margin, not the flag, or score the policy on the problem without
  `U`.
- **Fix:** none planned yet (clip the interpolated command to the grid's input box).

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
