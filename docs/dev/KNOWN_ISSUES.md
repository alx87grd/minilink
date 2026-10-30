# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the design question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-10-31.

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
  keys, so the change has no effect and nothing says so. Diagram-level params check their
  block ids; a block's own keys are not checked.
- **Workaround:** read the key back (`plant.params["m"]`) after a change.
- **Fix:** a Later idea in TODO §6 (a `validate_params` hook on `System`).

### `copy.deepcopy` of a plant fails after a JAX simulation
- **Symptom:** `plant.compute_trajectory(backend="jax")` then `copy.deepcopy(plant)` raises
  "cannot pickle 'mappingproxy' object": the run's trajectory kept on the plant (`plant.traj`)
  does not deep-copy. The same on `main`.
- **Workaround:** deep-copy before the run, set `plant.traj = None` first, or copy
  `plant.params` into a new plant.
- **Fix:** none planned yet (TODO §1.10, the core's hardening).

## Planning and learning

### `ReinforcementLearningPlanner` sizes its default `NeuralPolicy` from the plant's port bounds instead of `problem.U`
- **Symptom:** a user subclass of `MechanicalSystem` has unbounded port bounds (`[-inf, inf]`); declaring finite bounds on the problem (`U=BoxInputSet.from_bounds(...)`) and calling `ReinforcementLearningPlanner(problem)` raises `ValueError: NeuralPolicy needs finite bounds on the plant's action port` because `NeuralPolicy` in `control/neural.py` reads `sys.inputs[command_port(sys)].box` instead of `problem.U`. Conversely, omitting `U` on a catalog plant lets `ReinforcementLearningPlanner` scale actions to the plant's port bounds while `problem.U` is unconstrained, and omitting `X` lets `StateSpaceGrid.state_bounds` in `planning/dynamic_programming/discretizer.py` silently fall back to `sys.state.box` (against the rule that `X` and `U` are unconstrained unless declared).
- **Workaround:** set `plant.inputs["u"].lower_bound` and `upper_bound` on custom plants before calling `ReinforcementLearningPlanner`, and pass `X=plant.state.box, U=plant.input_box` explicitly on `PlanningProblem`.
- **Fix:** V1.2's rest in [TODO.md](../plans/TODO.md) §2 (`ReinforcementLearningPlanner` reads `problem.U.bounding_box()` and refuses an unbounded `problem.U`; `StateSpaceGrid.state_bounds` aligns with `input_bounds`).

## Tooling

### The RL-to-Bode showcase notebook runs close to the notebook checker's time limit
- **Symptom:** `showcase_from_rl_to_bode` takes about 8 minutes on a cloud machine (465 s
  alone, the same before and after the 2026-10-24 restructure); run beside the test suite it
  hits the checker's 300 s limit.
- **Workaround:** run the notebook checks alone, or with `--timeout 1000`.
- **Fix:** none planned; raise its timeout in the notebook checker if CI starts to time out.
