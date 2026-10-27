# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the design question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-10-25.

## Simulation and evaluation

### The same key draws different trials on NumPy and on JAX
- **Symptom:** `evaluate(..., key=0, backend="numpy")` and `backend="jax"` draw different
  starts and parameters: the key seeds a NumPy generator on one backend and a JAX PRNG key on
  the other (`planning/evaluator.py`, `trial_key`). Each backend is deterministic on its own.
- **Workaround:** compare runs within one backend.
- **Fix:** RN-4 (the v0.3 core's draws, TODO §1.3), one draw convention on both backends.

### `problem.compile` accepts the stateless MPC and re-solves it at the RK4 stages
- **Symptom:** `problem.compile(mpc, dt)` with an `MPCStatelessController` runs, but the MPC is a
  plain `System`: its tick latch keys on `(k, y)`, the measurement changes at every RK4 stage, and
  the NLP is re-solved up to four times per step, a continuous-feedback MPC the user did not
  build. A `StepSystem` controller is refused (`require_continuous`), the stateless MPC is not.
- **Workaround:** evaluate an MPC through its sampled loop, `mpc % dt @ plant`, with
  `compute_trajectory`.
- **Fix:** the refusal row in [TODO.md](../plans/TODO.md) §1.5, then the sampled face.

## Composition and control

### `ComputedTorque` cannot drive an underactuated plant
- **Symptom:** `ComputedTorque(CartPole()) @ CartPole()` fails at wiring: "Port dimension
  mismatch: controller:u has dim 2, plant:u has dim 1". The law outputs one torque per
  coordinate, which fits only when `B = I`; the boat and the 3-D plane (body-frame velocities)
  are refused as not mechanical.
- **Workaround:** a fully actuated plant, or the allocation `u = B⁺ τ` in a custom block.
- **Fix:** the one-mechanical-family work (S32, postponed): a model-based law will refuse an
  underactuated plant at construction, with a clear message (R102).

### Connecting an already-connected input replaces its source silently
- **Symptom:** a second `diagram.connect(..., "i", "u")` onto the same input rewires it to the
  new source with no warning (the old source is dropped).
- **Workaround:** check `diagram.connections` or set `diagram.connection_verbose = True`.
- **Fix:** S66 (`connect` refuses to rewire a connected input), an ask on the workboard (the v0.3
  core's hardening, TODO §1.10).

### An output that reads an input it does not declare reads a stale value
- **Symptom:** a port whose `compute` reads an input missing from its `dependencies` is
  scheduled as if it did not, so the compiled diagram can evaluate it before its source and
  hand it the empty buffer (zero), silently; in a loop it solves the wrong fixed point.
- **Reproduce:** a static block `y = 2 u` declared with `dependencies=()`, added before a
  `Step(final_value=[1.0])` that drives it: the compiled output at `t = 2` is `0`, not `2`
  (declared `dependencies=("u",)`, it is `2`).
- **Workaround:** declare every input a port reads (`dependencies=("u",)` or `"all"`).
- **Fix:** S66 (the compile probe detects it and raises, R116).

### A misspelled key in a block's `params` is ignored
- **Symptom:** `plant.params["mass_typo"] = 3.0` is accepted; the block keeps reading its own
  keys, so the change has no effect and nothing says so. Diagram-level params check their
  block ids; a block's own keys are not checked.
- **Workaround:** read the key back (`plant.params["m"]`) after a change.
- **Fix:** a Later idea in TODO §6 (a `validate_params` hook on `System`).

## Planning and learning

### `ReinforcementLearningPlanner` sizes its default `NeuralPolicyController` from the plant's port bounds instead of `problem.U`
- **Symptom:** a user subclass of `MechanicalSystem` has unbounded port bounds (`[-inf, inf]`, R68); declaring finite bounds on the problem (`U=BoxInputSet.from_bounds(...)`) and calling `ReinforcementLearningPlanner(problem)` raises `ValueError: NeuralPolicyController needs finite bounds on the plant's action port` because `NeuralPolicyController` in `control/neural.py` reads `sys.inputs[command_port(sys)].box` instead of `problem.U`. Conversely, omitting `U` on a catalog plant lets `ReinforcementLearningPlanner` scale actions to the plant's port bounds while `problem.U` is unconstrained, and omitting `X` lets `StateSpaceGrid.state_bounds` in `planning/dynamic_programming/discretizer.py` silently fall back to `sys.state.box` (against R21).
- **Workaround:** set `plant.inputs["u"].lower_bound` and `upper_bound` on custom plants before calling `ReinforcementLearningPlanner`, and pass `X=plant.state.box, U=plant.input_box` explicitly on `PlanningProblem`.
- **Fix:** V1.2's rest in [TODO.md](../plans/TODO.md) §2 (`ReinforcementLearningPlanner` reads `problem.U.bounding_box()` and refuses an unbounded `problem.U`; `StateSpaceGrid.state_bounds` aligns with `input_bounds`).

## Tooling

### The RL-to-Bode showcase notebook runs close to the notebook checker's time limit
- **Symptom:** `showcase_from_rl_to_bode` takes about 8 minutes on a cloud machine (465 s
  alone, the same before and after the 2026-10-24 restructure); run beside the test suite it
  hits the checker's 300 s limit.
- **Workaround:** run the notebook checks alone, or with `--timeout 1000`.
- **Fix:** none planned; raise its timeout in the notebook checker if CI starts to time out.
