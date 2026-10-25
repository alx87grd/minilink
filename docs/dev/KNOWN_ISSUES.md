# Known issues

Defects and surprising behaviour a user can hit in the code today, each reproduced. An entry
names the symptom, a minimal reproduction, a workaround, and the workboard row that fixes it
(or the open question it waits on, in [QUESTIONS.md](QUESTIONS.md)). The fix is planned work
in [TODO.md](../plans/TODO.md); this list is the index, and an entry leaves it when its fix
lands. Last checked against the code: 2026-10-24.

## Simulation and evaluation

### The same key draws different trials on NumPy and on JAX
- **Symptom:** `evaluate(..., key=0, backend="numpy")` and `backend="jax"` draw different
  starts and parameters: the key seeds a NumPy generator on one backend and a JAX PRNG key on
  the other (`planning/evaluator.py`, `trial_key`). Each backend is deterministic on its own.
- **Workaround:** compare runs within one backend.
- **Fix:** RN-4 (v0.4), one draw convention on both backends.

### A diagram's `params=None` reads live parameters on NumPy, the first call's on JAX
- **Symptom:** compile a diagram without `bind_params`, change a block's `params`, call
  `ev.f` again: NumPy uses the new values, JAX the values of its first call (its compiled
  form keeps them). The same before and after the 2026-10-24 evaluator merge.
- **Reproduce:** `loop = Gain(2.0, dim=1) >> Pendulum(); ev = loop.compile(backend=…)`; call
  `ev.f(x, u)`, triple `loop.subsystems["Pendulum"].params["m"]`, call it again.
- **Workaround:** recompile after editing parameters, or pass them explicitly
  (`ev.f(x, u, t, params)`), or compile with `bind_params=True` for a snapshot on both.
- **Fix:** waits on a decision (QUESTIONS.md: live parameters or a snapshot).

## Composition and control

### `ComputedTorque` cannot drive an underactuated plant
- **Symptom:** `ComputedTorque(CartPole()) @ CartPole()` fails at wiring: "Port dimension
  mismatch: controller:u has dim 2, plant:u has dim 1". The law outputs one torque per
  coordinate, which fits only when `B = I`; the boat and the 3-D plane (body-frame velocities)
  are refused as not mechanical.
- **Workaround:** a fully actuated plant, or the allocation `u = B⁺ τ` in a custom block.
- **Fix:** the one-mechanical-family work (S32, postponed), whose three open questions are in
  QUESTIONS.md.

### Connecting an already-connected input replaces its source silently
- **Symptom:** a second `diagram.connect(..., "i", "u")` onto the same input rewires it to the
  new source with no warning (the old source is dropped).
- **Workaround:** check `diagram.connections` or set `diagram.connection_verbose = True`.
- **Fix:** S66 (`connect` refuses to rewire a connected input), an ask on the workboard.

### An output that reads an input it does not declare reads a stale value
- **Symptom:** a port whose `compute` reads an input missing from its `dependencies` is
  scheduled as if it did not, so the compiled diagram can evaluate it before its source and
  hand it the empty buffer (zero), silently; in a loop it solves the wrong fixed point.
- **Reproduce:** a static block `y = 2 u` declared with `dependencies=()`, added before a
  `Step(final_value=[1.0])` that drives it: the compiled output at `t = 2` is `0`, not `2`
  (declared `dependencies=("u",)`, it is `2`).
- **Workaround:** declare every input a port reads (`dependencies=("u",)` or `"all"`).
- **Fix:** S66 (the compile probe detects it; raise or warn is QUESTIONS.md's).

### A misspelled key in a block's `params` is ignored
- **Symptom:** `plant.params["mass_typo"] = 3.0` is accepted; the block keeps reading its own
  keys, so the change has no effect and nothing says so. Diagram-level params check their
  block ids; a block's own keys are not checked.
- **Workaround:** read the key back (`plant.params["m"]`) after a change.
- **Fix:** a Later idea in TODO §7 (a `validate_params` hook on `System`).

## Tooling

### The RL-to-Bode showcase notebook runs close to the notebook checker's time limit
- **Symptom:** `showcase_from_rl_to_bode` takes about 8 minutes on a cloud machine (465 s
  alone, the same before and after the 2026-10-24 restructure); run beside the test suite it
  hits the checker's 300 s limit.
- **Workaround:** run the notebook checks alone, or with `--timeout 1000`.
- **Fix:** none planned; raise its timeout in the notebook checker if CI starts to time out.
