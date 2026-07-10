# Phase 6: MPC `StepBlock` API

**Status: pending** — next hybrid program milestone.

**After [Phase 5](05-hybrid-simulation.md).** Refactor the MPC planning layer so
`MPCPlanner` can export a **`StepSystem`** leaf for hybrid / step simulation — replacing
hand-rolled outer loops in MPC demos.

**Requires:** Phases **1–2** (`StepSystem`, `StepDiagramSystem`), Phase **1b** (flow diagram IS-A
`DynamicSystem`), Phase 5 (`HybridSimulator`).
Phase 4 **`Computer`** on the step side when used inside hybrid sim.

**Files:** `minilink/planning/mpc/step_block.py` (name TBD), `minilink/planning/mpc/planner.py`

## Motivation

Today MPC demos call `MPCPlanner.step(x, initial_guess=...)` inside a Python `while t < tf`
loop with manual `u_hold`, `SUBSTEPS`, and warm-start shifting. Phase 5 delivers
`HybridSimulator`; Phase 6 moves MPC **into** the step-diagram contract so demos become:

```python
hybrid = HybridDiagram(computer=Computer(controller_with_mpc, schedule), plant=plant)
HybridSimulator(hybrid, ...).solve_forced(u)
```

`MPCPlanner` remains the NLP engine; **`MPCStepBlock`** (working name) is the `StepSystem`
adapter the diagram and **Computer** call.

## Split of concerns

| Piece | Role |
| --- | --- |
| **`MPCPlanner`** | Compile-once NLP; `step(x_start, initial_guess=...)` → `Trajectory` |
| **`MPCStepBlock`** | `StepSystem` façade: map diagram `u` / `x` ↔ planner call; expose `u_cmd` on `h` |
| **`HybridSimulator`** | Clock, boundary ZOH/sample, plant — unchanged from Phase 5 |

**Coordinate note:** `MPCStepBlock.step(x, u, k, …)` uses integer **`k`** (Computer fire
index). `MPCPlanner.step(x_meas, …)` is the planning API — different object, different meaning;
no wall time on the block ([Phase 1](01-step-core.md)).

## Milestones

| Milestone | Block state | Warm start | Demo |
| --- | --- | --- | --- |
| **6a** | **`n = 0`** — stateless | `initial_guess=None` every tick (planner default) | Refactor straight-line MPC demo |
| **6b** | **`n > 0`** — packed last plan | Shift previous horizon in `step` | Same demo; match shifted-guess hand loop |

**Gate:** **6a** before **6b**. **6a** before claiming MPC parity in hybrid acceptance tests.

## `MPCStepBlock` contract (6a — stateless)

```python
class MPCStepBlock(StepSystem):
  """
  Stateless MPC controller block for step / hybrid simulation.

  Diagram inputs ``u`` carry measurements and references (e.g. ``y``, ``r``).
  Output ``y`` (via ``h``) is the first control move ``u_cmd`` from the planner.
  """

  def __init__(self, planner: MPCPlanner, *, input_layout: ...):
      super().__init__(n=0, m=..., p=...)
      self._planner = planner
      ...

  def step(self, x, u, k=0, params=None):
      x_meas = extract_measurement(u, ...)
      plan = self._planner.step(x_meas, initial_guess=None)
      self._u_cmd = plan.u[:, 0].copy()
      return np.array([])   # x_new — empty for n=0

  def h(self, x, u, k=0, params=None):
      return self._u_cmd
```

### 6a rules

- **`n = 0`** — no block state vector; warm-start trajectory is **not** retained across ticks.
- **`step` is the sole planner call site** per Computer fire (no duplicate solve in `h`).
- Input unpacking (which entries of `u` are `y`, `r`, etc.) is explicit at construct time or via
  port metadata — not implicit globals.
- Factory helper encouraged:

```python
def mpc_step_block(planner: MPCPlanner, *, y_idx=..., r_idx=...) -> MPCStepBlock: ...
```

### 6a acceptance

- Straight-line MPC demo drops the outer `while` / `SUBSTEPS` loop; uses `HybridSimulator`.
- Closed-loop trajectory matches **stateless** hand-rolled loop (default guess each tick) within tolerance.
- One demo refactor only; preserve user tuning constants in the script.

## `MPCStepBlock` contract (6b — warm-start state)

Extend the same class (or `MPCStepBlockWarmStart` subclass — prefer **one class**, `n` toggled by
option) so block state holds the **previous plan** for horizon shifting.

```python
def step(self, x, u, k=0, params=None):
    x_meas = extract_measurement(u, ...)
    guess = shift_plan_state(x) if x.size else None
    plan = self._planner.step(x_meas, initial_guess=guess)
    return pack_plan_state(plan)   # x_new for next tick
```

### State encoding (6b) — use `z`, not a core Trajectory flatten

**Do not** add a `Trajectory` → flat vector helper in `minilink/core/`. Flatten layout is
**transcription-specific** (direct collocation packs `[x.ravel(); u.ravel()]`, shooting packs
`u.ravel()` only) and already lives on the transcription:

```191:224:minilink/planning/trajectory_optimization/direct_collocation.py
    def pack(
        self, x: np.ndarray, u: np.ndarray, problem: PlanningProblem
    ) -> np.ndarray:
        """Pack sampled state and input matrices into one decision vector."""
        return np.concatenate((x.reshape(-1), u.reshape(-1)))
    ...
    def pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        ...
        if isinstance(guess, Trajectory):
            resampled = guess.resample(t_new=self.options.t)
            return self.pack(resampled.x, resampled.u, problem)

        return guess.reshape(-1)
```

**`MPCStepBlock` warm-start state = previous optimizer decision `z`**, not a hand-rolled
trajectory flatten:

| Representation | Role |
| --- | --- |
| **`z`** (`result.z` from last solve) | **Block state `x`** — fixed `n = transcription.decision_dimension(problem)` |
| **`Trajectory`** | User / demo API; `MPCPlanner.step(..., initial_guess=traj)`; shift logic in trajectory space |
| **`pack` / `unpack` / `pack_initial_guess`** | Transcription bridge — planning layer only |

```python
n_z = planner.transcription.decision_dimension(planner.problem)
# block constructed with n=n_z when warm_start=True

def step(self, x, u, k=0, params=None):
    x_meas = extract_measurement(u, ...)
    z_guess = shift_mpc_initial_guess(
        x, x_meas, planner.transcription, planner.problem, dt_shift=planner_dt
    ) if x.size else None
    plan = self._planner.step(x_meas, initial_guess=z_guess)
    return np.asarray(self._planner.last_optimization_result.z).copy()
```

**Warm-start shift** (today's demo logic in `Trajectory` space) becomes a **planning helper**
(e.g. `minilink/planning/mpc/warm_start.py`):

1. `unpack(z_prev) → (x, u)` on the fixed collocation grid.
2. Shift / truncate horizon; pin `x[:, 0] = x_meas` (same semantics as straight-line demo).
3. `pack(x, u)` **or** build a `Trajectory` and call `pack_initial_guess` (resamples to grid).

Either path ends in `z` before the solver; prefer **matrix shift + `pack`** to avoid an extra
`Trajectory` alloc in the hot loop. **`Trajectory` stays the logging / plotting / user-facing
type**; it is not the canonical simulation state vector.

**6a:** `n = 0`, no `z` retained. **6b:** `n = decision_dimension`, state is last `z`.

### 6b acceptance

- Same hybrid demo matches **warm-started** hand-rolled loop (shifted `prev_plan` guess) within tolerance.
- Stateless mode (`n=0` or `warm_start=False`) still works for tests and teaching.

## `MPCPlanner` refactor scope

Minimal surface changes — planner API stays:

```python
plan = planner.step(x_start, initial_guess=guess)
```

Phase 6 work:

1. **`mpc_step_block(planner, ...)`** factory in `planning/mpc/`.
2. Document input/output port layout for bicycle (and one generic template).
3. **`shift_mpc_initial_guess(z_prev, x_meas, transcription, problem, dt_shift)`** in
   `planning/mpc/` — factors demo shift logic; **not** a core `Trajectory` flatten tool.
4. **Do not** move compile-once / `prepare()` into the sim loop — block holds prepared planner from `__init__`.

## Hybrid wiring (6a)

```python
mpc = mpc_step_block(mpc_planner, ...)
controller = StepDiagramSystem()
controller.add_subsystem(mpc, "mpc")
controller.add_input_port("y")
controller.connect_new_output_port("mpc", "u", "u")

schedule = StepSchedule(dt_base=MPC_DT)
hybrid = HybridDiagram(computer=Computer(controller, schedule), plant=plant)
hybrid.connect_boundary(direction="computer_to_plant", computer_port="u", plant_port="u")
hybrid.connect_boundary(direction="plant_to_computer", computer_port="y", plant_port="y")
HybridSimulator(hybrid, plant_dt_inner=SIM_DT, ...).solve_forced(u)
```

## Tests

| File | Cases |
| --- | --- |
| `test_mpc_step_block.py` | 6a stateless `step`/`h`; port unpack; planner called once per tick |
| `test_mpc_step_block_warm_start.py` | 6b state pack/shift; guess matches demo shift |
| `test_mpc_hybrid_demo_parity.py` | hybrid vs hand-rolled straight-line (6a and 6b) |

## Deferred

- JAX-differentiable MPC inside `step` (planner stays NumPy for v1).
- Multi-rate MPC inside one diagram (use Phase 5b `fire` on `mpc` sys_id).
- Exposing full `Trajectory` on diagram boundary every tick (logging via simulator traj only).
