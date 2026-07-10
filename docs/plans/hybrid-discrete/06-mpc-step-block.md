# Phase 6: MPC block for hybrid simulation

**Status: 6a and 6b done** — `MPCController`, `MPCStepBlock`, hybrid demo, warm-start parity.

**After [Phase 5](05-hybrid-simulation.md).** Expose MPC as a step-diagram leaf usable in
`Computer` / `HybridDiagram`, replacing hand-rolled outer loops in MPC demos.

**Requires:** Phases **1–2** (`StepSystem`, `StepDiagramSystem`), Phase **5**
(`HybridSimulator`), Phase **4** (`Computer`).

**Files:** `minilink/planning/mpc/controller.py`, `minilink/planning/mpc/step_block.py`,
`minilink/planning/mpc/warm_start.py`, `minilink/planning/mpc/plan_reconstruct.py`;

## Motivation

Today MPC demos call `MPCPlanner.step(x, initial_guess=...)` inside a Python `while t < tf`
loop with manual `u_hold`, `SUBSTEPS`, and warm-start shifting. Phase 5 delivers
`HybridSimulator`; Phase 6 moves MPC **into** the step-diagram contract:

```python
hybrid = HybridDiagram(computer=Computer(step_diagram, schedule), plant=plant_diagram)
result = hybrid.compute_forced(..., plant_dt_inner=SIM_DT)
```

`MPCPlanner` remains the compile-once NLP engine; new blocks adapt planner output to diagram
ports and `Computer.tick`.

## Split: algebraic (6a) vs step (6b)

| Mode | Base type | State | Mechanism in `Computer.tick` |
| --- | --- | --- | --- |
| **6a stateless** | `System` (`n=0`) | none | **port `compute` only** (same as `SlidingModeController` in hybrid SMC demos) |
| **6b warm-start** | `StepSystem` (`n=n_z`) | packed decision `z` | port ops solve once (latch) → feedforward outs; **`step` commits `z` only** (no second NLP) |

**Invalid assumption:** `MPCStepBlock(StepSystem, n=0)` — `StepSystem.__init__` raises if
`n < 1`. Static blocks use `System(n=0)`; step diagrams compile them as **port ops only**
(`step_compiler.py` — no `step_ops`).

**Why not `step`/`h` for stateless MPC?** `Computer.tick` runs **port ops before step ops**.
SMC parity relies on algebraic `port.compute` returning control for the **current** tick. A
`step`+`h` latch would apply **lagged** `u_ff` (solve at end of tick, `h` reads previous latch).
Stateless MPC must be algebraic.

```mermaid
sequenceDiagram
  participant C as Computer.tick k
  participant P as port_ops mpc
  participant S as step_op mpc 6b only
  participant NLP as planner.step once

  C->>P: local_x = z_k-1 or empty
  P->>NLP: first port triggers _solve_for_tick
  NLP-->>P: latch plan z
  P->>P: u_ff x_ff z read latch
  C->>S: step returns latch.z
  Note over C: outs = port values; computer.x = z_k
```

```mermaid
flowchart TB
  subgraph solve [Single planner.step per tick k]
    S[MPCPlanner.step y]
    S --> Plan[Trajectory plan]
    Plan --> Uff["u_ff = plan.u[:,0]"]
    Plan --> Xff["x_ff = plan.x[:,1]"]
    Plan --> Zfull["z = packed decision"]
  end
  subgraph ports [Three outputs same latch]
    U[out u_ff]
    X[out x_ff]
    Z[out z]
  end
  solve --> ports
```

## Order of operations (Computer contract)

**`Computer.tick` always runs port ops, then step ops**, per subsystem (`computer.py`). We do
**not** change that order.

### Per-tick timeline (one MPC block firing at tick `k`)

| Phase | What runs | MPC block sees | MPC block produces |
| --- | --- | --- | --- |
| 1. Port ops | Each output port `compute` | `local_x` = **pre-step** state (`z_{k-1}` for 6b; empty for 6a); `local_u` includes `y_k` | Writes `u_ff`, `x_ff`, `z` to signal buffer |
| 2. Step op | `step` (6b only) | Same `local_x` = `z_{k-1}` | Returns `z_k` → `computer.x` |
| 3. Commit | `outs` = boundary slices of signal buffer; `k += 1` | — | Plant ZOH uses `outs["u_ff"]` |

Feedforward outputs at tick `k` come from the NLP solved **at** tick `k` using measurement
`y_k`. That matches hand-loop demos (`solve` then `u_hold = plan.u[:, 0]`). Port-ops-before-step
does **not** lag `u_ff` as long as the solve happens in port ops (not in `h` after a previous
tick's step).

### Why not solve in `step` only?

- Port ops run **before** `step` → `h` would expose **previous** tick's plan (one MPC period late).
- Solving in `step` after port ops → `outs` already committed without current `u_ff`.

**Sole NLP call site: port ops**, via a shared latch. `step` (6b) only **commits** latched `z`.

### Single-solve latch (shared by 6a and 6b)

```python
def _solve_for_tick(self, k, y, z_warm=None):
    if self._latch_k == k:
        return self._latch
    plan = self._planner.step(y, initial_guess=guess_from(z_warm))
    self._latch_k = k
    self._latch = TickSolve(plan=plan, z=result.z,
                           u_ff=plan.u[:, 0], x_ff=plan.x[:, 1])
    return self._latch
```

Each output port `compute(k, ...)` calls `_solve_for_tick` and returns one field. **First port
evaluated at tick `k` runs the NLP; the other two read the latch.** Tests assert
`planner.step` call count == number of MPC fires.

**Purity note:** port `compute` is conventionally pure in `(x, u, t, params)`. The latch is a
deliberate, documented exception for multi-output MPC leaves, keyed on integer tick `k` (always
passed by `Computer`).

### 6a static vs 6b step

| | **6a `MPCController`** | **6b `MPCStepBlock`** |
| --- | --- | --- |
| Base | `System`, `n=0` | `StepSystem`, `n=n_z` |
| `step_op` | none | `return self._latch.z` (no NLP) |
| Warm start | `initial_guess=None` | `z_warm` = `local_x` = `z_{k-1}`, shifted via `warm_start.py` |
| `z` persistence | ephemeral (`signals["z"]` only) | `computer.x` **and** `signals["z"]` after tick `k` |

**Do not use `expose_state=True` for `z` on 6b:** the state port is evaluated in port ops
**before** `step`, showing `z_{k-1}`. Use explicit output port `z` returning latched `z_k`, and/or
read `computer.x[:, k]` post-tick.

### Feedforward port semantics

- `u_ff = plan.u[:, 0]` — apply over current MPC interval (matches existing demos).
- `x_ff = plan.x[:, 1]` — nominal state at the **next** collocation node (for low-level feedback).
- `z` — packed `last_optimization_result.z`; logging, animation, 6b warm-start seed.

Factory validates `transcription.options.n_steps >= 2`.

## Block contracts

### `MPCController` — Phase 6a

**File:** `minilink/planning/mpc/controller.py`

- Extends `System`, `n=0` (mirror `ProportionalController`)
- **Input:** `y` (measurement, dim `planner.problem.sys.n`)
- **Outputs (three ports, one solve):** `u_ff`, `x_ff`, `z` as above
- **Solve latch:** `_solve_for_tick(k, y, z_warm=None)` — not a `StepSystem.step` in 6a
- Factory: `mpc_controller(planner)` — validates prepared `MPCPlanner`, `n_steps >= 2`, port dims

### `MPCStepBlock` — Phase 6b (pending)

*(User-facing name: warm-start MPC step block; not a separate “DynamicStepBlock” type.)*

- Extends `StepSystem`, `n = transcription.decision_dimension(problem)`
- **Same three output ports** and **same `MPCTickLatch`** as 6a
- **`step`:** `return latch.solve_for_tick(k, y, z_warm=local_x).z` — commits latched `z_k`, **no second NLP**
- Port `compute` paths pass `z_warm=local_x` (`z_{k-1}`) into the latch; latch calls `warm_start.py` before `planner.step`
- **`expose_state=False`** — do not wire `z` through the state port (pre-step `z_{k-1}`); use output port `z` and `computer.x[:, k]`

## Diagram outputs → `StepRollout` (no `plan_log`)

```python
step_diagram.connect_new_output_port("mpc", "u_ff", "u_ff")
step_diagram.connect_new_output_port("mpc", "x_ff", "x_ff")
step_diagram.connect_new_output_port("mpc", "z", "z")
```

`HybridSimulator` records boundary `outs` into `StepRollout.signals`. Shapes per tick:
`u_ff` `(m,)`, `x_ff` `(n,)`, `z` `(n_z,)`.

6a demo wires `u_ff` to plant; `x_ff` and `z` for optional feedback and post-sim horizon.

**Post-sim reconstruction** — `minilink/planning/mpc/plan_reconstruct.py`:

```python
def mpc_plans_from_rollout(
    computer: StepRollout,
    transcription,
    problem,
    *,
    z_source: str = "signals",
    t0: float,
    dt_mpc: float,
) -> list[tuple[float, Trajectory]]:
    ...
```

Animation: `HorizonPolyline(mpc_plans_from_rollout(...))` on `hybrid.animate(overlays=[...])`.

## Phase 6a deliverables

| Item | Detail |
| --- | --- |
| Block | `MPCController` + `mpc_controller()` |
| Demo | `examples/scripts/hybrid/demo_dynamic_bicycle_rate_mpc_straight_line.py` — generic `HybridDiagram`; **do not edit** `examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py` |
| Tests | `test_mpc_controller.py`; `test_mpc_hybrid_straight_line.py` (stateless parity on `u_ff`) |
| Exports | `mpc_controller`, `MPCController`, `mpc_plans_from_rollout` from `planning/mpc/__init__.py` |

### Demo wiring (sketch)

```python
mpc = mpc_controller(mpc_planner)
step_diagram = StepDiagramSystem()
step_diagram.add_subsystem(mpc, "mpc")
step_diagram.add_input_port("y")
step_diagram.connect("input", "y", "mpc", "y")
step_diagram.connect_new_output_port("mpc", "u_ff", "u_ff")
step_diagram.connect_new_output_port("mpc", "x_ff", "x_ff")
step_diagram.connect_new_output_port("mpc", "z", "z")

plant_diagram = DiagramSystem()
# rate bicycle + Demux(dims=(1,1)) at boundary u

hybrid = HybridDiagram(computer=Computer(step_diagram, schedule), plant=plant_diagram)
hybrid.connect_boundary(direction="computer_to_plant", computer_port="u_ff", plant_port="u")
hybrid.connect_boundary(direction="plant_to_computer", computer_port="y", plant_port="y")
```

**Parity (6a):** stateless — `initial_guess=None` every MPC tick; match hand loop on `u_ff`.

## Phase 6b implementation plan (warm-start `MPCStepBlock`)

**Goal:** Match the warm-started hand loop in
`examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py` (shifted
`prev_plan` guess, pin `x[:,0] = y`) when MPC runs inside `HybridDiagram` with
optimizer state `z` carried on `Computer.x`.

**Parity contract:** For the same plant, planner, schedule, and `x0`, hybrid
`signals["u_ff"]` at MPC fires equals the hand loop’s `u_hold` sequence (within
existing MPC demo tolerances). Stateless 6a hybrid demo stays unchanged.

### Architecture (reuse 6a latch)

```mermaid
sequenceDiagram
  participant C as Computer.tick k
  participant P as port_ops MPCStepBlock
  participant L as MPCTickLatch
  participant W as warm_start
  participant NLP as planner.step
  participant S as step_op

  Note over C: local_x = z_k-1 from computer.x
  P->>L: solve_for_tick(k, y, z_warm=local_x)
  L->>W: initial_guess from z_k-1 + y
  W-->>L: Trajectory or packed z0
  L->>NLP: planner.step(y, initial_guess=...)
  NLP-->>L: plan, z_k
  P-->>C: u_ff, x_ff, z_k to signals
  C->>S: step(local_x, u)
  S->>L: solve_for_tick (cached)
  S-->>C: return z_k
```

No tick-order changes. Port ops still own the sole NLP; `step` only commits the
same latched `z`.

### New / touched files

| File | Role |
| --- | --- |
| `minilink/planning/mpc/warm_start.py` | Shift logic extracted from MPC straight-line demo |
| `minilink/planning/mpc/step_block.py` | `MPCStepBlock`, `mpc_step_block()` factory |
| `minilink/planning/mpc/tick_latch.py` | Route `z_warm` through `warm_start` (not raw `z` passthrough) |
| `minilink/planning/mpc/controller.py` | Optional: shared port wiring helper (avoid duplicating three `add_output_port` blocks) |
| `minilink/planning/mpc/plan_reconstruct.py` | Optional: `z_source="x"` for `computer.x` history |
| `tests/unittest/test_mpc_step_block.py` | Unit: warm shift, one solve/tick, `step` commits `z` |
| `tests/unittest/test_mpc_hybrid_warm_start_parity.py` | Integration: hybrid vs warm-started hand loop on `u_ff` |

### `warm_start.py` API

Extract the demo’s shift (lines 122–137) into library functions. Two layers:

1. **`shift_plan_trajectory(plan, x_meas, *, dt_shift, horizon)`** — pure Trajectory
   shift (same mask / `t_shift` / pin-first-state logic as the hand loop).
2. **`mpc_warm_start_guess(z_prev, y, planner, *, dt_mpc)`** — entry for the latch:
   - If `z_prev` is missing or first tick: `default_initial_trajectory` on the
     collocation grid (same as demo `else` branch).
   - Else: `unpack(z_prev)` → build `Trajectory` on `transcription.options.t` →
     `shift_plan_trajectory` → return `Trajectory` for `planner.step(..., initial_guess=...)`.

**Why Trajectory shift, not matrix-only drop-first-column:** Parity with the
reference demo is defined in wall-clock / plan-time space. `pack_initial_guess`
already resamples a `Trajectory` onto the collocation grid inside
`MPCPlanner.step`.

```python
# warm_start.py (sketch)
def shift_plan_trajectory(plan, x_meas, *, dt_shift, horizon):
  ...

def mpc_warm_start_guess(z_prev, y, planner, *, dt_mpc):
  if z_prev is None:
      return default_initial_trajectory(...)
  x_mat, u_mat = planner.transcription.unpack(z_prev, planner.problem)
  plan = Trajectory(t=planner.transcription.options.t, x=x_mat, u=u_mat)
  return shift_plan_trajectory(plan, y, dt_shift=dt_mpc, horizon=...)
```

### `MPCTickLatch` change

Today `z_warm` is passed straight through as `initial_guess`. For 6b:

```python
def solve_for_tick(self, k, y, *, z_warm=None, dt_mpc=None, initial_guess=None):
    ...
    if guess is None and z_warm is not None:
        guess = mpc_warm_start_guess(z_warm, y, self._planner, dt_mpc=dt_mpc)
    elif guess is None and z_warm is None:
        guess = None  # 6a: planner default inside step()
```

`MPCController` port ops keep `z_warm=None`. `MPCStepBlock` port ops and `step`
pass `z_warm=x` (local state) and `dt_mpc` from the block.

### `MPCStepBlock` contract

```python
class MPCStepBlock(StepSystem):
    def __init__(self, planner: MPCPlanner, *, dt_mpc: float):
        n_z = planner.transcription.decision_dimension(planner.problem)
        super().__init__(n_z, expose_state=False)
        self._planner = planner
        self._latch = MPCTickLatch(planner)
        self._dt_mpc = float(dt_mpc)
        # input y + outputs u_ff, x_ff, z (same dims / deps as MPCController)

    def step(self, x, u, k=0, params=None):
        y = self._measurement(u)
        return self._latch.solve_for_tick(
            k, y, z_warm=x, dt_mpc=self._dt_mpc
        ).z

    def _compute_u_ff(self, x, u, t=0, params=None):
        return self._latch.solve_for_tick(
            t, self._measurement(u), z_warm=x, dt_mpc=self._dt_mpc
        ).u_ff
    # x_ff, z analogous
```

Factory **`mpc_step_block(planner, *, dt_mpc)`** — same validation as
`mpc_controller` (`prepare()`, `n_steps >= 2`).

**`x0` for `Computer.reset`:** packed default trajectory:

```python
from minilink.planning.initial_guess import default_initial_trajectory

z0 = planner.transcription.pack_initial_guess(
    planner.problem,
    default_initial_trajectory(
        planner.problem,
        planner.transcription.initial_guess_time_grid(planner.problem),
    ),
)
```

**Latch reset:** Call `block._latch.reset_latch()` from `Computer.reset` or
document that hybrid re-build / explicit reset clears memo between runs. If
`Computer.reset` does not reach blocks today, add a minimal hook or reset latch
in `mpc_step_block` factory doc + test setup.

### Shared port wiring (optional refactor)

If `controller.py` and `step_block.py` duplicate port setup, extract a small
internal helper (same file or `_ports.py`):

```python
def _add_mpc_feedforward_ports(block, latch, measurement_fn, *, dt_mpc=None):
    ...
```

Keep public surface unchanged; no new user-facing abstractions.

### Demo strategy

| Option | Pros | Choice |
| --- | --- | --- |
| Flag on existing hybrid demo | One script, A/B | **Preferred:** `USE_WARM_START = True` switches `mpc_controller` ↔ `mpc_step_block` |
| Separate hybrid warm-start demo | Clear filenames | Defer unless flag gets noisy |

When warm-start is on:

- `Computer.reset(x0=z0)` with packed default above.
- Horizon animation: `mpc_plans_from_rollout(..., z_source="signals")` still works;
  optionally support `z_source="x"` if `signals["z"]` and `computer.x` diverge in tests.

**Do not edit** `examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py`.

### Tests

| File | Cases |
| --- | --- |
| `test_mpc_step_block.py` | `n == decision_dimension`; `step` returns latched `z`; port ops use `z_warm=local_x`; `warm_start.shift_plan_trajectory` matches demo mask on toy plan; exactly one `planner.step` per tick |
| `test_mpc_hybrid_warm_start_parity.py` | Same bicycle rate setup as hybrid demo; warm `MPCStepBlock` hybrid vs extracted warm hand loop on `u_ff` at MPC fires (skip or relax if JAX MPC flaky in CI) |

Reuse planner fixtures from `test_mpc_controller.py` where possible.

### Exports and docs

- `from minilink.planning.mpc import MPCStepBlock, mpc_step_block`
- `DESIGN.md` — hybrid MPC subsection: 6b state = packed `z`, warm-start path
- `ROADMAP.md` — check 6b when parity lands
- `README.md` — one line in hybrid / MPC examples if demo flag exists
- This file + `00-master-plan.md` — mark 6b done when checklist complete

### Implementation order

1. **`warm_start.py`** + unit tests on `shift_plan_trajectory` (Trajectory-only, no planner).
2. **`tick_latch.py`** — `dt_mpc` + `mpc_warm_start_guess` integration; 6a tests still pass (`z_warm=None`).
3. **`step_block.py`** + `test_mpc_step_block.py`.
4. **Hybrid demo flag** + `test_mpc_hybrid_warm_start_parity.py`.
5. **Docs / exports**; optional `plan_reconstruct` `z_source="x"`.
6. **Verification** (below).

### Phase 6b deliverables checklist

- [x] `warm_start.py` (`shift_plan_trajectory`, `mpc_warm_start_guess`)
- [x] `MPCStepBlock` + `mpc_step_block()` in `step_block.py`
- [x] `MPCTickLatch` warm-start routing (`dt_mpc`, not raw `z`)
- [x] `test_mpc_step_block.py`
- [x] `test_mpc_hybrid_demo_parity.py` (full baseline vs `mpc/` hand loop)
- [x] `test_mpc_hybrid_warm_start_parity.py`
- [x] Hybrid demo warm-start switch (`USE_WARM_START`)
- [x] Exports; DESIGN / ROADMAP / README; master plan 6b done

## Explicit non-changes

- No `hybrid_mpc_closed_loop()` shortcut
- No `MPCPlanner` API changes
- No `HybridSimulator` / `Computer` tick-order changes
- Existing `examples/scripts/mpc/*` demos untouched until 6b parity

## Implementation checklist

- [x] `MPCController` + `mpc_controller()` in `controller.py`
- [x] `mpc_plans_from_rollout()` in `plan_reconstruct.py`
- [x] Exports; DESIGN / ROADMAP / README updates
- [x] `test_mpc_controller.py`, `test_mpc_hybrid_straight_line.py`
- [x] Hybrid straight-line demo + horizon overlay from `signals["z"]`
- [x] **6b:** `MPCStepBlock`, `warm_start.py`, warm-started demo parity

## Verification

**6a (current):**

```bash
conda activate minilink
ruff check . && ruff format --check .
pytest tests/unittest/test_mpc_controller.py tests/unittest/test_mpc_hybrid_straight_line.py
MPLBACKEND=Agg python examples/scripts/hybrid/demo_dynamic_bicycle_rate_mpc_straight_line.py
```

**6b (after implementation):**

```bash
pytest tests/unittest/test_mpc_step_block.py tests/unittest/test_mpc_hybrid_warm_start_parity.py tests/unittest/test_mpc_hybrid_demo_parity.py
MPLBACKEND=Agg python examples/scripts/hybrid/demo_dynamic_bicycle_rate_mpc_straight_line.py  # warm-start on
```

## Tests (6a + 6b)

| File | Cases |
| --- | --- |
| `test_mpc_controller.py` | `u_ff`/`x_ff`/`z` ports; exactly one `planner.step` per tick |
| `test_mpc_hybrid_straight_line.py` | hybrid vs stateless hand loop on `u_ff` |
| `test_mpc_step_block.py` (6b) | warm-start `z` state; shift matches demo; one solve per tick |
| `test_mpc_hybrid_demo_parity.py` (6b) | full hybrid warm-start vs `mpc/` hand loop (`u_ff`, plant `x`) |
| `test_mpc_hybrid_warm_start_parity.py` (6b) | hybrid vs warm-started hand loop on `u_ff` |

## Deferred

- JAX-differentiable MPC inside block (planner stays NumPy for v1)
- Multi-rate MPC via `StepSchedule.fire` on `mpc` sys_id
- `hybrid_mpc_closed_loop()` sugar when a second demo repeats Demux boilerplate — superseded by ``block % schedule`` + ``Computer @ plant`` and `JaxDynamicBicycleRateInputsUY`
