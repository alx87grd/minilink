# Layer 3: Hybrid Simulation

**Files:** `minilink/core/hybrid_diagram.py`, `minilink/simulation/hybrid_simulator.py`

Two-side topology: a **step diagram** (controller) and a **continuous diagram** (plant), linked by
explicit **boundary connections** — not a single hard-coded `u` / `y` pair.

```text
  r ──► [ StepDiagramSystem ] ──u_cmd──ZOH──► plant.u
              ▲                                    │
              ├──y_meas──sample── plant.y ─────────┤
              └──v_fb────sample── plant.v ─────────┘   (optional extra ports)

  World references (`r`, disturbances) attach to diagram boundary inputs on one side only —
  they are not hybrid cross-connections.
```

**Minimal case (MPC / SMC demos):** one step→plant edge (command vector) + one plant→step edge
(measurements). Same API; fewer `connect_boundary` calls.

## `BoundaryConnection`

Each edge crosses the hybrid interface **output → input**, with fixed Phase A semantics:

| Direction | Semantics |
| --- | --- |
| **Step → continuous** | **ZOH** — hold last step-side value over `[t, t + dt_base]` |
| **Continuous → step** | **Sample** — latch plant value at tick boundary for the next step |

```python
@dataclass(frozen=True)
class BoundaryConnection:
    """
    One directed edge across the hybrid interface.

    Exactly one of (step side, continuous side) is an output port; the other is the
    matching input. Validation at connect time: port dims match, roles consistent.
    """
    step: tuple[str, str]          # (sys_id | "output", port_id) on step diagram
    continuous: tuple[str, str]    # (sys_id | "input", port_id) on continuous diagram
    # Direction inferred: step output → continuous input, or continuous output → step input
```

**Examples:**

```python
# Command vector: step diagram boundary output "u" → plant input "u"
BoundaryConnection(step=("output", "u"), continuous=("plant", "u"))

# Measurement: plant output "y" → step diagram boundary input "y"
BoundaryConnection(step=("input", "y"), continuous=("plant", "y"))
```

Use **`HybridDiagram.connect_boundary(...)`** (mirrors `DiagramSystem.connect` validation:
existence, dimension, output→input).

### Phase A boundary rules

| Rule | Behavior |
| --- | --- |
| Multiple step→plant edges | Each signal ZOH'd independently; assembled into plant boundary `u` |
| Multiple plant→step edges | Each signal sampled independently; assembled into step external inputs |
| Unconnected diagram inputs | Nominal / world inputs (`r`, disturbances) via each side's own boundary API |
| Cross-boundary algebraic loop | **Rejected or one-tick delay** — sample plant outputs before step (see tick order) |
| Per-port FOH, delay, async rates | **Deferred** — ZOH + sample @ `dt_base` only |

## `HybridDiagram`

```python
@dataclass
class HybridDiagram:
    step: StepDiagramSystem          # logical diagram (not expanded by default)
    continuous: DiagramSystem
    connections: list[BoundaryConnection]
    dt_base: float                   # Phase A outer tick (= Ts); Phase B = schedule.dt_base
    schedule: StepSchedule | None = None   # None => all blocks fire every tick (Phase A)

    def connect_boundary(
        self,
        step_sys_id,
        step_port_id,
        continuous_sys_id,
        continuous_port_id,
    ) -> None: ...
```

Compile builds **`BoundaryMap`**: slices into step `u_ext` / continuous `u` vectors, plus
metadata for `HybridSimulator` buffers (same idea as `ExecutionPlan` `EXTERNAL_INPUT` gathers).

## `HybridSimulator`

Executes step and continuous sides per **`dt_base`** tick. Boundary I/O is driven entirely by
`connections` — no special case for port names `"u"` / `"y"`.

### Tick order (documented convention)

At tick `k` with wall time `t_k = t0 + k · dt_base`:

1. **Sample** — read continuous boundary outputs at end of previous plant interval; write
   **`sample_buffers`** for each continuous→step connection; assemble **`u_step`** (world +
   samples).
2. **Step** — `step_evaluator.step(x_step, u_step, k)` (or `ScheduledStepOrchestrator.tick`
   when `schedule` is set).
3. **ZOH** — read step boundary outputs; write **`zoh_buffers`** for each step→continuous
   connection; assemble **`u_plant`**.
4. **Flow** — `cont_evaluator.rk4_rollout_zoh(x_flow, u_plant, t_k, dt_base)` (optional inner
   substeps `dt_plant << dt_base`).

Sample-before-step gives a standard one-tick delay on feedback and avoids same-instant
algebraic loops across the boundary.

### Buffers

| Buffer | Keys | Role |
| --- | --- | --- |
| **`zoh_buffers`** | connection id or `(continuous_sys, port)` | Last step output held for plant |
| **`sample_buffers`** | connection id or `(step_sys, port)` | Last plant output for step inputs |

Export on trajectory as **`HybridSimResult.boundary_buffers`** (debug / plot), analogous to
orchestrator `buffer_history`.

### Pseudocode (multi-port)

```python
x_flow, x_step = x_flow_0, x_step_0
zoh_buffers, sample_buffers = {}, {}
t, k = t0, 0

while t < tf:
    u_step = assemble_step_inputs(
        hybrid, world_inputs, sample_buffers, x_flow, zoh_buffers, t, k
    )
    x_step = step_evaluator.step(x_step, u_step, k)  # or orchestrator.tick(...)

    step_out = step_evaluator.outputs(x_step, u_step, k)
    u_plant = assemble_plant_inputs(hybrid, zoh_buffers, step_out)

    x_flow = cont_evaluator.rk4_rollout_zoh(x_flow, u_plant, t, dt_base)
    plant_out = cont_evaluator.outputs(x_flow, u_plant, t + dt_base)
    update_sample_buffers(hybrid, sample_buffers, plant_out)

    t += dt_base
    k += 1
```

**Phase B:** replace the single `step_evaluator.step` line with
`ScheduledStepOrchestrator.tick(...)`; **plant boundary path unchanged**.

## Control Blocks

**Location:** `minilink/planning/mpc/`, `minilink/control/`

### `MPCBlock`
- **`StepSystem`** implementing `step(x, u, k)` by wrapping `MPCPlanner.step` (planner API unchanged).
- Warm-start state initialized in `__init__`; no `core/compile/` imports.
- Explicitly separates `sys_mpc` vs `sys_sim` for teaching clarity.
- Demos: one step→plant + one plant→step boundary edge (vector ports OK).

### `SMCBlock` (Sampled Sliding-Mode Controller)
- **`StepSystem`** with discrete-time `step(x, u, k)` (sliding-mode law or equivalent).
- Included in Phase A to prove hybrid sim is generalized beyond MPC.

## End-to-end API (minimal MPC loop)

```python
controller = StepDiagramSystem()
controller.add_subsystem(mpc_block, "mpc")
controller.add_input_port("y")
controller.connect_new_output_port("mpc", "u", "u")

plant = DiagramSystem()
plant.add_subsystem(bicycle, "plant")
# ... plant wiring ...

hybrid = HybridDiagram(step=controller, continuous=plant, dt_base=MPC_DT)
hybrid.connect_boundary("output", "u", "plant", "u")   # step → plant, ZOH
hybrid.connect_boundary("input", "y", "plant", "y")   # plant → step, sample
# hybrid.connect_boundary("input", "v", "plant", "v")  # optional extra feedback

HybridSimulator(hybrid, ...).run()
```

## Tests

- `test_rk4_rollout_zoh.py`: NumPy/JAX parity for ZOH integration.
- `test_hybrid_simulator.py`: multi-port boundary — ZOH → plant → sample → step; minimal u/y
  loop matches hand-rolled MPC tolerance.
- `test_hybrid_boundary_connect.py`: dim mismatch and invalid direction raise at connect time.
- `test_mpc_block.py` and `test_smc_hybrid.py`: smoke testing the controllers.
