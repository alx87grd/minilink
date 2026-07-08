# Layer 3: Hybrid Simulation

**Files:** `minilink/core/hybrid_diagram.py`, `minilink/simulation/hybrid_simulator.py`

Represents a simple two-side topology only:

```
[ StepDiagramSystem ]  --ZOH on u-->  [ DiagramSystem plant ]
         ^                                      |
         +----------- sample y -------------------+
```

## `HybridDiagram`

```python
@dataclass
class HybridDiagram:
    step: StepDiagramSystem          # logical diagram (not expanded by default)
    continuous: DiagramSystem
    connections: list[BoundaryConnection]
    dt_base: float                   # Phase A outer tick (= Ts); Phase B = schedule.dt_base
    schedule: StepSchedule | None = None   # None => all blocks fire every tick (Phase A)
```

## `HybridSimulator`

Executes the interactions between the step-domain and the continuous domain per `dt_base` tick.

**Algorithm:**
1. **Step side:** Calls `step_evaluator.step` (all blocks) or uses `ScheduledStepOrchestrator.tick` if `schedule` is present.
2. **Boundary:** Holds the controller output `u` via Zero-Order Hold (ZOH) for the plant over `[t, t + dt_base]`.
3. **Continuous:** Evaluates `cont_evaluator.rk4_rollout_zoh(x, u_hold, t, dt_base)` (using JAX scan for inner loops).
4. **Feedback:** Samples plant outputs `y` at the end of the tick boundary to feed into controller inputs for the next tick.

## Control Blocks

**Location:** `minilink/planning/mpc/`, `minilink/control/`

### `MPCBlock`
- **`StepSystem`** wrapping `MPCPlanner.step`.
- Warm-start state initialized in `__init__`; no `core/compile/` imports.
- Explicitly separates `sys_mpc` vs `sys_sim` for teaching clarity.

### `SMCBlock` (Sampled Sliding-Mode Controller)
- **`StepSystem`** implementing a discrete-time sliding-mode law (or equivalent).
- Included in Phase A to prove hybrid sim is generalized beyond MPC.

### Tests
- `test_rk4_rollout_zoh.py`: NumPy/JAX parity for ZOH integration.
- `test_hybrid_simulator.py`: ZOH → plant → sample feedback.
- `test_mpc_block.py` and `test_smc_hybrid.py`: Smoke testing the controllers.\n