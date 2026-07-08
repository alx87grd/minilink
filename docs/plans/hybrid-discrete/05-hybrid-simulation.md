# Phase 5: Hybrid Simulation

**After [Phase 4](04-scheduled-orchestrator.md).** Two-side topology: **step diagram** (controller)
+ **continuous diagram** (plant), linked by explicit boundary connections.

**Requires:** Phases 1–2 (step compile), Phase 4 (`StepSchedule` + orchestrator on step side).
Phase 3 (`discretize`) **not** required. **MPC hybrid demos** land in
[Phase 6](06-mpc-step-block.md); Phase 5a validates hybrid with **SMC** or generic `StepSystem` controllers.

**Files:** `minilink/core/hybrid_diagram.py`, `minilink/simulation/hybrid_simulator.py`

## Split of concerns (Phase 5)

```text
HybridSimulator @ schedule.dt_base
│
├── ScheduledStepOrchestrator     ← step diagram clock + who fires (Phase 4)
│       trivial fire: single-rate (5a)
│       non-trivial fire: filter + slow block cascade (5b)
│
├── boundary ZOH / sample         ← step ↔ plant edges (HybridSimulator only)
│
└── rk4_rollout_zoh               ← continuous plant over dt_base (optional inner subdivide)
```

```text
  r ──► [ StepDiagramSystem ] ──u_cmd──ZOH──► plant.u
              ▲                                    │
              ├──y_meas──sample── plant.y ─────────┤
              └──v_fb────sample── plant.v ─────────┘   (optional)

  World inputs (`r`) attach to step/plant diagram boundaries — not hybrid edges.
```

## `BoundaryConnection`

Connections are between **diagram boundary port names** on the step and continuous sides — not
subsystem ids.

| Direction | Semantics | `direction` value |
| --- | --- | --- |
| Step diagram output → plant diagram input | **ZOH** over `[t_k, t_k + dt_base)` | `"step_to_plant"` |
| Plant diagram output → step diagram input | **Sample** at tick boundary | `"plant_to_step"` |

```python
@dataclass(frozen=True)
class BoundaryConnection:
    direction: Literal["step_to_plant", "plant_to_step"]
    step_port: str
    continuous_port: str
```

```python
def connect_boundary(
    self,
    *,
    direction: Literal["step_to_plant", "plant_to_step"],
    step_port: str,
    continuous_port: str,
) -> None: ...
```

Validate port existence and dimensions at connect time.

## `HybridDiagram`

```python
@dataclass
class HybridDiagram:
    step: StepDiagramSystem
    continuous: DiagramSystem
    schedule: StepSchedule              # required — dt_base lives here
    connections: list[BoundaryConnection] = field(default_factory=list)
```

No separate `dt_base` field — use **`schedule.dt_base`** for step ticks and plant hold interval (v1).

## `HybridSimulator`

**Always** calls **`ScheduledStepOrchestrator.tick`** on the step side (single-rate = trivial
`fire`). Owns boundary buffers and plant rollout.

### Tick order and buffers

Logical order each base tick **`k`** (hybrid simulator also tracks **`t_k = t0 + k · dt_base`**
for the **plant only**):

1. **Sample (read)** — assemble `u_step` from **`sample_buffers`** (plant outputs latched at end
   of tick `k-1`) and world refs on step-diagram boundaries.
2. **Step** — `orchestrator.tick(x_step, u_step, k)` → `step_outputs` (**`k` only** into step
   diagram eval — no `t` on `StepSystem` leaves).
3. **ZOH (write)** — step boundary outputs → **`zoh_buffers`** → assemble `u_plant`.
4. **Flow** — `rk4_rollout_zoh(x_flow, u_plant, t_k, schedule.dt_base, dt_inner=...)` — **plant
   uses float `t`**.
5. **Sample (write)** — plant boundary outputs at `t_k + dt_base` → **`sample_buffers`** for tick
   `k+1`.

**Coordinate split:** step side = **`k` (int)**; continuous plant = **`t` (float)**. Do not pass
`t_k` into `StepSystem.step` or step-diagram port gather.

**One-tick feedback delay:** the controller at tick `k` sees plant measurements from the **end**
of interval `k-1`, not the state after integrating interval `k`.

**Tick 0 initialization:**

- **`sample_buffers`**: evaluate plant diagram outputs at `(x_flow_0, u_plant_0, t_0)` where
  `u_plant_0` is nominal / zero hold before the first step command (document chosen default).
- **`zoh_buffers`**: empty until the first `orchestrator.tick` completes.
- Do **not** pass raw `x_flow` into step inputs — boundary samples only.

### `rk4_rollout_zoh`

```python
def rk4_rollout_zoh(
    self, x0, u_hold, t0, dt_hold, *, dt_inner: float | None = None
) -> np.ndarray:
    """
    Integrate with piecewise-constant u_hold over [t0, t0 + dt_hold].

    dt_inner: RK4 sub-step (default dt_hold). Set to SIM_DT when dt_hold = MPC_DT
    and plant should sub-step (matches today's SUBSTEPS loops).
    """
```

`HybridSimulator` accepts `plant_dt_inner` (or equivalent) and forwards to the evaluator.

## Milestones

| Milestone | Content |
| --- | --- |
| **5a** | `rk4_rollout_zoh`, hybrid + **trivial schedule**, `SMCBlock` (or generic step controller), hybrid acceptance vs hand-rolled **non-MPC** loop |
| **5b** | Cascade hybrid — **non-trivial `fire`** (e.g. filter @ 100 Hz + slow `StepSystem` @ 10 Hz); MPC slot filled by [Phase 6a](06-mpc-step-block.md) in demo refresh |

## End-to-end API (5a — single-rate SMC)

```python
controller = StepDiagramSystem()
controller.add_subsystem(smc_block, "smc")
controller.add_input_port("y")
controller.connect_new_output_port("smc", "u", "u")

plant = DiagramSystem()
plant.add_subsystem(bicycle, "plant")

schedule = StepSchedule(dt_base=TS)  # empty fire => all blocks every tick

hybrid = HybridDiagram(step=controller, continuous=plant, schedule=schedule)
hybrid.connect_boundary(direction="step_to_plant", step_port="u", continuous_port="u")
hybrid.connect_boundary(direction="plant_to_step", step_port="y", continuous_port="y")

HybridSimulator(hybrid, plant_dt_inner=SIM_DT, ...).run()
```

## End-to-end API (5b — multi-rate controller)

```python
schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "slow_ctrl": 10})
hybrid = HybridDiagram(step=controller, continuous=plant, schedule=schedule)
hybrid.connect_boundary(direction="step_to_plant", step_port="u", continuous_port="u")
hybrid.connect_boundary(direction="plant_to_step", step_port="y", continuous_port="y")
HybridSimulator(hybrid, plant_dt_inner=SIM_DT, ...).run()
```

## Control blocks (Phase 5)

- **`SMCBlock`** (or documented `StepSystem` pattern) — Phase 5a deliverable.
- **MPC** — [Phase 6](06-mpc-step-block.md) (`MPCStepBlock`); do not duplicate a second MPC adapter in Phase 5.

## Tests

- `test_rk4_rollout_zoh.py` — NumPy/JAX; `dt_inner` subdivides `dt_hold`
- `test_hybrid_simulator.py` — multi-port boundary; 5a matches hand-rolled SMC (or test double)
- `test_hybrid_boundary_connect.py` — invalid boundary wiring
- `test_hybrid_cascade.py` — 5b filter + slow block
- `test_smc_hybrid.py` — 5a smoke

**5c** (viz + shortcuts): [05c-hybrid-viz-shortcuts.md](05c-hybrid-viz-shortcuts.md).
