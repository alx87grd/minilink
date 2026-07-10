# Phase 5: Hybrid Simulation

**Status: Phase 5 done** (July 2026, `0ccba60`). **5a** Pyro SMC hybrid compare demo done; **5b** multi-rate
demo done; **5c** composite `plot_diagram` + wiring shortcuts done. Fine plant recording at
``plant_dt_inner`` via ``integrate_zoh_rollout``; façade cache ``traj`` / ``last_result`` /
``rollout``; ``animate()`` on plant geometry.

**After [Phase 4](04-computer.md).** v1 hybrid topology: **`Computer`** (discrete side) +
**`DiagramSystem`** plant (continuous side), linked by **one boundary** with **multi-channel**
`connect_boundary` edges.

**Requires:** Phases **1–2** (step compile), Phase **1b** (`DiagramSystem` IS-A `DynamicSystem` for
plant side), Phase 4 (`Computer` on step side).
Phase 3 (`discretize`) **not** required. **MPC hybrid demos** land in
[Phase 6](06-mpc-step-block.md); Phase 5a validates hybrid with **SMC** or generic `StepSystem` controllers.

**Files:** `minilink/core/hybrid_diagram.py`, `minilink/simulation/hybrid_simulator.py`

## v1 topology (frozen)

```text
HybridDiagram
├── computer: Computer          # StepDiagramSystem + StepSchedule + .tick()
├── plant: DiagramSystem      # continuous side (rich internals OK)
└── connections: list[BoundaryConnection]   # ONE boundary, many channels
```

- **Not** a true arbitrary `DiagramSystem` — heterogeneity lives only here.
- **Inside** each side: arbitrary subsystems and wiring (same as homogeneous diagrams).
- **Between** sides: explicit list of boundary port pairs — **multi-channel from day one**
  (e.g. `u` command + auxiliary feedforward), all labeled ZOH or sample.

## Split of concerns (Phase 5)

```text
HybridSimulator @ computer.schedule.dt_base
│
├── Computer.tick               ← step diagram clock + who fires (Phase 4)
│       trivial fire: single-rate (5a)
│       non-trivial fire: filter + slow block cascade (5b)
│
├── boundary ZOH / sample     ← single computer↔plant interface (HybridSimulator only)
│
└── integrate_zoh           ← continuous plant over dt_base (optional inner subdivide)
```

```text
  r ──► [ Computer / StepDiagram ] ──u_cmd──ZOH──► plant.u
              ▲                                        │
              ├──y_meas──sample── plant.y ─────────────┤
              └──v_fb────sample── plant.v ─────────────┘   (extra channels OK)

  World inputs (`r`) attach to computer/plant diagram boundaries — not hybrid edges.
```

## `BoundaryConnection`

Connections are between **diagram boundary port names** on the computer and plant sides — not
subsystem ids.

| Direction | Semantics | `direction` value |
| --- | --- | --- |
| Computer diagram output → plant diagram input | **ZOH** over `[t_k, t_k + dt_base)` | `"computer_to_plant"` |
| Plant diagram output → computer diagram input | **Sample** at tick boundary | `"plant_to_computer"` |

Legacy alias values `"step_to_plant"` / `"plant_to_step"` may be accepted in v1 for readability;
canonical names are **`computer_to_plant`** / **`plant_to_computer`**.

```python
@dataclass(frozen=True)
class BoundaryConnection:
    direction: Literal["computer_to_plant", "plant_to_computer"]
    computer_port: str
    plant_port: str
```

```python
def connect_boundary(
    self,
    *,
    direction: Literal["computer_to_plant", "plant_to_computer"],
    computer_port: str,
    plant_port: str,
) -> None: ...
```

Validate port existence and dimensions at connect time. **Multiple** connections form the single
logical boundary (multi-channel vectors and multiple named ports).

## `HybridDiagram`

```python
@dataclass
class HybridDiagram:
    computer: Computer
    plant: DiagramSystem
    connections: list[BoundaryConnection] = field(default_factory=list)
```

- **`computer`** — bundles `StepDiagramSystem` + `StepSchedule`; sample time is
  **`computer.schedule.dt_base`**.
- **`plant`** — continuous `DiagramSystem` (may wrap many internal blocks).
- No separate `schedule` field — schedule lives on **`Computer`**.

Convenience constructor (optional): `HybridDiagram.from_diagrams(step_diagram, plant, schedule)`
wraps `Computer(step_diagram, schedule)`.

## `HybridSimulator`

**Always** calls **`computer.tick(u)`** on the discrete side (single-rate = trivial `fire`). Owns
**`x_plant`**, hybrid boundary buffers, and plant rollout. Does **not** pass **`k`** into
Computer — **`computer.k`** is the single discrete index; HybridSimulator derives
**`t_k = t0 + computer.k * dt_base`** for the plant.

**Public API** mirrors continuous :class:`~minilink.simulation.simulator.Simulator`:
construct with **`t0` / `tf`**, then **`solve()`** / **`solve_forced(u)`** →
:class:`~minilink.simulation.hybrid_simulator.HybridSimResult`. No ``run(n_ticks)``.
Façades on :class:`HybridDiagram`: ``compute_trajectory`` / ``compute_forced``.
Cache: ``self.traj`` = plant :class:`~minilink.core.trajectory.Trajectory`;
``self.last_result`` = full :class:`~minilink.simulation.hybrid_simulator.HybridSimResult`;
``self.rollout`` = ``last_result.computer``. ``animate()`` drives plant geometry from
``self.traj``.

**Result contract:** ``HybridSimResult.computer`` is a tick-indexed
:class:`~minilink.core.step_rollout.StepRollout`; ``HybridSimResult.plant`` is a
continuous-time :class:`~minilink.core.trajectory.Trajectory` recorded at
``plant_dt_inner`` (or ``dt_base`` when unset). ``plot()`` / ``plot_trajectory()`` default
to the plant view; ``plot_computer()`` covers tick-indexed internals.

### Tick order and buffers

Logical order each **base tick** (let **`k = computer.k`** at tick start; plant time
**`t_k = t0 + k · dt_base`**):

1. **Sample (read)** — assemble boundary vector **`u`** for the computer diagram from
   **`sample_buffers`** (plant outputs latched at end of tick `k-1`) and world refs.
2. **Computer** — **`outs = computer.tick(u)`** (stateful; internal **`k`** advances; leaf blocks
   receive **`k`** internally — no **`t`**).
3. **ZOH (write)** — **`outs`** → **`zoh_buffers`** → assemble **`u_plant`**.
4. **Flow** — **`integrate_zoh(x_plant, u_plant, t_k, dt_base, dt_inner=...)`**.
5. **Sample (write)** — plant boundary outputs at **`t_k + dt_base`** → **`sample_buffers`** for
   the next tick.

**Coordinate split:** computer side = **`k` (int)**; continuous plant = **`t` (float)**. Do not pass
`t_k` into `StepSystem.step` or step-diagram port gather.

**One-tick feedback delay:** the controller at tick `k` sees plant measurements from the **end**
of interval `k-1`, not the state after integrating interval `k`.

**Tick 0 initialization:**

- **`sample_buffers`**: evaluate plant diagram outputs at `(x_plant_0, u_plant_0, t_0)` where
  `u_plant_0` is nominal / zero hold before the first step command (document chosen default).
- **`zoh_buffers`**: empty until the first **`computer.tick(u)`** completes.
- Do **not** pass raw `x_plant` into computer inputs — boundary samples only.

### `integrate_zoh`

Sugar on :class:`~minilink.core.compile.evaluators.integration.IntegrationMixin`
(wraps existing :meth:`~minilink.core.compile.evaluators.integration.IntegrationMixin.integrate`
with repeated ``u_hold`` rows):

```python
def integrate_zoh(
    self, x0, u_hold, t0, dt_hold, *, dt_inner: float | None = None
) -> np.ndarray:
    """
    Integrate with piecewise-constant u_hold over [t0, t0 + dt_hold].

    dt_inner: RK4 sub-step (default dt_hold). Set to SIM_DT when dt_hold = MPC_DT
    and plant should sub-step (matches today's SUBSTEPS loops).
    """
```

`HybridSimulator` accepts `plant_dt_inner` (or equivalent) and forwards to
:meth:`~minilink.core.compile.evaluators.integration.IntegrationMixin.integrate_zoh_rollout`
for fine plant trajectory recording (final state from last sample when only integration is needed).

## Fine plant recording (`0ccba60`)

When ``plant_dt_inner < dt_base``, ``HybridSimulator`` records a plant
:class:`~minilink.core.trajectory.Trajectory` on the inner grid while the computer side stays
tick-indexed. ``HybridSimResult.plant`` is the default plot/animate view;
``HybridSimResult.computer`` (and ``hybrid.rollout``) cover tick-indexed internals.

``hybrid_closed_loop`` copies leaf ``camera_scale`` / ``camera_target`` onto the wrapped plant
diagram so ``hybrid.animate()`` matches direct ``plant.animate()`` framing.

## Milestones

| Milestone | Content |
| --- | --- |
| **5 core** | `integrate_zoh` / `integrate_zoh_rollout`, `HybridDiagram`, `HybridSimulator` + `HybridSimResult.plot`, multi-rate demo, fine plant traj |
| **5a** | Pyro SMC continuous vs hybrid compare — `demo_smc_pendulum_compare.py`; `test_smc_hybrid.py` |
| **5b** | Cascade hybrid — **non-trivial `fire`** (e.g. filter @ 100 Hz + slow `StepSystem` @ 10 Hz) |

## End-to-end API (5a — single-rate SMC)

```python
controller = StepDiagramSystem()
controller.add_subsystem(smc_block, "smc")
controller.add_input_port("y")
controller.connect_new_output_port("smc", "u", "u")

plant = DiagramSystem()
plant.add_subsystem(bicycle, "plant")

computer = Computer(controller, StepSchedule(dt_base=TS))  # empty fire => all blocks every tick

hybrid = HybridDiagram(computer=computer, plant=plant)
hybrid.connect_boundary(direction="computer_to_plant", computer_port="u", plant_port="u")
hybrid.connect_boundary(direction="plant_to_computer", computer_port="y", plant_port="y")

HybridSimulator(hybrid, t0=0, tf=TS, plant_dt_inner=SIM_DT).solve_forced(u)
```

## End-to-end API (5b — multi-rate controller)

```python
computer = Computer(
    controller,
    StepSchedule(dt_base=0.01, fire={"filter": 1, "slow_ctrl": 10}),
)
hybrid = HybridDiagram(computer=computer, plant=plant)
hybrid.connect_boundary(direction="computer_to_plant", computer_port="u", plant_port="u")
hybrid.connect_boundary(direction="plant_to_computer", computer_port="y", plant_port="y")
HybridSimulator(hybrid, t0=0, tf=TS, plant_dt_inner=SIM_DT).solve_forced(u)
```

## Control blocks (Phase 5)

- **Static SMC** — `SlidingModeController(...)` in `hybrid_closed_loop(..., schedule=dt)` (Phase 5a).
- **MPC** — [Phase 6](06-mpc-step-block.md) (`MPCStepBlock`); do not duplicate a second MPC adapter in Phase 5.

## Tests

- `test_integrate_zoh.py` — NumPy; `integrate_zoh_rollout` grid; `dt_inner` subdivides `dt_hold`
- `test_hybrid_fine_recording.py` — fine plant samples vs computer ticks
- `test_hybrid_simulator.py` — **multi-channel** boundary; cache `traj` / `last_result`; `animate` smoke
- `test_hybrid_boundary_connect.py` — invalid boundary wiring; dimension mismatch
- `test_hybrid_cascade.py` — 5b filter + slow block
- `test_smc_hybrid.py` — 5a smoke

**5c** (viz + shortcuts): [05c-hybrid-viz-shortcuts.md](05c-hybrid-viz-shortcuts.md).
