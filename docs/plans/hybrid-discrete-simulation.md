# Hybrid and step simulation (design only)

Status: draft plan (July 2026). No implementation in this phase.

Architecture for **step maps** (difference equations, jumps, turn-based dynamics),
step diagrams, **scheduled multi-rate orchestration**, clock-driven hybrid simulation
(step side ZOH → continuous plant ← measurements), and a path toward **full switched /
hybrid** systems. Supersedes the continuous-time-only stance in [DESIGN.md](../../DESIGN.md)
§3 for this **subset** — not full Simulink parity.

Implemented contracts (when landed) live in [DESIGN.md](../../DESIGN.md),
[ROADMAP.md](../../ROADMAP.md), and [README.md](../../README.md) call chains.

---

## Summary

Build in layers on **two fundamental state-evolution maps**:

| Map | Leaf type | Equation | Meaning |
| --- | --- | --- | --- |
| **Flow `f`** | `DynamicSystem` | `dx = f(x, u, t; p)` | ODE / continuous evolution |
| **Step `phi`** | `StepSystem` | `x⁺ = φ(x, u, t; p)` | One-shot update (difference eq., jump, turn) |

**Static** blocks (`StaticSystem`, `n = 0`) and **output** map `h` are shared across all
kinds. **Hybrid** is not a third map — it is **orchestration** that calls flow and step
engines and decides **when** each applies.

Near-term deliverables:

1. **`StepSystem`** — sibling of `DynamicSystem`; **`phi`** returns next state (not `dx`).
2. **Step diagram pipeline** — sibling `StepDiagramSystem` + compile + `StepRunner`
   (clock-free) + timed `TimedStepSimulator`; reuse wiring and `ExecutionPlan`.
3. **Hybrid simulation (v1)** — two-side topology only: step side → ZOH → continuous plant
   ← measurements; **`HybridSimulator`** owns the orchestration loop.

Multi-rate cascade (e.g. 10 Hz MPC + 100 Hz filter) keeps the **logical**
`StepDiagramSystem` unchanged; a **`ScheduledStepOrchestrator`** runs at `dt_base`,
fires each block on schedule, and owns **hold/latch buffers** for cross-rate signals.
**Optional lowering:** `expand_scheduled_step()` rewrites the diagram with internal
`RateGate` / hold blocks — for JAX scan or teaching, not the default path.

---

## Short-term use cases (what we are building for)

The first **hybrid** capability is **not** an arbitrary mixed diagram. It is one recurring
pattern:

```text
[ sampled controller (step side) ]  --ZOH on u-->  [ continuous plant (flow side) ]
              ^                                              |
              +---------------- measurements y ----------------+
```

Both primary near-term targets share this **two-side** structure; only the controller block
differs.

| Use case | Controller (step side) | Plant (flow side) | Sample time |
| --- | --- | --- | --- |
| **MPC closed loop** | `MPCBlock` (`StepSystem` wrapping `MPCPlanner.step`) | `DynamicBicycle` (or other catalog plant) | `Ts` = MPC period |
| **Sliding-mode control (SMC)** | `SMCBlock` or generic sampled `StepSystem` controller | same continuous plant | `Ts` = SMC period |

**ZOH semantics:** between controller updates, plant input `u` is **held constant**
(zero-order hold). Measurements `y` are **sampled** at controller ticks (typically end of each
plant sub-interval, or start — pick one convention and document in `HybridSimulator`).

This is the **only** hybrid topology in scope for v1. Controllers may contain static wiring
(gains, saturations, mux) inside the step diagram; the plant stays a homogeneous flow diagram.

### Phase A — single-rate controller (ship first)

One controller block, one sample period `Ts`, plant integrated with inner step `dt_plant`
where `dt_plant = Ts` or subdivides `Ts` (e.g. `Ts = 0.1 s`, plant RK4 at `0.01 s` per
`HybridSimulator` tick).

```text
Phase A topology (single Ts):

  r ──► [ MPC or SMC @ 1/Ts ] ──ZOH──► [ plant ] ──► y
              ▲                              │
              └──────── y sampled ───────────┘
```

**Deliverables:**

- `HybridDiagram` + `HybridSimulator` with boundary ZOH + measurement sample.
- `MPCBlock` + refactor one MPC demo (straight-line bicycle).
- **`SMCBlock`** (or documented pattern): `StepSystem` implementing discrete-time sliding
  mode; same `HybridSimulator` loop as MPC — **no second sim path**.
- Controller step diagram may be a **single leaf** (`MPCBlock` / `SMCBlock`) plus static
  blocks; no multi-rate expansion required yet.

**Acceptance:** closed-loop sim matches hand-rolled demo loops within tolerance; plant sees
piecewise-constant `u` between samples.

### Phase B — multi-block controller, integer multiples of base clock

Cascade or multi-rate **controller only** — e.g. outer MPC @ 10 Hz, inner reference filter @
100 Hz, still driving one continuous plant. All controller blocks run at rates that are
**integer divisors of one `dt_base`** (no non-integer ratios in v1).

```text
Phase B topology (orchestrator on logical diagram):

  r ──► [ filter @ 100 Hz ] ──► [ MPC @ 10 Hz ] ──ZOH──► [ plant @ continuous ]
              StepDiagramSystem (logical)  +  StepSchedule  +  ScheduledStepOrchestrator
```

**Mechanism:** user wires logical `StepDiagramSystem`; attaches
`StepSchedule(dt_base, fire={...})` on `HybridDiagram` (or orchestrator); **no graph
rewrite**. `ScheduledStepOrchestrator` @ `dt_base` updates buffers and calls each block's
`phi` when its divisor fires. Plant side unchanged from Phase A.

**Example schedule** (`dt_base = 0.01 s` = 100 Hz):

| Block | Rate | `fire[sys_id]` (divisor k) |
| --- | --- | --- |
| `filter` | 100 Hz | `1` |
| `mpc` | 10 Hz | `10` |

**Deliverables:** `StepSchedule`, `ScheduledStepOrchestrator`, buffer snapshot API for
debug; cascade demo (filter fast + MPC slow).

**Deferred in Phase B:** non-integer ratios (e.g. 30 Hz vs 100 Hz), FOH, delays, mixing flow
blocks on the controller side.

### Multi-rate Phase B: orchestrator vs expansion

Two ways to implement the **same high-level API** (logical cascade + schedule). **Default:
orchestrator buffers.**

| | **A — Orchestrator buffers** (adopt) | **B — Diagram expansion** (optional) |
| --- | --- | --- |
| **Idea** | Logical diagram unchanged; orchestrator owns ZOH/latch **buffers** and fires blocks on schedule | `expand_scheduled_step()` inserts `RateGate`, `HoldRegister`, `SampleLatch` as step blocks with state |
| **Block contract** | Pure `phi`; rate via `StepSchedule.fire` (optional per-block hint in `params`) | Pure `phi` wrapped in infrastructure blocks |
| **Hold state** | Orchestrator side buffers (export via `buffer_history` on traj) | In diagram state vector `x` |
| **Sim loop** | Orchestrator tick @ `dt_base` → subset of `phi` calls | Full `StepEvaluator.step` on expanded diagram |

**Orchestrator — pros:** pure control-law blocks; logical diagram = mental model; complexity
in orchestrators ([AGENTS.md](../../AGENTS.md)); natural Phase A → B (same
`HybridSimulator`, add schedule + buffers); no graph-rewrite pass; path to event hybrid
(orchestrator already owns *when*).

**Orchestrator — cons:** hold state not in `x` unless exported; subset firing is a Python
(or staged) loop — weaker single JAX scan for whole controller; orchestrator must implement
cross-rate rules once (fast→slow sample, slow→fast ZOH).

**Expansion — pros:** one `StepEvaluator.step` @ `dt_base`; holds in `x` (plottable,
teachable); fixed `ExecutionPlan` — better JAX scan story.

**Expansion — cons:** graph rewrite complexity; diagram pollution (`_hold_*` subsystems);
hold/latch as fake dynamics; re-expand on topology/schedule change; feels heavy for simple
MPC + filter cascade.

**Decision:** Phase B **primary** = `ScheduledStepOrchestrator`. Document
`expand_scheduled_step()` as **optional lowering** for perf/JAX/teaching — not required for
MPC/SMC or cascade demos.

### Cascade multi-rate: orchestrator at `dt_base` (Phase B default)

**Do not simulate multiple clocks.** Simulate **one base tick**; the orchestrator decides
which controller blocks fire and what each input reads (live signal vs held buffer).

```text
User writes (logical topology, no holds):

  r → [ filter ] → [ MPC ] → u_cmd
        100 Hz      10 Hz

Orchestrator @ dt_base (ScheduledStepOrchestrator):

  tick k:
    if filter fires:  run filter.phi(...); buffer["filter:y"] ← output
    else:              use buffer["filter:y"]  (ZOH for downstream)
    if mpc fires:      run mpc.phi(..., buffer["filter:y"]); buffer["mpc:u"] ← u_cmd
    ...
  HybridSimulator: ZOH buffer["mpc:u"] → plant → sample y
```

**Three user-facing pieces:**

| Piece | Role |
| --- | --- |
| `StepDiagramSystem` | Wire cascade normally (`filter >> mpc`); blocks stay pure `phi` |
| `StepSchedule(dt_base, fire)` | Integer divisors per `sys_id`; lives on `HybridDiagram` / orchestrator |
| `ScheduledStepOrchestrator` | Buffers + fire mask @ `dt_base`; embedded in `HybridSimulator` for Phase B |
| `HybridSimulator` | Orchestrator tick + ZOH `u` → continuous plant (Phase A/B) |

**End-to-end API (Phase B):**

```python
controller = StepDiagramSystem()
controller.add_subsystem(ref_filter, "filter")
controller.add_subsystem(mpc_block, "mpc")
controller.add_input_port("r")
controller.connect("input", "r", "filter", "r")
controller.connect("filter", "y", "mpc", "r")
controller.connect_new_output_port("mpc", "u", "u")

schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "mpc": 10})

hybrid = HybridDiagram(
    step=controller,              # logical diagram — not expanded
    continuous=plant,
    schedule=schedule,
    connections=[...],
)
HybridSimulator(hybrid, ...).run()
```

**Cross-rate rules (orchestrator, v1):**

| Situation | Orchestrator behavior |
| --- | --- |
| Same rate | Both fire same ticks |
| Slow consumer ← fast producer | **Sample** producer output when slow block fires |
| Fast consumer ← slow producer | **ZOH** — hold last slow output in buffer |
| Block divisor `k > 1` | Run `phi` every k ticks; skip otherwise (buffer holds) |

**Debug:** orchestrator records `buffer_history` (or `HybridSimResult.buffers`) so internal
holds are plottable like diagram internal signals — without putting hold state in `x`.

**Defaults:**

- **`dt_base`** = fastest controller block period.
- **`fire[sys_id] = k`** = block fires every **k** base steps.
- Plant boundary: same ZOH + sample as Phase A.

**Optional lowering (expansion path):**

```python
sync = expand_scheduled_step(controller, dt_base=0.01, schedule={"filter": 1, "mpc": 10})
# → plain StepEvaluator.step @ dt_base; holds become diagram state
```

Use when a single fused JAX scan over the whole controller matters; not required for v1.

**Why this is not “full Simulink”:**

| Full Simulink multi-rate | This plan (Phase B) |
| --- | --- |
| Arbitrary flow + step in one canvas | Controller = step diagram; plant = flow diagram |
| Many opaque clock domains | **One `dt_base`** + explicit `StepSchedule` |
| FOH, delays, triggered subs, events | **ZOH + sample + integer divisors** only |
| General hybrid scheduler | Narrow **`HybridSimulator` + `ScheduledStepOrchestrator`** |

### What this is not (short term)

| Out of scope | Instead |
| --- | --- |
| One flat diagram with Integrator + MPC | Two sides + `HybridSimulator` |
| Controller and plant at unrelated clocks | Phase B `StepSchedule` + orchestrator |
| Event-driven switching (relay, impact) | Future layer 5 |
| Full Simulink hybrid library | ZOH + sample only |

```mermaid
flowchart LR
    subgraph phaseA [Phase A single Ts]
        C1[MPC or SMC block]
        P1[continuous plant]
        C1 -->|ZOH u| P1
        P1 -->|sample y| C1
    end

    subgraph phaseB [Phase B multi-rate controller]
        C2[logical step diagram + orchestrator]
        P2[continuous plant]
        C2 -->|ZOH u| P2
        P2 -->|sample y| C2
    end

    phaseA --> phaseB
```

---

## Four system kinds

Every block extends the same `System` shell (ports, params, `h`, visualization). Evolution
differs by kind:

| Kind | State | Core map(s) | Equation | Role |
| --- | --- | --- | --- | --- |
| **Static** | `n = 0` | `h` | `y = h(u, t; p)` | Algebra: gains, mux, saturation |
| **Flow** | `n > 0` | `f` | `dx = f(x, u, t; p)` | Plants, integrators, ODE controllers |
| **Step** | `n > 0` | `φ` (`phi`) | `x⁺ = φ(x, u, t; p)` | Digital control laws, games, jumps |
| **Hybrid** | `n > 0` | **`f` + `φ`** | flow between steps; step at instants | MPC+plant, switched systems, impacts |

```mermaid
flowchart TB
    subgraph leaves [Leaf maps]
        ST[StaticSystem h only]
        FL[DynamicSystem f flow]
        SP[StepSystem phi step]
    end

    subgraph diagrams [Homogeneous diagrams]
        D0[static wiring only]
        DF[FlowDiagramSystem]
        DS[StepDiagramSystem]
    end

    subgraph orch [Hybrid orchestration]
        O[scheduler when flow vs step]
    end

    ST --> D0
    FL --> DF
    SP --> DS
    D0 --> DF
    D0 --> DS
    DF --> O
    DS --> O
```

**Homogeneous diagram rule (v1):** flow diagrams contain only static + flow blocks; step
diagrams contain only static + step blocks. Do **not** mix `Integrator` and `StepSystem` in
one flat diagram without an orchestrator — heterogeneous evolution belongs in **`HybridDiagram`**
(layer 4) or a future event hybrid engine (layer 5).

---

## Architecture decision: sibling types, not a flag

Investigated options for the step core:

| Option | Verdict |
| --- | --- |
| Same `DynamicSystem` + `time_domain` flag on `f` | **Reject** — reverses meaning of `f`, breaks textbook clarity, confuses analysis tools |
| `StepSystem(DynamicSystem)` inheritance | **Reject** — same method name, opposite semantics; inherits RK4-facing APIs |
| **Sibling `StepSystem` + shared wiring mixin** | **Adopt** — matches `StaticSystem` / `DynamicSystem` pattern |
| Unified `DiagramSystem` with flag | **Reject for v1** — heterogeneous diagrams need orchestrator, not one compile path |

**Shared without duplication:** port wiring, params nesting, algebraic-loop detection,
`ExecutionPlan`, `PortOperation`, `StateOperation`. **Forked:** leaf type, evaluator public
API (`f` → `dx` vs `step` → `x⁺`), and advance loop (ODE integrator vs `x ← φ`).

Do **not** use `solver_info["continuous_time_equation"]` as the primary semantic switch —
evolution kind is **type-level** (`isinstance(sys, StepSystem)`). The existing simulator
hook may remain for facades only.

---

## Naming: `f` vs `phi`

| | Flow | Step |
| --- | --- | --- |
| **Math** | `dx = f(x, u, t; p)` | `x⁺ = φ(x, u, t; p)` or `x_{k+1} = φ(x_k, u_k, t_k; p)` |
| **Leaf method** | `f(...)` on `DynamicSystem` | **`phi(...)`** on `StepSystem` |
| **Evaluator** | `DynamicsEvaluator.f` → `dx` | **`StepEvaluator.step`** → `x⁺` (calls stored `phi`) |
| **Avoid** | — | **`g`** (gravity, costs, NLP constraints already use `g` in minilink) |

`phi` is the standard one-step **transition map** in dynamical-systems literature and pairs
cleanly with flow `f`. **`step`** is the orchestrator/evaluator verb, not the leaf equation
name.

Phase-1 **`StepSystem` has no clock on the class** — turn-based games use `StepRunner` with
`t` as move index. Sample periods for digital control live in **`StepSchedule`** (Phase B) or
outer `HybridSimulator` tick (Phase A).

---

## Hybrid orchestration

When flow and step combine, three pieces are required:

1. **Flow engine** — integrate `dx = f(...)` (existing `DynamicsEvaluator` + solvers).
2. **Step engine** — apply `x ← φ(...)` (`StepEvaluator.step`).
3. **Scheduler** — decides *when* to flow, *when* to step, and boundary I/O between sides.

The scheduler is **not** a fourth fundamental map. Clocks, guards, and triggers live in
orchestrators — not inside leaf `f` or `phi`.

### Trigger policies (what the scheduler implements)

| Policy | Trigger | Example | Engine (phase) |
| --- | --- | --- | --- |
| **Clock-driven** | every `dt_base` or tick `k` | MPC @ 10 Hz + plant | `HybridSimulator`, `TimedStepSimulator` |
| **Turn / index-driven** | caller advances one step | chess, RL `env.step` | `StepRunner` only (no `dt`) |
| **Event-driven** | guard `g(x,u,t)` during flow | impact, relay crossing | **Deferred** — flow + event locator + jump `φ` |
| **Mode-driven** | discrete mode `σ` selects `f_σ` | switched continuous plant | **Deferred** — mode in state or params + scheduler |
| **Enable / triggered** | enable signal | triggered subsystem | orchestrator skip-fire (future) |

```text
         ┌──────────────── orchestrator ────────────────┐
         │                                              │
  u ───► │  [step side φ] ──ZOH──► [flow side f] ──y──►│──► y_meas
         │       ▲                      │               │
         │       └──── sample y ────────┘               │
         └──────────────────────────────────────────────┘
```

### Orchestrator responsibilities

| Responsibility | Owner |
| --- | --- |
| Time / tick index | scheduler |
| Call integrate `f` on plant vs step/orchestrate controller | `HybridSimulator` |
| Cross-rate holds / latches | `ScheduledStepOrchestrator` buffers |
| Boundary: ZOH, sample, port mapping | `HybridDiagram` + scheduler |
| Homogeneous compile per side | `compile()` vs `compile_step()` |
| Guards / zero-crossings | future `EventHybrid` (layer 5) |

### v1 hybrid cycle (clock-driven, pseudocode)

**Phase A** (`schedule is None` — all blocks fire):

```python
x_flow, x_step = x_flow_0, x_step_0
t = t0
while t < tf:
    u_hold = step_evaluator.outputs(x_step, u_ext, t)["u"]
    x_step = step_evaluator.step(x_step, u_ext, t)
    x_flow = cont_evaluator.rk4_rollout_zoh(x_flow, u_hold, t, dt)
    y_meas = cont_evaluator.outputs(x_flow, u_hold, t + dt)["y"]
    u_ext = sample_for_step_side(y_meas)
    t += dt
```

**Phase B** — replace step side with orchestrator tick:

```python
while t < tf:
    x_step, u_hold, _bufs = orchestrator.tick(x_step, u_ext, t, k)
    x_flow = cont_evaluator.rk4_rollout_zoh(x_flow, u_hold, t, dt_base)
    ...
    t += dt_base
    k += 1
```

Static blocks remain **inside** each diagram; the orchestrator sees only boundary ports.

---

## Future: switched and full hybrid systems

Path from this plan to hybrid automata / switched systems **without changing the leaf core**:

| Level | Description | Uses |
| --- | --- | --- |
| **A — Compositional** | Two-side MPC + plant | `HybridSimulator` (this plan) |
| **B — Switched flow** | `f_σ(x,u)`, mode `σ` | `DynamicSystem` + mode in state/params; scheduler picks `σ` |
| **C — Full hybrid** | flow + guard + jump | integrate `f` until guard; then `x ← φ_jump` |

```mermaid
flowchart TB
    subgraph primitives [Leaf primitives — build now]
        DS[DynamicSystem flow f]
        SS[StepSystem step phi]
        ST[StaticSystem]
    end

    subgraph schedulers [Orchestrators — phased]
        SR[StepRunner clock-free]
        TS[TimedStepSimulator dt grid]
        SO[ScheduledStepOrchestrator multi-rate]
        H1[HybridSimulator two-side]
        H2[EventHybrid guards jumps — future]
    end

    DS --> H1
    SS --> H1
    SS --> SR --> TS
    SS --> SO --> H1
    H1 --> H2
    DS --> H2
```

**Do not build toward full hybrid by:** merging flow and step into one flagged `f`, or mixing
heterogeneous blocks in one diagram class before event scheduling exists.

---

## Recommendation

**Build two leaf maps (`f` and `phi`), homogeneous diagrams, then hybrid as thin glue.**

- Each layer is testable in isolation (step closed loops before touching the plant).
- Reuses minilink's pattern: `System` → diagram → compile → evaluator → runner/simulator.
- Hybrid schedules two mature pipelines; no fused `ExecutionPlan` mixing `dx` and `x⁺`.

| Layer | Delivers | Does not deliver |
| --- | --- | --- |
| **1** | `StepSystem`, `ZOHHold` (math: `phi` only) | Clock, diagrams, simulation |
| **2a** | `StepDiagramSystem` — logical topology, compile | Multi-rate scheduling |
| **2b** | compile + `StepRunner` + `TimedStepSimulator` | Multi-rate orchestrator |
| **2c** | `StepSchedule` + `ScheduledStepOrchestrator` | Non-integer rate ratios |
| **3a** | **Phase A:** `HybridSimulator`, `MPCBlock`, `SMCBlock`, one `Ts` | Multi-rate controller |
| **3b** | **Phase B:** cascade via orchestrator @ `dt_base` | Expansion lowering (optional) |
| **4–5** | (future) events, guards, switched modes; optional `expand_scheduled_step` | — |

**Implementation gates:** Layer 3a (single-rate hybrid) after step compile works. Phase B
orchestrator does **not** block MPC/SMC. Expansion lowering is optional post-Phase B.

---

## Layer overview

```mermaid
flowchart TB
    subgraph layer1 [Layer 1 StepSystem]
        SS[StepSystem leaf phi]
        ZOH[ZOHHold block]
    end

    subgraph layer2 [Layer 2 Step diagram pipeline]
        SDS[StepDiagramSystem]
        SC[compile_step]
        SE[StepEvaluator]
        SR[StepRunner / TimedStepSimulator]
        SO[ScheduledStepOrchestrator]
        SDS --> SC --> SE --> SR
        SE --> SO
    end

    subgraph layer3 [Layer 3 Hybrid two-side]
        HD[HybridDiagram + StepSchedule]
        HS[HybridSimulator]
        CE[DynamicsEvaluator rk4_rollout_zoh]
        HD --> HS
        HS --> SO
        HS --> CE
    end

    layer1 --> layer2
    layer2 --> layer3
```

---

## Clock architecture

**Phase B rule:** user never hand-wires holds in the logical diagram. Multi-rate is
**`StepSchedule` + `ScheduledStepOrchestrator`** (see
[Multi-rate Phase B](#multi-rate-phase-b-orchestrator-vs-expansion)). Optional
`expand_scheduled_step()` lowers to sync diagram state — not the default.

| Question | Answer |
| --- | --- |
| Reuse continuous wiring for step diagrams? | **Yes** — sibling `StepDiagramSystem` + shared mixin / `ExecutionPlan` |
| Where does clock live? | **`StepSchedule.dt_base`** + `fire` divisors; not on leaf `StepSystem` |
| Where do holds/latches live? | **Orchestrator buffers** (default); or diagram state if expanded |
| Second diagram topology class? | **Yes** — `StepDiagramSystem` sibling of `DiagramSystem` |
| Multiple sim loops for cascade? | **No** — one orchestrator tick @ `dt_base` |
| Public multi-rate API | **`HybridDiagram.schedule`** or orchestrator arg — not on `TimedStepSimulator` alone |

### Phase A: single-rate (no schedule object)

```python
hybrid = HybridDiagram(step=controller, continuous=plant, connections=[...], dt_base=Ts)
HybridSimulator(hybrid, ...).run()   # every tick: all blocks fire
```

### Phase B: logical diagram + schedule (default)

```python
controller = StepDiagramSystem()
controller.add_subsystem(ref_filter, "filter")
controller.add_subsystem(mpc_block, "mpc")
controller.connect("input", "r", "filter", "r")
controller.connect("filter", "y", "mpc", "r")

schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "mpc": 10})

hybrid = HybridDiagram(step=controller, continuous=plant, schedule=schedule, connections=[...])
HybridSimulator(hybrid, ...).run()
```

### Optional: expansion lowering

```python
sync = expand_scheduled_step(controller, dt_base=0.01, schedule={"filter": 1, "mpc": 10})
hybrid = HybridDiagram(step=sync, continuous=plant, connections=[...], dt_base=0.01)
# StepEvaluator.step on expanded diagram — holds in x, no orchestrator buffers
```

**Rate metadata on blocks:** optional `params["sample_period"]` or class doc hint for
documentation; **authoritative** divisors live in `StepSchedule.fire`. Leaf `phi` stays
pure — mirrors continuous minilink: **`phi` is math; time grid lives in the orchestrator.**

### Optional expansion blocks (lowering only)

If `expand_scheduled_step()` is implemented later:

| Block | Role when expanded |
| --- | --- |
| `RateGate` | Wrap slow subsystem; fire inner `phi` every k ticks |
| `HoldRegister` | ZOH on slow→fast edges |
| `SampleLatch` | Sample on fast→slow edges |

Not required for Phase B orchestrator path.

---

## Context: minilink today

| Piece | Today | This plan |
| --- | --- | --- |
| Static leaf | `StaticSystem`: `y = h(u)` | Unchanged |
| Flow leaf | `DynamicSystem`: `dx = f(...)` | Unchanged |
| Flow diagram | `DiagramSystem` + `compile()` → `DynamicsEvaluator` | Unchanged |
| Step | Out of scope in DESIGN §3 | Layers 1–3 (subset) |
| MPC demos | Manual Python loop × 7 | Layer 3 via `MPCBlock` + `HybridSimulator` |
| JAX plant rollout | `rk4_rollout_ivp` scan | add `rk4_rollout_zoh` for hybrid inner loop |
| `game()` loop | Euler on `f` | Later: branch for `StepSystem` → `x = phi(x,u,t)` |

---

## Layer 1: `StepSystem`

**Files:** `minilink/core/system.py`, `minilink/blocks/`

```python
class StepSystem(System):
    """
    x⁺ = φ(x, u, t; p)
    y  = h(x, u, t; p)
    """
    def phi(self, x, u, t=0, params=None):
        ...
```

- Same port/params surface as `DynamicSystem` (`u`, `y`, optional `x` ports).
- **`phi` returns next state**, not derivative; do **not** overload `f` on this class.
- `solver_info`: `continuous_time_equation=False`.
- `dt` or sample time inside `phi` via `params` or wiring only when the **model** needs it —
  not a class-level clock.
- Parallel continuous work: **`rk4_rollout_zoh`** on flow evaluator (JAX scan) for Layer 3.

---

## Layer 2: Step diagram pipeline

### Shared wiring extraction

**File:** `minilink/core/wiring.py` (or `WiredDiagramBase` mixin) — used by both
`DiagramSystem` and `StepDiagramSystem`:

- `connect`, `get_local_input`, params nesting, `state_index`, boundary outputs.

### `StepDiagramSystem`

**File:** `minilink/core/step_diagram.py` — **sibling** of `DiagramSystem`.

- **`x⁺ = φ(x, u, t)`** — same stacked loop as flow `DiagramSystem.f`; docstring and method
  name differ (`phi` or diagram-level `step_state`).
- Evaluator exposes **`step(x, u, t)`** calling each block's `phi`.
- Subsystems: `StepSystem`, `StaticSystem`, `ZOHHold`. Reject `DynamicSystem` at compile.
- Reuse `build_execution_plan()` and `ExecutionPlan` — do not fork.

| Flow | Step |
| --- | --- |
| `compile_diagram()` → `DynamicsEvaluator` | `compile_step_diagram()` → `StepEvaluator` |
| `evaluator.f → dx` | `evaluator.step → x⁺` |
| RK4, rollout, SciPy | stepping only |

Mark step compile path **`TODO: User Architectural Review`** until closed-loop tests pass.

### `StepRunner` and `TimedStepSimulator`

**Files:** `minilink/simulation/step_runner.py`, `minilink/simulation/timed_step_simulator.py`

**`StepRunner`** — clock-free; caller owns `t` sequence (turn-based, generic stepping):

```python
def run_steps(evaluator, x0, u_sequence, t_sequence) -> StepResult
```

**`TimedStepSimulator`** — uniform grid wrapper (digital control):

```python
@dataclass
class TimedStepOptions:
    tf: float
    sync_dt: float

def simulate_steps(diagram, x0, *, options) -> StepResult
```

Multi-rate: `ScheduledStepOrchestrator` when `schedule is not None`; single-rate uses plain
`StepEvaluator.step` or orchestrator with all divisors `1`.

### Sibling diagram + shared core (not subclass)

| Layer | Shared? |
| --- | --- |
| Port wiring | **Yes** — `core/wiring.py` mixin |
| Diagram state map | **Yes** — same loop; `dx` vs `x⁺` docstring |
| `ExecutionPlan` | **Yes** |
| Leaf type | **Sibling** — `DynamicSystem` vs `StepSystem` |
| Evaluator | **Sibling** — `DynamicsEvaluator` vs `StepEvaluator` |
| Runner / simulator | **Different** — ODE vs `x ← φ` |

---

## Layer 3: Hybrid — sampled controller + ZOH + continuous plant

The **only** v1 hybrid structure (see [Short-term use cases](#short-term-use-cases-what-we-are-building-for)):

```
[ StepDiagramSystem ]  --ZOH on u-->  [ DiagramSystem plant ]
         ^                                      |
         +----------- sample y -------------------+
```

Phase A: one controller block at sample period `Ts`. Phase B: logical step diagram +
`StepSchedule` + `ScheduledStepOrchestrator` @ `dt_base`, same plant boundary.

### `HybridDiagram` — `minilink/core/hybrid_diagram.py`

```python
@dataclass
class StepSchedule:
    dt_base: float
    fire: dict[str, int]   # sys_id -> tick divisor; default 1 for every block

@dataclass
class HybridDiagram:
    step: StepDiagramSystem          # logical diagram (not expanded by default)
    continuous: DiagramSystem
    connections: list[BoundaryConnection]
    dt_base: float                   # Phase A outer tick (= Ts); Phase B = schedule.dt_base
    schedule: StepSchedule | None = None   # None => all blocks fire every tick (Phase A)
```

### `ScheduledStepOrchestrator` — `minilink/simulation/scheduled_step.py`

Runs a **logical** compiled step diagram @ `dt_base`:

```python
class ScheduledStepOrchestrator:
    def __init__(self, diagram, schedule: StepSchedule, *, compile_backend="numpy"): ...

    def tick(self, x_step, u_ext, t, k) -> tuple[x_step, u_plant, BufferSnapshot]: ...
    """Fire blocks per schedule.fire; update hold/latch buffers; return plant input."""

    def buffer_history(self) -> dict[str, np.ndarray]: ...  # debug / plot internal holds
```

Uses compiled port wiring and per-block `phi` callables; **does not** require expanded
diagram. Embedded by `HybridSimulator` when `schedule is not None`.

### `HybridSimulator` — `minilink/simulation/hybrid_simulator.py`

**Phase A** — `schedule is None`; every block fires each tick:

Per controller tick:

1. Step side — `step_evaluator.step` (all blocks) or orchestrator with all `fire=1`.
2. Boundary — hold controller output `u` (ZOH) for the plant over `[t, t + Ts]`.
3. Continuous — `cont_evaluator.rk4_rollout_zoh(x, u_hold, t, Ts)` (optional inner substeps
   `dt_plant << Ts`).
4. Feedback — sample plant outputs `y` at tick boundary → controller inputs.

**Phase B** — `ScheduledStepOrchestrator.tick` @ `dt_base`; plant path unchanged from Phase A.

Python outer (step + MPC/SMC solvers) / JAX inner (plant rollout).

### `MPCBlock` — `minilink/planning/mpc/`

- `StepSystem` wrapping `MPCPlanner.step`.
- Warm-start state in `__init__`; no `core/compile/` imports.
- `sys_mpc` vs `sys_sim` stays explicit in demos.
- Primary **Phase A** reference controller.

### Sampled sliding-mode controller (`SMCBlock` or pattern)

- **`StepSystem`** implementing discrete-time sliding-mode law (or equivalent sampled SMC).
- Lives on step side of `HybridDiagram`; **same ZOH boundary** as MPC.
- May live in `minilink/control/` as factory + block; no `core/compile/` imports.
- **Phase A** deliverable alongside MPC — proves hybrid sim is not MPC-specific.

### Demo targets

| Phase | Demo | Notes |
| --- | --- | --- |
| **A** | Refactor `examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py` | Preserve user tuning |
| **A** | New or existing SMC + plant demo (e.g. bicycle or pendulum) | Same `HybridSimulator` API |
| **B** | Cascade: filter @ fast rate + MPC @ slow rate | `StepSchedule` + orchestrator |

---

## Scope

### In scope

- Layers 1–3 as above.
- **Phase A:** single-rate MPC and SMC (sampled controller + ZOH + continuous plant).
- **Phase B:** multi-block controller at integer multiples of `dt_base`.
- DESIGN / ROADMAP / README sync on implementation.

### Deferred

| Item | Why |
| --- | --- |
| Flow + step in one flat diagram | v1 uses two sides + orchestrator |
| Non-integer divisor ratios | resampling / drift |
| Evaluator hold-buffer scheduler (public) | orchestrator is the public contract |
| `expand_scheduled_step` / infra hold blocks | optional lowering; post-Phase B if needed |
| FOH, delay, events, full Simulink multi-rate | ZOH subset only |
| `EventHybrid` (guards, zero-crossing, jump maps) | layer 5; needs event engine |
| Switched-mode flow without events | mode in state; scheduler later |
| `linearize` / Bode on native `StepSystem` | Jacobian ∂φ/∂x is discrete A; separate pass |

---

## Tests (per layer)

| Layer | File | Cases |
| --- | --- | --- |
| 1 | `test_step_system.py` | leaf `phi`; `ZOHHold`; `solver_info` |
| 1 | `test_rk4_rollout_zoh.py` | NumPy/JAX vs repeated `rk4_step` |
| 2 | `test_step_diagram.py` | wiring; `@` closed loop |
| 2 | `test_scheduled_step_orchestrator.py` | Phase B: fire mask, ZOH/sample buffers, cascade |
| 2 | `test_step_runner.py` | clock-free stepping |
| 2 | `test_timed_step_simulator.py` | sync closed loop; single-rate step diagram |
| 3 | `test_hybrid_simulator.py` | Phase A: ZOH → plant → sample y; MPC and SMC smoke |
| 3 | `test_mpc_block.py` | MPCBlock closed loop |
| 3 | `test_smc_hybrid.py` | SMC + plant via same HybridSimulator (Phase A) |
| 2–3 | `test_hybrid_cascade.py` | Phase B: filter fast + MPC slow via orchestrator |
| 2 | `test_expand_scheduled_step.py` | optional: expansion lowering parity vs orchestrator |

---

## Implementation order

| Step | Layer | Deliverable | Phase |
| --- | --- | --- | --- |
| 1 | 1 | `StepSystem`, `ZOHHold`, leaf tests | — |
| 2 | 1 | `rk4_rollout_zoh` | — |
| 3 | 2 | shared diagram wiring mixin (`core/wiring.py`) | — |
| 4 | 2 | `StepDiagramSystem` | — |
| 5 | 2 | `compile_step_diagram` + `StepEvaluator` | — |
| 6 | 2 | `StepRunner` + `TimedStepSimulator` + closed-loop tests | — |
| 7 | 3 | `HybridDiagram` + `HybridSimulator` (ZOH + sample) | **A** |
| 8 | 3 | `MPCBlock` + straight-line MPC demo | **A** |
| 9 | 3 | `SMCBlock` (or pattern) + SMC hybrid demo | **A** |
| 10 | 2 | `StepSchedule` + `ScheduledStepOrchestrator` + tests | **B** |
| 11 | 3 | Cascade controller hybrid demo | **B** |
| 12 | 2 | *(optional)* `expand_scheduled_step` + hold blocks | post-B |
| 13 | all | DESIGN / ROADMAP / README | — |

**Gate:** Phase A hybrid (steps 7–9) after step compile + `rk4_rollout_zoh`. Phase B
(steps 10–11) after Phase A hybrid tests pass. Expansion (step 12) optional — do not block
MPC/SMC or cascade on it.

---

## AGENTS.md alignment (summary)

- Textbook math: `dx = f(...)`, `x⁺ = φ(...)`; bare `f` / `phi` / `h` on equation paths.
- Complexity in orchestrators + simulators; thin leaf types (pure `phi`).
- Reuse port gather / `ExecutionPlan`; no blind fork.
- `MPCBlock` in `planning/mpc/`; warm-start in `__init__`.
- Preserve user demo tuning; one demo refactor only.
- Pre-push: `ruff check .`, `ruff format --check .`, proportionate `pytest`.

---

## Expected outcome

- Step blocks compose like flow systems (diagram + ports).
- Turn-based and clock-driven both use `StepSystem`; only the orchestrator differs.
- Multi-rate cascade: **logical diagram + `StepSchedule` + orchestrator @ `dt_base`**
  (holds in orchestrator buffers; optional expansion lowering later).
- Hybrid v1: thin scheduler over step `φ` + `rk4_rollout_zoh` plant — **MPC and SMC** at one
  `Ts` (Phase A), then cascade controllers on `dt_base` (Phase B).
- MPC and SMC demos drop hand-rolled outer loops; same `HybridSimulator` API.
- Future switched/hybrid: same `f` and `phi` atoms + event scheduler — no leaf rewrite.
