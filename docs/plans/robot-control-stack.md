# Robot control stack and closed-loop wiring

Status: **implementation-ready** (June 2026). Detailed build spec below; execute phases in order.

Consolidates architecture discussion for plant port tiers, closed-loop shortcuts,
controller generalization, and robot control band. Updates
[manipulator-abstraction.md](manipulator-abstraction.md) on execution.

**Pyro backlog:** [pyro-port-remaining.md](pyro-port-remaining.md) — robot control
blocks unblock ~50 pendulum/arm demos.

---

## Goals

1. Finish **robot band**: catalog on `Manipulator`, joint/task impedance, then computed torque / sliding mode.
2. **Standardize closed-loop wiring** without layout metadata types.
3. **Generalize controller family** (vector DOF, velocity reference, filtered MIMO).
4. Stay **future-proof** for augmented state via `x2q` + wiring shortcuts.

---

## Explicit non-goals

| Idea | Verdict |
| --- | --- |
| Stacked `[q; dq]` port type on plant | **No** |
| Layout metadata on plants | **No** |
| Controller inheritance tree | **No** |
| `r`/`dr` on plant (Pyro) | **No** — plant `p`/`pdot`; controller `r` |
| Rename `MechanicalSystem` / `Manipulator` | **No** — dynamics names stay; **impedance profile is domain-neutral** |

---

## Plant hierarchy (dynamics — not the control profile name)

```text
DynamicSystem
    └── MechanicalSystem     #  EoM in q;  view ports q, dq
            └── Manipulator  #  + task ports p, pdot
```

`MechanicalSystem` is a **dynamics shelf** (EoM structure), not the name of the
impedance feedback profile. The same **`impedance`** profile applies anywhere a
plant exposes configuration + rate views — arms (`q`,`dq`), pendulum (`y` stack),
later vehicles (`pose`,`bodyvel` in vehicle-abstraction doc).

| Port | Role |
| --- | --- |
| `x` | Full state |
| `y` | Primary output — **not assumed `[config; rate]`** |
| `q`, `dq` | Generalized configuration + velocity views (`x2q`) |
| `p`, `pdot` | Task views (manipulator only) |
| `r` | Controller reference only |

---

## Closed-loop wiring

### `autowire()` — `y` only, never Mux

- `controller.y` ← `plant.y` when dims match.
- Never insert Mux.

### `closed_loop(..., feedback=...)`

Keyword values — **short, no underscores**:

```python
closed_loop(controller, plant, feedback="auto")   # default
# feedback="y"    plant.y → controller.y
# feedback="qdq"  Mux(plant.q, plant.dq) → controller.y
```

**`feedback="auto"` order:** `y` match → Mux `qdq` → `x` state → `ValueError`.

Optional alias: `closed_loop_qdq(...)` = `feedback="qdq"` (teaching diagrams).

Future task-space: `feedback="ppdot"` when `TaskImpedance` lands.

### Cheat sheet

| Situation | Wiring |
| --- | --- |
| Pendulum `@` | `plant.y → controller.y` |
| `(Step + Impedance + plant).autowire()` | `plant.y → controller.y` |
| Teach split config/rate ports | `closed_loop(..., feedback="qdq")` |
| Augmented `x` | `auto` → Mux `qdq` |
| LQR | `plant.x → controller.x` |

---

## Feedback profiles (organize without inheritance)

Five profile ids — **short, no underscores** — one module each, plus optional
`feedback_profile` class attribute (string tag, not a base class).

| Profile | Module | Blocks | Measurement |
| --- | --- | --- | --- |
| `output` | `control/output.py` | `ProportionalController` | `y` (static; may be coupled MIMO) |
| `impedance` | `control/impedance.py` | `ImpedanceController`, `ImpedanceIntegralController` | `y = [pos; rate]` dim `2n` |
| `state` | `control/state.py` | `StateFeedbackController` | `x` |
| `siso` | `control/siso.py` | `FilteredController`, future PI / lead-lag / … | `y` dim `n` only — **one decoupled loop per channel** |
| `task` | `control/robotic.py` | `TaskImpedance`, … | Mux `p`,`pdot` + `q` for `J` |

**Model-based mechanical laws** (inverse dynamics, sliding surfaces) live in **`control/modelbased.py`** — not a sixth `closed_loop` profile; optional class tag `feedback_profile = "modelbased"`. See [Mechanical model-based band](#mechanical-model-based-band-controlmodelbasedpy) below.

### `siso` vs `impedance` (split by wiring, not by “PID”)

| | **`impedance`** | **`siso`** |
| --- | --- | --- |
| Plant gives rate? | **Yes** — explicit on `y` or Mux `qdq` | **No** — config/position only |
| D-term source | Measured rate from plant | Internal (filtered derivative, etc.) |
| Coupling | Virtual spring-damper on stacked measurement | **Independent** loops per scalar channel |
| Multi-axis | Vector `Kp`, `Kd` on one `[pos; rate]` stack | `dof=n` = **n parallel SISO loops** (state `2n`, diagonal gains) |
| Examples | Pendulum/arm with Mux or stacked `y` | Filtered PID, speed loop on scalar `u`, future lead-lag |

`ImpedanceIntegralController` (ex-`PIDController` with explicit rate) stays under **`impedance`**, not `siso` — the plant must wire `[pos; rate]`.

`ProportionalController` stays under **`output`** — static output feedback; matrix `K` may couple channels (not decoupled SISO).

Open-loop **`blocks/transfer_function.py`** stays in `blocks/` — port pattern `u → y`, not closed-loop `r, y → u`. Future SISO compensators **as controllers** land in `siso.py`.

Start with a single module **`control/siso.py`**; promote to `control/siso/` subpackage only if the catalog grows large.

```python
class ImpedanceController(StaticSystem):
    """Virtual spring-damper on [position; rate]: u = Kp e_p + Kd e_v."""

    feedback_profile = "impedance"


class FilteredController(DynamicSystem):
    """Decoupled SISO PID with filtered derivative and anti-windup."""

    feedback_profile = "siso"
```

### Rename map (pre-1.0, fix all call sites)

| Today | Target |
| --- | --- |
| `PDController` | `ImpedanceController` |
| `PIDController` | `ImpedanceIntegralController` |
| `LinearStateFeedbackController` | `StateFeedbackController` |
| `FilteredPIDController` | `FilteredController` |
| `control/linear.py` | split → `output.py`, `impedance.py`, `state.py` |
| `control/pid.py` | `control/siso.py` |

Target `control/` layout:

```text
control/
  output.py       # ProportionalController
  impedance.py    # ImpedanceController, ImpedanceIntegralController
  state.py        # StateFeedbackController
  siso.py         # FilteredController, …
  robotic.py      # JointImpedance, TaskImpedance  (manipulator wrappers)
  modelbased.py   # ComputedTorqueController, SlidingModeController
  lqr.py          # synthesize_lqr, TrajectoryLQRController
```

### Manipulator band (`control/robotic.py`)

Pyro **`robotcontrollers.py`** — manipulator-specific impedance **wrappers** only (need `Manipulator` ports / Jacobian):

| Block | Profile | Phase | Notes |
| --- | --- | --- | --- |
| `JointImpedance` | `impedance` | 3 | Factory over `ImpedanceController(dof=n)` |
| `TaskImpedance` | `task` | 3 | Mux `p`,`pdot`; extra `q` for `J^T` |

### Mechanical model-based band (`control/modelbased.py`)

Pyro **`nonlinear.py`** — laws that call the plant **EoM** (`H`, `C`, `g` / inverse dynamics). Any
`MechanicalSystem` or `Manipulator` (pendulum, cartpole, arms — not vehicle-flat plants).

| Block | Phase | Notes |
| --- | --- | --- |
| `ComputedTorqueController` | 4 | `τ = H(q)v + C(q,dq)dq + g(q)` |
| `SlidingModeController` | 4 | joint or task sliding surface |

```python
class ComputedTorqueController(DynamicSystem):
    """Inverse-dynamics tracking on a mechanical plant."""

    feedback_profile = "modelbased"  # optional tag; not used by closed_loop in v1
```

**Naming choice (vs `nonlinear.py` / `MechanicalModelBased`):**

| Option | Verdict |
| --- | --- |
| `nonlinear.py` | Pyro filename parity, but **too vague** — LQR on a nonlinear plant is also “nonlinear” |
| `control/mechanical.py` | **Reject** — collides with `dynamics/abstraction/mechanical.py` |
| `MechanicalModelBased` (class/module) | Right **concept**; module file **`modelbased.py`** (short, no underscores in id) |
| Keep CT/SMC in `robotic.py` | **Reject** — pendulum/cartpole demos aren’t “robotic” |

Split **`robotic`** (manipulator wrappers) vs **`modelbased`** (EoM-driven laws) from the start.

**Reference `r`:** dim `n` = regulation (zero rate ref); dim `2n` = stacked `[pos_d; vel_d]`.

---

## Controller catalog — where everything lives

Full inventory of **library feedback blocks** (Pyro port + Minilink today + planned).
**Not** in `control/`: planning policies, sources, learned blocks, open-loop TF plants.

### `control/` modules (by profile)

| Pyro / today | Target class | Module | Profile | Ports / wiring |
| --- | --- | --- | --- | --- |
| `ProportionalController`, `ProportionalGain` | `ProportionalController` | `output.py` | `output` | `r`, `y` → `u`; static; matrix `K` may couple |
| `PDController` (pyro linear) | `ImpedanceController` | `impedance.py` | `impedance` | `r`, `y=[pos;rate]` → `u`; ex scalar PD |
| `PIDController` on `[y, dy_dt]` (minilink linear) | `ImpedanceIntegralController` | `impedance.py` | `impedance` | `r`, `y` dim `2n` → `u`; integral state |
| `LinearStateFeedbackController` | `StateFeedbackController` | `state.py` | `state` | `x`, `r` → `u`; LQR target form |
| `FilteredPIDController` (pyro/minilink pid) | `FilteredController` | `siso.py` | `siso` | `r`, `y` dim `n` → `u`; filter + anti-windup |
| *(future)* PI, lead-lag, per-axis velocity PID | TBD | `siso.py` | `siso` | same decoupled `y`-only contract |
| `JointPD` / joint impedance demos | `JointImpedance` | `robotic.py` | `impedance` | factory → `ImpedanceController(dof=n)` + `qdq` wiring |
| `EndEffectorPD` / task impedance demos | `TaskImpedance` | `robotic.py` | `task` | Mux `p`,`pdot` → `y`; extra `q` for `J^T` |
| `ComputedTorqueController` | `ComputedTorqueController` | `modelbased.py` | `modelbased` | `q`, `dq`, ref accel; plant EoM |
| `SlidingModeController` | `SlidingModeController` | `modelbased.py` | `modelbased` | joint or task sliding surface |

### `control/lqr.py` — design factories (return `state` blocks)

| Pyro / today | Target | Notes |
| --- | --- | --- |
| `synthesize_lqr_controller` | `synthesize_lqr()` | Returns `StateFeedbackController` |
| `linearize_and_synthesize_lqr_controller` | `lqr_at_operating_point()` | Same return type |
| `TrajectoryLQRController` | `TrajectoryLQRController` | **TODO** — time-varying `K(t)` or scheduled gains; still profile `state`, lives in `lqr.py` |

### Outside `control/` (still closed-loop capable, different bands)

| Pyro / today | Minilink home | Role |
| --- | --- | --- |
| `OpenLoopController` | `blocks/sources.py` → `TrajectorySource` | Feedforward reference trajectory, not feedback |
| `LookUpTableController` | `planning/policy_synthesis/lookup_policy.py` | DP / grid policy lookup |
| `stable_baseline3_controller` | `interfaces/` (stub) | RL wrapper — train outside library |
| `NeuralNetwork` | `blocks/neural.py` | Learned `u = f(r, y)` or custom inputs; not a profile-tagged catalog block |
| `TransferFunction` | `blocks/transfer_function.py` | Open-loop LTI `u → y` (plant or compensator segment), not `r,y,u` controller |
| `StaticController` / `ClosedLoopSystem` | `core/composition.py` | Wiring (`@`, `closed_loop`, `autowire`) — not a law |

### Demo-local loops (promote when reused)

These live inline in examples today; if promoted to library:

| Example block | Likely home | Profile |
| --- | --- | --- |
| `VelocityPID` (bicycle demo) | `siso.py` | `siso` — scalar speed on `y`, internal integral + filter |
| `HeadingLoop`, `YawRateLoop` (cascade) | `siso.py` or `robotic.py` | `output` / `siso` — P on heading; PD on yaw rate depending on ports |

Cascade path tracking stays a **diagram composition** of small loops + sources, not one monolithic controller class.

### Quick decision tree (pick a module)

```text
Needs full plant state x?     → state.py  (+ lqr.py factory)
Plant gives [pos; rate] on y? → impedance.py  (or robotic.py wrapper)
Plant gives y only, per-axis? → siso.py
Uses H(q), C, g from plant?   → modelbased.py (ComputedTorque, SlidingMode)
Manipulator task / Jacobian?  → robotic.py (TaskImpedance)
Static K on output error?     → output.py
```

---

## Implementation phases

| Phase | Work |
| --- | --- |
| **1** | DESIGN §4 profiles + wiring; `closed_loop(..., feedback=auto/y/qdq)` |
| **2** | Split `control/`; impedance/siso renames; `feedback_profile` tags; vector DOF + integral + `FilteredController(dof=n)` |
| **3** | Catalog arms rebase; `robotic.py`: `JointImpedance`, `TaskImpedance` demo |
| **4** | `modelbased.py`: `ComputedTorqueController`, `SlidingModeController` (blocked on EoM hooks) |
| **5** | `vehicle-abstraction.md` — same `impedance` profile, vehicle-specific view port ids |

---

## Decision log

| Question | Decision |
| --- | --- |
| PD vs Impedance naming? | **`ImpedanceController`** — virtual spring-damper law on `[pos; rate]` |
| Module for ex-PD? | **`impedance.py`** (not `second_order.py`) |
| Filtered PID home? | **`siso.py`** — profile **`siso`**, not `filtered.py` (future-proof for all decoupled TF/PID loops) |
| Profile id? | **`impedance`** — domain-neutral (not `mechanical`); **`siso`** for per-channel dynamic loops |
| Underscores? | Avoid in **profile ids** and **`feedback=` values** (`qdq`, not `q_dq`) |
| `MechanicalSystem` name? | **Keep** — dynamics class; impedance profile is separate |
| CT / SMC module? | **`modelbased.py`** — mechanical EoM laws (Pyro `nonlinear.py`); **`robotic.py`** = manipulator wrappers only |
| `nonlinear.py` name? | **Not used** — too vague; prefer **`modelbased`** |
| `control/mechanical.py`? | **No** — collides with dynamics `mechanical.py` |
| Autowire? | **`plant.y`** only; never Mux |

---

## Detailed build specification

Execute in order. Full task breakdown is also in the Cursor plan **Robot control stack build**.

### Phase 1 — Wiring (no renames yet)

- [`minilink/core/composition.py`](../../minilink/core/composition.py): add `feedback="auto"|"y"|"qdq"` to `closed_loop`; add `closed_loop_qdq()` alias; `autowire` unchanged.
- Tests: [`tests/unittest/test_composition.py`](../../tests/unittest/test_composition.py) — pendulum `@` still uses `y`; `qdq` inserts `Mux`; augmented plant auto-Mux.
- Docs: DESIGN §4 profiles; manipulator-abstraction wiring section.

### Phase 2 — Control split (large import churn)

- Create `output.py`, `impedance.py`, `state.py`, `siso.py`; delete `linear.py`, `pid.py`.
- Renames: `ImpedanceController`, `ImpedanceIntegralController`, `StateFeedbackController`, `FilteredController`.
- Vector `dof=n`; `r` dim `n` (regulation) vs `2n` (tracking); `FilteredController(dof=n)` diagonal MIMO.
- Fix all call sites (~35): tests, examples, notebooks, README, DESIGN, `lqr.py`.
- Update tests: subsystem ids change (`pd_controller` → `impedance_controller`).

### Phase 3 — Manipulator band

- [`arms.py`](../../minilink/dynamics/catalog/manipulators/arms.py): subclass `Manipulator`; `forward_kinematic_effector` → `forward_kinematics`; verify `J`, `p`, `pdot`.
- [`control/robotic.py`](../../minilink/control/robotic.py): `JointImpedance`, `TaskImpedance`; two-link demo.

### Phase 4 — Model-based

- [`control/modelbased.py`](../../minilink/control/modelbased.py): `ComputedTorqueController`, `SlidingModeController` (uses `inverse_dynamics` / EoM hooks).
- Pendulum or one-link CT demo + tests.

### Phase 5 — Vehicle design doc only

- [`vehicle-abstraction.md`](vehicle-abstraction.md): pose/bodyvel views; no code.

### Verification

```bash
conda activate minilink
pytest tests/unittest/test_composition.py tests/unittest/test_control*.py tests/unittest/test_catalog_migration.py -q
rg 'PDController|control\.linear|control\.pid' --glob '!docs/plans/*'  # zero after Phase 2
```
