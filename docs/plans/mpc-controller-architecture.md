# MPC / receding-horizon architecture — requirements & brainstorm

Status: **brainstorm / requirements draft** (July 2026). **Not** a locked
architecture. No implementation claim in this phase.

Companion brainstorm: [planning-pipeline-architecture.md](planning-pipeline-architecture.md)
(planner I/O families, parametric NLP). This doc focuses on **MPC / closed-loop
export / broadcast / deploy** and how that couples to a wider **Planner**
contract review.

PoC baseline (what exists today): [DESIGN.md](../../DESIGN.md) step/hybrid +
Phase 6a–6b; code under [`minilink/planning/mpc/`](../../minilink/planning/mpc/).

---

## 1. Why this review

The current MPC path is a **working proof of concept**. Roles were not designed
against a durable Minilink contract: planner vs control block vs export vs real
deploy. We need a **system-requirements** pass for an updated MPC / receding-horizon
(RH) system that one source of code can serve from **planner tuning → closed-loop
sim → real controller** (e.g. RAS / ROS-style node).

This document lists **requirements discussed**, **open questions**, and
**brainstorm sketches**. Prefer sketches labeled *idea* over *decision*.

---

## 2. PoC snapshot (facts, not solutions)

| Layer today | What it does | Gap vs desired |
| --- | --- | --- |
| `MPCPlanner` | compile-once NLP; `step(x0)` | Good kernel; hard-tied to “is the MPC system” |
| `MPCStatelessController` / `MPCStatefulController` | algebraic / warm-start leaves; `u_ff`/`x_ff`/`z` | Single-rate; only first-move slices; only `MPCPlanner` |
| `MPCTickLatch` | one NLP per tick | No plan flatten port; no ctl-rate nominals |
| Hybrid demos | `mpc % dt` then `computer @ plant` | Ceremony-heavy vs aim `mpc @ plant` |
| Deploy / RAS | none | No clocked tick API aimed at a real node |
| Debug | `step_disp` + post-hoc overlays | No shared debug mode across hand-loop / hybrid / node |

---

## 3. Open architecture questions (must resolve later)

These are still **open**. Any sketch in §5–§8 is a candidate, not the answer.

1. **What *is* MPC in Minilink?**
   - Only a planner?
   - Only a control block (`System` / `StepSystem`)?
   - A planner that **exports** as a control block / Computer?
   - A controller object that **imports** a planner (or several planner kinds)?
   - Another type that can be exported both ways?

2. **Should “MPC” mean NLP-MPC only, or any receding-horizon drafted plan?**
   - Narrow: parametric NLP + hybrid export.
   - Broad: same RH loop for trajopt / RRT / DP-rollout backends.

3. **How does this couple to the global `Planner` contract?**
   - Keep one `Planner` ABC + split results (`PathPlan` / `PolicyPlan`)?
   - Add specialized subclasses / mixins / adapters?
   - What minimal “online tick” surface do closed-loop exporters need?

4. **Where do warm-start state, plan latch, and high-rate broadcast live?**
   - Inside planner? Facade? Diagram sub-blocks? Shared latch?

5. **Naming:** `MPCController` vs `RecedingHorizonController` vs export-only helpers.

---

## 4. System requirements (from discussion)

Requirements are **R-*** ids. They should survive whichever architecture we pick.

### R1 — Dual diagram export (sim)

The MPC / RH product must be usable as:

- a **static** (`n=0`) block when no optimizer warm state is needed, and/or
- a **stepping** (`StepSystem`) block when warm-starting the next optimization
  (decision vector or equivalent warm state on `Computer.x`),

and must connect to a **continuous plant** for hybrid simulation (today’s
`Computer @ plant` / `HybridDiagram` world).

### R2 — Real-pipeline surface (RAS / deploy)

Same source must expose methods suitable for wrapping in a **real control node**
(lab RAS, later ROS2, etc.) — not only Minilink hybrid sim:

- clocked tick API (measurement in → command / plan out)
- success / timing / fail-soft hooks suitable for deadlines
- no GUI imports on the hot path

*(“RAS” treated as generic real-time node; ROS2 is a planned thin wrapper later.)*

### R3 — One source, three phases

Same core code supports:

| Phase | Need |
| --- | --- |
| **Tune planner** | prepare / step / inspect plan / plot / cost fields / timing |
| **Closed-loop sim** | hybrid with plant; warm-start; overlays |
| **Deploy** | node tick + publish plan / nominals / metadata |

### R4 — Control outputs: drafted plan is baseline

Outgoing action options should not be limited to “first move u”.

**Baseline:** expose the **whole latest drafted trajectory** (flattened plan)
so any downstream law can reconstruct the horizon.

Also keep convenience slices (e.g. first-move / next-state) for simple FF demos.

### R5 — Exposure for any downstream law (do *not* ship the laws)

Do **not** prioritize building a library of trackers / FF-FB modes as the product.

**Do** expose ports and information so downstream can implement:

- pure feedforward
- plan hold
- feedback / tracking on nominals
- cascade / custom laws

from the **same** exported surface.

### R6 — Faster-than-MPC-tick nominal broadcast

Between MPC / RH ticks, support **time-dependent broadcast** of nominals at a
higher rate (or continuous eval):

- `u_nom(t)`, `x_nom(t)`
- if possible `du_nom(t)`, `dx_nom(t)`

Idea under discussion: the MPC / RH export is not a lone leaf — it may be a
**Computer / small diagram** that includes a **faster interpolating broadcast
sub-block** reading the latched plan.

### R7 — Standard flatten / reconstruct of the whole plan

Provide a **standard** method to pack the whole plan as a big array and unpack
it back to a trajectory-like object so:

- diagram ports can carry one vector
- middleware / RAS can publish one payload
- downstream blocks can reconstruct the full plan if they want

Brainstorm preference voiced in discussion: put flatten/inflate on
**`Trajectory`** (alongside `save`/`load`/`resample`) rather than inventing a
separate `PlanCodec` type — still open to refinement.

### R8 — Aim UX: easy high-level closed loop

Desire: super-easy high-level with defaults, roughly:

```text
mpc @ plant
```

after the planning problem / planner exists — same mental model as `ctl @ plant`,
without forcing users to assemble `% schedule`, multi-rate fire tables, and
manual computer `x0` by hand. Advanced ports remain available when needed.

### R9 — Debug / observability

Debug mode covering development phases:

- prints (timings, success) — keep/extend `step_disp`
- static plots (last horizon, cost fields, plant vs plan)
- dynamic / animation overlays (horizon history, nominals vs plant)
- failure dumps (last plan / warm state / residuals as available)

### R10 — JAX posture

- **Today’s SciPy (or host) NLP tick need not be JAX-traceable.**
- **Most surrounding math should stay full JAX-traceable** (native arrays /
  `xp`, no forced NumPy coercion) for the day a **full JAX NLP solver** exists:
  pack/unpack, nominal eval / derivatives, warm-start shift, parametric `J/h/g`
  already on the JAX path, etc.
- Prefer **not** a parallel public `JaxTraj` twin unless later evidence shows it;
  keep `Trajectory` as the NumPy reporting bag (DESIGN), and keep equation-path
  helpers xp-clean.

### R11 — Planner diversity with the RH concept

It would be valuable if **RRT, DP, trajopt, …** could combine with the
**receding-horizon** idea — not only NLP-MPC — and if the MPC / RH control
surface could **import** more than one planner kind. This is tightly coupled to
reviewing the **Planner contract** (and possibly soft sub-contracts /
adapters). See §8.

---

## 5. Brainstorm sketches (ideas, not fixed)

### 5.1 Layering ideas for “what MPC is”

**Idea A — Planner owns NLP; thin System adapters export**  
Close to PoC. Clear Planner-vs-System law. Weak if every new backend needs a new
block class.

**Idea B — Facade controller imports a planner / horizon backend**  
User-facing object for hand-loop, export, RAS. Planner stays a tool. Matches
“MPC controller could import any type of planner” exploration.

**Idea C — Export is a multi-rate Computer (tick + broadcast)**  
Default product is not a lone leaf: slow replan tick + fast nominal interpolator
sub-block; boundary exposes plan + nominals. Satisfies R1+R5+R6 as one package.

**Idea D — Rename for honesty**  
If backends include RRT/DP-rollout, “MPC” may be too narrow; a generic
“receding-horizon controller” name may fit — with `MPC*` as the NLP-flavored
entry. Naming unsettled.

These can combine (e.g. B+C). **No pick is locked.**

### 5.2 Drafted-plan wire (R4, R7)

**Idea:**

```text
plan  →  flat vector  →  port / RAS payload  →  reconstruct plan
```

- Baseline public artifact = whole horizon, not only `u_ff`.
- Distinct from NLP decision vector `z` (warm-start / transcription-specific).
- Flatten API brainstorm: `Trajectory.to_flat` / `from_flat` (NumPy report);
  optional xp-native `pack_horizon` / `unpack_horizon` for traceable math (R10).

### 5.3 High-rate nominal broadcast (R6)

**Idea — two rates:**

| Rate | Role |
| --- | --- |
| `dt_mpc` (or replan period) | produce / latch latest plan |
| `dt_ctl` (faster) or continuous `t` | interpolate latched plan → `u_nom`, `x_nom`, optional rates |

**Derivative brainstorm:**

- `du`: FD / spline on input knots
- `dx`: prefer model `f(x_nom, u_nom, t)` when available; else FD

**Packaging brainstorm:** export builds a small step diagram + `StepSchedule.fire`
(multi-rate already exists on `Computer`) with a broadcast sub-block.

### 5.4 Downstream laws (R5)

**Requirement emphasis:** expose information, don’t ship the law library.

Minimum exposure brainstorm:

| Signal | Why |
| --- | --- |
| `plan_flat` | reconstruct whole plan |
| `u_nom`, `x_nom` | FF or tracking at ctl rate |
| `du_nom`, `dx_nom` | richer laws / filters |
| `u_ff`, `x_ff` | simple FF / seeds |
| `z` / warm state | warm-start & debug when applicable |
| `success` / metadata | fail-soft |

Auto-wire for aim demos may pick one applied `u` (e.g. `u_nom` or `u_ff`) —
exact default still open.

### 5.5 Aim closed loop (R8)

**Desire:**

```python
# after planner / problem exists
hybrid = mpc @ plant
```

Brainstorm defaults worth designing for (values TBD): replan period, faster
broadcast period, warm-start on/off, applied-port choice, computer `x0`.

Today’s ceremony (`stateful controller` → `% dt` → `@` → manual `x0_computer`)
is the **pain** this requirement targets.

### 5.6 Real node (R2)

**Idea:** same facade tick as the diagram latch:

```text
cmd = mpc.compute_command(y, warm_state=...)
# publish plan_flat / u / metadata; high-rate side evals nominals from latched plan
```

Diagram export and RAS should share plan flatten + nominal-eval helpers.

### 5.7 Debug (R9)

Shared debug flag / levels across hand-loop, hybrid, and node; overlays of
nominals vs plant; horizon history; keep timing prints.

---

## 6. Feature checklist by development phase (requirements view)

### P0 — Planner tuning

- [ ] Build problem + (prepare) planner
- [ ] Single / multi tick from `x0` with warm start where applicable
- [ ] Inspect full plan; flatten round-trip
- [ ] Plots / cost-field guides / timing
- [ ] Parametric scene updates when that work lands (other plan)

### P1 — Open-loop / export surface

- [ ] Flatten ↔ reconstruct whole plan
- [ ] Dense `u_nom`/`x_nom` (and rate) checks vs knot interp / `f`
- [ ] Multi-rate export sketch validated (if that packaging stays)

### P2 — Closed-loop Minilink

- [ ] Aim path toward `mpc @ plant` with defaults
- [ ] Warm-start parity vs hand loop
- [ ] User can wire FF **or** custom FB using exposed ports (demo only OK)
- [ ] Overlays + debug mode

### P3 — Deploy

- [ ] `compute_command` (or equivalent) for RAS/ROS wrap
- [ ] Publish drafted plan / nominals / metadata
- [ ] Fail-soft / deadline fields
- [ ] Later thin `ros2.py` (roadmap) — out of scope to implement here

---

## 7. JAX requirements (R10) — clarity

| Piece | Expectation |
| --- | --- |
| Host NLP (SciPy today) | Not required to be JIT/traceable |
| Pack / unpack / nominal eval / warm-start shift | Keep xp-clean / traceable |
| Parametric transcription / `J,h,g` on JAX path | Keep for future JAX NLP drop-in |
| `Trajectory` object | Remains NumPy reporting / I/O (DESIGN); don’t force it under `jit` |
| Parallel `JaxTraj` type | Avoid unless proven necessary |

Rule of thumb: **solver backend swappable later; horizon wire + broadcast math
should not freeze into NumPy-only APIs just because today’s NLP is host-side.**

---

## 8. Planner contract brainstorm (R11) — coupled review

### 8.1 Tension

Calling raw `Planner.compute_solution()` every control tick is a poor fit:

- RRT/trajopt bind start in `PlanningProblem` unless rebuilt
- DP returns a **policy table**, not a horizon
- Only `MPCPlanner.step(x0)` is shaped for online RH today

Yet we still want **RRT / DP / trajopt** to participate somehow.

### 8.2 Modes to keep distinct (brainstorm taxonomy)

| Mode | Meaning | Example |
| --- | --- | --- |
| **H1 — Horizon backend** | Each tick emit a drafted plan from `x0` | NLP-MPC step; RRT from `x0`; trajopt re-solve; DP **rollout** to a path |
| **H2 — Cascade / warm refine** | One tool proposes; another refines | RRT path → MPC/trajopt warm start |
| **H3 — Problem composition** | Offline artifact shapes online NLP | DP cost-to-go as terminal cost; track path as soft cost |

Closed-loop export (sim/RAS) cares first about **H1**. H2/H3 are valuable but
different API seams.

### 8.3 Organization ideas (still open)

**Idea — keep one `Planner` ABC** (matches planning-pipeline draft A): shared
`PlanningProblem`; split **results** (`PathPlan` vs `PolicyPlan`); avoid deep
`PathPlanner`/`PolicyPlanner` class trees.

**Idea — soft capabilities / adapters** instead of deep subclasses:

- `result_kind`: path vs policy
- `as_horizon_source(...)` or equivalent for things that can do H1
- DP online path into RH mainly via **`rollout(x0) → path`**, while true policy
  feedback stays `controller() @ plant`

**Idea — explicit online surface** the closed-loop product imports, e.g. duck
typing along the lines of “given `x0`, return a drafted path plan” — name TBD
(`horizon source`, `receding backend`, …).

**Idea — package placement:** adapters near `planning/` core; NLP-MPC + export
bundle under `planning/mpc/`; avoid forcing `search/` / DP to import MPC
internals.

### 8.4 Open questions (Planner review)

- Exact names and whether mixins help or hurt textbook clarity
- Whether `PolicyPlan.rollout` is required before RH+DP demos
- How much RRT tree reuse belongs in v1 vs “replan every tick”
- Whether the user-facing closed-loop object is named MPC-only or RH-generic

---

## 9. Consistency map — conversation topics ↔ requirements

| What was asked | Where captured |
| --- | --- |
| Planner vs block vs export; import planners | §3, §5.1, §8 |
| Static + stepping export; hybrid with continuous plant | R1, §5.1 Idea C |
| RAS / real pipeline methods | R2, §5.6 |
| Debug plots / prints | R9, §5.7 |
| Phases: tune → closed loop → deploy, one source | R3, §6 |
| Flattened drafted plan as baseline out | R4, R7, §5.2 |
| Downstream FF→FB via **exposure**, not law toolkit | R5, §5.4 |
| Faster nominal `u(t)`,`x(t)` (+ rates); export may include broadcast sub-block | R6, §5.3 |
| Aim `mpc @ plant` with defaults | R8, §5.5 |
| Flatten on `Trajectory`? | §5.2 (preference, not lock) |
| JAX: NLP not traceable now; plumbing stay traceable for future JAX NLP | R10, §7 |
| RRT/DP/trajopt with RH; Planner contract / sub-contracts | R11, §8 |

---

## 10. Explicitly *not* decided here

- Final class diagram / public names
- Whether multi-rate Computer is the only export shape
- Default applied port (`u_nom` vs `u_ff`)
- Exact flatten layout / header
- Whether to implement planner-class forks vs adapters
- ROS2 package, acados, shooting MPC, tracker library
- Updating DESIGN/ROADMAP in this brainstorm (doc-only for now)

---

## 11. Suggested next steps (process, not implementation commit)

1. Agree which **R-*** are must-have for the first rewrite milestone.
2. Pick a **thin vertical slice** (e.g. flatten + nominal broadcast exposure +
   aim `@` with NLP-MPC only) before multi-planner adapters.
3. Align with plan A (`PathPlan` / `PolicyPlan`) before locking RH import types.
4. Revisit §3 questions with the slice in hand — then write the real DESIGN
   contract.

---

## 12. Non-goals of *this* document

- Implementing the rewrite
- Freezing architecture as “the” Minilink MPC design
- Shipping downstream tracking controllers as a required deliverable
- Claiming maturity / ROADMAP checkboxes yet
