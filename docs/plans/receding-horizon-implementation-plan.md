# Receding-horizon vision & implementation plan

Status: **master plan — approved vision** (July 2026).

This is the **master implementation plan for the MPC / receding-horizon
refactor** (Planner 2×2 API, `TrajectoryPlan`, `RecedingHorizonController`,
merge of `MPCPlanner` into parametric TOP). Requirements brainstorm remains in
the related docs; when they disagree, **this file wins** for implementation.

Related plans:

- [mpc-controller-architecture.md](mpc-controller-architecture.md) — requirements R1–R12, Option β
- [planning-pipeline-architecture.md](planning-pipeline-architecture.md) — `TrajectoryPlan` / parametric NLP
- [standard-planning-problems.md](standard-planning-problems.md) — problem taxonomy (stochastic later; **out of this scope**)

Grounded in today’s code: `PlanningProblem`, `Planner`,
`TrajectoryOptimizationPlanner`, `MPCPlanner` + `MPC*Controller` leaves,
hybrid `Computer @ plant`.

---

## 1. End-goal vision (approve this picture)

### One-sentence product story

> Describe a **deterministic planning task** once (`PlanningProblem`).
> Solve it offline or online with a **Planner** that returns a typed
> **TrajectoryPlan**. Drive a plant in closed loop by wrapping any
> plan-producing solver in a **RecedingHorizonController** that re-plans
> from measured \(x\) each tick, latches the drafted horizon, and exports
> as a discrete control block / hybrid `Computer`.

“MPC” in demos is the common nickname for **compile-once trajopt used as
the `solve_trajectory_from` planner** inside that controller — not a second planner ABC.

### Big picture

```mermaid
flowchart LR
  subgraph task["Task description"]
    PP["PlanningProblem<br/>sys, X, U, X0/Xf, cost, params"]
  end

  subgraph offline["Offline / tuning"]
    PL["Planner"]
    TOP["TrajectoryOptimizationPlanner<br/>(incl. parametric / compile-once mode)"]
    TP["TrajectoryPlan<br/>trajectory + warm_state + metadata"]
    PP --> PL
    PL --> TOP
    TOP --> TP
  end

  subgraph online["Online plan-and-act"]
    RHC["RecedingHorizonController"]
    CMD["Command / ports<br/>plan_flat, u_ff, …"]
    COMP["Computer + schedule"]
    PLANT["Continuous plant<br/>DynamicSystem"]
    HYB["HybridDiagram"]

    TOP -->|"solve_trajectory_from(x0)"| RHC
    RHC -->|compute_command| CMD
    RHC -->|export / @| COMP
    COMP --> HYB
    PLANT --> HYB
  end

  TP -.->|"Command.plan is TrajectoryPlan"| CMD
```

### Mental model (roles)

| Object | Role | Band |
| --- | --- | --- |
| `PlanningProblem` | **What** to achieve (math task), incl. continuous \(T\) | planning |
| `Planner` | **How** to compute a solution (tool) | planning |
| `TrajectoryOptimizationPlanner` | NLP trajopt planner (shooting / collocation; optional compile-once) | planning |
| `TrajectoryPlan` | Typed **result**: open-loop schedule + solve metadata (+ warm state) | planning |
| `RecedingHorizonController` | Online **control product**: tick → latch → ports / RAS / `@ plant` | planning (controller façade) |
| `Computer` / `HybridDiagram` | Existing step/hybrid host | simulation / core |

**No `HorizonSource` type.** Online capability is just the planner method
`solve_trajectory_from(x0, …) -> TrajectoryPlan` (see §2.2 / §3.2). An extra
adapter object was considered and dropped as redundant with that surface.

**Laws this vision obeys**

- Continuous-time core stays clean; RH lives on **sibling** step/hybrid types.
- Planners are **tools**; the RH object is a **System / StepSystem façade**
  (or builds one). Prefer Option β: do **not** put `@ plant` on raw `Planner`.
- One `Planner` ABC; split **results** (`TrajectoryPlan` vs later `PolicyPlan`),
  not planner hierarchies.
- Receding horizon is a **loop pattern**, not a special algorithm class.

### Aim UX (after prepare)

```python
# tune (offline / fixed problem)
plan = planner.solve()              # short entry → TrajectoryPlan (or .trajectory during migration)
# plan = planner.solve_trajectory() # typed equivalent
planner.plot_solution()

# closed loop (defaults) — pass the planner directly
rhc = RecedingHorizonController(planner, dt_mpc=0.2)
hybrid = rhc @ plant                 # plant sim facade (not planner)
hybrid.compute_trajectory(tf=10.0)

# deploy (same core)
cmd = rhc.compute_command(y)         # calls planner.solve_trajectory_from(y)
# publish cmd.plan_flat / cmd.u_ff / cmd.metadata
```

`MPCController` may remain a thin alias / factory for “NLP compile-once +
defaults” for teaching demos.

---

## 2. Planning stack — contracts

### 2.1 `PlanningProblem` — the task

**What it is:** declarative deterministic task (exists today).
Plant + sets + cost + parameter scaffold. **Does not solve itself.**

**Owns**

| Field | Meaning |
| --- | --- |
| `sys` | Continuous plant / diagram to plan for |
| `X`, `U` | Path sets |
| `X0`, `Xf` | Boundary sets (authoritative) |
| `x_start`, `x_goal` | Representative points / shortcuts |
| `cost` | Optional Bolza / path cost (integrand / terminal — **not** the interval length) |
| `horizon` | **Continuous-time** prediction / planning interval length \(T\) (see §2.1b) |
| `params` | `ProblemParameters` (`system`, `cost`, `sets`; later `scene`) |
| `metadata` | Opaque tags |

**Must implement / keep**

- Construction validation (dims, membership of `x_start` in `X0`, …)
- `require_cost()` / `require_goal()` helpers (planner-facing)
- Optional `horizon` validation when set (`T > 0`, finite)

**Does not implement**

- Discretization knot count `n_steps`, time-grid layout, warm-start, JIT, tick loops
- Optimizer presets

```text
PlanningProblem
  in:  (constructor fields, including optional continuous horizon T)
  out: immutable task description consumed by every Planner
```

### 2.1b Continuous horizon \(T\) vs discretization \(N\) — revised law

These are **different objects**. Do not lump them as “horizon options.”

| Quantity | Math role | Home | Varies online? |
| --- | --- | --- | --- |
| **\(T\)** (continuous time horizon) | Domain of the OCP: plan over \([0, T]\) (or \([t, t+T]\)) | **`PlanningProblem.horizon`** | Allowed in principle (RH may update \(T\)); see below |
| **\(N\)** (`n_steps`) | How the solver discretizes that interval | **Transcription options** only | Fixed with NLP structure / decision size for compile-once |

**Not on the cost.** `cost` defines \(L\), \(\Phi\), weights, barriers. The interval length \(T\) is part of the **task statement**, not a cost coefficient.

**Grounding in today’s demos (to migrate):** every MPC script sets both on transcription today:

```text
DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
# MPC_HORIZON → belongs on PlanningProblem.horizon  (task)
# MPC_STEPS   → stays on DirectCollocationOptions.n_steps  (solver)
```

`TF_SIM` / hybrid `tf=` remains **simulation** length — unrelated to planning \(T\).
RH **`dt_mpc`** remains **replan period** on `RecedingHorizonController` — how often we
resolve — also not \(T\).

**Planners that ignore \(T\):** RRT edge “horizon” and DP backup horizons are
*different* concepts. Those planners leave `problem.horizon` unused (or only
use it if a path-cost / rollout helper wants a duration). Field is **optional**
so RRT/DP construction does not break.

**Compatibility (do not break old call sites):**

1. Add optional `PlanningProblem.horizon: float | None = None`.
2. Trajopt / MPC transcription **prefer** `problem.horizon` when set; else fall
   back to `options.tf` (today’s demos keep working unchanged).
3. If both set and disagree → clear error (or documented override rule:
   problem wins). Long-term: deprecate `FixedGridOptions.tf` as the task source;
   options keep `n_steps` (+ derived `t` grid from problem \(T\)).
4. No required migration of every demo in Phase 1; migrate flagship RH demos
   when wiring `RecedingHorizonController`.

**Varying \(T\) in the RH controller (vision, not Phase 1 must):**

- Updating \(T\) each tick is a **problem / params** change, not an `n_steps` change.
- With **fixed \(N\)**: knot spacing \(\Delta t = T/(N-1)\) can track \(T\) if the
  parametric NLP treats time scale as runtime data (future); or cheap re-`prepare`
  if structure allows.
- Changing \(N\) with \(T\) is a **structural** NLP change (new \(n_z\)) → re-JIT —
  that stays a solver concern, not a reason to put \(N\) on the problem.
- Phase 1 ships **fixed \(T\)** (as today). Phase 4+ may expose runtime
  `horizon` / params updates once parametric time scaling is designed.

*Note:* [standard-planning-problems.md](standard-planning-problems.md) currently
says horizon stays method-side — revise that doc to match this \(T\) vs \(N\)
split when DESIGN is synced (continuous \(T\) on problem; \(N\) method-side).

---

### 2.2 `Planner` — mother class alignment (2×2 API)

**What it is:** shared orchestrator ABC (exists in
[`planning/planner.py`](../../minilink/planning/planner.py)). One ABC for
trajopt, RRT, DP — **no** `TrajectoryPlanner` / `PolicyPlanner` class fork.
A concrete planner may implement **any subset** of the four methods below;
unimplemented ones stay as `NotImplementedError` on the base.

#### Today (problem)

```text
Planner
  + problem
  + last_result          # untyped
  + compute_solution()*  # return type left to subclass — rename to solve()
  + plot_solution()      # assumes Trajectory
  + animate_solution()   # assumes Trajectory
```

| Subclass | `compute_solution()` returns today |
| --- | --- |
| `TrajectoryOptimizationPlanner` | `Trajectory` |
| `MPCPlanner` | `Trajectory` (via `step(x_start)`) |
| `RRTPlanner` / `RRTStarPlanner` | `Trajectory` |
| `DynamicProgrammingPlanner` | `DynamicProgrammingResult` (policy table) |

**Rename scope (repo check, July 2026):** `compute_solution` appears in on the order of
**~40–60 call sites** (tests heaviest in `test_rrt.py`, plus trajopt demos,
benchmarks, a few notebooks, README/DESIGN). Definitions live on
`Planner` + TOP / MPC / RRT / DP / `PolicyEvaluator`. Manageable mechanical
rename — not a large job; do it with the Planner API pass (no long alias
period required pre-1.0).

#### Naming decision: `solve_*` (not `compute_*`)

**Verb:** use **`solve`**, not `compute`, for all four.

| Reason | Detail |
| --- | --- |
| Clash | `System.compute_trajectory` / hybrid `compute_trajectory` already mean **simulate the plant**. A planner method named `compute_trajectory` would collide mentally (and in autocomplete). |
| Domain | Trajopt / MPC / RRT / DP “solve” a problem; `Optimizer.solve` already exists. |
| One verb | Offline and online share the same verb; only the `_from` suffix marks the parameterized entry. |

**Drop “plan” from the method name** — the object is already a `Planner`; the
return type is `TrajectoryPlan` / `PolicyPlan`. So:
`solve_trajectory` not `solve_trajectory_plan` / `compute_trajectory_plan`.

Family marker: **`traj`** \| **`policy`** (not `"path"`).

#### The 2×2 matrix (locked)

|  | **Trajectory** (`TrajectoryPlan`) | **Policy** (`PolicyPlan`) |
| --- | --- | --- |
| **Fixed** (problem data as built; offline tune) | `solve_trajectory(**kw)` | `solve_policy(**kw)` |
| **From runtime args** (online; primarily \(x_0\)) | `solve_trajectory_from(x0, *, params=None, warm_start=None)` | `solve_policy_from(x0, *, params=None, warm_start=None)` |

```text
                    artifact
                 traj              policy
              ┌─────────────────┬─────────────────┐
 fixed        │ solve_trajectory│ solve_policy    │
 (offline)    │                 │                 │
              ├─────────────────┼─────────────────┤
 from(x0[,p]) │ solve_trajectory│ solve_policy    │
 (online)     │ _from           │ _from           │
              └─────────────────┴─────────────────┘
```

**Semantics**

| Method | Meaning |
| --- | --- |
| `solve_trajectory` | Solve using the planner’s bound `PlanningProblem` as-is (template `x_start`, fixed \(T\), …). Returns `TrajectoryPlan`. |
| `solve_trajectory_from(x0, *, params=None, warm_start=None)` | Same algo, but start (and optional runtime `params`) supplied at the call. Primary RH / MPC entry. |
| `solve_policy` | Offline policy synthesis (DP). Returns `PolicyPlan`. |
| `solve_policy_from(…)` | Parameterized policy solve if/when needed (rare at first; façade reserved). |

**Who implements what (typical)**

| Planner | Implements |
| --- | --- |
| TOP batch | `solve_trajectory`; `solve_trajectory_from` = refresh start + re-transcribe (slow OK) |
| TOP parametric / today’s MPC | `prepare` + `solve_trajectory_from`; `solve_trajectory` ≡ `solve_trajectory_from(problem.x_start)` |
| RRT | `solve_trajectory` (+ optional `_from` = replan from new start) |
| DP | `solve_policy`; optional later `solve_trajectory_from` via rollout if used inside RH |

RH **only** requires `solve_trajectory_from`. It never needs policy methods.

#### Target mother contract

```text
Planner
  + problem: PlanningProblem

  # ---- cached solutions (parallel slots — not a single last_result) ----
  + last_trajectory_plan: TrajectoryPlan | None
  + last_policy_plan: PolicyPlan | None
  # deprecated shim: last_result → prefer traj slot, else policy (migration only)

  + result_kind: "traj" | "policy"    # primary family; not exclusive of the other slot

  # ---- 2×2 (defaults: NotImplementedError) ----
  + solve_trajectory(**kw) -> TrajectoryPlan
        # stores last_trajectory_plan
  + solve_trajectory_from(x0, *, params=None, warm_start=None) -> TrajectoryPlan
        # stores last_trajectory_plan
  + solve_policy(**kw) -> PolicyPlan
        # stores last_policy_plan
  + solve_policy_from(x0, *, params=None, warm_start=None) -> PolicyPlan
        # stores last_policy_plan

  # ---- short offline entry (replaces compute_solution) ----
  + solve(**kw)
        # default dispatch by primary result_kind; subclass may fill BOTH slots
        # e.g. DP: solve_policy → last_policy_plan, then optional rollout →
        #          last_trajectory_plan (same call or explicit follow-up)
        # return: primary artifact for that planner (see subclass)

  + plot_solution / animate_solution
        # use last_trajectory_plan.trajectory (roll out first if only policy exists)
```

** Dual slots (why not one `last_result`)**

A planner can hold **both** a policy and a trajectory at once. Classic case: DP
`solve_policy` fills `last_policy_plan`; rolling out \(\pi\) from \(x_0\) fills
`last_trajectory_plan` without discarding the policy table. A single
`last_result` would collide / force awkward unions.

| Slot | Set by | Cleared? |
| --- | --- | --- |
| `last_trajectory_plan` | any `solve_trajectory*` (and optional rollout after policy) | only when overwritten by a new traj solve |
| `last_policy_plan` | any `solve_policy*` | only when overwritten by a new policy solve |

Solving one family does **not** auto-clear the other. Helpers:
`require_trajectory_plan()` / `require_policy_plan()` (clear errors if missing).

**`bind(x0)`** stays **internal** to parametric evaluators (called inside
`solve_trajectory_from`). Not part of the `Planner` ABC.

**`solve()` vs typed methods:** demos/tests may keep calling `planner.solve()`
for the short offline path (today’s `compute_solution`). Typed
`solve_trajectory` / `solve_policy` are the authoritative family APIs;
`solve()` is only the thin dispatcher — **not** a fifth algorithm.

**Decision locked for this plan**

| Choice | Rule |
| --- | --- |
| Verb | **`solve_*`** for the 2×2; short offline entry is **`solve()`** |
| Method names | `solve_trajectory`, `solve_policy`, `solve_trajectory_from`, `solve_policy_from`, plus `solve` |
| Rename | **`compute_solution` → `solve`** repo-wide (mechanical; ~40–60 call sites) |
| Not used | `compute_trajectory*` on Planner (sim clash); bare `solve_from`; keep no `compute_solution` alias after rename |
| Fork ABC? | **No** — one mother; implement 1–4 typed methods (+ inherited `solve` dispatch) |
| What RH imports | `planner.solve_trajectory_from(…) → TrajectoryPlan` (not `solve()`) |
| Result cache | **`last_trajectory_plan` + `last_policy_plan`** in parallel; retire single `last_result` |

```mermaid
classDiagram
  class Planner {
    +problem
    +last_trajectory_plan
    +last_policy_plan
    +result_kind
    +solve()
    +solve_trajectory() TrajectoryPlan
    +solve_trajectory_from(x0, params) TrajectoryPlan
    +solve_policy() PolicyPlan
    +solve_policy_from(x0, params) PolicyPlan
  }
  class TrajectoryOptimizationPlanner {
    +result_kind = traj
    +prepare()
    +solve_trajectory()
    +solve_trajectory_from(x0, params)
  }
  class RRTPlanner {
    +result_kind = traj
    +solve_trajectory()
  }
  class DynamicProgrammingPlanner {
    +result_kind = policy
    +solve_policy()
  }
  class TrajectoryPlan
  class PolicyPlan

  Planner <|-- TrajectoryOptimizationPlanner
  Planner <|-- RRTPlanner
  Planner <|-- DynamicProgrammingPlanner
  TrajectoryOptimizationPlanner --> TrajectoryPlan
  RRTPlanner --> TrajectoryPlan
  DynamicProgrammingPlanner --> PolicyPlan
```

**Does not own:** hybrid export, RAS tick, multi-rate broadcast, plant ports.
Those live on `RecedingHorizonController`.

---

### 2.3 `TrajectoryOptimizationPlanner` — detailed contract + merge with `MPCPlanner`

**What it is:** traj-family planner that builds
`problem → transcription → (parametric) MathematicalProgram → Optimizer → TrajectoryPlan`.

#### 2.3.1 What exists today (two sibling classes)

| | `TrajectoryOptimizationPlanner` | `MPCPlanner` |
| --- | --- | --- |
| File | `trajectory_optimization/planner.py` | `mpc/planner.py` |
| Transcription | `DirectCollocationTranscription.transcribe` → `MathematicalProgram` | `MPCDirectCollocationTranscription.transcribe_parametric` → `ParametricMathematicalProgram` |
| Compile | Per `solve()` / `solve_trajectory` (re-transcribe + Optimizer compile) | Once in `prepare()`; JIT `J,h,g` |
| Runtime bind | None — `x_start` frozen in program closure | `bind(x0)` on equality `h(z, x0)` only |
| Online API | — | `step(x_start) -> Trajectory` |
| Offline API | `compute_solution() -> Trajectory` (→ rename `solve`) | same via `step(template x_start)` |
| Cost / sets in JIT | Baked at each transcribe | Baked at `prepare` (scene moves ⇒ re-prepare) |
| Warm-start | options / last Trajectory as guess | packed `z` + `mpc_warm_start_guess` |

Both subclass `Planner` independently. Collocation math is largely duplicated.

#### 2.3.2 End goal — one class, two **program** modes

Same `TrajectoryOptimizationPlanner`; mode selected by options (or by whether
transcription supports parametric export).

```text
compile_mode = "batch" | "parametric"
```

| Mode | Compiled artifact | Solve entry | Typical use |
| --- | --- | --- | --- |
| **`batch`** | Fixed `MathematicalProgram` — \(J(z), h(z), g(z)\); no runtime \(p\) | `solve_trajectory()` (re-transcribe each call, as today) | Open-loop trajopt demos |
| **`parametric`** | `ParametricMathematicalProgram` — structure fixed; runtime bag \(p\) | `prepare()` once, then `solve_trajectory_from(x0, params=…)` | RH / “MPC”, sweeps |

```mermaid
flowchart TD
  PP[PlanningProblem + horizon T]
  TOP[TrajectoryOptimizationPlanner]
  TX[Transcription + n_steps]
  PP --> TOP
  TX --> TOP
  TOP --> mode{compile_mode}
  mode -->|batch| MP["MathematicalProgram\nJz hz gz"]
  mode -->|parametric| PMP["ParametricMathematicalProgram\nJ(z,p) h(z,p) g(z,p)"]
  MP --> OPT1[Optimizer.solve]
  PMP --> PREP[prepare / JIT]
  PREP --> BIND["bind p = x0 + params"]
  BIND --> OPT2[host NLP solve]
  OPT1 --> TP[TrajectoryPlan]
  OPT2 --> TP
```

#### 2.3.3 Parametric runtime bag \(p\) — façade even before scene lands

**Today:** `ParametricMathematicalProgram` is only \(h(z, x0)\); \(J(z)\), \(g(z)\)
closed over build-time data.

**Target shape of \(p\)** (align with pipeline plan B / `ProblemParameters`):

```text
p  ~  {
  x0:     ndarray(n,),           # always — measurement / start
  scene:  …,                     # later ObstacleBank arrays
  cost:   …,                     # later weight overrides
  # horizon T: later time-scale if varying-T with fixed N
}
```

**Phase 1 / 2 implementation bar**

| Capability | When |
| --- | --- |
| `solve_trajectory_from(x0, *, params=None, warm_start=None) -> TrajectoryPlan` public | Phase 2 merge (Phase 1: wrap `MPCPlanner.step`) |
| `params is None` → behave as today’s `step(x0)` (`p = {x0}`) | required |
| `params is not None` but scene not wired → **clear error** or ignore-with-warning (prefer error) | Phase 2 |
| Structured `ProblemParameters` / scene in JIT | Phase 4 |
| `solve_trajectory_from(..., params=)` signature exists | Phase 1 on MPC / Phase 2 native — **façade first** |

So: prepare the **API seam** for variable planning parameters now; do not
block RH UX on ObstacleBank.

#### 2.3.4 Target public methods on `TrajectoryOptimizationPlanner`

```text
TrajectoryOptimizationPlanner(
    problem,
    *,
    transcription,                    # owns n_steps; T from problem.horizon
    options: TrajectoryOptimizationOptions,
)
# options.compile_mode: "batch" | "parametric"
# options.compile_backend: numpy | jax (parametric: jax today)

# --- lifecycle ---
prepare() -> None
    batch:       no-op or caches nothing required
    parametric:  resolve T from problem; transcribe_parametric;
                 JIT evaluator; store program + sample z0

# --- Planner mother (traj family) ---
result_kind == "traj"

solve_trajectory(*, initial_guess=None, warm_start=None) -> TrajectoryPlan
    batch:       transcribe fixed MathematicalProgram → solve → wrap TrajectoryPlan
    parametric:  equivalent to solve_trajectory_from(problem.x_start, ...)

solve(**kw) -> TrajectoryPlan   # dispatcher; may return .trajectory during call-site migration

# --- online / RH ---
solve_trajectory_from(x0, *, params=None, warm_start=None) -> TrajectoryPlan
    batch:       rebuild problem x_start (and apply params if any) →
                 full re-transcribe (slow but valid)
    parametric:  bind p={x0, **params}; host NLP; reconstruct;
                 warm_state = result.z in TrajectoryPlan

# warm_state_dim / default_warm_state: optional attrs RH may read (n_z or 0)
```

**Internal attributes (parametric mode)** — merge of today’s TOP + MPCPlanner:

| Attribute | Source today | Role |
| --- | --- | --- |
| `transcription` | both | grid \(N\), pack/unpack |
| `program` | MPC: `ParametricMathematicalProgram`; TOP: last `MathematicalProgram` | compiled NLP |
| `program_evaluator` | MPC JIT bind | SciPy-facing `J,h,g` |
| `last_optimization_result` | both | fills `SolveMetadata` |
| `compile_time_s` / solve timing | MPC | metadata / `step_disp` |
| `_dynamics` | MPC reconstruct | optional consistent \(x\) rollout |

#### 2.3.5 Combining the two classes (merge map)

```text
Phase 1:  RecedingHorizonController  --adapter-->  MPCPlanner.step
Phase 2:  MPCPlanner body moves into TrajectoryOptimizationPlanner(compile_mode="parametric")
          MPCPlanner = thin alias / factory  (pre-1.0 deprecation)
```

| Concern | Keep from TOP | Keep from MPCPlanner | Unified home |
| --- | --- | --- | --- |
| Batch `transcribe` → `MathematicalProgram` | yes | — | `transcription.transcribe` |
| Parametric `transcribe_parametric` | — | yes | same transcription API + `compile_mode` |
| `Optimizer` wrapper + history callbacks | yes | SciPy backend direct | prefer TOP `Optimizer` path where possible |
| `bind(x0)` | — | yes | generalize → `bind(p)` with at least `x0` |
| `step` name | — | yes | rename to **`solve_trajectory_from`**; `step` alias deprecated |
| `MPCOptions` vs `TrajectoryOptimizationOptions` | TOP options | MPC options | one options dataclass + parametric flags (`step_disp`, `compile_mode`) |
| `MPCDirectCollocationTranscription` vs `DirectCollocationTranscription` | both | fold parametric method onto one collocation class (pipeline B8) | Phase 2 |

#### 2.3.6 Program export surface (conceptual)

Not every user needs raw programs, but the planner owns this split:

```text
# batch (fixed)
program = transcription.transcribe(problem)     # MathematicalProgram
#   minimize J(z)
#   s.t.   h(z) = 0,  g(z) >= 0
#   x0 appears only as data closed inside h/J at build time

# parametric (x0 / later params as inputs)
program = transcription.transcribe_parametric(problem)  # ParametricMathematicalProgram
#   minimize J(z, p)          # Phase 2 may still be J(z) until scene
#   s.t.   h(z, p) = 0        # today: h(z, x0); façade: h(z, p)
#          g(z, p) >= 0       # today: g(z); later g(z, p_scene)
#   evaluate.bind(p) before each solve_trajectory_from
```

**Façade rule:** public `solve_trajectory_from(x0, params=None)` always accepts
`params`. Evaluator `bind` accepts a small dict/pytree with required `x0`.
Extra keys reserved for Phase 4; unused keys error until implemented so silent
ignore does not hide perception bugs.

#### 2.3.7 Implements (internally)

- Resolve \(T\) from `problem.horizon` (preferred) else `options.tf`
- Transcription owns **`n_steps` only** (long-term)
- Pack / unpack \(z\); warm-start helpers
- Build `SolveMetadata` from `OptimizationResult`
- Store `TrajectoryPlan(trajectory, metadata, warm_state=z)`

#### 2.3.8 What Phase 1 does **not** require

- Full merge of `MPCPlanner` into TOP (Phase 2)
- Scene-parametric `J(z,p)` / `g(z,p)` (Phase 4)
- Changing mother ABC to remove `compute_solution` (keep + forward)

---

### 2.4 `TrajectoryPlan` — the traj-family result

**What it is:** standard payload of a traj planner (and of each RH tick).

```text
TrajectoryPlan
  trajectory: Trajectory          # (t, x, u) — NumPy reporting bag
  metadata: SolveMetadata         # success, message, solve_time_s, cost, …
  warm_state: array | None        # e.g. packed NLP z for next tick
  x_dot: ndarray | None           # optional dx/dt on the plan grid (n, N)
  u_dot: ndarray | None           # optional du/dt on the plan grid (m, N)
```

**Must implement**

| Method / field | Why |
| --- | --- |
| `.trajectory` / `.metadata` / `.warm_state` | Primary payload |
| `.x_dot` / `.u_dot` | Optional reserved rates (see below); default `None` |
| `to_flat()` / `from_flat()` | Diagram port + RAS one-vector wire (R4/R7); rates in flat layout TBD / versioned |
| Convenience: `.t`, `.x`, `.u` or `.as_trajectory()` | Teach / plot |

**Optional rates (`x_dot`, `u_dot`) — locked as reserved, not required**

Worth reserving on `TrajectoryPlan` (not stuffing into core `Trajectory`) so RH
broadcast / richer FF laws (R6) have a known place for knot-level rates without
polluting the shared sim schedule type.

| Rule | Detail |
| --- | --- |
| Default | Both `None` — RRT / simple trajopt need not compute them |
| Who fills | Optional post-process after solve (FD/spline on knots; or `x_dot ≈ f(x,u,t)` when dynamics available). Never required for `solve_trajectory*` success |
| Not a substitute for ctl-rate interp | High-rate `u_nom(t)` / `dx_nom(t)` between knots still live in broadcast / `get_nominal(t, include_derivatives=…)`; stored plan rates are knot samples or a cache, not the only evaluation path |
| Why not only `Trajectory.signals` | Signals stay valid for ad hoc channels; **reserved fields** advertise a stable contract for RH/Command. Planners should not put NLP `z` in signals |
| Phase 1 | Ship fields as `None`; no planner must fill them; demos unchanged |

**Does not implement**

- Replanning, hybrid export, interpolation policy (those are controller /
  broadcast concerns)

`SolveMetadata`: shared dataclass (`success`, `message`, `solve_time_s`,
`cost`, `iterations`, …) filled from today’s side channels
(`OptimizationResult`, …).

**Policy family (later, not this milestone):** `PolicyPlan` with
`controller()` / `rollout(x0) → TrajectoryPlan`. DP plugs into RH by
implementing `solve_trajectory_from` (e.g. via rollout) on that planner — no
separate `HorizonSource` type.

---

### 2.5 Planning-stack interaction

```mermaid
classDiagram
  class PlanningProblem {
    +sys
    +X U X0 Xf
    +cost
    +horizon
    +params
  }
  class Planner {
    +problem
    +solve()*
    +solve_trajectory()*
    +solve_trajectory_from(x0)*
    +solve_policy()*
    +solve_policy_from(x0)*
  }
  class TrajectoryOptimizationPlanner {
    +transcription
    +options
    +prepare()
    +solve_trajectory()
    +solve_trajectory_from(x0, params)
  }
  class SolveMetadata {
    +success
    +solve_time_s
    +cost
  }
  class TrajectoryPlan {
    +trajectory
    +metadata
    +warm_state
    +to_flat()
    +from_flat()
  }
  class Trajectory {
    +t x u
  }

  Planner --> PlanningProblem : owns
  Planner <|-- TrajectoryOptimizationPlanner
  TrajectoryOptimizationPlanner --> TrajectoryPlan : returns
  TrajectoryPlan --> Trajectory
  TrajectoryPlan --> SolveMetadata
```

---

## 3. Receding-horizon control — contracts

### 3.1 What `RecedingHorizonController` is

User-facing **online plan-and-act product** (Option β).

Not a Planner. Not the NLP. It **holds a traj planner** (or any duck with
`solve_trajectory_from`) and runs the online loop:

1. On each control tick: `planner.solve_trajectory_from(y, …)` → latch `TrajectoryPlan`
2. Expose ports / `Command` for FF, full plan, warm state, success
3. Export to `Computer` for hybrid sim, or serve RAS via `compute_command`
4. Aim: `rhc @ plant` with defaults

Alias: `MPCController` = RH façade with NLP parametric planner defaults
(optional naming sugar).

### 3.2 Online planner surface — no `HorizonSource` intermediary

**Decision:** drop the separate `HorizonSource` layer. Online entry is the
mother method `solve_trajectory_from` (see §2.2).

`RecedingHorizonController` takes a **planner duck** that implements:

```text
# required
solve_trajectory_from(x0, *, params=None, warm_start=None) -> TrajectoryPlan

# optional (RH uses defaults if missing)
prepare()?                      # call once if present
warm_state_dim -> int           # default 0
default_warm_state()?           # optional
```

| Backend | How it meets `solve_trajectory_from` |
| --- | --- |
| Parametric TOP / today’s `MPCPlanner` | native / `step` wrapped → `TrajectoryPlan` |
| Batch TOP | re-transcribe from updated `x_start` (slow, valid) |
| RRT | replan from \(x\) → `TrajectoryPlan` |
| DP | later: `solve_trajectory_from` via rollout on that planner if needed |

**When a one-off wrapper is still OK:** only if an object cannot grow
`solve_trajectory_from` without lying (rare). Prefer adding the method on the
planner subclass. Do **not** invent a framework-wide `HorizonSource` type.

**Why this is enough:** RH only needs “given \(x[,p]\), return a
`TrajectoryPlan`.” Offline methods (`solve_trajectory`, `plot_solution`) stay
on the same object; RH simply never calls them on the hot path.

### 3.3 `RecedingHorizonController` — I/O contract

**Construction**

```text
RecedingHorizonController(
    planner,                       # duck: solve_trajectory_from → TrajectoryPlan
    *,
    dt_mpc: float,
    dt_ctl: float | None = None,   # Phase later: broadcast rate; default = dt_mpc
    warm_start: bool = True,
    applied_u: str = "u_ff",       # default ZOH first-move for @ plant
    debug: … = None,
)
```

**Runtime API (deploy + tests)**

```text
compute_command(y, *, params=None, warm_state=None, t=None, k=None) -> Command

Command:
  plan: TrajectoryPlan
  plan_flat: ndarray
  u_ff, x_ff: ndarray          # convenience slices
  z / warm_state: ndarray | None
  metadata: SolveMetadata
```

**Diagram export**

```text
as_stateless_block() -> System          # n=0, optional
as_step_block() -> StepSystem           # state = warm_state when dim>0
export_to_computer(schedule=None) -> Computer
__matmul__(plant) -> HybridDiagram      # aim: rhc @ plant
reset()
```

**Default diagram ports (tick block inside export)**

| Direction | Port | Dim / notes |
| --- | --- | --- |
| in | `y` | measurement ≈ \(x\) (`n`) |
| in | `params` / scene | optional; Phase later (R12) |
| out | `plan_flat` | flattened drafted horizon (baseline) |
| out | `u_ff` | first-move \(u\) (default applied) |
| out | `x_ff` | next planned state slice |
| out | `z` | warm state when used |
| out | `success` | from metadata |

**Phase later (R6):** bundled `BroadcastBlock` reading latched `plan_flat` →
`u_nom(t)`, `x_nom(t)` at `dt_ctl`. **Not required** for first `@ plant`
(hybrid ZOH of `u_ff` already exists).

### 3.4 Tick loop (generic)

```mermaid
sequenceDiagram
  participant Plant
  participant RHC as RecedingHorizonController
  participant PL as TrajPlanner

  Plant->>RHC: y ≈ x (tick k)
  opt perception
    Note over RHC: params / scene (later)
  end
  RHC->>PL: solve_trajectory_from(x, params, warm_start)
  PL-->>RHC: TrajectoryPlan
  RHC->>RHC: latch plan; slice u_ff, plan_flat
  RHC-->>Plant: u (ZOH / applied port)
```

### 3.5 RH class diagram

```mermaid
classDiagram
  class Planner {
    +solve_trajectory_from(x0, params, warm_start) TrajectoryPlan
    +warm_state_dim
  }
  class RecedingHorizonController {
    +planner
    +dt_mpc
    +compute_command(y) Command
    +export_to_computer() Computer
    +__matmul__(plant) HybridDiagram
    +as_step_block() StepSystem
  }
  class Command {
    +plan
    +plan_flat
    +u_ff
    +x_ff
    +warm_state
    +metadata
  }
  class TickLatch {
    +solve_for_tick(k, y)
    +last_plan TrajectoryPlan
  }
  class Computer
  class HybridDiagram
  class DynamicSystem

  RecedingHorizonController --> Planner : holds
  RecedingHorizonController --> TickLatch : owns
  RecedingHorizonController --> Command : produces
  RecedingHorizonController --> Computer : export
  RecedingHorizonController --> HybridDiagram : matmul plant
  Computer --> HybridDiagram
  DynamicSystem --> HybridDiagram
  Planner ..> TrajectoryPlan : solve_trajectory_from
```

---

## 4. Where utilities live

Keep **algorithm / transcription** vs **online loop** vs **analysis** separate.

| Concern | Home | Examples |
| --- | --- | --- |
| Transcription, pack/unpack \(z\), parametric NLP | Planner / `trajectory_optimization/` (+ today’s `mpc/transcription.py` until merge) | collocation defects, `prepare` |
| Warm-start shift of \(z\) / plan | Planner-adjacent helpers (`mpc/warm_start.py` or shared) | used by latch + `solve_trajectory_from` |
| Tick latch (one NLP per \(k\)) | Inside RH controller package | evolve today’s `MPCTickLatch` |
| Flatten plan for ports / RAS | `TrajectoryPlan` methods | `to_flat` / `from_flat` |
| Nominal interpolation / rates | RH export helpers or small `BroadcastBlock` | Phase later (R6) |
| Hybrid export / `@` | `RecedingHorizonController` | wraps existing `Computer` / `hybrid_closed_loop` |
| RAS tick API | `RecedingHorizonController.compute_command` | no GUI imports |
| `step_disp`, solve timing prints | Source planner options **and/or** RH `debug=` that forwards | don’t duplicate blindly |
| Horizon overlays / plan history from rollout | `planning/mpc/` analysis helpers **or** `graphical` adapters called from demos | `mpc_plans_from_rollout`, `mpc_animation_overlays` stay **tools**, not methods on `Planner` |
| Plot last closed-loop vs plan | RH façade convenience (`plot_*` / `debug_*`) that calls graphical tools | optional sugar |

```mermaid
flowchart TB
  subgraph tools["Tools — planning/"]
    PP[PlanningProblem]
    PL[Planner / TOP]
    TP[TrajectoryPlan]
    WS[warm_start helpers]
    AN[overlays / plans_from_rollout]
  end

  subgraph product["Product — RecedingHorizonController"]
    LT[TickLatch]
    EXP[export_to_computer / matmul]
    CMD[compute_command]
  end

  subgraph host["Existing host"]
    COMP[Computer]
    HYB[HybridSimulator / HybridDiagram]
    GFX[graphical overlays]
  end

  PP --> PL
  PL --> TP
  PL -->|"solve_trajectory_from"| LT
  WS --> LT
  LT --> CMD
  LT --> EXP
  EXP --> COMP --> HYB
  AN --> GFX
  CMD -.->|demos/RAS| GFX
```

**Rule of thumb**

- If it makes a **better NLP / plan** → planner / transcription.
- If it makes a **better online tick / deploy / `@ plant`** → RH controller.
- If it **visualizes or post-processes** rollouts → keep as free functions /
  graphical helpers; RH may expose thin `debug_*` that call them.

---

## 5. End-to-end pipeline graphic

```mermaid
flowchart TD
  A["1. Build PlanningProblem<br/>sys, cost, sets, params, horizon T"]
  B["2. Build TrajectoryOptimizationPlanner<br/>transcription n_steps + options"]
  C{"Mode?"}
  D["prepare / JIT<br/>parametric"]
  E["solve_trajectory<br/>offline tune"]
  G["RecedingHorizonController(planner)<br/>dt_mpc, warm_start"]
  H{"Use?"}
  I["compute_command y<br/>calls solve_trajectory_from"]
  J["rhc @ plant<br/>HybridDiagram"]
  K["HybridSimulator<br/>plant traj + computer rollout"]
  L["Overlays / debug<br/>plans_from_rollout"]

  A --> B --> C
  C -->|parametric| D --> G
  C -->|batch| E --> G
  G --> H
  H --> I
  H --> J --> K --> L
  I --> L
```

---

## 6. Migration from today’s PoC

| Today | End goal |
| --- | --- |
| `MPCPlanner` | Parametric mode of `TrajectoryOptimizationPlanner` (+ thin alias) |
| `MPCStatelessController` / `MPCStatefulController` | Generated / replaced by RH `as_*_block` / export |
| `MPCTickLatch` → `u_ff`,`x_ff`,`z` | Latch holds `TrajectoryPlan`; expose `plan_flat` |
| Hand `while` closed-loop demos | Same NLP via `compute_command` or `rhc @ plant` |
| `mpc % dt` then `computer @ plant` | `rhc @ plant` (still builds `Computer` under the hood) |
| Scene baked at build | Later: `params.scene` / ObstacleBank (pipeline B) |

**Important sequencing choice (recommended)**

1. Land **RH façade + `TrajectoryPlan`** on **today’s `MPCPlanner`** by adding
   `solve_trajectory_from` (thin wrap of `step` → `TrajectoryPlan`).
2. Prove `compute_command` + `rhc @ plant` on `demo_mpc_hybrid_minimal`.
3. **Then** merge compile-once into `TrajectoryOptimizationPlanner`.
4. Scene parametric / broadcast / other planners’ `solve_trajectory_from` after that.

Do **not** block aim UX on the planner merge.

---

## 7. Implementation phases (for approval)

### Phase 0 — Plan approval (this doc)

- [ ] Agree vision diagrams and roles (§1–§4)
- [ ] Agree default names: `RecedingHorizonController` (+ optional `MPCController` alias)
- [ ] Agree default applied port: `u_ff` (ZOH); broadcast deferred
- [x] Agree **no `HorizonSource`** — RH takes planner with `solve_trajectory_from`
- [ ] Agree Phase 1 uses `solve_trajectory_from` on `MPCPlanner`, merge into TOP later

### Phase 1 — Result contract + RH façade v0

**Deliverables**

1. Optional `PlanningProblem.horizon` (\(T\)); trajopt/MPC resolve grid with
   problem \(T\) preferred, else `options.tf` (no demo breakage)
2. `SolveMetadata`, `TrajectoryPlan` (+ `to_flat` / `from_flat` minimal)
3. **Rename `Planner.compute_solution` → `solve`** across library, tests, demos,
   benchmarks, README/DESIGN pointers (mechanical; include `PolicyEvaluator` if
   it shares the name). Wire `solve()` as dispatcher to typed methods as they land.
4. `MPCPlanner.solve_trajectory_from` → `TrajectoryPlan` (wrap `step`; `params` accepted, unused keys error later)
5. `RecedingHorizonController(planner, …)` with:
   - `compute_command`
   - `export_to_computer` / `__matmul__`
   - latch holding full plan; ports include `plan_flat` (and keep `u_ff`/`x_ff`/`z`)
6. Migrate `examples/scripts/hybrid/demo_mpc_hybrid_minimal.py` to
   `PlanningProblem(..., horizon=T)` + `rhc @ plant` (`n_steps` still on options)
7. Tests: command vs `step` parity; warm-start parity; horizon resolve
   (problem-only / options-only / conflict)

**Exit:** one source of truth for tick API; \(T\) vs \(N\) split visible in one
demo; PoC leaves can remain temporarily.

### Phase 2 — Planner merge (parametric TOP)

- Fold `MPCPlanner` prepare/step into `TrajectoryOptimizationPlanner`
  parametric mode (`solve_trajectory_from`)
- Transcription options emphasize `n_steps`; `tf` becomes fallback / deprecated
  as task source
- Deprecate public `MPCPlanner` (alias OK pre-1.0)
- Sync DESIGN §6 / ROADMAP / standard-planning-problems (\(T\) vs \(N\)); update demos

### Phase 3 — Observability polish

- RH `debug` / telemetry hooks (`get_solve_metadata`, last plan)
- Keep overlays as tools; optional façade helpers
- Document RAS wrap pattern (no ROS2 package yet)

### Phase 4 — Parametric scene + optional varying \(T\) (pipeline B / R12)

- `ProblemParameters.scene`, `ObstacleBank`
- `solve_trajectory_from(..., params=)` without re-JIT
- Optional RH input ports for scene
- Perception demo
- Design note / optional spike: runtime \(T\) with **fixed \(N\)** (time-scale
  param) for varying-horizon RH — not required to close Phase 4

### Phase 5 — Broadcast + swappable backends

- Multi-rate `BroadcastBlock` (`u_nom`/`x_nom`)
- `solve_trajectory_from` on batch trajopt / RRT (smoke)
- DP via `solve_trajectory_from` + rollout only if needed

---

## 8. Package placement (proposed)

| Piece | Home |
| --- | --- |
| `PlanningProblem`, `ProblemParameters` | `planning/problems.py` (exists) |
| `Planner` | `planning/planner.py` (exists) |
| `TrajectoryOptimizationPlanner` | `planning/trajectory_optimization/` |
| `TrajectoryPlan`, `SolveMetadata` | `planning/results.py` (new) or `planning/plans.py` |
| `RecedingHorizonController`, `Command`, latch | `planning/mpc/` (keep package; broaden beyond NLP-only) |
| Warm-start / overlays | stay under `planning/mpc/` as helpers until merge |
| Quarantine | none for this work |

Dependency law: RH façade may import `Computer` / hybrid composition (same
as today’s MPC export). Planners must not grow hybrid/ROS imports.

---

## 9. Explicitly out of scope (this plan)

- Stochastic / robust problem classes ([standard-planning-problems.md](standard-planning-problems.md))
- Tracker / FF–FB law library (expose ports only)
- ROS2 package, acados, shooting-MPC transcription
- Hard moving-obstacle constraints in first scene pass
- Forking `TrajectoryPlanner` / `PolicyPlanner` ABCs

---

## 10. Open decisions (checkboxes for sign-off)

Please mark / reply before implementation:

1. [ ] **Name:** ship `RecedingHorizonController` as public; `MPCController` as alias?
2. [ ] **Phase 1 kernel:** adapter on today’s `MPCPlanner` before TOP merge?
3. [ ] **Default `@` applied signal:** `u_ff` (ZOH), not `u_nom` yet?
4. [ ] **Flatten:** methods on `TrajectoryPlan` (not bare `Trajectory`)?
5. [x] **No `HorizonSource`:** RH holds planner with `solve_trajectory_from` (locked)
6. [ ] **PoC leaves:** keep `MPCStateless/StatefulController` until Phase 2, then generate-from-façade or delete?
7. [ ] **Doc sync:** update DESIGN/ROADMAP only after Phase 1 lands (vs with this approval)?
8. [x] **Continuous \(T\):** optional `PlanningProblem.horizon`; **`n_steps` stays on transcription** (locked by this revision)
9. [ ] **Conflict rule** when both `problem.horizon` and `options.tf` set: problem wins vs hard error?
10. [x] **Mother 2×2 + `solve()`:** typed `solve_trajectory` / `solve_policy` / `*_from`; short offline **`solve()`** replaces `compute_solution` (repo-wide rename, ~40–60 sites); no `compute_solution` alias (locked)
10b. [x] **Dual result slots:** `last_trajectory_plan` + `last_policy_plan` (not one `last_result`); DP may hold policy and rolled-out traj together (locked)
10c. [x] **`TrajectoryPlan` rates:** optional reserved `x_dot` / `u_dot` (default `None`); not required at solve; ctl-rate eval stays on broadcast (locked)
11. [x] **TOP modes:** `compile_mode="batch"|"parametric"`; `solve_trajectory_from(..., params=None)` façade for Phase 4 (locked)
12. [x] **Merge:** Phase 1 `solve_trajectory_from` on `MPCPlanner`; Phase 2 fold into parametric TOP (locked)

---

## 11. Approval summary

When this doc is approved, engineering starts at **Phase 1** with the
vision locked as:

```text
PlanningProblem (incl. continuous T)  →  Planner (TOP ± parametric)  →  TrajectoryPlan
         n_steps on transcription only         ↓ solve_trajectory_from(x0)
                                    RecedingHorizonController(planner)  →  Command / Computer / @ plant
```

Reply with decisions on §10 (or “approve all defaults”) to proceed.
