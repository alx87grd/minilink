# Minilink Roadmap

Maturity and priorities — the **plan of record**. Contracts and product
identity: [DESIGN.md](DESIGN.md). Agent rules: [AGENTS.md](AGENTS.md).
Operational backlog (step-level): [docs/plans/TODO.md](docs/plans/TODO.md).
Point-in-time audits: [docs/reviews/](docs/reviews/).

## 1. North star

Minilink is the pyro successor for teaching dynamics, control, and optimal
control — readable equations, arbitrary diagrams, one `f` that simulates,
linearizes, optimizes, and differentiates on NumPy or JAX — and the research
substrate of the maintainer's group. Full identity and landscape position:
[DESIGN.md — Product identity & scope](DESIGN.md#product-identity--scope).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end.** Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via `Sys2Gym` + SB3. See §4. | Fall 2026 — term in progress; hardening lands during the term without breaking names the course notebooks already use |
| **v0.2** | **Pyro parity + the GMC714 modelling ladder** (manipulators, four-rung vehicle ladder, robotic controllers), the deferred v0.1 items in §5 Phase 2, frequency-analysis completion (decision in §6), and a `pip install minilink` option (conda stays the recommended local install). | Winter 2027 |
| **v1.0** | The foundation questions deferred in §6 (hybrid as a `System`, evaluator/solver layering, geometry unification), after two cohorts. | 2027 |

## 2. Two lanes

Minilink serves two audiences with one codebase. The boundary between them is
a **contract**, not a documentation convention.

| Lane | What it is | Rules |
| --- | --- | --- |
| **Teaching surface** | The names students meet: root prelude (`from minilink import …`) and the band facades (`minilink.catalog`, `.blocks`, `.control`, `.analysis`, `.simulation`, `.planning`). Registered in one place, tested as a set. | *Soft rule:* nothing enters without a demo or notebook, a both-backends test where it defines dynamics, and a docstring. Names and semantics change only with a deprecation note. Student-facing examples and notebooks import **only** through it (CI-checked). |
| **Research lane** | Everything else: provisional bands (hybrid, MPC, realtime, spatial), quarantine (`symbolic/`, `dynamics/engines/`), `interfaces/c_export`, `examples/projects/`, `examples/sandbox/`. | No stability promise, no entry requirements, importable from a git checkout. Stays **out of the wheel** and out of the release contract. Deep imports always remain valid. |

**Wheel scope:** the published package ships the teaching surface plus the
provisional planning/MPC/hybrid bands. Quarantine, `c_export`, projects, and
sandbox are repo-only.

## 3. Maturity (TRL)

Readiness levels are an internal maturity scale for planning and review — not
a release process by themselves.

| Level | Name | Description |
| --- | --- | --- |
| **TRL 1** | Agent MVP | Initial code exists and works |
| **TRL 2** | User-check MVP | User performs a high-level functional review |
| **TRL 3** | Architecture Validated | High-level architecture is approved |
| **TRL 4** | Integration Proposed | Final integration/refactor is proposed |
| **TRL 5** | Integration Validated | User approves main-codebase integration |
| **TRL 6** | Automated Tests Pass | Final pytest coverage exists and passes |
| **TRL 7** | Details Validated | Naming and implementation details are approved |
| **TRL 8** | Demo Released | Demo script is created and validated |
| **TRL 9** | Mission Complete | Tests, demo, and user approval are all complete |

| Area | Lane | TRL | Rationale | Next |
| --- | --- | --- | --- | --- |
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested. | Shape validation at compile; derived `x0` (v1.0). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; ~30 unreferenced integration helpers and `_jit` aliases still on the surface. | Delete the unused grid; float64 policy; re-layering deferred to v1.0. |
| Simulation | teaching | 7 | Mature workflow; stable solvers/forcing. | Fixed output count by default; unify `verbose` flag names. |
| Dynamics (abstraction + catalog) | teaching | 6 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA. **30 of 49 catalog plants are NumPy-only.** | `xp` sweep to dual-backend; both-backends contract test; four-rung vehicle teaching ladder, research rungs → projects. |
| Control | teaching | 6 | Linear, LQR, PID; model-based SMC; robotic impedance/kinematic. | Robotic PID wrappers; traj LQR (v0.2). |
| Analysis | teaching | 5 | Linearize, structural, equilibria, modal, SISO Bode. | Frequency completion — `pole_zero_map()`, `nyquist()`, `margin()`, `ss2tf()` as minimal NumPy tools, or a python-control bridge; decision postponed to v0.2 (§6). |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources, TF, 1-layer NN. | `Sine`/`Ramp`/`Chirp`/`Delay`/`Switch` (v0.2). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration, `loop`/`numpy`/`jax` backends, lookup controller, `PolicyEvaluator`. | Honest `final_time` / `success` metadata; `vi_ctl @ plant` in notebooks. |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot. **`success` echoes solver status; float32 by default on JAX.** | float64 policy; `success` = defects satisfied; multiple-shooting parametric guard. |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt. | Harden SciPy/Ipopt before TRL 6. |
| Interfaces / RL bridge | teaching (GRO860) | 3 | `Sys2Gym` + `SB3Controller`; step is a Python forward-Euler loop. | Compiled `step` on `integrate_zoh` (optional JAX) for training speed. |
| Planning / search (RRT) | provisional | 4 | RRT/RRT*; spatial `Scene`. | RRT-Connect later. |
| Geometry / spatial | provisional | 4 | SDF + `Scene` / fields / bodies; JAX twins tested. | Glyph/solid naming split (v1.0). |
| Graphics / animation | teaching | 4 | Frame-keyed `tf` / geometry / overlays; four renderers. | Renderer polish; matplotlib renderer coverage. |
| Hybrid / step / MPC | provisional | 5 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, MPC with parametric JAX. Not a `System`; not a GRO860 topic. | Keep names through the term; `HybridLoop` / promotion question at v1.0. |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review. |
| Estimation | planned | 1 | Placeholder. | Luenberger, Kalman (v0.2). |
| Identification | planned | 2 | Parametric-tier prototype only. | `fitting.py` (v0.2); batched `rollout_batch` facade first. |
| C export (`interfaces/c_export`) | research | 2 | Experimental JAX→C transpiler; two demos pass locally; not in CI. | Repo-only; add to the nightly sweep. |
| Quarantine (symbolic, hand-rolled contact) | research | 1 | Experimental; not teaching path. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later (`interfaces/mjx.py`). |
| Pyro 2.0 overall | v0.2 | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md). |

## 4. v0.1 — GRO860 end to end

Ready when every row is green on the teaching surface — in Colab (git-clone
cell) and in the conda env — and the cross-cutting gates hold.

| Topic | Surface | Material | Gate |
| --- | --- | --- | --- |
| Value iteration / DP | `PlanningProblem`, `StateSpaceGrid`, `DynamicProgrammingPlanner`, `LookupTableController`, `plot_cost2go` / `plot_policy` | `pendulum_swing_up_cost_function_vi`, `pendulum_swing_up_vi_vs_lqr`, `demos/planning/value_iteration/` | `final_time` reads `problem.tf`; `success` reports convergence; notebooks wire with `vi_ctl @ plant` |
| LQR + linearization | `linearize`, `lqr`, `lqr_at_operating_point`, `plot_control_law` | `03_control`, `04_analysis`, `demos/statespace/` | — (green today) |
| Trajectory optimization | `PlanningProblem`, `TrajectoryOptimizationPlanner` (`direct_collocation`, `shooting`), `QuadraticCost` | `09_planning`, `demos/trajopt/` | float64 by default on JAX; `success` means defects satisfied; canonical problems succeed with default optimizer |
| RL via `Sys2Gym` + SB3 | `Sys2Gym`, `SB3Controller`, `plot_control_law` | `drone_ppo_learn_to_fly`, `pendulum_swing_up_vi_vs_lqr_vs_ppo` | compiled `step`; PPO notebook trains to the same qualitative policy |

**Cross-cutting gates**

1. Wrong-shape `f` / `h` / port computes fail at `compile()` on both backends;
   a forgotten `super().__init__()` gives a named error; the README custom
   plant composes with `@`.
2. Default `compute_trajectory` returns a fixed output count (1 001) and never
   selects a solver from the point count.
3. Every GRO860 notebook and every `examples/learn/` and `examples/demos/`
   file imports only through the teaching surface (CI test).
4. The Basic tier (NumPy + SciPy + Matplotlib, nothing else) runs
   sim / plot / phase plane / animate / linearize / LQR / VI in a clean
   environment (CI test).
5. The Colab cell (git clone + path) and the conda environment from
   `environment.yml` both run every GRO860 notebook top to bottom.
6. `ruff` + `pytest` + notebook smoke green; nightly full demo sweep green.
7. No name a GRO860 notebook imports today changes during the term.

Out of the v0.1 checklist by decision: MPC/hybrid (provisional, lesson keeps
shipping), estimation, identification, frequency-domain tools (decision
postponed to v0.2), PyPI publication (later; conda remains the local path).

## 5. Phases

Step-level specs, files, and "done when" criteria live in
[docs/plans/TODO.md](docs/plans/TODO.md); the audit trail is in
[docs/reviews/](docs/reviews/).

**Status 2026-09-05 (evening session, `dev-fable`):** Phase D complete; Phase 0
complete (S03 dropped by decision); Phase 1 landed S11–S15, S18–S21, S41, S42 —
open: S16/S17 (evaluator names, your pick), S33 (`Sys2Gym` compiled step), S38
(DP metadata), S39 (demo headers — needs a joint editorial pass). Phase 2 not
started. Session log: [docs/reviews/2026-09-05-evening-session.md](docs/reviews/2026-09-05-evening-session.md).

| Phase | Scope | When |
| --- | --- | --- |
| **D — docs as plan of record** | ROADMAP, README, DESIGN, AGENTS, TODO, plans index, install/examples READMEs aligned to §1–§4. No Python changes. | now, 1–2 days |
| **0 — first-hour safety** | Default grid · shape validation · README example + `@` message · float64 policy · `super().__init__` guard · `verbose` flag names · nbstripout · trajopt `success` semantics. | immediately after D, ~1 agent-day |
| **1 — teaching contract + the GRO860 path** | Teaching-surface registry + Basic-tier clean-env test · import-layer CI check · `simulation`/`planning`/`core` band facades · rewrite imports in `learn/` then `demos/` · compiled `Sys2Gym.step` · DP metadata honesty · delete `_jit` aliases and unreferenced evaluator methods · wheel excludes research lane · `c_export` in nightly · nightly demo sweep · branch hygiene · **consolidation inventory** (duplicated code and parallel implementations, ranked; no feature removal — see below). | weeks 2–4 of the term |
| **2 — the JAX claim, and the research facade** | `xp` sweep of the NumPy-only catalog · both-backends contract test · retire `JaxCartPole` · four-rung vehicle ladder, `named_ports=` flag, research rungs → projects · `rollout_batch` for parameter-family sweeps · consolidation passes picked from the inventory. | rest of the term (v0.1.x) → v0.2 |
| **3 — foundations** | Derived `x0` · geometry glyph/solid rename · `HybridLoop` or promotion · mechanical-base unification · frequency tools (native or bridge, per §6) · PyPI option · iLQR from parts (idea) · Diffrax as optional JAX solver (later) · evaluator/solver re-layering. | after the term (v0.2 → v1.0) |

**Simplify and consolidate — a standing principle, not a phase.** The repo
must stay manageable by one maintainer, so every phase carries consolidation
work — but the target is *maintenance cost*, never features, and never line
count for its own sake. Clean, well-placed code is not a liability; a
deliberate ladder of implementations is not duplication (the DP planner's
`loop` / `numpy` / `jax` backends are three teaching and research tiers and
stay). What is a liability: text that must be edited twice when the code
changes (long header comments on demo scripts restating what the code below
does — the code and its inline comments should tell the whole story), dead
API (unreferenced evaluator methods, `_jit` aliases), boilerplate classes a
flag would replace (`*Ports` twins), NumPy/JAX twin plants that one `xp`
body covers, research scenarios inside the teaching tree, and plan docs that
are already self-marked complete. Phase 1 produces a ranked inventory; the
maintainer picks each item; later phases execute only what was picked.

## 6. Review queue

Decisions that block or shape a milestone (maintainer sign-off). Settled
2026-09-05 unless marked open.

- ~~Public export policy for `minilink/__init__.py`~~ — band facades everywhere in student-facing material; deep imports stay valid.
- ~~Control-block contract~~ — landed; decision record folded into DESIGN.
- ~~Diagram validation as separate `validate()` vs inline wiring~~ — wiring validates at `connect()`; unconnected inputs are **nominal by design** (no warning).
- **Open (v0.2):** optional `KinematicModel` delegate — adopt or drop.
- ~~Dynamic bicycle module split / vehicle ladder~~ — four teaching rungs in `catalog/`, research rungs to `examples/projects/`.
- **Open (v0.2):** pyro game demos — port the rest to `simulation/realtime/` or explicitly drop.
- **Open (v0.2):** frequency analysis — minimal NumPy-only `pole_zero_map`/`nyquist`/`margin`/`ss2tf` in minilink (original intent) vs a python-control bridge. Postponed; not part of the v0.1 consolidation.
- **Open (v0.2):** PyPI publication — wanted eventually as a third install option; conda stays recommended.
- **Open (v1.0):** `HybridDiagram` as a `System` (state `[plant; computer]`, periodic discrete update) vs an honest `HybridLoop` rename.
- **Open (v1.0):** evaluator/solver layering — evaluators keep pure maps and one scannable step; integrators move to `simulation/solvers/`; Diffrax as an optional JAX solve (later).
- **Open (v0.2):** which pyro demos the courses still need (drives the parity audit's remaining rows).

## 7. Out of scope

By decision — see [DESIGN.md](DESIGN.md):

Full Simulink parity (GUI, DAE, arbitrary multi-clock scheduling,
event-driven switching as a framework feature); becoming a multibody/contact
OS or a batched RL physics engine.

What *is* in scope but subsidiary: the step/hybrid path (`StepSystem`,
`StepDiagramSystem`, `Computer` with integer-divisor multi-rate schedules,
`HybridDiagram`, `HybridSimulator`) exists so discrete control laws can close
the loop on continuous plants; it is provisional, lives outside the `System`
hierarchy today, and is not a v0.1 teaching topic. Live interaction is
`simulation/realtime/`.

## 8. Backlog homes

| Doc | Job |
| --- | --- |
| [docs/plans/TODO.md](docs/plans/TODO.md) | Step-level plan for Phases D–3, small fixes, Later ideas |
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Open pyro parity rows (v0.2) |
| [docs/plans/](docs/plans/) | Research-lane design writeups (see [plans README](docs/plans/README.md)) |
| [docs/reviews/](docs/reviews/) | Dated architecture audits and decision records |
