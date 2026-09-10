# Minilink Roadmap

Maturity and priorities — the **plan of record**. Contracts and product
identity: [DESIGN.md](DESIGN.md). Agent rules: [AGENTS.md](AGENTS.md).
Operational backlog (step-level): [docs/plans/TODO.md](docs/plans/TODO.md).
Point-in-time audits: [docs/reviews/](docs/reviews/).

## 1. North star

Minilink is the pyro successor for teaching dynamics, control, and optimal
control, and the research substrate of the maintainer's group. The promise:
**write the equations once, then simulate, analyze, control, plan, optimize
and learn on the same model.** A model is three functions of `(x, u, t; p)`:
`f` (dynamics), `h` (outputs, default `y = x`) and `tf` (body poses). Three
claims follow, each containing the previous:

1. **Block diagrams as readable Python, with graphics for free** — `+`, `>>`,
   `@`; from `f` and `tf` come simulation, plots, the same animation on four
   renderers and a keyboard game mode; the core is NumPy, SciPy, Matplotlib.
2. **One interface, every tool, textbook objects in between** — everything is
   a `System`; `PlanningProblem = sys + cost + boundaries` feeds every planner,
   `MathematicalProgram` → `Optimizer` is the one NLP shape, `Trajectory` is
   what every tool returns.
3. **The same `f` is differentiable and compiled** — exact Jacobians,
   sensitivity to the physics, gradients through a rollout, batches over
   parameter families, on NumPy or JAX.

The material that carries this pitch (README, `docs/pitch/`, the two showcase
notebooks) is a spec: wherever the code needs a workaround to make a slide
true, that is a priority. Full identity and landscape position:
[DESIGN.md — Product identity & scope](DESIGN.md#product-identity--scope).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end.** Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via `Sys2Gym` + SB3. See §4.1. | Fall 2026 — term in progress; hardening lands during the term without breaking names the course notebooks already use |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2), **pyro parity + the GMC714 modelling ladder** (manipulators, four-rung vehicle ladder, robotic controllers), the deferred v0.1 items in §5 Phase 2, and a `pip install minilink` option (conda stays the recommended local install). | Winter 2027 |
| **v1.0** | The foundation questions deferred in §6 (hybrid as a `System`, evaluator/solver layering, geometry unification), after two cohorts. | 2027 |

## 2. Two lanes

Minilink serves two audiences with one codebase. The boundary between them is
a **contract**, not a documentation convention.

| Lane | What it is | Rules |
| --- | --- | --- |
| **Teaching surface** | The names students meet: root prelude (`from minilink import …`) and the band facades (`minilink.catalog`, `.blocks`, `.control`, `.analysis`, `.simulation`, `.planning`). Registered in one place, tested as a set. | *Soft rule:* nothing enters without a demo or notebook, a both-backends test where it defines dynamics, and a docstring. Names and semantics change only with a deprecation note. Student-facing examples and notebooks import **only** through it (CI-checked). |
| **Research lane** | Everything else: provisional bands (hybrid, MPC, realtime, spatial), the `minilink/experimental/` tier (symbolic mechanics, contact engines, C export), `examples/projects/`, `examples/experimental/`. | No stability promise, no entry requirements, importable from a git checkout. Stays **out of the wheel** and out of the release contract. Deep imports always remain valid. |

**Wheel scope:** the published package ships the teaching surface plus the
provisional planning/MPC/hybrid bands. `experimental/`, projects, and
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
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; ~30 unreferenced integration helpers and `_jit` aliases still on the surface. Speed lives in batches: 1000 rollouts × 1000 RK4 steps in 27 ms; a single jitted `f` call is no faster than NumPy. | Delete the unused grid; float64 policy; profile `rollout_batch` with a `params` family (10× slower than the plain batch, 2026-09-09); re-layering deferred to v1.0. |
| Simulation | teaching | 7 | Mature workflow; stable solvers/forcing. | Fixed output count by default; unify `verbose` flag names. |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA. **Every catalog plant compiles on both backends** (`xp` sweep 2026-09-06, contract test `test_catalog_backends.py`); `JaxCartPole` retired. | Four-rung vehicle teaching ladder, research rungs → projects. |
| Control | teaching | 6 | Linear, LQR, `P` / `PI` / `PD` / `PID`; model-based SMC; robotic impedance/kinematic. **Each compensator form carries only the states its terms need** (landed 2026-09-07), so pole and zero counts match the hand calculation. | `place()` for GRO501 (v0.2); robotic PID wrappers; traj LQR (v0.2). |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with margins, pole-zero, root locus, Nyquist, step response — matplotlib and plotly. **The automatic frequency band brackets the 0 dB crossing** (landed 2026-09-07), so integrator and high-gain loops no longer report infinite margins. | `minreal`; named `S`/`T`/`PS`/`CS`; Nichols chart; multi-system overlays. Discrete-time (z) plots held. |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources, TF, 1-layer NN. | `Sine`/`Ramp`/`Chirp`/`Delay`/`Switch` (v0.2). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration, `loop`/`numpy`/`jax` backends, lookup controller, `PolicyEvaluator`. | Honest `final_time` / `success` metadata; `vi_ctl @ plant` in notebooks; `plot_cost2go` colour scale clipped at the out-of-bound cost by default. |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot. **`success` echoes solver status; float32 by default on JAX.** | float64 policy; `success` = defects satisfied; multiple-shooting parametric guard. |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt. | Harden SciPy/Ipopt before TRL 6. |
| Interfaces / RL bridge | teaching (GRO860) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call (jitted under JAX when the plant traces, NumPy otherwise; Euler kept as an option). | Vectorized envs; re-train the PPO notebooks on the RK4 step. |
| Planning / search (RRT) | provisional | 5 | RRT/RRT*; spatial `Scene`. **`RRTPlanner(problem)` works from the input bounds alone** (bang-bang `KinodynamicExtender` default, 0.3 s edges; extenders and `RRTOptions` on the planning band, landed 2026-09-09). | RRT-Connect later. |
| Geometry / spatial | provisional | 4 | SDF + `Scene` / fields / bodies; JAX twins tested. | Glyph/solid naming split (v1.0). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers. **Auto-fit camera** (`camera_scale=None`, the `System` default, landed 2026-09-09): hint-less plants frame the drawn geometry over the whole animation; backdrops (`ground_line`, `Plane`) and force glyphs excluded. | Renderer polish; matplotlib renderer coverage; constructor-derived `camera_scale` hints → auto or params-derived so `params` changes keep the framing. |
| Hybrid / step / MPC | provisional | 5 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, MPC with parametric JAX. Not a `System`; not a GRO860 topic. Pitch-visible seam: the sampled loop is the one thing that is not a `System` (no `linearize`, no nesting). | Keep names through the term; `HybridLoop` / promotion question at v1.0. |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review. |
| Estimation | planned (GRO501) | 1 | Placeholder. **The largest single GRO501 gap** (§4.2). | Luenberger, then Kalman, as diagram blocks (v0.2). |
| Identification | planned | 2 | Parametric-tier prototype only. | `fitting.py` (v0.2); batched `rollout_batch` facade first. |
| C export (`experimental/c_export`) | research | 2 | Experimental JAX→C transpiler; two demos pass locally; not in CI. On the pitch deck as *experimental* since 2026-09-09. | Repo-only; add to the nightly sweep (pitch-visible). |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Experimental; not teaching path. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later (`interfaces/mjx.py`). |
| Pyro 2.0 overall | v0.2 | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md). |

## 4. Course objectives

Two courses drive the release contract: GRO860 (§4.1, v0.1, running now) and
GRO501 (§4.2, v0.2, parallel objective adopted 2026-09-07). A course is
"end to end" when every topic row is green on the teaching surface — in Colab
(git-clone cell) and in the conda env — and the cross-cutting gates hold.

### 4.1 v0.1 — GRO860 end to end

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
shipping), estimation, identification, frequency-domain tools (landed
2026-09-07 but gated by §4.2, not by v0.1), PyPI publication (later; conda
remains the local path).

### 4.2 v0.2 — GRO501 end to end

The classical-control course (*Systèmes asservis*, BSc Génie Robotique):
an APP problem on the UdeS-Racecar. APP2 is multi-physics modelling of the
DC-motor propulsion plus SISO speed and position loops designed to time- and
frequency-domain specifications and implemented as difference equations on an
Arduino; APP4 is the bicycle-model MIMO plant with LQR, nested loops, pole
placement and a Kalman observer. Student guide: `GRO501_Guide.pdf`
(maintainer's copy); references Dorf & Bishop and Åström & Murray.

Baseline audit 2026-09-07 on `dev-fable`:
[docs/reviews/2026-09-07-gro501-coverage.md](docs/reviews/2026-09-07-gro501-coverage.md).
Roughly four-fifths of the course runs today; the gaps below are the release
contract. Step-level work:
[docs/plans/gro501-classical-control.md](docs/plans/gro501-classical-control.md).

| Topic | Surface | Status | Gate |
| --- | --- | --- | --- |
| Multi-physics modelling — nonlinear `f`/`h`, block diagram, linearize, `H(s)` | custom `DynamicSystem`, `plot_diagram`, `linearize`, `transfer_function` | green | a DC-motor + longitudinal-vehicle plant in the catalog; order reduction (`minreal`) available |
| Closed-loop analysis — poles, root locus, Bode, margins, step specs | `pzmap`, `root_locus`, `bode`, `margins`, `step_info`, `P` / `PI` / `PD` / `PID` | green | met 2026-09-07: every compensator form reports the poles and zeros of the hand calculation, and margins are found wherever the crossover sits |
| Design to specification — rise time, overshoot, final error, phase margin | `PI`, `PD`, `PID`, `Lead`, `Lag`, `step_info`, `margins` | green | the Table 2 specs of the guide are checkable in one notebook |
| Loop-shaping specs — disturbance and measurement-noise sensitivity in dB at a frequency | reachable today via `of="block:port"` | needs a named verb | `S` / `T` / `PS` / `CS` are one call on a closed-loop diagram |
| Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models) | continuous only, **z tier held** | the sampled loop is validated by simulation; a z tier stays out of v0.2 unless the sommatif examines z-plane analysis (§6) |
| State-space MIMO — bicycle model, controllability at every nominal speed | `KinematicBicycle`, `controllability`, `observability` | green | — |
| Optimal control — LQR on the guide's cost, closed-loop poles, nonlinear check | `lqr_at_operating_point`, `StateFeedbackController` | green | — |
| Pole placement — `K_sta` for a prescribed pole set | — | missing | `place(A, B, poles)` returning a `StateFeedbackController` |
| Nested loops — inner speed loop, outer position loop | `@` composition | green (verified) | stays green with the observer in the loop |
| State estimation — Luenberger observer and Kalman filter | `minilink/estimation/` placeholder | missing, **held** | an observer block that closes the loop on a diagram and simulates with measurement noise; not scheduled — the release contract will need revisiting if it stays out |
| Reference scaling — the `N` matrix giving `y = r` at steady state | — | missing | a helper or a documented recipe |

**Cross-cutting gates**

1. Every topic row above is green, with a demo or a notebook.
2. One `examples/learn/teaching/` notebook per APP, Colab-first, Basic tier
   (NumPy + SciPy + Matplotlib) — no JAX on the GRO501 path.
3. Every tool agrees with the hand calculation for the guide's §9 exercises;
   the closed-loop analysis exercises (§9.6, §9.7) are the acceptance test.
4. Student-facing GRO501 material imports only through the teaching surface.

Out of the GRO501 checklist by decision: hardware, ROS, and the Arduino
firmware itself (the course owns those); system identification from logged
runs beyond what `identification/` already offers.

## 5. Phases

Step-level specs, files, and "done when" criteria live in
[docs/plans/TODO.md](docs/plans/TODO.md); the audit trail is in
[docs/reviews/](docs/reviews/).

**Status 2026-09-06 (`dev-fable`):** Phases D, 0, 1 and 2 complete — the review pass, the organisation pass (`minilink/experimental/`, `lazy_facade`, demo folders keyed to the intro chapters), S33 (compiled `Sys2Gym` step), S38 (DP metadata), and Phase 2 in full: the `xp` sweep (every catalog plant on both backends, contract test with an empty NumPy-only list), `JaxCartPole` retired, the four-rung vehicle ladder with research rungs in `examples/projects/car_trajopt/vehicles/`, and `rollout_batch`. S16/S17 closed as keep. Next: Phase 3 (after the term).

**Status 2026-09-09:** pitch material landed (README, five-slide deck as the
docs landing page, both showcases rebuilt); from the pitch: auto-fit camera,
`RRTPlanner` default extender; open items listed in TODO §5 (S41–S45).

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
- ~~Frequency analysis — NumPy-only vs a python-control bridge~~ — NumPy-only, landed 2026-09-07: `pzmap`, `nyquist`, `margins`, `root_locus`, `step_response` on the state-space channel (`analysis/linear.py`), plots on matplotlib and plotly.
- ~~`PID` spurious modes~~ — settled 2026-09-07: dedicated `PI` and `PD` classes carry only the states their terms need (`ProportionalController` already covered P); `PID` keeps its fixed `2n` layout so its gains stay tunable from zero. `minreal` still wanted for the general case (P2).
- **Held (v0.2):** discrete-domain scope for GRO501 — a z-domain tier in `analysis/` (ZOH/Tustin, z-plane `pzmap`, discrete Bode) vs teaching the Arduino law with `discretize` + simulation only. Maintainer paused this 2026-09-07; default is simulation only.
- **Held (v0.2):** the `estimation/` band (Luenberger, Kalman). Paused 2026-09-07; it is the last unmet §4.2 row.
- **Open (v0.2):** PyPI publication — wanted eventually as a third install option; conda stays recommended.
- **Open (v1.0):** `HybridDiagram` as a `System` (state `[plant; computer]`, periodic discrete update) vs an honest `HybridLoop` rename. Pitch-visible since 2026-09-09.
- **Open (v1.0):** a single posed-geometry hook so the pitch's "two functions" (`f` and a drawing function) is literal; today animation is `tf` plus skin geometry.
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
