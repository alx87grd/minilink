# Minilink Roadmap

Maturity and priorities — the **plan of record**. Identity:
[CONSTITUTION.md](CONSTITUTION.md). Contracts: [DESIGN.md](DESIGN.md).
Code rules: [RULES.md](RULES.md). Agent workflow: [AGENTS.md](AGENTS.md).
Operational backlog (step-level): [docs/plans/TODO.md](docs/plans/TODO.md).
Point-in-time audits: [docs/reviews/](docs/reviews/).

## 1. Releases

Identity: [CONSTITUTION.md](CONSTITUTION.md).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end**, plus a working **`pip install minilink` on PyPI**. Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via native `ReinforcementLearningPlanner` (with `Sys2Gym` + SB3 as an optional bridge). See §4.1. | Fall 2026 — **0.1.0 cut**; the `0.1.0` tag publishes the wheel from GitHub. Conda from `environment.yml` stays the Full local stack. Names the course notebooks already use stay frozen. |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2), **pyro parity + the GMC714 modelling ladder** (manipulators, four-rung vehicle ladder, robotic controllers), and the deferred v0.1 items in §5 Phase 2. Conda stays the recommended Full local install. | Winter 2027 |
| **v1.0** | The foundation questions deferred in §6 (hybrid as a `System`, evaluator/solver layering, posed-geometry drawing hook), after two cohorts. Workspace-geometry unification is designed ([geometry-module.md](docs/plans/geometry-module.md); implement when scheduled). | 2027 |

## 2. Two lanes

Philosophy: [CONSTITUTION.md](CONSTITUTION.md) §3 (the student syntax never
breaks). Placement shorthand: [RULES.md](RULES.md) §3.3. The operating
contract is the table below — not a pointer.

Minilink serves two audiences with one codebase. The boundary between them is
a **contract**, not a documentation convention.

| Lane | What it is | Rules |
| --- | --- | --- |
| **Teaching surface** | The names students meet: root prelude (`from minilink import …`) and the band facades (`minilink.catalog`, `.blocks`, `.control`, `.analysis`, `.simulation`, `.planning`). Registered in one place, tested as a set. | *Soft rule:* nothing enters without a demo or notebook, a both-backends test where it defines dynamics, and a docstring. Names and semantics change only with a deprecation note. Student-facing examples and notebooks import **only** through it (CI-checked). |
| **Research lane** | Everything else: provisional bands (hybrid, MPC, realtime, spatial), the `minilink/experimental/` tier (symbolic mechanics, contact engines, C export), `examples/projects/`, `examples/experimental/`. | No stability promise, no entry requirements, importable from a git checkout. Stays **out of the wheel** and out of the release contract. Deep imports always remain valid. |

**Wheel scope:** the published package ships the teaching surface plus the
provisional planning/MPC/hybrid bands. `experimental/`, projects, and
sandbox are repo-only.

**Deprecations.** 2026-09-15: `TrajectoryPlan`, `PolicyPlan`, `SolveMetadata`
and `MonteCarloReport` are removed without aliases; every planner returns a
`PlanningSolution` (`policy`, `solver`, `trajectory`, `evaluation`,
`cost_to_go`) and the evaluator an `Evaluation`. Read `solution.trajectory`
where you read `plan.trajectory`, `solution.solver` where you read
`plan.metadata`; `solve(evaluate=True)` fills the trajectory and evaluation a
solver does not compute natively. No course notebook imported the retired
names.

2026-09-15 (same day): `PlanningProblem.on_exit` is removed and `exit_cost` is
renamed `infeasible_cost`; `X` is unconstrained unless declared (it defaulted to the
state bounds), `U` still defaults to the input ports' box. Files that meant the state
bounds as a constraint now say `X=plant.state.box`; the RL planner's new
`training_zone` (the state box by default) keeps every RL demo's training unchanged.

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
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested. Shape validation at compile landed (S02). | Derived `x0` (v1.0). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; ~30 unreferenced integration helpers and `_jit` aliases still on the surface (S16/S17 kept). Speed lives in batches: 1000 rollouts × 1000 RK4 steps in 27 ms; a single jitted `f` call is no faster than NumPy. Float64 on JAX evaluators landed (S05). | Profile `rollout_batch` with a `params` family (S53); re-layering deferred to v1.0. |
| Simulation | teaching | 7 | Mature workflow; stable solvers/forcing. Fixed output count and `verbose` names landed (S01, S07). | — |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA. **Every catalog plant compiles on both backends** (`xp` sweep 2026-09-06, contract test `test_catalog_backends.py`); `JaxCartPole` retired. Four-rung vehicle teaching ladder landed (S25). | — |
| Control | teaching | 6 | Linear, LQR, `P` / `PI` / `PD` / `PID`; model-based SMC; robotic impedance/kinematic. **Each compensator form carries only the states its terms need** (landed 2026-09-07), so pole and zero counts match the hand calculation. `trajectory_lqr` landed 2026-09-15; `LQRPlanner` (the Riccati design as a planner returning a `PlanningSolution` with the cost-to-go) landed 2026-09-17. | `place()` for GRO501 (v0.2); robotic PID wrappers. |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with margins, pole-zero, root locus, Nyquist, step response — matplotlib and plotly. **The automatic frequency band brackets the 0 dB crossing** (landed 2026-09-07), so integrator and high-gain loops no longer report infinite margins. | `minreal`; named `S`/`T`/`PS`/`CS`; Nichols chart; multi-system overlays. Discrete-time (z) plots held. |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources, TF, 1-layer NN. | `Sine`/`Ramp`/`Chirp`/`Delay`/`Switch` (v0.2). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration, `loop`/`numpy`/`jax` backends, lookup controller, `PolicyEvaluator`; returns a `PlanningSolution` (lookup policy, `ValueIterationRecord`, `cost_to_go`; 2026-09-15). Notebooks wire with `vi_ctl @ plant`. `PlanningSolution` carries its `problem` and draws what the planner produced (`plot_control_law`, `plot_cost_to_go`, `plot_trajectory`); `compare(...)` reads solutions side by side (2026-09-17). | `plot_cost2go` colour scale clipped at the out-of-bound cost by default (S54). |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot; returns a `PlanningSolution` with a linear `TrajectorySource` policy and a `TrajectoryOptimizationRecord` (`success` = defects satisfied to `feasibility_tol`). JAX evaluators enable float64 on construction (`MINILINK_JAX_X64=0` opts out). | Harden SciPy/Ipopt before TRL 6; multiple-shooting parametric guard landed. |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt. | Harden SciPy/Ipopt before TRL 6. |
| Interfaces / RL bridge | research lane (bridge) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call (jitted under JAX when the plant traces, NumPy otherwise; Euler kept as an option). Kept as an optional external bridge; not the primary course path. | Keep module for external interop; course material uses native RL. |
| Analysis / Lyapunov certificates | provisional (research) | 4 | Landed 2026-09-11 from [lyapunov-certificates.md](docs/plans/lyapunov-certificates.md): `region_of_attraction` + `LyapunovCertificate` (`verify`, `plot`, `contains`), two `System` shortcuts, demo `analysis_region_of_attraction.py` and showcase §11 of `showcase_from_rl_to_bode.ipynb`. Quadratic `V` only; the level is a sampled estimate and the search reports `sample_limited` when halves disagree. | Student cohort validation; SOS and discrete-time later. |
| Planning / RL planner (`reinforcement_learning/`) | teaching (GRO860) | 6 | Landed 2026-09-10: cost horizon/discount + problem exit rule, `StochasticPlanningProblem` + distributions, `MonteCarloEvaluator`, `NeuralPolicyController` + `MLP`, `ReinforcementLearningPlanner` with PPO and SAC in pure JAX; verified on UR5 impedance showcase. Rebuilt 2026-09-15 on domain objects (`StochasticPolicy`, one collector, algorithms owning their state; bit-identical to the seeded baseline) and completed with the course's foundational algorithms: `REINFORCE`, `ActorCritic`, and `TabularLearningPlanner` (`QLearning`, `SARSA`, `MonteCarloControl`, `EpsilonGreedy`, `UCB`) on the value-iteration grid, NumPy only. S48 landed 2026-09-15: both return a `PlanningSolution`, the evaluator an `Evaluation`. S46 landed 2026-09-16: the GRO860 names are on the root prelude. | Canonical demos in `examples/demos/rl/` (pendulum, cart-pole, drone, rocket), intro chapter `11_reinforcement_learning.ipynb`, native twins `pendulum_value_iteration_vs_lqr_vs_rl.ipynb` and `drone_ppo.ipynb`. |
| Planning / search (RRT) | provisional | 5 | RRT/RRT*; spatial `Scene`. **`RRTPlanner(problem)` works from the input bounds alone** (bang-bang `KinodynamicExtender` default, 0.3 s edges; extenders and `RRTOptions` on the planning band, landed 2026-09-09). Returns a `PlanningSolution` with a held `TrajectorySource` policy and a `TreeSearchRecord` (2026-09-15). | RRT-Connect later. |
| Geometry / spatial | provisional | 4 | SDF + `Scene` / fields / bodies live under `planning/spatial/` today (JAX twins tested). **Home designed 2026-09-21:** [`docs/plans/geometry-module.md`](docs/plans/geometry-module.md) — package `core/geometry/` (Path, Track, Scene, bind, spatial Fields, course catalog); `planning.spatial` retires; control/analysis import it with no planner. Not started. | Package move + catalog; Bicubic SDF and CBF filter (`docs/plans/cbf-safety-filter.md`); glyph/solid naming split (S30, v1.0, graphical). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers. **Auto-fit camera** (`camera_scale=None`, the `System` default, landed 2026-09-09): hint-less plants frame the drawn geometry over the whole animation; backdrops (`ground_line`, `Plane`) and force glyphs excluded. | Renderer polish; matplotlib renderer coverage; constructor-derived `camera_scale` hints → auto or params-derived so `params` changes keep the framing. |
| Hybrid / step / MPC | provisional (research) | 4 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, MPC with parametric JAX. Research scaffold to exercise MPC; not the library core narrative. The sampled loop is the one thing that is not a `System`. | Keep names through the term; full promotion to official `HybridLoop` deferred to v1.0+. |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review. |
| Estimation | planned (GRO501) | 1 | Placeholder. **The largest single GRO501 gap** (§4.2). | Luenberger, then Kalman, as diagram blocks (v0.2). |
| Identification | planned | 2 | Parametric-tier prototype only. | `fitting.py` (v0.2); batched `rollout_batch` facade first. |
| C export (`experimental/c_export`) | research | 1 | Agent MVP: JAX→C transpiler; two demos; flagship smoke in the JAX regression job. Not a user-check, not autodoc, not the nightly demo sweep. | Keep isolated; repo-only. |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Experimental; not teaching path; not on the Sphinx API site. | Keep isolated; repo-only. |
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
| Value iteration / DP | `PlanningProblem`, `StateSpaceGrid`, `DynamicProgrammingPlanner`, `LookupTableController`, `plot_cost2go` / `plot_policy` | `pendulum_value_iteration`, `pendulum_value_iteration_vs_lqr`, `demos/value_iteration/` | `final_time` reads `problem.tf`; `success` reports convergence; notebooks wire with `vi_ctl @ plant` |
| LQR + linearization | `linearize`, `lqr`, `lqr_at_operating_point`, `plot_control_law` | `03_control`, `04_analysis`, `demos/control/` | — (green today) |
| Trajectory optimization | `PlanningProblem`, `TrajectoryOptimizationPlanner` (`direct_collocation`, `shooting`), `QuadraticCost` | `09_planning`, `demos/trajopt/` | float64 by default on JAX; `success` means defects satisfied; canonical problems succeed with default optimizer |
| Reinforcement Learning (tabular on the grid, neural in native JAX) | `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control; `EpsilonGreedy`, `UCB`), `StochasticPlanningProblem`, `ReinforcementLearningPlanner` (REINFORCE / actor-critic / PPO / SAC), `NeuralPolicyController`, `MonteCarloEvaluator` | `11_reinforcement_learning.ipynb`, `pendulum_value_iteration_vs_lqr_vs_rl.ipynb`, `drone_ppo.ipynb`, `gymnasium_interface.ipynb`, `showcase_from_rl_to_bode.ipynb`, `demos/rl/` (pendulum, cart-pole, drone, rocket) | pure-JAX training on compiled plant (measured 2026-09-12: pendulum swing-up, 120k steps in 12.5 s on CPU, 0% failure over 50 trials); neural policy closed loop linearizes, simulates, and plots Bode. **S46 landed:** those names (plus `Evaluation`, `Gaussian` / `Uniform`) are on the root prelude and the teaching-surface registry; `angle_features` stays on the control band. Notebooks solve with the native planner; Gymnasium stays taught as the domain standard and `Sys2Gym` + SB3 stay as the external bridge |

**Cross-cutting gates**

1. Wrong-shape `f` / `h` / port computes fail at `compile()` on both backends;
   a forgotten `super().__init__()` gives a named error; the README custom
   plant composes with `@`.
2. Default `compute_trajectory` returns a fixed output count (10 001, the fine
   reporting grid ruled 2026-09-07) and never selects a solver from the point count.
3. Every GRO860 notebook and every `examples/tutorial/`, `examples/teaching/`, and `examples/demos/`
   file imports only through the teaching surface (CI test).
4. The Basic tier (NumPy + SciPy + Matplotlib, nothing else) runs
   sim / plot / phase plane / animate / linearize / LQR / VI in a clean
   environment (CI test).
5. The Colab cell (git clone + path) and the conda environment from
   `environment.yml` both run every GRO860 notebook top to bottom.
6. `ruff` + `pytest` + notebook smoke green; nightly full demo sweep green.
7. No name a GRO860 notebook imports today changes during the term.
8. Tagging `0.1.0` publishes `minilink` to PyPI from GitHub (Trusted Publishing
   in `.github/workflows/publish.yml`). `pip install minilink` installs the
   teaching surface; extras (`[jax]`, `[visualization]`, …) match
   `pyproject.toml`. Conda remains the Full local stack (JAX, Ipopt, notebooks).

Out of the v0.1 checklist by decision: MPC/hybrid (provisional, lesson keeps
shipping), estimation, identification, frequency-domain tools (landed
2026-09-07 but gated by §4.2, not by v0.1).

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
| Loop-shaping specs — disturbance and measurement-noise sensitivity in dB at a frequency | `analysis` (v0.2) | scheduled v0.2 | named `S` / `T` / `PS` / `CS` sensitivity shortcuts on closed-loop diagrams |
| Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models) | continuous only, **z tier held** | the sampled loop is validated by simulation; a z tier stays out of v0.2 unless the sommatif examines z-plane analysis (§6) |
| State-space MIMO — bicycle model, controllability at every nominal speed | `KinematicBicycle`, `controllability`, `observability` | green | — |
| Optimal control — LQR on the guide's cost, closed-loop poles, nonlinear check | `lqr_at_operating_point`, `StateFeedbackController` | green | — |
| Pole placement — `K_sta` for a prescribed pole set | `control` (v0.2) | scheduled v0.2 | `place(A, B, poles)` returning a `StateFeedbackController` |
| Nested loops — inner speed loop, outer position loop | `@` composition | green (verified) | stays green with the observer in the loop |
| State estimation — Luenberger observer and Kalman filter | `estimation` (v0.2) | scheduled v0.2 | `LuenbergerObserver` and steady-state `KalmanFilter` closing the loop as standard diagram blocks |
| Reference scaling — the `N` matrix giving `y = r` at steady state | `control` (v0.2) | scheduled v0.2 | `steady_state_feedforward(sys)` or `N` matrix helper for tracking |

**Cross-cutting gates**

1. Every topic row above is green, with a demo or a notebook.
2. One `examples/teaching/` notebook per APP, Colab-first, Basic tier
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
`RRTPlanner` default extender; open items then listed in TODO §5 (later
renumbered; `c_export` nightly duplicate retired).

**Status 2026-09-16:** Sphinx site is autodoc of the teaching-lane API only
(`docs/api/`; experimental page dropped); the five-slide deck and
`docs/roadmap.rst` are gone. The user guide is `examples/tutorial/`. C export
is TRL 1. **PyPI is a v0.1 goal:** hatch-vcs versions the wheel, the wheel
excludes `experimental/`, and `.github/workflows/publish.yml` publishes from a
`0.*` tag via Trusted Publishing. Conda from `environment.yml` stays the Full
local stack.

| Phase | Scope | When |
| --- | --- | --- |
| **D — docs as plan of record** | ROADMAP, README, DESIGN, AGENTS, TODO, plans index, install/examples READMEs aligned to §1–§4. No Python changes. | now, 1–2 days |
| **0 — first-hour safety** | Default grid · shape validation · README example + `@` message · float64 policy · `super().__init__` guard · `verbose` flag names · nbstripout · trajopt `success` semantics. | immediately after D, ~1 agent-day |
| **1 — teaching contract + the GRO860 path** | Teaching-surface registry + Basic-tier clean-env test · import-layer CI check · `simulation`/`planning`/`core` band facades · rewrite imports in `learn/` then `demos/` · compiled `Sys2Gym.step` · DP metadata honesty · delete `_jit` aliases and unreferenced evaluator methods · wheel excludes research lane · `c_export` flagship smoke · nightly demo sweep · branch hygiene · **consolidation inventory** (duplicated code and parallel implementations, ranked; no feature removal — see below). | weeks 2–4 of the term |
| **2 — the JAX claim, and the research facade** | `xp` sweep of the NumPy-only catalog · both-backends contract test · retire `JaxCartPole` · four-rung vehicle ladder, `named_ports=` flag, research rungs → projects · `rollout_batch` for parameter-family sweeps · consolidation passes picked from the inventory. | rest of the term (v0.1.x) → v0.2 |
| **3 — foundations** | Derived `x0` · `core/geometry/` package ([geometry-module.md](docs/plans/geometry-module.md), designed 2026-09-21) · geometry glyph/solid rename · `HybridLoop` or promotion · mechanical-base unification · frequency tools (native or bridge, per §6) · iLQR from parts (idea) · Diffrax as optional JAX solver (later) · evaluator/solver re-layering · **RL as a planner** landed (2026-09-10; R7 retired the prototype 2026-09-15 — DESIGN / this TRL table). | after the term (v0.2 → v1.0) |

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
- **Open (v0.2):** the `estimation/` band (Luenberger observer, steady-state Kalman filter) — scheduled as the primary gap for the GRO501 wave (§4.2).
- ~~**v0.1:** PyPI publication~~ — settled 2026-09-16: `pip install minilink` is a v0.1 goal. The wheel is hatch-vcs versioned, excludes the research lane, and `.github/workflows/publish.yml` uploads from a `0.*` GitHub tag (Trusted Publishing). Conda stays the recommended Full local stack. First upload is the `0.1.0` tag after a PyPI trusted publisher is registered for `alx87grd/minilink` / `publish.yml` / environment `pypi`.
- **Open (after v0.2):** Zenodo archive and a citable DOI — tag a release, deposit it, add `CITATION.cff`. Sequenced after PyPI so a citation points at a release rather than a branch. This is also the JOSS entry ticket if that route is wanted: JOSS reviews the software against a checklist (install, docs, tests, license, contributing and issue guidelines) and requires a state-of-the-field section naming related tools — the one text that may, as a paper and not as repo prose, carry the landscape table from the 2026-09-12 editorial review. Decide the RULES 6.9 carve-out then, not before.
- **Open (v1.0):** `HybridDiagram` as a `System` (state `[plant; computer]`, periodic discrete update) vs an honest `HybridLoop` rename. Kept as research scaffold through v0.1.
- **Open (v1.0):** a single posed-geometry hook so "two functions" (`f` and a drawing function) is literal; today animation is `tf` plus skin geometry.
- **Designed (2026-09-21, not started):** workspace geometry lives in `core/geometry/` — Path, Track, Scene, bind, spatial Fields, a small course catalog; `planning.spatial` retires. Placement test is scoring a controller with no planner. [docs/plans/geometry-module.md](docs/plans/geometry-module.md). Glyph/solid rename (S30) and the posed-geometry drawing hook stay separate.
- **Open (v1.0):** evaluator/solver layering — evaluators keep pure maps and one scannable step; integrators move to `simulation/solvers/`; Diffrax as an optional JAX solve (later).
- **Open (v0.2 → v1.0):** a differentiable closed-loop cost, `J = F(problem params)` as one traced scalar — simulate the closed loop and integrate the plant's cost with every parameter an input, so `jax.grad` and `vmap` reach plant, controller, cost, sets and `x0` alike (today tutorial 11 and `pid_autotuning_jax` write that scan by hand). Needs first one parameter dictionary for a `PlanningProblem` — `system` nested by subsystem id, `cost`, `sets`, `x0` — with one owner per value; today `ProblemParameters` covers `system` / `cost` / `sets` and `x0` lives on the problem. The post-hoc path it extends landed 2026-09-13: `cost.total_cost(loop.trajectory_of(plant))`. First step, the cost dictionary nested like diagram params: design in docs/plans/cost-params.md.
- **Open (v0.2):** what the terminal cost `h(x_f, t_f)` means, one rule for every tool. Today each decides on its own: `CostFunction.total_cost` and `sys.compute_cost` always add `h` at the last sample; the Monte Carlo score and the RL environment add it only when a finite horizon is reached; trajectory optimization adds it at `tf`; value iteration starts from it (the terminal value for a finite horizon, an initial guess for an infinite one); `Sys2Gym` charges it when a task-defined termination is reached. So a cost with a nonzero `h` on an infinite-horizon task scores differently per tool (the rocket landing's constant `h = 100`, labeled "crash", is an exit penalty written as a terminal cost). To settle: whether `h` exists only for a finite horizon (and an infinite-horizon cost with nonzero `h` warns or raises), whether `total_cost` follows `horizon_kind`, and that exits are priced only by the problem's `exit_cost`.
- **Open (v0.2, before `estimation/`):** the disturbance convention. `disturbances={port: Distribution}` draws one value per control period and holds it, so its variance does not scale with `dt`; `WhiteNoise` carries `var` + `sample_period`, the same ambiguity. A Kalman filter needs the rule once: per-step covariance `Q_d`, or spectral density `Q_c` with `Q_d = Q_c / dt` (RULES 4.12 applies). Recorded in docs/reviews/2026-09-15-foundations-review.md (F9).
- **Open (v0.2):** one naming rule for blocks and signals — class name, display `name`, `id`, role keys (`ref`, `ctl`, `sys`), diagram keys, wire strings (`"ctl:u"`), params paths (`"sys.mass"`), state labels, diagram block labels (`Name::key`), plot titles and printed text. Settled 2026-09-13: string keys stay, with `id` as the explicit override (no variable-name or attribute magic). Audit 2026-09-13 (no text form for `print(sys)`, user subclasses named `DynamicSystem`, three closed-loop and shortcut naming styles, `sys` vs `plant` for one plant across loop kinds, mixed separators and display-name styles), six quick wins and the larger alignments: docs/plans/naming.md.
- **Open (v0.2):** which pyro demos the courses still need (drives the parity audit's remaining rows).
- **Open (v0.2):** Lyapunov certificates in the analysis band — [docs/plans/lyapunov-certificates.md](docs/plans/lyapunov-certificates.md): `region_of_attraction(sys)` returning a `LyapunovCertificate` with `contains`, `verify` (Monte Carlo inside the certified set) and `plot`, plus the two `System` shortcuts. Quadratic `V` only; provisional research lane pending cohort review and SOS study.
- ~~RL as a planner~~ — settled 2026-09-11: native pure-JAX `ReinforcementLearningPlanner` + `NeuralPolicyController` is the canonical GRO860 teaching path (`11_reinforcement_learning.ipynb`, `demos/rl/`, `showcase_from_rl_to_bode.ipynb`); `Sys2Gym` + SB3 retained as an external bridge in `interfaces/`. Extended 2026-09-12: the two paths coexist **by role**, not by replacement — notebooks *solve* with the native planner (it trains what SB3 tuning did not, and needs no pip install on Colab), while the Gymnasium environment interface is still *taught* as the standard of the domain, so the GRO860 week-5 lab and the `step` / `reset` exercise keep `Sys2Gym`. S46 (2026-09-16) put the RL names on the root prelude.
- **Open (v0.2 / Later):** Control Barrier Functions (CBF) & Spatial Safety Filters — [docs/plans/cbf-safety-filter.md](docs/plans/cbf-safety-filter.md): $C^1$ bicubic SDF interpolation in JAX, DCBF with slacks, HOCBF relative degree, and `CBFSafetyFilter` block.

## 7. Out of scope

By decision — see [CONSTITUTION.md](CONSTITUTION.md) §1.4:

Full Simulink parity (GUI, DAE, arbitrary multi-clock scheduling,
event-driven switching as a framework feature); becoming a multibody/contact
OS or a batched RL physics engine.

What *is* in scope but subsidiary: the step/hybrid path (`StepSystem`,
`StepDiagramSystem`, `Computer` with integer-divisor multi-rate schedules,
`HybridDiagram`, `HybridSimulator`) exists so discrete control laws can close
the loop on continuous plants. It is provisional, and the seam is narrower than
"outside the hierarchy": `StepSystem` and `StepDiagramSystem` subclass
`System`, while `Computer` and `HybridDiagram` do not. No v0.1 course row
depends on it, but MPC is the intro surface's hybrid exemplar (RULES 3.7), so
README, the showcases and the tutorial do present it. Live interaction is
`simulation/realtime/`.

## 8. Backlog homes

| Doc | Job |
| --- | --- |
| [docs/plans/TODO.md](docs/plans/TODO.md) | Step-level plan for Phases D–3, small fixes, Later ideas |
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Open pyro parity rows (v0.2) |
| [docs/plans/](docs/plans/) | Research-lane design writeups (see [plans README](docs/plans/README.md)) |
| [docs/reviews/](docs/reviews/) | Dated architecture audits and decision records |
