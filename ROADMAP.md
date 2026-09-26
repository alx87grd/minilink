# Minilink Roadmap

Maturity and priorities — the **plan of record**. Identity:
[CONSTITUTION.md](CONSTITUTION.md). Contracts: [DESIGN.md](DESIGN.md).
Code rules: [RULES.md](RULES.md). Agent workflow: [AGENTS.md](AGENTS.md).
Workboard (every open step, by rung): [docs/plans/TODO.md](docs/plans/TODO.md).
Point-in-time audits: [docs/reviews/](docs/reviews/).

## 1. Releases

Identity: [CONSTITUTION.md](CONSTITUTION.md).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end**, plus a working **`pip install minilink` on PyPI**. Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via native `ReinforcementLearningPlanner` (with `Sys2Gym` + SB3 as an optional bridge). See §4.1. | Fall 2026 — **`0.1.0` on PyPI** (2026-09-16); **`0.1.1`** is the first release a GitHub tag publishes. Conda from `environment.yml` stays the Full local stack. Names the course notebooks already use stay frozen until v1.0. Close-out steps: §5.1. |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2) and **the textbook objects finished** (fields, cost parameters, workspace geometry — §5.2 wave A). Conda stays the recommended Full local install. | October 2026 |
| **v0.3** | **GMC714 end to end** — the teaching layer solidified for the class: robust control · robotic arm · trajectory optimization · MPC · vehicle models · nonlinear control (§4.3), with **pyro parity** and the catalog work of §5.3. | December 2026 |
| **v1.0** | The foundation questions deferred to §5.4 (hybrid as a `System`, derived `x0`, evaluator/solver layering, the posed-geometry drawing hook, one mechanical base, the differentiable closed-loop cost), after two cohorts; the Stable-Baselines3 notebooks retire and the name freeze lifts. | 2027 |

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

State as of 2026-09-22. Step ids (`S29`, `P3`, `T2`, …) are the rows of
[docs/plans/TODO.md](docs/plans/TODO.md).

| Area | Lane | TRL | State | Next |
| --- | --- | --- | --- | --- |
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested; wrong-shape `f` / `h` fails at `compile()`. | Derived `x0` (S29, v1.0). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; the frozen evaluator subset is named in DESIGN §5, the stable-internal helper grid kept by ruling. Speed lives in batches (1000 rollouts × 1000 RK4 steps in 27 ms); float64 on JAX by default. | `rollout_batch` with a `params` family (S53); evaluator/solver re-layering (S37, v1.0). |
| Simulation | teaching | 7 | Mature; fixed 10 001-point default grid; `verbose` names unified. | Textbook pass (T4). |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA; **every catalog plant compiles on both backends** (contract test); four-rung vehicle ladder; UdeS racecar (kinematic, dynamic, 3-D). | Textbook pass (T3); mechanical-base unification (S32, v1.0). |
| Control | teaching | 6 | Linear, LQR (infinite horizon, finite horizon, along a trajectory), `P` / `PI` / `PD` / `PID` carrying only the states their terms need, model-based SMC and computed torque, robotic impedance and kinematic laws, neural policy block. | `place()` (P3) and reference scaling (P8, v0.2); textbook pass (T2). |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with margins, pole-zero, root locus, Nyquist, step response on matplotlib and plotly; the frequency band brackets the 0 dB crossing. | `minreal` (P2), named `S` / `T` / `PS` / `CS` (P5), ζ / ω_n (P8); z tier held; Nichols and overlays later; textbook pass (T4). |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources, TF, 1-layer NN. | `Sine` / `Ramp` / `Chirp` / `Delay` / `Switch` (v0.2); textbook pass (T2). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration (`loop` / `numpy` / `jax`), lookup controller, `PolicyEvaluator`, `LQRPlanner`; every planner returns a `PlanningSolution`; `compare()` reads solutions side by side; `dp.py` is the textbook reference of the style (2026-09-18). | cost-to-go as a `Field` (A1); policy iteration (S51). |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot; `success` means defects satisfied to `feasibility_tol`; float64 by default on JAX. | Harden SciPy/Ipopt before TRL 6; textbook pass (T5). |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt. | Harden SciPy/Ipopt before TRL 6. |
| Interfaces / RL bridge | research lane (bridge) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call. | Keep for external interop; courses solve with the native planner. |
| Analysis / Lyapunov certificates | provisional (research) | 4 | `region_of_attraction` → `LyapunovCertificate` (quadratic `V`, sampled level, `verify`, `plot`, `contains`); demo and showcase §11. | Cohort validation; `V` as a `QuadraticField` (A1); SOS and discrete time later. |
| Planning / RL planner | teaching (GRO860) | 6 | `ReinforcementLearningPlanner` (REINFORCE, actor-critic, PPO, SAC in pure JAX), `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control), `StochasticPlanningProblem`, `MonteCarloEvaluator`; the names are on the root prelude; canonical demos in `examples/demos/rl/`, chapter 11 and the native teaching notebooks. | SAC actor step (S50); deep Q-learning (S52); textbook pass (T5). |
| Planning / search (RRT) | provisional | 5 | RRT / RRT*; `RRTPlanner(problem)` works from the input bounds alone (bang-bang extender); returns a `PlanningSolution`. | RRT-Connect later; textbook pass (T5). |
| Geometry / spatial | provisional | 4 | SDF + `Scene` / fields / bodies under `planning/spatial/`, JAX twins tested. Home designed: `core/geometry/` (S57). | Package move and course catalog (A3); bicubic SDF and CBF filter (research). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers; auto-fit camera. | Constructor-derived camera hints (S43); glyph/solid rename (S30, v1.0). |
| Hybrid / step / MPC | provisional (research) | 4 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, MPC with parametric JAX. The sampled loop is the one thing that is not a `System`. | No new hybrid features before the v1.0 decision (S31); textbook pass on `mpc/controller.py` (T6). |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review (v1.0). |
| Estimation | planned (GRO501) | 1 | Placeholder. The largest GRO501 gap. | Luenberger, then Kalman, as diagram blocks (P4, v0.2), after the disturbance-convention decision (§6). |
| Identification | planned | 2 | Parametric-tier prototype only. | `fitting.py` (v0.2). |
| C export (`experimental/c_export`) | research | 1 | JAX→C transpiler; two demos; flagship smoke in the JAX regression job. | Keep isolated; repo-only. |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Not on the teaching path nor the API site. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later. |
| Pyro 2.0 overall | v0.2 | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md). |

## 4. Course objectives

Three courses drive the release contract: GRO860 (§4.1, v0.1, running now),
GRO501 (§4.2, v0.2, parallel objective adopted 2026-09-07) and GMC714 (§4.3,
v0.3, adopted 2026-09-26). A course is
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
7. No name a GRO860 notebook imports today changes before v1.0 (extended
   from the term on 2026-09-26).
8. `minilink 0.1.0` is on PyPI (2026-09-16); from `0.1.1` on, a tag publishes
   from GitHub (Trusted Publishing in `.github/workflows/publish.yml`, the
   version read from the tag). `pip install minilink` installs the
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

### 4.3 v0.3 — GMC714 end to end

The robotics course: the teaching layer solidified for GMC714, December 2026.
Adopted 2026-09-26; no baseline audit yet — step G1 of the workboard runs it,
as the 2026-09-07 audit did for GRO501, and turns this table into the release
contract.

| Topic | Surface today (to confirm in G1) | Status | Gate |
| --- | --- | --- | --- |
| Vehicle models — the four-rung ladder, kinematic to dynamic with tires | `dynamics.catalog` vehicles, `KinematicBicycle` | audit pending | the ladder runs as one lesson (C2) |
| Robotic arm — kinematics, dynamics, joint and task-space control | `manipulators`, `UR5Manipulator`, `control.robotic`, `control.impedance` | audit pending | robotic PID wrappers (C2) |
| Nonlinear control — feedback linearization, computed torque, sliding mode, Lyapunov | `control.modelbased`, `control.geometric`, `analysis.lyapunov` | audit pending | set by G1 |
| Robust control — uncertainty, margins, robust design | `analysis` frequency tools | audit pending | set by G1 |
| Trajectory optimization — direct collocation, shooting | `planning.trajectory_optimization` | audit pending | set by G1 |
| MPC — receding horizon on the vehicle and the arm | `control.mpc` (provisional, research lane) | audit pending | MPC joins the teaching surface, or stays a lesson on the research lane, decided in G1 |

**Cross-cutting gates** (as for GRO501): every topic row green with a demo
or a notebook; one `examples/teaching/` notebook per topic, Colab-first;
student-facing GMC714 material imports only through the teaching surface.

## 5. The path to v1.0

One ladder replaces the phase log kept here until 2026-09-22 (Phases D, 0, 1
and 2 are complete; their history is in git and in
[docs/reviews/](docs/reviews/)). Each rung names its steps; the steps
themselves, with files and "done when" gates, are the rows of
[docs/plans/TODO.md](docs/plans/TODO.md). A rung is done when every step in
it is landed or explicitly dropped by the maintainer.

### 5.1 v0.1 — close-out (now)

The GRO860 path is green (§4.1). What remains is release mechanics and the
term's hygiene:

- **R1** Publish `0.1.1` from a GitHub tag. `0.1.0` is on PyPI since
  2026-09-16, but no git tag marks its commit, and the pre-release checks
  below did not ride it; they gate `0.1.1`. **[ask]**
- CI runs on `dev`, the working branch (R2, landed 2026-09-26).
- **R3** Keep `ruff check .` and `ruff format --check .` green on every
  push; since 2026-09-26 the pre-commit hooks run both with the dev
  extra's ruff (`pre-commit install` once per clone).
- The 46 bugs of the 2026-09-22 improvement scan and ten sibling defects
  its fix reviews found landed 2026-09-23 (S58; outcomes in
  [docs/reviews/2026-09-22-improvement-suggestions.md](docs/reviews/2026-09-22-improvement-suggestions.md)).
- The docs drift the scan found landed 2026-09-26 (S59): the CI commands
  written once, the lane and ROADMAP-ownership lines in AGENTS and RULES, the
  plan-doc ids, the analysis and control API pages.
- The teaching surface is checked by a test and documented page by page
  (S60, landed 2026-09-26): every band facade walked like the root prelude,
  every exported name's module on an API page, Sphinx built with `-W`.
- **S54** The colour scale tops at the price of leaving; the DP
  `out_of_bound_cost` default stays `1e6` (landed and decided 2026-09-26).
- Name freeze: no name a GRO860 notebook imports changes before v1.0
  (§4.1 gate 7).

### 5.2 v0.2 — GRO501 and the textbook objects (October 2026)

Three waves. A and B can run in parallel; D runs throughout and carries on
into v0.3. Wave C moved to v0.3 on 2026-09-26 (§5.3).

**Wave A — the textbook objects finish speaking.** The nouns the constitution
names exist; three of them are not yet the objects the tools carry.

- **A1** `core/fields.py`: `Field` on `(x, u, t)` with `as_constraint`,
  `as_input_constraint`, `as_cost`; `QuadraticField` (the Lyapunov `V`, the
  LQR value, `S(t)` from the Riccati sweep), `GridField` (the DP and tabular
  tables), `CallableField` (the RL critic); `LinearApproximator` as a field.
  Plan: [fields.md](docs/plans/fields.md). **[ask — core]**
- **A2** Cost parameters: `params` on every library cost, composite costs
  nested like a diagram's. Plan: [cost-params.md](docs/plans/cost-params.md).
  **[ask — core]**
- **A3** Workspace geometry: package `core/geometry/` (shapes, Path, Track,
  Scene, probes, `bind`, spatial fields, a course catalog); `planning.spatial`
  retires; `PurePursuit` reads a Path. Plan:
  [geometry-module.md](docs/plans/geometry-module.md) (S57). **[ask — core]**
- **A4** Naming quick wins 1–4 and 6 of [naming.md](docs/plans/naming.md):
  class name as the default `name`, one closed-loop name, informative
  shortcut names, `id` honoured by the sampled loop, `id` documented.
- **A5** The later nouns, each with its first consumer: `Gaussian(cov=)` +
  `log_prob` with the disturbance convention (before P4), sets and
  distributions over parameter dictionaries (with identification or the
  robust problem), `NoiseSource(distribution)`, `UnionSet`,
  `PlanningProblem.hamiltonian()` only if the course teaches Pontryagin.

**Wave B — GRO501 end to end** (§4.2; plan
[gro501-classical-control.md](docs/plans/gro501-classical-control.md)).

- **B1** Correctness first: **P2** `minreal`, **P3** `place()`.
- **B2** The missing surface: **P5** named `S` / `T` / `PS` / `CS`, **P7**
  generated analysis facades, **P8** ζ / ω_n and the `N` matrix.
- **B3** **P4** `estimation/`: `LuenbergerObserver`, `luenberger()`,
  `kalman()`, and how observer and state feedback compose. Held 2026-09-07;
  it is the largest §4.2 gap, so v0.2 needs the hold lifted and the
  disturbance convention (§6) decided first. **[ask]**
- **B4** Polish: **P9** `TransferFunction` ports built once, **P10** the
  three `@` dispatch paths documented and pinned by a test; **P6** the z tier
  stays held (teach with `discretize` + simulation) unless the sommatif
  examines z-plane analysis; **S61** every analysis verb takes a `System`;
  **S62** the LQR family on the control band facade. **[ask — public names]**
- **B5** **P11** the two GRO501 notebooks (APP2 propulsion, APP4 autopilot),
  Basic tier, Colab-first. **[maintainer]**

**Wave D — the code reads like the textbook, and the demos like the API.**
Standing work, behaviour-preserving, one module per step.

- **T1–T6** The textbook pass, module by module, in the order of the
  2026-09-22 audit ([docs/reviews/2026-09-22-consolidation-review.md](docs/reviews/2026-09-22-consolidation-review.md)):
  core (T1), blocks and control (T2), the dynamics catalog (T3), analysis
  and simulation (T4), planning (T5), then `composition.py` and
  `mpc/controller.py` (T6, each a maintainer conversation first). Every step
  follows the AGENTS refactor recipe: seeded baseline, byte-identical after.
- **D1** Demos and notebooks to the minimal rule (RULES 6.1 / 6.10 / 6.11):
  the native plots and prints the demos hand-roll first (agent lane), then
  the flatness ratchet in `test_teaching_imports.py`, then the sweep file by
  file. **[ask per file]**
- **D2** Consolidation picks from the 2026-09-05 inventory still open:
  `Source.show_signal`, the dead modules, the deprecated benchmark shims, the
  MPC debug figure, the `HybridDiagram` hand-copied facades, plotting homes.
- **D3** Hardening rows carried over (research lane): RRT extender ignores
  `problem.params.system`; MPC port computes drop `params`;
  `ShootingTranscription` orphaned from presets; parametric evaluator
  duplication; `HybridSimulator` conventions; realtime review;
  `StepDiagramSystem.step` writes in place under JAX; camera hints (S43);
  `rollout_batch` family profile (S53).
- **T7** The graphical band joins the textbook pass. **S65** One owner per
  rule (the automatic `dt`, the backend fallback, the input-hold model, the
  Monte Carlo defaults); **S66** wiring mistakes fail at wiring time
  **[ask — core]**; **S67** one plot vocabulary; **S68** contracts as tests
  (the RULES 5.8 ratchet, ROADMAP ids against the workboard, every notebook
  executed nightly).

### 5.3 v0.3 — GMC714 (December 2026)

**Wave C — GMC714, the catalog, pyro parity.** Follows wave A.

- **G1** GMC714 baseline audit: every §4.3 topic run on the teaching
  surface, the gaps written as the release contract (the 2026-09-07 GRO501
  audit is the model). **[ask — course scope]**

- **C1** Pyro parity open rows ([pyro-port-remaining.md](docs/plans/pyro-port-remaining.md))
  and the pyro → minilink migration guide in README.
- **C2** GMC714 modelling ladder: manipulators and the vehicle ladder as a
  `02_dynamics` lesson; robotic PID wrappers.
- **C3** Blocks: `Sine` / `Ramp` / `Chirp` / `Delay` / `Switch`.
- **C4** `identification/fitting.py` on `rollout_batch`;
  `trajectory_generation/` port; SMC trajectory-following demo.
- **C5** RL follow-ups: **S50** SAC actor step, **S51** policy iteration on
  the grid, **S52** deep Q-learning; approximate dynamic programming on the
  approximation bases.
- **S63** DP reads a finite horizon from `problem.tf`, as LQR does; **S64**
  catalog hygiene (bounds each plant states, port labels read from the
  state, one wheelbase owner). **[ask]**

### 5.4 v1.0 — one `System`, closed (2027)

The foundation questions, after two cohorts have run on v0.2:

- **S31** The sampled loop as a `System` (state `[plant; computer]`, periodic
  discrete update) or an honest `HybridLoop` rename; `%` stays the one hybrid
  operator either way. **[ask — core]**
- **S29** `DiagramSystem.x0` / `n` / `state` as derived properties.
  **[ask — core]**
- **S37** Evaluator / solver re-layering: evaluators keep pure maps and one
  scannable step, integrators move to `simulation/solvers/`; **S27** Diffrax
  as an optional JAX solve. **[ask — core]**
- **S44** One posed-geometry hook so "two functions" (`f` and a drawing
  function) is literal; **S30** the glyph / solid rename. **[ask]**
- **S32** One mechanical base (`N = I` special case); `Boat2D` / `Plane3D`
  gain `q` / `dq` ports. **[ask — core]**
- **V1** The differentiable closed-loop cost, `J = F(problem params)` as one
  traced scalar, on the nested parameter dictionary A2 introduces.
  **[ask — core]**
- **S36** iLQR from parts (idea, research lane).
- **S49** Retire the two Stable-Baselines3 teaching notebooks at the end of
  the term, once the course notes point only at the native twins; the
  GRO860 name freeze lifts with v1.0. **[ask]**
- **V2** Release hygiene: Zenodo archive and DOI, `CITATION.cff`; the JOSS
  decision and the RULES 6.9 carve-out it would need, decided then.

**Simplify and consolidate — a standing principle, not a rung.** The repo
must stay manageable by one maintainer, so every rung carries consolidation
work, and the target is *maintenance cost*, never features: text edited
twice when code changes, dead API, boilerplate a flag would replace, twins
one `xp` body covers, research scenarios inside the teaching tree, plan docs
already self-marked complete. A deliberate ladder of implementations (the DP
planner's three backends) is not duplication. The maintainer picks each
item; agents execute only what was picked.

## 6. Decisions

Open decisions, by the rung they block. Settled decisions are recorded in
DESIGN (contracts) and [docs/reviews/](docs/reviews/) (the rulings), not
here. Each open item needs the maintainer.

**Before wave B3 (`estimation/`)**

- **The disturbance convention.** `disturbances={port: Distribution}` draws
  one value per control period and holds it, so its variance does not scale
  with `dt`; `WhiteNoise` carries `var` + `sample_period`, the same
  ambiguity. A Kalman filter needs the rule once: per-step covariance `Q_d`,
  or spectral density `Q_c` with `Q_d = Q_c / dt` (RULES 4.12 applies).
  Recorded in docs/reviews/2026-09-15-foundations-review.md (F9).
- **The terminal cost `h(x_f, t_f)`, one rule for every tool.** Since the
  one-horizon ruling (2026-09-17) `h` is charged exactly when `tf` is
  finite; still to settle whether an infinite-horizon cost with a nonzero `h`
  warns or raises, and that exits are priced only by `infeasible_cost` (the
  rocket landing's constant `h = 100` is an exit penalty written as a
  terminal cost).
- **Observer + state feedback composition** (P4): a `compensator(observer,
  controller)` block with ports `r`, `y` → `u`, or an explicit `connect`
  recipe in the notebook.

**Before `0.1.1` (R1)**

- **`plot_diagram()` without the `graphviz` wrapper** — decided 2026-09-26:
  warn once and skip the figure, as a missing binary already does; the
  wrapper stays in the `diagrams` extra. Landed the same day, hybrid
  diagrams included.
- **`showcase_jax.ipynb` imports `experimental.c_export`**, which the wheel
  does not ship — decided 2026-09-26: accepted for a showcase, which runs
  from the clone; revisit if the notebooks move to `pip install minilink`.

**During v0.2**

- **Undeclared feedthrough** (S66): an output that moves with an input it
  does not declare simulates a wrong fixed point silently today; raise or
  warn at compile.
- **Mechanical default bounds** (S64): keep the invented ±2π / ±5 on every
  `MechanicalSystem`, or each plant states its own and `StateSpaceGrid`
  refuses infinite bounds.
- **`JaxMechanicalSystem`**: retire the twin (an alias for one release) now
  that the `xp` base traces.
- **`PlanningProblem.metadata`**: document it or retire it.
- **The z tier** (P6): teach the Arduino law with `discretize` + simulation
  (default), or build a z-domain analysis family. Depends on what the
  sommatif examines.
- **Optional `KinematicModel` delegate** — adopt or drop.
- **Pyro game demos** — port the rest to `simulation/realtime/` or drop.
- **Which pyro demos the courses still need** (drives the parity rows).
- **Lyapunov certificates**: a GRO860 checklist row of their own, or an
  analysis tool the lecture uses; `method=` naming the certificate family
  (accepted deviation from the band's calling pattern).
- **Solver-specific views on the planning records** (from the
  planning-solution work, 2026-09-17).
- **Naming, the larger alignments** ([naming.md](docs/plans/naming.md)): one
  separator rule for wires, params paths, block labels and duplicate state
  labels; `sys` versus `plant` across loop kinds (changes a tested key);
  plot labels for internal signals; a style sweep of display names.
- **CBF safety filter** ([cbf-safety-filter.md](docs/plans/cbf-safety-filter.md)):
  research lane until a course asks; the barrier is a `Field` once A1 lands.

**Before v1.0**

- `HybridDiagram` as a `System` vs `HybridLoop` (S31), and the sampled
  seam's time argument: integer ticks (today) or seconds, which would remove
  three copies of `t0` / `dt_mpc` in the MPC block.
- Evaluator / solver layering, Diffrax (S37, S27).
- The posed-geometry hook (S44) and the glyph / solid rename (S30).
- Zenodo / DOI, and whether JOSS is wanted (V2).

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
| [docs/plans/TODO.md](docs/plans/TODO.md) | The workboard: every open step of §5, by rung, with files and "done when" |
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Open pyro parity rows (v0.2) |
| [docs/plans/](docs/plans/) | Design writeups for the steps that need one (see [plans README](docs/plans/README.md)) |
| [docs/reviews/](docs/reviews/) | Dated architecture audits and decision records |
