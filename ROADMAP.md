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
| **v0.1** | **GRO860 end to end**, plus a working **`pip install minilink` on PyPI**. Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via native `ReinforcementLearningPlanner` (with `Sys2Gym` + SB3 as an optional bridge). See §4.1. | Fall 2026 — **`0.1.0` on PyPI** (2026-09-16); **`0.1.1`** (2026-09-26) is the first release a GitHub tag published. Conda from `environment.yml` stays the Full local stack. Names the course notebooks already use stay frozen until v1.0. Close-out steps: §5.1. |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2). Conda stays the recommended Full local install. | October 2026 |
| **v0.3** | **GMC714 end to end** — the teaching layer solidified for the class: robust control · robotic arm · trajectory optimization · MPC · vehicle models · nonlinear control (§4.3), with **pyro parity**, the catalog work of §5.3 and **the textbook objects** (fields, cost parameters, workspace geometry — moved from v0.2 on 2026-09-26). | December 2026 |
| **v0.9** | **Freeze candidate** (§5.4): the foundation questions that could still break a public name are decided (hybrid as a `System`, derived `x0`, evaluator/solver layering, the glyph/solid rename); the trust contract is in place (deprecation policy, pinned export list, CHANGELOG, pinned installs); the standard-syllabus holes close (estimation, discrete time, identification); the docs site renders the tutorials. | Early 2027 |
| **v1.0** | **A colleague can build a course on it and trust it for three years** (§5.5): the API frozen under the deprecation policy, the standard control syllabus covered and checked against textbook worked examples, course-neutral labs an instructor adopts outside UdeS, green on Windows, macOS and Linux; after two cohorts. The Stable-Baselines3 notebooks retire and the GRO860 name freeze becomes the general one. | 2027 |

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

2026-09-30: `WhiteNoise` is rewritten as a pure block, `WhiteNoise(p, *, psd,
sample_period, seed, hold)`; its `var`, `mean`, `t0` and `tf` params and its
`refresh()` are removed without aliases, and an old per-sample variance reads as
the intensity `psd = var × sample_period`. Seeded realizations change once (a
counter-based cipher replaces the drawn table); a diagram holding noise now picks
fixed-step RK4 on a divisor of the sample period by itself, and is linearized at
the noise's mean instead of raising. The tutorial and course cells were rewritten
in the same commit.

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

State as of 2026-10-02. Step ids (`S29`, `P3`, `T2`, …) are the rows of
[docs/plans/TODO.md](docs/plans/TODO.md).

| Area | Lane | TRL | State | Next |
| --- | --- | --- | --- | --- |
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested; wrong-shape `f` / `h` fails at `compile()`. | Derived `x0` (S29, v0.9). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; the frozen evaluator subset is named in DESIGN §5, the stable-internal helper grid kept by ruling. Speed lives in batches (1000 rollouts × 1000 RK4 steps in 27 ms); float64 on JAX by default. | `rollout_batch` with a `params` family (S53); evaluator/solver re-layering (S37, v0.9). |
| Simulation | teaching | 7 | Mature; fixed 10 001-point default grid; `verbose` names unified. | Textbook pass (T4). |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA; **every catalog plant compiles on both backends** (contract test); four-rung vehicle ladder; UdeS racecar (kinematic, dynamic, 3-D). | Textbook pass (T3); mechanical-base unification (S32, after v1.0). |
| Control | teaching | 6 | Linear, LQR (infinite horizon, finite horizon, along a trajectory), pole placement, `P` / `PI` / `PD` / `PID` carrying only the states their terms need, model-based SMC and computed torque, robotic impedance and kinematic laws, neural policy block. | Reference scaling (P8, v0.2); textbook pass (TB-b, T2). |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with margins, pole-zero, root locus, Nyquist, step response on matplotlib and plotly; the frequency band brackets the 0 dB crossing. | named `S` / `T` / `PS` / `CS` (P5), ζ / ω_n (P8); z tier held; Nichols and overlays later; textbook pass (T4). |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources, TF, 1-layer NN. | `Ramp` / `Chirp` / `Delay` / `Switch` (C3, v0.3; `Sine` landed with P5); textbook pass (T2). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration (`loop` / `numpy` / `jax`), lookup controller, `PolicyEvaluator`, `LQRPlanner`; every planner returns a `PlanningSolution`; `compare()` reads solutions side by side; `dp.py` is the textbook reference of the style (2026-09-18). | cost-to-go as a `Field` (A1); policy iteration (S51). |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot; `success` means defects satisfied to `feasibility_tol`; float64 by default on JAX. | Harden SciPy/Ipopt before TRL 6; textbook pass (T5). |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt. | Harden SciPy/Ipopt before TRL 6. |
| Interfaces / RL bridge | research lane (bridge) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call. | Keep for external interop; courses solve with the native planner. |
| Analysis / Lyapunov certificates | provisional (research) | 4 | `region_of_attraction` → `LyapunovCertificate` (quadratic `V`, sampled level, `verify`, `plot`, `contains`); demo and showcase §11. | Cohort validation; `V` as a `QuadraticField` (A1); SOS and discrete time later. |
| Planning / RL planner | teaching (GRO860) | 6 | `ReinforcementLearningPlanner` (REINFORCE, actor-critic, PPO, SAC in pure JAX), `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control), `StochasticPlanningProblem`, `MonteCarloEvaluator`; the names are on the root prelude; canonical demos in `examples/demos/rl/`, chapter 11 and the native teaching notebooks. | SAC actor step (S50); deep Q-learning (S52); automatic reward scaling ([rl-reward-scaling.md](docs/plans/rl-reward-scaling.md)); textbook pass (T5). |
| Planning / search (RRT) | provisional | 5 | RRT / RRT*; `RRTPlanner(problem)` works from the input bounds alone (bang-bang extender); returns a `PlanningSolution`. | RRT-Connect later; textbook pass (T5). |
| Planning / path integral (MPPI) | research lane (planned) | 1 | Two standalone scripts (`examples/experimental/mppi/`, 2026-10-02): the pendulum swings up; the kinematic racecar laps the cone circuit at a lower closed-loop cost than the collocation MPC on the same problem, 9 ms a tick on CPU. Design [mppi.md](docs/plans/mppi.md) (2026-10-02): a `PathIntegralPlanner` beside trajopt, wrapped by the existing `ModelPredictiveController`; both problem classes (input noise on a deterministic problem, plant draws per sample on a stochastic one); the batched JAX rollout and `RolloutEnvironment` as the step's owner. | The planner (S72) after RN-4 / RN-5, on the evaluator's batched rollout over realizations; the MPC block reads three planner verbs instead of trajopt internals (T6) before v0.9; teaching form with V3. |
| Geometry / spatial | provisional | 4 | SDF + `Scene` / fields / bodies under `planning/spatial/`, JAX twins tested. Home designed: `core/geometry/` (S57). | Package move and course catalog (A3); bicubic SDF and CBF filter (research). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers; auto-fit camera. | Constructor-derived camera hints (S43); glyph/solid rename (S30, v0.9). |
| Hybrid / step / MPC | provisional (research) | 4 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, MPC with parametric JAX. The sampled loop is the one thing that is not a `System`. | No new hybrid features before the v0.9 decision (S31); textbook pass on `mpc/controller.py` (T6). |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review (v1.0). |
| Estimation | planned (GRO501) | 1 | Placeholder. The largest GRO501 gap. | Luenberger, then Kalman, as diagram blocks (P4, v0.2), after step RN-1 of [randomness.md](docs/plans/randomness.md): the `WhiteNoise` whose `psd` the Kalman design reads (the disturbance convention, decided 2026-09-26). |
| Identification | planned | 2 | Parametric-tier prototype only. | `fitting.py` (C4, v0.3). |
| C export (`experimental/c_export`) | research | 1 | JAX→C transpiler; two demos; flagship smoke in the JAX regression job. | Keep isolated; repo-only. |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Not on the teaching path nor the API site. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later. |
| Pyro 2.0 overall | v0.3 | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md). |

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
   from the term on 2026-09-26). **Paused 2026-10-10** (maintainer): between
   terms the teaching design is unfrozen; a rename migrates every course
   notebook that uses the name in the same commit, with no alias. The freeze
   resumes when the next term's notebooks are pinned.
8. `minilink 0.1.1` is on PyPI (2026-09-26, after `0.1.0` on 2026-09-16); a tag
   publishes from GitHub (Trusted Publishing in `.github/workflows/publish.yml`, the
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
| Multi-physics modelling — nonlinear `f`/`h`, block diagram, linearize, `H(s)` | custom `DynamicSystem`, `plot_diagram`, `linearize`, `transfer_function` | green | a DC-motor + longitudinal-vehicle plant in the catalog; pole/zero cancellation landed 2026-09-26 (P2); order reduction by fast-mode neglect is out of v0.2 (decided 2026-09-26) |
| Closed-loop analysis — poles, root locus, Bode, margins, step specs | `pzmap`, `root_locus`, `bode`, `margins`, `step_info`, `P` / `PI` / `PD` / `PID` | green | met 2026-09-07: every compensator form reports the poles and zeros of the hand calculation, and margins are found wherever the crossover sits |
| Design to specification — rise time, overshoot, final error, phase margin | `PI`, `PD`, `PID`, `Lead`, `Lag`, `step_info`, `margins` | green | the Table 2 specs of the guide are checkable in one notebook |
| Loop-shaping specs — disturbance and measurement-noise sensitivity in dB at a frequency | `analysis` (v0.2) | scheduled v0.2 | named `S` / `T` / `PS` / `CS` sensitivity shortcuts on closed-loop diagrams |
| Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models) | continuous only, **z tier held** | the sampled loop is validated by simulation; a z tier stays out of v0.2 unless the sommatif examines z-plane analysis (§6) |
| State-space MIMO — bicycle model, controllability at every nominal speed | `KinematicBicycle`, `controllability`, `observability` | green | — |
| Optimal control — LQR on the guide's cost, closed-loop poles, nonlinear check | `lqr_at_operating_point`, `StateFeedbackController` | green | — |
| Pole placement — `K_sta` for a prescribed pole set | `place`, `place_at_operating_point` | green (2026-09-26) | — |
| Nested loops — inner speed loop, outer position loop | `@` composition | green for error-driven compensators (`PID @ (PID @ G)`); an outer `r, y` or state controller is refused today (2026-10-10 review, D5) | AC-3 of [automation-by-convention.md](docs/plans/automation-by-convention.md) nests the inner loop as the outer loop's `plant`; stays green with the observer in the loop |
| State estimation — Luenberger observer and Kalman filter | `estimation` (v0.2) | scheduled v0.2 | `LuenbergerObserver` and steady-state `KalmanFilter` closing the loop as standard diagram blocks |
| Reference scaling — the `N` matrix giving `y = r` at steady state | `StateFeedbackController(K, N=N)` | green (2026-10-10) | the student writes `N = -(C (A - B K)⁻¹ B)⁻¹`; the block takes the output reference |

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
| MPC — receding horizon on the vehicle and the arm | `control.mpc` (provisional, research lane) | audit pending | MPC joins the teaching surface, or stays a lesson on the research lane, decided in G1; the sampling flavour (MPPI, [mppi.md](docs/plans/mppi.md), S72) is on the table for that audit |

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

- `0.1.1` published from the GitHub tag on 2026-09-26 (R1), the first
  release through the pre-release checks of `publish.yml`.
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
  (§4.1 gate 7); paused between terms on 2026-10-10, renames migrating the
  course notebooks in the same commit.

### 5.2 v0.2 — GRO501 (October 2026)

Three waves. A and B can run in parallel; D runs throughout and carries on
into v0.3. Wave C moved to v0.3 on 2026-09-26 (§5.3).

**Wave A — the textbook objects finish speaking.** The nouns the constitution
names exist; three of them are not yet the objects the tools carry. A1–A3 moved
to v0.3 on 2026-09-26 (§5.3): GRO501 needs none of them, and October goes to
wave B. A4 and A5 stay here.

- **A4** Naming quick wins 1–4 and 6 of [naming.md](docs/plans/naming.md):
  class name as the default `name`, one closed-loop name, informative
  shortcut names, `id` honoured by the sampled loop, `id` documented.
- **A5** The later nouns, each with its first consumer: `log_prob` on the
  distributions, sets and distributions over parameter dictionaries (with
  identification or the robust problem), `UnionSet`,
  `PlanningProblem.hamiltonian()` only if the course teaches Pontryagin.
  `Gaussian(cov=)`, `NoiseSource` and the draw convention moved to
  [randomness.md](docs/plans/randomness.md) (steps RN-1 to RN-6). Its **RN-2** joins this wave:
  distributions read `params`, `Gaussian(cov=)`, every `sample` takes a key.

**Wave B — GRO501 end to end** (§4.2; plan
[gro501-classical-control.md](docs/plans/gro501-classical-control.md)).

- **B1** Correctness first: **P2** pole/zero cancellation (landed 2026-09-26:
  `pzmap` / `root_locus` / `transfer_function` cancel by default), **P3** `place()`
  (landed 2026-09-26: Ackermann with one input, `place_poles` with several).
  Order reduction (P2b) is out of v0.2 by decision (2026-09-26; TODO §7).
- **TB-a** The analysis toolbox reads like the textbook — landed 2026-09-26:
  the System shortcuts cut to objects and plots, the audit
  ([2026-09-26-analysis-toolbox-audit.md](docs/reviews/2026-09-26-analysis-toolbox-audit.md)),
  the byte-identical `dp.py`-style pass of `linear`, `frequency`,
  `time_response`, `structural`, `linearize` and `modal`, and four fixes
  (`root_locus` through K = −1/d, `step_info` on an unsettled response, MIMO
  input to the SISO functions, one rank rule).
- **B2** The missing surface, on the cleaned toolbox: **P5** named
  `S` / `T` / `PS` / `CS` (landed 2026-10-10: the four functions from the loop's
  pieces, `closed_loop(r=, w=, v=)`, `Sine`, the `sensitivity_functions` teaching
  notebook), **P8** the `N` matrix (landed 2026-10-10:
  `StateFeedbackController(K, N=N)` on an output reference, `print` of `LTISystem`,
  `TransferFunction` and `StructuralResult`; a `damping` verb and a `poles` verb were
  dropped by the maintainer the same day: both are two textbook lines on
  `np.linalg.eigvals`). **P7** landed
  2026-09-26: the shortcuts TB-a kept stay written out and are pinned to
  their band functions by a test, not generated.
- **TB-b** The control objects and the loop (`siso`, `state`, `lqr`, `place`,
  `TransferFunction`, `LTISystem`, `feedback` / `@`) read like the textbook;
  after P5, before P11.
- **B3** **P4** `estimation/`: `LuenbergerObserver`, `luenberger()`,
  `kalman()`, and how observer and state feedback compose. Held 2026-09-07;
  it is the largest §4.2 gap. Decided 2026-09-26: the clean-up and
  solidification (TB-a and P7 landed; P5, P8, TB-b, S61, P9, P10) land first, then
  RN-1 of [randomness.md](docs/plans/randomness.md) (`WhiteNoise` with its `psd`), then P4. The
  disturbance convention was decided 2026-09-26 (§6). **[ask]**
- **B4** Polish: **P9** `TransferFunction` ports built once (landed 2026-10-10:
  `LTISystem(..., declare_ports=False)` for a subclass's own layout), **P10**
  automation by convention ([automation-by-convention.md](docs/plans/automation-by-convention.md),
  v2 2026-10-10): standard port names drive the wiring, three reserved subsystem ids
  (`plant`, `controller`, `estimator`) are the roles every other tool reads, and the cost is
  scored on the plant. AC-0 to AC-3 land in this wave:
  - AC-0, no controller classes;
  - AC-1, bug fixes (landed 2026-10-10);
  - AC-2, the resolver and the id rename, before RN-4;
  - AC-3, composition on standard names, after the §6 decisions;
  - AC-OP, the operators: none modifies its operands, and `>>` becomes a plain serial
    connection, after the operator review closes.

  The later steps: AC-4 beside P4 and before P11; AC-5 / AC-6 in the plotting lane; AC-7
  (cost relative to the plant) in v0.3 with RN-4 / RN-5; AC-8 in v0.9 with S31;
  **P6** the z tier
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
- **S69** Students' laptops: Windows and macOS legs in CI (Basic tier and a
  notebook smoke), coverage reported (no gate). Lands in v0.2, before the
  GRO501 cohort installs.
- **T7** The graphical band joins the textbook pass. **S65** One owner per
  rule (the automatic `dt`, the backend fallback, the input-hold model, the
  Monte Carlo defaults); **S66** wiring mistakes fail at wiring time
  **[ask — core]**; **S67** one plot vocabulary; **S68** contracts as tests
  (the RULES 5.8 ratchet, ROADMAP ids against the workboard, every notebook
  executed nightly).

### 5.3 v0.3 — GMC714 (December 2026)

**Wave A's core objects** (moved from v0.2 on 2026-09-26), before wave C:

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

**Wave C — GMC714, the catalog, pyro parity.** Follows wave A.

- **G1** GMC714 baseline audit: every §4.3 topic run on the teaching
  surface, the gaps written as the release contract (the 2026-09-07 GRO501
  audit is the model). **[ask — course scope]**

- **C1** Pyro parity open rows ([pyro-port-remaining.md](docs/plans/pyro-port-remaining.md))
  and the pyro → minilink migration guide in README.
- **C2** GMC714 modelling ladder: manipulators and the vehicle ladder as a
  `02_dynamics` lesson; robotic PID wrappers.
- **C3** Blocks: `Ramp` / `Chirp` / `Delay` / `Switch` (`Sine` landed with P5).
- **C4** `identification/fitting.py` on `rollout_batch`;
  `trajectory_generation/` port; SMC trajectory-following demo.
- **C5** RL follow-ups: **S50** SAC actor step, **S51** policy iteration on
  the grid, **S52** deep Q-learning; approximate dynamic programming on the
  approximation bases.
- **S63** DP reads a finite horizon from `problem.tf`, as LQR does; **S64**
  catalog hygiene (bounds each plant states, port labels read from the
  state, one wheelbase owner). **[ask]**
- **S72** The path-integral planner (MPPI) on the research lane
  ([mppi.md](docs/plans/mppi.md), proposed 2026-10-02): two standalone scripts
  run today (`examples/experimental/mppi/`: the pendulum swing-up, the racecar
  circuit against the collocation MPC); the planner,
  `PathIntegralPlanner`, lands after RN-4 / RN-5 on the evaluator's batched
  rollout over realizations (decided 2026-10-02), deterministic first (noise on
  the inputs), then the stochastic branch (a plant realization per sample from
  the problem's distributions); wrapping it in `ModelPredictiveController` is
  the T6 contract narrowing of `mpc/controller.py` (steps MP-1 to MP-3 here,
  MP-4 and MP-5 before v0.9, MP-6 with V3). **[ask — name, the MPC block contract]**
- **RN-4 / RN-5** of [randomness.md](docs/plans/randomness.md), together, after the fall term:
  `realize(key)` and signals on `disturbances=` (the frozen-noise defect
  fixed), then the Monte Carlo evaluator's test set. Both change public
  behaviour and the Monte Carlo numbers once, so they land between cohorts
  and before the v0.9 freeze. **RN-3** `NoiseSource` lands here too, unless a
  GRO501 notebook needs sampled sensor noise first (then with P11).

### 5.4 v0.9 — freeze candidate (early 2027)

What v1.0 asks is adoption, not architecture: a colleague builds a course on
minilink and trusts it for three years. The
[adoption review of 2026-09-26](docs/reviews/2026-09-26-adoption-review.md)
found the engine ready and three things missing around it: a stability
promise, the two syllabus holes every course hits (estimation, discrete
time), and material packaged for an instructor. v0.9 closes the first two
and settles every question that could still break a public name; v1.0 (§5.5)
freezes.

**The foundations that can break names** — decided before the freeze, not
after:

- **S31** The sampled loop as a `System` (state `[plant; computer]`, periodic
  discrete update) or an honest `HybridLoop` rename; `%` stays the one hybrid
  operator either way. **[ask — core]**
- **S29** `DiagramSystem.x0` / `n` / `state` as derived properties.
  **[ask — core]**
- **S37** Evaluator / solver re-layering: evaluators keep pure maps and one
  scannable step, integrators move to `simulation/solvers/`, and the fixed-step
  solvers take a sample stride (`substeps`, TODO §7); **S27** Diffrax
  as an optional JAX solve. **[ask — core]**
- **S30** The glyph / solid rename; **S44** one posed-geometry hook so "two
  functions" (`f` and a drawing function) is literal. **[ask]**
- **V3** The frozen surface: which names the freeze covers. The provisional
  bands on the root and band facades today (`StepSystem`,
  `StepDiagramSystem`, `ZOHHold`, `control.mpc`) either join the frozen
  surface or move behind a visibly provisional name. **[ask — public names]**

**The trust contract:**

- **V4** Deprecation policy: from v1.0 on, removing or renaming a
  teaching-surface name costs one minor release with a `DeprecationWarning`
  shim (RULES "no deprecated aliases" keeps holding until then). A snapshot
  test pins the teaching-surface export list; a change to it names its
  CHANGELOG line. **[ask]**
- **V5** `CHANGELOG.md` for users (`pyproject` points there, not at this
  file); every release from v0.9 carries its entry.
- **V6** One install story: `pip install minilink` first, conda for the Full
  stack; the Colab cell installs a pinned release instead of cloning `main`
  (the repo-contract test follows); `install.md` current.
- **S70** A `MinilinkError` family (wiring, shape, solver) under the plain
  `ValueError` / `TypeError` it subclasses, so a message and an autograder
  can catch the category. **[ask — core]**
- **V11** A light history: the notebook outputs committed before the
  `nbstripout` hook worked (169 of the 176 MB a clone downloads, five
  notebooks deleted since May) leave the git history. It breaks every commit
  id and every local clone, so it runs once, in December 2026 after the v0.3
  tag, between the fall and winter terms. **[ask]**

**The syllabus holes** (beyond GRO501's P4 / P5 / P8):

- **P6** The z tier leaves hold: exact ZOH and Tustin `c2d`, z-plane pole-zero
  map, discrete step and Bode. Every digital-control course needs it, whether
  or not the GRO501 sommatif examines it. **[ask — public names]**
- **C4** `identification/fitting.py` (least squares, ARX, step-response fit)
  and trajectory generation (polynomial, trapezoidal, minimum jerk), pulled
  from v0.3 if GMC714 does not land them.
- **S71** The classic first plants: DC motor, tank / thermal process, ball and
  beam, differential drive, 3-D quadrotor. LQI beside the `N` matrix (P8).

**The reference site:**

- **V7** The docs site renders the tutorials (myst-nb or nbsphinx), a plant
  gallery with pictures, and a short concept guide (what a `System` is,
  composition, tools as verbs).

### 5.5 v1.0 — a colleague can build a course on it (2027)

After two cohorts have run on v0.2 and the freeze candidate has held a term:

- **V8** A textbook-validation suite: Dorf / Ogata / Franklin worked examples
  asserted numerically (margins, step specifications, LQR and Kalman gains,
  root locus), course-neutral and permanent; GRO501 gate 3 is its seed. It is
  also the page an instructor reads first.
- **V9** Course-neutral labs: `examples/teaching/topics/` grown to 10–15
  self-contained labs (objectives, prerequisites, time, starter and solution),
  classical control and robotics at the depth optimal control and RL already
  have; the UdeS course folders stay as case studies. A "For instructors"
  page: pin a version, run in Colab, adapt a lab, report a bug.
  **[maintainer]**
- **V10** The public face: the README opens on a teaching quickstart (JAX
  speed, compile tiers and the Gym bridge lower down or on the site); no
  internal vocabulary (TRL, lanes, wave and step ids) in user-facing files;
  whether the agent and governance files stay at the root is decided;
  `CONTRIBUTING.md`, issue templates, a code of conduct. **[ask]**
- **V2** Release hygiene: Zenodo archive and DOI, `CITATION.cff`; the JOSS
  decision and the RULES 6.9 carve-out it would need, decided then.
- **S49** Retire the two Stable-Baselines3 teaching notebooks at the end of
  the term, once the course notes point only at the native twins. **[ask]**
- The freeze: the V3 surface under the V4 policy; the GRO860 name freeze
  becomes the general one.

**After v1.0 (1.x features, additive by construction):** **V1** the
differentiable closed-loop cost, `J = F(problem params)` as one traced scalar
on the nested parameter dictionary A2 introduces **[ask — core]**; **S32** one
mechanical base (`N = I` special case), if it lands without breaking a name
**[ask — core]**; **S36** iLQR from parts (research lane); a general serial
chain (DH or URDF to RNEA / ABA).

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

- **The disturbance convention — decided 2026-09-26** in [randomness.md](docs/plans/randomness.md)
  (rulings D1–D12; finding F9 of the 2026-09-15 foundations review). A
  `Distribution` has no time: it is the law of one draw. A noise signal is a
  block holding draws over a sample period Δ: `NoiseSource(law, Δ)` for
  sampled noise, `WhiteNoise(psd=W, Δ)` for continuous white noise of
  two-sided density `W`, drawn as `w_k ~ N(0, W / Δ)` so its physics does not
  change with Δ. Seed, Δ and magnitude are params; `h` draws with a
  counter-based generator (no table, no time window, traceable). A problem's
  disturbance port takes a signal; `realize(key)` gives every random block
  and parameter its own stream; the Monte Carlo evaluator scores every law on
  one test set drawn once. Kalman reads `Q = B_w W B_wᵀ`, `R = V`. This entry
  leaves §6 when step RN-6 moves the contract to DESIGN. Reviewed 2026-09-30
  (§10 of the plan). Decided that day, D13: the noise block draws with one
  counter generator on NumPy and JAX, so the same seed is the same signal on
  both, and the test set holds starts, parameter values and seeds, no noise
  values (it amends D11). A2–A10 ruled the same day as D14–D23, and D24
  added: RN-1 lands now with the course cell rewritten in the same commit;
  `WhiteNoise(p, *, psd, sample_period, seed, hold)`; a noisy diagram
  publishes Δ as its solver hint and runs fixed-step RK4 at Δ by
  itself; `seed = None` is the mean, so analysis linearizes at `E[w] = 0`;
  streams are derived by name with the library's cipher; and the Monte
  Carlo evaluator simulates the closed-loop diagram, batched, so any
  controller and any noise block go through the one simulation path. The
  implementation plan is the plan's §9. RN-1 landed 2026-09-30 (the block,
  the cipher, the solver hint, `realize(key)` on every `System`, the two
  warnings, the notebooks); three implementation notes there: the hint is a
  float `sample_period` key, the stepped-tools check warns on a Jacobian
  probe, and `realize(key)` landed whole with the systems.
- **The terminal cost `h(x_f, t_f)`, one rule for every tool.** Since the
  one-horizon ruling (2026-09-17) `h` is charged exactly when `tf` is
  finite; still to settle whether an infinite-horizon cost with a nonzero `h`
  warns or raises, and that exits are priced only by `infeasible_cost` (the
  rocket landing's constant `h = 100` is an exit penalty written as a
  terminal cost).
- **Observer + state feedback composition** (P4): a `compensator(observer,
  controller)` block with ports `r`, `y` → `u`, or an explicit `connect`
  recipe in the notebook.

**Decided for `0.1.1` (R1, published 2026-09-26)**

- **`plot_diagram()` without the `graphviz` wrapper** — decided 2026-09-26:
  warn once and skip the figure, as a missing binary already does; the
  wrapper stays in the `diagrams` extra. Landed the same day, hybrid
  diagrams included.
- **`showcase_jax.ipynb` imports `experimental.c_export`**, which the wheel
  does not ship — decided 2026-09-26: accepted for a showcase, which runs
  from the clone; revisit if the notebooks move to `pip install minilink`.

**During v0.2**

- **System analysis shortcuts** — decided 2026-09-26: a method on every
  `System` is reserved for the very common "linearize + analyse/plot"
  operations; the rest is called as a band function. `minreal` is the first
  case: no shortcut, it runs inside `pzmap` / `root_locus` /
  `transfer_function`. Landed 2026-09-26 as a clean cut, no deprecation: a
  shortcut is an object or a plot (`linearize`, `find_equilibrium`,
  `transfer_function`, `plot_phase_plane`, `plot_bode`, `plot_pzmap`,
  `plot_root_locus`, `animate_modal`); `plot_step_response` lives on
  `LTISystem` only; `bode`, `pzmap`, `margins`, `root_locus`, `step_response`,
  `nyquist`, `plot_nyquist`, `region_of_attraction`,
  `plot_region_of_attraction` and `modal_analysis` are band functions only.
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
  labels; `sys` versus `plant` across loop kinds (changes a tested key; settled
  by decision 5 of automation by convention below, if taken);
  plot labels for internal signals; a style sweep of display names.
- **CBF safety filter** ([cbf-safety-filter.md](docs/plans/cbf-safety-filter.md)):
  research lane until a course asks; the barrier is a `Field` once A1 lands.

**Automation by convention** ([automation-by-convention.md](docs/plans/automation-by-convention.md) §5, P10; v2 2026-10-10)

- **Principles and the two convention tables.**
  - Manual first: a loop is `add_subsystem` + `connect` of plain Systems; shortcuts build
    that diagram and store nothing else.
  - Port names drive the wiring.
  - Three reserved subsystem ids (`plant`, `controller`, `estimator`) are the roles every
    tool reads, at use time, never inferred from the graph.
  - State is `n`, and every leaf exposes `x`.
  - RULES 4.9 is extended with `e`, `x`, `q`, `dq` and the ids, and DESIGN gets a §4
    "Conventions" section.
  - The compromise (plan §3.9): standard diagrams resolve automatically. Custom port
    names, several controllers or several plants take a few manual steps, and they keep
    the plots, camera and cost through the role keys.

  Recommended: yes.
- **Standard names only.** `feedback_profile`, `PROFILE_PORTS`, the override attributes,
  `closed_loop`'s port keywords and the shape-based ids go. MPC becomes `x` → `u`. Custom
  names are wired by hand. Recommended: yes.
- **Loop inputs.** The plant, or the wrapper the loop builds around it, owns `w` / `v`;
  `r=`, `w=`, `v=` accept a source block; `estimator=` comes with P4. This reconciles
  randomness.md A9 / D24. Recommended: yes.
- **Cost relative to the plant.**
  - The evaluators build the loop themselves and score `trajectory_of(problem.sys)` by
    identity.
  - Controller and estimator states are never scored.
  - `command_port` (`u`, else the single non-`w`/`v` input) is the only input that
    planners, LQR, `place`, value iteration, Gym and the cost decide.

  Recommended: yes, in v0.3 with RN-4 / RN-5.
- **Role ids.** The roles are reserved subsystem keys; blocks get words and signals get
  symbols.
  - Renamed once, in flow and hybrid, before RN-4 names the random streams by id path:
    `ctl` → `controller`, `sys` → `plant`, `ref` → `reference`.
  - Sources are named `disturbance` and `noise`.
  - A role id wins over `System.id`.

  Recommended: yes, in v0.2.
- **P11's `Controller(feedback=…)`**: unnecessary once ports are the declaration.
  Recommended: drop the ask.
- **No controller classes.**
  - `Controller` and `DynamicController` are deleted, with no aliases, and the four
    course notebooks are migrated.
  - `plot_control_law()` moves onto `System`, beside `plot_input_output_map()`.

  Agreed in principle on 2026-10-10.
- **No operator modifies its operands** (plan §3.10, decision 8). `+`, `>>` and `@`
  return a new flat diagram sharing the operands' blocks; this reverses the v0.1 DESIGN
  record. Agreed on 2026-10-10. Code waits for the operator review to close.
- **`>>` is a plain serial connection** (decision 9). It connects one output to one input:
  one-to-one, else the same name, else `y` → `u`, else it refuses. There is no `r`
  preference and no hidden memory. Agreed in principle on 2026-10-10. A replay of the
  104 `>>` calls that run shows 100 unchanged.
- **Ruling:** the GRO860 name freeze (§4.1 gate 7) is paused between terms (maintainer,
  2026-10-10).

**Before v0.3 wave C (S72, the path-integral planner)**

- **MPPI** — proposed 2026-10-02 in [mppi.md](docs/plans/mppi.md) §10: the
  planner's name (`PathIntegralPlanner`, so that the composition with
  `ModelPredictiveController` spells the acronym, or `MPPIPlanner`); the three
  planner verbs the MPC block reads instead of trajopt internals
  (`decision_dimension`, `prepare_online`, `warm_start_guess`, and `z` on the
  tick's record — the shape of T6); the stochastic default (plant draws per
  control sample); where `RolloutEnvironment` lives once a second band reads it.
  Timing decided 2026-10-02: RN-4 and RN-5 first, then the planner.

**Before v0.9 (the freeze candidate)**

- `HybridDiagram` as a `System` vs `HybridLoop` (S31), and the sampled
  seam's time argument: integer ticks (today) or seconds, which would remove
  three copies of `t0` / `dt_mpc` in the MPC block.
- Evaluator / solver layering, Diffrax (S37, S27).
- The posed-geometry hook (S44) and the glyph / solid rename (S30).
- The frozen surface (V3): do `StepSystem`, `StepDiagramSystem`, `ZOHHold`
  and `control.mpc` join it, or sit behind a provisional name.
- The deprecation policy (V4): shims from v1.0 on, overriding RULES "no
  deprecated aliases" from then.
- The z tier (P6) in scope for v0.9 (proposed yes).
- A `MinilinkError` family (S70).

**Before v1.0**

- Whether AGENTS.md, CLAUDE.md, RULES.md and CONSTITUTION.md stay at the
  repository root or move under `docs/dev/` (V10).
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
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Open pyro parity rows (v0.3) |
| [docs/plans/](docs/plans/) | Design writeups for the steps that need one (see [plans README](docs/plans/README.md)) |
| [docs/reviews/](docs/reviews/) | Dated architecture audits and decision records |
