# Minilink Roadmap

Maturity and priorities, the **plan of record**; the steps themselves are the workboard,
[docs/plans/TODO.md](../plans/TODO.md), and the doc map is [AGENTS.md](../../AGENTS.md).

## 1. Releases

Identity: [CONSTITUTION.md](CONSTITUTION.md).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end**, plus a working **`pip install minilink` on PyPI**. Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via native `ReinforcementLearningPlanner` (with `Sys2Gym` + SB3 as an optional bridge). See §4.1. | Fall 2026 — `0.1.0` on PyPI (2026-09-16); `0.1.1` (2026-09-26) is the first release a GitHub tag published. Conda from `environment.yml` is the recommended install. The course notebooks' names are frozen until v1.0, with the freeze paused between terms (§4.1 gate 7). |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2). Conda stays the recommended install. | `0.2.0` on PyPI (2026-10-17), from the GitHub tag; every §4.2 row green |
| **v0.3** | **A cleaner core and the differentiable path; GMC714 covered** (§5.3; GMC714 needs no new feature, R48): **the convergence pass** (the core kept, side features to the experimental tier), **the continuous core** ([continuous-core.md](../plans/continuous-core.md): one step model under every sampled tool, one evaluation of any run, and the deterministic differentiable path to the cost behind a clean API: `evaluate`, `compare`, `problem.compile(loop)`, the program export, cost parameters), then the sampled loop as a tool if the calendar leaves room. The teaching surface covers every GMC714 lab (§4.3); the labs move from pyro to minilink as the maintainer's teaching work, outside this roadmap. | December 2026 |
| **v0.4** | **The draws and the textbook objects** (§5.4): the mean over the problem's draws through the same differentiable path (the Monte Carlo evaluator one path, its numbers changing once), the sampled face of the compiled problem, identification as its first data consumer; fields and workspace geometry; MPPI on the sampled kernel; the parametric MPC program (S76) and the general serial chain (C6). | Winter 2027 (the window between GRO860 cohorts, whose material the evaluator's numbers touch) |
| **v0.9** | **Freeze candidate** (§5.5): the foundation questions that could still break a public name are decided (derived `x0`, Diffrax as an optional solve, the glyph/solid rename, the frozen surface); the trust contract is in place (deprecation policy, pinned export list, CHANGELOG, pinned installs); the standard-syllabus holes close (estimation; the discrete-time one closes with S75 in v0.3); the docs site renders the tutorials. | Spring 2027 |
| **v1.0** | **A colleague can build a course on it and trust it for three years** (§5.6): the API frozen under the deprecation policy, the standard control syllabus covered and checked against textbook worked examples, course-neutral labs an instructor adopts outside UdeS, green on Windows, macOS and Linux; after two cohorts. Two Stable-Baselines3 teaching notebooks retire (S49) and the GRO860 name freeze becomes the general one. | 2027 |

## 2. Two lanes

Philosophy: [CONSTITUTION.md](CONSTITUTION.md) §3 (the student syntax never
breaks). Placement shorthand: [RULES.md](RULES.md) §3.3. The operating
contract is the table below — not a pointer.

Minilink serves two audiences with one codebase. The boundary between them is
a **contract**, not a documentation convention.

| Lane | What it is | Rules |
| --- | --- | --- |
| **Teaching surface** | The names students meet: root prelude (`from minilink import …`) and the band facades (`minilink.catalog`, `.blocks`, `.control`, `.analysis`, `.simulation`, `.planning`). Registered in one place, tested as a set. | *Soft rule:* nothing enters without a demo or notebook, a both-backends test where it defines dynamics, and a docstring. Names and semantics change only with a deprecation note. Student-facing examples and notebooks import **only** through it (CI-checked). |
| **Research lane** | Everything else: the provisional bands (hybrid, MPC, realtime), the experimental tier (`minilink/experimental/`: symbolic mechanics, the contact-engine leaf, C export, the unsupported plants; the top-level `experimental/` folder of prototype scripts), `examples/projects/`. | No stability promise, no entry requirements, out of the release contract. The provisional bands ship in the wheel. The experimental tier is **out of the wheel**, **untested by default** (nightly smoke at most; `c_export`'s flagship smoke and the quick C-export demo of `showcase_jax.ipynb` the two gate exceptions, ruled 2026-10-23), off the docs site, never cited by a course, and the staging area for what may leave into its own project at v1.0 (ruled 2026-10-22). Deep imports always remain valid. |

**Provisional teaching names:** the spatial names the planning facade exports (`Scene`,
`bind`, `ReferenceTrack`, `car_outline`, `plot_track`, …) sit in the teaching lane under
`planning.spatial`, checked like every band-facade name (a docstring, a teaching-lane home)
but outside the registry, and stay provisional until A3 moves them to `core/geometry/`.

**Wheel scope:** the published wheel is the `minilink` package without
`minilink/experimental/`: the teaching surface, the provisional bands with no stability
promise, and the provisional spatial names. `minilink/experimental/` and the research
examples (`examples/projects/`, the top-level `experimental/` folder) are repo-only.

**Breaking changes so far** (each migrated every course notebook in the same commit; from
v0.9 they go to `CHANGELOG.md`, step V5):
- 2026-09-15: every planner returns a `PlanningSolution` (`policy`, `solver`, `trajectory`,
  `evaluation`, `cost_to_go`) and the evaluator an `Evaluation`; `TrajectoryPlan`,
  `PolicyPlan`, `SolveMetadata` and `MonteCarloReport` are gone. `PlanningProblem.on_exit`
  is gone, `exit_cost` is `infeasible_cost`, and `X` is unconstrained unless declared.
- 2026-09-30: `WhiteNoise(p, *, psd, sample_period, seed, hold)` is a pure block; its
  `var`, `mean`, `t0`, `tf` and `refresh()` are gone (an old variance reads as
  `psd = var × sample_period`), and seeded realizations changed once.
- 2026-10-14: the state-feedback designs live in `control/design.py` (`lqr`, `place`,
  `lqr_finite_horizon`, `lqg`, `trajectory_lqr`); the modules `control/lqr.py`, `place.py`
  and `output.py` are gone, their names unchanged on the teaching surface.
- 2026-10-17 (`0.2.0`): a block's default `name` is its class name and a shortcut is named
  as the expression it builds, so diagram labels and plot titles changed; the pip extras are
  `full` and `dev` only (`[jax]`, `[plotting]`, `[visualization]`, `[diagrams]`,
  `[symbolic]`, `[rl]`, `[ipopt]` are gone); `install.md` and `environment-basic.yml` are
  gone, the install story is the README's.

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

Step ids (`S29`, `P4`, `T2`, …) are the rows of
[docs/plans/TODO.md](../plans/TODO.md).

| Area | Lane | TRL | State | Next |
| --- | --- | --- | --- | --- |
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested; wrong-shape `f` / `h` fails at `compile()`. | Automation by convention (P10, v0.2); derived `x0` (S29, v0.9). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; the frozen evaluator subset is named in DESIGN §5, the stable-internal helper grid kept by ruling. Speed lives in batches (1000 rollouts × 1000 RK4 steps in 27 ms); float64 on JAX by default. | the step model under every sampled tool (S75, v0.3); `rollout_batch` with a `params` family (S53); the differentiable closed-loop cost (V1: the deterministic path v0.3, the draws and the sampled face v0.4); the evaluator grid trimmed and the integrators moved to `simulation/solvers/` (S75's SM-6, v0.3); Diffrax (S27, v0.9). |
| Simulation | teaching | 7 | Mature; fixed 10 001-point default grid; `verbose` names unified. | One owner per rule (S65). |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA; **every catalog plant compiles on both backends** (contract test); four-rung vehicle ladder; UdeS racecar (kinematic, dynamic, 3-D). | Catalog hygiene (S64); the general serial chain (C6, v0.3); one mechanical family (S32, v0.3, R62). |
| Control | teaching | 6 | Linear, LQR (infinite horizon, finite horizon, along a trajectory), pole placement, reference scaling (`N`), `P` / `PI` / `PD` / `PID` and the rate-measured `ImpedanceController` / `ImpedanceIntegralController` as one family (what is measured decides the derivative; limits with infinite defaults, anti-windup once a limit is declared; the shared pieces three pure functions), model-based SMC and computed torque, robotic impedance and kinematic laws, neural policy block; a controller is a plain `System`. | P10's resolver and composition steps. |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with grid-free margins, pole-zero, root locus, Nyquist, the linear step response on matplotlib and plotly; the sensitivity functions `S` / `T` / `PS` / `CS`. Every verb takes any `System`, the linear model as is, the rest linearized at the operating point (2026-10-15). | Nichols and overlays later; Bode, poles and margins traceable for sensitivity to params (idea to review, S73). |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources (with `Sine`), TF, 1-layer NN. | `Ramp` / `Chirp` / `Delay` / `Switch` (C3, v0.3). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration (`loop` / `numpy` / `jax`), lookup controller, `PolicyEvaluator`, `LQRPlanner`; every planner returns a `PlanningSolution`; `compare()` reads solutions side by side; `dp.py` is the textbook reference of the style. | cost-to-go as a `Field` (A1); policy iteration (S51). |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot; `success` means defects satisfied to `feasibility_tol`; float64 by default on JAX. | Harden SciPy/Ipopt before TRL 6. |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt. | Harden SciPy/Ipopt before TRL 6. |
| Interfaces / RL bridge | research lane (bridge) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call. | Keep for external interop; courses solve with the native planner. |
| Analysis / Lyapunov certificates | provisional (research) | 4 | `region_of_attraction` → `LyapunovCertificate` (quadratic `V`, sampled level, `verify`, `plot`, `contains`); demo and showcase §11. | Cohort validation; `V` as a `QuadraticField` (A1); SOS and discrete time later. |
| Planning / RL planner | teaching (GRO860) | 6 | `ReinforcementLearningPlanner` (REINFORCE, actor-critic, PPO, SAC in pure JAX), `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control), `StochasticPlanningProblem`, `MonteCarloEvaluator`; on the root prelude; canonical demos in `examples/demos/rl/`, chapter 11 and the native teaching notebooks. | SAC actor step (S50); deep Q-learning (S52); automatic reward scaling ([rl-reward-scaling.md](../plans/rl-reward-scaling.md)). |
| Planning / search (RRT) | provisional → teaching by January (G3, R49) | 5 | RRT / RRT*; `RRTPlanner(problem)` works from the input bounds alone (bang-bang extender); returns a `PlanningSolution`. | RRT-Connect later. |
| Planning / path integral (MPPI) | research lane (planned) | 1 | Two standalone scripts (`experimental/mppi/`): the pendulum swings up; the kinematic racecar laps the cone circuit at a lower closed-loop cost than the collocation MPC. Design: [mppi.md](../plans/mppi.md). | The planner (S72, v0.4) on the sampled kernel, after RN-4 / RN-5. |
| Geometry / spatial | provisional (teaching) | 4 | SDF + `Scene` / fields / bodies under `planning/spatial/`, JAX twins tested; the planning facade's spatial names are provisional teaching names (§2). Home designed: `core/geometry/`. | Package move and course catalog (A3); bicubic SDF and CBF filter (research). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers; auto-fit camera. | Camera and plot defaults from roles (P10); constructor-derived camera hints (S43); glyph/solid rename (S30, v0.9). |
| Hybrid / step / MPC | provisional (research); MPC's teaching name by January (G3, R49) | 4 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, MPC with parametric JAX. A side helper for MPC simulation; the sampled loop is the one thing that is not a `System`. | Keeps in step with the core rules (P10); no new hybrid features before S31 (ruled a tool on one tick-scan kernel, R13–R14; its code after the continuous core); the compiled MPC program binds every problem parameter, not only `x0` (S76, v0.4; its names ruled: `ParametricProgram`, the key `plant`). |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review (v1.0); later the human-demonstration recorder of the behavior-cloning pipeline (after v1.0). |
| Estimation | teaching (GRO501) | 6 | `StateObserver` (innovation form, operating point on the block, hat labels), `luenberger` and `kalman` on arrays, a linear model or the plant (the duals of `place` and `lqr`, 2026-10-14); `lqg` returns the observer + gain box; an observer is refused as a controller; `closed_loop(gain, plant, observer=obs)` wires the LQG loop by role (AC-4, 2026-10-14). | the trajectory observer and the finite-horizon filter, the EKF, the estimation problem and its tools (v0.9, [the plan](../plans/estimation-and-state-feedback.md)). |
| Identification | planned | 2 | Parametric-tier prototype only. | V1's program export with a data cost (C4, v0.4). |
| C export (`experimental/c_export`) | research | 1 | JAX→C transpiler; two demos; flagship smoke in the JAX regression job. | Keep isolated; repo-only. |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Not on the teaching path nor the API site. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later. |
| Pyro 2.0 overall | v0.3 | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](../plans/pyro-port-remaining.md). |

## 4. Course objectives

Three courses drive the release contract: GRO860 (§4.1, v0.1, running now),
GRO501 (§4.2, v0.2) and GMC714 (§4.3, v0.3). A course is
"end to end" when every topic row is green on the teaching surface — in Colab
(git-clone cell) and in the conda env — and the cross-cutting gates hold.

### 4.1 v0.1 — GRO860 end to end

| Topic | Surface | Material | Gate |
| --- | --- | --- | --- |
| Value iteration / DP | `PlanningProblem`, `StateSpaceGrid`, `DynamicProgrammingPlanner`, `LookupTableController`, `plot_cost2go` / `plot_policy` | `pendulum_value_iteration`, `pendulum_value_iteration_vs_lqr`, `demos/value_iteration/` | `final_time` reads `problem.tf`; `success` reports convergence; notebooks wire with `vi_ctl @ plant` |
| LQR + linearization | `linearize`, `lqr` (on arrays, a linear model or the plant), `plot_control_law` | `03_control`, `04_analysis`, `demos/control/` | — (green today) |
| Trajectory optimization | `PlanningProblem`, `TrajectoryOptimizationPlanner` (`direct_collocation`, `shooting`), `QuadraticCost` | `09_planning`, `demos/trajopt/` | float64 by default on JAX; `success` means defects satisfied; canonical problems succeed with default optimizer |
| Reinforcement Learning (tabular on the grid, neural in native JAX) | `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control; `EpsilonGreedy`, `UCB`), `StochasticPlanningProblem`, `ReinforcementLearningPlanner` (REINFORCE / actor-critic / PPO / SAC), `NeuralPolicyController`, `MonteCarloEvaluator` | `11_reinforcement_learning.ipynb`, `pendulum_value_iteration_vs_lqr_vs_rl.ipynb`, `drone_ppo.ipynb`, `gymnasium_interface.ipynb`, `showcase_from_rl_to_bode.ipynb`, `demos/rl/` (pendulum, cart-pole, drone, rocket) | pure-JAX training on compiled plant (pendulum swing-up, 120k steps in 12.5 s on CPU, 0% failure over 50 trials); neural policy closed loop linearizes, simulates, and plots Bode; those names (plus `Evaluation`, `Gaussian` / `Uniform`) are on the root prelude. Notebooks solve with the native planner; Gymnasium stays taught as the domain standard and `Sys2Gym` + SB3 stay as the external bridge |

**Cross-cutting gates**

1. Wrong-shape `f` / `h` / port computes fail at `compile()` on both backends;
   a forgotten `super().__init__()` gives a named error; the README custom
   plant composes with `@`.
2. Default `compute_trajectory` returns a fixed output count (10 001, the fine
   reporting grid) and never selects a solver from the point count.
3. Every GRO860 notebook and every `examples/tutorial/`, `examples/teaching/`, and `examples/demos/`
   file imports only through the teaching surface (CI test).
4. The Basic tier (NumPy + SciPy + Matplotlib, nothing else) runs
   sim / plot / phase plane / animate / linearize / LQR / VI in a clean
   environment (CI test).
5. The Colab cell (git clone + path) and the conda environment from
   `environment.yml` both run every GRO860 notebook top to bottom.
6. `ruff` + `pytest` + notebook smoke green; nightly full demo sweep green.
7. No name a GRO860 notebook imports changes before v1.0. **Paused 2026-10-10**
   (maintainer): between terms the teaching design is unfrozen; a rename migrates every
   course notebook that uses the name in the same commit, with no alias. The freeze
   resumes when the next term's notebooks are pinned.
8. `pip install minilink` installs the teaching surface, published from a GitHub tag
   (Trusted Publishing in `.github/workflows/publish.yml`, the version read from the
   tag); extras (`[jax]`, `[visualization]`, …) match `pyproject.toml`. Conda remains
   the Full local stack (JAX, Ipopt, notebooks).

Out of the v0.1 checklist by decision: MPC/hybrid (provisional, lesson keeps
shipping), estimation, identification, frequency-domain tools (gated by §4.2).

### 4.2 v0.2 — GRO501 end to end

The classical-control course (*Systèmes asservis*, BSc Génie Robotique):
an APP problem on the UdeS-Racecar. APP2 is multi-physics modelling of the
DC-motor propulsion plus SISO speed and position loops designed to time- and
frequency-domain specifications and implemented as difference equations on an
Arduino; APP4 is the bicycle-model MIMO plant with LQR, nested loops, pole
placement and a Kalman observer. Student guide: `GRO501_Guide.pdf`
(maintainer's copy); references Dorf & Bishop and Åström & Murray.

Baseline audit:
[docs/reviews/2026-09-07-gro501-coverage.md](../reviews/2026-09-07-gro501-coverage.md).
The step-level plan doc retired 2026-10-17, its steps landed.

| Topic | Surface | Status | Gate |
| --- | --- | --- | --- |
| Multi-physics modelling — nonlinear `f`/`h`, block diagram, linearize, `H(s)` | custom `DynamicSystem`, `plot_diagram`, `linearize`, `transfer_function` | green | a longitudinal-vehicle plant in the catalog (met: `LongitudinalFrontWheelDriveCarWithTorqueInput` on `minilink.catalog`; a DC-motor plant is off the critical path, S71); pole/zero cancellation by default; order reduction by fast-mode neglect is out of v0.2 |
| Closed-loop analysis — poles, root locus, Bode, margins, step specs | `pzmap`, `root_locus`, `bode`, `margins`, `step_info`, `P` / `PI` / `PD` / `PID` | green | every compensator form reports the poles and zeros of the hand calculation, and margins are found wherever the crossover sits |
| Design to specification — rise time, overshoot, final error, phase margin | `PI`, `PD`, `PID`, `Lead`, `step_info`, `margins` (`Lag` removed, R54) | green | the Table 2 specs of the guide are checkable in one notebook |
| Loop-shaping specs — disturbance and measurement-noise sensitivity in dB at a frequency | `sensitivity`, `complementary_sensitivity`, `load_sensitivity`, `noise_sensitivity` | green (2026-10-10) | the `sensitivity_functions` teaching notebook |
| Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models) | green (2026-10-17) | the law is designed in continuous time, discretized, and the sampled loop validated by simulation; no z-domain analysis family (ruled 2026-10-17, [the record](../reviews/2026-10-17-z-tier-dropped.md)) |
| State-space MIMO — bicycle model, controllability at every nominal speed | `KinematicBicycle`, `controllability`, `observability` | green | — |
| Optimal control — LQR on the guide's cost, closed-loop poles, nonlinear check | `lqr(plant, Q, R, x_bar=)`, `StateFeedbackController` | green | — |
| Pole placement — `K_sta` for a prescribed pole set | `place` (on a linear model or the plant; `place_gain` on arrays) | green | — |
| Nested loops — inner speed loop, outer position loop | `@` composition | green for error-driven compensators (`PID @ (PID @ G)`); an outer `r, y` or state controller is refused today | P10 ([automation-by-convention.md](../plans/automation-by-convention.md)) nests the inner loop as the outer loop's `plant`; stays green with the observer in the loop |
| State estimation — Luenberger observer and Kalman filter | `StateObserver`, `luenberger`, `kalman`, `lqg` | green (2026-10-14) | the observer closes the loop as a standard diagram block: by role with `closed_loop(gain, plant, observer=obs)`, in the `lqg` box, or by hand |
| Reference scaling — the `N` matrix giving `y = r` at steady state | `OutputReferenceController(K, N)` | green (2026-10-10) | the student writes `N = -(C (A - B K)⁻¹ B)⁻¹`; the block takes the output reference |

**Cross-cutting gates**

1. Every topic row above is green, with a demo or a notebook.
2. No APP notebook in the repository: the students write the APP work themselves (ruled
   2026-10-15). The demos and notebooks of gate 1 run Colab-first on the Basic tier
   (NumPy + SciPy + Matplotlib) — no JAX on the GRO501 path.
3. Every tool agrees with the hand calculation for the guide's §9 exercises;
   the closed-loop analysis exercises (§9.6, §9.7) are the acceptance test.
4. Student-facing GRO501 material imports only through the teaching surface.

Out of the GRO501 checklist by decision: hardware, ROS, and the Arduino
firmware itself (the course owns those); system identification from logged
runs beyond what `identification/` already offers.

### 4.3 v0.3 — GMC714 end to end

The robotics course (*Robotique : modélisation et commande*), winter term, week 1 on
January 9. Syllabus: [the course page](https://www.alexandregirard.com/courses/gmc714/) and its
notes (*Notes_Robotique_UdeS*), read 2026-10-23. Its labs run today on pyro, through Colab
notebooks one per homework. **Ruled 2026-10-23 (R48): GMC714 needs no new feature.** Its labs
move from pyro to minilink as the maintainer's teaching work, outside this roadmap; the
release's part is that the teaching surface covers every lab, checked the same day:

Names as ruled by R52 and R63 (the renames land with A6 before the term).

| Week | Topic (notes chapters) | The lab's numerical part | Surface | Status |
| --- | --- | --- | --- | --- |
| 1 | Phase space, model types (1, 7) | lateral control of a bicycle model, the phase plane of the closed loop | `KinematicBicycle`, `sys.plot_phase_plane` | on the surface |
| 2–3 | Arm kinematics, then dynamics (2–6) | forward and inverse kinematics, Jacobian, manipulability; `M`, `C`, `g` (mostly by hand) | `TwoLinkManipulator`, `ThreeLinkManipulator`, `UR5Manipulator` | on the surface |
| 4 | Arm control I, quasi-static (7, 8) | impedance control of a 3-DoF drilling robot in simulation | `ThreeLinkManipulator`, `Impedance`, `TaskImpedance`, `JointImpedance` | on the surface; the drilling contact is the notebook's |
| 5 | Arm control II, dynamic (9, 20) | computed torque, sliding mode, adaptive control | `ComputedTorque`, `SlidingMode`, `DynamicSystem` | on the surface; the adaptive law built by hand in the lab as a custom `DynamicSystem` with the controller's ports (pyro's `DynamicController`; R48) |
| 6 | Vehicle modelling (11) | longitudinal model with tire slip; the dynamic bicycle, oversteer, linearization | `LongitudinalTorqueCar`, `DynamicBicycle`, `linearize` | on the surface |
| 7 | Vehicle planning (13, 16, 21) | trajectory optimization of a pendulum (`Q`, `R`, the torque limit, `n`, `dt`); RRT then trajectory optimization on a cart-pole | `TrajectoryOptimizationPlanner`, `RRTPlanner`, `QuadraticCost`, `Pendulum`, `CartPole` | on the surface |
| 8 | To come | — | — | — |
| 10–13 | Projects | "a new system or controller integrated into the library" | the `System` contract | on the surface |

Not a week of the syllabus: MPC (its teaching name stable by January all the same, R49), robust control as a topic of its own
(one section of week 5, the sliding mode), the draws and Monte Carlo, gradient tuning.

**The gate:** every row above stays on the teaching surface through the v0.3 changes, so a lab
written against the v0.3 interface needs nothing more; and (R49) the tools the labs use carry
one public, stable teaching name by January (RRT, MPC, trajectory optimization, computed
torque, sliding mode, the task- and joint-space impedance laws, task-space position control
with a nullspace posture, `Manipulator`), frozen for the term (TODO §4, G3). The lab notebooks themselves are the
maintainer's teaching design (AGENTS), written when he chooses; agents draft on request.

## 5. The path to v1.0

Each rung names its open steps, one line each. The steps themselves, with files, order and
"done when", are the rows of [docs/plans/TODO.md](../plans/TODO.md). A rung is done when
every step in it is landed or explicitly dropped by the maintainer. Landed work leaves this
section; its history is in git and in [docs/reviews/](../reviews/). This section names the
rows of the architecture and course waves; wave D's rows (the T rows, D1–D3, S43, S53, S65–S69,
A4, P4) are named in TODO only.

**The order is the commitment; the course calendar is secondary** (ruled 2026-10-22,
[record](../reviews/2026-10-22-second-roadmap-review-rulings.md)). GRO860 runs in the fall
term (running now) and GMC714 in the winter term, which v0.3 prepares. Two windows the plan
cites: *between terms* is late December 2026 to early January 2027 (the between-terms moves
of K7 and A6, V11); *between cohorts* means between GRO860 cohorts, the course whose material
reads the Monte Carlo evaluator and the RL numbers (v0.4's one-time number changes). Neither
window delays a wave: a step that would move a running course's numbers ships behind the
pinned release the course runs on (V6) rather than waiting.

**The shape the rungs build** (proposed, the first ask of K7, maintainer-owned:
[core-consolidation.md](../plans/core-consolidation.md) §2). One spine every approach is a
special case of: a `System`, compiled, rolled out to a `Trajectory`, scored by a cost,
lowered to a `MathematicalProgram`, solved by the `Optimizer`. Two step paths on it: the
continuous one (`Simulator`, the differentiable evaluator) and the sampled one (the step
model and the sampled loop, [sampled-path.md](../plans/sampled-path.md)). Six pillars on the
spine: classical and modern control, nonlinear control, trajectory optimization and MPC,
reinforcement learning, identification and imitation, estimation. v0.3 converges the library,
builds the sampled path and the deterministic differentiable path to the cost, then adds what
GMC714 is missing; v0.4 adds the draws and the textbook objects; v0.9 decides what can still break a name; v1.0 freezes.

**At a glance** (indicative dates; the order is the commitment, the dates are not):

```mermaid
gantt
    title minilink, the path to v1.0
    dateFormat YYYY-MM-DD
    axisFormat %b %Y
    section Decide
    Rulings sessions 1 and 2 (R1-R29)     :done, rul1, 2026-10-22, 2d
    GMC714 audit G1 (no gap, R48)         :done, g1, 2026-10-23, 1d
    section v0.3 cleaner core
    Wave K convergence (K1-K4, K7-K9)     :k, 2026-11-01, 3w
    section v0.3 continuous core
    Step model S75 (SM-0 to SM-7)         :s75, 2026-11-02, 18d
    V1 steps 1-2, Trajectory, evaluation  :v1a, 2026-11-02, 12d
    V1 steps 3-5, run, engine, program    :v1b, after s75, 10d
    A2 cost parameters (V1 step 6)        :a2, after v1b, 5d
    Sampled loop S31, if room             :s31, after a2, 7d
    v0.3 tag                              :milestone, v03, 2026-12-21, 0d
    section v0.4 draws and objects
    V1 steps 7-9, the draws, C4           :v1c, 2027-01-04, 4w
    S76 parametric program, wave G rest   :s76, 2027-01-04, 4w
    V1 sampled face, A1 fields, A3        :a13, after v1c, 6w
    MPPI, C6 serial chain, RL follow-ups  :mp, after v1c, 5w
    v0.4 tag                              :milestone, v04, 2027-04-30, 0d
    section v0.9 freeze candidate
    Foundations and the trust contract    :v09, 2027-05-03, 8w
    section v1.0
    Two cohorts, then the freeze          :v10, 2027-07-01, 20w
```

### 5.1 v0.1 — close-out

The GRO860 path is green (§4.1) and `0.1.1` is published; nothing is open.

### 5.2 v0.2 — shipped; the carry-over

`0.2.0` was tagged on 2026-10-17 with every §4.2 row green. The steps below were planned for
v0.2 and not needed for the tag; they carry into v0.3 and v0.4, each in the slot the order of
TODO §4 and §5 gives it, wave D standing.

**Wave A — the textbook objects finish speaking.**
- **A6** Short catalog names (RULES 2.6): compact public class names, the long explicit display
  names dropped, one ruled table of renames migrating the notebooks in the same commit (between terms,
  with wave K's between-terms moves, while the name freeze is paused, and before the GMC714 term
  starts on January 9, R50). **[ask — public
  names]**
- **A5** The later nouns, each with its first consumer: `log_prob`, sets and distributions
  over parameter dictionaries, `UnionSet`, `PlanningProblem.hamiltonian()` only if a course
  teaches Pontryagin (sets reading `params` with A2, V1's step 6 in v0.3, `Set.as_cost` with V1's step 8
  in v0.4).
- **RN-2** of [randomness.md](../plans/randomness.md): distributions read `params`,
  `Gaussian(cov=)`, every `sample` takes a key (before V1's step 7, the draws, in v0.4).

**Wave B — GRO501 end to end** (§4.2; the plan doc retired 2026-10-17, its steps landed).
- **P10** Automation by convention ([automation-by-convention.md](../plans/automation-by-convention.md),
  decided 2026-10-11): standard port names drive the wiring, three reserved names (`plant`,
  `controller`, `observer`) are the roles every tool reads, and the cost is scored on the
  plant. Its composition steps land in this wave; the order is in TODO §2 and the plan's
  §7.

**Wave D — the code reads like the textbook, and the demos like the API.** Standing work,
behaviour-preserving, one module per step.
- The textbook pass landed 2026-10-16/17 ([record](../reviews/2026-10-16-textbook-pass.md),
  [rulings](../reviews/2026-10-17-textbook-pass-follow-up-rulings.md)); its behaviour and API
  leftovers are TODO §3's T rows.
- **D1** Demos and notebooks to the minimal rule (RULES 6.1 / 6.10 / 6.11). **[ask per
  file]**
- **D2** Consolidation picks still open from the 2026-09-05 inventory.
- **D3** Hardening rows carried over (research lane).
- **S65** One owner per rule (the automatic `dt`, the backend fallback, the input-hold
  model, the Monte Carlo defaults); **S66** wiring mistakes fail at wiring time **[ask —
  core]**; **S67** one plot vocabulary; **S68** contracts as tests.

### 5.3 v0.3 — a cleaner core and the differentiable path (December 2026)

Reorganized 2026-10-21 ([record](../reviews/2026-10-21-roadmap-reorganization.md)); the
steps, in order, are TODO §4. **First, November: a cleaner core and the full differentiable
path to the cost behind a clean API** (wave K and the continuous core). GMC714 needs no new feature (R48:
the audit G1 found the teaching surface covers every lab), so v0.3 is the clean-up and the
continuous core.

**Rulings first**, in two sessions ordered by what each unblocks: the shape, the convergence
and the step model, then the sampled loop, the compiled problem and the names. No foundation
code before the session that rules it. **Both sessions ruled on
2026-10-22/23** (R1–R29, [record](../reviews/2026-10-22-rulings-session-1.md)): the design of record for
the continuous-core phase is [continuous-core.md](../plans/continuous-core.md) (the decision
ledger with the alternatives: [end-state-api.md](../plans/end-state-api.md)). **Done 2026-10-23:**
the plan docs aligned to it and its steps written as workboard rows with gates (TODO §4); the
points that pass and an independent second opinion raised are under "Raised 2026-10-23" in §6.
**Next:** implement, the step model's baseline (SM-0) first. The sampled loop's details
(D13–D15), the optimizer upgrade (S74, deferred by R25) and the v0.4 names stay open.

**The audit, done 2026-10-23** (G1): the GMC714 syllabus read and every lab's ingredients
checked on the teaching surface (§4.3); no gap; the maintainer ruled that GMC714 needs no new
feature (R48).

**Wave K — convergence** (November, first, so everything after lands in a smaller library):
- **K1** The inventory: every public name and every demo with its callers and a maturity
  label; [core-consolidation.md](../plans/core-consolidation.md) §3–§4 is most of it.
- **K2** Unused features removed and **K3** prototypes moved to `minilink/experimental/`,
  from one ruling pass: K7's tiers. **[ask — removals, public names]**
- **K4** One demo per feature, before D1.3 sweeps the rest: the maintainer's own pass (R54).
- **K10** The package structure: `planning/` regrouped by pillar (dynamic programming, trajectory optimization with MPC, search), RL to a new `minilink.learning` band (R66). **[ask — public names]**, ruled.
- **K8** The catalog clean-up (R2: the root prelude trimmed to the course set, `minilink.catalog`
  the flat index of tested plants, the unsupported ones to the experimental tier) and **K9**
  the experimental tier out of the CI gate (R3). Ruled.
- **K7** The convergence pass: the spine, the six pillars and the named content stay; side
  features leave for the experimental tier or the example that motivated them; the first
  tier any time, the between-terms moves in the late-December window (§5's calendar rule;
  §6). **[ask — core, public names]**

**The continuous core** (November, beside wave K; the design of record
[continuous-core.md](../plans/continuous-core.md), ruled R1–R29; the steps and gates are TODO
§4):
- **S75** The step model ([sampled-path.md](../plans/sampled-path.md)): one compiled step taking
  `params`, `discretize(plant, dt, *, solver, substeps) -> StepSystem` on it for every
  sampled consumer, `compute_forced` on the same rollout (a sampled input held, its forced numbers changing once, R18, R40),
  the evaluator trimmed to four traced bodies and the integrators moved to `simulation/solvers/`
  (steps SM-0 to SM-8; S37 absorbed, S27 Diffrax stays in v0.9). **[ask — core]**, ruled.
- **V1** The path to the cost (steps 1–6 and the Monte Carlo fold of
  [differentiable-problem.md](../plans/differentiable-problem.md)): the native `Trajectory`, one
  evaluation of any run (`evaluate_trajectory` and the `Evaluation` record, the integral to `tf`,
  the problem's fields with one meaning each), the whole run out of the scan,
  `problem.compile(loop)` with `evaluate` and `compare` as its facades, the program export to
  today's `Optimizer` (no tuning facade, R23), and the Monte Carlo evaluator folded into
  `evaluate(..., n_trials, key)` (R44); the record's names `cost`, `J`, `objective` (R45). A
  sampled controller is refused until the hybrid phase; the noise blocks' per-trial seeds are
  v0.4 (§5.4). Nothing merges to `main` until every notebook has moved (R47). **[ask — core]**, ruled.
- **A2** Cost parameters ([cost-params.md](../plans/cost-params.md)), V1's step 6, so the path
  reaches the cost's own weights. **[ask — core]**, ruled.
- **S31** The sampled loop, ruled a tool with ticks (R13, R14), after the continuous core: in
  v0.3 if the calendar leaves room, else the first core row of v0.4; its details
  (D13–D15) parked until then. **[ask — core]**
- **S74** The optimizer upgrade: deferred by R25 to after the continuous core, with OPW-1–OPW-5;
  it lands before S76 (§5.4), which builds on its backend factory.

**GMC714** (R48): no new feature; the teaching surface already covers every lab (§4.3).
- **G3** GMC714's names stable by January (R49): `RRTPlanner` to the teaching lane, the robotic
  laws onto the prelude, the textbook model forms on a new `minilink.dynamics` band
  facade, MPC as `ModelPredictiveController` with a frozen constructor (R50); A6's short catalog
  names land before the term (R50).
- **S32** One mechanical family (R62, pulled forward from after v1.0): `GeneralizedMechanicalSystem`
  a child of `MechanicalSystem` beside `Manipulator`, `H` the one mass-matrix name, the model-based
  controllers accepting every mechanical plant. **[ask — core]**, ruled.
  **[ask — public names]**
- The former wave G rows (C1's pyro parity and migration guide, C2's robotic PID wrappers, C3's
  blocks, S63, S64) move to v0.4 (§5.4); the labs are the maintainer's teaching work, outside
  this roadmap.

### 5.4 v0.4 — the draws and the textbook objects (winter 2027)

Winter 2027, the window between GRO860 cohorts (§5's calendar rule: the GMC714 term runs
meanwhile and does not gate it); the steps, in order, are TODO §5.

**The draws on the path** ([differentiable-problem.md](../plans/differentiable-problem.md)):
- **V1, the draws' noise** (steps 7–9): one seed per random block in `problem.realize(key, n)`
  under the same `evaluate(..., n_trials, key)` (the fold itself landed in v0.3, R44), as one row
  with **RN-3 / RN-4 / RN-5** of [randomness.md](../plans/randomness.md) and **AC-7** (the cost
  scored on the plant); the sets as costs; the material. The engine and `program` stay
  provisional until GMC714 or GRO860 has run them (R28). **[ask — core]**
- **V1, the sampled face** (step 7b): `problem.compile(sampled controller)` dispatches to
  S31's kernel on its JAX face, so a digital PID, a held neural policy, MPPI and the RL
  policies get the same `evaluate` and `program` as a continuous law. **[ask — core]**
- **S76** The compiled MPC program binds every problem parameter, not only `x0`
  ([parametric-program.md](../plans/parametric-program.md)): the start, the plant, the cost
  and the sets as one parameter tree bound at each solve; recommended, one program family
  whose frozen form is a binding (the plain `MathematicalProgram` unchanged); its names ruled
  (`ParametricProgram`, the key `plant`, D19); on S74's backend factory, so S74 lands first.
  **[ask — core]**
- **C4** Identification as the path's first data consumer: V1's program export with a data-mismatch
  cost; trajectory generation and experiment design beside it.
- **LQGPlanner** with RN-4.

**The textbook objects:**
- **A1** `core/fields.py` ([fields.md](../plans/fields.md)): the Lyapunov `V` and the
  value-function approximators become fields. **[ask — core]**
- **A3** Workspace geometry, `core/geometry/` ([geometry-module.md](../plans/geometry-module.md)),
  `PurePursuit` on a `Track`. **[ask — core]**

**On the spine:**
- **S72** The path-integral planner (MPPI) on the sampled kernel, after RN-4 / RN-5
  ([mppi.md](../plans/mppi.md)). **[ask — name, the MPC block contract]**
- **C5** RL follow-ups: **S50** SAC actor step, **S51** policy iteration on the grid,
  **S52** deep Q-learning, and the standard stochastic policy as a seeded block (the RL
  heads rebuilt on it, the RL numbers changing once).
- **C6** General serial chain: one mechanism description (DH or URDF) feeding RNEA / ABA, the
  GMC714 arm topic's modelling rung; design first. **[ask — core]**
- The former wave G rows (R48): **C1** pyro parity and the pyro → minilink migration guide,
  **C2**'s robotic PID wrappers, **C3** the blocks (`Ramp` / `Chirp` / `Delay` / `Switch`),
  **S63** DP reading a finite horizon, **S64** catalog hygiene.
- **S73** Traceable linear analysis, any time, self-contained.

### 5.5 v0.9 — freeze candidate (spring 2027)

What v1.0 asks is adoption, not architecture: a colleague builds a course on minilink and
trusts it for three years. The
[adoption review of 2026-09-26](../reviews/2026-09-26-adoption-review.md) found the
engine ready and three things missing around it: a stability promise, the two syllabus
holes every course hits (estimation, discrete time), and material packaged for an
instructor. The discrete-time hole closes with S75 in v0.3 (§5.3); v0.9 closes the stability
promise and the estimation hole, and settles every question that could still break a
public name; v1.0 (§5.6) freezes.

**The foundations that can break names** — decided before the freeze:
- **S29** `DiagramSystem.x0` / `n` / `state` as derived properties. **[ask — core]**
- **S27** Diffrax as an optional JAX solve, on the integrators S75's SM-6 moved to
  `simulation/solvers/` (S37 is absorbed by S75). **[ask — core]**
- **S30** The glyph / solid rename; **S44** one posed-geometry hook. **[ask]**
- **V3** The frozen surface: which names the freeze covers. **[ask — public names]**

**The repository's shape** — between terms, after GMC714 and before the freeze:
- **K5** Course material and/or the research projects in their own repositories
  (§6). **[ask — student-facing]**
- **K6** The move, if K5 says so: history kept, the citing course documents updated, the new
  repositories pinned on a minilink release. **[ask — student-facing]**

**The trust contract:**
- **V4** Deprecation policy from v1.0: one minor release with a `DeprecationWarning` shim;
  a snapshot test pins the teaching-surface export list. The renames held back to v1.0
  (`xbar` / `ubar`, `plot_cost2go`; `JaxMechanicalSystem` is deleted instead, R61) are on its list. **[ask]**
- **V5** `CHANGELOG.md` for users; every release from v0.9 carries its entry.
- **V6** The Colab cell installs a pinned release (the install story itself landed
  2026-10-17: conda recommended, `pip install minilink` plus one `full` extra, no
  `install.md`).
- **S70** A `MinilinkError` family (wiring, shape, solver) under the plain
  `ValueError` / `TypeError`. **[ask — core]**
- **V11** A light history: the notebook outputs committed before `nbstripout` leave the
  git history, once, in December 2026 after the v0.3 tag. **[ask]**

**The syllabus holes** (beyond GRO501's P4):
- **`discretize` follow-ups**: moved to S75 in v0.3 (its step SM-2).
- **Estimation follow-ups** after P4: EKF, time-varying and discrete Kalman; online parameter
  estimators (recursive least squares, gradient laws).
- **C4** Identification: moved to v0.4 (V1's program export with a data cost).
- **S71** The classic first plants (DC motor, tank / thermal process, ball and beam,
  differential drive, 3-D quadrotor); LQI beside the `N` matrix.

**The reference site:**
- **V7** The docs site renders the tutorials, a plant gallery with pictures, and a short
  concept guide.

### 5.6 v1.0 — a colleague can build a course on it (2027)

After two cohorts have run on v0.2 and the freeze candidate has held a term:
- **V8** A textbook-validation suite: Dorf / Ogata / Franklin worked examples asserted
  numerically; GRO501 gate 3 is its seed.
- **V9** Course-neutral labs (10–15 in `examples/teaching/topics/`) and a "For
  instructors" page. **[maintainer]**
- **V10** The public face: a teaching quickstart in the README; no internal vocabulary in
  user-facing files; `CONTRIBUTING.md`, issue templates, a code of conduct. **[ask]**
- **V2** Release hygiene: Zenodo archive and DOI, `CITATION.cff`; the JOSS decision.
- **S49** The four Stable-Baselines3 notebooks of GRO860: `drone_ppo_sb3` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3` retire; `particle_gymnasium_sb3` and
  `gymnasium_interface` are decided then. **[ask]**
- The realtime band's architectural review (`RealtimeSimulator`, the pygame I/O). **[ask —
  core]**
- The freeze: the V3 surface under the V4 policy; the GRO860 name freeze becomes the
  general one.

**After v1.0 (1.x features, additive by construction):** **S36** iLQR from parts (research lane); a
**behavior-cloning pipeline** end to end:
a game mode on `RealtimeSimulator` that records human demonstrations (keyboard or joystick
driving the plant live) as trajectory datasets, a planner that fits a policy to them
(imitation beside the RL planner, the neural policy block as its output), and the cloned
controller closed on the plant and compared against the demonstrator **[ask]**. How the
current proposals carry it, and diffusion policy after it:
[future-policies.md](../plans/future-policies.md).

**Simplify and consolidate — a standing principle, not a rung.** The repo must stay
manageable by one maintainer, so every rung carries consolidation work, and the target is
*maintenance cost*, never features: text edited twice when code changes, dead API,
boilerplate a flag would replace, twins one `xp` body covers, research scenarios inside the
teaching tree, plan docs already self-marked complete. A deliberate ladder of
implementations (the DP planner's three backends) is not duplication. The maintainer picks
each item; agents execute only what was picked.

## 6. Decisions

Open decisions only, by the rung they block; each needs the maintainer. A decision leaves
this section when it is taken: its ruling goes to the plan doc it shapes and to a dated
record in [docs/reviews/](../reviews/), and its contract to DESIGN when the code lands.
The automation-by-convention rulings (2026-10-11) are in
[their record](../reviews/2026-10-11-automation-by-convention.md); the disturbance
convention in [randomness.md](../plans/randomness.md); the docs-consistency rulings
(2026-10-12) in [theirs](../reviews/2026-10-12-docs-consistency-rulings.md); the
state-space design rulings (2026-10-14: the nominal state, the model-taking verbs, the
observer, `lqg`, the role name) in
[theirs](../reviews/2026-10-14-state-space-design-rulings.md); the linear analysis band
rulings (2026-10-15: the step response linearized and labelled, the structural verbs on a
system, the command port as the default channel) in
[theirs](../reviews/2026-10-15-linear-analysis-rulings.md); the classical control
family rulings (2026-10-15: `P`, the impedance pair on the family's grammar, matrix
stiffness and damping, the shared pure functions) in
[theirs](../reviews/2026-10-15-classical-control-family.md); the z tier dropped (2026-10-17) in
[its record](../reviews/2026-10-17-z-tier-dropped.md); the one install story (2026-10-17) in
[its record](../reviews/2026-10-17-one-install-story.md); the root layout (2026-10-17) in
[its record](../reviews/2026-10-17-root-layout.md); the roadmap reorganization (2026-10-21:
v0.4 added, the course audit G1 now, the sampled spine before the differentiable one) in
[its record](../reviews/2026-10-21-roadmap-reorganization.md); the second roadmap review
(2026-10-22: the calendar rule, one home per item, wave G ranked by G1, the two rulings
sessions, the sampled face of V1, the missing rows) in
[its record](../reviews/2026-10-22-second-roadmap-review-rulings.md).

**Ruled 2026-10-22/23, the two sessions** (R1–R29,
[record](../reviews/2026-10-22-rulings-session-1.md); the design of record
[continuous-core.md](../plans/continuous-core.md)): the shape (three foundations, six tooling
pillars), the catalog clean-up (K8), the experimental tier untested (K9), no exit rule, the
silent prelude trim; the step model's contract, knobs and defaults, DP on Euler, one traced
body per integrator; the sampled loop a tool with ticks (S31: R13, R14); one engine, the
compiled fixed-step run the judge, `evaluate` and `compare` its facades (R15, R16); one
evaluation of any run, the integral to `tf` with no cut, the terminal cost charged at a finite
`tf` and exits priced only by `infeasible_cost` (R19); the cost object (R20), the sets explicit
(R21), the loop the pipeline reads (R22), the program as this phase's export (R23), the
criterion on the problem (R24), the names and the record (R26, R27; S76's names among them:
`ParametricProgram`, the key `plant`), the lane (R28), `x_goal` and `Xf=None` (R29); the
optimizer upgrade deferred (R25). Parked to the hybrid phase: D13–D15 (the run-state caches,
controller memory and draws on the tick, the dispatch of a sampled controller). To discuss
when convenient, not blocking: how the proposals carry MPPI, behavior cloning and diffusion
policy ([future-policies.md](../plans/future-policies.md) §7).

**Ruled 2026-10-23, the evening session** (R30–R47, the same
[record](../reviews/2026-10-22-rulings-session-1.md), on the points of
[the alignment and second-opinion record](../reviews/2026-10-23-v03-alignment-and-second-opinion.md)):
the GMC714 audit now (R30); a non-finite run failed (R31); the price one switch read the same
way by every pipeline, `infeasible_cost=None | True | a number`, with the cost's upper-bound hint
and `problem.cost_bound()` for DP and RL (R32); `x_goal` never creates `Xf` (R33); the
criterion's three values reducing `J` (R34); the settings map (R35); `dt` required on `evaluate`
and `compare` (R36); `U` unbounded unless declared, `plant.input_box` added (R37); the problem
never fills `r` (R38); role-less trajectories (R39); the hold on sequences only (R40); one home
per parameter, `ProblemParameters` retired (R41); `method=` for the verb's own method, `solver=`
for time (R42); the RL episode a training setting (R43); the Monte Carlo fold in v0.3 (R44); the
record's names `cost`, `J`, `objective` (R45); no regularizer hook (R46); v0.3 on `dev` until
every notebook has moved, no pin (R47).

**Still open from the second opinion:** v0.3's scope against the maintainer's attention, the
proposed cuts to v0.4 (SM-7, SM-6's move, A2, K4, K8-3, S31), now that GMC714 needs no feature
(R48) (record item 8; item 9 is answered
by R48: neither MPC nor the draws); the slots of S31 and S74 (item 18; with R49, MPC's loop syntax, `mpc @ plant` as every demo and lesson writes it, must stay
stable through S31, whichever slot it takes); `discretize`'s `params["dt"]` fallback
(item 21, dropped by R70); and the program with the price off (the default), where one
diverging trial makes the objective `+inf`: written as a caveat (set `infeasible_cost=True` for a
search that may reach unstable gains), to confirm.

**Before v0.4 (S76, the parametric program)**

- **The compiled MPC program's parameters** (S76): one program family, the library
  building `J(z, p)`, `h(z, p)`, `g(z, p)` on the problem's parameter tree and the frozen
  `MathematicalProgram` its binding, or two parallel types, or one class whose functions
  always take `p`. Recommended (2026-10-22): the family, so hand-written programs stay `J(z)`
  and one evaluator serves both (end-state-api.md D25). Still to rule with it (the names apart,
  ruled: `ParametricProgram`, the key `plant`):
  whether a hand-written program may declare its own parameters, and the MPC block's
  live-parameter ports. [parametric-program.md](../plans/parametric-program.md) §2–§3.

**Before v0.3's convergence (wave K)**

- **What is core, what is a side feature** (K7): closed 2026-10-23 (R69), answered by R1–R5 and
  R52–R67.

**Before the former wave G rows (v0.4, R48)**

- **The optional `KinematicModel` delegate**: adopt or drop, decided with C6's mechanism
  description.
- **Mechanical default bounds** (S64): ruled 2026-10-23 (R68): each plant states its own, the base
  class invents none; lands in v0.3 with V1.2.
- **Which pyro demos the courses still need** (drives the parity rows).
- **Pyro game demos** — port the rest to `simulation/realtime/` or drop (the
  behavior-cloning game mode after v1.0 would reuse them).

**Standing (wave D)**

- **Undeclared feedthrough** (S66): an output that moves with an input it does not declare
  simulates a wrong fixed point silently today; raise or warn at compile.
- **Naming, the larger alignments**: one separator rule for wires, params paths, block
  labels and duplicate state labels; plot labels for internal signals (the display-name
  sweep is step A6).

**Before v0.4**

- **Lyapunov certificates**: a GRO860 checklist row of their own, or an analysis tool the
  lecture uses; `method=` naming the certificate family.
- **Solver-specific views on the planning records.**
- **`PlanningProblem.metadata`**: document it or retire it.
- **MPPI** ([mppi.md](../plans/mppi.md) §10): the planner's name (`PathIntegralPlanner`
  or `MPPIPlanner`); the three planner verbs the MPC block reads instead of trajopt
  internals; the stochastic default (plant draws per control sample); where
  `RolloutEnvironment` lives once a second band reads it (answered by S75 and S31 if they
  are ruled as recommended: it becomes a caller of the step model and the sampled kernel).
- **CBF safety filter** ([cbf-safety-filter.md](../plans/cbf-safety-filter.md)): research
  lane until a course asks; the barrier is a `Field` once A1 lands.

**Before v0.9 (the freeze candidate)**

- **Where the course material and the projects live** (K5): `examples/teaching/` (course
  notebooks, cited by the GRO860 notes and webpage and GRO501's `\colab{}`) and
  `examples/projects/` (research projects) stay here, or move to their own repositories.
  Here: one checkout, one CI, renames migrate them in one commit. Apart: a library repo that
  is only the library, course material versioned per term and pinned on a release, but two
  CIs and a release before every notebook change that needs a new feature. Move between
  terms only. Options and a recommendation (pin the course notebooks on a release now, a
  course repository at v1.0; with the examples-tree and package-split questions):
  [the repository-structure writeup](../plans/repository-structure.md).
- Diffrax as an optional JAX solve (S27).
- The posed-geometry hook (S44) and the glyph / solid rename (S30).
- The frozen surface (V3): do `StepSystem`, `StepDiagramSystem`, `ZOHHold` and
  `control.mpc` join it, or sit behind a provisional name.
- The deprecation policy (V4): shims from v1.0 on, overriding RULES "no deprecated
  aliases" from then.
- A `MinilinkError` family (S70).

**Before v1.0**

- Zenodo / DOI, and whether JOSS is wanted (V2).

## 7. Out of scope

By decision — see [CONSTITUTION.md](CONSTITUTION.md) §1.4:

Full Simulink parity (GUI, DAE, arbitrary multi-clock scheduling,
event-driven switching as a framework feature); becoming a multibody/contact
OS or a batched RL physics engine.

What *is* in scope but subsidiary: the sampled loop (`StepSystem`, `Computer`,
`HybridDiagram`, `HybridSimulator`), so discrete control laws can close the loop on
continuous plants; it is provisional, and MPC is its exemplar in the tutorials (RULES 3.7).