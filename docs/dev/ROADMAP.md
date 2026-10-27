# Minilink Roadmap

Maturity and priorities, the **plan of record**; the steps themselves are the workboard,
[docs/plans/TODO.md](../plans/TODO.md), and the doc map is [AGENTS.md](../../AGENTS.md).

## 1. Releases

Identity: [CONSTITUTION.md](CONSTITUTION.md).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end**, plus a working **`pip install minilink` on PyPI**. Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via native `ReinforcementLearningPlanner` (with `Sys2Gym` + SB3 as an optional bridge). See §4.1. | Fall 2026 — `0.1.0` on PyPI (2026-09-16); `0.1.1` (2026-09-26) is the first release a GitHub tag published. Conda from `environment.yml` is the recommended install. The course notebooks' names are frozen until v1.0, with the freeze paused between terms (§4.1 gate 7). |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2). Conda stays the recommended install. | `0.2.0` on PyPI (2026-10-17), from the GitHub tag; every §4.2 row green |
| **v0.3** | **The complete core** (§5.3; the maintainer's direction of 2026-10-26, [core-architecture.md](../plans/core-architecture.md)): one architecture from the plant to the evaluation and the optimizer, supporting stochastic problems and the hybrid sampled loop, built in the natural order of the code. Landed first (2026-10-24, DESIGN §4–§6): **the convergence pass** (the core kept, side features to the experimental tier) and **the continuous core** (one step model under every sampled tool, one evaluation of any run, and the deterministic differentiable path to the cost behind a clean API: `evaluate`, `compare`, `problem.compile(loop)`, the program export, cost parameters; a closing PID-tuning example by hand and by the pipeline, S77). Then two foundations (every object pure, every number a parameter; the step algebra first-class) and seven pillars (the eight items the maintainer named, the parametric and the stochastic programs grouped as one): one discrete judge, the draws over starts, parameters, disturbances and block seeds, the seeded stochastic policy, the hybrid loop (one model, two runners), one generic parametric program with the stochastic program its case, a native solver and an MPC on it as a proof of concept, Fields. The teaching surface covers every GMC714 lab (§4.3, R48); the course material stays on `main` until v0.3 merges. | When the core is complete (no date; the order is the commitment) |
| **v0.4** | **Features on the core** (§5.4), not intertwined with it: MPPI on the sampled kernel; identification as the program export's first data consumer; `LQGPlanner`; workspace geometry; the RL follow-ups; native solvers beyond the proof of concept; the general serial chain (C6); pyro parity and the former wave G rows; traceable linear analysis. | After v0.3 (no date) |
| **v0.9** | **Freeze candidate** (§5.5): the foundation questions that could still break a public name are decided (derived `x0`, Diffrax as an optional solve, the glyph/solid rename, the frozen surface); the trust contract is in place (deprecation policy, pinned export list, CHANGELOG, pinned installs); the standard-syllabus holes close (estimation; the discrete-time one closes with S75 in v0.3); the docs site renders the tutorials. | Spring 2027 (indicative, set before 2026-10-26; the releases carry no dates now) |
| **v1.0** | **A colleague can build a course on it and trust it for three years** (§5.6): the API frozen under the deprecation policy, the standard control syllabus covered and checked against textbook worked examples, course-neutral labs an instructor adopts outside UdeS, green on Windows, macOS and Linux; after two cohorts. Two Stable-Baselines3 teaching notebooks retire (S49) and the GRO860 name freeze becomes the general one. | 2027 (indicative, set before 2026-10-26; v1.0 comes when the API feels mature) |

## 2. Two lanes

Philosophy: [CONSTITUTION.md](CONSTITUTION.md) §3 (the student syntax never
breaks). Placement shorthand: [RULES.md](RULES.md) §3.3. The operating
contract is the table below — not a pointer.

Minilink serves two audiences with one codebase. The boundary between them is
a **contract**, not a documentation convention.

| Lane | What it is | Rules |
| --- | --- | --- |
| **Teaching surface** | The names students meet: root prelude (`from minilink import …`) and the band facades (`minilink.catalog`, `.blocks`, `.control`, `.estimation`, `.analysis`, `.simulation`, `.planning`, `.learning`, `.optimization`, `.dynamics`, `.core`). Registered in one place, tested as a set. | *Soft rule:* nothing enters without a demo or notebook, a both-backends test where it defines dynamics, and a docstring. Names and semantics change only with a deprecation note. Student-facing examples and notebooks import **only** through it (CI-checked). |
| **Research lane** | Everything else: the provisional bands (hybrid, MPC, realtime), the experimental tier (`minilink/experimental/`: symbolic mechanics, the contact-engine leaf, C export, the unsupported plants once the catalog clean-up moves them; the top-level `experimental/` folder of prototype scripts), `examples/projects/`. | No stability promise, no entry requirements, out of the release contract. The provisional bands ship in the wheel. The experimental tier is **out of the wheel**, **untested by default** (nightly smoke at most; `c_export`'s flagship smoke and the quick C-export demo of `showcase_jax.ipynb` the two gate exceptions, ruled 2026-10-23), off the docs site, never cited by a course, and the staging area for what may leave into its own project at v1.0 (ruled 2026-10-22). Deep imports always remain valid. |

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
- 2026-10-24 (on `dev-v0.3`, pending the maintainer's merge; every lesson migrated in the same
  commits): the removals of R54 and R64 (`ProblemEnv` off the facade, `to_gymnasium`, the camera
  helpers, `Replay`, `Sampler`, `RateLimiter`, `Lag`, `DeadZone`, `Relay`, `NotchFilter`,
  `Washout`, `SimpleIntegrator`; `ZOHHold` is `UnitDelay`, `NeuralNetwork` folded into `MLP`);
  the package moves of R66 (`planning/dynamic_programming/`, `minilink.learning`, MPC under
  trajectory optimization); A6's short names and every block's default id its class name
  (R52, R53); the step model: the evaluator's integrator grid is `ev.step` / `ev.rollout`
  (`solver=`, `substeps=`, `hold=`), `integrator=` is `solver=` everywhere; every evaluator
  method one body with `params=None`, compiled once on JAX (`f_p`, `*_trace`, `*_jit`, the
  `_ivp` and SciPy adapters and `ev.jit` gone, R12),
  `discretize(sys, dt, *, solver, substeps)` with `dt` required, and `compute_forced` on
  `rk4_fixedsteps` holds a sampled input (those forced numbers changed once); the problem: a
  plain class whose `params` is a live view of its parts (`ProblemParameters` and `params=`
  gone), `U` unconstrained unless declared (`U=plant.input_box`), `x_goal` never creates `Xf`,
  `discount_rate` on the problem (gone from the cost), `infeasible_cost` the switch
  `None` / `True` / a number with `cost_bound()`, value iteration's default price the cost
  bound instead of `1e6` (its numbers changed once); the costs: parameters on the System
  pattern, a sum's terms keyed by id, `evaluate_trajectory` / `terminal_cost` / `total_cost` off
  the cost (`traj.with_cost`, `evaluate_trajectory(problem, traj)`); the evaluation:
  `MonteCarloEvaluator`, `score_trajectory` and `Comparison.evaluate` gone for
  `problem.compile(controller, dt)`, `evaluate` and `compare(problem, ...)` (the continuous loop
  integrated to `tf`, so the scores changed once); the optimizer: presets `gd`, `adam`,
  `l-bfgs`, `method="auto"`, `z0` optional; `MechanicalSystem` and `GeneralizedMechanicalSystem`
  invent no bounds (each catalog plant states its own, the same numbers), so a user's own
  mechanical subclass starts unbounded (R68).

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
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested; wrong-shape `f` / `h` fails at `compile()`; the modules by job (the operators, the closed loop, the roles, the port graph; the discrete-time pieces in `core/discrete/`, the `System` shortcuts in `core/facades/`, 2026-10-24). | Automation by convention, the rest (P10: plots and animation by role; the cost on the plant's draws, the v0.3 core); the three owners of a start (S29, R124, the v0.3 core). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; the frozen evaluator subset is named in DESIGN §5; one evaluator class per system kind on either backend, one body per method compiled on JAX (SM-6, 2026-10-24); the step model (`ev.step` / `ev.rollout`, `solver=`, `substeps=`, `hold=`) under every sampled tool, `discretize` on it (S75, 2026-10-24). Speed lives in batches (1000 rollouts × 1000 RK4 steps in 27 ms); float64 on JAX by default. | `rollout_batch` with a `params` family (S53); the step algebra first-class, one score, the draws and the sampled face (the v0.3 core, [core-architecture.md](../plans/core-architecture.md) §2, §3, §5, §6; the deterministic path landed); Diffrax (S27, v0.9). |
| Simulation | teaching | 7 | Mature; fixed 10 001-point default grid; `verbose` names unified. | One owner per rule (S65). |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA; **every catalog plant compiles on both backends** (contract test); four-rung vehicle ladder; UdeS racecar (kinematic, dynamic, 3-D); every plant states its own bounds, the mechanical bases invent none (R68, 2026-10-24). | The catalog clean-up (K8, v0.3); one mechanical family (S32, v0.3, R62; postponed by the maintainer on three open questions, two since ruled (R101, R102), `DynamicBicycle`'s base left, TODO §2); catalog hygiene (S64) and the general serial chain (C6), v0.4. |
| Control | teaching | 6 | Linear, LQR (infinite horizon, finite horizon, along a trajectory), pole placement, reference scaling (`N`), `P` / `PI` / `PD` / `PID` and the rate-measured `Impedance` / `IntegralImpedance` as one family (what is measured decides the derivative; limits with infinite defaults, anti-windup once a limit is declared; the shared pieces three pure functions), model-based SMC and computed torque, robotic impedance and kinematic laws, neural policy block; a controller is a plain `System`; MPC's teaching name `ModelPredictiveController` on `minilink.control` (G3). | The robotic PID wrappers (C2, v0.4). |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with grid-free margins, pole-zero, root locus, Nyquist, the linear step response on matplotlib and plotly; the sensitivity functions `S` / `T` / `PS` / `CS`. Every verb takes any `System`, the linear model as is, the rest linearized at the operating point (2026-10-15). | Nichols and overlays later; Bode, poles and margins traceable for sensitivity to params (idea to review, S73). |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources (with `Sine`), TF, `MLP`, `UnitDelay`. | `Ramp` / `Chirp` / `Delay` / `Switch` (C3, v0.4). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration (`loop` / `numpy` / `jax`), lookup controller, `PolicyEvaluator`, `LQRPlanner`; every planner returns a `PlanningSolution`; `compare()` reads solutions side by side, and `compare(problem, …, dt=)` scores them on the problem; an inadmissible pair priced by the problem's cost bound; `dp.py` is the textbook reference of the style. | cost-to-go as a `Field` (A1) and DP on the one score (the v0.3 core); policy iteration (S51); DP reading a finite horizon (S63), v0.4. |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot; `success` means defects satisfied to `feasibility_tol`; float64 by default on JAX. | Harden SciPy/Ipopt before TRL 6. |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt; the method table (`gd`, `adam`, `l-bfgs`, `method="auto"`) with a first-order backend, `z0` optional (S74, 2026-10-24). | Harden SciPy/Ipopt before TRL 6; the generic parametric program on the backend table, the stochastic program its case (S76, OPW, the v0.3 core, [core-architecture.md](../plans/core-architecture.md) §7); a native fixed-step solver as a proof of concept (the v0.3 core, §8), native solvers beyond it in v0.4 ([native-optimizers.md](../plans/native-optimizers.md)). |
| Interfaces / RL bridge | research lane (bridge) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call. | Keep for external interop; courses solve with the native planner. |
| Analysis / Lyapunov certificates | provisional (research) | 4 | `region_of_attraction` → `LyapunovCertificate` (quadratic `V`, sampled level, `verify`, `plot`, `contains`); demo and showcase §11. | Cohort validation; `V` as a `QuadraticField` (A1, the v0.3 core); SOS and discrete time later. |
| Planning / RL planner | teaching (GRO860) | 6 | `ReinforcementLearningPlanner` (REINFORCE, actor-critic, PPO, SAC in pure JAX), `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control), `StochasticPlanningProblem`, `evaluate` and its `Evaluation`; on the root prelude; canonical demos in `examples/demos/rl/`, chapter 11 and the native teaching notebooks. | SAC actor step (S50); deep Q-learning (S52); automatic reward scaling ([rl-reward-scaling.md](../plans/rl-reward-scaling.md)). |
| Planning / evaluation (the path to the cost) | provisional until a course runs it (R28) | 4 | `problem.compile(controller, dt)` → the problem evaluator; `evaluate` (one run or `n_trials` draws of the continuous loop, integrated to `tf`, each trial realizing the loop's random blocks, R92) and `compare(problem, …)` its facades, `evaluate_trajectory` for a held run; `jax.grad` reaches every gain, plant parameter, cost weight and start; `pe.program(wrt=)` to the `Optimizer`; the closing PID example (S77). On `dev-v0.3`, pending the maintainer's review (R79). | The `disturbances=` port signals, cross-backend start/parameter draws and the sampled face (V1, the v0.3 core); identification's data cost (C4, v0.4). |
| Planning / search (RRT) | teaching (G3, R49) | 5 | RRT / RRT*; `RRTPlanner(problem)` works from the problem's input box alone (bang-bang extender; an unbounded `U` refused with the line to write); returns a `PlanningSolution`. | RRT-Connect later. |
| Planning / path integral (MPPI) | research lane (planned) | 1 | Two standalone scripts (`experimental/mppi/`): the pendulum swings up; the kinematic racecar laps the cone circuit at a lower closed-loop cost than the collocation MPC. Design: [mppi.md](../plans/mppi.md). | The planner (S72, v0.4) on the sampled kernel, after RN-4 / RN-5 (both the v0.3 core). |
| Geometry / spatial | provisional (teaching) | 4 | SDF + `Scene` / fields / bodies under `planning/spatial/`, JAX twins tested; the planning facade's spatial names are provisional teaching names (§2). Home designed: `core/geometry/`. | Package move and course catalog (A3); bicubic SDF and CBF filter (research). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers; auto-fit camera. | Camera and plot defaults from roles (P10); constructor-derived camera hints (S43); glyph/solid rename (S30, v0.9). |
| Hybrid / step / MPC | provisional (research); MPC's teaching name `ModelPredictiveController` (G3, R49) | 4 | `StepSystem`, `Computer`, `HybridDiagram`, `HybridSimulator` (in `core/discrete/` and `simulation/discrete/`, 2026-10-24), MPC with parametric JAX under `planning/trajectory_optimization/mpc/`. A side helper for MPC simulation; the sampled loop is the one thing that is not a `System`. | The v0.3 core's hybrid-loop pillar, one model and two runners ([core-architecture.md](../plans/core-architecture.md) §5): keeps in step with the core rules (P10); no new hybrid features before S31 (ruled a tool on one tick-scan kernel, R13–R14; its code after the continuous core); the compiled MPC program binds every problem parameter, not only `x0` (S76, the v0.3 core; the key `plant` ruled, R26; the program's name open, QUESTIONS); a fixed-iteration MPC tick, compiled and differentiable with its closed loop on S31's JAX face, as a proof of concept in the v0.3 core (MPPI's tick a v0.4 feature; [native-optimizers.md](../plans/native-optimizers.md) §4.6); the tentative draft vision of `computer @ plant` as a step diagram on a base clock, with the MPC's computational delay ruled (R123; [sampled-path.md](../plans/sampled-path.md) §8.1–§8.5, not decided). |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review (v1.0); later the human-demonstration recorder of the behavior-cloning pipeline (after v1.0). |
| Estimation | teaching (GRO501) | 6 | `StateObserver` (innovation form, operating point on the block, hat labels), `luenberger` and `kalman` on arrays, a linear model or the plant (the duals of `place` and `lqr`, 2026-10-14); `lqg` returns the observer + gain box; an observer is refused as a controller; `closed_loop(gain, plant, observer=obs)` wires the LQG loop by role (AC-4, 2026-10-14). | the trajectory observer and the finite-horizon filter, the EKF, the estimation problem and its tools (v0.9, [the plan](../plans/estimation-and-state-feedback.md)); the sampled filters on the core's hybrid loop. |
| Identification | planned | 1 | No library module (the `identification/` stub deleted, R67; its home `minilink.learning` when C4 lands, R66); the `params_gradient` demo fits plant parameters by a hand gradient loop. | V1's program export with a data cost (C4, v0.4). |
| C export (`experimental/c_export`) | research | 1 | JAX→C transpiler; two demos; flagship smoke in the JAX regression job. | Keep isolated; repo-only. |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Not on the teaching path nor the API site. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later. |
| Pyro 2.0 overall | v0.4 (C1, R48) | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](../plans/pyro-port-remaining.md). |

## 4. Course objectives

Three courses drive the release contract: GRO860 (§4.1, v0.1, running now),
GRO501 (§4.2, v0.2) and GMC714 (§4.3, v0.3). A course is
"end to end" when every topic row is green on the teaching surface — in Colab
(git-clone cell) and in the conda env — and the cross-cutting gates hold.

### 4.1 v0.1 — GRO860 end to end

| Topic | Surface | Material | Gate |
| --- | --- | --- | --- |
| Value iteration / DP | `PlanningProblem`, `StateSpaceGrid`, `DynamicProgrammingPlanner`, `LookupPolicy`, `plot_cost2go` / `plot_policy` | `pendulum_value_iteration`, `pendulum_value_iteration_vs_lqr`, `demos/value_iteration/` | `final_time` reads `problem.tf`; `success` reports convergence; notebooks wire with `vi_ctl @ plant` |
| LQR + linearization | `linearize`, `lqr` (on arrays, a linear model or the plant), `plot_control_law` | `03_control`, `04_analysis`, `demos/control/` | — (green today) |
| Trajectory optimization | `PlanningProblem`, `TrajectoryOptimizationPlanner` (`direct_collocation`, `shooting`), `QuadraticCost` | `09_planning`, `demos/trajopt/` | float64 by default on JAX; `success` means defects satisfied; canonical problems succeed with default optimizer |
| Reinforcement Learning (tabular on the grid, neural in native JAX) | `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control; `EpsilonGreedy`, `UCB`), `StochasticPlanningProblem`, `ReinforcementLearningPlanner` (REINFORCE / actor-critic / PPO / SAC), `NeuralPolicyController`, `evaluate` | `11_reinforcement_learning.ipynb`, `pendulum_value_iteration_vs_lqr_vs_rl.ipynb`, `drone_ppo.ipynb`, `gymnasium_interface.ipynb`, `showcase_from_rl_to_bode.ipynb`, `demos/rl/` (pendulum, cart-pole, drone, rocket) | pure-JAX training on compiled plant (pendulum swing-up, 120k steps in 12.5 s on CPU, 0% failure over 50 trials); neural policy closed loop linearizes, simulates, and plots Bode; those names (plus `Evaluation`, `Gaussian` / `Uniform`) are on the root prelude. Notebooks solve with the native planner; Gymnasium stays taught as the domain standard and `Sys2Gym` + SB3 stay as the external bridge |

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
   tag); extras (`full`, `dev`) match `pyproject.toml`. Conda remains
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

The baseline audit (2026-09-07) and the step-level plan doc retired, their steps landed.

| Topic | Surface | Status | Gate |
| --- | --- | --- | --- |
| Multi-physics modelling — nonlinear `f`/`h`, block diagram, linearize, `H(s)` | custom `DynamicSystem`, `plot_diagram`, `linearize`, `transfer_function` | green | a longitudinal-vehicle plant in the catalog (met: `LongitudinalTorqueCar` on `minilink.catalog`; a DC-motor plant is off the critical path, S71); pole/zero cancellation by default; order reduction by fast-mode neglect is out of v0.2 |
| Closed-loop analysis — poles, root locus, Bode, margins, step specs | `pzmap`, `root_locus`, `bode`, `margins`, `step_info`, `P` / `PI` / `PD` / `PID` | green | every compensator form reports the poles and zeros of the hand calculation, and margins are found wherever the crossover sits |
| Design to specification — rise time, overshoot, final error, phase margin | `PI`, `PD`, `PID`, `Lead`, `step_info`, `margins` (`Lag` removed, R54) | green | the Table 2 specs of the guide are checkable in one notebook |
| Loop-shaping specs — disturbance and measurement-noise sensitivity in dB at a frequency | `sensitivity`, `complementary_sensitivity`, `load_sensitivity`, `noise_sensitivity` | green (2026-10-10) | the `sensitivity_functions` teaching notebook |
| Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models) | green (2026-10-17) | the law is designed in continuous time, discretized, and the sampled loop validated by simulation; no z-domain analysis family (ruled 2026-10-17, [the record](../reviews/2026-10-17-z-tier-dropped.md)) |
| State-space MIMO — bicycle model, controllability at every nominal speed | `KinematicBicycle`, `controllability`, `observability` | green | — |
| Optimal control — LQR on the guide's cost, closed-loop poles, nonlinear check | `lqr(plant, Q, R, x_bar=)`, `StateFeedback` | green | — |
| Pole placement — `K_sta` for a prescribed pole set | `place` (on a linear model or the plant; `place_gain` on arrays) | green | — |
| Nested loops — inner speed loop, outer position loop | `@` composition | green for error-driven compensators (`PID @ (PID @ G)`); an outer `r, y` or state controller is refused today | P10 ([automation-by-convention.md](../plans/automation-by-convention.md)) nests the inner loop as the outer loop's `plant`; stays green with the observer in the loop |
| State estimation — Luenberger observer and Kalman filter | `StateObserver`, `luenberger`, `kalman`, `lqg` | green (2026-10-14) | the observer closes the loop as a standard diagram block: by role with `closed_loop(gain, plant, observer=obs)`, in the `lqg` box, or by hand |
| Reference scaling — the `N` matrix giving `y = r` at steady state | `OutputReference(K, N)` | green (2026-10-10) | the student writes `N = -(C (A - B K)⁻¹ B)⁻¹`; the block takes the output reference |

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
runs.

### 4.3 v0.3 — GMC714 end to end

The robotics course (*Robotique : modélisation et commande*), winter term, week 1 on
January 9. Syllabus: [the course page](https://www.alexandregirard.com/courses/gmc714/) and its
notes (*Notes_Robotique_UdeS*), read 2026-10-23. Its labs run today on pyro, through Colab
notebooks one per homework. **Ruled 2026-10-23 (R48): GMC714 needs no new feature.** Its labs
move from pyro to minilink as the maintainer's teaching work, outside this roadmap; the
release's part is that the teaching surface covers every lab, checked the same day:

Names as ruled by R52 and R63 (the renames landed with A6 on 2026-10-24, on `dev-v0.3`).

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

Not a week of the syllabus: MPC (its teaching name stable all the same, R49: when v0.3 merges,
by January as planned before 2026-10-26), robust control as a topic of its own
(one section of week 5, the sliding mode), the draws and Monte Carlo, gradient tuning.

**The gate:** every row above stays on the teaching surface through the v0.3 changes, so a lab
written against the v0.3 interface needs nothing more; and (R49) the tools the labs use carry
one public, stable teaching name when v0.3 merges (RRT, MPC, trajectory optimization, computed
torque, sliding mode, the task- and joint-space impedance laws, task-space position control
with a nullspace posture, `Manipulator`), frozen for the term (TODO §2, G3). The lab notebooks themselves are the
maintainer's teaching design (AGENTS), written when he chooses; agents draft on request.
**Reworded 2026-10-26** (the maintainer's direction): the course material stays on `main` until
the new v0.3 core is ready; the gate read "by January" before.

## 5. The path to v1.0

**How the rungs work:**
- Each rung names its open steps, one line each. The steps themselves, with files, order and
  "done when", are the rows of [docs/plans/TODO.md](../plans/TODO.md).
- A rung is done when every step in it has landed or the maintainer has dropped it.
- Landed work leaves this section; its history is in git and in [docs/reviews/](../reviews/).
- Wave D's rows (the T rows, D1–D3, S43, S67–S69, A4, P4) are named in TODO only, beyond §5.2's
  summary.

**The order is the commitment; the course calendar is secondary** (ruled 2026-10-22,
[record](../reviews/2026-10-22-second-roadmap-review-rulings.md)). **From 2026-10-26 no course
calendar shapes a step** (the maintainer's direction,
[record](../reviews/2026-10-26-core-consolidation.md)).
- The class material stays on `main` until the new v0.3 core is ready, and the releases carry
  no dates.
- GRO860 runs in the fall term and GMC714 in the winter term.
- Two windows the plan cites:
  - *between terms*, late December 2026 to early January 2027 (V11 and the examples moves);
  - *between cohorts*, between GRO860 cohorts, whose material reads the Monte Carlo evaluator
    and the RL numbers.
- Neither window delays a step: a step that would move a running course's numbers ships behind
  the pinned release the course runs on (V6) rather than waiting.

**The shape the rungs build.** The spine is CONSTITUTION §1.1 (R119): a `System`, compiled,
rolled out to a `Trajectory`, scored by a cost, lowered to a `MathematicalProgram`, solved by the
`Optimizer`. There are two step paths on it:
- the continuous one (`Simulator`, the differentiable evaluator);
- the sampled one (the step model, DESIGN §5, and the sampled loop,
  [sampled-path.md](../plans/sampled-path.md)).

The toolbox chapters on the spine are:
- classical and modern control;
- nonlinear control;
- trajectory optimization and MPC;
- reinforcement learning;
- identification and imitation;
- estimation.

The releases: **v0.3 builds the complete core** ([core-architecture.md](../plans/core-architecture.md)),
**v0.4 adds the features built on it**, v0.9 decides what can still break a name, and v1.0
freezes when the API is mature.

| Rung | What | Rows |
| --- | --- | --- |
| v0.3 | The landed continuous core; then two invariants and seven core pillars in three lanes (below); the clean-up lane beside | TODO §1, §2 |
| v0.4 | Features that read the core and never change it | TODO §4 |
| v0.9 | What could still break a name; the trust contract; the syllabus holes; the reference site | TODO §5 |
| v1.0 | A colleague can build a course on it | TODO §5 |

### 5.1 v0.1 — close-out

The GRO860 path is green (§4.1) and `0.1.1` is published; nothing is open.

### 5.2 v0.2 — shipped; the carry-over

`0.2.0` was tagged on 2026-10-17 with every §4.2 row green. The steps below were planned for
v0.2 and not needed for the tag; they carry into v0.3 and v0.4, each in the slot the order of
TODO §1 to §4 gives it, wave D standing. On 2026-10-26 the core's rows among them (A5, RN-2,
AC-7, AC-8, S65, S66) joined v0.3's core (TODO §1).

**Wave A — the textbook objects finish speaking.**
- **A5** The later nouns, each with its first consumer: `log_prob`, sets and distributions
  over parameter dictionaries, `UnionSet`, `PlanningProblem.hamiltonian()` only if a course
  teaches Pontryagin (the library sets reading `params` and `Set.as_cost`, the v0.3 core).
- **RN-2** of [randomness.md](../plans/randomness.md): distributions read `params`,
  `Gaussian(cov=)`, every `sample` takes a key (the v0.3 core).

**Wave B — GRO501 end to end** (§4.2; the plan doc retired 2026-10-17, its steps landed).
- **P10** Automation by convention ([automation-by-convention.md](../plans/automation-by-convention.md),
  decided 2026-10-11): standard port names drive the wiring, three reserved names (`plant`,
  `controller`, `observer`) are the roles every tool reads, and the cost is scored on the
  plant. Its composition steps landed in this wave (2026-10-13/14); the rest: plots and
  animation by role (the plotting lane), the cost's draws (AC-7) and the sampled loop (AC-8),
  the v0.3 core; TODO §1 and §3, the plan's §7.

**Wave D — the code reads like the textbook, and the demos like the API.** Standing work,
behaviour-preserving, one module per step.
- The textbook pass landed 2026-10-16/17 ([rulings](../reviews/2026-10-17-textbook-pass-follow-up-rulings.md)); its behaviour and API
  leftovers are TODO §3's T rows (T1 and T2 in the core, TODO §1.1).
- **D1** Demos and notebooks to the minimal rule (RULES 6.1 / 6.10 / 6.11). **[ask per
  file]**
- **D2** Consolidation picks still open from the 2026-09-05 inventory.
- **D3** Hardening rows carried over (research lane).
- **S65** One owner per rule (the automatic `dt`, the backend fallback, the input-hold
  model, the Monte Carlo defaults), the core; **S66** wiring mistakes fail at wiring time
  **[ask — core]**, the core's hardening; **S67** one plot vocabulary; **S68** contracts as tests.

### 5.3 v0.3 — the complete core

**The direction (2026-10-26, [record](../reviews/2026-10-26-core-consolidation.md)).** v0.3 is
the complete core: one architecture from the plant to the evaluation and the optimizer,
supporting stochastic problems and the hybrid sampled loop, built in the natural order of the
code, with no date. It supersedes R86's "v0.3 stays continuous". Where the pieces live:
- **The design home:** [core-architecture.md](../plans/core-architecture.md).
- **The rows:** TODO §1.
- **The open decisions:** QUESTIONS.md, sorted by what each gates: the merge into `dev`, the
  critical path, later core work, the clean-up lane.
- **The history of how v0.3 got here:** [the simplification record](../reviews/2026-10-26-plan-simplification.md)
  §4.

**Landed, pending the maintainer's review (R79)** (2026-10-24/25, DESIGN §4–§6):
- the step model (S75);
- the path to the cost (V1's steps 1–6 and 7a, with the cost parameters A2 and the Monte Carlo
  fold);
- the optimizer's method table (S74);
- the closing PID example (S77).

**The workstreams** (each **[ask — core]**). One number per workstream, the same in
core-architecture §N, TODO §1.N and the QUESTIONS tags:

| § | Workstream | The main rows |
| --- | --- | --- |
| 1 | Every object pure, every number a parameter (invariant) | the three owners of a start (S29, R124), A5, RN-2, the sets as costs (was V1 step 8), one parameter-tree module, T1, T2 |
| 2 | The step algebra first-class (invariant) | step evaluators trace and batch, the `StepRollout` fold (V1.1's rest), S53, S65 |
| 3 | The draws | per-block seeds and disturbance signals (V1 step 7, RN-4, RN-5, AC-7), RN-3, `Sys2Gym`'s start law |
| 4 | The seeded stochastic policy | the seeded block (D29) |
| 5 | The hybrid loop: one model, two runners | the sampled loop (S31, AC-8b, D13), the sampled face (was V1 step 7b), the `HybridSimulator` drift |
| 6 | One score | the discrete problem and one score (a draft) |
| 7 | The parametric program; the stochastic program its seed case | S76 (PP-1–PP-5), OPW-1–OPW-5, the stochastic program |
| 8 | The native proof of concept | NS-1, NS-2's fixed-`K` form, NS-8, NS-9 |
| 9 | Fields | A1 |
| 10 | Hardening and the merge | S66, material and contracts (was V1 step 9), the notebooks (R47), the merge (R79), the tag |

**The order, a proposal** ([core-architecture.md](../plans/core-architecture.md) §10):
- **Three lanes:**
  - objects and draws: §1, §3, §4;
  - the loop, the critical path, which starts first: §2, §5;
  - the program: §7, §8's kernel.
- **Two joins:**
  - one score (§6);
  - the MPC's closed loop in one scan (§8).
- **Beside, from §1 on:** Fields (§9), additive to the path.
- **Then last:** §10, hardening and the merge.

The definition of done is proposed as acceptance scripts (core-architecture §11; QUESTIONS).

**The clean-up lane, beside the core** (TODO §2):
- **K4: one demo per feature.** It runs before D1.3 sweeps the rest, and it is the maintainer's
  own pass (R54).
- **K8: the catalog clean-up** (R2):
  - the root prelude trimmed to the course set;
  - `minilink.catalog` the flat index of tested plants;
  - the unsupported plants moved to the experimental tier.
- **K9: the experimental tier out of the CI gate** (R3). Its last items, the unsupported plants'
  tests, leave with K8. Ruled.
- **G3: GMC714's names stable** (R49):
  - `RRTPlanner` to the teaching lane;
  - the robotic laws onto the prelude;
  - the textbook model forms on a new `minilink.dynamics` band facade;
  - MPC as `ModelPredictiveController`, with a frozen constructor (R50).

  The moves and renames landed 2026-10-24, and its gate is left. The January date is lifted
  (2026-10-26): the names are pinned when v0.3 merges.
- **S32: one mechanical family** (R62). `GeneralizedMechanicalSystem` becomes a child of
  `MechanicalSystem` beside `Manipulator`, `H` is the one mass-matrix name, and the model-based
  controllers accept every mechanical plant.
  - **[ask — core]**, ruled; **[ask — public names]**.
  - Postponed by the maintainer (2026-10-24). `DynamicBicycle`'s base is still open.
- **What the landed rows left:**
  - the LQR warning when `U` is declared (V1.2's rest, OD-33);
  - the hand descent loops of three compile demos moved onto `Optimizer` (S74's rest, OD-32).

  Each is the maintainer's call **[ask — student-facing]**.

**GMC714** (R48): no new feature. The teaching surface already covers every lab (§4.3). The
audit was done 2026-10-23 (G1).

### 5.4 v0.4 — features on the core

**The maintainer's direction (2026-10-26):** v0.4 adds features built on the finished core, not
intertwined with it; any order, each independent. The rows are TODO §4; the map of which pillar
each reads is [core-architecture.md](../plans/core-architecture.md) §12.
- **S72** The path-integral planner (MPPI) on the core's sampled kernel and draws
  ([mppi.md](../plans/mppi.md)), with the MPC block's planner contract (D27, T6). **[ask — the
  MPC block contract]**
- **C4** Identification as the path's first data consumer: the program export with a
  data-mismatch cost; trajectory generation and experiment design beside it.
- **LQGPlanner** with RN-4 (the core).
- **A3** Workspace geometry, `core/geometry/` ([geometry-module.md](../plans/geometry-module.md)),
  `PurePursuit` on a `Track`. **[ask — core]** The CBF safety filter after it (unscheduled).
- **C5** RL follow-ups: **S50** SAC actor step, **S51** policy iteration on the grid,
  **S52** deep Q-learning, approximate dynamic programming; the RL heads rebuilt on the seeded
  policy (core or here: open, QUESTIONS.md).
- **Native solvers beyond the proof of concept** ([native-optimizers.md](../plans/native-optimizers.md)):
  the tolerance form, implicit differentiation, least squares, the augmented Lagrangian, the
  stochastic category, convergence as stability.
- **C6** General serial chain: one mechanism description (DH or URDF) feeding RNEA / ABA, the
  GMC714 arm topic's modelling rung; design first. **[ask — core]**
- The former wave G rows (R48): **C1** pyro parity and the pyro → minilink migration guide,
  **C2**'s robotic PID wrappers, **C3** the blocks (`Ramp` / `Chirp` / `Delay` / `Switch`),
  **S63** DP reading a finite horizon, **S64** catalog hygiene.
- **S73** Traceable linear analysis, any time, self-contained.

### 5.5 v0.9 — freeze candidate (spring 2027, indicative, set before 2026-10-26)

What v1.0 asks is adoption, not architecture: a colleague builds a course on minilink and
trusts it for three years. The
adoption review of 2026-09-26 found the
engine ready and three things missing around it: a stability promise, the two syllabus
holes every course hits (estimation, discrete time), and material packaged for an
instructor. The discrete-time hole closed with S75 in v0.3 (§5.3); v0.9 closes the stability
promise and the estimation hole, and settles every question that could still break a
public name; v1.0 (§5.6) freezes.

**The foundations that can break names** — decided before the freeze:
- **S27** Diffrax as an optional JAX solve, beside the step formulas S75 kept in
  `core/compile/` (OD-11; S37 is absorbed by S75). **[ask — core]**
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
  `ValueError` / `TypeError`. **[ask — core]** (Whether it lands before the core's S66 is open,
  QUESTIONS.md.)
- **V11** A light history: the notebook outputs committed before `nbstripout` leave the
  git history, once, after the v0.3 tag (planned for December 2026 when v0.3 had that date). **[ask]**

**The syllabus holes** (beyond GRO501's P4):
- **Estimation follow-ups** after P4: EKF, time-varying and discrete Kalman; online parameter
  estimators (recursive least squares, gradient laws).
- **C4** Identification: moved to v0.4 (V1's program export with a data cost).
- **S71** The classic first plants (DC motor, tank / thermal process, ball and beam,
  differential drive, 3-D quadrotor); LQI beside the `N` matrix.

**The reference site:**
- **V7** The docs site renders the tutorials, a plant gallery with pictures, and a short
  concept guide.

### 5.6 v1.0 — a colleague can build a course on it (2027, indicative, set before 2026-10-26)

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

The open decisions live in [QUESTIONS.md](QUESTIONS.md), one entry each, by the release each
blocks. A decision leaves it when it is taken: its ruling goes to the plan doc it shapes and to
a dated record in [docs/reviews/](../reviews/), and its contract to DESIGN when the code lands.

Past rulings: the dated records in [docs/reviews/](../reviews/), named by date and subject.

## 7. Out of scope

By decision — see [CONSTITUTION.md](CONSTITUTION.md) §1.4:

Full Simulink parity (GUI, DAE, arbitrary multi-clock scheduling,
event-driven switching as a framework feature); becoming a multibody/contact
OS or a batched RL physics engine.

What *is* in scope but subsidiary: the sampled loop (`StepSystem`, `Computer`,
`HybridDiagram`, `HybridSimulator`), so discrete control laws can close the loop on
continuous plants; it is provisional, and MPC is its exemplar in the tutorials (RULES 3.7).
