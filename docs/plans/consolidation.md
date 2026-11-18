# The consolidation: the inputs of the maintainer's pass over demos and features

**What it is:** everything the maintainer's pass needs in one place: the draft of the demo
set and the tutorials beside it (§4), what the kept surface runs of the library (§5), one
recommendation per area of the repository (§6, §7), the candidates behind the kept surface
(§8), and the test layers of the clean slate that follows (§9). The sums, a suggested order
and the open questions close it (§10 to §12). Its decision ids carry the `MS-` prefix
(docs/plans/README.md). The row and its "done when": [TODO.md](TODO.md) §1.12; the size
orientations: ROADMAP §5.4.

## 1. The goal and the rulings

**The goal:** a surface designed from the core outward, small enough that the maintainer
knows every demo and every test layer, and a CI that answers in minutes.

- **Nothing is deleted before the maintainer's pass.** He decides himself which demos and
  which features stay. This document lists candidates, recommendations and drafts, not
  decisions: "delete", "move" and "reduce" name what the maintainer may choose, not what an
  agent does. `examples/demos/` (94 scripts) and `examples/demos_proposed_minimal/` (9) both
  stay as they are until then.
- **The value-iteration animations stay** (`animate_cost_to_go`, `animate_control_law`): they
  are used in class, whatever the demo set. They are in no candidate row. The sweep history
  they replay (`record_history=True`, `record.history`) is not separately removable: the
  animations read it.
- **MS-D1, tests (2026-11-07):** at the end of v0.4 the maintainer rebuilds a minimalist
  demo set himself; once the new demos are what he wants, the test infrastructure is rebuilt
  from scratch around the new library (§9).
- **MS-D4, the demo set (2026-11-07):** the maintainer's, decided in that rebuild (§4 is the
  draft he reviews).
- **The notebooks (2026-11-13):** the course and topic notebooks live in the teaching
  repository ([repository-structure.md](repository-structure.md)); their content and their
  teaching are the maintainer's, and no row here changes what one does.

**The constraint:** the course PDFs link course notebooks by path, on minilink's `main`
(`Notes_OptiRL.pdf` 12, `Notes_SystemesAsservis.pdf` 4, checked 2026-10-30). The links move
with the merge of `dev` into `main` after the term (repository-structure.md §2); a notebook
on an active course's list is not renamed or moved during its term.

## 2. How to read the tables

### 2.1 How each area was judged (§6, §7)

Usage by the current demos is one input among nine, and never the deciding one.

| # | Criterion | What it asks |
| --- | --- | --- |
| 1 | **Course material** | Does a chapter, a lab or a homework of GRO860, GRO501 or GMC714 need it? If yes it stays, used by a demo today or not. |
| 2 | **Direction** | Is it a building block of where the library goes (one problem for every planner, the evaluation pipeline, differentiable everything, certificates, the vehicle ladder)? |
| 3 | **Coupling** | A leaf nobody imports costs almost nothing to keep. A gear many files import costs on every refactor. |
| 4 | **Noise for a reader** | Does it sit in the path of someone learning the library (the top-level names, a package's listing), or out of the way in a catalog? |
| 5 | **Textbook value** | Is it a readable reference implementation of a chapter of the notes (the way `dp.py` is)? |
| 6 | **Trust** | Is it exercised by anything, or is it code that claims to work and never runs? |
| 7 | **Dependency weight** | Does it pull an optional install (SB3, torch, MeshCat, pygame, Plotly, Ipopt) or CI time? |
| 8 | **Duplication** | Is it a second way to do what another part already does? |
| 9 | **Reversibility and timing** | A move after a snapshot tag is reversible; a rename during a term breaks course links. |

**Recommendations:** keep · clean (keep, tidy the code) · minimize the surface (keep the
feature, fewer public names and options) · reduce to core (keep one small feature, drop the rest) ·
move to the teaching repo (public, pinned to a release) · move to the research repo (private) ·
move to the scratchpad (personal, private) · delete.

**Priority:** 1 = do first (safe, large gain in clarity) · 2 = do with the API review ·
3 = when the file is next touched, or leave.
**Confidence:** H = I would do it without asking · M = sound but your call matters ·
L = I miss information; the row says what.

**Measured columns.** Lines: `wc -l` on the files named, blank lines included, rounded
(re-counted 2026-11-18, after the plotting contract; the library is 81 600 lines, 65 900 of
them not blank). Use: share of statements run by the kept surface (the showcase, the
tutorials, the nine proposed demos, the 25 course and topic notebooks, since moved to the
teaching repository; §5.1), with its blind spots (MeshCat, pygame, real
time, the SB3 notebooks did not run). The use column is the run of 2026-11-12 and was not
repeated: the plotting files and the region of attraction changed since. Imported by: library
files that import the module.

### 2.2 How the internals were measured (§8)

- **Every definition counted.** All 3 837 functions, methods and classes of the kept library
  were listed with where their name appears (library, kept examples, old demos and projects,
  tests) and whether the kept surface runs them (the coverage run of §5.1).
- **Three independent readers**, one per zone (core, compile and simulation; planning,
  optimization and learning; graphics, analysis and control), each told to verify a claim by
  search before reporting it. Their 66 findings were then checked here: about fifteen against
  the code, each of them holding with one correction; the rest are as reported.
- **"No caller"** means none in the library, the tutorials, the proposed demos or the course
  notebooks. A test does not count as a caller.
- **Line counts are estimates** from the readers, to the nearest ten or so, unless a file is
  named with its size (`wc -l`; "not blank" where said).
- **Re-checked 2026-11-18,** after the plotting contract and the cut of the region of
  attraction: every name in the tables exists, the callers are re-counted by search over
  `minilink/`, `examples/`, `tests/` and `benchmarks/`, and the rows whose subject is gone
  are removed. The readers' line estimates and the "written N times" counts of §8.1 and §8.5
  were not re-measured.

## 3. What the courses need

| Course | Topics (the site, the notes' table of contents, ROADMAP §4) | What it pins in the library |
| --- | --- | --- |
| **GRO860**, reinforcement learning and optimal control (fall) | cost and policy; dynamic programming on a grid; stochastic and minimax control; partial observation and estimation; Bellman equations, value iteration, policy evaluation; LQR and LQG; planning algorithms, receding horizon; Q-learning, SARSA, exploration; function approximation; policy gradient, actor-critic, PPO; Gymnasium and Stable-Baselines3 lab | `PlanningProblem`, the grid and `DynamicProgrammingPlanner`, `LookupPolicy`, the cost-to-go and control-law plots, `lqr`, `kalman`, `lqg`, the tabular learners and exploration rules, the linear approximators and features, REINFORCE, actor-critic, PPO, `NeuralPolicy`, `evaluate` and `compare`, MPC and MPPI, the Gymnasium interface, pendulum, cart-pole, double integrator, drone, quarter-car suspension, the grid world |
| **GRO501**, classical control (the racecar APP) | modelling and block diagrams, linearization, transfer functions; poles, root locus, Bode, margins, step specifications; PID, lead; sensitivity functions; digital implementation; state space, controllability, LQR, pole placement, nested loops, observer and Kalman filter, reference scaling | the whole of `analysis/` except the region of attraction, `control/siso.py`, `state.py`, `design.py`, `estimation/`, `blocks/`, `core/discrete/`, the diagram operators, `KinematicBicycle`, `LongitudinalTorqueCar` |
| **GMC714**, robotics: modelling and control (winter, from January 9) | phase plane; arm kinematics and dynamics; impedance control; computed torque, sliding mode, adaptive control; vehicle modelling with tire slip, the dynamic bicycle; trajectory optimization and RRT | the phase plane, the manipulators and `UR5Manipulator`, `Impedance`, `JointImpedance`, `TaskImpedance`, the task-space kinematic laws, `ComputedTorque`, `SlidingMode`, the vehicle ladder and tires, `TrajectoryOptimizationPlanner`, `RRTPlanner` |

Research on the site that points the direction: low-adhesion and off-road vehicle control,
dimensionless policies, closed-loop stability of learned policies, estimation for inspection
drones, impedance and force control of arms, sampling-based planning.

Three corrections this makes to the usage-only reading of §5.1: the
computed-torque and sliding-mode laws (0 % run) are GMC714 week 5; the tabular learners (27 %)
and the linear approximators (0 %) are the centre of GRO860; the region of attraction (0 %)
is the direction of the stability research.

## 4. The demo set (MS-D4) and the tutorials: a draft for the maintainer's review

A draft of 2026-11-12, written on the maintainer's go after he read the proposal; the list is
his, and each row is open until he rules it.

### 4.1 The problem: a demo per plant and per tool

The catalog holds many plants and the library many tools, so the demo folder tends to hold a
script per pair. Pyro did, twice over: a folder by system (the pendulum with value iteration,
with RRT, with MPC) and a folder by tool. Minilink's design removes the reason: any tool takes
any `System`, and one `PlanningProblem` goes to every planner under one judge, so a script
whose content is "tool X on plant Y" shows nothing a reader could not write in three lines.

The 94 scripts of `examples/demos/` as counted on 2026-11-12:

- 31 use the pendulum, each with another tool: one row of the table, in 31 files;
- six racecar scripts and six value-iteration scripts are variants of one another;
- 29 walk one tool (10 analysis, 7 wiring, 6 step systems, 4 plotting, 2 optimizer), as the
  18 tutorial notebooks already do, in the same order;
- seven say in their title that they port a Pyro script.

### 4.2 One home per need

| A reader's need | Its one home | Not |
| --- | --- | --- |
| How do I use tool X? | the tutorial notebooks, on the pendulum | a demo per tool |
| What does plant Y look like? | the plant's own module, run directly (RULES 5.18) | a demo per plant |
| What can minilink do? | the demos below, each a story | a demo per pair |

A row of the table is one script: one problem handed to every planner and compared in one
table. A demo is neither filed by system nor by tool: the set is small enough for one flat
folder, each script named by its plant and its task.

### 4.3 The thirteen

| # | Demo | The story | From |
| --- | --- | --- | --- |
| | **The library's thesis** | | |
| 1 | Pendulum, every planner | one swing-up problem; value iteration, LQR, trajectory optimization, MPPI and RRT; one comparison table | **new**; replaces `vi_pendulum_lqr`, `vi_pendulum_swingup`, `mppi_pendulum`, `rrt_pendulum_swingup`, `evaluate_controller` |
| 2 | Cart-pole LQG | a Kalman filter and LQR with noise on the ports, scored by Monte Carlo | merges `control/lqg_cartpole` and `compile/lqg_monte_carlo` |
| 3 | The differentiable double loop | a neural inner loop, an MPC around it, a gradient through both | `compile/differentiable_double_loop` |
| | **Flagship platforms** | | |
| 4 | Cart-pole swing-up | a planned trajectory, then LQR along it | `control/trajectory_lqr_cartpole` |
| 5 | Racecar MPC | model predictive control round a circuit | one of `mpc/mpc_car_minimal`, `udes_racecar/mpc_racecar_dynamic`, `mpc_racecar_kinematic` |
| 6 | Racecar path tracking | pure pursuit, Stanley and line of sight on one car and circuit | `udes_racecar/path_tracking_racecar_compare` |
| 7 | Two-link arm | task impedance at the end effector | `robotic/task_impedance_two_link` |
| 8 | Rocket landing | reinforcement learning on a stochastic problem | `rl/rocket_landing_rl` |
| 9 | Off-road car | the 3-D car over rolling hills, open loop | `dynamics/offroad_car_3d` |
| | **The rest of the surface** | | |
| 10 | Digital PID tuned by gradient | a law on its clock, the loop differentiated | `compile/digital_pid_tuning` |
| 11 | RRT* among obstacles | a scene, a tree rewiring live | `rrt/rrt_star_live` |
| 12 | Cart-pole game | the arrow keys, in real time, no hardware | `realtime/game_cartpole` |
| 13 | The README's examples | runnable, in the README's order | `core/readme_examples` |

Open in the list, each the maintainer's:

- **Demo 1's size.** Five planners in one script is the best showing of "one problem, one
  judge" and the hardest script to keep short. The bar proposed: about 80 lines and a
  minute's run. Reinforcement learning stays out of it (training takes minutes) and keeps
  demo 8; past the bar the script splits in two.
- **Demo 5's model:** the kinematic racecar (shortest) or the dynamic one (the flagship).
- **Demo 8's plant:** the rocket, or the drone or the cart-pole (its budget was an open row).
- **Demo 9 runs outside the 3-D car's stated validity** (hills of 0.8 m; the model is good on
  mostly flat ground, as `SuspensionCar3D`'s docstring says): a flat course with the formula preset is the alternative.
- **The folder and the file names:** student-facing, not proposed here.
- A tabular Q-learning demo beside value iteration and an MPC hyperparameter-tuning demo are
  not in the thirteen; they return only as a story.

### 4.4 Where the other scripts go

| Today | Scripts | Goes to |
| --- | --- | --- |
| `analysis/` | 10 | the tutorials (analysis and control); none kept |
| `core/`, `blocks/` | 8 | the tutorials (systems); `readme_examples` kept (13) |
| `hybrid/` | 6 | the tutorial on discrete and sampled systems; none kept |
| `graphical/`, `optimization/` | 6 | the tutorials; none kept |
| `compile/` | 14 | 3 and 10 kept; `lqg_monte_carlo` into 2; the rest to the differentiable tutorial or the private repository |
| `control/` | 9 | 4 kept; `lqg_cartpole` into 2; the rest to the tutorials, the two Pyro ports out |
| `value_iteration/` | 6 | into 1; the 4-D cases to the private repository |
| `mpc/`, `udes_racecar/` | 11 | 5 and 6 kept; `mppi_pendulum` into 1; the rest out |
| `trajopt/`, `rrt/` | 5 | 11 kept; `rrt_pendulum_swingup` into 1; the corridor scripts out |
| `rl/` | 4 | 8 kept; the rest out |
| `robotic/` | 6 | 7 kept; the five others out (four Pyro ports, the custom one-link arm to the tutorials) |
| `dynamics/` | 4 | 9 kept; Lorenz and the three bodies run from their own modules |
| `realtime/` | 4 | 12 kept; the gamepad games to the private repository or kept as extras |
| `evaluation/` | 1 | into 1 |

"Out" is the maintainer's private repository, after the snapshot tag and after his pass:
nothing is deleted without a copy, and nothing before he rules it. Before a tool's walk-through leaves the demos, its tutorial section is
checked to exist; the tutorials are reduced after the demos, not with them.

### 4.5 The tutorials beside the demos

The 18 tutorial notebooks (code lines counted 2026-11-18), and what the maintainer ruled on
2026-11-12:

| Layer | Notebooks | Code lines | What it is | Ruling |
| --- | --- | --- | --- | --- |
| `basics/` | 3 | 201 | story-led: a model, a loop, analysis, a plan | kept |
| `advanced/` | 5 | 480 | one topic each | kept |
| `core/` | 9 | 806 | the core objects listed method by method, written for the maintainer's API review; the ninth is the plots and animations | **kept until that review is done,** then they go: a stand-in for the documentation of the core classes, not teaching material |
| the showcase | 1 | 176 | the landing page | **one showcase** |

- **The one showcase** is `showcase_minilink.ipynb`. A first rewrite (2026-11-12) opened on
  a 40-line hand-written pendulum class; the maintainer refused it the same day: a showcase
  shows how little it takes to start, and writing a class cleanly is a tutorial's job. The
  second rewrite mirrors the README, section for section, with code that runs: ten lines on
  the catalog pendulum; diagrams as code, the loop then on a clock; a short model of one's
  own; one `f` differentiated (Jacobians, LQR, a batch of rollouts); one problem for three
  planners and an MPC; the one optimization boundary (a program, two optimizers); a law
  learned by reinforcement and scored; the same lines on the UR5. 40 cells, 176 code
  lines, about a minute. The pendulum family's 2-D look was redrawn for it (a dark rod, a
  blue bob, a hub on a grey mount).
- **`showcase_jax` and `showcase_from_rl_to_bode` are gone** (the maintainer,
  2026-11-12). The first is covered by `advanced/05_differentiable`; what it showed that no
  tutorial does (identification by gradient, the C export) is to be covered in the API
  walk-through or not at all. The second was first kept as a demo script, then deleted too
  (the maintainer, 2026-11-12): it was a quick test that a Bode plot can be drawn through a
  neural law, tools plugged together to see whether they connect, not a story worth showing.
- **The tutorials change the demo list of §4.3.** Three of the thirteen are tutorial
  sections already: demo 1 is `advanced/02_optimal_control` (four planners, one comparison),
  demo 3 the last section of `advanced/05_differentiable`, demo 10 is covered between
  `advanced/03` and `advanced/05`. The split proposed: the tutorials carry what fits in a
  notebook, the demos only what a notebook shows badly (long runs, 3-D scenes, real-time
  games, the racecar on its circuit, the rocket), about nine scripts. Not yet ruled.
- **Taught in the tutorials and used nowhere else** (a word match over the 94 demos, the
  course and topic notebooks and the projects, 2026-11-12; to redo after the demo rebuild):
  47 of the 218 library names the tutorials use, nearly all from `core/`. Fifteen are
  internals of the problem shown as user tools (`goal_point`, `horizon_kind`, `cost_bound`,
  `drawn_paths`, `realize`, ...). The candidates to leave the library, used by almost
  nothing inside it either: the evaluation's `feasibility_rate`, `reachability_rate`,
  `success_rate`, `rollout_cost`, `terminal_cost`, `cvar`; `Particles`, `MarginField`,
  `SampledCost`, `Batched`, `angle_features`, `as_step_system`, the function
  `value_iteration`. With one or two users in the repository: `region_of_attraction`
  (one demo, a 400-line module), `TransferFunction`, `Lead`, `step_response`,
  `find_equilibrium`, `discretize`, `QuadraticField`, `JointImpedance`, `IntegralImpedance`,
  `LowPassFilter`.
- **Found while writing the showcase:**
  - `array_module`, the one line a hand-written `f` needs to run on both backends, has no
    public import: two core notebooks and five demos reach `minilink.core.backends` through
    the import allowlist;
  - the comparison table of `advanced/02_optimal_control` reports value iteration as not
    converged (3000 sweeps, the change still 20) and three of its four answers as missing
    the goal on the judged run, on the catalog pendulum as on a hand-written one.

### 4.6 The tutorials and the drafted demos (2026-11-12)

What each layer is for, and the draft the maintainer reviews:

- **Each layer for its reader.** `basics/` is for engineering students who design an LQR on
  a pendulum and a cart-pole and simulate it: the cart-pole balance and its animation join
  the pendulum's. `advanced/` is for research students, whose plants are larger: the MPC
  notebook gains a car (the dynamic bicycle brought back onto its lane), the
  reinforcement-learning one a planar drone learning to hover and a pointer to the UR5 demo.
  `core/` is the maintainer's reading documentation of the core objects.
- **`advanced/02_optimal_control` said the opposite of its table:** value iteration "ends in
  the goal with the lowest cost" beside a table reporting it not converged, off the goal and
  the most expensive. With a cost for leaving the state box it converges (1136 sweeps) and
  is the cheapest answer, in the goal.
- **The core notebooks are exact:** every quoted signature resolved against the code by a
  script, which found 46 mismatches and 8 names that did not exist, none now. What the same
  check found in the library is in QUESTIONS.md ("What the check of the core notebooks found
  in the API") and KNOWN_ISSUES ("The compiled evaluators").
- **The proposed demos are drafted** in `examples/demos_proposed_minimal/`, its README the
  list. The first draft was ten scripts, most of them existing demos copied and tidied;
  the maintainer refused it the same day: a copy keeps a script's accidents (every option
  of an API it was written to exercise, a figure too many, a diagram never shown), and a
  quick test is not a story. The second draft is seven scripts, each redesigned as one
  story that shows its diagram where the wiring is the point and ends on one plot and one
  animation:
  - `cartpole_swingup.py` and `cartpole_lqg.py`, kept, each cut to its story's figures;
  - `racecar_mpc.py`, rewritten: a plan made on the kinematic bicycle drives the dynamic
    racecar through an inner speed loop (a PI on the speed, the steering to the servo), the
    cascade drawn. It replaces the dynamic-model MPC with its three spatial fields, which
    is research material, and the comparison of path-tracking laws, which is too;
  - `ur5_impedance.py`, the UR5 in 3-D, in place of the two-link arm;
  - `drone_learns_to_fly.py` in place of the rocket landing, which does not learn its task
    (below); at 600 000 steps, 40 s, the drone ends within 0.15 m of its hover and every
    scored run stays within the limits;
  - `rrt_star_obstacles.py`, kept;
  - `offroad_car_game.py`: the 3-D car as the game demo, in place of its open-loop run and
    of the cart-pole game (matplotlib and pygame together are not yet reliable enough for
    a kept demo).

  Each ran to its end, with its animation on matplotlib in place of MeshCat and the game
  replaced by a short simulated run.

Seen while drafting, and left as they are:

- `udes_racecar/mpc_racecar_dynamic.py` does not do what its comments say: in its 5 s the
  car covers 21.7 m of the 36.6 m circuit, not "a little more than one lap", and its centre
  strays 0.81 m from the line, beyond the 0.6 m lane.
- `rl/rocket_landing_rl.py` trains 4 million steps in about two minutes and does not learn
  the landing: over 100 drawn starts, 54 % of the runs stay within the limits and the mean
  cost is 7.9 × 10⁴ (this container, JAX 0.12).
- In `racecar_mpc.py` the car laps on the outer half of its lane, not on the line: the cost
  only keeps it inside the lane. A pull toward the line (the track's distance field) made
  the plan stall, and a narrower lane too; neither is in the draft.
- `Comparison.plot_trajectory` scores an infinite-horizon answer on a default 10 s, with a
  warning, whatever the horizon of the problem it was compared on (`advanced/02`).

### 4.7 Demos to add later (noted 2026-11-12, none drafted)

The seven of §4.6 are the shortlist. Candidates for two or three more, the maintainer's to
pick:

- **Two MPCs on the car, each on the model that suits it** (the maintainer's idea),
  prototyped the same day in `examples/demos_proposed_minimal/`, both on the 18 m circuit
  of `racecar_mpc.py`:
  - `racecar_mppi.py`: the cascade of `racecar_mpc.py` with its planner replaced by MPPI
    on the kinematic bicycle (2048 sequences per tick, 1 s ahead). It laps in 6.4 s, the
    car's centre within 0.43 m of the line (0.50 m with the collocation planner), and
    runs in 27 s.
  - `racecar_fast_lap.py`: SQP on the dynamic model, asked for 5.5 m/s. The plan brakes
    to 3.0 m/s into the corners and reaches 4.75 m/s on the straights: a lap in 4.8 s
    against 6.4 s for the kinematic plan, the centre within 0.58 m of the line of a 0.6 m
    lane (the plan is given a lane of 0.4 m as its margin).

  The pair tells one story: the planner follows the model, sampling where the model is
  cheap and the cost rough, gradients where the model is rich and the cost smooth. Open:
  whether `racecar_mppi.py` replaces `racecar_mpc.py` (same cascade, and it tracks the
  line better) or stands beside it.
- **Identification of a car from a driving log, by gradient through the simulator:** the
  differentiable path has no demo on anything larger than a pendulum's gains; the numbers
  to find are the tire parts' (`params["tire"]`).
- **A rocket landing that works:** the README's front row shows one, and
  `rl/rocket_landing_rl.py` does not learn it (§4.6); by a larger training budget, or by
  trajectory optimization with feedback.
- **One controller on the kinematic, the dynamic and the 3-D car:** after the ruling on the
  3-D car's rear axle (QUESTIONS.md).

### 4.8 The rule that keeps the set small

- A new demo tells a story none of the others tells.
- A new plant goes in the catalog, with its own hello-world.
- A new tool gets a tutorial section, and a line in demo 1 when it is a planner.

Decided with the set (TODO §1.12): the supported plants (K8-3) and GMC714's pinned names (G3).

## 5. What the library carries, measured

### 5.1 What the kept surface runs of the library (measured 2026-11-12)

A first input for the core cut. Every notebook and script of the surface as proposed was
run under line coverage of `minilink/`: the showcase, the 16 tutorials, the 9 proposed
demos and the 25 course and topic notebooks, 51 runs on one cloud machine. The library had
32 741 executable statements. Not repeated since: the plotting contract rewrote the plot
code of `graphical` and `planning`, the region of attraction went from 910 lines to 400, and
a ninth core notebook (plots and animations) joined, so the `graphical`, `planning` and
`analysis` rows are the ones to re-run before they decide anything.

| Package | Statements | Run by the showcase, basics, advanced and demos | + core notebooks | + course notebooks |
| --- | ---: | ---: | ---: | ---: |
| `planning` | 7 386 | 62 % | 65 % | 72 % |
| `core` | 6 762 | 72 % | 77 % | 79 % |
| `graphical` | 5 060 | 48 % | 48 % | 54 % |
| `dynamics` | 4 989 | 44 % | 45 % | 45 % |
| `analysis` | 1 481 | 49 % | 51 % | 66 % |
| `control` | 1 360 | 47 % | 47 % | 53 % |
| `simulation` | 1 333 | 32 % | 35 % | 35 % |
| `experimental` | 1 326 | 0 % | 0 % | 0 % |
| `learning` | 1 113 | 57 % | 57 % | 57 % |
| `optimization` | 1 105 | 73 % | 79 % | 79 % |
| the rest | 826 | | | |
| **all** | **32 741** | **54 %** | **57 %** | **61 %** |

- **Never imported by anything kept:** 29 files, 3 258 statements: `experimental/` whole
  (1 326); `analysis/lyapunov.py` (321 statements then) and `modal.py`; `control/geometric.py`,
  `modelbased.py` and `stochastic.py` (454 together); `dp_jax.py` and `approximation.py`
  of dynamic programming (306); the boat and the three bodies with the boat's skin; the
  Plotly live signal plot (the Plotly control-plot backend is gone); `multiple_shooting.py`; the static
  simulator.
- **Imported and mostly not run:** catalog plants nothing kept uses (the plane, the arms
  other than the UR5, the double pendulum, the longitudinal cars, the mountain car, the
  linear mass-spring-dampers: about 1 300 statements); the tabular learners and SAC
  (240); `core/inspect.py` (106 of 154 not run).
- **Kept alive by the course notebooks alone:** the Gymnasium interface, the sensitivity
  functions and the time responses, parts of the fields, the costs and the step problems.
- **What the measure cannot say:** the MeshCat and pygame renderers, the realtime simulator
  and most of the 3-D car read as unused because the runs replaced MeshCat by matplotlib
  and skipped the games; four course notebooks did not run (Stable-Baselines3, PyTorch and
  Gymnasium's extras are not installed); a statement not run is not a statement to remove
  (refusals, the other backend, an option nobody passed here). The Ipopt optimizer reads as
  unused for the same reason.

### 5.2 What a count of every definition says

| Fact | Measure |
| --- | --- |
| Code named nowhere at all | 5 definitions, 120 lines |
| Code named only by tests | 18 definitions, 300 lines |
| Departing features (experimental, path-tracking laws, boat, three-body) | referenced outside their own files only by export-table lines: they lift out with no surgery |
| Plotly | the exception: 22 library files name it |
| Validators (`require_*`) | 299 functions, 2 340 lines, 3 % of the lines; 47 never run by the kept surface and about ten exact duplicates (the count of 2026-11-12) |
| Docstrings | 13 000 lines of the library's 81 600 (65 900 not blank) |
| `if __name__ == "__main__"` blocks inside the library | 37 files, 490 lines |
| Export tables | 267 public names in 389 entries; 120 names are listed in two tables (the count of 2026-11-12) |

So the library is not carrying dead weight in the plain sense. What it carries is of five
kinds: **twins** (the same logic written twice), **reporting extras** (animations, histories,
live plots built for old demos), **the large drawing code** (a second plot library, the 3-D
renderer, the skins), **options nobody passes**, and **leftovers** of removed designs.

## 6. The library by area (`minilink/`, 81 600 lines)

### 6.1 Core and compile

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| The `System` family, diagrams, wiring, params, roles, signals, kinematics (`core/system.py`, `diagram.py`, `wiring.py`, `assembly.py`, `closed_loop.py`, `operators.py`, `params.py`, …) | 5 500 | 70–90 % | keep, clean | 2 | H | The gear everything turns on (`system.py` imported by 78 files). Only your API review should move it; the 19 inconsistencies logged in QUESTIONS are its input. |
| The core math objects (`trajectory.py`, `sets.py`, `costs.py`, `distributions.py`) | 2 400 | 68–86 % | keep, clean to textbook | 2 | H | The objects a student reads. The place where "reads like a textbook" matters most. |
| Fields (`core/fields.py`: `Field`, `GridField`, `MarginField`, `QuadraticField`, `EllipsoidSet`, field costs) | 950 | 50 % | minimize the surface | 2 | M | A building block of the direction (cost-to-go as a terminal cost, margins, safety), imported by 16 files. Too many classes for what is used today; decide in the API review which ones are the object and which are helpers. |
| Shaping (`core/shaping.py`: hinge, excess, barrier, occupancy) | 175 | 56 % | keep | 3 | H | Small, and the lane costs of the racecar demos and both research projects use it. |
| Inspection text (`core/inspect.py`, `reporting.py`) | 320 | 27 % | minimize the surface | 3 | M | One formatter per object type, imported by 16 files, mostly not run. Worth one pass: keep `repr` and the notebook display, drop what no object calls. |
| Compile (`core/compile/`: plan, compiler, evaluators) | 3 900 | 57–93 % | keep, clean | 2 | H | A gear: every simulation, planner and gradient goes through it. `evaluators/discrete.py` (846 lines) and `jacobian.py` (57 %) are the two to read first; the three defects in KNOWN_ISSUES live here. |
| Sampled and step systems (`core/discrete/`) | 1 400 | 57–79 % | keep | 3 | H | GRO501's digital implementation and the `%` operator. |
| Facades (`core/facades/`: the methods `sys.plot…`, `sys.animate`, `sys.compute_trajectory`) | 1 700 | 50–63 % | minimize the surface | 2 | M | The first thing a student types. Half of each file is options nobody passes; the API review decides which keywords stay. |
| Paths and shapes (`core/geometry/`) | 600 | 62–80 % | keep | 3 | M | Tracks and obstacles need it; its plan doc (geometry-module.md) is open. |

### 6.2 Analysis, control, estimation, blocks

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| Linear analysis (`linearization`, `linear`, `frequency`, `time_response`, `sensitivity_functions`, `structural`, `riccati`, `derivatives`, `equilibria`, `discretization`) | 3 300 | 75–100 % with the course notebooks | keep | 3 | H | All of GRO501. Leaves, well tested, low upkeep. |
| Region of attraction (`analysis/lyapunov.py`) | 400 | 0 % | keep | 3 | H | Reduced to its core: `region_of_attraction`, its certificate and one plot. A leaf (nothing imports it), the direction of the stability research. One old demo calls it. |
| Modal analysis (`analysis/modal.py`: `poles`, `modal_analysis`, `animate_modal`) | 200 | 0 % | keep | 3 | M | A small leaf, a classic of vibrations teaching, and `poles` is used by GRO501. No course of the three animates modes today; if none will, it goes to the scratchpad. |
| Classical and state-space laws (`control/siso.py`, `state.py`, `design.py`) | 1 200 | 86–96 % | keep | 3 | H | GRO501. |
| Arm laws (`control/impedance.py`, `robotic.py`) | 770 | 63 % | keep | 3 | H | GMC714 weeks 4–5 and the UR5 demo. |
| Computed torque and sliding mode (`control/modelbased.py`) | 270 | 0 % | keep | 3 | H | GMC714 week 5. Unused by the kept demos, needed by the course. |
| Path-tracking laws (`control/geometric.py`: pure pursuit, Stanley, line of sight, heading) | 620 | 0 % | move to the research repo | 1 | M | You called path tracking research. Four public names at the top level for a topic no course teaches. Called by four old racecar demos, no project and no tutorial; they would travel with the path-tracking research. If one stays as a teaching example, keep pure pursuit alone. |
| Neural policy (`control/neural.py`) | 340 | 70 % | keep | 3 | H | What every learner returns. |
| Gaussian policies (`control/stochastic.py`) | 260 | 0 % | delete, or keep as the notes' stochastic-policy object | 2 | M | Traced: no library file calls them (only the export table of `control/` names them; the learners sample through their own `policy.py`), no example either; three test files do. Your call: they are the textbook object `π(u | x)` as a block. |
| Observers (`estimation/`) | 375 | 87–92 % | keep | 3 | H | GRO501, the LQG chapter of GRO860, the estimation research. |
| Signal blocks (`blocks/`) | 1 230 | 48–100 % | keep | 3 | H | Small leaves, the vocabulary of diagrams. |

### 6.3 Dynamics catalog

A catalog plant is a leaf: nothing depends on it, and it is out of a reader's path. The
question for each is trust and size, not use.

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| Abstractions (`state_space`, `mechanical`, `manipulator`) | 1 070 | 67–72 % | keep | 3 | H | The base classes of the catalog. |
| `GeneralizedMechanicalSystem` | 230 | 17 % | keep, check | 3 | M | Two plants subclass it, the boat and the 3-D plane. If the boat leaves it has one subclass, and it is `MechanicalSystem` with one extra matrix: fold it then. |
| Pendulum, cart-pole, double pendulum | 1 250 | 16–62 % | keep | 3 | H | The teaching plants of all three courses. A third pendulum file is a leaf. |
| Mass-spring-damper, integrators, oscillators | 720 | 23–48 % | keep | 3 | H | The README's example, the double integrator of GRO860, Van der Pol and Lorenz for the phase plane. |
| Manipulators (`arms.py`: one to five links, kinematic; `ur5.py`) | 1 650 | 13 % / 82 % | keep, clean `arms.py` | 2 | H | GMC714 weeks 2–5. `arms.py` is 1 040 lines and the file a robotics student opens to see `M`, `C`, `g`: the best candidate in the catalog for a textbook pass. |
| The vehicle ladder (kinematic bicycle, dynamic bicycle, drivetrain, racecar, tires, terrain) | 2 200 | 70–92 % | keep | 2 | H | GRO501, GMC714 week 6, the vehicle research. Its open design points are in QUESTIONS. |
| The 3-D car (`suspension_car.py`, presets, 3-D skins) | 1 750 | 22 % | keep, marked as a prototype | 3 | M | The direction of the off-road research and the game demo. Valid on mostly flat ground only (stated in its docstring). If it stops growing, it belongs in the research repo. |
| Longitudinal cars (`propulsion.py`), quarter car (`suspension.py`), mountain car | 630 | 17–23 % | keep | 3 | H | GRO501 and GMC714 week 6; the active-suspension exercises of GRO860; a classic RL problem. Small leaves. |
| Drone, rocket | 550 | 62 % / 20 % | keep | 3 | H | The RL demos and the project page. The rocket-landing training does not learn here: a known issue, not a reason to drop the plant. |
| Planes (`aerial/plane.py`: 2-D and 3-D) | 840 | 10 % | reduce to core | 2 | L | The largest unused plant. I do not know whether "learning to fly" in GRO860 uses the plane or the drone. If the plane: keep the 2-D one, move the 3-D one to the scratchpad. If the drone: move both. |
| Boat and its skin, three-body | 840 | 0 % | move to the scratchpad | 1 | M | No course, no research project on the site, no demo in the kept set. Cheap leaves, so low gain; the reason to move them is trust (never run) more than size. |

### 6.4 Planning, learning, optimization

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| The planning framework (`problems.py`, `evaluator.py`, `evaluation.py`, `comparison.py`, `results.py`, `planner.py`, `environment.py`, `step_model.py`, `step_problems.py`, `policies.py`) | 6 100 | 52–86 % | keep, clean; own subpackage | 2 | H | The direction of the library and just built; normal that few demos use `evaluate` and `compare` yet. `evaluator.py` (1 415 lines) and `problems.py` (1 253) are the two largest files of the library and the first to split. The subpackage question is in QUESTIONS. |
| Value iteration and LQR, the reference (`dp.py`, `discretizer.py`, `lookup_policy.py`, `lqr.py`, `progress.py`) | 2 600 | 54–80 % | keep | 3 | H | GRO860's centre and the textbook reference of the code base. |
| Value iteration on JAX (`dp_jax.py`) | 450 | 0 % | keep, and give it a kept demo | 2 | M | The fast path of the same algorithm. It is tested (`test_dynamic_programming.py` runs both backends), benchmarked (`benchmarks/suites/dp_speed.py`) and called by three old demos, but by nothing in the kept surface: one demo (the cart-pole swing-up table) keeps it in view. |
| Linear approximators and features (`approximation.py`) | 200 | 0 % | keep | 3 | M | GRO860 week 8. The course notebook builds its own by hand today; the library version is the tidy one a later term can use. |
| Planner plots (`dynamic_programming/plotting.py`, `planning/plotting.py`, `spatial/plotting.py`, `search/plotting.py`, `mpc/viz.py`, the two live plots) | 2 600 | 35–51 % without the courses | minimize the surface | 2 | M | Five plotting files and two live plots, a seventh of the planning code by volume. Each plot builds a figure description now (DESIGN §7), so what is left to cut is options and whole plots, not drawing code: one pass per file with the notebooks open. The value-iteration animations stay (ruled). |
| RRT and RRT* (`planning/search/`) | 1 800 without plots | 81–96 % | keep | 3 | H | GMC714 week 7. Dubins steering (115 lines, 9 %) is a leaf worth keeping for cars. |
| Scenes, tracks, collision (`planning/spatial/`) | 900 without plots | 58–73 % | keep | 3 | H | Every obstacle and racecar demo, both research projects. |
| Trajectory optimization and MPC (`planner.py`, `direct_collocation.py`, `shooting.py`, `transcription.py`, `compiled.py`, `mpc/predictive.py`, `path_integral.py`) | 3 800 | 66–95 % | keep | 3 | H | GMC714 week 7, GRO860 chapters 12–13 and 21, every racecar demo. |
| Multiple shooting | 160 | 0 % | keep | 3 | M | A leaf the roadmap names for GRO860; one line in a tutorial would run it. |
| Policy-gradient learners (`planner.py`, `ppo.py`, `reinforce.py`, `actor_critic.py`, `policy.py`, `critics.py`, `collect.py`) | 2 100 | 23–98 % | keep | 3 | H | REINFORCE and actor-critic are chapters 18 and 19 of the notes: textbook value beyond their use. |
| SAC (`sac.py`) | 200 | 14 % | keep | 3 | M | The advanced chapter and the project page name it. A leaf; drop it only if it fails where PPO works. |
| Tabular learners (`tabular.py`: Q-learning, SARSA, Monte Carlo, ε-greedy, UCB) | 620 | 27 % | keep | 3 | H | GRO860 chapters 14 to 16. |
| Gymnasium and SB3 interface (`interfaces/gymnasium.py`) | 420 | 38 % | keep | 3 | H | GRO860 week 5 is a lab on exactly this. |
| Optimization boundary (`mathematical_program.py`, `optimizer.py`, evaluators, first-order, SciPy) | 2 560 | 60–100 % | keep | 2 | H | The second pillar of the showcase. `optimizer.py` (754 lines) is the one to clean. |
| Ipopt backend | 110 | 0 % | keep | 3 | M | An optional leaf; the conda stack installs it. Untested on this machine. |

### 6.5 Simulation and graphics

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| Simulator and solvers | 1 750 | 42–86 % | keep | 3 | H | A gear, stable. |
| Static simulator, clock runner | 270 | 0 % | keep the first; the second is a candidate | 2 | M | Traced: `compute_trajectory` on a static block reaches the static simulator (164 lines), so it is a user path. The host clock runner (105 lines) is reached only by `solver="scipy"` on a sampled system, which one test passes (§8.4). |
| Real time and games (`simulation/realtime/`, the pygame renderer) | 1 700 | not measured | keep, and fix the two-window bug first | 2 | M | Games are the best first contact for a student. The Matplotlib-plus-pygame bug you named decides whether it deserves its place. |
| Animation core (animator, camera, primitives, drawables, the Matplotlib renderer, the records) | 2 800 | 24–97 % | keep | 3 | H | Every `animate()`. |
| MeshCat renderer | 1 080 (955 not blank) | not measured | keep | 3 | M | The 3-D view of the UR5 and the cars: the showcase and the plotting tutorial, five proposed demos and fourteen old demos select it. The largest renderer, untouched by the plotting contract; hangs without a browser (known issue). A candidate for a cleaning pass, not a removal (§8.3). |
| Plotly: the animation renderer (730 lines), the figure renderer (470), the live signal plot (240) | 1 440 | not measured | your call on Colab | 1 | M | A second way to draw what Matplotlib draws, in three pieces of unequal use: the figure renderer draws every plot's description (`renderer="plotly"`: one tutorial, two old demos); the animation renderer plays `animate(renderer="plotly")` (two tutorials); the live signal plot is the `plot_renderer="plotly"` of the games (the proposed off-road game, two old ones). Each piece and what its removal costs: §8.3. |
| Skins (`racecar_skin.py` 1 330 lines, `car_skins_3d.py` 640, `skins.py`, `shapes.py`) | 2 660 | 11–83 % | clean | 3 | M | Drawing code, leaves, but the racecar skin alone is larger than half the vehicle ladder's dynamics, and the two car files are 1 660 lines not blank. Worth shrinking when touched, not before (§8.3). |
| Mesh tools (`graphical/meshes.py`: STL and Collada loaders, weld, simplify, refine) | 610 | 45 % | minimize the surface | 3 | M | Needed for the UR5; the simplify and refine tools look like one-time build tools. |
| The figure description and its Matplotlib renderer, diagram pictures, phase plane, port map, time signals | 4 100 | 45–97 % | keep | 3 | H | Used by all three courses. One description every plot builds (`graphical/figure.py`) and the renderer that draws it; `port_map.py` (660 lines) is the one to read with the notebooks open. |

### 6.6 Experimental

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| C export, the contact and tire engines, the world (`experimental/c_export.py`, `engines/`) | 1 400 | 0 % | move to the research repo | 1 | H | Research prototypes with their own tests; nothing on the kept surface touches them. One course notebook mentions `experimental`: check it before the move. |
| Symbolic mechanics (`experimental/symbolic/`) | 1 100 | 0 % | move to the research repo | 1 | M | Derives `M`, `C`, `g` symbolically, which is GMC714 week 3 done by hand today. If you want it in a lab one day, it returns as a finished feature. |

## 7. Outside the library

| Chunk | Size | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- |
| Research projects (`examples/projects/`: car trajectory optimization, MPC, path tracking) | 31 files, 7 900 lines of Python in 29 scripts | move to the research repo | 1 | H | More code than all demos together, read-only for agents, and it subclasses catalog classes, so it blocks renames. Pinned to a release in its own repository it stops constraining the library. |
| The old demo folders (`examples/demos/`, 94 scripts) | 5 800 lines | the maintainer's pass: which stay, which are rewritten, which go, after a snapshot tag | 1 | H | The curated set would replace them. The good ideas are listed in §4.4 and §4.7; your own favourites go to the scratchpad first. |
| The proposed demos (`examples/demos_proposed_minimal/`, 9 scripts) | 790 lines | keep; becomes `examples/demos/` once you choose | 1 | M | Waits for your selection, and for the choice between the two racecar cascade demos. |
| Showcase, basics, advanced tutorials | 9 notebooks | keep | 3 | H | Just brought up to date; one per audience. |
| Core notebooks (`examples/tutorial/core/`, 9) | 9 notebooks | keep until the API review, then turn into the developer docs or delete | 2 | H | Your working documents for that review. |
| Unit tests (`tests/unittest/`, 80 files) | 41 400 lines | reduce, by rewriting around contracts | 2 | M | Half the size of the library. Five files over 1 500 lines. Not a deletion: each file is rewritten when its module passes the API review, keeping the contract tests and the textbook checks. |
| Vocabulary test (`test_vocabulary.py`) | 1 090 lines | minimize | 2 | M | Guards the public names; shrinks with the surface. |
| Tests of experimental code (`tests/experimental/`) | 430 lines | move with the code | 1 | H | |
| Demo and notebook checks (`tests/demo_checks/`, `tests/run/`) | 1 600 lines | keep | 3 | H | The gate that kept the tutorials honest in this run. |
| Benchmarks (`benchmarks/`, 26 files) | 5 900 lines | reduce to core | 2 | M | Keep the regression gate CI runs and the speed claims the README makes (four suites). The scenarios and host profiles beyond that are the scratchpad's. |
| Developer docs (`docs/dev/`) | 4 600 lines | reduce | 2 | M | DESIGN and RULES are law; ROADMAP and QUESTIONS shrink as v0.4 closes. |
| Plan docs (`docs/plans/`, 10 beside TODO) | 3 400 lines | keep | 3 | H | The landed plans are deleted; what remains is the design of what is not built, and this document. The pyro parity table is re-audited after the demo pass. |
| API docs (`docs/api/`, Sphinx) | 550 lines | keep | 3 | H | Generated from docstrings. |

## 8. Behind the kept surface: what could be cleaned

A second look at the library's internals, assuming the public surface of §6 stays. The
question: which plumbing, duplicates and options exist for a feature, a caller or a demo that
is leaving, or for none?

### 8.1 Twins: the same logic twice

The largest gain, and the kind that most hurts a reader: two places to read, two places to
fix, and they drift.

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| Trajectory optimization: `direct_collocation.py`, `shooting.py`, `multiple_shooting.py` | Each transcription is written once for NumPy and once for JAX. They have drifted: the NumPy shooting path adds bound rows that are always infinite on an unconstrained problem, the JAX path skips them. One program written on `(z, params)`, looped on NumPy and vectorized on JAX, replaces both. | 300 | behaviour (NumPy programs gain `params`) | M |
| Compile: the step diagram's own plan and evaluator (`plan.py`, `evaluators/discrete.py`, `compiler.py`) | A tick diagram is the sampled diagram with one rate. Its plan, evaluator and trace check are line-for-line copies of the sampled ones. | 200 | behaviour; feasibility not proven (nested boxes are flattened by one and kept whole by the other) | L |
| Value iteration on JAX: three chunked table builders | Three near-identical `jit(vmap)` builders (`build_jax_sa_chunks` and `build_jax_node_chunks` in `dp_jax.py`, `build_xnext_jax_chunks` in `discretizer.py`), and a second copy of the out-of-bound penalty of `dp.py`. | 80 | none | H |
| Value-iteration plots against the generic planning plots | The heatmap panel and the 3-D surface panel are described twice (`field_figure`, `value_surface_figure` in `dynamic_programming/plotting.py`; `cost_to_go_panel`, `cost_to_go_surface` in `planning/plotting.py`). The drawing itself and the state-label helper are one since the plotting contract. | 40 | none | H |
| The "problem leaves" tree | The parameter tree of a problem is assembled in six places; two of them differ by one argument. | 30 | none | H |
| "The problem's box, else the plant's" | Coded five times (problem, plot, grid, RRT, step problem). | 15 | none | H |
| The closed-loop score of a step problem (`planner.step_evaluation`) | Repeats `score_run` line for line. The two discount the terminal cost differently (γᴺ against e^(−ρT)), so the merge needs care. | 50 | behaviour | M |
| The Gymnasium environment's episode rules | `ProblemEnv.step` repeats the failure and termination rules of `RolloutEnvironment.step`. | 20 | none | M |
| Evaluators | `input_sequence_of` on the continuous and discrete evaluators differ by one argument; the dynamic and static evaluators repeat their constructor; `step_body` and `step_path` repeat the substep loop. | 50 | none (the last one: trace shape, unchecked) | M |
| Simulation facade | `compute_trajectory` and `compute_rollout` of a stepped system repeat the same setup and the same roles block. | 25 | none | H |
| Small exact twins | `costs.unique_term_id` = `assembly.unique_id`; `frequency.caller_stacklevel` = `core.options`; two spring coils; two Rodrigues rotations; `meshes.translation_matrix` = `kinematics.translation`; `require_mechanical_plant` (`robotic.py`, `modelbased.py`), `require_manipulator` (`robotic.py`, `arms.py`), `require_axis` (`port_map.py`, `phase_plane.py`), `require_integer_key` (`evaluator.py`, `problems.py`) each twice; `plot_bode` redoing `bode` and `margins`. | 150 | none | H |
| The two mechanical base classes | `GeneralizedMechanicalSystem` (231 lines) has two subclasses, the boat and the 3-D plane; if the boat leaves it has one, and it is `MechanicalSystem` with one extra matrix. | 200 | public base class | M |

**Not twins, checked:** the paired analysis names (`step_response`, `margins`, `root_locus`,
`poles` in two files each) are a kernel on matrices and a verb on systems, each with its own
callers. The three `plot_control_law` are three layers. The two `shapes.py` do different
jobs. The NumPy and JAX program evaluators are legitimately different.

### 8.2 Reporting extras built for the old demos

Animations, histories and live plots whose only callers are scripts of `examples/demos/` or
the projects. The algorithms do not need them. Whether a demo that shows one stays is the
maintainer's pass; a row here goes only with its last caller.

| Where | What | Called by | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- | --- |
| RRT: `animate_search`, `animate_convergence`, the snapshot history | The tree growing, and RRT*'s path improving. `plot_tree` and the live plot (`LiveSearchPlot`) are used by the kept surface and stay. | one old demo each (`rrt_pendulum_swingup.py`, `rrt_star_live.py`) | 300 | two public methods | M |
| Trajectory optimization: `live_plot=`, `record_history=`, the iteration record, `live_plot.py`; the evaluator's `live_plot` | `callback=` could stay as a plain pass-through to the optimizer; no example passes it to this planner. | `live_plot=`: one old demo (`trajopt_cartpole_collocation_jax.py`); the evaluator's: one old demo (`pid_autotuning_native.py`); `record_history=` on this planner: none | 230 | public keywords | M |
| Soft cost fields in a `Scene` (`workspace_fields`, `cost_density`, `GaussianField` drawing) | The kept surface uses a scene for obstacles and clearance only. | one old demo (`rrt_holonomic_obstacles.py`) | 200 | a `Scene` argument | M |
| The cost-field figure, `plot_cost_field` and its surface (`is_3d=True`) | The raster of a spatial cost and its 3-D surface. | three project scripts and one project notebook, one old demo (`mpc_car_circuit.py`); the surface by two project scripts, the notebook and that demo | 190 | none: it moves with the projects | H |
| The MPC run pictures (`mpc/viz.py`, 491 lines): the horizon and trail overlays (`mpc_animation_overlays`), the sampled-futures plot (`plot_sampled_futures`) | Pictures of a closed-loop MPC run; the planner and the loop do not need them. No tutorial and no proposed demo draws them: the three proposed racecar demos animate the lap without the horizon. | the overlays: five old demos, three project scripts and one project notebook; the plot: one old demo (`mppi_pendulum.py`) | 490 | public functions; the horizon over the car is the MPC's teaching picture | M |
| Racecar skin: `palette`, `equipment`, `scene`, `heading_arrow`, `wheel_radius`; a debug look (`frame_axes_skin`) | A skin is only ever attached as `car.skin = racecar_skin_3d` and called as `skin(plant)`; the keywords are never passed. | none | 55 | public keywords | H |

A judgment that is yours: an animation of the search tree is a strong teaching picture, as
the value sweeps are. If a course will show one, it stays and gets a demo; the point of the
row is that today nothing kept shows it.

### 8.3 The large drawing code

The plotting contract (one figure description, two renderers that draw it, one record) left
the library the same size: net +207 lines on about 66 000 not blank. It removed the second
copy of each plot's drawing, not the drawing code that is large on its own. Five pieces
remain large and were not touched. Each is a candidate; the column says what a user loses.

| Where | Lines (not blank) | Who uses it | What a removal would cost the user | Risk | Conf. |
| --- | --- | --- | --- | --- | --- |
| The MeshCat animation renderer (`graphical/animation/renderers/meshcat_renderer.py`) | 955 | the showcase and the plotting tutorial, five proposed demos, fourteen old demos | every 3-D animation in the browser: the UR5, the racecar and 3-D car skins, the saved `.html` clips. Matplotlib's 3-D view is the fallback, without meshes at a usable speed. A cleaning pass (22 underscore methods, the flipbook of changing geometry) is the realistic candidate, not a removal. | behaviour, for every 3-D demo | M |
| The Plotly animation renderer (`renderers/plotly_renderer.py`) | 643 | two tutorials (`basics/01`, `core/09`); no demo | `animate(renderer="plotly")`: an interactive clip in a notebook with no viewer to start. Matplotlib's inline player and MeshCat's inline page cover Jupyter and Colab. | a renderer name refused | M |
| The Plotly figure renderer (`graphical/figure_renderers/plotly.py`) | 405 | one tutorial (`core/09`), two old demos | `renderer="plotly"` on every plot: zoom, pan and hover. The description stays; only Matplotlib would draw it, and `renderer=` would have one value. | a keyword value on every `plot_*` | M |
| The Plotly live signal plot (`graphical/signals/plotly_backend.py`: a polling HTTP server behind a browser tab) | 195 | the proposed off-road game and two old games, by `plot_renderer="plotly"` | live signals beside a pygame or MeshCat game without Matplotlib's window, the pairing that is not reliable today (the two-window bug). Fixing that bug removes the reason for this file. | one keyword of `game()` | M |
| The port map and the block control-law plot (`graphical/port_map.py`: `plot_input_output_map`, `plot_control_law` on a block) | 569 | `plot_control_law`: three tutorials (on a controller, and behind `solution.plot_control_law` and `Comparison.plot_control_law`) and eleven old demos; `plot_input_output_map`: no example, one line of the systems notebook | the picture of a law `u(x)` on a plane, for any block, a neural policy included: a GRO860 and GMC714 picture. The candidate is its inside (the sweep axes and the error-space options: 360 of its lines are helpers under the two plots), read with the notebooks open; `plot_input_output_map` alone has no caller. | public methods | M |
| The racecar skin and the 3-D car skins (`graphical/catalog/racecar_skin.py`, `car_skins_3d.py`) | 1 658 | the racecar skin: three proposed demos and seven old ones; the two 3-D car skins: the two presets | the look of the flagship platform and of the two 3-D presets; `car_skin_3d` (in `skins.py`) is the plain fallback. The racecar skin holds a scene (floor tiles, walls, painted lines), a 2-D top view, a lidar, a camera and a shadow: the candidate is those extras, each a few dozen lines. | looks only | M |

If Plotly goes whole (its three pieces, 1 240 lines not blank), these go with it: the style
table (`common/plotly_style.py`), the Plotly branch of the show step and of the two records,
the renderer dispatch of the live signal plot and its handle base class with one subclass
left, and the `"plotly"` value of `renderer=` and `plot_renderer=`. **What blocks it:** the
proposed off-road game passes `plot_renderer="plotly"`, and two tutorials show
`renderer="plotly"`. Your call on Colab.

### 8.4 Options and paths nobody uses

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| Sampled systems: `solver="scipy"` and the host clock runner (`simulation/discrete/clock_runner.py`) | Reached only by that keyword on a sampled system; one test passes it. | 140 | a keyword value refused | M |
| The evaluator's adaptive SciPy path (`ProblemEvaluator`, `solver="scipy…"`) | Tests only. | 70 | a keyword value | M |
| `compile(flatten=True)` | Tests only; on a step diagram it raises "not supported yet". | 50 | public keyword | H |
| The `"direct"` transcription backend | Passed nowhere. | 15 | behaviour | M |
| Dubins steering for RRT (`search/dubins.py`, `DubinsSteering`) | Tests only: no demo, tutorial or project. Keep with a car demo, or it goes to the scratchpad. | 220 | public class | L |
| `Tree(nearest_method="kd_tree")` | Tests only. | 35 | none | M |
| The reward-rebuild branch of the learning planner (`reward="stage"`) | A keyword no caller passes; the environment's own option stays. | 15 | none | M |
| Custom gravity hook on the arm impedance laws (`gravity=`, signature probing) | Tests only; two of its guards can never fire. | 35 | public keyword | M |
| Law keywords: label and unit overrides on `Impedance`, `y_filt0`, integrator clamps, `autowire(strict=)` | Passed by no example, except `autowire(strict=True)` in two old demos. | 50 | public keywords | M |
| A pluggable diagram-exporter layer with one backend (`graphical/diagrams/export.py`) | Graphviz is the only exporter; the layer exists for a test. | 90 | exported names | H |
| Mesh tools: the Collada loader (`load_collada`, `collada_mesh`), `split_by_radius`, `refine` | No caller, tests included. | 200 | none | H |
| JAX setup: `configure_jax`, two backend tuples | No library caller; called by the projects, the benchmarks and the tests, always with the default. | 35 | a public name | M |
| Save and load of a grid's successor tables (`Discretizer.save`, `load`) | No caller, tests included. Useful for a long solve; keep if a course uses it. | 15 | public methods | L |

### 8.5 Leftovers of removed designs

Small, safe, and each one a line a reader stops on.

| Where | What | Lines |
| --- | --- | --- |
| `core/closed_loop.py` | `resolve_standard_feedback`, its record and helper: the docstring says "shared by the closed loop and the sampled loop"; neither calls it. | 65 |
| `core/roles.py` | A branch for a system with a `.computer` attribute; no class has one (verified). | 15 |
| `core/inspect.py` | The LaTeX path goes out and comes back to one class; its generic branch would recurse forever if reached. | 18 |
| `optimization/optimizer.py` | The method table is an identity copy of the presets; a method is validated three times. | 25 |
| `planning/evaluator.py` | `numbers_key` builds a cache key, and is only tested for `None`. | 10 |
| Records | Fields written and read by no library code: `frozen_at` on the value-iteration record (read by nothing); `last_program` and `last_optimizer` on the trajectory-optimization planner (read by tests and one benchmark scenario); `last_traj` on the two simulators (read by tests). | 60 |
| Fallbacks that cannot fire | About twenty `getattr(x, name, default)` on attributes every object has; six spellings of "is this a diagram"; `if problem.X is not None` (always true). | 60 |
| Forwarders | Functions that only pass their arguments on; variadic helpers every caller gives one argument; a dataclass around one dict. | 130 |
| `SampledSystem` | Three different fixes for one problem (defaults written through the view). | 8 |
| Gymnasium interface | A termination hook nothing overrides, so its branch never runs. | 15 |
| Helpers no library code calls | `require_complete_part`, `split_by_radius` (no caller at all); `signal_stack_figsize`, `camera_eye_position`, `bounding_radius`, `stiff_tires`, `resolve_standard_feedback` (tests only). | 80 |

About 490 lines, almost all at no risk.

### 8.6 Where the readers and the plan disagree

A reader does not know the rulings; these are its claims set against them.

| Reader's proposal | Why it does not apply as stated |
| --- | --- |
| Delete the linear approximators (no caller) | GRO860 week 8. Stays; a tutorial line would exercise it. |
| Delete value iteration on JAX (no user caller, 620 lines with its plumbing) | The tests run both backends and the speed benchmark runs it (`benchmarks/suites/dp_speed.py`), so it is exercised; three old demos call it; and it is the path to large grids. Keep, merge its three builders, give it a kept demo. |
| Drop the loop form of value iteration (`method="loop"`) | The textbook reference the docstrings point to. Stays. |
| Delete the Gaussian policies (`control/stochastic.py`, 259 lines, no caller outside tests) | Open, and yours: no library code and no example calls them (three test files do). Deleted, unless you want them as the notes' stochastic-policy object. |

### 8.7 Three of the largest files, by what is inside

| File | Lines | The core | The rest |
| --- | --- | --- | --- |
| `planning/evaluator.py` | 1 415 | about 400: construct, evaluate, program, realize, run, score | validation 150; sampled-loop support 70; batch-axis tree utilities 70 (they belong with params); program packing 60; failure-cost recompute 60; held plans 40; the adaptive path and the live plot (above) |
| `planning/problems.py` | 1 253 | about 500, of which 120 are the class docstring | coercers and validators 350; the stochastic-draw helpers 150; print text 60 |
| `planning/dynamic_programming/dp.py` | 1 086 | about 350: the backup, the two cost tables, the solution | policy evaluation 100 (used in teaching); the loop reference 60; JAX dispatch and plot forwarders; the table and record classes |

For the first two, the split into a subpackage (the open question in QUESTIONS) would put the
core in a file a student can read in one sitting.

### 8.8 House-rule debts seen in passing

- **Leading underscores** (RULES 5.8), counted as methods: 22 in the MeshCat renderer, 26 in
  the Plotly animation renderer, 9 in the Matplotlib and pygame ones, 5 in the Gymnasium
  interface; the `_jit_*` attributes of the JAX program evaluator.
- **One primitive with two names:** `Line = CustomLine`, both used at about twenty sites each.
- **The racecar skin reads an STL file at import time** (`GEOMETRY = racecar_geometry()`).
- **`core/inspect.py` reads planning-layer attributes** (`scene`, `track`) from core.

## 9. The tests' clean slate (MS-D1, MS-D2)

### 9.1 What the audit found (2026-10-31)

Kept for the clean slate:
- six test files check the repository, not the library: `test_textbook_style`,
  `test_repo_contract`, `test_public_names`, `test_teaching_imports`, `test_packaging`,
  `test_demo_check_runners`;
- the demos held pyro ports (six robotics, two control), one-concept step toys (six),
  duplicates (six value-iteration demos, five racecar MPCs, two hybrid sliding modes, the
  closed loop wired three times), and few demos written for the v0.3 core;
- about 40 tests made half of the suite's time.

### 9.2 The test layers (MS-D2)

**Taken (the maintainer, 2026-10-31):** the layers are cut by what they prove, one folder per
layer; two are essential, the contract and the textbook math. The rest is decided at the
clean slate (deferred 2026-11-07).

| Layer | The question it answers | Status |
| --- | --- | --- |
| `tests/contract/` | Does every public object build, wire and run: each catalog plant, block, law, planner, solver, its ports and its `params`? | **essential** |
| `tests/math/` | Are the numbers right against a textbook worked example or a closed form: Riccati and LQR gains, an RK4 convergence order, a known optimum, a value-iteration fixed point, a Kalman covariance? | **essential** |
| Backend parity | Does NumPy equal JAX, and does a gradient exist through a loop, a planner, a solver? | open (MS-D2a) |
| Composition | Do the pieces work together: diagrams, sampled loops, problem → evaluator → optimizer? | open (MS-D2b) |
| Examples | Do the kept demos and the course notebooks run? | open (MS-D2c) |
| Repo rules | Textbook style, prose, public names, imports, packaging | open (MS-D2d) |

- **MS-D2a. Backend parity.** A. one clause of the contract (each object also runs on JAX, its
  outputs equal to NumPy's on one input); B. its own layer; C. none beyond what the math
  layer runs on both backends. *Agent:* A.
- **MS-D2b. Composition.** A. the core's end-to-end scripts only (`test_v03_core.py`, a dozen
  cases) as a third layer; B. inside the contract (a composition is a public object);
  C. none, the demos show it. *Agent:* A.
- **MS-D2c. Examples.** A. a runner, outside pytest, nightly and before a release; B. inside
  pytest; C. none in CI, the maintainer runs them. *Agent:* A.
- **MS-D2d. Repo rules.** A. a pre-commit and CI check outside pytest (seconds); B. a test
  layer; C. drop the ones nobody acts on. *Agent:* A, with C for any rule the maintainer
  does not want enforced.
- **MS-D2e. When each runs.** *Agent's proposal:* contract and repo rules on every change, math
  and composition on every pull request, examples nightly and before a release.

Once the layers are answered, each file of `tests/unittest/` is sorted case by case into
contract, math, the open layers, or deletion. The obvious ones: **repo rules**
`test_textbook_style`, `test_repo_contract`, `test_public_names`, `test_public_imports`,
`test_teaching_imports`, `test_teaching_surface`, `test_packaging`; **composition**
`test_v03_core`, `test_loop_grammar`, `test_sampled_loop`; **contract** `test_contract`,
`test_catalog_backends`, `test_blocks`; **deleted with their subject** `test_demo_check_runners`
(MS-D2c), the pyro-port cases inside `test_mechanical_robotics` and `test_dynamics_catalog`.
The slowest cases (`pytest --durations=40`) are flagged in the sort.

## 10. The sums

### 10.1 The areas, by destination (§6, §7)

| Destination | What goes there | Lines leaving the library |
| --- | --- | --- |
| **Research repo** (private) | the three projects; the experimental package and its tests; the path-tracking laws | about 3 100, plus 7 900 of projects |
| **Scratchpad** (personal) | the old demos worth a second look; boat, three-body, the 3-D plane; the extra benchmark scenarios | about 1 400 |
| **Deleted** | the old demos the pass does not keep; the Gaussian policies and the host clock runner, if you say so | about 370 |
| **Reduced in place** | the planner plots; fields, facades, inspection; the skins; Plotly, by your answer on Colab | about 3 000 to 4 000, an estimate, not a measure |
| **Kept as is** | everything a course names, the planning framework, compile, the catalog's teaching plants | |

The course and topic notebooks are in the teaching repository already
([repository-structure.md](repository-structure.md) §1).

The library would land near 73 000 lines from these rows alone. The 55 000 to 60 000
orientation of ROADMAP §5.4 is reached only by the cleaning pass of the large files
(`evaluator.py`, `problems.py`, `dp.py`, `arms.py`, `racecar_skin.py`, the renderers, the
plotting files), which is the API review's work, not a move. For scale: the plotting contract
rewrote every plot onto one description and left the library the same size (net +207 lines).

### 10.2 The internals, by kind (§8)

| Kind | Lines, about | Mostly at risk of |
| --- | --- | --- |
| Twins | 1 150 | behaviour, for the two large ones |
| Reporting extras | 1 500 | public methods and keywords |
| The large drawing code | 3 800 not blank in all; 1 240 if Plotly goes whole, the rest by cleaning | a second renderer; looks |
| Unused options and paths | 1 000 | public keywords |
| Leftovers | 490 | nothing |

The first, second, fourth and fifth rows sum to about 4 100 lines; the drawing code is of
another kind, since most of it is kept and cleaned, not removed. This is on top of the moves
of §10.1, and smaller than them: the internals are in better shape than the surface
around them.

## 11. A suggested order

Each step waits for the maintainer's choice of its rows.

1. **Tag a snapshot**, so every later move is reversible.
2. **The demo pass:** which demos stay (§4), and with them which features.
3. **Moves that touch no kept code** (priority 1, high confidence): the projects, the
   experimental package, the old demo folders the pass does not keep.
4. **The small reductions** (priority 1): the path-tracking laws, Plotly by the answer on
   Colab, value iteration on JAX under a kept demo.
5. **The internals that move no public name:** the leftovers (§8.5) and the small exact
   twins; the seeded baselines stay byte-identical, one commit per file.
6. **The reporting extras** (§8.2), once the maintainer says which pictures a course will
   show; the other drawing code (§8.3) when its file is opened.
7. **The API review**, module by module, with the test file of each module rewritten in the
   same step (priority 2 rows), and with it the unused options (§8.4), since each is a
   keyword.
8. **The two large twins** (the transcriptions, the step plan, §8.1): each is a refactor of
   its own with a baseline first, after the API review of that module.
9. **The tests' clean slate** (§9), once the demos are what the maintainer wants.
10. **Priority 3 rows**: only when a file is opened for another reason.

## 12. What I need from you

- **The planes:** does "learning to fly" in GRO860 use `Plane2D` or the drone?
- **Modal analysis:** will a course animate vibration modes?
- **Path-tracking laws:** all to the research repo, or pure pursuit stays as a teaching law?
- **Plotly:** do the Colab plots and animations of the courses rely on Plotly (the figure
  renderer, the animation renderer, the live plot of the games)?
- **Gaussian policies:** deleted, or kept as the notes' stochastic-policy object?
- **Symbolic mechanics:** research repo, or a future GMC714 lab feature?
- **The 3-D car:** stays in the catalog as a marked prototype, or moves with the research?
- **The reporting extras** (§8.2): which pictures will a course show (the search tree
  growing, the horizon over the car, the live trajectory-optimization plot)?
- **The demo set** (§4.3, §4.6, §4.7): the open rows of the thirteen, the split between
  tutorials and demos, and whether `racecar_mppi.py` replaces `racecar_mpc.py`.
- **The test layers** (§9.2): the four open layers (MS-D2a to MS-D2d) and when each runs
  (MS-D2e).
