# The minimal surface: the inputs of the end-of-v0.4 rebuild

**Status:** the plan's decisions are ruled (MS-D1 to MS-D7); what landed in v0.4 is the CI
shape (MS-D3), the short-term test restructure (the `slow` marker and `--runslow`), the
notebooks (MS-D5) and the benchmarks reduced to the regression gate (MS-D6). Left: the
maintainer's demo rebuild, then the tests' clean slate (MS-D1), the last steps of v0.4. This
doc keeps what those two need: the constraint, the test layers and their open options, and
the draft of the demo set (§4). Its decision ids carry the `MS-` prefix
(docs/plans/README.md). The row and its "done when": [TODO.md](TODO.md) §1.12.

## 1. The goal and the rulings

**The goal:** a surface designed from the core outward, small enough that the maintainer
knows every demo and every test layer, and a CI that answers in minutes.

- **MS-D1, tests (2026-11-07):** restructure on the short term, the net for v0.4's sweeps
  (landed); at the end of v0.4 the maintainer deletes all demos and rebuilds a minimalist set
  himself; once the new demos are what he wants, the test infrastructure is rebuilt from
  scratch around the new library.
- **MS-D4, the demo set (2026-11-07):** the maintainer's, decided in that rebuild (§4 is the
  draft he reviews).
- **MS-D5, notebooks (2026-11-07):** the course notebooks keep their paths, their content the
  maintainer's; the five unique topic pages stay until the repository-structure decision (K5,
  v0.9).

**The constraint:** the course PDFs link course notebooks by path (`Notes_OptiRL.pdf` 12,
`Notes_SystemesAsservis.pdf` 4, checked 2026-10-30). A deleted or renamed course notebook
breaks a student's link unless the PDF changes with it.

## 2. What the audit found (2026-10-31)

Kept for the clean slate:
- six test files check the repository, not the library: `test_textbook_style`,
  `test_repo_contract`, `test_public_names`, `test_teaching_imports`, `test_packaging`,
  `test_demo_check_runners`;
- the demos held pyro ports (six robotics, two control), one-concept step toys (six),
  duplicates (six value-iteration demos, five racecar MPCs, two hybrid sliding modes, the
  closed loop wired three times), and few demos written for the v0.3 core;
- about 40 tests made half of the suite's time.

## 3. The test layers (MS-D2), for the clean slate

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

## 4. The demo set (MS-D4): a draft for the maintainer's review

**Status:** a draft of 2026-11-12, written on the maintainer's go after he read the proposal;
the list is his, and each row is open until he rules it. It replaces the 19-row starting list
of 2026-10-31 (one demo per feature), which kept the shape this section argues against.

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
  19 tutorial notebooks already do, in the same order;
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
- **Demo 9 runs outside the 3-D car's stated validity** (hills of 0.8 m; vehicle-models.md
  §10.7): a flat course with the formula preset is the alternative.
- **The folder and the file names:** student-facing, not proposed here.
- The rows the 2026-10-31 list attached (a tabular Q-learning demo beside value iteration,
  the MPC hyperparameter-tuning demo) are not in the thirteen; they return only as a story.

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

"Out" is the maintainer's private repository, after the snapshot tag: nothing is deleted
without a copy. Before a tool's walk-through leaves the demos, its tutorial section is
checked to exist; the tutorials are reduced after the demos, not with them.

### 4.5 The tutorials beside the demos (reviewed 2026-11-12)

The 19 tutorial notebooks, as read on 2026-11-12, and what the maintainer ruled the same day:

| Layer | Notebooks | Code lines | What it is | Ruling |
| --- | --- | --- | --- | --- |
| `basics/` | 3 | 228 | story-led: a model, a loop, analysis, a plan | kept |
| `advanced/` | 5 | 470 | one topic each | kept |
| `core/` | 8 | 799 | the core objects listed method by method, written for the maintainer's API review | **kept until that review is done,** then they go: a stand-in for the documentation of the core classes, not teaching material |
| showcases | 3 | 670 | landing pages | **one showcase** |

- **The one showcase** is `showcase_minilink.ipynb`. A first rewrite (2026-11-12) opened on
  a 40-line hand-written pendulum class; the maintainer refused it the same day: a showcase
  shows how little it takes to start, and writing a class cleanly is a tutorial's job. The
  second rewrite mirrors the README, section for section, with code that runs: ten lines on
  the catalog pendulum; diagrams as code, the loop then on a clock; a short model of one's
  own; one `f` differentiated (Jacobians, LQR, a batch of rollouts); one problem for three
  planners and an MPC; the one optimization boundary (a program, two optimizers); a law
  learned by reinforcement and scored; the same lines on the UR5. 40 cells, 218 code
  lines, about a minute. The pendulum family's 2-D look was redrawn for it (a dark rod, a
  blue bob, a hub on a grey mount).
- **`showcase_jax` and `showcase_from_rl_to_bode` are deleted** (the maintainer,
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
  (one demo, a 910-line module), `TransferFunction`, `Lead`, `step_response`,
  `find_equilibrium`, `discretize`, `QuadraticField`, `JointImpedance`, `IntegralImpedance`,
  `LowPassFilter`.
- **Found while writing the showcase:**
  - `array_module`, the one line a hand-written `f` needs to run on both backends, has no
    public import: the showcase and five demos reach `minilink.core.backends` through the
    import allowlist;
  - the comparison table of `advanced/02_optimal_control` reports value iteration as not
    converged (3000 sweeps, the change still 20) and three of its four answers as missing
    the goal on the judged run, on the catalog pendulum as on a hand-written one.

### 4.6 The tutorials made ready and the demos drafted (2026-11-12)

On the maintainer's go, in one run:

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

- **Two MPCs on the car, each on the model that suits it** (the maintainer's idea):
  - *MPPI on the kinematic bicycle, with the inner PI on the speed.* Sampling needs many
    cheap rollouts and no gradient, which a three-state model gives; it handles a cost that
    is not smooth (cones, lane edges) without the shaping a gradient method needs. It
    would replace the collocation planner of `racecar_mpc.py` in the same cascade.
  - *SQP on the dynamic model for fast cornering.* Near the limit of grip the plan must
    know the tires, and a gradient method on the dynamic model with exact derivatives is
    the tool: few iterations, a warm start, constraints held. Its starting point is
    `udes_racecar/mpc_racecar_dynamic.py`, which today neither completes its lap nor stays
    in its lane (§4.6), with fewer spatial fields.

  The pair tells one story: the planner follows the model, sampling where the model is
  cheap and the cost rough, gradients where the model is rich and the cost smooth. To
  check when drafting: that the MPPI lap is as clean as the collocation one, and that the
  dynamic lap is fast enough to show what the kinematic plan cannot do.
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
