# The minimal surface: demos, notebooks, tests and CI redrafted on the v0.3 core

**Status:** draft for the maintainer's review. Nothing here is decided; each choice below
lists its options, the agent's recommendation, and an **Answer** line for the maintainer.
**Proposed release:** the new v0.4 ("the minimal surface"); the features planned as v0.4
(ROADMAP §5.4) move to v0.5.

## 1. Why

v0.3 settled the core. What surrounds it grew around older APIs:

| | Today (`main`, 0.3.0) |
| --- | --- |
| Library (`minilink/`) | 76k lines |
| Unit tests (`tests/unittest/`) | 67 files, 33k lines, 1 775 test functions (2 161 cases), 18 min locally with the full stack |
| Test tooling (`tests/run/`, `tests/demo_checks/`, fixtures) | two layers of runner scripts, ~1 500 lines, 4 fixture sets |
| Benchmarks (`benchmarks/`) | 46 files, 30k lines (gates, studies, pyro parity) |
| Demos (`examples/demos/`) | 86 scripts in 17 folders |
| Projects (`examples/projects/`) | 29 scripts, 2 notebooks (car trajopt, MPC, path tracking) |
| Notebooks | 15 tutorial, 7 topics, 20 course (GRO501 6, GRO860 14) |
| CI per run (`test.yml`) | 45–60 min; 2–3 runs per change (push and pull-request events) |

**The goal:** a surface designed from the core outward, small enough that the maintainer
knows every demo and every test layer, and a CI that answers in minutes.

## 2. What the audit found

**CI** (`.github/workflows/test.yml`):
- Four Python versions plus macOS and Windows run pytest without JAX (10–15 min); about 450
  tests skip there, the compiled and differentiable core among them.
- A `regression` job runs after them: pytest again with JAX, the regression gates, the
  flagship demos, the notebooks (30–45 min). The core is only tested here.
- A push to a branch with an open pull request runs CI twice; a fix that crosses
  `fix → dev → main` runs it three times.
- The slowest test (`test_flagship_demos_exit_zero`, 167 s) runs the flagship demos the
  `regression` job runs again. About 40 tests make half of the suite's time.

**Tests:** six files check the repository, not the library: `test_textbook_style`,
`test_repo_contract`, `test_public_names`, `test_teaching_imports`, `test_packaging`,
`test_demo_check_runners`.

**Demos:** pyro ports (six robotics, two control), one-concept step toys (six), duplicates
(six value-iteration demos, five racecar MPCs, two hybrid sliding modes, the closed loop
wired three times), and few demos written for the v0.3 core.

**Constraint:** the course PDFs link course notebooks by path: `Notes_OptiRL.pdf` 12,
`Notes_SystemesAsservis.pdf` 4 (checked 2026-10-30, all run on 0.3.0). A deleted or renamed
course notebook breaks a student's link unless the PDF changes with it.

## 3. The decisions

### D1. Tests: rewrite from scratch, or restructure

- **A. Clean slate.** Delete `tests/`, write a new minimal suite from the core's contracts.
  Simplest result; loses the executable record of every behaviour v0.3 settled while the
  demos are also being rebuilt, so a regression has no net for the length of the rewrite.
- **B. Restructure.** Keep the cases; sort every file into the layers of D2; delete the
  tests of what is deleted (demos, pyro ports, benchmark studies); merge duplicates; move
  the slow tests out of the per-change layer.
- **C. Hybrid.** Write the new layers from scratch (checks, unit, acceptance), port into them
  only the cases a coverage diff shows uncovered, then delete the old suite.

*Agent's recommendation:* B, or C if the maintainer wants to read every test that stays.
**Answer:**

### D2. The test layers

**Taken (the maintainer, 2026-10-31):** the layers are cut by what they prove, one folder
per layer; two are essential, the contract and the textbook math. What the other kinds
become is open below.

| Layer | The question it answers | Status |
| --- | --- | --- |
| `tests/contract/` | Does every public object build, wire and run: each catalog plant, block, law, planner, solver, its ports and its `params`? | **essential** |
| `tests/math/` | Are the numbers right against a textbook worked example or a closed form: Riccati and LQR gains, an RK4 convergence order, a known optimum, a value-iteration fixed point, a Kalman covariance? | **essential** |
| Backend parity | Does NumPy equal JAX, and does a gradient exist through a loop, a planner, a solver? | open (D2a) |
| Composition | Do the pieces work together: diagrams, sampled loops, problem → evaluator → optimizer? | open (D2b) |
| Examples | Do the kept demos and the course notebooks run? | open (D2c) |
| Repo rules | Textbook style, prose, public names, imports, packaging | open (D2d) |

- **D2a. Backend parity.** A. one clause of the contract (each object also runs on JAX, its
  outputs equal to NumPy's on one input); B. its own layer; C. none beyond what the math
  layer runs on both backends. *Agent:* A. **Answer:**
- **D2b. Composition.** A. the core's end-to-end scripts only (`test_v03_core.py`, a dozen
  cases) as a third layer; B. inside the contract (a composition is a public object);
  C. none, the demos show it. *Agent:* A. **Answer:**
- **D2c. Examples.** A. a runner, outside pytest, nightly and before a release; B. inside
  pytest; C. none in CI, the maintainer runs them. *Agent:* A. **Answer:**
- **D2d. Repo rules.** A. a pre-commit and CI check outside pytest (seconds); B. a test
  layer; C. drop the ones nobody acts on. *Agent:* A, with C for any rule the maintainer
  does not want enforced. **Answer:**
- **D2e. When each runs.** *Agent's proposal:* contract and repo rules on every change, math
  and composition on every pull request, examples nightly and before a release. **Answer:**

### D3. CI shape

- **A.** One Linux job (Python 3.12, full stack) runs Checks, Unit and Acceptance on each
  pull request; pushes to a branch with a pull request do not run again; a newer push
  cancels the older run (`concurrency: cancel-in-progress`); the rest is nightly.
- **B.** As A, plus macOS and Windows on pull requests into `main` only.
- **C.** Keep the matrix on every run; only remove the duplicates.

*Agent's recommendation:* B. **Answer:**

### D4. Demos: what the set is

- **A. Clean slate.** Delete `examples/demos/` and `examples/projects/`; write a new set,
  one demo per core feature, against the v0.3 core.
- **B. Curate.** Keep the demos that already show one core feature well, delete the rest.
- **C. Clean slate for demos, keep projects** as the maintainer's research code, outside
  the demo checks.

A starting list for A or B (one demo per feature, 20; ✚ new, ✓ kept as is or redrafted):

| Feature | Demo |
| --- | --- |
| A system and its simulation | ✓ `core/diagram_closed_loop.py` (redrafted: a plant, a law, `@`) |
| Composition | ✓ `core/diagram_nested_loop.py` (cascade) |
| Analysis | ✓ `analysis/analysis_linearize.py`, ✓ `analysis/analysis_frequency.py` |
| Classical control | ✓ `control/classical_family.py` |
| State feedback and estimation | ✓ `control/lqg_cartpole.py` |
| Robotics | ✓ `robotic/task_impedance_two_link.py` |
| Sampled loop | ✓ `compile/multi_rate_computer.py` |
| Noise and Monte Carlo | ✓ `compile/lqg_monte_carlo.py` |
| Value iteration | ✓ `value_iteration/vi_pendulum_lqr.py` |
| Trajectory optimization | ✓ `trajopt/trajopt_cartpole_collocation_jax.py` |
| RRT | ✓ `rrt/rrt_pendulum_swingup.py` |
| MPC | ✓ `mpc/mpc_car_minimal.py` |
| MPPI | ✓ `mpc/mppi_pendulum.py` |
| Reinforcement learning | ✓ `rl/pendulum_swing_up_rl.py` |
| Gradients through a loop | ✓ `compile/digital_pid_tuning.py` |
| The core in one script | ✓ `compile/differentiable_double_loop.py` |
| Optimization | ✓ `optimization/optim_basic.py` |
| Animation | ✓ `graphical/animation_renderers.py` |
| Real-time | ✓ `realtime/game_cartpole.py` |

**Answer:** (the option, and edits to the list)

### D5. Notebooks

- **Course notebooks** (`examples/teaching/courses/`, 20): keep their paths (the PDFs link
  16); their content is the maintainer's teaching design. **Answer:**
- **Tutorial** (`examples/tutorial/`, 12 chapters and 3 showcases):
  - A. redraft the chapters in place on the v0.3 core, delete the three showcases;
  - B. delete, write a new shorter tutorial;
  - C. keep as is.
  **Answer:**
- **Topics** (`examples/teaching/topics/`, 7): A. fold into the tutorial or the courses and
  delete; B. keep. **Answer:**

### D6. Benchmarks

- **A.** Keep the regression gates the nightly layer runs (`run_regression_check.py`, its
  suites and baselines); delete the studies, the pyro parity and the one-off baselines
  (`step_model_baseline.py`, `v03_baseline.py`, `v03_budget.py`).
- **B.** Delete `benchmarks/` entirely; speed is watched by the acceptance layer's budget.
- **C.** Keep as is.

*Agent's recommendation:* A. **Answer:**

### D7. The release numbering

The new v0.4 is this plan; ROADMAP §5.4's features become v0.5, v0.9 and v1.0 unchanged.
ROADMAP §1 is the maintainer's. **Answer:**

## 4. Order, once the decisions are taken

1. CI first (D3): the fastest relief, independent of the rest.
2. Tests (D1, D2): the layers, then the moves and deletions, file by file.
3. Demos and projects (D4), then their checks.
4. Notebooks (D5), the course ones last and only with the maintainer.
5. Benchmarks (D6), docs (README, `examples/README.md`, `tests/README.md`, DESIGN's package
   map), the release.

## 5. The test files, by layer

To be written once D1 and D2a–D2e are answered: each of the 67 files of `tests/unittest/`
sorted case by case into contract, math, the open layers, or deletion. The obvious ones:
**repo rules** `test_textbook_style`, `test_repo_contract`, `test_public_names`,
`test_public_imports`, `test_teaching_imports`, `test_teaching_surface`, `test_packaging`;
**composition** `test_v03_core`, `test_loop_grammar`, `test_sampled_loop`; **contract**
`test_contract`, `test_catalog_backends`, `test_blocks`; **deleted with their subject**
`test_benchmark_helpers` (D6), `test_demo_check_runners` (D2c), the pyro-port cases inside
`test_mechanical_robotics` and `test_dynamics_catalog`. The ~40 slowest cases (over 4 s,
`pytest --durations=40`) are flagged in the sort.
