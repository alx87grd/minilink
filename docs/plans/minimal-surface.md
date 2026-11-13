# The minimal surface: the inputs of the end-of-v0.4 rebuild

**Status:** the plan's decisions are ruled (MS-D1 to MS-D7); what landed in v0.4 is the CI
shape (MS-D3), the short-term test restructure (the `slow` marker and `--runslow`), the
notebooks (MS-D5) and the benchmarks reduced to the regression gate (MS-D6). Left: the
maintainer's demo rebuild, then the tests' clean slate (MS-D1), the last steps of v0.4. This
doc keeps what those two need: the constraint, the test layers and their open options, and a
starting list for the demo set. Its decision ids carry the `MS-` prefix
(docs/plans/README.md). The row and its "done when": [TODO.md](TODO.md) §1.12.

## 1. The goal and the rulings

**The goal:** a surface designed from the core outward, small enough that the maintainer
knows every demo and every test layer, and a CI that answers in minutes.

- **MS-D1, tests (2026-11-07):** restructure on the short term, the net for v0.4's sweeps
  (landed); at the end of v0.4 the maintainer deletes all demos and rebuilds a minimalist set
  himself; once the new demos are what he wants, the test infrastructure is rebuilt from
  scratch around the new library.
- **MS-D4, the demo set (2026-11-07):** the maintainer's, decided in that rebuild (§4 is a
  starting list).
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

## 4. A starting list for the demo set (MS-D4)

One demo per core feature, 19 rows, each an existing script (a clean slate or a curated set
are both open; the maintainer decides in the rebuild):

| Feature | Demo |
| --- | --- |
| A system and its simulation | `core/diagram_closed_loop.py` (a plant, a law, `@`) |
| Composition | `core/diagram_nested_loop.py` (cascade) |
| Analysis | `analysis/analysis_linearize.py`, `analysis/analysis_frequency.py` |
| Classical control | `control/classical_family.py` |
| State feedback and estimation | `control/lqg_cartpole.py` |
| Robotics | `robotic/task_impedance_two_link.py` |
| Sampled loop | `compile/multi_rate_computer.py` |
| Noise and Monte Carlo | `compile/lqg_monte_carlo.py` |
| Value iteration | `value_iteration/vi_pendulum_lqr.py` |
| Trajectory optimization | `trajopt/trajopt_cartpole_collocation_jax.py` |
| RRT | `rrt/rrt_pendulum_swingup.py` |
| MPC | `mpc/mpc_car_minimal.py` |
| MPPI | `mpc/mppi_pendulum.py` |
| Reinforcement learning | `rl/pendulum_swing_up_rl.py` |
| Gradients through a loop | `compile/digital_pid_tuning.py` |
| The core in one script | `compile/differentiable_double_loop.py` |
| Optimization | `optimization/optim_basic.py` |
| Animation | `graphical/animation_renderers.py` |
| Real-time | `realtime/game_cartpole.py` |

Decided with the set (TODO §1.12): a tabular Q-learning demo beside value iteration, the MPC
hyperparameter-tuning demo, the cart-pole RL demo's budget; and, drawn from it, the supported
plants (K8-3) and GMC714's pinned names (G3).
