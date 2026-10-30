# The merge-review run (2026-10-31): the log

**Audience:** Prof. Alexandre Girard (Maintainer), for the next conversation.
**Context:** the run that implements [the merge-review plan](../plans/merge-review-run.md)
(rulings R180–R187 in [the record](2026-10-30-merge-review-rulings.md)), on
`dev-v0.3-merge-review` off `dev-v0.3` (`f51dcad7`), in a cloud session. Where a review would
have been asked, the decision is taken as the maintainer would (CONSTITUTION's precedence,
then the rulings, then the smaller reversible option), written in §2 to be raised with him,
and the run continues.

**How to read it.** §1 is the summary. §2 lists every decision taken without the maintainer
(M-1…), each with its context, the choice, the alternative and the commit to revert. §3 is the
ledger of moved numbers. §4 is the textbook-check statement. §5 is what was not done. §6 is
the progress record, one line per pushed row. §7 is row 1's sweep (the automatic step before
and after).

## 1. Summary

**Every row landed or is explained.** Rows 1–7, 9, 10 and 12 landed, one commit each (row 1
has a second, the one draw per step; a last commit shrinks the teaching-import allowlist after
rows 5 and 6); rows 8 and 11 needed no change and are explained in §5; row 13 is this record.
**Every gate is green** on the branch's head: ruff on the whole repo; "Cross-cutting or
handoff" (`pytest`: 2160 passed, 1 failed, 15 skipped, the skips for Graphviz and the SMC
adaptive smoke; the failure was the teaching-import allowlist's three stale entries after rows
5 and 6, removed in `81a62bf0`, `test_teaching_imports.py` then green); "Regression gates, CI flags" (all
passed, no gate moved); the notebook smoke (33 passed, 1 skipped); the docs build with `-W`.
The v0.3 baseline moved once, on purpose: `noise.white.x`, row 1's RK4 fix (§3); every other
row kept it byte-identical. `dev-v0.3-merge-review` is merged into `dev-v0.3` with `--no-ff`.

**To raise first** (ruled 2026-10-31, R188–R193, [the record](2026-10-31-release-review-rulings.md); R188 removed M-1's fix): M-7 (`SlidingMode` declares its surface, so the SMC demo steps Euler at
5 ms), M-8 (the kinematic bicycle declares no `τ` and keeps 1 ms), M-4 (the PID family and the
integrator counted as linear blocks), M-13 (the neural demo on `gd`, L-BFGS overfits), M-1
(how the held time reaches the held blocks, and where it does not: KNOWN_ISSUES), and §7's
GRO501 cart-pole notebook, now at the 1 ms fallback for its undeclared custom block.

The environment: a cloud container with no conda; the `dev` and `full` extras installed by
pip (JAX 0.12, NumPy 2.5, Python 3.13), no Stable-Baselines3 or torch, no Graphviz; for row
12 the system's Ipopt (3.11, apt) and `cyipopt` were installed, so the end gates ran the Ipopt
tests too. The v0.3
baseline (`benchmarks/v03_baseline.py`) was captured twice before the first edit and the two
files were byte-identical (116 entries).

## 2. Decisions taken without the maintainer

Each decision names its row; the commit to revert is the row's in §6 (row 1's are `39563f6e`
and `e26d9df1`).

- **M-1 (row 1) How a held block is read once per step.** The twin test showed the bias:
  `NoiseSource → LTISystem`, RK4 at `Δ`, `Δ/3`, `Δ/10`, `Δ/30` against `Δ/200`: errors
  1.65e-3, 5.4e-4, 1.6e-4, 4.7e-5, first order. The fix threads the step's start to the
  held blocks of the compiled diagram, not to `f`: the diagram evaluator marks the port
  operations of a leaf whose `solver_info` has a `sample_period`, `rhs_of(params, t_held)`
  gives the right-hand side with those blocks read at `t_held`, and `held_step_body`
  (`core/compile/evaluators/stepping.py`) runs each substep with the held blocks read at its
  start; the evaluator's `step`, `rollout`, `rollout_with_cost` and the simulator's
  `integrate_input` call it when the diagram holds a signal (`evaluator.holds`), and the code
  path of every other system is the old one, so its numbers are byte-identical. After the fix
  the same errors are 1.5e-8, 1.8e-10, 1.5e-12, 1.8e-14: fourth order. A held block that reads
  no input and has no state (a noise source) is then evaluated once at the step's start
  (`held_at`) and its sample reused by the four stages: one draw per step instead of four, the
  same numbers to the bit (`e26d9df1`). `f(x, u, t, params)`
  keeps its one signature (CONSTITUTION §2). A held block inside a nested sub-diagram still
  reads the stage time (`flatten=True` reaches it); a sampled loop's plant view and the step
  face call the plant's own `f` and are not touched (no demo holds a noise source inside a
  plant). *Alternative:* every stepping right-hand side taking the step's start as a fourth
  argument (a dozen call sites, the trajectory-optimization lambdas among them), or a last
  stage nudged below `t + h` (not exact, and it moves every time-varying RK4 run).
- **M-2 (row 1) One commit for the row.** The plan's step 2 says "the twin test and the RK4
  held-sample fix, alone, with its ledger"; the session's rule is one commit per row. The fix
  was measured alone (the baseline after it and before the rule: only `noise.white.x` moved)
  and has its own ledger line; it shares row 1's commit (and its speed-up, the one draw per
  step, came as the row's second commit, after the end gates' timing). *Alternative:* two commits for the
  row, as the consolidation's step 2.
- **M-3 (row 1) "Divides every hold period".** A diagram's `sample_period` is now the base
  period of its blocks' periods (their greatest common divisor, `base_period_of`, the sampled
  diagrams' rule), not their minimum; one period is kept as is, so no existing loop moves;
  distinct periods off the nanosecond grid are refused as the sampled diagrams refuse them.
  *Alternative:* keep the minimum (a slower source's switch could fall inside a step).
- **M-4 (row 1) Which linear blocks.** Beyond the ruling's list (the LTI blocks,
  `StateObserver`, so `kalman` and `luenberger`, the transfer functions through `LTISystem`):
  `StateSpaceSystem` (the property sits there, so `LTISystem`, `TransferFunction`,
  `LowPassFilter` and the catalog's mass-spring-dampers inherit it, `A` read at `t = 0`), the
  `Integrator` (a pole at the origin: `∞`), and the PID family (its poles: the integrators at
  0, each derivative filter at `−1/τ`; a `PI` is `∞`). Without the PID, every PID loop on a
  declared plant would stay pinned at the 1 ms default. *Alternative:* the ruling's list only.
- **M-5 (row 1) "Declares nothing" is `∞`.** `fastest_time_constant` returns `∞` for an
  all-zero spectrum; the diagram's minimum leaves a non-finite value out, and a system with no
  finite time constant at all takes the 1 ms default (`automatic_step_of`). So an integrator
  bounds nothing in a loop, and alone it steps as an undeclared plant. *Alternative:* the
  integrator left at the 1 ms default like an undeclared block (it would pin every loop that
  holds one).
- **M-6 (row 1) The physics wins over a hand-set hint.** A block that computes its own time
  constant (the racecar's pattern, now every linear block and the declared plants) overwrites
  a value set by hand in its `solver_info` at the next read; `dt=` at the run and `substeps=`
  at the compile are the overrides. Three test files set a Pendulum's hint by hand; they now
  use a test plant whose `fastest_mode()` returns the value. *Alternative:* a hand-set value
  wins (needs a marker of who wrote the key).
- **M-7 (row 1) `SlidingMode` declares its surface.** The SMC demo's loop is discontinuous,
  so it steps Euler at the automatic step; with the pendulum's `τ` (0.45 s) that is 45 ms on
  a sliding surface of slope `λ = 20 s⁻¹`. The law now declares `τ = 1 / max λᵢ` (a static
  block that sets its own counts): Euler at 5 ms instead of today's 0.1 ms. *Alternative:*
  leave the law undeclared, or give the demo `dt=` (a demo edit the row does not ask for).
- **M-8 (row 1) The kinematic bicycle declares nothing.** `KinematicBicycle` and
  `UdeSRacecar` are a pure kinematic chain (`θ̇ = v tan δ / L`): their linearization has a zero
  spectrum and their inputs no bound by default, so they have no time constant; their loops
  keep the 1 ms default and their numbers. *Alternative:* `τ = L / (v_max tan δ_max)` from an
  input box the plant does not declare.
- **M-9 (row 1) The dynamic bicycle's `τ`.** `DynamicBicycle.fastest_mode()` (inherited by
  `BicycleDynRate` and the 3-D car) is `1 / max |λᵢ|` of the body's exact 3 × 3 linearization
  at rest, where the slips sit on their `v_min_epsilon` floor: 1.25 ms for the default
  numbers, not halved as the racecar's estimate is (the racecar halves three one-degree
  estimates of coupled modes; here the coupled matrix is solved). *Alternative:* the racecar's
  halving (0.6 ms, every bicycle loop twice as slow as today).
- **M-10 (row 1) One rounding.** `automatic_step_of` rounds `⌈T_s / Δt⌉` with the substeps'
  `10⁻⁹` tolerance (`steps_per_period`), so the continuous grid and a loop's substeps agree
  when a period is an exact multiple of the step. *Alternative:* the old bare ceiling in the
  simulator (one step more than the loop at an exact multiple).
- **M-11 (row 2) `planner_evidence` keeps `backend=`.** The plan's signature has no
  `backend`; the RL planner's evidence runs on JAX (`backend="jax"`), so the keyword stays
  (default `"numpy"`) and the baseline stays byte-identical. *Alternative:* the plan's
  signature, the RL evidence on NumPy.
- **M-12 (row 4) What "a finite `U`" is.** `constrains_inputs(U)`: a finite bound on some
  axis of `U.bounding_box()`, or a set that is no box; the default unconstrained box is not.
  *Alternative:* every axis finite (`is_finite_box`).

- **M-13 (row 5) The neural controller descends by `gd`.** L-BFGS (`"l-bfgs"`) reaches a
  training loss of 1.5e-5 against the hand loop's 1.08e-3, but the network overfits the six
  training pairs: the demo's last run, from `y = 2` to `r = 0` (not a training pair), ends at
  `y = 0.17` instead of 0.0033. `Optimizer(..., method="gd", iterations=10000,
  options={"learning_rate": 0.8})`, the demo's own rate and count, lands on the hand loop's
  numbers to the printed digit (loss 1.075e-3, `y` 0.003286). *Alternative:* L-BFGS and a new
  last run (a demo whose figure no longer shows the story).
- **M-14 (row 5) The cart-pole schedule demo: one figure, the bounds as the program's box.**
  The hand loop's `jnp.clip` becomes the program's `lower`/`upper`; L-BFGS reaches (a)
  `J` 1.853e-2 (the hand loop 1.873e-2) and (b) 6.21e-3 (1.791e-2): the same swing-up, a lower
  cost. `print_result`, `make_trajectory` and `plant_from_params` go: the optimizer's result
  prints itself, the co-designed run is the compiled evaluator's `rollout` on the found
  numbers, plotted (and animated under the demo's `ANIMATE`) once; (a)'s run is printed, not
  drawn. *Alternative:* keep both figures (the ruling says one at most).
- **M-15 (row 5) The neural demo down to one figure.** The diagram's picture, the untrained
  run and its plot, and the before/after maps of the law (`surface_max` and a hand-laid
  matplotlib figure) go; the trained loop's `plot_trajectory` stays. `params_gradient`'s
  first part (the finite-difference table of `df/dp`) is not a descent loop and stays as it
  was. *Alternative:* keep the law's maps as a quarantined side cell (RULES 6.11).
- **M-16 (row 6) The closing example's hand loop descends the problem's own cost.** The
  problem, its compiled evaluator and its program come first; the hand loop is
  `z ← z − η ∇J(z)` with `∇J = jax.grad(program.J)`, then `Optimizer(method="gd")` from the
  same start lands on the same gains (an `assert_allclose`, `rtol` 1e-6, instead of a printed
  gap), and `method="auto"` finishes (L-BFGS, `J` 0.05538 as before). The live plot and its
  three `PLOT_*` constants go with their helpers; `N_STEPS` now sets the horizon
  (`tf=N_STEPS * DT`, the same 10 s). 224 lines to 81. *Alternative:* keep the hand-written
  rollout and trapezoid as the hand loop's loss (then the two sections minimize two costs).
- **M-17 (row 10) The map's new layer and the README's one sentence.** The composition
  inset that sat inside the `System` card moves to the first origin card, top left, where the
  map's closed-loop arrow now comes back in (the maintainer's request after the merge); the `System` card
  keeps its definition, centred. `docs/pipelines.md` keeps "The library on one page" (its text
  now names the three origins) and loses the other sections, its "Not drawn yet" list
  among them. README's sentence "the pipelines behind them are drawn one by one in
  docs/pipelines.md" was no longer true: it reads "the map is read in docs/pipelines.md", and
  the image's alt text names the new layer (a ruled change's direct consequence, two lines).
  The PNG was rendered by Node's Playwright with `svg_to_png`'s own steps (scale 2, a clear
  background): Python's Playwright is not installed here. *Alternative:* leave README's
  sentence for the maintainer.

## 3. Ledger of moved numbers

| Row | What moved | By how much | Why |
| --- | --- | --- | --- |
| 1, the RK4 held sample | `noise.white.x` of the v0.3 baseline; re-pinned in `benchmarks/baselines/v03_core.json` (that key only) | 3.9e-3 relative | the four RK4 stages read one sample: the last no longer reads the next draw (M-1) |
| 1, the step rule | no v0.3 baseline entry and no regression gate (every one names its step); the demos and notebooks of §7 | §7 | R180 |
| 1, the showcase's `(ctl % 0.1) @ plant` (triage 3.6) | the pendulum (`l = 5`) now at 2 RK4 steps a period (`τ` 0.73 s), before 1000 (0.1 ms), at first 1 | `x(8 s)` within 4.1e-6 of the 0.1 ms run (one step a period: 6.6e-5) | R180: the plant sets the step |
| 1, `lqg_cartpole` | 2400 steps of 3.3 ms instead of 800 of 10 ms (the Kalman observer's pole at −23 s⁻¹); the final state moves up to 6e-3 on the cart's speed | wall time 1.7–1.9× today's (0.81–0.88 s against 0.47–0.49 s, interleaved, this VM) | R180, and RK4 reading the noise once per step |
| 1, `sliding_mode_pendulum` | Euler at 5 ms (`SlidingMode`'s `1/λ`) instead of 0.1 ms; the final rate 1.1e-2 rad/s instead of 2e-4 (chattering on the surface, its band `∝ Δt`) | 12 s to 2 s | M-7 |
| 5, the cart-pole schedule | (a) `J` 1.873e-2 → 1.853e-2; (b) 1.791e-2 → 6.21e-3 | L-BFGS reaches a lower cost from the same pump | R186 |
| 5, `params_gradient`, `neural_controller_jax` | none (gravity 9.81 and damping 0.20; the loss 1.075e-3 and the final `y` 0.003286 to the printed digit) | — | R186 |
| 6, the closing example | section 1 now descends the problem's cost (`J` 0.07013 at its gains (6.683, 9.670, 3.735), the old loop's own trapezoid gave the same gains to 1e-3); section 2 unchanged (`J` 0.05538) | — | R186 |
| 12, the Ipopt test | its reference: the analytic optimum `u = 0` instead of SLSQP's early stop | — | the guess toward the cost's `xbar` (R177) |

## 4. Textbook check

Every math line of each row was read beside `planning/dynamic_programming/dp.py` (RULES §5:
5.3's blocks, 5.8's sections, 5.22's comments, 5.24's timeless prose). Row 1: `automatic_step_of`
and `fastest_time_constant` (`core/system.py`), each plant's `fastest_mode()` (bindings, one
parameter per line, the linearization a named matrix assembled row by row, a named result
returned), the `solver_info` properties (the racecar's pattern), `held_step_body`
(`step_body`'s substep loop, each step named), `held_at` (machinery, under the evaluator's
section). Row 2: `planner_evidence` under `Internal machinery`, no underscore (5.8). Row 4:
`constrains_inputs` a named boolean, the record's text built from named parts. Row 7: the
one-link arm's five methods in the three beats (bindings, parameters one per line, a named
result; `# d = b q̇` and `# J = ∂p/∂q` kept, the copies dropped). Row 6: the hand loop
`z = z - LR * grad_J(z)` under its textbook comment `z ← z − η ∇J(z)`. Row 12: the test's
optimum under its equation comment. No equation sits on a `return` line or in a helper away
from its page; no docstring names the design process (5.24).

## 5. Not done

- **Row 8, the MPC notebooks' prose: nothing left to delete.** Every markdown cell of
  `racecar_toward_mpc` §7–§8, the showcase's §8 and `mpc_spatial_stack` was read: each says
  what a cell builds, computes or draws ("the 2-D animation shows the lane, the cones, and the
  current planned horizon"); none says how the car behaves in a run. The narration RD-63
  quoted ("the first lap holds… the car spins") left on 2026-10-29 (`7c21f63f`, "no narration
  of the run's details in §9"). No commit; no code cell touched.
- **Row 11, the removed `x_goal` lines: checked, no fix.** `git log -S x_goal` on `examples/`:
  `fcd1da56` (C-18) removed it from `readme_examples.py`, `rrt_holonomic_obstacles.py`,
  `rrt_pendulum_swingup.py`, `rrt_star_live.py`, `trajopt_holonomic_corridor.py`, the five
  value-iteration demos, eight GRO860 and topic notebooks and the showcase. Each now reads the
  same point through `problem.goal_point()` (`Xf`'s centre for the three RRT demos and the
  corridor; the cost's `xbar` for the rest, `TimeCost`'s zero included), except GRO860's
  `pendulum_value_iteration.ipynb`: its hand-written cost keeps its target as
  `cost.x_target`, not a `params["xbar"]`, so its goal point is now the start. Nothing reads
  it there (a 2-state grid: the plot anchor pins no axis, value iteration needs no guess), so
  the notebook's numbers do not move; a fix would change the course's cost class, the
  maintainer's teaching design.
- **Row 1's rest of the catalog** (a TODO row now): the plants the demos and notebooks do not
  run on the automatic step keep their old hint; the kinematic bicycle declares none (M-8).
- **A held block where the held time does not reach** (KNOWN_ISSUES): inside a nested
  sub-diagram, a sampled loop's plant view, a step face (M-1).
- **`examples/tutorial/07_compile.ipynb`'s and the showcase's hand gradient loops** are
  notebooks, the maintainer's teaching design: unchanged.
- **The maintainer's local edits to `pid_autotuning_jax.py`** (2026-10-30) were not in the
  branch: the file's last commit is `ac0b5372` (2026-10-25); row 6 rewrote the committed
  version.
- **The diagram-shortcuts label** keeps "`WhiteNoise() -> pendulum.n`" beside the fixed
  `NoisyPendulum()`; the wire goes to the block `plant` (outside the row's one fix).
- **`docs/make_assets.py map`** needs Python's Playwright, absent here; the PNG was rendered
  by Node's with the same steps (M-17). Rebuilding it on the maintainer's machine gives the
  canonical file.

## 6. Progress

| Row | Commit | What landed | Gates of the row |
| --- | --- | --- | --- |
| 1 | `39563f6e`, `e26d9df1` | the step rule from the physics; the held sample read once per step, one draw | ruff; `test_simulation`, `test_sampled_loop`, `test_v03_core`, `test_racecar_plant`, `test_compile`, the noise and stochastic tests; the baseline (`noise.white.x` only, re-pinned); the sweep (§7) |
| 2 | `2b27625a` | `planner_evidence` | ruff; `test_dynamic_programming`, `test_planning`, `test_rl_tabular`; baseline byte-identical |
| 3 | `ce76d365` | the step-problem names off the root | ruff; `test_teaching_surface`, `test_teaching_imports`; the grid-world demo |
| 4 | `5f1e1e3e` | LQR's "U ignored" | ruff; `test_lqr_planner` |
| 5 | `154072c4` | three demos on `Optimizer` | ruff; each demo headless, its asserts |
| 6 | `dfe29abe` | the closing example | ruff; the demo headless, its assert |
| 7 | `6e17b47a` | two demos' style | both demos headless; the arm's numbers bit-identical |
| 8 | — | nothing to delete (§5) | — |
| 9 | `609452c8` | ROADMAP §4.1's RL row | — |
| 10 | `0536ae37` | the map, its new layer, the other figures gone | the docs build `-W` |
| 11 | — | each removal checked (§5) | — |
| 12 | `33dcd288` | the Ipopt test pinned to the optimum | `test_mpc` with Ipopt (cyipopt on the system's Ipopt 3.11) |
| — | `81a62bf0` | the allowlist's stale entries | `test_teaching_imports` |
| 13 | this record's commit | the rulings record's commit lines, the triage's §6, TODO, ROADMAP §3, KNOWN_ISSUES, `v03_budget.py`'s 5b comment, this log | the end gates (§1) |

## 7. Row 1's sweep: the automatic step, before and after

Every demo, project and notebook was run with an out-of-tree instrumentation that records each
automatic-step decision (a fixed-step simulation with no `dt`, a sampled loop's plant with no
`substeps`, the planners' evidence and `problem.compile(law % dt)` through the loop), on the
base tree. 22 files hit it; the v0.3 baseline and the regression suite never do (every entry
names its step). Each file was then run on the base tree and on row 1's tree (before its
second commit, the one draw per step), interleaved, two at a time. The columns: the automatic
decisions the run took (a simulation's `dt`, a loop plant's substeps `n` per period `T`), the
wall time of the whole file in seconds, the status, and the final state of the run's largest
recorded trajectory (four entries at most). **The times are noisy**: two files ran at once,
and from the MPC rows on the full test suite ran beside them; the step counts are the reliable
figure, and the clean, interleaved re-times follow the table.

| File | Before | After | Time (s) | Status | Largest run's final state, before → after |
| --- | --- | --- | --- | --- | --- |
| `demos/analysis/analysis_digital_pid_poles.py` | Pendulum T=0.01: n 100; Pendulum T=0.1: n 1000; Pendulum T=0.2: n 2000; Pendulum T=0.4: n 4000 | Pendulum T=0.01: n 1; Pendulum T=0.1: n 3; Pendulum T=0.2: n 5; Pendulum T=0.4: n 9 | 10 → 7 | pass/pass |  →  |
| `demos/compile/multi_rate_computer.py` | SingleMass T=0.01: n 100 | SingleMass T=0.01: n 1 | 3 → 2 | pass/pass | [-0.274392, 0.342637, -0.014929, 3.854396] → [-0.274392, 0.342637, -0.014929, 3.854396] |
| `demos/control/lqg_cartpole.py` | StateFeedback @ NoisyCartPol: dt 0.01 | StateFeedback @ NoisyCartPol: dt 0.00333 | 6 → 7 | pass/pass | [-0.100627, 3.140189, 0.046856, -0.038212] → [-0.100697, 3.140241, 0.049862, -0.032128] |
| `demos/control/sliding_mode_pendulum.py` | Step >> SlidingMode @ Pendul: dt 0.0001 | Step >> SlidingMode @ Pendul: dt 0.005 | 12 → 2 | pass/pass | [2e-06, -0.000206] → [-0.000143, -0.010654] |
| `demos/core/diagram_noise_ports.py` | Pendulum with Measurement No: dt 0.01; Pendulum with Process Noise : dt 0.01; Pendulum without Noise: dt 0.01 | Pendulum with Measurement No: dt 0.01; Pendulum with Process Noise : dt 0.01; Pendulum without Noise: dt 0.01 | 6 → 6 | pass/pass | [0.688394, 0.000101] → [0.688394, 0.000101] |
| `demos/hybrid/sampled_smc_pendulum.py` | Pendulum T=0.05: n 500 | Pendulum T=0.05: n 2 | 2 → 2 | pass/pass | [0.005075, 0.108108] → [0.005075, 0.108108] |
| `demos/mpc/mpc_car_circuit.py` | BicycleDynRate T=0.2: n 2000 | BicycleDynRate T=0.2: n 1554 | 386 → 345 | pass/pass | [-8.832127, -6.82856, -4.843769, -2.862517] → [-8.832127, -6.82856, -4.843769, -2.862517] |
| `demos/mpc/mpc_car_minimal.py` | BicycleDynRate T=0.05: n 500 | BicycleDynRate T=0.05: n 400 | 6 → 9 | pass/pass | [18.450578, 20.401703, 22.351569, 24.302279] → [18.450578, 20.401703, 22.351569, 24.302279] |
| `demos/mpc/mppi_pendulum.py` | Pendulum T=0.05: n 500 | Pendulum T=0.05: n 2 | 770 → 475 | pass/pass | [-0.27018, -0.195235, 0.030443, -0.076321] → [-0.27018, -0.195235, 0.030443, -0.076321] |
| `demos/mpc/mppi_racecar.py` | UdeSRacecar T=0.1: n 1000 | UdeSRacecar T=0.1: n 1000 | 113 → 127 | pass/pass | [3.447523, 0.214601, 3.50525, 0.150013] → [3.447523, 0.214601, 3.50525, 0.150013] |
| `demos/udes_racecar/mpc_racecar_dynamic.py` | UdeSRacecarDyn T=0.1: n 3557 | UdeSRacecarDyn T=0.1: n 3557 | 296 → 51 | pass/pass | [-4.68646, -5.102902, -5.468246, -5.776003] → [-4.68646, -5.102902, -5.468246, -5.776003] |
| `demos/udes_racecar/mpc_racecar_kinematic.py` | UdeSRacecar T=0.1: n 1000 | UdeSRacecar T=0.1: n 1000 | 40 → 32 | pass/pass | [0.824634, 1.316025, 1.801811, 2.261928] → [0.824634, 1.316025, 1.801811, 2.261928] |
| `projects/mpc/mpc_dual_rate.py` | BicycleDynRate T=0.01: n 100 | BicycleDynRate T=0.01: n 80 | 14 → 39 | pass/pass | [16.465149, 16.867791, 17.270323, 17.672743] → [16.465149, 16.867791, 17.270323, 17.672743] |
| `projects/mpc/mpc_slalom.py` | BicycleDynRate T=0.2: n 2000 | BicycleDynRate T=0.2: n 1554 | 188 → 575 | pass/pass | [81.105593, 82.102776, 83.100905, 84.099934] → [81.105593, 82.102776, 83.100905, 84.099934] |
| `projects/mpc/mpc_spatial_stack.ipynb` | BicycleDynRate T=0.2: n 2000 | BicycleDynRate T=0.2: n 1554 | 471 → 466 | pass/pass | [-8.287966, -6.25958, -4.246113, -2.219932] → [-8.287966, -6.25958, -4.246113, -2.219932] |
| `projects/pathtracking/mpc_v1/run_demo.py` | Engine bicycle casca T=0.01: n 100 | Engine bicycle casca T=0.01: n 100 | 185 → 85 | pass/pass | [-39.687194, -40.14394, -40.412954, -40.533893] → [-39.687194, -40.14394, -40.412954, -40.533893] |
| `teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb` | DiagramSystem: dt 0.01 | DiagramSystem: dt 0.0001 | 26 → 97 | pass/pass | [-0.146114, -0.004576, 0.054022, -0.002533] → [-0.149349, -0.003104, 0.051925, -0.001029] |
| `teaching/courses/udes_gro501/racecar_toward_mpc.ipynb` | DiagramSystem T=0.05: n 1779; UdeSRacecar T=0.1: n 1000 | DiagramSystem T=0.05: n 1779; UdeSRacecar T=0.1: n 1000 | 274 → 385 | pass/pass | [-3.465557, -3.594368, -3.655264, -3.638217] → [-3.465557, -3.594368, -3.655264, -3.638217] |
| `teaching/topics/classical_control/state_space_design.ipynb` | StateFeedback @ InvertedPend: dt 0.01 | StateFeedback @ InvertedPend: dt 0.01 | 49 → 50 | pass/pass | [1e-06, 8e-06] → [1e-06, 8e-06] |
| `tutorial/06_hybrid.ipynb` | Integrator T=0.1: n 1000 | Integrator T=0.1: n 1000 | 4 → 4 | pass/pass | [0.999867] → [0.999867] |
| `tutorial/11_reinforcement_learning.ipynb` | Pendulum T=0.05: n 500 | Pendulum T=0.05: n 2 | 154 → 158 | pass/pass | [-3.226458, 3.4e-05] → [-3.226458, 3.4e-05] |
| `tutorial/showcase_minilink.ipynb` | BicycleDynRate T=0.2: n 2000; Pendulum T=0.1: n 1000 | BicycleDynRate T=0.2: n 1600; Pendulum T=0.1: n 2 | 147 → 119 | pass/pass | [0.451992, -0.234524] → [-0.071543, 0.670143] |

Reading the table:
- **Most MPC demos keep their numbers to the printed digit**: their closed loop names its step
  (`dt=` at the run, `substeps=` at the compile); the automatic decision recorded is the one a
  loop takes at wiring, or its evidence's, never their drawn run.
- **The plants now declared run far fewer steps**: the pendulum loops (`analysis_digital_pid_poles`,
  `sampled_smc_pendulum`, `mppi_pendulum`, the RL tutorial, the showcase) go from 500–4000
  substeps a period to 1–9; `mppi_pendulum` 770 s → 475 s; the mass-spring loop 100 → 1.
- **`cartpole_dynamic_controller` (GRO501)** now steps at 0.1 ms: its loop holds a custom
  stateful block that declares no time constant, so the 1 ms fallback applies (R180: no
  automatic linearization). Before, the exception stepped it at the held value's 10 ms. Its
  final state moves by 3e-3; the course block could declare its own `τ` (the maintainer's
  teaching call).
- **The showcase's largest record** changed identity, not value: its sampled pendulum loop's
  dense run is now 80 × 2 points, not 80 × 1000, so another trajectory is the largest; the
  loop's own state at 8 s is within 4.1e-6 of the 0.1 ms run (§3).
- **The kinematic racecar's loops** (`mppi_racecar`, `mpc_racecar_kinematic`, the GRO501
  notebook's first MPC) keep 1000 substeps (M-8); the dynamic racecar keeps its own `τ`.

**Clean re-times** (the branch's head, so with the one draw per step, against the base tree;
run one at a time, interleaved, on an idle machine; seconds for the whole file):

| File | Base | Head |
| --- | --- | --- |
| `projects/mpc/mpc_dual_rate.py` (two runs) | 11.3, 10.4 | 9.7, 11.1 |
| `demos/mpc/mpc_car_minimal.py` (two runs) | 4.6, 4.4 | 4.5, 4.3 |
| `projects/mpc/mpc_slalom.py` | 11.9 | 11.9 |
| `teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb` | 24.8 | 63.8 |
| `demos/control/lqg_cartpole.py`, its 8 s run alone (two runs each) | 0.49, 0.47 | 0.81, 0.88 |

The table's slow "after" MPC times (`mpc_dual_rate` 39 s, `mpc_slalom` 575 s) were the test
suite running beside them: alone, the MPC demos take the time they took. The GRO501 cart-pole
notebook is 2.6× slower at the 1 ms fallback (§7's reading); `lqg_cartpole` is within the
plan's 3× (1.7–1.9×).
