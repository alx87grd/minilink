# The merge review's rulings, implemented: the plan (2026-10-30)

**Read first.** This is the one plan for the work the maintainer ruled on 2026-10-30, when he
reviewed the four agent runs' decisions in one sitting
([the triage](../reviews/2026-10-30-merge-review-triage.md); the rulings R180–R187 in
[the record](../reviews/2026-10-30-merge-review-rulings.md)). An agent picks it up and implements
it, in a cloud session that sees only this repository. Nothing here is open: where a row leaves a
choice to the agent, the agent decides as the maintainer would (CONSTITUTION's precedence, then
the rulings, then the smaller reversible option) and logs the decision. Each row gives the code
as it stands, the ruled design, what to delete, what to write, and its "done when". The order of
the rows is the order to work in.

**The working rules** are AGENTS.md's:
- **Branch:** `dev-v0.3-merge-review` off `dev-v0.3` (fetch first; stop if behind). One commit
  per row, pushed as it lands. Commit messages are subject and body only, no trailer of any kind.
- **The baseline:** capture `benchmarks/v03_baseline.py` twice before the first edit and `cmp`
  the two; after every row, capture again and `cmp`. Byte-identical unless the row says numbers
  move; then a ledger line says what moved, by how much and why.
- **The textbook rule** (RULES §5): every math line read beside
  `planning/dynamic_programming/dp.py`, and said so in the log.
- **Gates** (tests/README.md, Agent table): every row, ruff on the whole repo and its narrow
  tests; at the end, "Cross-cutting or handoff", "Regression gates, CI flags", "Notebook change"
  (notebooks move in rows 8 and 1), "Docs build" (rows 10 and 12 move docs).
- **Demos and notebooks:** tuning constants never change; a row that rewrites a demo keeps its
  numbers unless the row says otherwise. Notebook outputs are stripped (`pre-commit run
  nbstripout --all-files` in a clone without the filter).
- **Speed:** a cloud machine's absolute timings are unreliable; compare versions as ratios in
  the same session, interleaved.
- **The log:** `docs/reviews/2026-10-31-merge-review-run-log.md`, in the format of
  [the consolidation's log](../reviews/2026-10-29-consolidation-run-log.md): a summary first,
  the decisions taken without the maintainer (M-1, M-2, …, each with its context, the choice,
  the alternative and the commit to revert), the ledger of moved numbers, the textbook check,
  what is not done, and a progress table. It is written before the next row starts, so a
  resumed session reads this plan, then the log, then `git status`, and continues.
- **The end:** when every gate is green, merge `dev-v0.3-merge-review` into `dev-v0.3` with
  `--no-ff` and push. With a gate red, push the branch and say in the log's summary what is red.

**The spirit, in the maintainer's words.** The plant is the bottleneck that sets the step, not a
default held value. Evidence is only useful when it does not repeat the planner's own belief.
Keep the simple code where a speed-up is not worth it. A demo is the simplest demonstration of a
feature that works.

---

## 1. One automatic-step rule, from the physics (R180)

**The code today.**
- `simulation/simulator.py`, the time grid (around line 331): a fixed-step run with no `dt`
  steps at `AUTO_DT_SCALE × τ_min` (`AUTO_DT_SCALE = 0.1`, `core/system.py`), except that a held
  signal on a smooth loop whose `τ_min` is the 1 ms default takes
  `HELD_SIGNAL_STEPS_PER_SAMPLE = 1` step per sample (the `undeclared` branch).
- `core/discrete/sampled_system.py`, `sampled_substeps`: a sampled loop's plant at
  `⌈Δ / (0.1 τ)⌉` steps per period, `τ` the plant's declared time constant or the 1 ms default,
  so an undeclared plant runs at 0.1 ms (the consolidation removed the exception here).
- `core/wiring.py` (around line 711): a diagram's `τ_min` is the smallest over its blocks with
  states or a declared value; an undeclared block with states counts as the 1 ms default.
- `blocks/sources.py` (`NoiseSource`, `WhiteNoise`) and `control/stochastic.py` declare their
  hold period as their `smallest_time_constant`.
- `dynamics/catalog/vehicles/racecar.py`'s `fastest_mode()`: the pattern of a plant that
  computes its own `τ` from its `params`.

**The evidence** (`examples/demos/control/lqg_cartpole.py`, `tf = 8 s`, measured 2026-10-30 on
the maintainer's Mac): today's run steps at 0.01 s (the exception), 800 steps in 0.2 s; at the
default 0.1 ms it takes 21 s. The loop's fastest mode is the Kalman observer's pole at −23 s⁻¹
(τ = 0.043 s); the cart-pole's own poles are ±4.65 s⁻¹, the LQR modes −8.8 and −3.4 s⁻¹. With the
noise on, the error against the 0.1 ms run shrinks only in proportion to the step (9.8e-3 at
10 ms, 3.2e-3 at 3.3 ms, 9e-4 at 1 ms on the pole's rate); with the noise off it is 15 times
smaller at 10 ms and shrinks faster. The first-order term is most likely RK4's last stage, at
`t + h = t_{k+1}`, reading the next held draw.

**The ruled design.**
- One rule everywhere a fixed-step run picks its own step (a continuous diagram, a sampled loop's
  plant, `problem.compile(law % dt)` and a planner's evidence): `dt = AUTO_DT_SCALE × τ_min` over
  the blocks that have dynamics, rounded down so it divides every hold period of the loop.
  An explicit `dt=` at a run and `substeps=` at a compile still win.
- `HELD_SIGNAL_STEPS_PER_SAMPLE` and the `undeclared` branch go.
- A hold period places the grid's points; it is not a time constant: `NoiseSource`, `WhiteNoise`
  and the stochastic policy stop declaring one.
- Linear blocks compute their `τ` from their eigenvalues, read from their current `params`:
  `τ = 1 / max |λ|` over the nonzero eigenvalues (an all-zero spectrum, an integrator, declares
  nothing). The LTI blocks (`LTISystem` and its kin), the observers (`StateObserver`, so
  `kalman` and `luenberger`: the eigenvalues of `A − L C`), the transfer-function blocks.
- Catalog nonlinear plants declare their `τ` from their physics, as the racecar does: a property
  read from `params`, from the linearization at the nominal point by a closed formula (the
  pendulum's `√(l/g)`, the cart-pole's unstable pole). **First scope:** the catalog plants the
  public demos, projects and notebooks run on the automatic step. The rest of the catalog is a
  TODO row.
- An undeclared custom plant keeps the 1 ms fallback; no automatic linearization of an
  arbitrary plant.
- The RK4 held sample (TODO §6's row "A fixed-step RK4 that samples held sources once per step"):
  first a twin test (noise on; `dt`, `dt/3`, `dt/10` against a fine run; first order means the
  bias is real). If it is, a held source is read once per step, at the step's start, so the four
  stages read one sample: fourth order with no boundary error, one draw per step instead of four.
  The TODO row closes.

**The work, in order.**
1. A sweep: every demo, project, notebook, benchmark and baseline entry that runs on the automatic
   step (a fixed-step run with no `dt`, a held signal or a discontinuous loop; a sampled loop
   with no `dt`; a compile with no `substeps`; a planner's evidence). Its numbers and wall time
   before, in a table in the log.
2. The twin test and the RK4 held-sample fix, alone, with its ledger.
3. The rule, the hold sources, the linear blocks' `τ`, then the catalog plants of the sweep.
4. The sweep again: numbers and wall time after, in the ledger. The showcase notebook's
   `(ctl % 0.1) @ plant` (triage 3.6) now gets a physics step: check that its numbers come back
   near the old one-step-a-period run, and record them.
5. DESIGN §5 (the step rule, one paragraph, no exception), the docstrings of the time grid and
   `sampled_substeps`.

**Done when:** no exception constant; tests: the digital controller's loop and the noise loop on
the same undeclared plant follow the same rule; a declared plant's step comes from its `τ`; an LTI
block's and an observer's `τ` from their eigenvalues; a hold period never sets `τ`; RK4 on a held
input whose switches sit on the grid is fourth order; `lqg_cartpole` within 3× of today's time;
every moved number in the ledger.

## 2. A planner's evidence, the problem evaluator's job (R181)

**The code today.** `Planner.evaluate(policy, *, dt, n_trials, tf, backend, problem)`
(`planning/planner.py`, around line 178) is a thin wrapper: it holds a feedback law at `dt`
(`held_feedback`, except LQR, `holds_policy = False`), returns `None` with no cost, draws
`n_trials` only for a random problem, then calls `evaluate(problem, law, dt=, …)`. Only the
planners' own `solve(evaluate=True)` call it: value iteration (`dp.py`), LQR, trajectory
optimization, MPPI (`path_integral.py`), RRT, the RL planner and the tabular learner. No demo,
test, notebook or benchmark calls it.

**The ruled design.** The method leaves the public API. It becomes a module function in
`planning/planner.py`, `planner_evidence(planner, policy, *, dt, n_trials=50, tf=None,
problem=None)` (no leading underscore, RULES 5.8), the seven `solve()` call it, and its docstring
points to the public way to score a law: `problem.compile(ctl % dt).evaluate()`, with
`substeps=` for the planner's own physics. `solve(evaluate=True)` stays, default off, and the
evidence runs on the real plant at the R180 step, the planner's own `J` beside it. DESIGN §6
(around line 1290) says so.

**Done when:** `Planner` has no `evaluate`; the baseline is byte-identical; DESIGN updated.

## 3. The step-problem names off the root (R185)

**The code today.** `minilink/__init__.py` exports `StepCost`, `SampledCost` (from
`minilink.core.costs`) and `StepPlanningProblem` (from `minilink.planning.step_problems`) on the
root, the teaching surface. No lesson uses them; `examples/demos/compile/grid_world_step_problem.py`
imports them from the root.

**The ruled design.** They leave the root and stay on their bands (`minilink.core.StepCost`,
`minilink.core.SampledCost`, `minilink.planning.StepPlanningProblem`); `StepSystem` stays where
it is. The demo imports them from the bands. `tests/unittest/test_teaching_surface.py`, the
teaching import allowlist, DESIGN §2 and any ROADMAP §2 list of root names follow. A ruling's
direct consequence in ROADMAP §2 is in scope; log it.

**Done when:** the three names are off the root; the surface test pins it; the demo runs.

## 4. LQR says "U ignored" (R186, amends R21)

**The code today.** LQR ignores the problem's input set `U`; the U sweep of 2026-10-24 wrote
`U=plant.input_box` on about 90 problems, the GRO860 LQR lessons among them, and the planned
"LQR warns once when `U` is declared" (TODO §2, "V1.2, the rest") waits on that.

**The ruled design.** LQR never warns. Its `RiccatiRecord` (`planning/dynamic_programming/lqr.py`)
prints "U ignored" in its `str` when the problem declares a finite `U`. No lesson changes. DESIGN
names it beside R21; the TODO row's LQR part closes; the summary table row "The LQR warning on a
declared `U`" goes.

**Done when:** a test on the record's text with and without a finite `U`; no warning anywhere.

## 5. The three hand gradient-loop demos onto `Optimizer` (R186)

**The code today.** `examples/demos/compile/cartpole_rollout_gradients.py` (an open-loop input
schedule), `neural_controller_jax.py` and `params_gradient.py` (equation-error identification of
gravity and damping) each descend with a hand loop `theta = theta - lr * grad`.

**The ruled design.** Each moves onto the library: its loss stays exactly as written, wrapped in a
`MathematicalProgram(n_z=…, J=loss, backend="jax")` and solved by `Optimizer(program, z0=…,
method=…)`. The agent picks the method that reaches the demo's own result (its asserts and its
printed values within their tolerances); L-BFGS (`"l-bfgs"` or `"auto"`) first. The schedule demo
needs no source with parametric samples: the hand program's `J(z)` rolls the schedule `z` out as
today. Each demo stays minimal (prints through the library's own reports, one figure at most, no
helpers that only print). One hand loop stays in the library's examples: the closing example's
first section (row 6). `examples/tutorial/07_compile.ipynb`'s hand loop is a notebook, the
maintainer's teaching design: unchanged, logged. `examples/README.md`'s map follows.

**Done when:** no `theta - lr * grad` loop in `examples/demos/compile/` but the closing example's;
each demo runs headless (`MPLBACKEND=Agg`) and its asserts pass.

## 6. The closing example, as simple as possible (R186)

**The file.** `examples/demos/compile/pid_autotuning_jax.py`: a PID tuned by a hand gradient loop
on the problem's cost (section 1), then by the pipeline `problem.compile` → `pe.program` →
`Optimizer` landing on the same gains with `method="gd"`, then finishing the descent with
`method="auto"` (section 2) (the night of 2026-10-24, OD-20).

**The ruled design.** The simplest demonstration of the features that work: one hand loop on the
problem's cost, then the library doing the same and finishing the descent. Delete its two
helpers, the prints that carry arithmetic and the hand-made last figure; print the library's own
records (`pe.evaluate`, the optimizer's result) and plot with the library's own plots
(`plot_trajectory` or `compare`). The live plot during the hand loop goes if it needs a helper.
Its tuned constants stay (`TF`, `DT`, `N_STEPS`, `N_LEARN_STEPS`, `LR`, the start gains).

**The maintainer's local edits.** On 2026-10-30 the maintainer had uncommitted edits to this
file. Work on the version at the branch tip; if its last commit predates 2026-10-30, say in the
log that the maintainer's local edits were not in the branch.

**Done when:** the demo runs headless, sections 1 and 2 agree on the gains, the file is about half
its length.

## 7. Two demos' style (R186)

- `examples/demos/core/diagram_shortcuts.py` labels a `NoisyPendulum()` as `Pendulum()`: fix the
  label.
- `examples/demos/robotic/custom_one_link_arm.py` reads module constants inside its equations:
  read them from `params` (RULES §5, R169), the numbers the same.

**Done when:** both run headless with the same numbers.

## 8. The MPC notebooks' behaviour prose (R186)

**The notebooks.** `examples/teaching/courses/udes_gro501/racecar_toward_mpc.ipynb` (§7–§8),
`examples/tutorial/showcase_minilink.ipynb` (its MPC section), `examples/projects/mpc/mpc_spatial_stack.ipynb`.

**The ruled design.** Delete every sentence that describes how the car behaves in a run (laps,
lap times, distances, cutting or missing a corner, smoothness, speed, comparisons between runs).
Keep the code cells and the prose that says what a cell builds or computes. The maintainer
re-runs them and writes the behaviour back himself. No code cell changes.

**Done when:** the notebook smoke check passes; the log lists each deleted sentence's notebook
and section.

## 9. ROADMAP §4.1's RL row (R186)

`docs/dev/ROADMAP.md` §4.1's RL row names `StochasticPlanningProblem`, which is gone: write
"`PlanningProblem` with laws". Nothing else in §4 moves.

## 10. The pipeline figures: the map, with where a `System` comes from (R186)

**The code today.** `docs/pipeline_figures.py` draws ten figures into `docs/_static/pipeline_*.svg`
(`FIGURES`, around line 1469); `docs/make_assets.py`'s `pipeline_map` turns the map into the
README's `pipeline_map.png`. `docs/pipelines.md` shows six of them; DESIGN §6 (around line 1319)
shows the two evaluator figures.

**The ruled design.**
- Keep the library map, `library_map` (`pipeline_map.svg` and `.png`; README line 62;
  `docs/pipelines.md`'s "The library on one page").
- Delete the others for now: their functions and `FIGURES` entries, their `.svg` files, their
  sections in `docs/pipelines.md`, DESIGN §6's two figures and the sentence that points to them.
- Add a layer above the map's `System`: how a `System` is made, three cards in the map's
  palette and notation: (a) your own class (`class Arm(System): def f(self, x, u, t, params)`),
  (b) the catalog (`Pendulum()`), (c) a composition of blocks (`ctl @ plant`, `a >> b`,
  `DiagramSystem`). The arrows from all three meet the `System`.
- Rebuild the `.svg` and the README `.png` (`python docs/make_assets.py map`). If the cloud
  machine has no SVG-to-PNG converter, commit the `.svg` and say in the log that the `.png`
  needs `python docs/make_assets.py map` on the maintainer's machine.

**Done when:** the docs build passes with `-W`; one figure left, with the new layer.

## 11. The removed `x_goal` lines, checked (triage 3.8)

The consolidation (C-18) removed every `x_goal=` from the demos where the cost's `xbar` or the
given `Xf` is the same point; RRT's became `Xf=BallSet(goal, tol)`. Check each removal in the
consolidation's diff (`git log -S x_goal` on `examples/`): the target the demo now reads is the
point it read before. List the files in the log; fix an exception as its own commit with a ledger
line, or log it if the fix would change what the demo teaches.

## 12. The failing Ipopt test

`tests/unittest/test_mpc.py::TestMPCSolveTrajectoryFrom::test_the_compiled_tick_solves_with_ipopt`
fails on a machine with Ipopt (the maintainer's Mac, 2026-10-30): the compiled tick's plan is
0.023 away from its reference in 3 of 5 entries. It fails the same way on `f36fdd3b`, so it came
with the consolidation (likely the one MPC box, R176, or the guess toward `Xf`, R177). Find the
cause; fix it, or re-pin the reference if the new plan is the right one, with a ledger line. If
the cloud machine has no Ipopt, log that the test was skipped there and leave the diagnosis.

## 13. The records

- The rulings record: one line per row with its commit, under the table.
- [The triage](../reviews/2026-10-30-merge-review-triage.md): its §6 says the sitting is done and
  points to this plan and the log.
- `docs/plans/TODO.md`: the review row of "Rows waiting for the maintainer's go" closes; the rows
  this plan lands close (the LQR warning; S74's rest; the RK4 held sample; K4's three demos'
  style); new rows: the rest of the catalog declaring `τ` (R180), a training run that rebinds its
  numbers (R182, later, research), `lqr_finite_horizon` handing out its field (R185, later, with
  the fields' consumers).
- `docs/dev/DESIGN.md`: the step rule (row 1), the evidence (row 2), the root (row 3), LQR's record
  (row 4), the figures (row 10). `docs/dev/ROADMAP.md` §3's TRL ledger where a component's
  evidence moved. `docs/dev/KNOWN_ISSUES.md` for a defect found and not fixed.
- `benchmarks/v03_budget.py`'s comment on entry 5b: a drawn mass is slower because each trial
  solves its own mass matrix, and JAX's batched solve of many tiny matrices is slow on CPU; it
  stays reported, not judged, and the mechanical base's `solve(H, …)` is unchanged (R184, the
  maintainer: "not worth it").
