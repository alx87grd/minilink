# The night run (2026-10-29): the brief — signals, the compile cache, MPPI, the fixes, then speed until the flagships are as fast as before

**Audience:** the overnight session (local, on the maintainer's M4 Max), and Prof. Alexandre
Girard (Maintainer) for the morning review. Approved in the session of 2026-10-28 evening. The
run's record is [the night log](2026-10-29-night-run-log.md).

## Context

The v0.3 core run (2026-10-28, cloud) landed the new architecture. Tonight's local review on the
M4 Max found:
- **The zero-delay MPC demos were 4–15× slower.** The animation re-solved the MPC several times
  per frame. That is fixed in the working tree (the sampled loop animates its plant from its own
  run), but every read-back still re-solves once per tick, because a run records no signals.
  That gap is TODO §1.5's row "Recording the signals, and reconstructing a few", your decision 6
  of 10-28.
- **The compile-once cache is fragile.** It is a global 16-entry LRU keyed by guessing structure
  from attributes. Its known stale cases: a function or helper stored on a block, `jnp` arrays,
  class monkeypatches, `id()` reuse of functions, port bounds, the RL rollout after a plant edit,
  and the problem evaluator's partial key. It also sits uneasily with CONSTITUTION §4.4. And
  RD-68's frozen-vs-parametric measurement is weak.
- **About a dozen bugs and textbook offences** were confirmed or found by review (listed in
  step 5).
- **`box % dt` hands the blocks inside the tick count where they read seconds** (decision 5 of
  10-28); the sampled diagram gets a cleanup pass with it (step 1).
- **The cart-pole RL demo learns worse at 2M steps.** The cause is the one-score reward
  (`−c_k`, the period's integral, in place of `−g·dt`), shown on 12 seeds. The seed system is not
  the cause.
- **You want MPPI on the new core now,** as a test of the planner's compiled face.

**Outcome wanted by morning:** these land on a branch, every flagship demo (MPC and RL at least)
is as fast as before the run or faster with at least equal quality, every decision is recorded
for review, and a full speed sweep backs it up.

## How the run works (last night's principle)

- **Decide as the maintainer would.** Use CONSTITUTION precedence (purity, closure, continuous
  contract, textbook readability, single path), then the rulings, then the smaller reversible
  option. Each decision goes in the run log with the choice, the reason, the alternative and the
  commit to revert. Nothing student-facing is removed; tuned constants stay unless a decision
  below says otherwise.
- **Branch:** `dev-v0.3-night` off `dev-v0.3` (`git fetch` first; stop if behind).
  - One row = one commit, pushed after each row.
  - At the end, if every gate is green: `--no-ff` merge into `dev-v0.3`, then push.
  - Commit messages: subject and body only, no trailer (AGENTS.md).
- **Records:**
  - `docs/reviews/2026-10-29-night-run-brief.md` (this plan, as the brief).
  - `docs/reviews/2026-10-29-night-run-log.md`, with the morning summary, the decisions (N-1…),
    the ledger of number changes, the speed tables, what is not done, and the textbook-check
    statements.
- **Resumable:** the log is written before the next row starts. A resumed session reads the
  brief, then the log, then `git status`, and continues.
- **Keep-awake:** `request_keep_awake` for the session, and `caffeinate -i` around long sweeps.
- **Gates:**
  - Every row: ruff (whole repo) plus its narrow tests; every math diff read beside
    `planning/dynamic_programming/dp.py`.
  - Every step: the full `pytest`, "Regression gates, CI flags", the flagship demos, and the
    notebook smoke check when notebooks are touched.
  - Byte-identical against a seeded `benchmarks/v03_baseline.py` capture taken first, or a ledger
    line.
- **Subagents:** at most a read-only review per step plus the profiling helpers; never more than
  2–3 at once.

## Step 0: land tonight's working tree (first commits)

The working tree already holds:
- the sampled loop's `animate` / `plant_run_of` fix (`core/discrete/sampled_diagram.py`,
  `mpc/viz.py`);
- MPC `verbose` (`mpc/predictive.py`, DESIGN, `planner-and-mpc.md`);
- two tests;
- your `TRAINING_TIMESTEPS = 3_000_000`.

Commit these as three commits. Then capture the baseline JSON twice and `cmp` the two.

## Step 1: the `%` operator and a clean pass over the sampled diagram

**The bug (TODO §1.5, "Clocking a box"; your decision 5 of 10-28; the zero-delay MPC record
§2.7).** `box % dt` on a step diagram wraps the whole box as one tick law, so a static block
inside reads the tick count `0, 1, 2` where seconds are `0, 0.1, 0.2`: `t` and `k` are mixed up.
The zero-delay MPC works around it with its own `__mod__`.

**The fix:** `box % dt` clocks each of the box's blocks and keeps their faces (a block in seconds
reads seconds, a tick law its own count). Then delete the zero-delay MPC's `__mod__`. The
baselines stay byte-identical.

**A review pass over the sampled path, for a clean and future-proof `SampledDiagramSystem`**
(`core/discrete/sampled_diagram.py`, `sampled_system.py`, `step_diagram.py`, `core/compile/plan.py`
`SampledExecutionPlan`, `evaluators/discrete.py`):
- Write a time table of the semantics: which time every block kind reads, nested or not, under
  `%`, under `@`, under `>>` (the sampled-path.md §8 scenarios, reviewer C §2), and test every
  row.
- Close RD-17: a bare tick law inside a nested box must not run faster when the outer loop is
  finer. Either refuse it or give it its box's clock; decision N-8, logged.
- Plant lookup by role:
  - make the plant's identity one owner, the plan's `plant_op`;
  - delete `state_index["plant"]` lookups (`mpc/viz.py` `plant_states_of`).
- One copy of each rule: the holds, `divisor_of`, `t_k` to seconds, `k // d`, the dense
  substeps.
- Textbook names (`h` vs `dt`), refusals that say what is supported, no leading underscores.
- The `x0` read-only view: element writes on a diagram's start fail with a message naming
  the block to write (in place of numpy's "assignment destination is read-only").
- **Done when:** the time table's tests pass, the MPC's own `%` is gone, the baselines are
  byte-identical, and sampled-path.md / DESIGN describe what the code does.

## Step 2: record signals while simulating, reconstruct only what is asked

**API (decision N-1):** `compute_trajectory(..., record=("controller:u", "solve:z"))`. The run
returns a `Trajectory` whose `signals` hold those ports at every sample.
- A sampled loop always records its plant's input as `plant:u`, with the role
  `roles={"plant": {"x": …, "u": "plant:u"}}` set, because it is cheap and every read-back needs
  it.
- Evaluator method: `rollout_dense(..., record=keys)` / `rollout(..., record=keys)`, which
  returns `(x, signals)`.
- `reconstruct_internal_signals(traj, ports=None)` gains `ports=`, so only the requested ports
  and their ancestors are computed.

**Where:**
- **Sampled dense run:** `core/compile/evaluators/discrete.py`. The `period` closure in
  `rollout_dense` already has the tick's buffer (`port_signals`). Emit the selected slices as
  extra scan output in `stepping.iterate_dense`, held over the period's `S` samples, with a final
  pass for the end sample.
- **Continuous diagrams:** the rk4/euler `rollout` (`evaluators/base.py`, `stepping.iterate`)
  records the stage-1 buffer at `x_k`. The scipy path records by one ancestor-pruned pass over
  the output samples (no in-flight hook).
- **Ancestor pruning:** in `core/compile/plan.py`, a helper that maps `output_slices` back to
  `(sys, port)` and walks `gather_sources` backwards to a pruned tuple of `port_ops`. A held port
  stops the walk.
- **Callers:** `plant_run_of` and the animation, `mpc/viz.py` (`plan_history_of` already reads
  `"<path>:z"` signals), `graphical/signals/time_signals.py` (skips recorded signals, and the
  partial reconstruct uses `ports=`), and `PlanningEvaluator.run` (`signals_at_ticks` /
  `signals_along` become recording in the scan).
- **JAX:** the record set is a static tuple, so the jit keys on it.

**Done when** (the TODO row's own bar):
- a zero-delay MPC run with its overlays and its animation solves exactly once per tick, counted
  by a test;
- the delayed form's read-backs stay free;
- the baselines are byte-identical;
- DESIGN §4 and TODO are updated (the row closed).

## Step 3: the compile cache, made exact and explicit (decision N-2)

**Verdict:** fragile as built. The frozen/parametric split is right; the global cache's key is
the problem.

**Keep:**
- the two forms behind one method name (`params=None` frozen, a tree parametric);
- `call_params()` digest switching, because values can never go stale;
- the handles (`sys.compile()`, `problem.compile()`, `planner.compile()`).

**Change:**
- **Exact keys from edit counters.**
  - `System.__setattr__` and the port and signal setters bump a per-block `edits` counter.
  - The key becomes `(backend, x64 after the evaluator is built, wiring, ((block, block.edits)
    for every leaf))`, with the entry holding the blocks (and the functions) alive.
  - This replaces `fingerprint`'s attribute walk (`core/compile/cache.py`).
  - Covered: a function or helper swapped on a block, `jnp` attributes, port bounds, rewiring,
    `id()` reuse.
  - What remains: an in-place edit inside a held object (`block.helper.k = 2`) and a class
    monkeypatch. Both are documented in KNOWN_ISSUES with the explicit remedy
    `sys.compile(backend="jax")` (a fresh handle), and `clear_compiled` stays.
- **Same rule elsewhere:**
  - the problem evaluator's `problem_parts` (all parts, `X0`, `sys`, the laws);
  - the RL `rollout_forms`, keyed by the env and plant edits;
  - the planner offline key: digest JAX leaves by value, not by id; key options by
    `structure_value`, not `repr`.
- **Re-measure frozen vs parametric properly**, frozen meaning constants baked as in C1, on the
  rollout, `pe.evaluate`, and the native MPC loop. If frozen gains ≥ 15 %, the convenience calls
  keep it (the RD-68 note revised); otherwise parametric only, a simplification. Record the
  numbers.
- **Tests:** one per stale case in the review's list, plus LRU eviction and x64.
  - `test_a_structural_edit_is_seen_after_clear_compiled` is fixed to test what it says.

## Step 4: MPPI on the compiled face (TODO S72; `docs/plans/mppi.md`, rulings R106–R108)

- **Prerequisite:** `RolloutEnvironment` moves to `planning/environment.py` (R108), byte-identical.
- **`MPPIPlanner` and `MPPIRecord`** in `planning/trajectory_optimization/path_integral.py`,
  per mppi.md §2 to §6:
  - the solve body is a pure JAX function;
  - `n_z = N·m + 1`;
  - `shift` repeats the last input, and `command` reads `U` at `t`;
  - `temperature`, `noise` and `control_cost` are under `planner.params["method"]`, with `seed`
    beside them;
  - the seed comes from the block's seed and the tick (R104).
  - This lands RD-54's `"method"`/`seed` leaves now, MPPI being their first user (decision N-3).
- **The MPC block** stops reaching past the face (`pv.transcription`, `pv.offsets`,
  `pv.problem.sys`). The face gains `predicts`, the knot period and `model` per the face table,
  so `ModelPredictiveController(MPPIPlanner(...))` works in all three forms.
- **Demos:** `examples/demos/mpc/mppi_pendulum.py` and `mppi_racecar.py`, ported from
  `experimental/mppi/`, with the racecar measured against the SLSQP MPC as in mppi.md's table.
  The prototypes stay.
- **Tests:** mppi.md §8 (seeded determinism, LQR agreement on the double integrator, swing-up
  into `Xf`, MPPI twins in `test_mpc.py`).

## Step 5: fix what the reviews found (one commit each)

1. `examples/demos/compile/neural_controller_jax.py` writes `diagram.x0[0]`: write the plant's
   start instead. Plus any other diagram element-write the sweep finds.
2. `lookahead` with `delay=False`: the zero-delay and pure forms predict the start
   (`predicted_start`) as the delayed form does, with a test.
3. **MPC with a non-singleton `X0` (decision N-4):** the MPC pins the program's `X0` to the bound
   measurement, as `solve_trajectory_from_parametric` already does; covered by a test.
4. The compiled face's live `pv.params` against the program's snapshot: `planner.compile()`
   snapshots, so `pv.params` is the compiled tree, documented as a build verb.
5. The failure price's cache key reads the evaluation's `discount_rate`, `X` and `U` overrides.
6. Lyapunov `P = (P + Pᵀ)/2` after the solve.
7. Fields: `FieldSet.margin`, `FieldInputSet.margin`, `FieldCost.g` and `FieldTerminalCost.h`
   thread `params`. `Set`/`InputSet.with_params` return a copy. No shared mutable class-level
   `params = {}`.
8. **Textbook rules:**
   - `first_order.py` binds `lower`/`upper` (5.3);
   - one Adam step owner (5.6);
   - `h` → `dt` in `sampled_system.py` (5.4);
   - a timeless refusal in `step_problems.py` (5.24);
   - `_jac_validate_params` renamed (5.8).
9. `validate_equation_shapes` does not probe a native sampled law's `update` (option 1), so a
   delayed MPC no longer solves at compile.
10. **Value iteration `alpha=` with ρ > 0 (decision N-5):** refuse the pair, naming the one
    discount to give.
11. **The judge on an `exact` hold (decision N-6):** integrate `c_k` with the RK4 cost path at the
    plant's substeps beside the exact state step; else refuse clearly and add a TODO row.
12. **Traced first guess:** `pv.initial(params)` uses the measured start under a trace too. The
    delayed form's tick 0 shifts its guess to `t_start` of the run.
13. **RL reward (decision N-7, your call tonight):** keep `−c_k`, the one score with no cost
    shaping. The cart-pole demo trains 3M steps (your edit). Logged with the 12-seed evidence; no
    library change.

## Step 6: speed and quality, iterated until the flagships pass

- **Bar per flagship demo** (`tests/demo_checks/flagship_manifest.json` plus the MPC and RL
  flagships: `mpc_car_circuit`, `mpc_racecar_dynamic`/`kinematic`, `mpc_car_minimal`,
  `mpc_integrator_numpy`, `native_mpc_tuning`, `cartpole_swing_up_rl`, `rocket_landing_rl`, the
  MPPI demos, plus value iteration (`vi_pendulum_swingup`, `vi_cartpole_swingup`,
  `vi_double_pendulum_jax`, `vi_quadratic`, `vi_minimum_time`) and cart-pole trajectory
  optimization (`trajopt_cartpole_collocation_jax` and the other `examples/demos/trajopt/`
  scripts)):
  - wall-time median of 3 ≤ 1.05 × the pre-run time (`8db211c4`, or 0.2.0 where newer);
  - quality at least equal: MPC distance/laps/J and time outside the lane; RL swing-up success
    over 8 seeds at the demo's budget; value iteration's cost-to-go and policy (the RK4 grids
    moved by the one-score stage cost: same ledger tolerance) and its closed-loop `J`;
    trajectory optimization's solve status, iterations, `J` and solve time; final states for
    the rest.
  - Tonight's first sweep already shows value iteration 10–25 % slower than 0.2.0: profile it
    first (the augmented step's cost path is the suspect, RD-36).
  - The pre-run numbers are already measured tonight (`run_partial/`).
- **Loop:**
  1. Run the flagships on HEAD, 3 reps (`time_demos.py` with a HEAD worktree at the latest
     commit).
  2. For each miss, cProfile it, find the cause, fix it as its own row, and rerun that demo.
  3. Repeat until all pass, or until a miss is shown to be the plant's own cost or an intended
     change (logged with its evidence).
- **Then the wide sweep**, using the existing script
  `/private/tmp/claude-503/-Users-agirard--claude-spares-spare-difvj7/c0e92533-bb33-4fe9-a3d7-2264ad6fc401/scratchpad/overnight.sh`:
  - every common demo × 0.2.0 / pre-run / HEAD × 3 reps;
  - each version's regression suites;
  - `v03_budget.py`.
  - Any demo > 15 % slower or failing gets a row and another loop.
- **Morning summary tables:** per-demo wall time for the three versions, flagship quality, and
  regression suites.

## Verification (end of run)

- Full `pytest`; "Regression gates, CI flags"; flagship demos 100 % passing; notebook smoke
  check; docs build if docstrings changed; ruff whole repo.
- The counted-solve test for the zero-delay MPC (one per tick, read-backs included).
- The stale-cache tests (every case listed in step 3).
- The sampled-path time table's tests (step 1), the zero-delay MPC with no `__mod__` of its own.
- MPPI: the racecar beats or matches the SLSQP MPC's J at a lower tick time.
- Speed: the step 6 bar met for every flagship, or the remaining misses explained in the log.
- `git status` clean on `dev-v0.3-night`, pushed; merged into `dev-v0.3` only when all of the
  above is green.

## Not tonight

- The three-block MPC for every form (decision 7): it follows steps 1 and 2 but changes the
  delayed form's structure; logged as next.
- Ipopt's Lagrangian Hessian.
- Notebook prose.
