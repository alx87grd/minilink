# The night run (2026-10-29): the log

**Audience:** Prof. Alexandre Girard (Maintainer), for the morning review.
**Context:** the local overnight run on `dev-v0.3-night` under [the brief](2026-10-29-night-run-brief.md),
after the evening review of [the v0.3 core run](2026-10-29-v03-run-log.md). Where a review would
have been asked, the decision is taken as the maintainer would, written in §2, and the run
continues.

**How to read it.** §1 is the morning summary. §2 lists every decision taken without the
maintainer (N-1…), each with the step, the choice, the reason, the alternative and the commit to
revert. §3 is the ledger of intended number changes. §4 holds the speed and quality tables. §5
is what was not done or is blocked. §6 holds the textbook-check statement of every math diff. §7
is the progress record, one line per pushed row; a resumed session reads it to continue.

## 1. Morning summary

**The run is done** on `dev-v0.3-night` (pushed; merged into `dev-v0.3` at the end when the gate
was green, §7). Every step of the brief landed; the gate is green: ruff, the full `pytest`, the
regression gates with the CI flags, the flagship demos (17, one interactive skipped), the
catalog (50), the notebook smoke check (34), the docs build.

**What changed, in one paragraph each:**

1. **The MPC demos are faster than before the run.** The 4–15× slowdown was the animation
   re-solving a zero-delay MPC at every frame (`37c45da4`); the run now records the plant's input
   in its own signal pass, so a run solves once per tick with its read-backs (`9ea45d66`). On the
   M4 Max, median of 3: `mpc_car_circuit` 10.6 → 8.7 s, `mpc_racecar_dynamic` 22.9 → 17.5 s,
   `mpc_racecar_kinematic` 35.7 → 23.5 s (before the run → tonight).
2. **`box % dt` clocks each block with its own face** (`f39e1010`): a static block in a box reads
   seconds, a tick law its count; the zero-delay MPC lost its own `%`.
3. **Signals are recorded while simulating** (`compute_trajectory(record=...)`, the evaluators'
   `record=`, `reconstruct_internal_signals(traj, ports=...)`): your decision 6 of 10-28.
4. **The compile cache's key is exact** (`8ed9ce00`): each block by identity and an edit count
   bumped by every assignment; the evening's stale JAX results (a function, a helper, an edit
   inside a helper) are now correct, a test each. The frozen form is kept, measured worth it
   (54 % on a rollout); a JAX optimization program now has one too, compiled for its consumer
   (`da27b503`).
5. **MPPI is on the compiled face** (`MPPIPlanner`, `137e5290`): the MPC runs it in every form;
   on the UdeS circuit it laps 2.09 times in 10 s at 4.0 ms a tick (J 0.35).
6. **The review's fixes:** `lookahead` with `delay=False`, the MPC refusing a start set, the face
   keeping its compiled tree, the failure price at the evaluation's rate, Lyapunov's `P`, the
   compile probe of a native law, the exact-hold message, the read-only start of the
   `neural_controller_jax` demo.
7. **Speed:** the misses the step-6 loop found (value iteration 9–13 %, the cart-pole
   collocation 8 %, `diagram_compiling` 9 %) are fixed; `readme_examples` stays 6 % slower
   (N-21). The final sweep's table is §4.

**For your review, in this order:**
1. N-7 and N-22, the cart-pole RL demo: tonight's evidence says the one-score reward, not the
   seed system, makes it learn less reliably, and that 3M or 4M steps do not restore it (3/8,
   2/8 seeds against 9/12 before the run); its default seed swings up.
2. N-2 and N-13, the compile cache: kept and made exact, with the two blind spots left in
   KNOWN_ISSUES.
3. N-1, N-11, N-12, recording: the API, the end sample, the continuous path.
4. N-3, N-14 to N-17, MPPI: the `method`/`seed` leaves, the names on `minilink.planning`, the
   deterministic scope, the tick's key, `env.step(t_origin=)`.
5. N-19, a new design question in QUESTIONS.md (the fields' parameters).
6. The rest of §2 as a block.

## 2. Decisions taken without the maintainer

| Id | Step | Question | Choice | Reason | Alternative | Commit |
| --- | --- | --- | --- | --- | --- | --- |
| N-7 | 0 | The cart-pole RL demo fails to swing up at 2M steps since the run (seed 0) | Keep the one-score reward `−c_k` (no cost shaping, no training reward apart from the score); the demo trains 3M steps (the maintainer's edit) | 12 seeds at 2M: before the run 9/12 swing up (J < 60, median 34); now 4/12 (median 229); now with only the reward put back to `−g(x_k)·dt` 7/12 (median 37). The rewards agree to correlation 0.992 at the same scale; the integral credits the action's effect within the period, which this swing-up learns more slowly from. The noise cipher matches `jax.random` (mean, spread, tails, independence); leaving `U` cannot fire (the action is clipped first). One score everywhere (R142, R159) outweighs a training convenience | A training reward `−g·dt` recorded beside the scored one, as `gamma` is | `80efe02e` |
| N-8 | 1 | RD-17: a bare tick law inside a nested sampled box ticks on the outer loop's base, faster than in its box alone | Keep R158 (a bare tick law reads the base count of the loop it runs in) and refuse, at compile, a bare tick law in a nested box whose own clock differs from the loop's base, naming the clock to give it (`law % Δ_box`) | A silent rate change is the worst outcome; a refusal at compile is explicit and costs one `%` | Give the law its box's clock automatically (a hidden rule); or keep RD-17's silence | `f39e1010` |
| N-9 | 1 | A diagram's `x0` is a read-only stacked view; an element write raises numpy's "assignment destination is read-only", which names no block | Keep the plain read-only array; fix the demos that write into a diagram's start (step 5) | An `ndarray` subclass whose `__setitem__` names the block would leak into every arithmetic result on `x0` (a fragile type for one message) | A subclass with a friendly message | — |
| N-10 | 1 | What `box % dt` shares with the box | The blocks themselves: a clocked block is a view of its source (as `pid % dt` is), so the box's numbers and starts are shared; a native law's `%` still copies. The zero-delay MPC's old `__mod__` deep-copied its solve block's numbers | One rule for every clocked block, the view rule of `%` | Copy every block's numbers under `%` | `f39e1010` |
| N-1 | 2 | The API of recording | `compute_trajectory(..., record=("path/block:port", …))` on every run verb, the evaluator's `rollout_dense(..., record=keys)` (a static tuple, one jit per record set) and `rollout_with_cost(..., signals=True)`; a loop with a plant always records `"plant:u"` and its `plant` role | The TODO row's words ("a run that records the signals it computes"); `record` reads as the verb; the plant's input is what every read-back needs and costs `m` numbers per tick | `signals=` on the facade (it would shadow `traj.signals`) | `9ea45d66` |
| N-11 | 2 | The record's end sample: the last tick's value, or the next tick's (`u_N`, the input the loop would apply at `t_N`) | The end sample holds the last tick's value, in the record and in `plant_run_of`'s fallback alike | `u_N` costs one more signal pass after the last tick: one more MPC solve per run, and the input never acts inside `[t_start, tf]`. Two tests pinned `u_N` and now pin the hold | Compute `u_N` (one extra pass) | `9ea45d66` |
| N-12 | 2 | A continuous diagram's `record=` | One selective pass over the output samples after the run (the named ports and the blocks they read), not a hook inside the integrator | The adaptive solver has no fixed samples inside the run; a continuous diagram holds no solve, so the pass is cheap; one path for every solver | Record the RK4 stage-1 buffer inside the fixed-step scan | `9ea45d66` |
| N-2 | 3 | Is the compile-once cache sound (your "bad feeling") | Fragile as built, and the cause was the key, not the two forms. Kept: one global LRU of 16 JAX evaluators (a loop rebuilt over the same blocks by `closed_loop` or `problem.compile` must reuse one, which an instance-owned cache cannot do), the frozen and parametric forms, `call_params()`'s digest switch. Changed: the key reads each block's identity and an edit count that every attribute assignment bumps (values and run records aside), a plain-data helper and JAX arrays by value, functions by code. Remaining blind spots (a class's method replaced, a change deep in a rich held object) are in KNOWN_ISSUES | The evening's probe gave silently wrong JAX numbers for a function swapped on a block, a helper swapped and an edit inside a helper; all now match NumPy (a test per case). An explicit handle-only design (every convenience call compiles fresh, 0.35 s each) gives up the speed the maintainer asked for on 10-28 | An instance-owned cache with version counters; or handles only | `8ed9ce00` |
| N-13 | 3 | Is the frozen form worth its second trace (RD-68's measurement was weak) | Keep the frozen form for the evaluators' runs; the planning evaluator stays parametric only | Measured on the M4 Max, median of 40 calls: a 2000-step JAX rollout frozen 6.8 ms against parametric 10.5 ms (54 %), a 100-tick evaluation 0.18 against 0.26 ms (42 %, 0.08 ms in absolute). The evaluator's calls vary their tree in practice (sweeps, gradients), and a frozen form there would need a second path for 0.08 ms | Parametric everywhere (simpler, 30–50 % slower runs) | `8ed9ce00` |
| N-3 | 4 | When the `"method"` and `seed` leaves land (RD-54 deferred them to v0.4 with MPPI) | Now, on `MPPIPlanner.params`: MPPI is their first user and landed tonight; the trajectory-optimization face keeps its tree (its method is structure there) | RD-54's own condition (they come with MPPI) is met | Keep them for v0.4 | `137e5290` |
| N-14 | 4 | MPPI's public names (mppi.md's [ask]) | `MPPIPlanner` and `MPPIRecord` on the `minilink.planning` facade (the provisional band), not on the root | R106 ruled the names; the root carries the teaching surface, and MPPI is research-lane until the teaching decision (V3) | The root facade | `137e5290` |
| N-15 | 4 | What MPPI does with a stochastic problem tonight | The deterministic planner's warning and the nominal plan (`accepts_stochastic = False`); the stochastic branch (`n_plant_samples`, mean or CVaR) stays TODO S72's rest | A night's scope; the deterministic textbook algorithm is what the MPC and the demos need | The branch now | `137e5290` |
| N-16 | 4 | How an MPPI solve at a later tick draws new noise (R104) | The key is the seed leaf with the tick `round((t0 − t_start) / dt)` folded in, so a solve is a function of its tree and its time, and every tick draws afresh | R104 (the seed and the tick); the zero-delay MPC binds `t_start` per tick | The MPC drawing a seed per tick into the tree | `137e5290` |
| N-17 | 4 | Where an MPPI solve starts its receding horizon | `env.step(..., t_origin=t0)`: the environment's horizon counted from the solve's time, so the terminal cost comes at `t0 + tf`, the exit rule and the price keeping their one owner | Without it the environment counts the horizon from the problem's `t_start` and charges `h` at the wrong tick from the second solve on | A rollout of MPPI's own (a second owner of the exit rule) | `383d7db9` |
| N-4 | 5 | A planner whose problem has a point `x_start` beside an explicit `X0` box, under the MPC | Refuse it at the MPC's construction, naming the set (the brief proposed pinning `X0` to the measurement) | Pinning means rebuilding the planner's problem behind the user's back; a refusal is explicit, one line, and the MPC's contract is already "a point start" | Pin `X0` to the bound measurement | `193ebeec`…`fd50c735` series: `The MPC refuses a planner whose problem starts from a set` |
| N-5 | 5 | Value iteration given `alpha=` on a problem with `discount_rate > 0` (the review's finding 9: `c_k` keeps `e^{−ρτ}` inside the period while `alpha` replaces the discount between periods) | Keep the override as it is: `test_dp_reads_the_problem_discount_rate` pins "an explicit `alpha` wins". The refusal tried first broke that contract and was reverted | The pinned contract is a deliberate behaviour; the inner discount is second order (`ρ dt` small) | Refuse the pair; or make `alpha` turn off the inner discount | `91039fb0` (refusal) reverted in the next commit |
| N-6 | 5 | The judge on a loop whose plant is held exactly (`solver="exact"`): no stage-cost integral | A refusal that says what to do (discretize with `rk4` or `euler`) in place of "took 0 periods" | The exact hold integrates no path to carry `∫ g`; an exact cost integral (Van Loan's augmented exponential for a quadratic cost) is its own row | The integral now | `91039fb0` |
| N-18 | 5 | `AdamLaw` (flat, projected, `safe_root` for its gradient) and `Adam` (pytree, RL) write the same moments (RULES 5.6) | Keep both | Merging changes the differentiable native kernel's path (its `safe_root` at zero) and its numbers; the two serve different domains | One owner with a projection flag | — |
| N-19 | 5 | The fields' wrappers drop the tree they are given (the review's finding 4) | A design question in QUESTIONS.md, no code change | Threading the field's numbers adds leaves to every problem with a field cost, traced into the collocation programs: a design change, not a fix | Thread them tonight | — |
| N-20 | 5 | The new `SampledDiagramEvaluator._jac_validate_params` (RULES 5.8) | Keep: it is the existing `_jac_*` protocol of four evaluator classes; renaming the protocol collides with the program evaluators' `jacobian_g`/`jacobian_h` | A half-renamed protocol is worse than an allowlisted one | Rename all `_jac_*` | — |
| N-21 | 6 | `core/readme_examples` stays 6 % slower than before the run (10.45 against 9.85 s, median of 3) | Logged as an explained miss | Its sections: the JAX gradient and L-BFGS of the closed-loop cost +0.25 s (the one score's stage cost integrated by RK4 beside the state, R142: a bigger traced graph, the same 12 iterations and `J` to 1e-4), value iteration +0.11 s, collocation +0.04 s, the import +0.02 s; no overhead left to remove without changing the score | Chase the 0.2 s spread across the rest | — |
| N-22 | 6 | The cart-pole RL demo's reliability at its new budget | No change tonight (the maintainer's call: 3M, no cost shaping); the evidence goes to the morning | Tonight's code, 8 seeds, J < 60 as success: 3/8 at 3M (seed 0, the demo's default, swings up, J 1.49), 2/8 at 4M; before the run 9/12 at 2M. More steps do not fix it: some seeds settle in a poor optimum under the integrated reward. A training reward `−g·dt` recorded beside the scored one (as `gamma` is), or tuning the demo, are the maintainer's | Tune the demo's constants (maintainer-owned) | — |

## 3. Ledger of intended number changes

| Step | What changed | Problem and cost | Before | After | Relative difference |
| --- | --- | --- | --- | --- | --- |
| 5 | The Lyapunov certificate's `P` symmetrized after the solve | `benchmarks/v03_baseline.py`'s `lyapunov.level` (the pendulum's certificate) | 4.969173742168747 (its last bit) | the symmetric `P`'s | 1.8e-16 |

## 4. Speed and quality

**Flagship demos, before the run (`8db211c4`) against tonight, median of 3, M4 Max (step 6's first
measure, before its fixes):** the MPC demos faster (`mpc_car_circuit` 10.61 → 8.74 s,
`mpc_racecar_dynamic` 22.93 → 17.46 s, `mpc_racecar_kinematic` 35.70 → 23.47 s, `mpc_car_minimal`
1.70 → 1.73, `mpc_integrator_numpy` 0.71 → 0.73); RL throughput level (cart-pole 50 325 → 52 673
steps/s, rocket 53 004 → 51 239); misses fixed in step 6: value iteration (`vi_quadratic` 6.45 →
7.27 → **6.45**, `vi_minimum_time` 2.88 → 3.23 → **2.92**, `vi_pendulum_swingup` 18.46 → 20.18 →
**18.9**: the one score's Euler stage cost in one line, `f1a3339b`), `trajopt_cartpole_collocation_jax`
2.29 → 2.47 → **2.25** (the JAX program's frozen form, the commit after), `diagram_compiling` 3.15 →
3.44 → **3.18** (the large-leaf check read once, `ab02eb65`); `readme_examples` 9.85 → 10.45
(N-21). The rest within 3 %.

The evening's first sweep (one rep, 72 demos × 0.2.0 / pre-run `8db211c4` / `7c21f63f`, M4 Max):
every demo within about ±10 % except `mpc_car_circuit` 10.8 → 161 s and `mpc_racecar_dynamic`
23.0 → 97.5 s (the animation re-solving the MPC per frame: fixed in `37c45da4`, 13.5 s and 17.9 s
after), `mpc_car_minimal` 1.9 → 3.2 s, the value-iteration demos 10–25 % slower than 0.2.0, and
`compile/neural_controller_jax.py` failing (a diagram's `x0` element write).

## 5. Not done, or blocked

- **`e4.bicycle_parametric.total_s`** (an online tick's whole `solve_trajectory_from`, not
  gated) reads 0.146–0.150 s against its baseline's 0.059 s; it read the same on `7c21f63f`,
  the evening's starting point, so it predates tonight (the run's planner path).

- **The review's latent findings left:** the traced first guess (under a trace `pv.initial(params)`
  falls back to the planner's own start; latent while only native single-shooting programs
  trace), and the delayed form's tick 0 when a run starts away from the problem's `t_start` (a
  poor first seed, no wrong command). Both in the evening review; neither bites a demo.
- **The failure price under an evaluation's `X`/`U` overrides** still reads the problem's boxes
  (the discount rate now follows the evaluation).

- **The SLSQP collocation MPC on the racecar circuit of `mppi_racecar.py`** stalls at 0.18 laps
  in 10 s (J 1.01, 1.2 ms a tick: its solves return at once). The circuit starts heading along
  `+x`, the exact heading where SLSQP stops on a singular matrix (the zero-delay MPC record,
  finding 4, for the pathtracking project); MPPI on the same problem laps 2.09 times. Not a
  regression of tonight; not the flagship demos (they nudge their start heading).

## 6. Textbook check

## 7. Progress

| Step | Row | Commit | Gates |
| --- | --- | --- | --- |
| 0 | The sampled loop animates its plant from its own run | `37c45da4` | ruff; test_mpc, test_sampled_loop, test_graphics, test_teaching_surface, test_v03_core, test_textbook_style |
| 0 | MPC `verbose` | `44129d91` | the same |
| 0 | The maintainer's demo edits (cart-pole 3M, racecar verbose) | `80efe02e` | — |
| 1 | `box % dt` clocks each block with its face kept; the MPC's own `%` gone; a bare tick law refused where a nested box would speed it up | `f39e1010` | ruff; test_sampled_loop (the time table), test_mpc, test_step_discrete, test_v03_core, test_textbook_style; v0.3 baseline byte-identical; full `pytest` green (426 s) |
| 1 | `SampledSystem`'s period is `dt`; viz reads the plant from the plan | `486adff4` | the same |
| 2 | Recording signals in the run; selective reconstruction; the planning evaluator from its own pass | `9ea45d66` | ruff; test_mpc (read-backs solve nothing), test_sampled_loop, test_diagrams, test_graphics, test_engine, test_simulation, test_planning_*, test_compile_once, test_step_*; v0.3 baseline byte-identical; `mpc_car_circuit` 120 solves for 120 ticks; full `pytest` green |
| 3 | The compile cache's exact key; stale-case tests; KNOWN_ISSUES and DESIGN | `8ed9ce00` | ruff; test_compile_once (every stale case, eviction, the remedy), test_mpc, test_sampled_loop; v0.3 baseline byte-identical |
| 4 | `RolloutEnvironment` to `planning/environment.py` (R108); `env.step(t_origin=)` | `383d7db9` | ruff; test_rl_tabular, test_random, test_rl_planner |
| 4 | `MPPIPlanner`, its face, the MPC on it, two demos | `137e5290` | ruff; test_mppi (10), test_mpc, test_textbook_style, test_public_names; both demos run (5.8 s, 1.8 s) |
| 5 | `neural_controller_jax` writes the plant's start | `44248171` | the demo runs |
| 5 | `lookahead` in the zero-delay and pure MPC | `fd50c735` | test_mpc, test_mppi |
| 5 | The MPC refuses a start set; the face keeps its compiled tree; the price follows the evaluation's rate | through `f45ce7ce` | test_mpc, test_engine, test_planning_*, test_v03_core |
| 5 | Lyapunov `P`, the compile probe, a double discount, the exact-hold message, RULES 5.3 | `91039fb0` | test_dynamic_programming, test_analysis_lyapunov, test_costs_optimizer, test_mpc, test_fields, test_textbook_style, test_step_problems, test_v03_core |
| 6 | One Euler step's stage cost in one line | `f1a3339b` | v0.3 baseline unchanged by it; vi demos timed |
| 6 | The JAX program's frozen form | (after `f1a3339b`) | test_costs_optimizer (no retrace per value), test_mpc, test_jax_planning; baseline byte-identical |
| 6 | The large-leaf check read once | `ab02eb65` | test_compile_once, test_simulation, test_engine |
| 6 | `compile_parametric_program` warms the parametric form (the regression gate's `e4.bicycle_parametric.solve_s`) | `2a40c65a` | the e4 suite green (0.0085 s) |
| 7 | The gate on `2a40c65a`'s parent, then the regression gate again | — | ruff; full `pytest` green; regression gates with the CI flags green (after `2a40c65a`); flagship demos and catalog `--fast` green; notebook smoke 34/34; docs build `-W` green |
