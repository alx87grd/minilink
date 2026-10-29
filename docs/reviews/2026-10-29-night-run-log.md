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

(Written at the end.)

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

## 3. Ledger of intended number changes

| Step | What changed | Problem and cost | Before | After | Relative difference |
| --- | --- | --- | --- | --- | --- |

## 4. Speed and quality

The evening's first sweep (one rep, 72 demos × 0.2.0 / pre-run `8db211c4` / `7c21f63f`, M4 Max):
every demo within about ±10 % except `mpc_car_circuit` 10.8 → 161 s and `mpc_racecar_dynamic`
23.0 → 97.5 s (the animation re-solving the MPC per frame: fixed in `37c45da4`, 13.5 s and 17.9 s
after), `mpc_car_minimal` 1.9 → 3.2 s, the value-iteration demos 10–25 % slower than 0.2.0, and
`compile/neural_controller_jax.py` failing (a diagram's `x0` element write).

## 5. Not done, or blocked

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
