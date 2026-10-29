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
