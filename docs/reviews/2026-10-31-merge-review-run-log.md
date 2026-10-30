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

In progress (written before each next row starts).

The environment: a cloud container with no conda; the `dev` and `full` extras installed by
pip (JAX 0.12, NumPy 2.5, Python 3.13), no Ipopt, no Stable-Baselines3 or torch. The v0.3
baseline (`benchmarks/v03_baseline.py`) was captured twice before the first edit and the two
files were byte-identical (116 entries).

## 2. Decisions taken without the maintainer

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
  the same errors are 1.5e-8, 1.8e-10, 1.5e-12, 1.8e-14: fourth order. `f(x, u, t, params)`
  keeps its one signature (CONSTITUTION §2). A held block inside a nested sub-diagram still
  reads the stage time (`flatten=True` reaches it); a sampled loop's plant view and the step
  face call the plant's own `f` and are not touched (no demo holds a noise source inside a
  plant). *Alternative:* every stepping right-hand side taking the step's start as a fourth
  argument (a dozen call sites, the trajectory-optimization lambdas among them), or a last
  stage nudged below `t + h` (not exact, and it moves every time-varying RK4 run).
- **M-2 (row 1) One commit for the row.** The plan's step 2 says "the twin test and the RK4
  held-sample fix, alone, with its ledger"; the session's rule is one commit per row. The fix
  was measured alone (the baseline after it and before the rule: only `noise.white.x` moved)
  and has its own ledger line; it shares row 1's commit. *Alternative:* two commits for the
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

## 3. Ledger of moved numbers

| Row | What moved | By how much | Why |
| --- | --- | --- | --- |
| 1, the RK4 held sample | `noise.white.x` of the v0.3 baseline; re-pinned in `benchmarks/baselines/v03_core.json` (that key only) | 3.9e-3 relative | the four RK4 stages read one sample: the last no longer reads the next draw (M-1) |
| 1, the step rule | no v0.3 baseline entry and no regression gate (every one names its step); the demos and notebooks of §7 | §7 | R180 |

## 4. Textbook check

Every math line of each row was read beside `planning/dynamic_programming/dp.py` (RULES §5:
5.3's blocks, 5.8's sections, 5.22's comments, 5.24's timeless prose). Row 1: `automatic_step_of`
and `fastest_time_constant` (`core/system.py`), each plant's `fastest_mode()` (bindings, one
parameter per line, the linearization a named matrix assembled row by row, a named result
returned), the `solver_info` properties (the racecar's pattern), `held_step_body`
(`step_body`'s substep loop, each step named). No equation sits on a `return` line or in a
helper away from its page.

## 5. Not done

(filled as the run goes)

## 6. Progress

| Row | Commit | What landed | Gates of the row |
| --- | --- | --- | --- |

## 7. Row 1's sweep: the automatic step, before and after

Every demo, project and notebook was run with an out-of-tree instrumentation that records each
automatic-step decision (a fixed-step simulation with no `dt`, a sampled loop's plant with no
`substeps`, the planners' evidence and `problem.compile(law % dt)` through the loop), on the
base tree. 22 files hit it; the v0.3 baseline and the regression suite never do (every entry
names its step). Before/after (wall time of the whole file, run interleaved in one session,
two at a time; the plant's step):

(filled when the interleaved runs finish)
