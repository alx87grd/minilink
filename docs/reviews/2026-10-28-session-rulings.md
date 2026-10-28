# Rulings of 2026-10-28, session 1

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the six questions the independent review of the v0.3 core plan left
([the review](2026-10-27-independent-review-core-architecture.md) §7), taken one by one in
session. Each entry records the maintainer's choice and how it was reached. The questions on
time in the core (the clock port and what `t` is for every block) are under three independent
reviews and are recorded here when ruled.
**Status:** the architecture of record (R166): the "preliminary" caveat of R124–R158 is lifted.

| Ruling | In one line | Design home |
| --- | --- | --- |
| R154 | `discretize(problem, dt)` gives its `StepPlanningProblem` a `SampledCost`, the one-period integral of `g` as a `StepCost` | [differentiable-problem](../plans/differentiable-problem.md) §9.1 |
| R155 | The plan's origin is one entry of `z`; the MPC's state is `z` alone; `shift(z, t)` rebases to an absolute time | [planner-and-mpc](../plans/planner-and-mpc.md) §3, §6.2 |
| R156 | A performance budget: a benchmark script, the ninth item of the definition of done, 2× on five measured numbers | [core-architecture](../plans/core-architecture.md) §11 |
| R157 | `dt` at the calls, `sample_period` on the blocks that hold a clock | DESIGN §4 |
| R158 | Time belongs to the loop; `SampledSystem` the sampled-data kind with two faces; the MPC its first native child; no clock port | [sampled-path](../plans/sampled-path.md) §8.1–§8.2, [planner-and-mpc](../plans/planner-and-mpc.md) §6 |
| R159 | The one-score's three points: the sampled check, priced to the horizon with the input held, a solver's own objective allowed but the score the problem's | [differentiable-problem](../plans/differentiable-problem.md) §9.4 |
| R160 | The RL heads rebuilt on the seeded policy in v0.3, training performance unchanged as the gate | TODO §1.4 |
| R161 | The realtime runner's re-base on the pure blocks is a v0.4 feature; the v0.3 loop a thin caller | TODO §4 |
| R162 | The left operand of `@` is `controller` in both algebras, block or box | [automation-by-convention](../plans/automation-by-convention.md) §4.5 |
| R163 | The conventions' first sampled-loop step (AC-8a) folded into the loop lane | TODO §1.5 |
| R164 | The names: `update`, `rollout_dense`, `pv.command`, `z`, `PlanFollower`, `inputs=`, `delay=`, `lookahead=`, `t_start` everywhere, `program.unpack(z)` | [planner-and-mpc](../plans/planner-and-mpc.md) §11 |
| R165 | The failure price follows overridden cost weights (cached); the `as_constraint` → `as_set` rename given; plain exceptions for the wiring-time refusals | TODO §1.7, §1.9, §1.10 |
| R167 | The notebooks' code moved to the new API and debugged at the end of the run; no new notebook; four acceptance scripts and six new demo scripts added | core-architecture §11, the run brief |
| R166 | The architecture of record: R124–R165 confirmed, the go for every lane, the "ask, core" marks removed; the 10-24 run's decisions reviewed with the new run's log | TODO §1, QUESTIONS |

## R154 (2026-10-28, preliminary) The discretized problem's cost is a `SampledCost`

The question: R141 made `discretize(problem, dt)` return a `StepPlanningProblem` "with no cost
object of its own", its `c_k` produced by `transition` (R142), so the class had two shapes, a
native one with a `StepCost` and a sampled one whose `cost` a user could not call.

- **The choice:** option (a). `discretize(problem, dt, *, solver=, substeps=)` builds
  `SampledCost(g, h, plant, dt, *, solver=, substeps=)`, a `StepCost` whose `c(x, u, k)`
  integrates `e^{−ρτ} g` over one period from `x` under the held `u` when called directly, and
  whose `h(x, k)` is the terminal cost at `t_k`. The compiled face fuses it into the transition
  (R142), so the direct call is the only second plant evaluation.
- **The maintainer's precision:** the direct call uses the same `solver` and `substeps` as the
  plant's `SampledSystem`, so `sampled.cost.c(x, u, k)` and the fused transition agree to the bit;
  one test pins it.
- **Why:** R142's integral stated as an object; R141's "two classes by their fields" kept honest;
  about forty lines.
- Rejected: `problem.compile(dt=)` as the one entry in v0.3 with the explicit discretization
  deferred to v0.4 (the safe fallback, not needed); `cost = None` documented.

## R155 (2026-10-28, preliminary) The plan's origin lives in `z`; the MPC's state is `z` alone

The question: R148 had `pv.solve` record in `z` the origin it solved from and the delayed MPC
keep `t_last` in its own state, `n = pv.n_z + 1`: one number, two homes.

- **The choice:** option (a), after checking the packing of every planner. The origin is one
  entry of `z` (a float; an integer tick stored exactly on a step problem); the MPC's state is
  `z` alone, `n = pv.n_z`; `shift(z, t) -> z` rebases the plan to the absolute time `t` (the
  planner's data moved by `t − origin` on its own knots, the origin set to `t`), the identity for
  a time-invariant solution; `policy(z, x, t)` reads absolute time, as every equation path does.
  A failed solve keeps `z` with its old origin and the next tick rebases from it, no special case.
  The MPC reads nothing inside `z`.
- **The evidence:** no planner stores a time vector in `z`. Collocation packs `(n + m)·N` knot
  values, shooting `m·N` inputs, and the knot times are `linspace(0, tf, N)`, structural
  (`transcription.py`); MPPI's `U`, RRT's resampled path, the finite-horizon LQR's `K(t)` samples
  and every table are on grids of the same kind, `origin + fixed offsets`. So the origin is the
  only time in any `z`, and it duplicates nothing.
- **Why (a) over (b)** (`z` time-free, the block owning `t_last`): the maintainer's criterion,
  "`z` should be enough to use the plan", which R148's definition of `z` ("everything it needs to
  continue") already states; under (b) every consumer of `z` (the dual-rate follower, a logged
  plan) must be handed the origin beside it, and the follower needs a second port for it.
- Amends R148's delayed form: the state `(z, t_last)` becomes `z`; `shift(z, dt)` becomes
  `shift(z, t)`.

## R156 (2026-10-28, preliminary) A performance budget for the core

The question: the parametric program's gate ("if carrying `params` costs speed, an export option
bakes them in") had no number, and the definition of done no budget.

- **The choice:** option (a). A ninth script in the definition of done, run as a benchmark at the
  merge with its numbers in the report, not a CI gate: the compiled sampled loop, the one score
  and the native MPC's closed loop each within 2× of the vision's measurement at the same size
  (44 ms, 93 ms, 134 ms; CPU, float64, the vision §7.5); the parametric program's solve within 2×
  of the same program with its parameters frozen; a parameter family within 2× of the plain batch
  (S53's bar).
- **Why:** the numbers and the scripts exist; 2× passes a correct implementation without tuning
  and fails a design mistake (a retrace per tick, a Python loop left in) by an order of magnitude.
- Rejected: no budget; only the parametric gate quantified.

## R157 (2026-10-28, preliminary) `dt` at the calls, `sample_period` on the blocks

The question: the calls say `dt` and the objects say `sample_period` for what looks like one
quantity; R134's rule is one keyword per concept.

- **The choice:** option (a), two words for two things. `dt` is the step of a computation, at every
  call (`discretize(sys, dt)`, `sys % dt`, `problem.compile(dt=)`, the evaluator's `step(…, dt=)`);
  `sample_period` is the clock a block holds, on every such block (the sampled system, the noise
  sources, the seeded policy). `discretize(plant, dt, substeps=4)` already separates the period
  from the integration step underneath. One line in DESIGN §4.
- Rejected: `sample_period` at the calls (a long keyword in the student's line, a mismatch with
  the evaluator's `dt=`); `dt` on the blocks (a noise block's period is not an integration step,
  randomness RD-R3).

## R158 (2026-10-28, preliminary) Time belongs to the loop; `SampledSystem` is the sampled-data kind; no clock port

The question, as it grew in session: R148 gave the MPC a standard input port `t` so that a pure
`StepSystem` could read seconds. The maintainer asked what `t` is in `(x, u, t; params)` for every
block in every diagram, and rejected in turn a unit declaration on `StepSystem`, a wrapper passing
seconds into a tick law, three static kinds and a doubled static catalog. Three independent
reviews were run on the converged proposal ([A, the textbook and the student](2026-10-28-time-in-the-core-reviewer-a.md);
[B, the implementation, with prototypes](2026-10-28-time-in-the-core-reviewer-b.md);
[C, the future and the adversary](2026-10-28-time-in-the-core-reviewer-c.md)). All three kept its
skeleton and corrected the same two points; the maintainer ruled the amended design.

- **The principle.** Time is one channel, the third argument of every equation; what it holds is
  decided by the diagram a block sits in, never by the block: seconds wherever the diagram has a
  clock (continuous, sampled), the count where it has none (a pure step diagram). A tick law
  (`StepSystem`) reads its own count and never seconds. No block declares a unit or a clock, and a
  block's `params` never carry a sample time. One static catalog serves every diagram.
- **`SampledSystem` is the sampled-data kind,** `x(kh + h) = F(x(kh), u(kh), kh)`: `update(x, u, t)`
  in seconds is its state equation and its output ports carry their own functions in seconds, as on
  any system (amended the same day: the maintainer noted a block has many outputs, each a port, so
  there is no `output` method; `h` is the standard `y` port's function); `step(x, u, k)` and `h(x, u, k)` are its tick
  faces for the step algebra, `t = k·sample_period`, the origin 0, never overridden. The views
  (`discretize`, `%` on a continuous, a static or a tick source) write the seconds face from their
  source; a tick source keeps only its tick face. **A native sampled law subclasses it with no
  source** (the delayed MPC first; a discrete Kalman filter with a time-varying model, MPPI, an
  action chunk later); `%` on it sets its period. Revises R146's "nobody subclasses it". A sampled
  static block has `n = 0`.
- **The sampled diagram is the clock.** It stays a step diagram, one pure map on the base tick;
  its compiled plan holds the divisors and the hold slots, computes `t_k = t_start + k·Δ` once per
  tick and hands it to every block whose equations are in seconds (a static block, a seconds face),
  its own count `k // d` to a tick law under `%`, the base count to a bare tick law; the per-face
  decision is made at build by base class, as port ops and step ops already are. The pure step
  diagram's plan is untouched. `Sine() >> (pid % Δ) @ plant` equals `(Sine() % Δ) >> (pid % Δ)
  @ plant` bit for bit, so `%` on a static block means its period and nothing else.
- **The origin is the run's,** the time twin of `x0`: `compute_rollout(n_steps, t_start=0.0)` and
  the evaluator's `rollout`, bound by `problem.compile` from the problem's `t_start` leaf (R148),
  where the gradient and the Monte Carlo over it live; never a key of `params`, never read by an
  equation from an attribute. (The reviewers' option C; A and B wanted a leaf of the loop's tree,
  rejected as a second home beside the problem's leaf.)
- **Nesting:** a sampled box inside another is inlined into one flat plan; one clock, the gcd of
  every leaf's period. **The firing** is a `lax.cond` on `k mod d` or the hyperperiod unroll,
  never a `where`; the contract says only "fires on `k mod d = 0`, outputs held".
- **The MPC** is a native `SampledSystem`: its `u` port's function is `pv.policy(z, y, t)`,
  `update(z, y, t)` binds `t_start = t`, rebases and solves (R155). `inputs` defaults to
  `{"x_start": "y"}`; delay compensation is `lookahead=Δ` with a predictor bound to `x_start`.
  **The `t` port of R148 goes**, and with it the clock source block and automation-by-convention's
  §4.6 as written. "The MPC never knows `Δ`" becomes "its equations never read `Δ`".
- **A problem on a clock keeps seconds:** `discretize(problem, dt)` keeps `(dt, t_start)` in
  seconds and its transition forms `t_k = t_start + k·dt`; only a native step problem has an
  integer origin (amends R148's sentence).
- **D15 closed:** `problem.compile(controller, dt=)` on a sampled controller reads the loop's
  base period when `dt=` is omitted, and refuses a `dt=` that differs, naming it.
- **Why the reviews' corrections were taken:** under the earlier bullet (a bare static block in a
  sampled loop reading the count), `Sine(omega=1) >> (pid % 0.01) @ plant` ran its reference
  100× too fast silently, a `WhiteNoise` with its own period drew a fresh sample every base tick,
  the seeded policy's `⌊t/Δ⌋` broke common random numbers against the continuous evaluator, the
  gradient with respect to `t_start` was zero through every bare reference, and the host runner
  and the scan disagreed on any bare reference (C §3.2, A §3). And `t_start` "written through to
  every clocked block" had no pure home (A, B, C alike).
- **Draws, stated once:** a seconds reader draws from `(seed, ⌊t/Δ⌋)` of its own period, a tick
  law from `(seed, k)`.
- **Names for the sweep:** `update` (`advance` the alternative; outputs are ports); `t0`
  and `t_start` one word for the run's origin on every runner.

## R159 (2026-10-28) The one-score's three points

1. **Between the samples:** accepted as the sampled check. The sets and costs are evaluated at the
   sample times; the score is the score of the sampled loop, which is all the controller sees; a
   substep check may exist as a diagnostic on `compute_trajectory`, never inside the score.
2. **The failure rule and the hold:** a run is priced to the horizon, every violation priced
   (today's judge); the RL environment may still end an episode early as a training device but
   reports the judge's `J`; the input is held over each period (zero-order hold) unless a planner
   states its own interpolation. Ending the run at the exit would make `J` jump when a parameter
   moves the exit by one sample.
3. **A solver's objective, the maintainer's generalization:** a solver may optimize an objective
   that is not the problem's cost (a smoothed cost, a first-order hold, a training discount as in
   R150), but a solution's score is always the problem's cost through the common step function,
   and a planner whose objective differs says so in its record. Shooting therefore reports the
   judge's `J`.

## R160 (2026-10-28) The RL heads rebuilt on the seeded policy in v0.3

Rebuilt with the block, in the sweep that already touches every RL notebook (R143, R150, R151),
not in v0.4 as the reviewers of 10-26 leaned. **The gate** (the maintainer): training performance
is unchanged after the move, on the same demo with the same seed count: the learning curve and
the final score within the Monte Carlo spread, the training time within the budget's 2× bar
(R156).

## R161 (2026-10-28) The realtime runner's re-base is a v0.4 feature

The host clock runner serves the scipy cross-check and host-only blocks in v0.3; the realtime
runner (game mode, hardware in the loop) is re-based on the pure blocks as a v0.4 feature row,
with one condition in v0.3: the realtime loop calls the blocks' faces (`update`, the port
functions, `step`) and never re-implements a hold or a divisor, so it is a thin caller when its
row comes.

## R162 (2026-10-28) `controller` is the left operand of `@`

In both algebras, whatever the left operand is, one block or a box of blocks on clocks; a box's
leaves are reached through it (`controller.filter.tau`, `controller.law.Kp`). No `computer` role:
the word stays the runner's and the drawing's.

## R163 (2026-10-28) The conventions' first sampled-loop step is folded

AC-8a (standard port names only, the MPC gives `u`, no port guessing) edited code the loop lane
deletes; its intent is the lane's "done when".

## R164 (2026-10-28) The names

`update(x, u, t)`, the seconds state face of a system on a clock (outputs are ports, no verb);
`rollout_dense`, the evaluator method behind `compute_trajectory` on a sampled system;
`pv.command(z, x, t)`, the compiled planner's pure command function (the maintainer: `policy`
collides with `solution.policy`, the block; `command` is the library's own word, `command_port`);
the letter `z`; `PlanFollower`; `inputs=`; `delay=`; `lookahead=`; `t_start` for the run's origin
on every runner (today's `t0` renamed in the sweep); `program.unpack(z)` for the export's full
tuned tree (was `params_at`; the evaluator already has `pe.unpack`).

## R165 (2026-10-28) Three details from the triage of the core's rows

- **The failure price under cost overrides** follows the overridden weights, with a cache keyed on
  the cost's parameters; a price that does not move with the cost it is derived from is a surprise
  in a tuning loop.
- **The `as_constraint` → `as_set` and `as_input_constraint` → `as_input_set` rename** with every
  caller, the corridor and racetrack demos included, is given (student-facing).
- **The wiring-time refusals** use plain `ValueError` and `TypeError` with clear messages; the
  typed error family (S70) stays where it is planned.

## R166 (2026-10-28) The architecture of record, and the go

- R124–R165 are the architecture of record for the v0.3 core; the "preliminary, open to
  readjustment with the final architecture" caveat is lifted from every doc and record.
- Every row of TODO §1 has the go; the 24 "ask, core" marks are removed. The triage found no
  design decision left in them; the internal choices an agent meets (the default output stub's
  type under JAX, the open-loop compiled face as the evaluator or a sibling, the hold slots'
  labels, helper names) are decided as the maintainer would and logged in the run's record, the
  10-24 rule.
- The 10-24 overnight run's open decisions (OD-7 to OD-35) are reviewed in the morning after the
  new run, both logs together.

## R167 (2026-10-28) The notebooks at the end of the run; four more acceptance scripts

- At the end of the run the notebooks' code is moved to the new API and debugged (the notebook
  smoke check), every notebook, as the run's last step after the material and contracts; the
  racecar MPC notebook goes with that sweep and the racecar demos are given. No new tutorial or
  demo notebook is written on the new API.
- The definition of done gains four scripts (10 to 13: the multi-rate computer, a bare reference
  reading seconds in a sampled loop, a loop started at `t_start = 100`, the RL heads' gate) and
  the dual-rate clause on script 5; they live in `tests/unittest/test_v03_core.py`, the budget in
  `benchmarks/v03_budget.py`.
- **Six new demo scripts on the new pipeline, given** (the maintainer: "the final proof the new
  core architecture is working"): the digital PID tuned, the multi-rate computer, the LQG Monte
  Carlo, a grid world as a step problem, the native MPC tuned by gradient, a terminal field; in
  `examples/demos/compile/`, minimal and flat like the PID autotuning demo, each written at the
  seam that makes it possible and added to the flagship manifest (the run brief §8). The demo
  pass (K4) merges or trims them later.
