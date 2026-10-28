# Rulings of 2026-10-28, session 1

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the six questions the independent review of the v0.3 core plan left
([the review](2026-10-27-independent-review-core-architecture.md) §7), taken one by one in
session. Each entry records the maintainer's choice and how it was reached. The questions on
time in the core (the clock port and what `t` is for every block) are under three independent
reviews and are recorded here when ruled.
**Status:** preliminary choices, like R141–R153, open to readjustment with the final architecture.

| Ruling | In one line | Design home |
| --- | --- | --- |
| R154 | `discretize(problem, dt)` gives its `StepPlanningProblem` a `SampledCost`, the one-period integral of `g` as a `StepCost` | [differentiable-problem](../plans/differentiable-problem.md) §9.1 |
| R155 | The plan's origin is one entry of `z`; the MPC's state is `z` alone; `shift(z, t)` rebases to an absolute time | [planner-and-mpc](../plans/planner-and-mpc.md) §3, §6.2 |
| R156 | A performance budget: a benchmark script, the ninth item of the definition of done, 2× on five measured numbers | [core-architecture](../plans/core-architecture.md) §11 |
| R157 | `dt` at the calls, `sample_period` on the blocks that hold a clock | DESIGN §4 |
| R158 | Time belongs to the loop; `SampledSystem` the sampled-data kind with two faces; the MPC its first native child; no clock port | [sampled-path](../plans/sampled-path.md) §8.1–§8.2, [planner-and-mpc](../plans/planner-and-mpc.md) §6 |

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
