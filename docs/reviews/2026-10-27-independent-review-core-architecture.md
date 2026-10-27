# Independent review of the v0.3 core plan (2026-10-27, evening)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Object:** [docs/plans/core-architecture.md](../plans/core-architecture.md) after today's pass
(R141–R153 recorded, the plan docs brought onto them, the consistency fixes; 48 commits on
`dev-v0.3`), read against the rulings records, the topic plan docs, TODO §1 and the code.
**Status:** an agent's review, not a ruling. Nothing here changes a doc; each point names where
a change would go.

## 1. Verdict

The plan is good, and better than the material it consolidates: one pattern (a description,
and its compiled pure face) now explains systems, problems and planners alike; every workstream
has a goal, a status, an honest "today" and its rows; the definition of done is eight scripts;
the number changes are sequenced. The "today" paragraphs are accurate against the code (checked:
`Set.params = None`, the step evaluators' double signal pass, `require_continuous`, the MPC
latch, `ParametricMathematicalProgram` varying `x0` only, the thirteen `NeuralPolicyController`
files). The risks are not in the ideas. They are five places where the text contradicts itself
or glosses a point the code will force, and three process risks that the plan inherits.

## 2. Corrections (the text against itself)

1. **The class tree of R146 is drawn wrong.** `SampledDiagramSystem` sits under `SampledSystem`
   in core-architecture §0, sampled-path §8.1 and R146. A `SampledSystem` is a view of one
   `source`; a `SampledDiagramSystem` has blocks and wiring and is built by the assembly code
   ("the result is a `StepDiagramSystem`, or a `SampledDiagramSystem` when a block is clocked",
   sampled-path §8.4). It is a `StepDiagramSystem` with a base period, not a `SampledSystem`.
   The drawing decides which mixins it inherits, so fix it before the loop lane starts.
2. **The number-change sequence (§10) contradicts the lane order.** The MPC's one-period delay
   (R123) is listed in group 1, "the sampled loop", but it lands with the MPC rebuilt on the
   planner's face, Join 2, after the one-score `J` (group 2). And the MPC demos run through
   `Computer` today, so their numbers move with the same-tick semantic (R125) first, then with
   the judge's `J`, then with R123: three moves, not one. Either say so, or pin the MPC demos
   once at Join 2 and keep them out of the earlier re-pins.
3. **§5's "Open" list is short.** D15 (whether `problem.compile(controller, dt=)` on a sampled
   controller must equal the loop's base period or reads it) is open in sampled-path §8.7 and
   TODO §1.5 but absent from §5; so is the `core → simulation` lazy import once the hybrid
   modules go.
4. **Definition of done, script 6.** "A continuous problem's value iteration and judge giving one
   `J`" holds only if the judge scores the lookup policy *sampled* (`policy % dt`, R105). Today
   the gap between value iteration and `evaluate` is the hold as much as the quadrature:
   `planning/evaluator.py`'s `nominal_trajectory` holds `u_k` over the period while `evaluate`
   closes `controller @ plant` as a continuous diagram and re-evaluates the law at the RK4
   stages. §6's "Today" names the quadrature and the failure rules only; name the hold, and make
   script 6 say "through the sampled face".
5. **One concept, two keywords.** `discretize(sys, dt)`, `problem.compile(dt=)`, `discretize(problem,
   dt)` keep `dt`, while the object's attribute becomes `sample_period` (R146). R134's own rule
   is one keyword per concept. Decide it in the names sweep.

## 3. Design points the plan glosses

- **A. The discretized problem's cost (R141).** `discretize(problem, dt)` returns a
  `StepPlanningProblem` that "has no cost object of its own", its `c_k` fused into the
  transition. The class then has two shapes: a native one with a `StepCost`, a sampled one with
  none. Two ways out, both cleaner than the gloss: a `SampledCost(g, h, plant, dt)` whose
  `c(x, u, k)` integrates over one period when called directly (a second plant evaluation only
  on that direct call; the compiled face still fuses it), so the class keeps one shape; or
  `problem.compile(dt=)` the one entry in v0.3 and the explicit `discretize(problem, dt)` a v0.4
  row. List it as open in §6; it is not settled by R141 as written.
- **B. Two owners of the plan's time (R148).** `z` "records the origin it solved from" and the
  MPC's state also carries `t_last`; `shift(z, t_k − t_last)` then uses both. One number, two
  homes. Choose one: the MPC's state is `z` alone (`n = pv.n_z`), `shift(z, t)` rebases the
  plan to an absolute origin, and a failed solve keeps `z` with its old origin, which is the
  same fallback with no `t_last`. Simpler, and the MPC still never reads inside `z`.
- **C. The `t` port is a second channel for time.** Every equation path already takes `t`
  (CONSTITUTION §2, "One signature"); the port exists because the delayed MPC is a pure
  `StepSystem` that sees `k`, not seconds, and "never knows `Δ`". The price is a wiring
  convention every time-reading step block will need (a reference tracker, a scheduled gain,
  the MPC). The alternative is for the `SampledSystem` wrapper, which does know `Δ`, to hand its
  source the clock (`t = t_start + k·Δ`) at compile, as it already binds `t_k` for a continuous
  source. A question for the maintainer, since R148 is preliminary; the port is defensible, but
  it should be chosen knowingly.
- **D. "The same transition" is not literally the closed-loop scan.** With a controller that
  has state, the judge's scan is the loop's own step with the plant block augmented to
  `(x_next, c_k)` inside its substep body; that splice into the compiled loop (and the hold
  slots and divisors inside `plan.py`) is exactly what the vision's prototypes did by hand and
  lists as "not verified". §6 should say that `problem.compile(controller)` augments the plant
  block, and §10 should carry the vision's "not verified" list as the spikes to run before each
  lane starts (the hold slots in the plan; `solver="scipy"` between held inputs; Ipopt inside a
  tick; the DP sweep on `transition` under JAX).
- **E. The performance gate has no number.** §7 says "if carrying `params` costs speed, an
  export option bakes them in"; the definition of done has no budget. One line fixes it: the
  compiled sampled loop and the parametric program within a stated factor of today's timings
  (the vision §7.5 measured 44 ms for 1000 × 200 RK4 steps and 93 ms for 100 × 3000 ticks).

## 4. Process risks

- **F. The decision load is still the critical path** (reviewer C §2.6). Of the 43 open rows
  of TODO §1, 25 are marked **[ask]**, 23 of them **[ask — core]**. Now that the shapes are
  ruled (R141–R153), most of those asks have nothing left to ask. One go per lane from the
  maintainer, with the asks kept only where an option remains (A above, D15, the three
  one-score points), would let the lanes start.
- **G. The CONSTITUTION contradicts the plan and the amendment is deferred.** Invariants 1 and
  3 and §6's grammar sentence say the sampled loop is a simulation tool, not a `System`, and
  that `%` schedules a block into a `Computer`; R146 says the opposite. AGENTS.md makes every
  agent read the CONSTITUTION first, so the first agent on the loop lane meets a conflict the
  rules say to stop on. Draft the four sentences now (invariant 1, invariant 3, §6's `%`, §2's
  disturbances) as a proposal under docs/reviews and have them ruled before the lane starts.
- **H. The names sweep and `core/params.py` are a serial step 0, not part of a lane.** They
  touch every lane's files (`planning/evaluator.py`, `problems.py`, `jacobian.py`,
  `assembly.py`; `compile_backend` → `backend` across the API). The vision §10 had it right
  ("before any new signature"); core-architecture §10 folds it into the objects lane and calls
  all three lanes ready. They are ready after step 0.

## 5. Small notes

- `jax.grad(f, allow_int=True)` (R152) returns `float0` leaves for the seed that break the
  next tree arithmetic (`p − lr·g`); the DESIGN line should say it is for reading a gradient,
  and that tuning goes through `wrt`.
- The base period as "the gcd of the blocks' periods" (R146) is a rational rule on floats
  (`0.1 / 0.01` is not an integer in binary); today's `Computer` rounds to `1e-9`
  (`rate_to_divisor`). Write the tolerance down with the row.
- A fixed-`K` native solve has no failure, so the delayed MPC's `where` fallback is a host-only
  branch; say so in planner-and-mpc §6.2, it simplifies the traced tick.
- `evaluate_trajectory` keeping the trapezoid while `evaluate` scores `c_k` leaves two `J`s for
  one logged run by design (R142); a logged run that came from the step model at the same `dt`
  could be scored with the same quadrature, which makes the exception smaller.
- Script 1 of the definition of done (the LQG loop) is continuous; nothing checks a noise block
  inside a `SampledDiagramSystem` realized per trial, which is the draws × loop join. One
  script, or one clause on script 3.

## 6. What to keep as is

The description-and-compiled-face pattern (§1 of planner-and-mpc); the stage cost in the
stationary form (R142); `realize` as one tree (R144); periods on blocks with the schedule
derived (R146); the two verbs that agree (R147); `iterations=K` on the optimizer (R149); the
definition of done as scripts; the "today" paragraphs; the lanes replacing the phases.

## 7. Where each point went (the same evening)

Fixed in the docs, one commit: the R146 class tree (§2.1; sampled-path §8.1 and the record's
drawing, with a note); the number-change sequence and the MPC demos pinned once at join 2
(§2.2); §5's open list (§2.3); script 6 through the sampled face and the hold in §6's "today"
(§2.4); the augmented plant block stated in §6 and the vision's "not verified" list as spikes in
§10 (§3.D); step 0 before the lanes (§4.H); the fixed-count solve's fallback (§5); the ratio
tolerance on the periods row (§5); the `allow_int` line in DESIGN §6 (§5); the draws × loop
clause on script 1 (§5); the one-go-per-lane ask at the head of TODO (§4.F).

To the maintainer, as entries of QUESTIONS.md: the discretized problem's cost (§3.A); one owner
of the plan's time (§3.B); the clock port `t` (§3.C); the CONSTITUTION amendments, with their
text (§4.G); a performance budget (§3.E); `dt` or `sample_period` (§2.5).
