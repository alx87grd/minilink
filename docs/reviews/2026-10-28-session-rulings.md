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
