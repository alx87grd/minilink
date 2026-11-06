# A diagram run records the plant's input by default

Ruled 2026-11-06 by the maintainer, after two independent reviews of four options. It replaces
the 2026-11-05 bar ("record the plant's input by default only if it costs a few percent at
most"): the record is complete by default, and the run that needs speed says so. The order of
work and "done when" are TODO §1.11's row; this doc holds the design and its reasons, and goes
when the contract is in DESIGN.

## The ruling

A diagram run records the signals its `record=` names, the same rule on every solver and on
both kinds of loop:

| `record=` | the run records |
| --- | --- |
| `"plant"` (the default) | the plant's whole input, `"plant:u"`, when the diagram has a `plant` block; nothing otherwise |
| `None` | nothing: the fastest run |
| `("controller:u", "plant/arm:u", …)` | exactly those, a nested block by its path |
| `"all"` | every output port of every top-level block |

So `traj.of("plant")` is the plant's complete run on every default run, `compute_cost(of=
"plant")` and `evaluate_trajectory` read it, and nothing is rebuilt after the run unless the run
was asked for `None`. A leaf's run records nothing (its `x` and `u` are the trajectory); a
tuple of names on a leaf is refused.

`"block:u"` names the block's whole input when the block has no output port named `u`: the
input the diagram fed it, every port stacked in port order, what `traj.of(block).u` returns.

## Where the signals come from

- **A fixed-step run** (`rk4_fixedsteps`, `euler_fixedsteps`): the step's own pass. The first
  stage of each step is `f(x_k, u_k, t_k)`; the diagram evaluator fills its signal buffer
  there, reads `k₁` and the recorded slices off it, and takes the other stages: no extra
  evaluation of `f`, the states bit for bit the unrecorded run's. The last sample, which no
  step starts from, is one more pass.
- **An adaptive run** (`scipy*`) and the variable-step `euler`, and a forced run read at the
  stage times (`u` a function): a pass after the run, at the saved samples, inside the
  `Simulator`, on the numbers the run was made with. The adaptive run's saved samples are
  interpolations: the signals there were never computed, so they are computed there.
- **The sampled loop**: its scan's own signal pass, as today; its forced record of the plant's
  input becomes the same default, and `record=None` turns it off.

The rebuild is inside the run, on the run's numbers, so a held `Simulator` solved at other
numbers (`sim.solve(params=p)`) or a block edited after the run never gives a stale signal.
The after-the-fact readers (`reconstruct_internal_signals`, `block_run`, the plots, the cost on
a run made with `record=None`) stay, document that they read the blocks' current numbers, and
take `params=` for a run made at others.

## Why, and the cost

The empty input of an unrecorded run was the one seam of the block selector: `traj.of("plant")`
complete on one run, `(0, N)` on the next, and the evaluators taking it as the plant's run. Only
a default record closes it, and only a rebuild inside the run closes the stale-parameter
hazard. The reviewers ranked the options C (this ruling) > A (opt-in) > D (a second selector on
the diagram) > B (free where possible: presence would hang on the hidden automatic solver
choice); the second reviewer ranked A first under the old bar and C first once the bar is
reopened.

The cost, measured 2026-11-06 (adaptive default solver, 10 001 samples): PID @ Pendulum 5 ms
run, 194 ms with the record, 525 ms run and default plot; LQR @ CartPole 1 882 ms run, 1 905 ms
with the record. A fixed-step run records for free. The callers that need speed (a hand sweep,
the benchmarks, a library loop) pass `record=None`.

## What does not change

`record=None` is today's unrecorded run, the same program: on JAX the unrecorded scan traces to
the same jaxpr. The problem evaluator, the planners' faces and the MPC never go through
`compute_trajectory` and are untouched. The parameters baseline and the step-model baseline are
byte-identical.
