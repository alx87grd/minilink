# The planning solution: one result for every planner

Status: teaching surface, design converged with the maintainer (2026-09-12),
implementation postponed. The RL module cleanup prepares the RL planner for it.

## Problem

Planners return a `TrajectoryPlan` or a `PolicyPlan`, each wrapping a `SolveMetadata`.
Neither reads like the textbook, and they make two planners on one problem look unrelated.

- The closed-loop plan holds neither half of the optimal-control pair. `PolicyPlan.policy`
  is untyped: dynamic programming stores its tables there, RL a dictionary. The policy
  actually comes from `get_controller()`, and DP's cost-to-go from `value_at()`.
- The cost, the one domain fact every solve has, is an optional field in a bag beside a
  message, a solve time and a free-form `stats` dictionary.
- Solver internals ride on the returned object: the NLP warm start, flat vector packing,
  and two reserved rate fields nothing outside the library reads.
- Value iteration discards its plan and is used through planner methods; trajectory
  optimization is used only through its plan.
- Dynamic programming and RL cannot use `plot_solution` or `animate_solution`, which need
  a stored trajectory.

What callers read today, across the library, examples, tests and benchmarks: `.trajectory`
overwhelmingly; `success`, `cost`, `message` and solve time occasionally; `warm_state`,
knot rates, flat packing, `stats` and the policy payload almost never outside the library.
The four GRO860 notebooks touch only `planner.get_controller()`.

## Design

The GRO860 notes define the solution of an optimal control problem as a policy and its
cost-to-go. An open-loop action sequence is the special case of a policy that ignores the
state. Every planner therefore returns one `PlanningSolution`:

| Field | Type | Meaning |
| --- | --- | --- |
| `policy` | System | feedback `u = π(x)`, or open-loop `u = π(t)` as a source block |
| `trajectory` | `Trajectory` | what the policy produces from the problem's start, on the planner's control grid, nominal parameters |
| `evaluation` | `Evaluation` | the cost over the problem's draws under the one scoring contract; one trial for a deterministic problem |
| `cost_to_go` | function of state, or `None` | J*(x) where the method produces it |
| `solver` | typed record per planner | details a user compares: iterations, convergence, history |

| Field | Trajectory optimization | RRT | Dynamic programming | RL |
| --- | --- | --- | --- | --- |
| `policy` | open-loop `π(t)` | open-loop `π(t)` | lookup-table `π(x)` | neural `π(x)` |
| `trajectory` | the optimized one | the path found | policy rollout | policy rollout |
| `evaluation` | over the problem's draws | same | same | same |
| `cost_to_go` | `None` | `None` | interpolated table | `None` for now |
| `solver` | iterations, residuals | search tree | sweeps, convergence | training history in cost |

The open versus closed loop distinction lives in the policy block and in the operator
that connects it. An open-loop policy has no input port and connects in series; a
closed-loop policy connects in feedback. Verified: a `TrajectorySource` composes with
`>>` and is rejected by `@`.

```python
solution = planner.solve()
print(solution.evaluation)
plant.plot_trajectory(solution.trajectory)
closed = solution.policy @ plant     # closed-loop methods
driven = solution.policy >> plant    # open-loop methods
```

On a stochastic problem, an open-loop plan and a learned feedback policy are scored over
the same draws, which measures the case for feedback directly.

## Decisions (2026-09-12)

1. One universal type for every planner; no split by open and closed loop.
2. Every solution has a policy. Open-loop planners return a time-based source block.
3. A closed-loop planner's trajectory is its policy rolled out on the planner's grid, so
   the plot and the evaluation describe the same trajectory.
4. The planner fills `evaluation` by calling the standalone Monte Carlo evaluator, which
   stays a public verb. Open-loop policies are replayed over the problem's draws.
5. `cost_to_go` is optional.
6. Details a user compares go in the typed `solver` record. Internal handoffs between
   consecutive solves, the warm start and packed NLP vectors, stay on the planner.
7. Names: `PlanningSolution`; fields `policy`, `trajectory`, `evaluation`, `cost_to_go`,
   `solver`; `MonteCarloReport` is renamed `Evaluation`.
8. `planner.get_controller()` stays as a shortcut to the solution's policy.
9. The RL solver record reports mean episode cost J, positive and falling; return stays
   internal to the algorithms.

## Prepared by the RL module cleanup

The cleanup leaves one named producer per field: `self.controller` is the policy;
`nominal_trajectory()` produces the trajectory; `last_evaluation` holds the full report;
one function builds each training record; one function assembles the returned plan.
Switching RL then means replacing that assembly function.

## Consequences

- The open-loop block must hold the input the way the transcription assumed: linear
  between knots for collocation, zero-order hold for shooting. Otherwise simulating the
  policy would not reproduce the trajectory.
- The evaluator connects a policy by what its ports declare. Its JAX and NumPy backends
  accept only state laws today, so open-loop policies need a time-aware law or the
  simulator backend.
- Writing `@` on a source block should tell the user to connect it with `>>`; today it
  reports a dimension mismatch.
- `plot_solution` and `animate_solution` work for every planner.
- MPC reads the open-loop solution's trajectory each tick; its warm start already travels
  through the planner.
- An `Evaluation` with one trial prints a single J rather than a spread of zero.
- The evaluator's rollout loop and the RL planner's are two copies of one deterministic
  rollout; the redesign merges them.

## Open questions

1. Retiring `TrajectoryPlan`, `PolicyPlan` and `SolveMetadata`: a written deprecation note
   with no alias, which satisfies ROADMAP §2 and rule 3.4 and is safe because no notebook
   imports them; or aliases for one release.
2. Names of the per-planner `solver` records.
3. Whether RL fills `cost_to_go` from its critic, converted to cost and to the task's
   discount, or leaves it empty.
4. What `get_controller()` returns on an open-loop planner.
5. Whether the solution carries a top-level validity flag, or validity lives only in the
   evaluation's failure rate and the solver record's convergence or feasibility.
6. The exploration set owned by the RL algorithm, distinct from the task's start
   distribution.

## Migration outline

1. Add `PlanningSolution` and `Evaluation` beside the current types.
2. Move one planner at a time, tests first: RL, whose pieces already exist; dynamic
   programming; trajectory optimization with MPC; RRT.
3. Teach the evaluator to connect policies by their ports and to replay open-loop policies
   over draws; merge its rollout with the RL planner's.
4. Update the tutorials and demos that print `plan.metadata`; the course notebooks need no
   change.
5. Retire the old types per open question 1; update `DESIGN.md` §6, the planning band, the
   root prelude and the teaching-surface registry.
6. Fold the landed contract into `DESIGN.md` and delete this document.

## Verification

- For every planner, simulating `solution.policy` with its operator reproduces
  `solution.trajectory` on the planner's grid.
- One notebook solves the same stochastic problem by trajectory optimization and by RL,
  and both evaluations share the same draws.
- Teaching-surface test, notebook checks, regression gates, full `pytest`.
