# The design questions, one by one (2026-10-27)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** after the 2026-10-26 simplification, the maintainer asked that QUESTIONS hold only
fundamental design questions (architecture, API and naming, vision, performance), and to go
through them one at a time: a brief context, options with their consequences, a recommendation.
Each ruling below was taken in session; the plan doc it shapes and TODO carry it.

## R124 (2026-10-27) Three owners of a start; a diagram's state derived from its blocks

The question: does a diagram's start state become a parameter (S29)?

- **The block owns its initial state.** Each block keeps `x0`, its default initial state.
- **A diagram holds no copy.** Its `x0`, `n` and `state` are views computed from its blocks:
  - reading `diagram.x0` stacks the blocks' `x0` at that moment;
  - setting `diagram.x0 = v` **writes through**, cutting `v` by the state index into each
    block's `x0` (a nested diagram passes its slice on down);
  - `n`, `state_index` and the labels are structural, cached and recomputed when the wiring
    changes.

  `refresh()` has nothing left to do for the state and leaves `Simulator`. A block shared by
  two diagrams shares its `x0`, as it already shares its live `params`.
- **The problem owns the plant's start:**
  - a point or a `SingletonSet` for a deterministic problem;
  - a `Distribution` for a stochastic one;
  - a `Set` for trajectory optimization.

  The plant block's `x0` is only the fallback when the problem states none (today's
  `default_x_start`). The problem says nothing about the controller's state.
- **The policy owns its start, as a design parameter.** In the evaluator's tree, each stateful
  block gets an `x0` leaf beside its other parameters (`pe.params["controller"]["x0"]`,
  `pe.params["observer"]["x0"]`), read from the block at compile as the nominal. It is tunable,
  differentiable and vmappable. `x_start` stays the plant's slice, from the problem. A sampled
  controller's memory is covered by the same rule (R104).
- **The observer's x̂₀ is explicit:** the block's own `x0`. No rule defaults it to the mean of
  the problem's start law.
- `x0` (the start) and `state.nominal_value` (the operating point x̄) keep one owner each.

Options weighed: the whole start under `x_start` (one tree, but `x_start` changes shape and mixes
the task with the design); a read-only `diagram.x0` (purer, but breaks `loop.x0 = ...` in
lessons); keeping S29 in v0.9 (one exception to "every number a parameter" in the core).
Reviewers A, B and C all placed S29 in the core. The row moves from v0.9 to TODO §1.1.

## R125 (2026-10-27) One same-tick semantic: propagation within the tick

The question: a step diagram propagated a signal within its tick, while `Computer.tick` read the
previous tick's committed buffer (one tick of latency per wire). The same diagram gave different
numbers by runner.

- **The rule.** Every step diagram and every runner propagates within the tick: if A feeds B, B
  at tick k reads A's output at tick k, as in the textbook block diagram and a continuous
  diagram's feedthrough.
- **Delay is a block.** Latency exists only where a block states it: a `UnitDelay`, the MPC's
  one-period computational delay (R123), a held slow output.
- **The `Computer`'s hidden one-tick lag is removed** (S65's item "tested and stated, or
  removed" is settled: removed). The hybrid demos' numbers move once, with a closeness report
  (R80).
- **An algebraic loop** (a cycle with no delay) is refused at wiring time.

Options weighed: one tick per wire (simple to execute, but a diagram no longer means its
equations); either, per connection (two dialects). Reviewers A, B and C: propagation within the
tick.

## R126 (2026-10-27) The composition operators on step systems

The question: `@`, `>>` and `+` refused step systems; a step diagram was wired with `connect`
only.

- **The same `@`, `>>` and `+`, with the same meaning, on the step algebra**, built on the same
  assembly code (`core/assembly.py`), not a copy. The result is a `StepDiagramSystem`.
- **Mixing a continuous and a step operand is refused**, naming the fix (`%` or `discretize`).
- **CONSTITUTION §6's grammar sentence is amended** when the operators land: the three operators
  are the composition language of both algebras, and `%` stays the one operator between them.
  The text of the amendment comes to the maintainer with the landing commit.
- This is what `computer @ plant` as a step diagram needs (the hybrid-loop question).

Options weighed: a helper per pattern (`gradient_loop`); `connect` only. Reviewers A and B: the
operators; C: none in v0.3, extending `feedback()` if a lesson needs it.
