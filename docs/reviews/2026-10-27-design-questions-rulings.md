# The design questions, one by one (2026-10-27)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** after the 2026-10-26 simplification, the maintainer asked that QUESTIONS hold only
fundamental design questions (architecture, API and naming, vision, performance), and to go
through them one at a time: a brief context, options with their consequences, a recommendation.
**Status: preliminary choices, not final rulings.** The maintainer's words (2026-10-27): record
each answer as a preliminary choice; when the final architecture is drafted, these decisions stay
open to readjustment. The R-ids are anchors for citation; each heading says "preliminary". The
plan doc each choice shapes and TODO carry it, with the same caveat.

## R124 (2026-10-27, preliminary) Three owners of a start; a diagram's state derived from its blocks

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

## R125 (2026-10-27, preliminary) One same-tick semantic: propagation within the tick

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

## R126 (2026-10-27, preliminary) The composition operators on step systems

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

## R127 (2026-10-27, preliminary) `computer @ plant` is a step diagram at its base clock; two verbs, two runners

The question: what the coupling of a sampled computer and a continuous plant returns, its name,
and which runner its verbs use.

- **A `System` of the step algebra.** `computer @ plant` returns a `StepDiagramSystem` at its
  base clock: the computer's pure tick plus `discretize(plant, dt_base, substeps=n)`, keeping its
  continuous source (the `Discretization` precedent). Every step tool applies (`rollout`,
  `jacobian`, the sampled-data loop's poles, `problem.compile`, `vmap`, `grad`).
  **CONSTITUTION invariant 1 is amended** when it lands ("the coupling is a tool; its result, at
  the base clock, is a `System` of the step algebra"), against R13 and R120's "not a `System`";
  the text comes to the maintainer with the landing commit.
- **The name stays `HybridDiagram` for now** (a `StepDiagramSystem` subclass). `SampledDataSystem`
  (the reviewers' choice, the textbook term) stays an option for the naming of the frozen surface.
- **Two verbs, two runners, the same pure blocks:**
  - `loop.compute_rollout()` is the standard step-system rollout: the samples at the base ticks,
    pure, traced, differentiable. `problem.compile` and `evaluate` run this semantics, so `J`
    matches `compute_rollout`.
  - `loop.compute_trajectory()` is the specialized hybrid simulation: the clock orchestrator
    ticks the computer and a continuous solver integrates the plant between ticks (adaptive,
    restarted at each sample, since the sample times are known), giving the dense plant
    trajectory. It is the realtime runner's offline twin, and it differs from `evaluate`'s `J`
    by the integration error only.
- **Why two verbs:** a step rollout records only the ticks; the plant's substeps live inside
  `discretize`'s step. Recording them through the standard rollout would change the
  `StepSystem` contract. The orchestrator gives the dense trajectory without that change.
- **Both runners follow R125** (propagation within the tick), and the step diagram is the
  orchestrator's test oracle at the ticks.
- **Left to the implementation:** the plant solver's default in `compute_trajectory` (adaptive,
  per this ruling; today's fixed RK4 substeps kept as a `solver=` choice) and its tolerances.

Options weighed: stay a tool with a read-only step map (no amendment, two objects for one
loop); one verb with the step runner by default (one `J` everywhere, but the dense trajectory
needs a contract change). Reviewers A, B and C: a step diagram; `SampledDataSystem`; one verb.

## R128 (2026-10-27, preliminary) Disturbances are source blocks; one realization shape

The questions: CONSTITUTION §2 says a stochastic problem adds "distributions over …
disturbances", the code takes a bare `Distribution` per port and holds one draw for the run, and
randomness.md's RD-R4 says a port takes a signal, never a bare law; and three `realize` methods
return three shapes.

- **No `Distribution` on a port.** A disturbance is a source block wired into the problem's
  system: `WhiteNoise`, a held source, a `Step` load, a colored-noise filter. Its time behaviour
  is stated by the block.
- **Two forms, one diagram:**
  - explicit: the problem's system is a diagram whose sources feed the plant's ports (the
    `plant` role still names the plant, so the cost scores it and `x_start` is its slice, R124);
  - the shortcut: `disturbances={"d": WhiteNoise(...)}` accepts blocks only, and the problem
    wires each onto its port, building exactly that diagram. A bare law is refused, naming the
    fix.
- **For both problems.** A deterministic `PlanningProblem` carries a deterministic load (a
  `Step`) the same way; the stochastic sibling adds the laws on the start and the parameters.
- **The draws.** A source's seed is a leaf of `params`, found by its path (`"load.seed"`); a
  stateful source's start is an `x0` leaf (R124).
- **One realization shape:** `problem.realize(key, n_trials) -> (x0s, params)`, the batched tree
  carrying the parameter draws and every source's seed; the `"disturbances"` key goes; `n_trials`
  everywhere, as on `evaluate`.
- **CONSTITUTION §2 is amended** when it lands: "the stochastic sibling adds distributions over
  starts and parameters; disturbances are source blocks on the plant's ports". The text comes to
  the maintainer with the landing change.

Options weighed: accept both a law (one constant draw) and a signal (two meanings for one
argument); keep the bare law and amend RD-R4 (contradicts the textbook's noise intensity and the
LQG gate); the explicit diagram only (one way, but every stochastic task builds a diagram).
Reviewer A: signals, a refusal for a bare law; reviewer B: one `realize` shape.
