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

## R129 (2026-10-27, preliminary) The seeded stochastic policy: one block, its noise a deterministic signal of its seed

The question: the shape of the standard stochastic policy (D29), its draw convention and its
keywords.

- **One block,** `GaussianPolicy(mean, log_std, *, sample_period, seed=None)`:
  `u = μ_θ(x) + σ_θ(x) · ε(t)`.
- **Its noise is internal and deterministic given the seed:** `ε(t)` is the same counter-based
  signal a `WhiteNoise` source produces, a pure function of `seed` and the sample index
  `⌊t/Δ⌋`, so the block stays pure and its draws reproduce on NumPy and JAX.
- **`seed` is a parameter** (an integer leaf, found by the `seed` key, R118), realized per trial
  from one key and vmappable; `seed=None` gives the deterministic face `u = μ(x)`
  (`NeuralPolicyController`).
- **`log_prob(x, u)` and `entropy(x)` are methods of the block**, not of `Distribution`, which
  keeps its two methods.
- **One draw convention:** the sample index from the block's own period, `k = ⌊t/Δ⌋`; in a
  computer at `dt_base`, with `Δ = d·dt_base`, it is the tick index of R104.
- **The period keyword is `sample_period=`**, as on `WhiteNoise`; `dt` stays a tool's grid.
- Variants (the squashed Gaussian) are subclasses.

Options weighed: a deterministic law with a noise input port fed by a `WhiteNoise` source, from a
factory (randomness only in source blocks, but `log_prob` less direct); a block drawing by its own
rule. Reviewers A, B and C: `sample_period=` and one convention; A leaned to the law-plus-input
form.

## R130 (2026-10-27, preliminary) One program noun: `MathematicalProgram(..., params=None)`

The question: one noun with optional parameters, or a family (`ParametricProgram` with `bind(p)`)
whose binding is the plain program.

- **One noun, mirroring `f(x, u, t; params)`.** A `MathematicalProgram` may declare parameters;
  its functions are `J(z, params=None)`, `h(z, params=None)`, `g(z, params=None)`. The
  parameters are always there but optional: `params=None` reads the frozen default, as on a
  `System`.
- **The solve takes the values:** `Optimizer(program).solve(params=...)`; the MPC rebinds `x0`,
  the plant, the cost and the sets per tick without recompiling.
- **Students' form unchanged:** `MathematicalProgram(n_z=3, J=J)` with a one-argument `J`.
- **No `ParametricProgram` noun and no `bind` on programs.** `ParametricMathematicalProgram`
  (today, `x0` only) folds into the one noun.
- **A performance gate.** Optional parameters everywhere must not slow compilation or execution.
  Before the change lands, benchmark the compiled program carrying `params` against the same
  program with its parameters frozen (compile time and per-solve time, on the MPC and the PID
  examples). If carrying them costs speed, add an export option that compiles a program with no
  `params` argument, its parameters baked in as constants, for the fastest compile and run.
- **Its companions, recommended and confirmed with the naming question:** the MPC's optional
  live-parameter ports (`parameter_ports=` or `live=`), and `program.params_at(z)` returning the
  full tuned tree from an exported program.

Options weighed: a family with `bind` (two nouns, a fourth meaning of `bind`); one class always
taking `p` (students write an unused argument). Reviewers A and B: one noun with `params=None`;
C: the family, `bind` on the program only.

## R131 (2026-10-27, preliminary) Programs, parameters and randomness: the convention now, the seed strategies later

The question: native-optimizers.md §7.5's open points on how programs and solvers take
parameters and randomness, and the scope of the stochastic program.

- **The convention, as one block:**
  - inside, a program's functions always see a complete tree (R121); at the user boundary
    (`solve(params=...)`), a partial tree is merged once onto the defaults;
  - the seed is found by the `seed` key, an integer leaf, as on blocks, policies (R129) and
    sources (R128);
  - integer leaves never enter `wrt`; asking for one is refused;
  - sizes (the horizon `N`, the iterations `K`, batch sizes) are constructor arguments, and only
    values live in `params`;
  - the warm start is an argument of the solve call (`z0=`), not of the program;
  - sensitivities through a solve come later, with implicit differentiation;
  - `J(z, key)` is dropped: the seed lives in `params`.
- **The maintainer's note: the goal now is to build the playground.** How a solver handles the
  seed (keep it fixed, `vmap` over it, or advance it per iteration as stochastic approximation
  does) is an `Optimizer` strategy, tackled when the optimizers are worked on (the native
  solvers beyond the proof of concept, v0.4).
- **What exists today:** one pipeline produces a stochastic program, `pe.program(wrt, key=,
  n_trials=)` on a `StochasticPlanningProblem`: a deterministic `MathematicalProgram` whose `J` is
  the mean over `n_trials` draws realized once from `key` (a sample average). No demo uses it
  yet. RL trains with its own loop and keys, and MPPI averages inside its tick; neither builds a
  `MathematicalProgram`.
- **So the core keeps:** the convention above and the sample-average export. A Monte Carlo over
  solves (`vmap` over seeds) comes free once a solve traces. Stochastic approximation and the
  stochastic program category go with the optimizer strategies.

Reviewers A, B and C agreed on the convention; A and C limited the core to the sample average.

## R132 (2026-10-27, preliminary) A native solver is a pure kernel with a step-system view; categories with their first solver

The questions: is a native solver a `StepSystem` by construction or a kernel, and which program
categories and algorithms are in scope.

- **A pure kernel,** `init(z0, params) -> state` and `step(state, params) -> state` on the array
  module (the Optax shape the RL learners' `Adam` already uses), run by `Optimizer` (a
  `lax.scan` when traced).
- **A step-system view of it,** `as_step_system(solver, program)`, for the lesson ("the solve is
  a rollout") and, later, convergence analysed as stability. One implementation, not two.
- **Categories and algorithms: minimalist.** A program category is added with the first solver
  that needs it (least squares with identification; the stochastic category with the optimizer
  seed strategies, R131). Native algorithms are the textbook ones, each landing when something
  needs it; Ipopt stays the constrained workhorse. The core needs gradient descent and Adam as
  kernels, for the proof of concept.

Options weighed: every solver a `StepSystem` (Adam's tree state packed into a flat `x`); a kernel
with no view (loses the step-size-as-stability lesson). Reviewers A, B and C: the kernel with a
view.

## R133 (2026-10-27, preliminary) A minimal `Field` in the core; one `as_cost`; conversions named by what they return

The questions: why the `Field` noun at all; one home for `as_cost` and the shaping; the
conversions' names.

- **Why a `Field`:** one scalar function of the state appears under five APIs (the Lyapunov
  certificate, the DP cost-to-go, the LQR value, the RL critic, the workspace fields; the CBF
  barrier a sixth). The core's pipeline needs three conversions of it: a set as a soft cost, a
  value function as a terminal cost, a sublevel set.
- **The core builds a minimal `Field`:** `Field` (today's `StateField` moved to `core/`),
  `QuadraticField` (the LQR value, the Lyapunov `V`), the shaping functions moved from
  `planning/spatial/shaping.py` to `core/`, and the three conversions.
- **Later, with their consumers, after the core:** `GridField` (the DP tables), `CallableField`
  (the RL critics), the approximator, and the rebuilds of DP, the certificates and the critics
  on the noun (fields.md steps 4.4–4.6 and the rest).
- **One `as_cost`:** `Field.as_cost(weight=, shaping=)` is the one implementation,
  `g = weight · shaping(V(x, u, t))`. `Set.as_cost(weight=, shaping=)` is the same function on
  the set's margin seen as a field. The shaping move and `Set.as_cost` come first (the program
  bridge's soft constraints need them); `Field` generalizes later without changing the
  signature.
- **Names by what they return:** `as_set(lower=, upper=)` (a `Set`), `as_input_set(...)` (an
  `InputSet`), `as_cost(...)` (a `CostFunction`). `as_constraint` and `as_input_constraint` are
  renamed in one sweep with every caller, the track demos included, no alias (R82).

Options weighed: the full field plan in the core (most work off the core's path); no `Field`
noun (the five APIs stay separate, the CONSTITUTION's planned noun dropped); two independent
`as_cost` implementations. Reviewers A, B and C: one function, moved first; B: `as_set`;
C: Fields 4.1–4.3 in the core, 4.4–4.6 later.

## R134 (2026-10-27, preliminary) Names across the API

The question: cross-cutting names, settled before the core's new signatures copy today's
inconsistencies. All six lines taken as recommended.

1. **One keyword per concept**, written in DESIGN §4: `backend=` the array module, `solver=` the
   integrator, `method=` the optimization algorithm, `sample_period=` a block's period, `dt=` a
   tool's grid. `compile_backend=` (planners, `Optimizer`) becomes `backend=`, and the
   `Optimizer.backend` attribute (the solver adapter) becomes `adapter`; the MPC's `dt_mpc`
   becomes `sample_period` (its constructor frozen for GMC714, so one sweep with every caller,
   R82); `%`'s `dt` stays, as the tool's sampling grid.
2. **`bind` leaves the public surface.** The planning facade's geometry verb becomes
   `mount(geometry, on=sys, frame=)`; the RL algorithms' internal `bind` stays internal; programs
   have none (R130).
3. **One separator rule.** `.` walks a tree (blocks and parameters: `controller.Kp`,
   `plant.mu`); `:` names a signal of a block (a port or a state label: `plant:u`, `pid:e`).
   The plant input recorded as `plant.u` becomes `plant:u`.
4. **A learned or tabulated law `u = π(x)` is a `…Policy`** (`NeuralPolicy`, `LookupPolicy`,
   `GaussianPolicy`); classical controllers keep their textbook names. `NeuralPolicyController`
   becomes `NeuralPolicy` in the v1.0 rename window, under the deprecation policy; the RL
   package's internal `StochasticPolicy` is renamed so `GaussianPolicy` owns the idea.
5. **"Evaluate" and `Evaluation` mean scoring against a problem.** The compiled-model classes
   (`DynamicEvaluator`, `StepDiagramEvaluator`, …) become `Compiled…` at the v0.9 name pass;
   `PolicyEvaluator` keeps its textbook name; the one score adds no class.
6. **The MPC's live-parameter ports are `parameter_ports=[...]`**, and an exported program has
   `program.params_at(z)`, the full tuned tree (R130's companions).

Reviewer B proposed lines 1, 2, 4, 5 and 6 (C: `attach` for line 2); line 3 was the agent's.

## R135 (2026-10-27, preliminary) A generic planner interface; the loop mechanics stay in the MPC

The question: the contract between `ModelPredictiveController` and the planner it runs at every
tick, which today probes the planner with `hasattr` (D27, T6).

- **A planner does not know it runs in a loop.** It plans: given a problem's parameters (the
  start `x0` among them, R124, R130) it returns a solution.
- **The MPC class owns the loop mechanics:** the tick, its memory and the one-period
  computational delay (R123, R104), when to replan, and turning the plan into the input it
  applies.
- **To design: one generic planner interface** offering everything an MPC needs, the same for
  trajectory optimization, MPPI and the native fixed-iteration solver:
  - build once, then solve many times without recompiling (with `params` at the solve, R130);
  - solve from a given start and parameters, returning a plan the MPC can read (the input
    sequence, and the decision `z` where the planner has one);
  - a warm start, where the planner type has one: accept a guess built from a previous solution
    (what "shifting" means for its decision is the planner's knowledge, the decision to do it
    each tick is the MPC's).
- **Rejected:** a planner exposing a pure `tick` (it would make the planner loop-aware); D27's
  three verbs as written (shaped around a host NLP solve); keeping the probing.

The interface's exact verbs are drafted with the MPC work (T6) and before MPPI's planner (S72),
both consumers.

## R136 (2026-10-27, preliminary) Learning from data: decided at the behavior-cloning row; diffusion policy research-lane

The questions: how fitting to recorded data enters the API (a data-cost program export, or a
`fit` verb and a dataset noun), and the scope of diffusion policy.

- **Fitting to data is decided later, at the behavior-cloning row** (after v1.0). The options
  stay recorded there: the program export with a data-mismatch cost over `wrt` (a dataset as
  `Trajectory`s, no new verb), or a `fit` verb beside `solve` with a dataset noun. Identification
  (C4, v0.4) lands first; its design states which route it takes and flags it for this decision,
  so it does not set the pattern silently.
- **Diffusion policy is research-lane only,** JAX-only, after behavior cloning. The core only
  keeps it possible: a sampled controller with memory (R104) and a seeded draw (R129).
- A first imitation demo distilling MPC or MPPI into a neural policy stays a pointer, not a
  decision.

## R137 (2026-10-27, preliminary) No separate `KinematicModel`: the serial chain's description is a robot's one home for its frames

The question: adopt or drop the optional `KinematicModel` delegate on `System` (from the mid-2026
graphics and architecture review), now that the general serial chain (C6) and the posed-geometry
hook (S44) plan overlapping descriptions of a robot's geometry.

- **No separate delegate.** The general serial chain's one mechanism description (a DH table or a
  URDF: geometry, inertia, topology) is the one place a robot's frames live. It feeds the
  dynamics (RNEA, ABA), the kinematics (forward kinematics, the Jacobian) and the poses the
  drawing reads.
- **The posed-geometry hook (S44) is designed with it,** so the drawing reads the same frames.
- **A hand-written plant keeps its own methods;** no math moves off `System`.

Options weighed: a `KinematicModel` object for every plant with geometry, now (a new core
abstraction before its consumer); dropping the idea (the serial chain and the drawing hook could
describe the same frames twice).
