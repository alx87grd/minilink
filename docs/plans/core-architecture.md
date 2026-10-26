# The core architecture: the pillars of v0.3

**Status:** a consolidation, 2026-10-26: no new design and no ruling. It gathers, pillar by
pillar, what the plan docs, the rulings records, ROADMAP, TODO and QUESTIONS already say about
the core. Each piece keeps the status it has in its source. **Ruled** carries its R-id;
**direction** means the maintainer's direction of 2026-10-26
([the record](../reviews/2026-10-26-core-consolidation.md)); **draft** means a tentative vision,
not decided; **proposal** means the agent's, not ruled. It replaces `end-state-api.md`, whose
entries moved to the plan docs they shape (§11).
**Read first** for anything about the core. The detail stays in the plan doc each section
points to (one fact, one home: [README.md](README.md)). The steps are
[TODO.md](TODO.md) §1, the open decisions [QUESTIONS.md](../dev/QUESTIONS.md) ("Blocks v0.3"),
and the landed contracts [DESIGN.md](../dev/DESIGN.md) §4–§6.
**Rung:** v0.3, the complete core (direction). v0.4 holds the features built on it (§10).

## 1. The target

**The maintainer's direction (2026-10-26):** v0.3 is the complete core, one architecture from
the plant to the evaluation and the optimizer, supporting stochastic problems and the hybrid
sampled loop, built in the natural order of the code, not by release or course dates. v0.4 is
for features built on the core, not intertwined with it. v0.9 stays the freeze candidate, and
v1.0 comes when the API feels mature and stable. Class material stays on `main` until the new
core is ready.

**The spine (ruled, R119, CONSTITUTION §1.1).** Three foundations: the continuous model
(`System`), the task (`PlanningProblem` and `PlanningSolution`) and the solver boundary
(`MathematicalProgram` and `Optimizer`). They are joined by one differentiable path from the
model and its parameters to the cost. Every approach that controls a vehicle or a robot is a tool
on that path.

**The pipeline as it stands** (landed 2026-10-24/25; DESIGN §5–§6), and where each pillar enters
it:

```text
System  f(x, u, t; p)                                        §2.1  every number a leaf
  └─ compile → evaluator: f, step, rollout (the step model)  §2.2  the step algebra
PlanningProblem / StochasticPlanningProblem                   §5    the draws (realize)
  └─ problem.compile(controller, dt) → ProblemEvaluator       §6    a sampled controller (computer @ plant)
       run → Trajectory → score_run → Evaluation               §3    one discrete judge
       pe.program(wrt) → MathematicalProgram                   §7    one parametric family, J(z, p)
            └─ Optimizer (wrapped backends, gd, adam)          §8    the solve as a rollout
Field  value(x, u, t; p)  (sets, costs, cost-to-go, V, Q)      §9    the last textbook noun
```

**Seven pillars, two foundations.** The maintainer named seven pillars on 2026-10-26: the hybrid
loop, one model and two runners (§6); one discrete judge (§3); stochastic problems (§5); the
generic parametric program with the stochastic program as its case (§7); a native solver and an
MPC as a proof of concept (§8); Fields (§9); the seeded stochastic policy (§5.3). Two
foundations sit under them, gathered from the plan docs: every object pure with every number a
parameter (§2.1), and the step algebra first-class (§2.2).

## 2. The foundations

### 2.1 Every object pure, every number a parameter

**Vision.** CONSTITUTION §2 ("One signature"): an object's mathematics is a method on
`(x, u, t; p)` that traces in every argument, `params` included; "an object that does not yet
read `params` is a gap to close, not a design." Equation paths are pure (invariant 2). A
realization is `(x0, params)` and nothing else (randomness.md §5). A sampled controller's memory
is its state, and its draws come from its seed and the tick (ruled, R104). `params` holds numbers
and integer seeds only (ruled, R118). An override is `None` or a complete tree (ruled, R121).

**Today in the code.**
- The library sets hold no parameters: `Set.params = None` (OD-24). Sets reading `params` is A5's.
- Distributions keep `mean` / `std` as attributes, with no `params`, so they are not leaves of
  `pe.params`, although DESIGN §6's stochastic figure draws them as leaves.
- `TrajectorySource` keeps its table as an attribute (`sample_values`); differentiable-problem
  step 8 moves it into `params`.
- Only the plant slice of the start is a leaf of `pe.params` (`"x_start"`). A controller's or an
  observer's initial state is read from `loop.x0` at compile.
- `Computer` blocks read `op.bound_params`, frozen at compile.
- The MPC's tick latch (`MPCTickLatch`) is a hidden cache (R123 removes it).
- `StepSystem.rollout` and `HybridDiagram.last_result` are run caches (D13, open).

**Steps.** TODO §1.1: A5, RN-2, V1's step 8 (`Set.as_cost`, the open-loop table in `params`), T1,
T2. S29 (the diagram's derived `x0`, `n`, `state`) is a v0.9
row; whether it joins this foundation is open (QUESTIONS.md).

### 2.2 The step algebra first-class

**Vision.**
- CONSTITUTION invariant 1: discrete-time systems (`StepSystem`, `StepDiagramSystem`) form a
  closed sibling algebra on an integer tick.
- A step rollout is a `Trajectory` with `t = k·dt` (differentiable-problem.md §3, the
  `StepRollout` fold).
- The step tools linearize a step system at a fixed point and read its eigenvalues against the
  unit circle (ruled, R122; it comes back with NS-5, when solver stability needs it).
- `vmap` over starts, seeds and parameter families on step systems, as `rollout_batch` does
  for continuous ones (native-optimizers.md §3.1).

**Today in the code.**
- The discrete evaluators' `rollout` (`DiscreteEvaluator`, `core/compile/evaluators/discrete.py`)
  returns host NumPy, so a step rollout does not differentiate end to end. There is no `rollout_batch` for step systems.
- `compile_step_diagram(flatten=True)` raises `NotImplementedError`, and step diagrams refuse a
  continuous leaf.
- Two same-tick semantics live side by side: `StepDiagramEvaluator` propagates within a tick
  from a fresh buffer, while `Computer.tick` reads the previous tick's committed buffer.
- `@`, `>>` and `+` refuse step systems; giving the step algebra operators is a core-grammar ask
  (CONSTITUTION §6; native-optimizers.md §4.7).
- A step system has `jacobian` but no `linearize` or poles.
- The step formula (`stepping.py`'s `step_body`) is shared, but the loops around it are written
  separately in `nominal_trajectory`, `RolloutEnvironment`, `Sys2Gym`, the realtime simulator,
  `HybridSimulator`, shooting, multiple shooting and the DP discretizer.

**Steps.** TODO §1.2: the `StepRollout` fold (V1.1's rest), S53 (a parameter family's batch
price), S65 (one owner per rule: the automatic `dt`, the input hold, what `n_steps` counts, the
one-tick lag inside a `Computer`).

**Open points** (QUESTIONS.md):
- one same-tick semantic for a step diagram;
- whether the `Trajectory` fold also carries a hybrid run's dense plant substeps (sampled-path
  §8.4, option B);
- operators on step systems.

## 3. One discrete judge (a pillar; the idea a draft)

**Vision (draft, the maintainer, 2026-10-26; [differentiable-problem.md](differentiable-problem.md)
§9).** Once `dt` and the scheme are set, a continuous rollout is the same thing as a system that
is already discrete. So the evaluation is unified from that point on:
- `problem.discretize(dt, solver=, substeps=)` gives a discrete problem:
  - the per-step cost `c_k = ∫ e^{−ρt} g dt` with `u_k` held;
  - the sets at the index;
  - `N = tf / dt`, `γ = e^{−ρ dt}`.
- One discrete evaluator, a scan over `k`, scores every loop that steps.
- Value iteration, the RL environment, the evaluator and the sampled loop read the same
  problem, so a policy has one `J`.
- The continuous problem stays primary, and the tools of continuous time keep reading it:
  collocation, the continuous LQR, the linear analysis, `method="scipy"`.
- The agent's refinement: integrate the cost with the state, so the step returns `c_k` at the
  state's accuracy.

**Today in the code.**
- The run is already the step model, and the score already reads the samples.
- One policy gets three to four close `J`s:
  - a left Riemann sum `g dt` in value iteration (`dp.py`) and the RL reward;
  - a trapezoid in `score_run`;
  - shooting's own trapezoid with a first-order hold (`transcription.trapezoid_integral`).
- The failure rules differ:
  - `score_run` integrates to `tf` and checks `X` and `U` at every sample;
  - the RL environment stops at the first exit from `X` and checks `U` nowhere;
  - value iteration prices the cells that leave its grid.
- The rules are written twice, in the environment's step and in `score_run` (R108 gives them
  one owner later).

**Open points** (QUESTIONS.md, "Blocks v0.3"):
- §9.4's six (the per-step cost's rule, the margins between samples, the object and its name,
  time-varying objects, the discount, the order of work);
- one failure rule for the discrete problem;
- its hold;
- whether shooting's score lowers to it;
- value iteration reading a finite horizon (S63, a v0.4 row);
- whether planner policies acting at a fixed `dt` are scored through the sampled face.

**Steps.** TODO §1.3.

## 4. The proposed build order (a proposal, not ruled)

The natural order of the code, each phase building on the one before. An independent review
(2026-10-26) checked the order against the code; its points are recorded here and in QUESTIONS,
none ruled.

| Phase | What | Needs | Why here |
| --- | --- | --- | --- |
| P0 | The rulings the phases need (QUESTIONS.md, "Blocks v0.3"), and the clean-up lane (K8, S32, G3, K4) beside the core | — | No foundation code before the ruling that shapes it (the rule of the 2026-10-22 sessions) |
| P1 | Every object pure, every number a parameter (§2.1) | P0 | The draws, the hybrid tick and the programs all read leaves of `params` |
| P2 | The step algebra first-class (§2.2) | P1 | The judge and the hybrid kernel are step loops |
| P3 | One discrete judge (§3) | P2 | One kernel before the draws and the sampled loop plug into it |
| P4 | The draws and the seeded policy (§5) | P3 | One change site for the realization; the computer's noise blocks need RN-4's seeds by name |
| P5 | The hybrid loop: one model, two runners (§6) | P2–P4 | A sampled loop is one more step loop for the judge |
| P6a | The generic parametric program (§7) | P1 (cost and set leaves) | Collocation uses the continuous `f`, not the step model (parametric-program.md §4) |
| P6b | The stochastic program, its case (§7) | P4 (the seed leaf) | |
| P6c | The solve as a rollout, the MPC capstone (§8) | P5, P6a | The MPC's closed loop runs through the sampled face |
| P7 | Fields (§9) | P1 (`Set.as_cost`'s signature) | Additive to the path |
| P8 | Hardening (S66), the notebooks migrated (R47), the merge (R79) | all | |

**Number changes.** The rule is R80–R82: closeness on the same problem and cost, `rtol 1e-6` on a
restructure. Every phase step stays byte-identical except these, each a commit of its own with a
ledger line:
- the same-tick semantic;
- the one-judge `J`;
- the cross-backend draws and live disturbances;
- R123's one-period MPC delay;
- the hybrid defaults.

The proposal is to re-pin the demos, notebooks and baselines once, at the end of P5.

**Open points the order raises** (QUESTIONS.md):
- S70 (the error family, ruled R114, a v0.9 row) earlier than P8, since S66 raises its types,
  and S29 in P1 (both in the release question);
- whether `problem.compile` refuses the stateless MPC until P5 (R123's computational delay lands
  with the hybrid loop; today's stateless MPC is a plain `System` that `problem.compile` accepts);
- the release of the rows the core reads but that sit elsewhere (S29, S70, the realtime runner's
  re-base and the orchestrator's adaptive plant integration, the RL heads' rebuild);
- the CONSTITUTION amendment ruled before P5 and applied with it;
- the `StepRollout` fold, placed in P2 here, waits for S31 in its row (OD-36: it is the hybrid
  simulator's record), so it lands in P2 or with P5.

## 5. Stochastic problems: the draws (a pillar; ruled in its convention)

### 5.1 Vision

**Ruled**: [randomness.md](randomness.md), RD-D1–RD-D24, its evaluator rulings superseded in part
by R10, R15, R19, R27, R44.
- A `Distribution` has no time.
- A noise signal is a block that holds draws over a sample period, its seed in `params`,
  drawn by a counter-based cipher that is the same on NumPy and JAX.
- One key reproduces an experiment: `realize(key)` gives every random block and parameter its
  own stream, derived by name.
- A problem declares noise on ports, as signals, never as bare laws.
- Monte Carlo is the closed-loop diagram simulated on draws realized once from one key.
- Each trial of `evaluate` realizes the loop's random blocks (landed, R92).

### 5.2 Today in the code

- `problem.realize(key, n)` draws the starts and the parameter values, plus one held value per
  disturbance port.
- `evaluate` holds the `disturbances` ports at nominal and warns (`warn_if_disturbed`).
- The free loop inputs are constant over a run.
- The same integer key draws different starts and parameters on NumPy and JAX (`trial_key`;
  KNOWN_ISSUES).
- Distribution parameters are not leaves (§2.1).
- `Sys2Gym.from_problem` applies no parameter or disturbance draw, although DESIGN §6 says it
  draws.
- There is no `NoiseSource`.

### 5.3 The seeded stochastic policy

**Direction, 2026-10-22, a pillar on 2026-10-26**; future-policies.md §5 point 4; D29 open.

`u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)` is a plain `System` built like the noise block:
- `seed=None` gives the mean;
- a seed per experiment comes from `realize(key)`;
- `log_prob` and `entropy` are methods beside `h`.

The recommendation (D29, open) is `GaussianPolicy(mean, log_std, *, dt, seed=None)`, with
`NeuralPolicyController` as its deterministic face.

### 5.4 Open points

QUESTIONS.md:
- D29's contract;
- one draw convention for a policy: the held draw `⌊t/Δ⌋` of D29, or the tick index of R104
  (with `Δ = d·dt_base` they agree, a convention to state);
- whether the RL heads' rebuild on the block is core or an RL follow-up (v0.4);
- `Sys2Gym` against DESIGN.

### 5.5 Steps

TODO §1.4: V1 step 7, RN-3 / RN-4 / RN-5 (RN-6 with each), the rest of AC-7, the seeded policy,
`Sys2Gym`'s start law.

## 6. The hybrid loop: one model, two runners (a pillar; the vision a draft)

### 6.1 Ruled

- The sampled loop is a tool on integer ticks (R13, R14), its name `HybridDiagram` kept for now.
- CONSTITUTION invariant 1 calls the sampled loop a simulation tool, not a `System` (R120).
- A sampled controller's memory is its state, and its draws come from its seed and the tick
  (R104).
- `problem.compile(policy)` dispatches on the policy's kind: one `compile`, one `evaluate`, the
  same `J` (R105). v0.3 refuses a sampled controller today (`require_continuous`).
- The MPC's latch goes for a one-period computational delay (R123; its numbers move once).
- The lazy `core` → `simulation.discrete` import is a written exception; moving `Computer` into
  `core/discrete/` is reconsidered with the loop (R100).

### 6.2 Vision (draft, the maintainer, 2026-10-25/26; [sampled-path.md](sampled-path.md) §8.1–§8.5; named a pillar on 2026-10-26)

**One model.** The pure blocks plus the schedule (plain data). The pure tick of a multi-rate
loop holds the held signals as state and makes a block's firing a select on `k mod d`
(§8.1, Drafts 1–2).

**Two runners of that one model** (§8.2–§8.3):
- **The pure step diagram.** `computer @ plant` is a `StepDiagramSystem` subclass that knows it
  is hybrid. It holds the computer's pure step and `discretize(plant, dt_base, substeps=n)` and
  keeps its continuous source, the `Discretization` precedent (§8.5). It serves the algebra,
  analysis (the sampled-data loop's stability), `evaluate`, `jit`, `grad` and `vmap`. Its
  layouts are a nested scan for two clocks, a base tick with selects in general, and the
  hyperperiod unrolled as an optimization (§8.1, Draft 3).
- **The clock orchestrator.** Today's `Computer.tick` and `HybridSimulator` shape, over the same
  pure blocks. It serves realtime and game mode, hardware in the loop, host-only blocks (an
  Ipopt MPC), and efficient simulation: idle blocks are skipped, and an adaptive plant
  integration restarts only when the held input changes (§8.4).

Neither runner owns a copy of a block's equations, and the step diagram is the orchestrator's
test oracle. Deployment generates its runner from the schedule (§8.2).

### 6.3 Today in the code

- `Computer` is mutable (`k`, `x`, double buffers), NumPy-only, and reads `bound_params`.
- `HybridSimulator` is a Python tick loop, with no `params` argument, no roles and no link to a
  problem or cost.
- `StepRollout` is the computer's record.
- `computer @ plant` builds a `HybridDiagram`, a plain class beside the step algebra.
- `MPCStatelessController` is a plain `System`: `problem.compile` does not refuse it, and its
  latch then runs at the RK4 stages.
- `%` refuses a controller with continuous states, which is future work with no row
  (automation-by-convention.md §4.5).

### 6.4 Open points

QUESTIONS.md, "Blocks v0.3":
- the CONSTITUTION invariant-1 amendment (§8.5: the coupling stays a tool; its result is a
  `System` of the step algebra), against R13 and R120;
- the default of `compute_trajectory` on a hybrid loop (the step-diagram runner, or the
  orchestrator with an adaptive plant: §8.4), and the plant trajectory's resolution;
- the same-tick semantic (§2.2);
- D13, the run caches (now [sampled-path.md](sampled-path.md) §8.6);
- whether the realtime simulator's re-base on the pure blocks is core or a feature;
- AC-8a's port names (automation-by-convention.md §6);
- R123's B option (a Mealy `update`), a study.

### 6.5 Steps

TODO §1.5: the two-runner row (design open), S31, V1 step 7b, the `HybridSimulator` drift row;
AC-8a and AC-8b (automation-by-convention.md §7).

## 7. One generic parametric program; the stochastic program its case (a pillar)

### 7.1 Vision

**Direction, 2026-10-26.** A `MathematicalProgram` compiled once, with an explicit, optional
`params` argument (`x0`, the plant, the cost and the sets), so the MPC and every other user
rebind without recompiling. The generic parametric program is finished all at once.

**Recommended, not ruled** ([parametric-program.md](parametric-program.md) §2, option C):
- one family: the library builds `J(z, p)`, `h(z, p)`, `g(z, p)`, and `bind(p)` returns the
  plain `MathematicalProgram`;
- one evaluator body, a hand-written `J(z)` being the case `p = {}`;
- one backend table for the offline and the compiled paths (§6, OPW-1–OPW-5).

The names are ruled: `ParametricProgram`, the key `plant` (R26, D19).

**The stochastic program as its case** (direction, 2026-10-26, from the maintainer's leaning of
2026-10-25, [native-optimizers.md](native-optimizers.md) §7.2): every program is deterministic
given its parameters, and the seed is one of them. Solving a stochastic program is solving over
several seeds (§7.3–§7.4, a draft):
- a sample average, the `N` seeds fixed inside `J`;
- a Monte Carlo over solves, `vmap` over seeds;
- stochastic approximation, a solver advancing the seed `child(seed, k)` under four soundness
  conditions.

### 7.2 Today in the code

- `ParametricMathematicalProgram` varies `x0` only.
- Direct collocation captures the plant, cost and set parameters in closures, and
  `require_online_params` refuses every other key.
- The MPC port functions drop their `params`.
- `JaxParametricProgramEvaluator` repeats about half of the plain evaluator.
- The compiled path accepts SciPy only (`build_parametric_optimizer_backend`).
- `pe.program(wrt, key, n_trials)` realizes its draws once (a sample average) and refuses
  `"x_start"` in `wrt` on a stochastic problem.

### 7.3 Open points

QUESTIONS.md:
- native-optimizers.md §7.5's seven:
  - bind or pass `p`;
  - partial trees at `bind` (parametric-program.md §3.1) against complete trees (R121);
  - non-differentiable leaves;
  - which leaf is the seed;
  - structural against value parameters;
  - the warm start;
  - sensitivities;
- D25 (a hand-written program declares its own `p`; the MPC's live-parameter ports, now
  [parametric-program.md](parametric-program.md) §2.1);
- `bind` on both the evaluator and the program;
- single shooting with parameters (parametric-program.md §5), which the proof of concept needs;
- the stale `system.mu` of parametric-program.md §3.5 against the ruled key `plant`;
- native-optimizers.md §4.1's `J(z, key)` against the seed as a leaf of `p`.

### 7.4 Steps

TODO §1.6: S76 (PP-1–PP-5), OPW-1–OPW-5, the stochastic-program row (design open).

## 8. The solve as a rollout: a proof of concept (a pillar)

### 8.1 Vision

[native-optimizers.md](native-optimizers.md) §1, ideas to review; the maintainer's vision and
two goals, 2026-10-25:
- native algorithms are pure array code, like a `System`;
- the solve is a rollout;
- some solvers are fully traceable;
- **goal 1:** an MPC whose solver runs a fixed number of iterations, so its tick and its closed
  loop compile and differentiate;
- **goal 2, long term:** convergence analysed as stability.

**Scope (direction, 2026-10-26):** in v0.3 a proof of concept only:
- a simple fixed-step solver (NS-1; NS-2's fixed-`K` scan);
- an MPC using it (NS-8, on single shooting);
- its closed loop in one scan, tuned by gradient (NS-9).

State-of-the-art solvers and MPC are v0.4. MPPI's tick is the same shape and already pure
(native-optimizers.md §4.6b); MPPI itself is a v0.4 feature.

### 8.2 Today in the code

- `gd` and `adam` are a Python loop on NumPy (`optimizers/first_order.py`), and Adam's update
  is jitted.
- Nothing differentiates through a solve.
- The RL learners already run their gradient steps in one `lax.scan`.

### 8.3 Open points

QUESTIONS.md:
- the kernel's shape: a pure kernel with a step-system view (recommended), or a `StepSystem`;
- which categories and algorithms are in scope;
- the fixed-iteration MPC's rung is answered by the direction: the proof of concept is v0.3,
  the rest v0.4.

### 8.4 Steps

TODO §1.7; the rest of the plan is the v0.4 row "Native solvers beyond the proof of concept"
(TODO §4).

## 9. Fields (a pillar)

### 9.1 Vision

[fields.md](fields.md), agreed 2026-09-15, not started; CONSTITUTION §2 lists `Field` as
"planned":
- a scalar function on the model's `(x, u, t; p)`, with `value` and `gradient`;
- `as_constraint` gives a `Set`, `as_input_constraint` an `InputSet`, `as_cost` a
  `CostFunction`;
- `QuadraticField`, whose sublevel set is the ellipsoid;
- `GridField`, the DP and tabular tables;
- `CallableField`, the RL critics;
- `LyapunovCertificate.V`, the DP cost-to-go, the LQR value and the RL critics all speak it.

The barrier of the CBF plan is a field (step 4.7, done).

### 9.2 Today in the code

`StateField` lives in `planning/spatial/state_fields.py`; `V(x)` and `cost_to_go` are not fields.

### 9.3 Open point

QUESTIONS.md: `as_cost` and the shaping are planned in three places:
- `Set.as_cost` (A5, differentiable-problem step 8);
- `Field.as_cost` (fields.md 4.1);
- the shaping's move to `core/` (geometry-module.md).

They share one signature, and the shaping needs one home.

### 9.4 Steps

TODO §1.8 (A1).

## 10. The core and the features

**Direction (2026-10-26):** v0.4 adds features built on the finished core, not intertwined with
it. Each feature reads one or more pillars:

| Feature (TODO §4) | Reads | Plan |
| --- | --- | --- |
| MPPI (S72, MP-1–MP-6) and the MPC planner contract (D27, T6) | §3, §5, §6 | [mppi.md](mppi.md) |
| Identification and trajectory generation (C4) | §7 (the program export with a data cost) | [differentiable-problem.md](differentiable-problem.md) §1 |
| `LQGPlanner` | §5 | [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5 |
| Workspace geometry (A3), then the CBF safety filter | §9 | [geometry-module.md](geometry-module.md), [cbf-safety-filter.md](cbf-safety-filter.md) |
| The RL follow-ups (C5), the RL heads on the seeded policy (open: core or here), reward scaling | §5.3 | [future-policies.md](future-policies.md), [rl-reward-scaling.md](rl-reward-scaling.md) |
| Native solvers beyond the proof of concept (NS-2's tolerance form, NS-3–NS-7) | §7, §8 | [native-optimizers.md](native-optimizers.md) |
| Traceable linear analysis (S73) | independent (the Jacobian) | [differentiable-problem.md](differentiable-problem.md) §4.6 |
| The general serial chain (C6), pyro parity (C1), C2, C3, S63, S64 | — | [pyro-port-remaining.md](pyro-port-remaining.md) |
| The estimation follow-ups (v0.9 rows): the sampled filters | §6 | [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §6 |
| After v1.0: behavior cloning (a `fit` verb and a dataset noun, open), diffusion policy | §5.3, §6 | [future-policies.md](future-policies.md) §3–§5 |

## 11. Where `end-state-api.md`'s entries went

`end-state-api.md` held the decisions that fixed a v0.4 name or contract. Its earlier entries
(D1–D12, D16–D24, D31) were ruled 2026-10-22/23 and landed 2026-10-24 (D23, the optimizer's
method keys, with S74, OD-15 of the run's log); their history is the rulings records and git. Its
remaining entries moved verbatim on 2026-10-26, each to the plan doc it shapes:

| Entry | Status | New home |
| --- | --- | --- |
| D13 the run-state caches | open | [sampled-path.md](sampled-path.md) §8.6 |
| D14 controller memory and draws on the tick | ruled, R104 | [sampled-path.md](sampled-path.md) §8.6 |
| D15 the compiled problem dispatches on the policy's kind | ruled, R105 | [sampled-path.md](sampled-path.md) §8.6 |
| D25 the parametric program's two asks | open | [parametric-program.md](parametric-program.md) §2.1 |
| D26 MPPI's name | ruled, R106 | [mppi.md](mppi.md) §10 |
| D27 the MPC block's planner contract | open | [mppi.md](mppi.md) §10 |
| D28 MPPI's stochastic default | ruled, R107 | [mppi.md](mppi.md) §10 |
| D29 the standard stochastic policy as a seeded block | open | [future-policies.md](future-policies.md) §5.1 |
| D30 where `RolloutEnvironment` lives | ruled, R108 | [mppi.md](mppi.md) §10 |
| The CONSTITUTION §1 sentence | landed, R119 | CONSTITUTION §1.1 |
