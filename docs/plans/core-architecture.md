# The v0.3 core: the plan

**Read first for the core.** This is the one design home of v0.3's workstreams. Each gets a
goal, a status, what the code does today, its open decisions and its rows. Where the pieces live:
- **The rows** (files, "done when") are [TODO.md](TODO.md) §1.N, with the same N as the section
  here.
- **The open decisions** are [QUESTIONS.md](../dev/QUESTIONS.md), each tagged with the same §N.
- **The design detail** is in the topic plan doc each section links.
- **The landed contracts** are [DESIGN.md](../dev/DESIGN.md) §4–§6.

**Status words:**
- **landed**: in the code;
- **ruled**: an R-id in a dated record under [docs/reviews/](../reviews/);
- **direction**: the maintainer's words of 2026-10-26
  ([record](../reviews/2026-10-26-core-consolidation.md));
- **draft**: a vision, not decided;
- **proposal**: the agent's, not ruled.

Nothing in this doc rules anything.

## 0. The target

**The direction (2026-10-26).**
- **v0.3 is the complete core.** It is one architecture from the plant to the evaluation and the
  optimizer, supporting stochastic problems and the hybrid sampled loop, built in the natural
  order of the code, with no date.
- **v0.4** adds features built on the core, not intertwined with it.
- **v0.9** stays the freeze candidate.
- **v1.0** comes when the API feels mature.
- Class material stays on `main` until the new core is ready.

**The spine** (ruled, R119, CONSTITUTION §1.1). A `System`, compiled, is rolled out to a
`Trajectory`, scored by a problem's cost into an `Evaluation`, lowered to a `MathematicalProgram`
and solved by the `Optimizer`. Every approach is a tool on that path. Where each workstream
enters the pipeline, which landed 2026-10-24/25 (DESIGN §5–§6):

```text
System  f(x, u, t; p)                                        §1  every number a leaf
  └─ compile → evaluator: f, step, rollout (the step model)  §2  the step algebra
PlanningProblem / StochasticPlanningProblem                   §3  the draws (realize)   §4 seeded policy
  └─ problem.compile(controller, dt) → ProblemEvaluator       §5  a sampled controller (computer @ plant)
       run → Trajectory → score_run → Evaluation               §6  one score
       pe.program(wrt) → MathematicalProgram                   §7  one parametric family, the seed a leaf
            └─ Optimizer (wrapped backends, gd, adam)          §8  the solve as a rollout
Field  value(x, u, t; p)  (sets, costs, cost-to-go, V, Q)      §9  the last textbook noun
```

**The workstreams.** The maintainer named eight items on 2026-10-26. The plan docs add two
invariants under them. ROADMAP §1 calls those invariants "foundations"; that word also names the
CONSTITUTION's three objects, so this doc says "invariants".

| § | Workstream | Kind | The maintainer's item |
| --- | --- | --- | --- |
| 1 | Every object pure, every number a parameter | invariant | (under all) |
| 2 | The step algebra first-class | invariant | (under all) |
| 3 | The draws | pillar | stochastic problems |
| 4 | The seeded stochastic policy | pillar | the seeded stochastic policy |
| 5 | The hybrid loop: one model, two runners | pillar | `HybridDiagram` and `Computer`, two runners |
| 6 | One score | pillar | one discrete judge |
| 7 | The parametric program, the stochastic program its seed case | pillar | the generic parametric `MathematicalProgram`; the stochastic program |
| 8 | The native proof of concept | pillar | a native fixed-step solver and an MPC on it |
| 9 | Fields | pillar | Fields |

"Pillar" here means a core workstream. ROADMAP §5's six application families (classical
control, nonlinear control, trajectory optimization and MPC, RL, identification and imitation,
estimation) are "the toolbox chapters".

**What a user writes at the end of v0.3** (a proposal, illustrative, not a contract; each name
marked `open` is a QUESTIONS entry):

```python
problem = StochasticPlanningProblem(plant, cost=cost, tf=5.0,              # §3: laws on starts,
    x0=Gaussian(x_bar, std), params={"plant": {"m": Uniform(0.9, 1.1)}},   #     parameters and
    disturbances={"d": WhiteNoise(1, psd=0.1, sample_period=0.01)})        #     noise signals
pe = problem.compile(pid % 0.01, dt=0.01)        # §5: a digital PID, the sampled face
print(pe.evaluate(pe.params, n_trials=100, key=1))   # §3, §6: one key, one J
program = pe.program(wrt=["controller.Kp", "controller.Kd"])   # §7: params optional (open)
best = Optimizer(program).solve()
mpc = fixed_iteration_mpc(problem, K=20, sample_period=0.05)   # §8: name open
tuned = Optimizer(problem.compile(mpc, dt=0.01).program(wrt=["controller.cost.Q"])).solve()
V = QuadraticField(P)                            # §9: the terminal cost as a field
```

## 1. Every object pure, every number a parameter (invariant)

- **Goal.** CONSTITUTION §2, "One signature": an object's mathematics is a method on
  `(x, u, t; p)` that traces in every argument. "An object that does not yet read `params` is a
  gap to close." The equation paths are pure.
- **Status.** Ruled in its rules:
  - a realization is `(x0, params)` and nothing else (randomness.md §5);
  - a sampled controller's memory is its state, its draws from seed and tick (R104);
  - `params` holds numbers and integer seeds only (R118);
  - an override is `None` or a complete tree (R121).
- **Today.**
  - `Set.params = None` (OD-24).
  - Distributions keep `mean` / `std` as attributes; DESIGN §6's figure draws them as leaves.
  - `TrajectorySource` keeps its table as an attribute.
  - Only the plant's slice of the start is a leaf (`"x_start"`); a controller's or observer's
    start is read from `loop.x0`.
  - `Computer` blocks read `op.bound_params`, frozen at compile.
  - The MPC's `MPCTickLatch` is a hidden cache (R123 removes it).
  - `StepSystem.rollout` and `HybridDiagram.last_result` cache runs (D13).
  - Path lookup and merge of parameter trees are written four times:
    - `planning/evaluator.py`;
    - `planning/problems.py`;
    - `core/compile/evaluators/jacobian.py`;
    - `core/assembly.py`.

  Reviewer A §2.6 counted them.
- **Open:**
  - the derived start state (S29), in "Which release does each borderline row take";
  - the CONSTITUTION's disturbance sentence.
- **Rows:** [TODO §1.1](TODO.md#11-every-object-pure-every-number-a-parameter). Design:
  [randomness.md](randomness.md) (RN-2), [differentiable-problem.md](differentiable-problem.md)
  §8.

## 2. The step algebra first-class (invariant)

- **Goal.**
  - CONSTITUTION invariant 1: `StepSystem` and `StepDiagramSystem` form a closed sibling
    algebra on an integer tick.
  - A step rollout is a `Trajectory` with `t = k·dt`.
  - Step rollouts trace end to end and batch over starts, seeds and parameter families
    (native-optimizers.md §3.1).
  - The step tools linearize at a fixed point and read eigenvalues against the unit circle (R122;
    with NS-5, when solver stability needs it).
- **Status.** The invariant is ruled. Its closure is open.
- **Today.**
  - The discrete evaluators' `rollout` (`core/compile/evaluators/discrete.py`) returns host
    NumPy. There is no step `rollout_batch`.
  - `compile_step_diagram(flatten=True)` raises.
  - Two same-tick semantics: `StepDiagramEvaluator` propagates within a tick, while
    `Computer.tick` reads the last tick's buffer.
  - `@`, `>>` and `+` refuse step systems. There is `jacobian` but no `linearize` or poles.
  - One step formula (`stepping.py`'s `step_body`) sits under eight hand-written loops:
    - `nominal_trajectory`;
    - `RolloutEnvironment`;
    - `Sys2Gym`;
    - the realtime simulator;
    - `HybridSimulator`;
    - shooting;
    - multiple shooting;
    - the DP discretizer.
- **Open:**
  - one same-tick semantic;
  - operators on step systems;
  - whether the `Trajectory` fold carries a hybrid run's dense substeps (§5's default-runner
    entry).
- **Rows:** [TODO §1.2](TODO.md#12-the-step-algebra-first-class).

## 3. The draws (pillar)

- **Goal.** Stochastic problems on one convention ([randomness.md](randomness.md)):
  - a `Distribution` has no time;
  - a noise signal is a block holding draws over a sample period, its seed in `params`, drawn by
    a counter-based cipher, the same on NumPy and JAX;
  - `realize(key)` gives each random block and parameter its own stream, by name;
  - a problem declares noise on ports as signals, never as bare laws;
  - Monte Carlo is the closed loop simulated on draws realized once from one key.
- **Status.** Ruled: RD-D1–RD-D24, in part superseded by R10, R15, R19, R27, R44. Each trial
  realizing the loop's blocks landed (R92).
- **Today.**
  - `problem.realize(key, n)` draws the starts and parameters, plus one held value per
    disturbance port.
  - `evaluate` holds disturbances at nominal and warns.
  - One integer key draws different values on NumPy and JAX (`trial_key`; KNOWN_ISSUES).
  - `Sys2Gym.from_problem` applies no draw, although DESIGN §6 says it does.
  - There is no `NoiseSource`.
- **Open:**
  - one `realize` shape;
  - the CONSTITUTION §2 disturbance sentence against RD-R4.
- **Rows:** [TODO §1.3](TODO.md#13-the-draws).

## 4. The seeded stochastic policy (pillar)

- **Goal** (direction 2026-10-22, a pillar 2026-10-26).
  `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)` is a plain `System` built like the noise block:
  - `seed=None` gives the mean;
  - a seed per experiment comes from `realize(key)`;
  - `log_prob` and `entropy` sit beside `h`.

  A draft from reviewer A §3.4 is an alternative: a deterministic law with a noise input port.
- **Status.** The direction is set. The contract is open (D29,
  [future-policies.md](future-policies.md) §5.1). The recommendation is
  `GaussianPolicy(mean, log_std, *, dt, seed=None)`, with `NeuralPolicyController` its
  deterministic face.
- **Today.** The RL Gaussian heads draw from their own keys.
- **Open:**
  - the contract and its period keyword;
  - one draw convention, the held sample or the tick;
  - the RL heads' rebuild, in the core or in v0.4.
- **Rows:** [TODO §1.4](TODO.md#14-the-seeded-stochastic-policy).

## 5. The hybrid loop: one model, two runners (pillar)

- **Goal** (draft, the maintainer 2026-10-25/26; [sampled-path.md](sampled-path.md) §8.1–§8.5).
  - **One model:** the pure blocks plus the schedule as plain data. Held signals are state, and
    a block's firing is a select on `k mod d`.
  - **Runner 1, the pure step diagram.** `computer @ plant` is a `StepDiagramSystem` on the
    base clock: the computer's pure step and `discretize(plant, dt_base, substeps=n)`, keeping
    its continuous source (the `Discretization` precedent). It serves the algebra, the
    analysis, `evaluate`, `jit`, `grad` and `vmap`.
  - **Runner 2, the clock orchestrator:** today's `Computer.tick` / `HybridSimulator` shape over
    the same pure blocks. It serves realtime and game mode, hardware in the loop, host-only
    blocks such as an Ipopt MPC, and efficiency (idle blocks skipped).
  - Neither runner owns a copy of a block's equations. The step diagram is the orchestrator's
    test oracle.
  - The draft also carries three things:
    - three layouts: a nested scan, a base tick with selects, the hyperperiod unrolled;
    - the adaptive plant integration as one of two open defaults;
    - "deployment generates its runner from the schedule".

    Reviewer A §2.4 recommends one layout, the base tick with selects, in v0.3.
- **Status.** Ruled:
  - the sampled loop is a tool on integer ticks (R13, R14), `HybridDiagram` kept for now;
  - invariant 1 calls it a tool, not a `System` (R120);
  - memory is state, draws come from seed and tick (R104);
  - `problem.compile` dispatches on the policy's kind (R105);
  - the MPC latch becomes a one-period computational delay (R123);
  - the lazy `core` → `simulation.discrete` import is a written exception (R100).

  The rest is a draft.
- **Today.**
  - `Computer` is mutable and NumPy-only. `HybridSimulator` is a Python tick loop with no
    `params`, no roles and no problem.
  - `StepRollout` is the computer's record.
  - `problem.compile` refuses a `StepSystem` controller, but it accepts the stateless MPC,
    whose latch then runs at the RK4 stages.
  - `%` refuses a controller with continuous states (automation-by-convention.md §4.5, no row).
- **Open:**
  - the CONSTITUTION amendment, `computer @ plant` a step diagram;
  - `compute_trajectory`'s default runner and the plant trajectory's resolution;
  - the same-tick semantic (§2);
  - the run caches (D13);
  - refusing the stateless MPC now;
  - the name (`SampledDataSystem`);
  - the realtime re-base, core or later.

  Elsewhere:
  - AC-8a's port names (automation-by-convention.md §6–§7);
  - R123's option B, a study (sampled-path.md §8.3).
- **Rows:** [TODO §1.5](TODO.md#15-the-hybrid-loop-one-model-two-runners).

## 6. One score (pillar)

- **Goal** (draft, the maintainer 2026-10-26; [differentiable-problem.md](differentiable-problem.md)
  §9). Once `dt` and the scheme are set, a continuous rollout is a discrete system, so the
  evaluation unifies from there. The problem is discretized once:
  - the per-step cost `c_k = ∫ e^{−ρt} g dt` with `u_k` held;
  - the sets at the index;
  - `N = tf / dt`;
  - `γ = e^{−ρ dt}`.

  One scan over `k` scores every loop that steps. Value iteration, the RL environment, the
  evaluator and the sampled loop then give a policy one `J`. The continuous problem stays
  primary for collocation, the continuous LQR, the linear analysis and `solver="scipy"`. The
  agent's refinement: integrate the cost with the state. Under forward Euler this equals today's
  `g·dt` of value iteration and RL (reviewer A §3.1).
- **Status.** The direction names it a pillar. The idea is a draft. "Judge" was the working
  word; the function stays `score_run`.
- **Today.**
  - One policy gets three or four close `J`s:
    - a left Riemann sum `g dt` in `dp.py` and the RL reward;
    - a trapezoid in `score_run`;
    - shooting's trapezoid with a first-order hold (`transcription.trapezoid_integral`).
  - The failure rules differ:
    - `score_run` integrates to `tf` and checks `X` and `U` at every sample;
    - the RL environment stops at the first exit from `X` and checks no `U`;
    - value iteration prices the cells that leave its grid.
  - The rules are written twice, in the environment's step and in `score_run` (R108 gives them
    one owner).
- **Open:**
  - differentiable-problem §9.4's six points;
  - the failure rule, the hold and the scope: shooting's score, the planners' fixed-`dt`
    policies, S63;
  - the object: a new noun, the same `PlanningProblem` on a `StepSystem`, or internal.
- **Rows:** [TODO §1.6](TODO.md#16-one-score).

## 7. The parametric program; the stochastic program its seed case (pillar)

- **Goal** (direction 2026-10-26).
  - A `MathematicalProgram` is compiled once, with an explicit, optional `params` argument (`x0`,
    the plant, the cost, the sets), so the MPC and every other user rebinds without recompiling.
    It is finished all at once.
  - Every program is deterministic given its parameters, the seed one of them. A stochastic
    program is solved over several seeds ([native-optimizers.md](native-optimizers.md)
    §7.2–§7.4, a draft):
    - a sample average;
    - a Monte Carlo over solves (`vmap` over seeds);
    - stochastic approximation, the solver advancing `child(seed, k)` under four soundness
      conditions.
- **Status.** The direction is set.
  - **The recommendation, not ruled** ([parametric-program.md](parametric-program.md) §2,
    option C):
    - one family: `J(z, p)`, `h(z, p)`, `g(z, p)`, whose binding is the plain program;
    - one evaluator body, a hand-written `J(z)` the case `p = {}`;
    - one backend table for the offline and compiled paths (OPW-1–OPW-5).
  - The key `plant` is ruled (R26). The name `ParametricProgram` has no ruling text: R26 is
    the ruling of D19, and its text names the key and the evaluator's verbs, not the family.
    Reviewer B O1's alternative is a QUESTIONS entry.
- **Today.**
  - `ParametricMathematicalProgram` varies `x0` only.
  - Collocation closes over the plant, cost and set parameters, and `require_online_params`
    refuses every other key.
  - The MPC port functions drop `params`.
  - `JaxParametricProgramEvaluator` repeats about half of the plain evaluator.
  - The compiled path takes SciPy only.
  - `pe.program(wrt, key, n_trials)` realizes once (a sample average) and refuses `"x_start"`
    in `wrt` on a stochastic problem.
- **Open:**
  - one family, D25 and `bind`;
  - native-optimizers §7.5's seven points, with §4.1's `J(z, key)`;
  - `MathematicalProgram(..., params=None)` against a second noun;
  - the stochastic program's v0.3 scope.

  Elsewhere:
  - single shooting with parameters is in §8's row;
  - the stale `system.mu` of parametric-program §3.5 is annotated there.
- **Rows:** [TODO §1.7](TODO.md#17-the-parametric-program-the-stochastic-program-its-seed-case).

## 8. The native proof of concept: the solve as a rollout (pillar)

- **Goal** ([native-optimizers.md](native-optimizers.md) §1; the maintainer's vision,
  2026-10-25).
  - Native algorithms are pure array code, like a `System`.
  - The solve is a rollout, and some solvers trace fully.
  - Goal 1: an MPC whose solver runs a fixed number of iterations, so its tick and closed loop
    compile and differentiate.
  - Goal 2, long term: convergence analysed as stability.
- **Status.** The direction: in v0.3 a proof of concept only:
  - a simple fixed-step solver (NS-1, NS-2's fixed-`K` scan);
  - an MPC on it (NS-8, on single shooting);
  - its closed loop in one scan, tuned by gradient (NS-9).

  State-of-the-art solvers, MPC and MPPI are v0.4. The plan doc still says MPPI's tick comes
  first (§4.6b, §5); that is an open point.
- **Today.**
  - `gd` and `adam` are a Python loop on NumPy (`optimizers/first_order.py`); Adam's update is
    jitted.
  - Nothing differentiates through a solve.
  - The RL learners already run their steps in one `lax.scan`.
- **Open:**
  - the kernel's shape: a pure kernel with a step-system view, or a `StepSystem`;
  - which fixed-iteration tick comes first;
  - the categories and algorithms in scope (v0.4).
- **Rows:** [TODO §1.8](TODO.md#18-the-native-proof-of-concept).

## 9. Fields (pillar)

- **Goal** ([fields.md](fields.md), agreed 2026-09-15; CONSTITUTION §2 lists `Field` as
  planned).
  - A scalar function on `(x, u, t; p)` with `value` and `gradient`.
  - `as_constraint` gives a `Set`, `as_input_constraint` an `InputSet`, `as_cost` a
    `CostFunction`.
  - `QuadraticField` (its sublevel set the ellipsoid), `GridField` (the DP and tabular tables),
    `CallableField` (the RL critics).
  - The certificate's `V`, the DP cost-to-go, the LQR value and the critics all speak it. The CBF
    barrier is a field (step 4.7, done).
- **Status.** Agreed and not started. It is additive to the path.
- **Today.** `StateField` lives in `planning/spatial/state_fields.py`. `V(x)` and `cost_to_go`
  are not fields.
- **Open:**
  - one home and one signature for `as_cost` and the shaping, in three plans: `Set.as_cost`,
    `Field.as_cost`, and the geometry module's shaping;
  - the conversions' names (`as_set`?);
  - whether steps 4.4–4.6 wait for v0.4.
- **Rows:** [TODO §1.9](TODO.md#19-fields).

## 10. The order: three lanes, two joins (a proposal, not ruled)

It replaces the line of phases 0–8 of 2026-10-26 (the map is in
[the simplification record](../reviews/2026-10-26-plan-simplification.md)). Reviewer C §2.3
found two edges of that line that contradicted the plan docs:
- the one score came before the sampled loop it scores;
- the `StepRollout` fold was placed two phases before the sampled loop it waits on.

The lanes follow the dependencies instead. Each lane starts once the rulings it needs are taken.

| Lane | Workstreams, in order | Waits on |
| --- | --- | --- |
| **The objects-and-draws lane** | §1: the textbook leftovers, distributions and sets reading `params`, the shaping to `core/`, `Set.as_cost`, one parameter-tree module → §3: per-block seeds and disturbance signals, one draw on both backends, `NoiseSource` → §4: the seeded policy | the draws' rulings |
| **The loop lane** (the critical path; it starts first) | §2: step evaluators trace and batch, one same-tick semantic → §5: the pure computer tick (with `Computer.tick` as its oracle), the MPC delay, the caches gone, the step-diagram runner, the sampled face with a digital PID, the `StepRollout` fold, the orchestrator over pure blocks | §2's and §5's rulings |
| **The program lane** | §7: one backend table, the program with parameters, collocation reading the plant, the online call → §8: the native kernel, byte-identical, then its fixed-`K` traced form | §7's rulings |
| **Join 1** | §6: one per-step cost and one failure rule, read by the evaluator, then value iteration and RL | the objects and loop lanes |
| **Join 2** | §8: the fixed-iteration MPC tick, its closed loop in one scan tuned by gradient; §7: the stochastic program as a convention and a test | all three lanes |
| **Last** | §9: the `Field` noun and `QuadraticField`; hardening (S66, with the error family if moved in); the notebooks (R47); the merge | everything |

**The clean-up lane runs beside them** (TODO §2).

**Number changes** (R80–R82):
- an intended change is checked by closeness on the same problem and cost;
- a restructure agrees to `rtol 1e-6`;
- a move or rename stays byte-identical;
- no alias, one sweep.

The intended changes already named, in the order that moves each number once:
1. the same-tick semantic;
2. the MPC delay;
3. the hybrid defaults;
4. the one-score `J`, before any RL re-pin;
5. the cross-backend draws and live disturbances.

When to re-pin (per change, or once) is open.

## 11. Definition of done (a proposal; the maintainer confirms)

From reviewer C §3.3. v0.3 is done when these scripts run as tests:
1. **An LQG loop** with noise on `d` and `n`: the Monte Carlo mean is within the covariance
   prediction's sampling error, and one key gives the same draws on NumPy and JAX.
2. **A digital PID** (`pid % 0.01 @ plant`): `problem.compile(...).evaluate(p)` runs and
   `jax.grad` matches central differences.
3. **One compiled MPC program**, rebound with two plant `mu` values and a moved target, matches
   the rebuilds. `optimizer_method="ipopt"` works on the compiled path.
4. **The same digital PID** run by `compute_trajectory` and scored by `evaluate` gives one `J` to
   `rtol`.
5. **The fixed-iteration MPC's closed loop**: `problem.compile(mpc).evaluate(p)` runs, `vmap`s
   over starts, and its gradient tunes a cost weight.
6. **A `QuadraticField`** serves as a terminal cost and as the certificate's `V`, with the DP and
   RoA baselines unchanged.
7. **Every catalog plant and notebook** is green on the merge gate.

## 12. The core and the features

v0.4 reads the core and never changes it.

| Feature (TODO §4 unless named) | Reads | Plan |
| --- | --- | --- |
| MPPI (S72, MP-1–MP-6) and the MPC planner contract (D27; T6 in TODO §3) | §3, §5, §6 | [mppi.md](mppi.md) |
| Identification and trajectory generation (C4) | §7 (the export with a data cost) | [differentiable-problem.md](differentiable-problem.md) §1 |
| `LQGPlanner` | §3 | [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5 |
| Workspace geometry (A3), then the CBF safety filter (unscheduled) | §9 | [geometry-module.md](geometry-module.md), [cbf-safety-filter.md](cbf-safety-filter.md) |
| The RL follow-ups (C5), the RL heads (open: here or §4), reward scaling (TODO §6) | §4 | [future-policies.md](future-policies.md), [rl-reward-scaling.md](rl-reward-scaling.md) |
| Native solvers beyond the proof of concept (NS-2's tolerance form, NS-3–NS-7) | §7, §8 | [native-optimizers.md](native-optimizers.md) |
| Traceable linear analysis (S73) | the Jacobian | [differentiable-problem.md](differentiable-problem.md) §4.6 |
| The serial chain (C6), pyro parity (C1), C2, C3, S63, S64 | — | [pyro-port-remaining.md](pyro-port-remaining.md) |
| The estimation follow-ups (v0.9, TODO §5): the sampled filters | §5 | [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §6 |
| After v1.0: behavior cloning (a `fit` verb and a dataset noun, open), diffusion policy | §4, §5 | [future-policies.md](future-policies.md) §3–§5 |
