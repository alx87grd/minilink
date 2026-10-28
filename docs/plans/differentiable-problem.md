# The differentiable planning problem: one path from any parameter to any metric

**Status:**
- **Landed on 2026-10-24:** steps 0–6 and the Monte Carlo fold (step 7a):
  - the native `Trajectory`;
  - one scoring formula on the problem's fields;
  - the problem evaluator (`problem.compile`, `evaluate`, `compare`) and its program export;
  - the cost parameters;
  - the draws of the start and of the plant parameters.

  On 2026-10-25, R92 added per-trial realizations of the random blocks inside the evaluated
  loop.
- **The record:**
  - the contract is DESIGN §4–§6 (§6 "One scoring formula", "Stochastic problem", "The problem
    evaluator");
  - the decisions taken without the maintainer are
    [the run's log](../reviews/2026-10-24-v03-run-decisions.md) §2;
  - the rulings are [the rulings record](../reviews/2026-10-22-rulings-session-1.md) (R9–R11,
    R15–R47, R71–R77), [the 2026-10-25 rulings](../reviews/2026-10-25-open-questions-rulings.md)
    (R91–R97) and [the 2026-10-27 decisions](../reviews/2026-10-27-core-vision-decisions.md)
    (R141–R153);
  - the design of the landed steps is in git history.
- **This doc holds what remains:**
  - the `StepRollout` fold, now the two run verbs (R147);
  - step 7: the sources' seeds and the one-tree `problem.realize`;
  - step 7b: the sampled face;
  - step 8: sets as costs, a source policy's samples as leaves;
  - step 9: the material;
  - §9: the one score, its object and per-step cost decided (R141, R142), three points open;
  - traceable linear analysis (S73, §4.6).
- **Two leftovers of landed steps** wait for the maintainer in [TODO.md](TODO.md) §2: the LQR
  warning on a declared `U` (OD-33 of the run's log), and the three demos that teach a hand
  gradient loop (OD-32).

**Where it sits** (the core, [core-architecture.md](core-architecture.md); rows in
[TODO.md](TODO.md)):
- the `StepRollout` fold: §1.2;
- step 8: §1.1;
- step 7, with RN-4 of [randomness.md](randomness.md): §1.3;
- step 7b, with the hybrid loop: §1.5;
- §9's one score: §1.6;
- step 9, with the merge: §1.10;
- S73: any time (R76), a v0.4 row (§4).

The problem evaluator turns a planning problem and a controller into one traceable function
from the parameters to the run's record, and its program export hands that function to the
native `Optimizer`. What is left extends the same path to the parameters and the controllers
it does not reach yet: an open-loop input sequence, the sets' own parameters, the noise
blocks' draws, a sampled controller, and the linear-analysis metrics. Section and step
numbers are kept from the full plan because other documents cite them.

## 1. The target

**Any performance metric that is a traceable function of a run or of a linearization can be
optimized through the native `Optimizer`, with respect to any parameter the problem holds.**
The library provides the metric, JAX the gradient, the optimizer the descent loop; no demo
writes a scan, a cost formula or a descent loop of its own.

The parameters are the columns, the metrics the rows; each cell says whether the pair landed
or names the step of §8 that makes it one line (`—` where the pair has no meaning). On a run,
every metric is a field of the one record `pe.evaluate(params)` returns (DESIGN §6); its scalar
is `objective`, the problem's criterion over the runs' `J` (R45).

| Metric | controller params | open-loop input sequence | plant params | cost and set params | the start (`"x_start"`) |
| --- | --- | --- | --- | --- | --- |
| The run's `cost` and `J` (`Evaluation.cost`, `.J`, `.objective`) | landed | step 8 (a source policy's leaves) | landed | cost landed; sets step 8 | landed |
| The whole run (`pe.trajectory(params)`) | landed | step 8 | landed | — | landed |
| The criterion over the problem's draws (mean, CVaR, worst) | landed (loop noise blocks included, R92; disturbance sources step 7) | step 8 | landed | cost landed; sets step 8 | landed |
| The feasibility and reachability margins (`Evaluation.feasibility_margin`, `.reachability_margin`) | landed | step 8 | landed | step 8 | landed |
| Closed-loop poles, damping ratios, natural frequencies | S73 | — | S73 | — | — (linearized at `x̄`) |
| Gain and phase margins, Bode values, the sensitivity peaks `M_s`, `M_t` | S73 | — | S73 | — | — |

Every cell has the same consumer: the program export (`pe.program(wrt, bounds)`, DESIGN §6),
which today's `Optimizer` solves. The raw gradient over a tree that holds integer leaves (a
`seed`, a step problem's `t_start`) is `jax.grad(f, allow_int=True)` (R152); `pe.program(wrt)`
never sees integers (R131). The worked examples still to come, each one demo once it lands: an
open-loop input sequence by single shooting (today's `cartpole_rollout_gradients` hand loop and
the showcase cell); robust tuning on the mean over a plant-parameter family; parameter
identification from logged data (today's `params_gradient` and the showcase cell, C4); the
plant-parameter sensitivity of poles and margins (S73).

## 3. The objects still to change

| Object | Today | After (step) |
| --- | --- | --- |
| `StepRollout` (`core/discrete/step_rollout.py`) | the step twin of `Trajectory`: the hybrid simulator's computer record and the MPC's dual-rate view | gone: `compute_rollout` returns a `Trajectory` on every `StepSystem` (R147; the fold, §8) |
| `Set` | `margin` traces; `contains` is that test on `xp` (landed); the library sets hold no parameters | reads `params` with A5, `Set.as_cost(weight=, shaping=)` (R133; step 8) |
| `TrajectorySource` (`blocks/sources.py`) | an open-loop plan as a block; its `h` interpolates on the array module and traces (OD-30); its table an attribute (`sample_values`) | the table in `params["values"]`, so an open-loop plan is a policy and its samples are leaves (step 8) |

## 4. Implementation rules

The rules of the landed steps (§4.1–§4.5: the native `Trajectory`, the one evaluation, the run
out of the scan, the engine and its program, the optimizer upgrade) are DESIGN §4–§6. One
remains.

### 4.6 Traceable linear analysis (S73, independent of the evaluator)

`poles`, `frequency_response` and `margins` through the array module, so the sensitivity of
closed-loop poles, damping ratios, Bode values and margins to any plant or controller param is
`jax.grad` of the existing verbs on `jacobian("f", "params")`. The math lines stay the
textbook ones: `C (jωI − A)⁻¹ B + D` as one batched solve; eigenvalues through
`xp.linalg.eigvals`, whose derivative is the classical `vᴴ (∂A/∂p) u / (vᴴ u)`; Brent's method
on the crossovers replaced by a fixed number of Newton steps from the grid bracket, so the
crossover frequency carries the implicit-function derivative. Beside the crossover margins,
the sensitivity peaks `M_s = max_ω |S(jω)|` and `M_t = max_ω |T(jω)|` (and a `logsumexp`
surrogate over the grid) as traceable metrics: a bound `M_s ≤ 1.5` guarantees a gain margin
above `M_s / (M_s − 1)` and a phase margin above `2 asin(1 / 2M_s)` with no crossover to jump,
the smooth constraint for loop shaping through the export. Zeros stay NumPy (a generalized
eigenproblem JAX lacks); root-locus branch matching stays NumPy (plotting). Documented
non-differentiable spots: repeated poles, the switch of the minimum margin between crossovers;
the nonsymmetric eigensolver is CPU-only.

## 5. Guardrails for the open steps

The guardrails of the landed path (the margins carry the gradient, the setpoint is the
controller's, a block is not a leaf list, `Xf` written, `U` information, the program flat) are
DESIGN §6. Three steer steps 8 and 9 and S73:

- **Single shooting is not collocation.** The export with a source policy's leaves is single
  shooting with a held input, the showcase's hand loop as one line; through an unstable plant
  over a long horizon its gradients explode or vanish and the landscape is nonconvex, where
  collocation's defect constraints break the chain and expose sparsity. `ShootingTranscription`
  and `direct_collocation` stay the trajectory-optimization path; the export's shooting is an
  evaluation and tuning tool.
- **Backpropagation through the dynamics and policy gradients are complementary.** The engine
  differentiates *through* `f` and the reparameterized draws; REINFORCE, PPO and SAC
  differentiate the log-policy score and never `f`. Short smooth horizons favour the former by
  orders of magnitude; long horizons, saturations and stiff dynamics the latter. One demo puts
  `problem.compile(policy)` and `ReinforcementLearningPlanner` side by side on one problem and
  one `NeuralPolicy` (the name after R151; step 9).
- **Crossover margins jump; sensitivity peaks do not.** A gain or phase margin is defined at a
  crossover that can appear, vanish or switch during a descent, and `eig` has no derivative at
  a repeated pole; `M_s`, `M_t` are the smooth robustness bounds for an NLP, the margins the
  numbers a course reports (§4.6).

## 8. The steps still open, bottom-up

One commit per step or sub-step; each records a seeded baseline of every branch it touches
before any edit, runs it twice and `cmp`s, and ends byte-identical unless the step names its
one intended change, which gets its own test and a ledger line. Every math line reads next to
`planning/dynamic_programming/dp.py` before it lands. The order and the gates are the TODO rows
(listed under "Where it sits"); this list is the content.

- **Step 1, its last commit: the `StepRollout` fold, as the two run verbs** (R147; OD-36).
  - `compute_rollout` on every `StepSystem` returns a `Trajectory` of the ticks, `t = k` on a
    pure step system and `t = k·Δ` on a sampled one: what `evaluate` scores and `jax.grad`
    differentiates. The evaluator's `rollout` keeps its one shape `(N+1, n)`.
  - `compute_trajectory` wherever continuous time is underneath (a `DynamicSystem`, a
    `SampledSystem` or `SampledDiagramSystem` with a continuous source) is the same scan with
    its substeps kept, through one new evaluator method beside `rollout` (its name for the names
    sweep), so its value at each tick is the tick value, bit for bit; `solver="scipy"` is the
    adaptive cross-check.
  - `StepRollout` is today the record of `HybridSimulator` and `Computer` and the MPC's
    dual-rate view; those go with the hybrid loop (`computer @ plant` a `SampledDiagramSystem`,
    R146) and the MPC's redesign (`PlanFollower`, [planner-and-mpc.md](planner-and-mpc.md)
    §6.4), so the fold lands with them.
- **Step 7, the sources' seeds and the one-tree `realize`** (with RN-4 of
  [randomness.md](randomness.md)). Per-trial realization of random blocks inside the evaluated
  loop landed in v0.3 (`trial_seeds`, R92). The step moves it into
  `problem.realize(key, n_trials) -> params` (R144): one tree, `x_start` a leaf with a trial
  axis; a law in a leaf (the start, a plant parameter) drawn to a value from a sub-key by its
  path; a source block (a noise block, the seeded policy) given an integer seed leaf; one key,
  streams by name, the same on both backends. Disturbances are source blocks only (R128): the
  shortcut `disturbances={"d": block}` wires them onto their ports, so `evaluate` stops holding
  the ports at nominal (today `warn_if_disturbed` in `planning/evaluator.py`). With R143 there
  is no `StochasticPlanningProblem`: both problem classes realize the same way.
- **Step 7b, the sampled face** (with the hybrid loop). A sampled controller (a digital PID
  `pid % 0.01`, a held neural policy, MPPI, the RL policies, the MPC in either form of
  [planner-and-mpc.md](planner-and-mpc.md) §6) closes on the plant as a `SampledDiagramSystem`
  (R146), and `problem.compile` runs its tick scan, so `trajectory`, `evaluate` and `program`
  serve both kinds (R105, one compile, one evaluate, the same `J`). The refusal goes (today
  `require_continuous` in `planning/evaluator.py` refuses a `StepSystem` controller); a
  continuous law's numbers are unchanged.
- **Step 8, sets as costs, the lowerings, the metrics as constraints.** `shaping.py`
  (today `planning/spatial/shaping.py`) to `core/`, `Set.as_cost(weight=, shaping=)` (R133);
  sets reading `params`; `TrajectorySource` with its table in `params` (its `h` already traces),
  so the start `"x_start"` and `"controller.values"` are decision leaves with `Xf` lowered; the
  robust-tuning and identification demos on the export; the metric objective and the
  margin-floor example once S73 lands.
- **Step 9, material and contracts.** Each step updates DESIGN when it lands (§4 `Trajectory`
  and parameters, §5 the rollout, §6 the evaluation and the export) and ROADMAP §3's ledger;
  step 9 rewrites the showcase cells on the engine (tutorial 11 moved to `evaluate` with step
  7a), and K4 merges the compile demos (one per feature). Whether the three demos that teach a
  hand gradient loop move onto `Optimizer` is the maintainer's call (OD-32; TODO "S74, the
  rest").

**In parallel, any time:** S73 (§4.6), self-contained for a cloud agent.

---

## 9. One score

**Decided in part (2026-10-27, preliminary):** the object and the per-step cost
([R141, R142](../reviews/2026-10-27-core-vision-decisions.md)). Three points stay open (§9.4).
The question and the reviewers' views are in [QUESTIONS.md](../dev/QUESTIONS.md).

The idea (the maintainer, 2026-10-26): once `dt` and the scheme are set, a continuous rollout is
a step system, so the evaluation unifies from there, with time as an index.

### 9.1 The object: the compiled step face (R141)

- **Two problem classes, mirroring the systems.** `PlanningProblem`, unchanged (`tf`,
  `discount_rate`, a `CostFunction` whose `g` is a rate), and `StepPlanningProblem`, on a
  `StepSystem` (a horizon `N`, a discount per step `gamma`, a `StepCost` with stage cost
  `c(x, u, k)` and terminal `h(x, k)`). Both are descriptions; stochastic by their parts, not by
  a class (R143).
- **The step face is compiled.** `problem.compile(dt=, solver=, substeps=)`, with no controller,
  gives `transition(x, u, k, params) -> (x_next, c_k)`, `N = tf/dt`, `gamma = e^{−ρ dt}`, the
  sets at `t_k`, the price. `problem.compile(controller, dt=)` (today's `ProblemEvaluator`) is
  one scan of the same `transition`, so `evaluate`, `program` and the Monte Carlo score through
  the same `c_k`. A `StepPlanningProblem` compiles to the same face, with no `dt`.
- **The explicit discretization.** `discretize(problem, dt, *, solver=, substeps=)` (facade
  `problem.discretize(dt)`) returns a `StepPlanningProblem` on the plant's `SampledSystem`
  (R146), compiling to the same face to the bit (one test pins it). Its cost is a
  `SampledCost(g, h, plant, dt, *, solver=, substeps=)` (R154), a `StepCost` whose `c(x, u, k)`
  integrates `e^{−ρτ} g` over one period from `x` under the held `u` when called directly, with
  the plant's solver and substeps, and whose `h(x, k)` is the terminal cost at `t_k`; the compiled
  face fuses it into the transition, so the direct call is the only second plant evaluation.
- **Who reads which face.** Value iteration, the RL environment, the judge, shooting and the
  native MPC read only the step face and cannot tell a native problem from a sampled one.
  Collocation, the continuous LQR, the linear analysis and `solver="scipy"` read the continuous
  description.
- **Time.** With `t_start` (R148) the samples are `t_k = t_start + k·dt`; the discount and the
  horizon count from `t_start`, costs and sets read absolute time
  ([planner-and-mpc.md](planner-and-mpc.md) §5). The RL planner's discount is the face's
  `gamma` by default; its `gamma=` is a training override only (R150).
- **Open, internal:** whether the open-loop face is `ProblemEvaluator` with no controller or a
  smaller sibling the closed loop builds on; settled when the code lands.

### 9.2 The per-step cost (R142)

- `c_k` is integrated with the state inside the transition, by the same scheme and substeps
  (one plant evaluation per step), not by a post-pass over a recorded run.
- The stationary form: `c_k = ∫₀^dt e^{−ρτ} g dτ`, `J = Σ γ^k c_k + γ^N h`, so `c_k` does not
  depend on `k` and value iteration's backup `c + γ J'` reads it as is.

### 9.3 What it changes in today's code

- **Already discrete:** the evaluator's rollout is the step model
  (`core/compile/evaluators/stepping.py`, DESIGN §5 "The step model"), the same one-period body
  as `discretize(plant, dt, solver=, substeps=)`, the input held over each period; `score_run`
  (`planning/evaluation.py`) reads the samples `(t_k, x_k, u_k)`.
- **The quadratures become one.** Today one policy gets three or four close `J`s: a left
  Riemann sum `g·dt` in `dp.py` and the RL reward, the trapezoid in `score_run`
  (`CostFunction.integrate`, `core/costs.py`), shooting's trapezoid with a first-order hold. Under
  forward Euler R142's `c_k` is today's `g·dt` to the last bit, so value iteration and RL do not
  move; the judge's trapezoid `J` changes once (an intended change, R80). A logged run scored by
  `evaluate_trajectory` keeps the trapezoid and says so.

### 9.4 Still open

Three points, kept with their options in one home, [QUESTIONS.md](../dev/QUESTIONS.md) (the
one-score entry), for the maintainer:
1. time-varying sets and the margins between samples;
2. one failure rule and the input hold;
3. shooting's reported score.

