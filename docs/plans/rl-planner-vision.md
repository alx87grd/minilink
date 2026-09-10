# Reinforcement learning as a planner — vision and plan (draft)

Status: **in progress** (2026-09-10). Steps R1–R5b landed the same day
(`planning/reinforcement_learning/`, `planning/evaluation.py`,
`control/neural.py`, the problem/cost semantics); R6–R7 open. Decisions
D1–D5 applied as recommended; D3 landed as the single block with a feature
function (the composite, diagram-visible block is still open).
Lane: research → provisional planning band at v0.2.

Builds on: [standard-planning-problems.md](standard-planning-problems.md)
(the deterministic / stochastic / robust taxonomy — adopted here, not
redrawn), [neural-blocks-collection.md](neural-blocks-collection.md) (NN
blocks are ordinary blocks, weights are `params`, training lives outside),
[planning-pipeline-architecture.md](planning-pipeline-architecture.md)
(result families), DESIGN §6.

Evidence: `minilink/experimental/ppo_jax.py` and the six demos in
`examples/experimental/rl/`, with the tuning record in
[RL_README.md](../../examples/experimental/rl/RL_README.md).

---

## 0. What the prototype taught us (the facts the design must serve)

1. **The plant + cost + bounds already define the RL problem.** Every demo
   was `PPO(plant, cost, dt, tf, reset distribution, bounds rule)`. There is
   no algorithm-specific problem; there is a *stochastic* one (starts are
   drawn) with a *criterion* (expected return).
2. **The result is a policy block.** `ppo.controller @ plant` closed every
   loop, `plot_control_law` drew every law, `compute_trajectory` tested it at
   a finer step than training. This is exactly the DP planner's shape
   (`get_controller()` → a static `System`).
3. **A feature map was needed on four of six tasks** (periodic angles,
   order-one scaling, track-relative sensing). It is part of the *policy's
   parameterization*, not of the problem: the law is still `u = π(x)`.
4. **Normalized actions and scaled features are the difference between
   learning and not learning.** They derive from the port bounds and the
   state box, i.e. from data the `System` already carries.
5. **The state box is the sharpest semantic gap.** Trajopt treats `X` as a
   hard constraint, DP charges a finite `out_of_bound_cost` (a solver
   option), the prototype offers truncate-with-bootstrap or terminate-with-`h`.
   The same cost function meant three different things, and every RL failure
   this week was the policy exploiting one of those meanings.
6. **`tf` means two things.** For trajopt it is *the* horizon. For RL it was
   an *episode length* on an infinite-horizon task, which is why truncation
   at `tf` had to bootstrap while termination on exit must not.
7. **Bare JAX was enough**: ~500 lines including the MLP, Adam, GAE and the
   controller block; 30k to 90k plant steps per second on a laptop CPU, ten
   times the Gymnasium + stable-baselines3 path.

---

## 1. Target mental model

```text
StochasticPlanningProblem   what world, what is random, what we want (E[J] or max J)
        │
        ├── solve   ──►  Planner (policy family): ReinforcementLearningPlanner, DP, gain search
        │                     └─► PolicyPlan  ──►  get_controller() → Controller block
        │                                            (NeuralPolicyController: features ∘ MLP ∘ denormalize)
        ├── solve   ──►  Planner (trajectory family): trajopt, RRT on problem.nominal()
        │
        └── evaluate(controller) ──►  MonteCarloEvaluator → mean / std / worst / failure rate
                                       (same tool for LQR, DP, MPC, RL, hand-tuned PID)
```

Design laws carried over from the taxonomy plan and confirmed by the
prototype:

- One description, two verbs: `solve(problem)` and `evaluate(problem, κ)`.
- Split on **uncertainty + criterion**, never on algorithm. RL gets no
  special problem class; it consumes the stochastic one.
- The policy family (open-loop schedule, LQR gains, PID, NN weights) attaches
  at solve time, as a *block*.
- `PlanningProblem` stays exactly as it is. The stochastic class is a sibling
  with the same spine, plus `nominal()` back to it.

---

## 2. The problem description

### 2.1 `StochasticPlanningProblem`

Same spine as `PlanningProblem` (`sys, cost, X, U, Xf, tf, params, metadata`)
with the three uncertainty channels of the taxonomy, made concrete by what
the prototype and the Monte Carlo use case need:

```text
x0          ~ Distribution over the state         (replaces the singleton X0)
theta       ~ Distribution over sys.params        (domain randomization, robustness)
w           ~ {port: SignalDistribution}          (seeded disturbance signals on named input ports)
criterion     Expectation (default) | WorstCase | CVaR(alpha)
```

- **Distributions** are a tiny duck type: `sample(key, shape=())` that
  traces under JAX (resets happen inside `lax.scan`) and works with a NumPy
  `Generator` for plain Monte Carlo; an optional `support` Set for the
  `nominal()` bridge and for plots. `Gaussian(mean, std)`, `Uniform(lb, ub)`,
  `Particles(x_list)`, `AlongPath(track, ...)`-style task-specific callables
  (the car's reset) all fit. This is the `reset_mode` argument of the
  prototype, promoted to a problem field.
- **Parameters**: `theta ~ p_θ` samples a params dict per episode; the
  compiled `f_p` / `rk4_step_trace_p` tier already takes params, so a rollout
  batch vmaps over sampled params at no new cost (the `rollout_batch`
  facade from Phase 2 is the seam).
- **Disturbances**: a signal distribution per named input port
  (`PendulumWithNoisePort` already exposes `w` and `v`); `sample(key)` returns
  a callable `w(t)` or a per-step draw. Process noise *inside* `f` stays
  deferred, as the taxonomy says.
- **Criterion**: default expectation; worst case and CVaR are evaluation
  reducers first (a Monte Carlo report can show all three) and synthesis
  targets later.

`nominal()` returns the `PlanningProblem` with the mean start, mean
parameters and no disturbance, so trajopt, LQR and RRT keep working inside
the richer task, and MPC on the nominal problem can be *evaluated* on the
stochastic one.

### 2.2 The Gymnasium bridge becomes a view

`Sys2Gym(problem)`: `reset` samples `x0` (and `theta`, `w`), `step` is the
compiled RK4 step, `reward = -g dt`, done follows the exit rule of §4. The
current `Sys2Gym(sys, cost, ...)` constructor stays as the shortcut that
builds the stochastic problem from its arguments. Nothing RL-specific lives
in `interfaces/` beyond the adapter.

### 2.3 Monte Carlo evaluation is the second verb

`MonteCarloEvaluator(problem, n_trials, key)` scores any controller block on
the stochastic problem: vmapped closed-loop rollouts on JAX (`ctl @ plant`
compiled once), sequential `Simulator` runs on NumPy; report mean / std /
worst / failure rate (exits) / per-trial `J`, optional trajectories. It is
the tool the maintainer described for testing controllers over initial
states, disturbances and parameters, and it is what an RL planner logs
during training. `PolicyEvaluator` (grid DP) stays as the exact
cost-to-go of a law on the deterministic problem.

---

## 3. The planner and its result

### 3.1 `ReinforcementLearningPlanner` in `planning/reinforcement_learning/`

DP and RL are both policy-family planners; they should sit side by side and
read the same way:

```python
planner = DynamicProgrammingPlanner(problem, x_grid=(201, 201), u_grid=(21,), dt=0.05)
planner = ReinforcementLearningPlanner(problem, dt=0.05, policy=MLPPolicy(hidden=(64, 64)))

plan = planner.solve(timesteps=1_000_000)   # PolicyPlan (weights, spec, history, metadata)
ctl = planner.get_controller()              # NeuralPolicyController, ctl @ plant
planner.plot_learning_curve()
planner.solve_trajectory_from(x0)           # rollout of the learned law → TrajectoryPlan
```

- `solve()` / `solve_policy()` return a `PolicyPlan` whose payload is the
  policy spec + weights + training history; `solve_steps(n)` is the
  incremental call (the notebook's 20k + 80k), like DP's `solve_steps`.
- `solve_trajectory_from(x0)` rolls the learned law out so `plot_solution`
  and `animate_solution` work unchanged (the two output families the
  maintainer wants on one planner).
- The algorithm is a strategy inside the planner: `algorithm="ppo"` first,
  the class structure leaving room for SAC / evolutionary gain search later
  (the latter is the PID-tuning use case of the taxonomy on the same
  planner).
- Flat Tier-1 kwargs mirror `TrajectoryOptimizationOptions` /
  `DynamicProgrammingOptions`: `n_envs, n_steps, batch_size, n_epochs,
  learning_rate, gamma, gae_lambda, clip_range, ent_coef, log_std_init,
  episode_length, seed`; `options=PPOOptions(...)` is Tier 2.

### 3.5 An algorithm family, not one algorithm (added 2026-09-10)

The planner must host several RL methods the way stable-baselines3 hosts
PPO, A2C, SAC, TD3 and DQN: shared machinery, one class per method, a new
method written without touching the others. SB3's decomposition, mapped
onto minilink parts that already exist or fall out of §2–§3:

| SB3 piece | Role | minilink part |
| --- | --- | --- |
| `VecEnv` | batched `reset` / `step` / reward / done | `RolloutEnvironment`: pure functions on the compiled plant built from the stochastic problem (`reset(key)`, `step(x, t, u) → x', r, terminated, truncated`), vmapped by the collector |
| `BasePolicy` + `ActorCriticPolicy` / `SACPolicy` | networks + distribution head | `NeuralPolicyController` (the deterministic law, a block) + a **policy head** for training: `GaussianHead` (state-independent log-std; PPO family) or `SquashedGaussianHead` (state-dependent std, tanh; SAC family); critics `ValueFunction(x)` / `QFunction(x, a)` on the same MLP block |
| `RolloutBuffer` / `ReplayBuffer` | experience storage | on-policy **collector**: one `lax.scan` over `n_steps × n_envs`, returns the batch with log-probs and values; off-policy **replay buffer**: fixed-size JAX arrays with a cursor, uniform sampling |
| `OnPolicyAlgorithm` / `OffPolicyAlgorithm` | the train loop | the planner's `learn(timesteps)` runs one of two loops chosen by the algorithm's `on_policy` flag: collect → update, or step → store → sample → update |
| `PPO.train()`, `SAC.train()` | the update rule | an `Algorithm` object: `init(key, policy, critic) → train_state`, `update(train_state, batch, key) → train_state, stats`, `losses` written as readable math |
| callbacks / logger | monitoring | `history` records + the Monte Carlo evaluator for periodic deterministic scoring |

Contract that keeps a new method small: an algorithm owns **only** its
update rule and its train state (optimizer moments, target networks,
temperature). Environment, collectors, heads, critics, optimizer, the
result type and the controller block are shared. Adding SAC then means one
file: the squashed head, twin-Q targets with Polyak averaging, the
temperature loss, and `on_policy = False`; the replay path already exists.
Adding A2C is PPO without the clip. Adding a gradient-through-`f` policy
search is an algorithm whose "batch" is the differentiable rollout itself.

Layout inside `planning/reinforcement_learning/`:

```text
environment.py   RolloutEnvironment (problem + dt + exit rule + episode length)
heads.py         GaussianHead, SquashedGaussianHead   (sample, log_prob, mean, entropy)
critics.py       ValueFunction, QFunction
collect.py       rollout (on-policy scan, GAE), ReplayBuffer (off-policy)
optim.py         adam, clip_by_global_norm (Optax duck-typed if passed)
algorithms/      base.py (Algorithm), ppo.py, sac.py, ...
planner.py       ReinforcementLearningPlanner: learn loops, PolicyPlan, get_controller
```

The proof that the seam is right is a second algorithm of the *other*
family (off-policy SAC) landing without edits to the PPO file, the
collector, or the planner loop beyond the `on_policy` branch. That is step
R5b below.

### 3.2 The policy is a block: `control/neural.py`

`NeuralPolicyController(sys, features=..., hidden=..., activation=...)` is a
`Controller` with `measurement_port="x"`, `control_port="u"`, whose
`params` hold the weights (the neural-blocks contract), and whose output is

```text
u = u_mid + u_half * clip(MLP(features(x)), -1, 1)
```

so the planner trains `params` and the user never sees normalization. A
critic block is planner-internal (it is not part of the law). Users who
want another architecture pass any block exposing `params` and a
JAX-traceable compute; the planner only needs `π(params, x)` and the
pytree of trainable leaves.

### 3.3 Features: block or function?

Three options, with a recommendation:

| Option | Shape | Pros | Cons |
| --- | --- | --- | --- |
| A. Function argument | `features=callable` inside the policy block (prototype) | one line, fully general, traces | invisible in `plot_diagram`; not reusable across laws |
| B. Separate block | `FeatureMap >> NeuralPolicy` in the diagram | visible topology; `AngleFeatures`, `Normalize`, `TrackFeatures` become catalog blocks reusable for any controller | one more subsystem per law; `Controller` roles need the composite to still read as one block |
| C. Composite block (recommended) | `NeuralPolicyController` *is* a small diagram `features >> mlp >> denormalize`, built by a factory; `plot_diagram()` opens it, `ctl @ plant` sees one block | both readable and composable; the feature blocks are ordinary `StaticSystem`s in `blocks/`; the MLP `params` are the only trainable leaves | a composite `Controller` is new (the hybrid MPC block is the precedent) |

Standard feature blocks to seed `blocks/`: `Normalize(lb, ub)` (from state
bounds; the default when `normalize=True`), `AngleFeatures(indices)`
(`cos, sin`), `Scale(gains)`. Task-specific maps (the car's track features)
stay functions in the demo, wrapped in a `FunctionBlock`.

### 3.4 Normalization as an argument

`normalize=True` on the planner/policy factory means: observation features
scaled by the state box, actions mapped from `[-1, 1]` to the port bounds,
rewards left alone but the cost scale checked (warn when the mean episode
return is outside ~[1, 1000]). Running-statistics normalization
(VecNormalize) is deferred; the box-derived version fixed every scale
problem this week.

---

## 4. Cost, horizon and termination — the deep question

### 4.1 Where the three tools disagree today

| | trajopt | DP | RL prototype |
| --- | --- | --- | --- |
| horizon | `problem.tf` finite, required | `tf` finite (`final_time`) or infinite (to tolerance, discount `alpha`) | `tf` = episode length; `gamma` method-side |
| terminal cost `h` | at `tf` | initial `J = h` for finite `tf`; unused infinite | at `tf` only when terminating; truncation bootstraps |
| leaving `X` | infeasible (hard constraint) | `out_of_bound_cost` (planner option, 1e6) | truncate + bootstrap, or terminate + `h(x_exit)` |
| discount | none | `alpha` option | `gamma` option |

The cost function is the same object in all three, and it is silent on all
four rows. That silence is what the RL policies exploited.

### 4.2 Options

**O1 — keep `CostFunction` pure; all of it is method options.** Nothing
changes; each planner documents its convention. Cheapest, and it is what
DP already does with `out_of_bound_cost`. Cost: the meaning of "this policy
scored J" differs per tool, and RL users must rediscover the penalty rules
of RL_README §1 for every task.

**O2 — declare the exit rule on the problem, once.** `PlanningProblem`
(and the stochastic sibling) gains

```text
on_exit   : "infeasible" (default: X is a hard constraint)  |  "terminate"
exit_cost : float or callable(x, t) → cost charged when a trajectory leaves X
```

Every tool reads the same declaration: trajopt keeps `X` hard (`exit_cost`
unused), DP's `out_of_bound_cost` defaults to `exit_cost`, RL terminates the
episode and charges `exit_cost` with **no bootstrap**. The number is
tunable in one place, and its meaning is stated: *J of a trajectory that
leaves X is the cost accrued so far plus `exit_cost`*. This is the
Gymnasium "terminated" semantics made explicit and shared with DP.

**O3 — make `h` the cost of stopping, wherever you stop.** Redefine the
terminal cost as `h(x, t)` = the cost of the episode ending at `(x, t)` for
any reason (horizon, exit, goal reached). Mathematically clean and it gives
the exit penalty a *shape* (distance to goal, not a constant). But `h` then
carries two jobs (end-of-horizon weighting for trajopt, and the exit
penalty), and the prototype showed the exit penalty needs a task-dependent
*scale* (too small: escape; too large: timid), which is a training concern,
not a modeling one.

**O4 — horizon and discount on the cost, exit rule on the problem
(recommended).**

- `CostFunction` states its **horizon kind** and **discount**:
  `horizon="finite" | "infinite"` (default inferred from `problem.tf`:
  finite `tf` ⇒ finite) and a continuous discount rate `rho ≥ 0`
  (`J = ∫ e^{-ρt} g dt`; default 0). Methods convert: DP `alpha = e^{-ρ dt}`,
  RL `gamma = e^{-ρ dt}` by default. RL keeps a method-side `gamma` override
  because practitioners also use it as a variance knob (the pendulum learned
  fastest with 0.97 on an undiscounted task).
- `h` keeps its single textbook job: the cost at the **end of a finite
  horizon**. Infinite-horizon costs have no `h`.
- The **exit rule** (O2) lives on the problem, next to `X`, because it is a
  statement about the allowed set, not about the objective.
- `tf` on the problem keeps its meaning (the horizon). RL gets a separate
  `episode_length` option, used only when `tf = +inf`; then truncation
  bootstraps (correct for an infinite-horizon value), and when `tf` is
  finite the episode ends at `tf` with `h` and no bootstrap (correct for a
  finite-horizon value). Consequence worth stating: a finite-horizon optimal
  policy is time-varying, so the policy block must then take `t` as a
  feature; DP's `solve_steps` has the same property and today returns the
  last stationary table.

O4 answers the maintainer's question directly: **leaving the bounds is a
constraint violation whose price is declared on the problem, not a terminal
cost in the cost function.** The terminal cost stays the end-of-horizon
weighting it is in every textbook. Trajopt, DP and RL then compute the same
`J` for the same trajectory, which is what makes the Monte Carlo evaluator
meaningful across tools.

### 4.3 The default price

The prototype's rule of thumb becomes the default: `exit_cost` should exceed
the cost-to-go of completing the task from a typical start. A planner can
*estimate* that (the RL critic's `-V` at the starts, DP's cost-to-go) and
warn when the declared price is below it. DP's 1e6 default is fine for
grids; for RL it makes the policy timid, so the RL planner should refuse to
inherit an absurd number silently and ask for an explicit one when
`problem.exit_cost` is unset.

---

## 5. Bare JAX or Flax / Optax

| | bare JAX (prototype) | Flax + Optax | hybrid |
| --- | --- | --- | --- |
| dependencies | none beyond `jax` | two more optional extras | Flax optional, Optax optional |
| readability | `a = tanh(W @ z + b)`; Adam in 10 lines | module ceremony; opaque `TrainState` | core readable, extras behind `interfaces/` |
| params contract | already a pytree in `params` | Flax params are a pytree too, but nested under module names | wrap: `interfaces/flax.py` exports a Flax module as a `StaticSystem` with `params` |
| optimizers / schedules | Adam, clipping done; schedules trivial | schedules, AdamW, etc. for free | Optax accepted where an `optax.GradientTransformation` is passed |
| maintenance | ~150 lines we own | tracking two fast-moving APIs | small adapter surface |

Recommendation: **bare JAX for the library**, consistent with the neural
blocks plan and with the maintainer's "as simple as possible until it
works". Accept an optional `optimizer=` argument that duck-types Optax
(`init`, `update`) so schedules are one import away, and keep
`interfaces/flax.py` as the place where a user's Flax module becomes a
block. Architecture configuration is a small spec, not a framework:
`MLPPolicy(hidden=(64, 64), activation="tanh", log_std_init=0.0)` and
`MLPCritic(hidden=(64, 64))`, plus "bring your own block".

---

## 6. Placement

| Piece | Home | Note |
| --- | --- | --- |
| `StochasticPlanningProblem`, distributions | `planning/problems.py`, `planning/distributions.py` | taxonomy plan placement |
| `ReinforcementLearningPlanner`, PPO update, GAE | `planning/reinforcement_learning.py` (+ `ppo.py`) | sibling of `dp.py` |
| `NeuralPolicyController`, `MLP` layers | `control/neural.py`, `blocks/neural/` | neural-blocks plan |
| `Normalize`, `AngleFeatures`, `Scale` | `blocks/` | reusable beyond RL |
| `MonteCarloEvaluator` | `planning/evaluation.py` | the second verb |
| `Sys2Gym(problem)` | `interfaces/gymnasium.py` | view, not math |
| exit rule / horizon fields | `planning/problems.py`, `core/costs.py` | §4 |

Dependency law unchanged: `core/` knows nothing of planning; the planner
imports blocks and problems; `interfaces/` imports the planner's problem
type, never the reverse.

---

## 7. Phased plan (steps sized for iteration, not a commitment)

| Step | Deliverable | Done when |
| --- | --- | --- |
| **R0** | This draft reviewed; decisions D1–D6 below ruled | review queue cleared |
| **R1** (done) | `CostFunction.horizon` / `rho`; `PlanningProblem.on_exit` / `exit_cost`; DP reads `exit_cost`; DESIGN §6 states the shared `J` | DP and trajopt demos unchanged in output; contract test that the three tools agree on `J` of one trajectory |
| **R2** (done) | `StochasticPlanningProblem` + distributions + `nominal()` | the drone notebook's setup expressed as a problem; `Sys2Gym(problem)` passes the existing gym tests |
| **R3** (done) | `MonteCarloEvaluator` on JAX (`vmap`) and NumPy | report for LQR vs DP vs PPO on the pendulum over 100 starts |
| **R4** (done) | `blocks/`: `Normalize`, `AngleFeatures`; `control/neural.py`: `NeuralPolicyController` composite | `ctl @ plant`, `plot_diagram`, `plot_control_law`, both backends |
| **R5a** (done) | `ReinforcementLearningPlanner` + the shared machinery of §3.5 with PPO as the first algorithm; `PolicyPlan`, `get_controller`, `solve_trajectory_from` | the experimental demos re-expressed through the planner with the same outcomes and times |
| **R5b** | SAC as the second algorithm (off-policy family: squashed head, twin Q, replay) | **done**: `algorithms/sac.py` + `SquashedGaussianHead`; pendulum swing-up by ~19k steps; the PPO file was not edited, the planner gained the `on_policy` branch only |
| **R6** (done) | Teaching entry: `examples/learn/intro/11_reinforcement_learning.ipynb` (stochastic problem, Monte Carlo on a PD law, the RL planner, the differentiable closed loop) and `examples/demos/rl/` (five official demos mirroring the experimental scripts) | notebook smoke; examples README row |
| **R7** | Retire `experimental/ppo_jax.py`; TRL ledger row moves to Planning / RL | nothing imports the experimental module |

Deferred: SAC or other off-policy methods, running-statistics normalization,
process noise inside `f`, the robust (minimax) problem class, GPU claims.

---

## 8. Decisions for the review queue

- **D1 — exit rule on the problem (O2/O4) vs method options (O1) vs `h`
  doing both jobs (O3).** Recommendation: O4.
- **D2 — horizon kind and discount on `CostFunction`** (O4) vs on the
  problem only. Recommendation: on the cost, since a cost without a horizon
  kind is not a well-posed objective; `tf` stays on the problem.
- **D3 — feature layer: function, separate block, or composite block.**
  Recommendation: composite (C), with function (A) kept as the one-line
  escape.
- **D4 — bare JAX vs Flax/Optax.** Recommendation: bare JAX, Optax
  duck-typed, Flax in `interfaces/`.
- **D5 — planner naming and home:** `ReinforcementLearningPlanner` in
  `policy_synthesis/` beside DP, algorithm as a strategy, vs `PPOPlanner`
  as a class. Recommendation: one planner, `algorithm="ppo"`.
- **D6 — time-varying policies for finite `tf`:** feed `t` to the policy
  block (and to DP tables) or restrict RL to `tf = +inf` with an episode
  length. Recommendation: infinite-horizon first (every demo this week);
  finite horizon with `t` as a feature in a later step.

---

## 9. Non-goals

- A batched physics engine or GPU-first RL stack (DESIGN §1 scope).
- Replacing the Gymnasium bridge: it stays the door for external RL
  libraries and the SB3 notebooks.
- Model-free is the target of this plan; differentiating through the
  compiled rollout (already shown in `cartpole_rollout_gradients`) is a
  different planner family (policy search by gradient through `f`) and is
  worth its own short plan once the policy block exists.
