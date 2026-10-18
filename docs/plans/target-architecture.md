# The target architecture: one traceable path from any parameter to any metric

Status: **ideas to explore and adjust** (2026-10-18), consolidated from one morning's
discussion with the maintainer; nothing here is decided unless it says so. What is decided
lives elsewhere: the compiled evaluator and its steps in
[differentiable-cost.md](differentiable-cost.md), the cost parameters in
[cost-params.md](cost-params.md), the fields in [fields.md](fields.md), the open steps on
the workboard ([TODO.md](TODO.md) §4). The counts behind the verdict are in
[the core-objects review](../reviews/2026-10-18-core-objects-review.md).
Rung: v0.3 and after. Lane: research first.

## 1. The target, in one sentence and one table

**Any performance metric that is a traceable function of a run or of a linearization can be
optimized through the native `Optimizer`, with respect to any parameter set the problem
holds.** The library provides the metric as a verb, JAX provides the gradient, the optimizer
provides the descent loop; no demo writes a scan, a cost formula or a loop of its own.

The parameter sets are the columns; the metrics are the rows; each cell says where that pair
stands: traced today by hand in a demo, the step of the evaluator plan that makes it one
verb, the workboard row or the section below that proposes it, `—` where the pair has no
meaning.

| Metric (a verb on the compiled problem or the linearization) | controller params | open-loop input sequence | plant params | cost, set, disturbance params | `x0` |
| --- | --- | --- | --- | --- | --- |
| Trajectory cost `J` of one run | hand scan today → step 3 | step 3 with the open-loop `compile()` (§4.6) | hand scan today → step 3 | A2 (cost), A5 (sets) → step 3 | step 3 |
| Whole trajectory, final state | step 2 | step 3 | step 2 | step 3 | step 2 |
| Mean and worst cost over the problem's draws | step 4 | step 4 | step 4 | step 4; disturbance draws per the plan's ask 6 | step 4 |
| Failure rate, its smooth stand-in `feasibility` | step 4 (the plan's ask 5) | step 4 | step 4 | step 4 | step 4 |
| Closed-loop poles, damping ratios, natural frequencies | §4.7 (S73) | — | §4.7 (S73) | — | — (linearized at `x̄`) |
| Gain and phase margins, Bode magnitude and phase at a frequency, the sensitivity peaks | §4.7 (S73) | — | §4.7 (S73) | — | — |

The consumer of every cell is the same: the program bridge builds a `MathematicalProgram`
whose objective is one metric and whose constraints `g ≥ 0` can be other metrics (maximize
the phase margin over the PID gains subject to a gain margin above 6 dB and the gains in a
box), and `Optimizer` solves it with a method the program's class admits (§4.8). The worked
examples the table implies, each one demo once it lands:

- gain tuning on the closed-loop cost (`pid_autotuning_jax`, today's hand loop);
- gain tuning on a stability margin (§4.7 plus the bridge);
- an open-loop input sequence by single shooting (`cartpole_rollout_gradients`, today's
  hand loop);
- robust tuning on the mean cost over a plant-parameter family;
- plant-parameter sensitivity of poles and margins (the Buckingham-π question in a linear
  setting).

## 2. The pipeline: the nouns are right, the verbs are duplicated

![The planned pipeline](differentiable-cost-pipeline.png)

*The statement (green) holds the system, the cost, the sets and the distributions with one
params pytree at its root. The traced path (purple) is one compiled object per problem and
controller: one integrator kernel that emits the internal signals, a native `Trajectory`
pytree, one scoring kernel, an `Evaluation` of scalars. The NumPy consumers (grey) sit below
the kernels and call them; none keeps a twin. Sources: `differentiable-cost-pipeline.svg`
(the PNG) and `differentiable-cost-pipeline.mmd` (the same figure in Mermaid, rendered by GitHub).*

The review counted the state of each noun. Every law a user writes (`f`, `h`, `g`, `margin`)
exists once and traces through the array-module idiom; the duplication is one layer up, in
the code that runs the laws: integrating, scoring, cutting at exit, drawing samples and
reconstructing signals each exist in two to five copies, about half NumPy-only. So the
target needs no new noun beyond the compiled evaluator and its `Evaluation`; it needs one
traced kernel per verb with the NumPy consumers calling it. Two nouns become surplus
(`MonteCarloEvaluator` absorbed, `StepRollout` folded into `Trajectory`), none else appears
or disappears.

## 3. Every workflow on the same objects

![Ten workflows as lanes](differentiable-cost-workflows.png)

*One lane per workflow, left to right: the source object, the intermediate representations,
the result, the final consumer. Purple is traced and array in / array out today, green
becomes traced when the evaluator plan lands, grey is NumPy by contract (reporting, solver
loops, random search). Source: `differentiable-cost-workflows.svg`.*

What the lanes showed, beyond the single pipeline:

- the middle column is a family of solver-ready forms (§4.4);
- `Trajectory` is the result of six of the ten lanes, so its native form matters more than
  one box suggests, and the scorer should take any of them (§4.5);
- the open-loop form already exists inside the closed-loop one (§4.6);
- the gain-tuning lane ends in a hand-written step while trajectory optimization ends in the
  `Optimizer`; after the bridge they are the same consumer (§4.8);
- two lanes stay honestly grey in the middle: model predictive control (no implicit gradient
  of the inner solve) and the sampling planner (random search). The target does not promise
  to trace either.

## 4. The ideas, one by one

Each entry: what it is, why, what it changes, what it risks, where it sits. "Decided" marks
the two that already are.

### 4.1 Read the parameters work and the kernels as one job

Ten cost methods, twelve set methods and six shape methods take `params` and none reads it.
Making them read it (cost-params.md, the sets in the later-nouns row) is the first half of
the differentiable problem; the kernels are the second half. Schedule them back to back and
let the kernels' tests be the "done when" of the parameters work. Changes the order of the
workboard's v0.3 section only.

### 4.2 One kernel per verb (`score_trajectory` on `xp`); the NumPy consumers call it

Five scoring rules with four quadratures (the cost's NumPy evaluator, the planning scorer
`score_trajectory`, the transcriptions' `trajectory_cost` without discount, the Monte Carlo
evaluator in JAX, the RL per-step reward) collapse onto one dual-backend (`xp`)
`score_trajectory(problem, traj, *, params=None, constraints="cut", discount=True, ...)`
layered on a dual-backend `CostFunction.evaluate_trajectory` / `total_cost` (`core/costs.py`),
called by both JAX and NumPy consumers on the native `Trajectory` pytree (§4.3). Using the
static-shape `alive = xp.cumprod(inside)` interval mask rather than a dynamic slice
`t[:last+1]` makes the NumPy and JAX evaluations share one exact body. Passing
`discount=False` in the open-loop transcriptions keeps existing discounted-problem trajopt
baselines bit-identical while exposing `discount=True` (or a custom $\rho$) as an option, and
`constraints="cut" | "ignore" | "soft"` selects between episodic exit cutting, smooth uncut
gradient evaluation, and smooth penalty folding (§5.3). The RL reward stays the documented
exception, derived from the same `g`. Two signal reconstructions become one (`rollout_p`
emits the signals). `Set.contains` is derived from `margin ≥ 0`. The quadrature is stated
once in DESIGN and cited by the transcriptions. Sits in the evaluator plan, steps 1–3.

### 4.3 A native `Trajectory` (decided 2026-10-18)

The object keeps the arrays it is given through the array module, no `float` cast, no copy
of a traced value, registered as a JAX pytree; the reporting verbs stay NumPy by contract.
Step 2b of the evaluator plan. The follow-on, not decided: `StepRollout` becomes a
constructor of the same noun with `t = k·dt` (a November consolidation candidate).

### 4.4 The verb family on the problem

Every lane first turns the problem into a solver-ready form: the transcription (trajectory
optimization), the grid (dynamic programming), the environment (reinforcement learning), the
linear model (analysis), and now the compiled loop. Each was designed on its own, with its own
constructor and its own copy of the scoring rule. The idea: a small set of verbs on the
problem, each returning one frozen traced form (`compile`, `transcribe`, a grid verb,
`environment`), named together in the brainstorm of the evaluator plan's step 0 rather than
naming the new member alone. Changes names and the entry points, not the math. Risk: a
rename sweep across demos and notebooks; keep the old constructors one release.

### 4.5 `Evaluation` as a verb of the problem

Six lanes end in a `Trajectory`: the reference simulator, the MPC loop, the RRT tree and the
trajectory optimizer as much as the compiled rollout. The scoring kernel should be reachable
as a verb on the problem that takes any trajectory and returns the same `Evaluation`, the
compiled evaluator calling it on its own traced rollout. One scorer for every lane, and
`PlanningSolution.evaluation` reproducible for every planner. Changes where the kernel is
exposed, not what it computes. Sits with 4.2.

### 4.6 The open-loop `compile()`

`rollout_p` takes an input sequence, so compiling the problem without a controller is the
open-loop evaluator. Two lanes then collapse onto it: single-shooting trajectory optimization
is the program bridge with the inputs as decision leaves, and the RL environment is the same
object exposed one step at a time (its reward is one step of the scoring rule). Collocation
stays its own transcription: it decides the states and adds defect constraints. If accepted,
`problem.compile()` with no controller is a valid case from the first prototype, not a
closed-loop-only design opened later. Caution: merging the RL environment touches the
collectors, which carry a bit-identical baseline; that merge comes after the kernels exist
and are tested.

### 4.7 Traceable linear analysis (workboard row S73)

`poles`, `frequency_response` and `margins` through the array module, so the sensitivity of
closed-loop poles, damping ratios, Bode values and margins to any plant or controller param is
`jax.grad` of the existing verbs on the JAX jacobian. The math lines stay the textbook ones:
`C (jωI − A)⁻¹ B + D` as one batched solve instead of the Python loop; eigenvalues through
the array module, whose derivative is the classical `vᴴ (∂A/∂p) u / (vᴴ u)` sensitivity;
Brent's method on the crossovers replaced by a fixed number of Newton steps from the grid
bracket, so the crossover frequency carries the implicit-function derivative. Zeros stay NumPy
(a generalized eigenproblem JAX lacks); root-locus branch matching stays NumPy (plotting); the
root locus generalizes to any parameter. Non-differentiable spots to document: repeated poles,
the switch of the minimum margin between crossovers; the nonsymmetric eigensolver is
CPU-only. Independent of the evaluator (needs only the jacobian); self-contained for a cloud
agent. The done-when is on the workboard row.

### 4.8 The optimizer as the one descent loop (workboard row S74)

Today `Optimizer` offers three constrained-NLP presets on a flat `z`, and every gradient-based
tuning writes its own loop: four compile demos, three notebooks, while the RL algorithms share
one pytree `Adam` in the RL package. Three parts: (1) the program knows its class from its
fields (`unconstrained`, `bounded`, `constrained`, plus a `stochastic` flag the bridge sets
when the objective resamples keys); `Optimizer` refuses a method the class does not admit and
`method=None` picks by class (quasi-Newton for a smooth unconstrained or bounded program,
SLSQP otherwise, Adam when stochastic). (2) One first-order backend, gradient descent and
Adam on the flat `z`, the box by projection, the global clip; the RL package imports that Adam.
Two SciPy presets (`scipy_lbfgsb`, `scipy_bfgs`) come free. (3) Pytree decisions stay outside
`Optimizer`: the bridge ravels the chosen leaves to `z` and unravels the result. Then the demos
and notebooks end in one optimizer call. For the maintainer: whether one tutorial keeps a
hand-written loop on purpose, as the GRO501 from-scratch notebooks do. The done-when is on the
workboard row.

### 4.9 Metrics as constraints in the bridge

The bridge of the evaluator plan's step 5 takes one metric as `J`; the target of §1 wants
other metrics as `g ≥ 0` (a margin floor while tuning on cost). Design the bridge's signature
for that from the start, `program(objective=, constraints=(), wrt=)`, or keep `J` only in the
prototype (the plan's ask 9).

### 4.10 Consolidation rows the review found

Not part of the differentiable path, but they remove the code it would otherwise sit beside:

- the JAX evaluator's method grid (five families times up to six suffixes, 36 integrator
  methods, 1763 lines): keep the `_p` bodies and generate the bound and jitted variants;
- `StepRollout` folded into `Trajectory` once it is a pytree (4.3);
- the planned core `Field` absorbs the six field-like classes in `planning/spatial`, and a
  grid field with a JAX interpolation makes the lookup-table policy traceable;
- a test pinning that the simulator's fixed-step RK4 and the compiled rollout agree to
  round-off on one diagram;
- the compiled evaluator refuses the MPC controller (no implicit gradient) with a message
  saying so, as the Monte Carlo evaluator refuses stateful controllers today.

## 5. Architectural guardrails and recommendations (2026-10-18)

Pushing a unified differentiable mathematical-program pattern across classical frequency
tuning, closed-loop parameter optimization, trajectory optimization, and stochastic policy
learning introduces four numerical and architectural sharp edges. These guardrails keep each
regime mathematically well-posed and maintainable:

### 5.1 Smooth frequency-domain surrogates alongside crossover margins (S73)

- **The trap:** Gain and phase margins are defined at crossover frequencies ($\omega_{gc},
  \omega_{pc}$). During gradient-based gain optimization, crossovers can appear, disappear,
  or jump discontinuously between local peaks of $|L(j\omega)|$. In addition, nonsymmetric
  eigenvalue decomposition (`eig`) for closed-loop poles is CPU-only in JAX and has
  undefined gradients at repeated roots (exactly where root-locus branches break away).
- **Recommendation:** Keep `margins()` traceable via fixed Newton steps from the grid
  bracket (implicit-function theorem), **and** expose the **maximum sensitivity peaks**
  $M_s = \max_\omega |S(j\omega)|$ and $M_t = \max_\omega |T(j\omega)|$ (along with their
  smooth `logsumexp` approximations over the frequency grid $\omega$) as first-class
  traceable metrics on `sensitivity` and `complementary_sensitivity`. Bounding $M_s \le 1.5$
  is a smooth, crossover-free surrogate that simultaneously guarantees both gain margin
  ($\ge \frac{M_s}{M_s - 1}$) and phase margin ($\ge 2\arcsin(\frac{1}{2M_s})$) without
  crossover jumps during NLP-based PID loop shaping.

### 5.2 Open-loop `problem.compile()` vs. direct collocation (§4.6)

- **The trap:** Single shooting through an unstable nonlinear ODE over long horizons has
  exponentially exploding or vanishing gradients ($\prod_k \frac{\partial f}{\partial x_k}$)
  and a non-convex objective landscape where SLSQP or Adam can stall, whereas direct
  collocation succeeds in tens of iterations.
- **Recommendation:** Support `problem.compile(controller=None)` for open-loop evaluation
  and short-horizon single shooting so the algebra closes, but **do not position
  `problem.compile().program()` as a replacement for `TrajectoryOptimizationPlanner`**.
  Keep transcriptions (`direct_collocation`, `multiple_shooting`) as the primary trajectory
  NLP path because defect constraints $h(z) = 0$ break the temporal gradient chain and
  expose sparsity.

### 5.3 Multi-mode hard-constraint treatment (`X`, `U`, `Xf`) across all pipelines

- **The trap:** A single hard-constraint rule cannot simultaneously serve episodic reporting/RL,
  unconstrained gradient descent, and constrained SQP/interior-point optimization:
  1. **Episodic cut (`"cut"`):** In `score_trajectory` and `MonteCarloEvaluator.evaluate_jax`,
     cutting a rollout at the first exit from $X$ (`alive` mask) and charging `infeasible_cost`
     is a step function ($\mathbb{I}(x_k \in X)$) with **zero gradient almost everywhere**. If a
     controller clips $X$ at $t = 2\text{ s}$, `jax.grad(evaluator.cost)(params)` yields zero
     gradient for all behavior after $t = 2\text{ s}$ and no boundary derivative telling the
     optimizer how to pull the trajectory back inside $X$.
  2. **The `SingletonSet(x_goal)` trap on `Xf`:** `PlanningProblem` automatically builds
     `Xf = SingletonSet(x_goal)` whenever `x_goal` is passed (`problems.py`). In open-loop
     trajectory optimization and RRT, $x(t_f) = x_{\text{goal}}$ is a terminal boundary
     condition ($h(z) = 0$). In closed-loop regulation (`PID`, `LQR`, `RL`, `MonteCarlo`), the
     loop approaches $x_{\text{goal}}$ asymptotically ($e^{-t/\tau}$), so `Xf.contains(x_N)` is
     always `False` in floating point and treating it as a hard equality $h(z) = 0$ would make
     gain tuning with fewer parameters than states infeasible.
  3. **Unsaturated input bounds `U` in closed-loop tuning:** In open-loop planners (`trajopt`,
     `MPC`, `DP`, `RRT`, `MPPI`), $u_k \in U$ holds by construction. In closed-loop tuning
     (`problem.compile(controller)`), a linear `PID` or `StateFeedback` without a `Saturation`
     block can emit $u_k \notin U$ unless `U.margin` is either penalized or constrained.

- **Recommendation — make `score_trajectory` the single dual-backend (`xp`) scoring kernel with explicit `constraints=` and `discount=` options:**
  Once `Trajectory` is a native `xp` JAX pytree (§4.3 / Step 2b), `score_trajectory(problem, traj, *, params=None, constraints="cut", discount=True, ...)` replaces all three duplicate scoring implementations (`score_trajectory`, `evaluate_jax`'s inline scan, and `transcription.py::trajectory_cost`) without needing a separate `score_rollout` function:
  - **`discount: bool | float = True`:** `True` applies $\exp(-\rho t)$ from `problem.cost.discount_rate` (closed-loop evaluation, RL, DP, `evaluator.cost`); `False` (or `0.0`) integrates undiscounted $g$ (used by open-loop transcriptions `direct_collocation` and `multiple_shooting` so existing trajopt baselines stay bit-identical unless opted in); a float overrides $\rho$.
  - **Four explicit constraint modes (`constraints=` on `score_trajectory`, `evaluator.cost`, and `evaluator.program`) backed by one `Set.margin` contract:**
  1. **`constraints="ignore"` (Unconstrained rollout for gradient evaluation + independent
     success/failure reporting):**
     - `J` integrates the uncut rollout over $[0, t_f]$, ignoring `X`, `U`, and `Xf` inside the
       cost integral (guarding only `~isfinite(x)` so NaNs never poison `jax.grad`).
     - `evaluator.success_rate(params, keys)` (hard boolean fraction staying in $X$ and region
       $X_f$) and `evaluator.feasibility(params, keys)` (`softmin` of margins) are evaluated
       **independently** as diagnostics.
     - *Best for:* direct `jax.grad(evaluator.cost)` loops and unconstrained first-order tuning
       (`Adam`, `gd`, `scipy_lbfgsb`) when quadratic costs $Q, R$ already regularize the state
       and input, or when `X = sys.state.box` was only set as a nominal plotting/grid domain and
       early transient excursions must not zero out late-horizon gradients.
  2. **`constraints="soft"` (Smooth penalty / barrier folding into $J$, plus `Set.as_cost`):**
     - Folds any active constraint margin $m = \text{Set.margin}(z, t, \text{params})$ into
       $J_{\text{uncut}}$ using the shaping functions in `planning/spatial/shaping.py`:
       $$J_{\text{soft}}(\theta) = J_{\text{uncut}}(\theta) + \int_0^{t_f} e^{-\rho t} w_X \,\phi\big(m_X(x(t), t)\big)\,dt + \int_0^{t_f} e^{-\rho t} w_U \,\phi\big(m_U(x(t), u(t), t)\big)\,dt + e^{-\rho t_f} w_{X_f}\,\phi\big(m_{X_f}(x(t_f), t_f)\big)$$
       with default exterior hinge $\phi(m) = \sum_i \max(0, -m_i)^2$ (`quadratic_hinge(0.0)`,
       zero inside the safe set, $C^1$ at the boundary, quadratic in penetration depth to match
       `QuadraticCost` scaling) or interior `inverse_barrier(epsilon)`.
     - **Close the noun duality with `Set.as_cost(weight=100.0, shaping=quadratic_hinge(0.0)) -> CostFunction`:**
       Just as `Field` offers `.as_constraint()` and `.as_cost()`, every `Set` (`BoxSet`,
       `BallSet`, `FieldSet`, `IntersectionSet`) already computes native-array `margin`;
       giving `Set` an `.as_cost()` method lets users explicitly soften individual sets
       (`cost = tracking_cost + problem.X.as_cost(weight=50.0)`) and powers
       `constraints="soft"` with zero duplicate code.
     - *Best for:* first-order and box-bounded optimizers (`Adam`, `gd`, `scipy_lbfgsb`) that
       cannot take nonlinear inequalities $g(z) \ge 0$; sampling-based planners
       (`PathIntegralPlanner` / `MPPI`) in narrow corridors where near-miss rollouts still carry
       informative weights; and stochastic expectation tuning (`mean_cost(params, keys)`) where
       a single $3\sigma$ disturbance grazing $X$ would make hard all-draw constraints
       infeasible.
  3. **`constraints="hard"` (Explicit NLP inequality / equality constraints $g(z) \ge 0$,
     $h(z) = 0$):**
     - `J(z)` is the **uncut** rollout cost (`"ignore"`), while active sets lower to the
       `MathematicalProgram` constraint vectors:
       - In **open-loop transcriptions (`direct_collocation`, `multiple_shooting`) and `MPC`:**
         `BoxSet` / `BoxInputSet` $\to$ variable box bounds `lower <= z <= upper`; general
         `X`, `U` $\to$ sparse per-knot inequalities $g_k(z) = \text{margin}(x_k, u_k, t_k) \ge 0$;
         `SingletonSet` on $X_0, X_f$ $\to$ equality $h(z) = \text{residual}(x_N) = 0$; region
         $X_f$ $\to$ $g_N(z) = X_f.\text{margin}(x_N, t_N) \ge 0$.
       - In **`evaluator.program(wrt=..., constraints="hard")` (closed-loop parameter tuning):**
         Because $z \in \mathbb{R}^{n_\theta}$ is low-dimensional while the horizon has $N+1$
         steps, aggregate path margins over time via the smooth inner approximation
         $g_X(z) = \operatorname{softmin}_k X.\text{margin}(x_k(z), t_k) \ge 0$ (one constraint
         per margin component, guaranteeing $\min_k m_k \ge 0$ while staying $C^\infty$), and
         similarly $g_U(z) = \operatorname{softmin}_k U.\text{margin}(u_k(z)) \ge 0$ when the
         loop has no `Saturation` block. For $X_f$, enforce $g_{X_f}(z) = X_f.\text{margin}(x_N(z)) \ge 0$
         for region sets, and enforce $h_{X_f}(z) = X_f.\text{residual}(x_N(z)) = 0$ only in
         open-loop shooting (`controller=None`). Over stochastic draws (`keys`), aggregate across
         draws via worst-case `softmin` or tail `CVaR` of the per-draw margins.
     - *Best for:* constrained SQP / interior-point solvers (`scipy_slsqp`,
       `scipy_trust_constr`, `ipopt`) when strict safety or performance bounds (including S73
       phase-margin / $M_s$ floors) must hold without tuning penalty weights.
  4. **`constraints="cut"` (Episodic cut at first exit + `infeasible_cost`):**
     - Integrates $g$ up to the first exit $k_{\text{exit}}$ from $X$ (`alive` mask), skips
       terminal $h(x_N)$ on early exit, and adds `infeasible_penalty(x_exit, t_exit)` (from
       `problem.infeasible_cost` or the Halton-sampled `feasible_cost_bound`).
     - *Best for:* `MonteCarloEvaluator` / `Evaluation` (`score_trajectory`) benchmarking,
       `RolloutEnvironment` in RL (where leaving $X$ sets `terminated=True, failed=True` and
       subtracts `infeasible_penalty`, kept distinct from `training_zone` where leaving sets
       `truncated=True` so PPO/SAC bootstrap $V(x_{\text{next}})$ without a penalty), and
       `DynamicProgrammingPlanner` (`out_of_bound_cost`).

| Pipeline | Mode for `X` (State) | Mode for `U` (Input) | Mode for `Xf` (Terminal) |
| --- | --- | --- | --- |
| `MonteCarloEvaluator` / `score_trajectory` | `"cut"` (`infeasible_penalty`, `failed` flag) | Enforced by loop; optional diagnostic | Region `Xf` checked; `SingletonSet(x_goal)` is reference only |
| `RolloutEnvironment` (PPO, SAC, REINFORCE, Tabular) | `"cut"` (`terminated=True` + penalty on $X$; `truncated=True` on `training_zone`) | Action space `BoxInputSet` bounds policy head | Region `Xf` terminal check / bonus |
| `DynamicProgrammingPlanner` (VI / PI) | `"cut"` (`out_of_bound_cost` on leaving $X$ / grid) | Discrete action grid inside `U` | Terminal cost $h$ or absorbing goal set |
| `RRTPlanner` | Hard boolean `X.contains(x, t)` | Sampled inside `U` | `Xf.contains(x)` (with goal tolerance) |
| `PathIntegralPlanner` (`MPPI`) | `"soft"` (`X.as_cost()` hinge) or `"cut"` (`infeasible_penalty`) | Clipped to `U.bounding_box()` | Terminal $h(x_N)$ or `Xf.as_cost()` |
| `TrajectoryOptimizationPlanner` & `MPC` | `"hard"` (`BoxSet` bounds + per-knot `X.margin >= 0`) or `"soft"` (`X.as_cost()`) | `"hard"` (`BoxInputSet` bounds + `U.margin >= 0`) | `"hard"` (`SingletonSet` $h(z)=0$, region $g(z)\ge 0$) |
| `problem.compile` + `jax.grad` / `Adam` / `L-BFGS-B` | `"ignore"` (uncut $J$ + independent `success_rate` / `feasibility`) or `"soft"` (`X.as_cost()`) | Loop `Saturation` or `"soft"` (`U.as_cost()`) | Terminal $h(x_N)$ or `"soft"` on region `Xf` |
| `evaluator.program` + `scipy_slsqp` / `ipopt` | `"hard"` (uncut $J$ + `softmin_k X.margin >= 0`) or `"soft"` | `"hard"` (`softmin_k U.margin >= 0` if unsaturated) | Region `Xf` $\to$ `Xf.margin >= 0`; `SingletonSet` $\to$ $h(z)=0$ only when `controller=None` |

### 5.4 Differentiable rollout (BPTT) vs. policy gradients (RL) as complementary tools

- **The trap:** Direct backpropagation through time (`jax.grad(evaluator.mean_cost)`)
  differentiates *through* the dynamics $\frac{\partial f}{\partial x}$ and reparameterized
  draws, whereas policy-gradient RL (`REINFORCE`, `PPO`, `SAC`) differentiates the log-policy
  score $\nabla_\theta \log \pi_\theta(a|s)$ without differentiating through $f(x, u)$. Long
  horizons, saturations, or stiff dynamics cause BPTT gradients to explode or exhibit high
  variance, whereas short smooth horizons converge orders of magnitude faster with BPTT than
  with PPO.
- **Recommendation:** Position `problem.compile(policy)` (first-order through dynamics) and
  `ReinforcementLearningPlanner` (zero-order through dynamics, surrogate critic) side by
  side on the exact same `StochasticPlanningProblem` and `NeuralPolicyController`, making
  the sample-complexity vs. gradient-variance trade-off a signature pedagogical comparison.

### 5.5 Keep `MathematicalProgram` flat; ravel pytrees at the bridge, not in `Optimizer` (S74)

- **Recommendation:** Keep `MathematicalProgram` strictly as $J(z), h(z)=0, g(z)\ge 0$ on a
  flat vector $z \in \mathbb{R}^{n_z}$ (CONSTITUTION §1.4: Minilink is not an optimization
  modeling language). The bridge `evaluator.program(wrt=...)` owns `jax.flatten_util.ravel_pytree`
  and exposes `program.unravel(result.z)` (or `evaluator.update_params(result.z)`), so IPOPT,
  SciPy SLSQP, L-BFGS-B, and first-order Adam all solve the exact same `MathematicalProgram`
  interchangeably. In tutorials, keep **one** explicit 6-line `jax.value_and_grad` loop in
  `07_compile.ipynb` / `showcase_jax.ipynb` so students see there is no magic, followed
  immediately by the `Optimizer(evaluator.program(...)).solve()` one-liner used in all
  downstream demos.

## 6. How this fits the existing plans

| Document | Owns |
| --- | --- |
| [differentiable-cost.md](differentiable-cost.md) | the compiled evaluator: its design (§3), the verified facts, steps 0–6 with their done-whens, the asks |
| [differentiable-cost-options.md](differentiable-cost-options.md) | the architectures and naming systems weighed for the evaluator; feeds the brainstorm of step 0 |
| [cost-params.md](cost-params.md) | every cost reads `params`, through to the program's objective (4.1's first half) |
| [fields.md](fields.md) | the `Field` object; gains the "absorbs `planning/spatial`" done-when if 4.10 is accepted |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | the shared optimizer backend factory for parametric MPC, which 4.8's backend joins |
| [TODO.md](TODO.md) §4 | the rows: V1 (the evaluator), A2, A5, S73 (4.7), S74 (4.8), the November consolidation rows |
| [the core-objects review](../reviews/2026-10-18-core-objects-review.md) | the dated record: the per-noun table and the counts |

## 7. An order, if the ideas are accepted (not scheduled)

**Current baseline order (split `V1` across `K2–K4` and `A2`):**

1. The evaluator plan's steps 0–3 (names, kernel, trajectory, compile) with 4.5 and 4.6
   folded into step 0 so the names and the no-controller case are settled before code.
2. The cost parameters (A2), then the evaluator's steps 4–6, with the bridge signature of 4.9
   decided at step 5.
3. 4.7 (traceable analysis) any time, beside the above; 4.8 (the optimizer) at or after the
   bridge, where its first customers appear.
4. The consolidation rows of 4.10 in the November pass, the RL-environment merge of 4.6 last.

### 7.1 Alternative bottom-up order (to review later — avoids writing `score_trajectory` or the compile demos twice)

Because the unified `score_trajectory(problem, traj, *, params, constraints, discount)`
operates directly on a `Trajectory` object, layers on top of `CostFunction.total_cost`, and
uses `Set.as_cost(weight=, shaping=)` for `constraints="soft"`, running `V1` Step 1 *before*
`Trajectory` is a JAX pytree (`Step 2b`), *before* `CostFunction` is `xp`-native and reads
`params` (`A2`), and *before* `shaping.py` moves to `core/` would require temporary
workarounds and touching the compile demos twice (once in `K4` and once in `V1-6` / `S74`).
This alternative order builds strictly bottom-up:

| Phase | Step | What Lands | Why Here |
| --- | --- | --- | --- |
| **Phase 1: Inventory & Surface Cleanup** | **0. Naming & `K1–K3`** | Step 0 brainstorm (`problem.compile`, `score_trajectory` options, verb family); `K1` inventory table; `K2` dead-code removal; `K3` prototypes to `experimental/`. | Settles all public names once and shrinks the codebase before core edits begin (`K4` demo sweep waits until after Phase 4). |
| **Phase 2: Core Nouns on `xp` & `params`** | **1. Native `Trajectory` (`V1-2b`)** | `Trajectory` (and `StepRollout`) as a native `xp` JAX pytree (`t`, `x`, `u`, `signals` leaves; `Trajectory.from_rollout`); reporting verbs stay NumPy. | Prerequisite for `CostFunction.total_cost(traj)` and `score_trajectory(problem, traj)` to trace. |
| | **2. `Set` & `shaping` in `core/` (from `A5`/`A3`)** | Move `shaping.py` to `minilink/core/shaping.py`; derive `Set.contains` from `Set.margin >= 0`; add `Set.as_cost(weight=, shaping=)`. | Prerequisite for `score_trajectory(..., constraints="soft")` without `core` $\to$ `planning` imports. |
| | **3. Cost `params` & `xp` cost integral (`A2`)** | Library costs read `params`; `SumCost` / `ScaledCost` nest `params` like `DiagramSystem`; upgrade `CostFunction.evaluate_trajectory` & `total_cost` to `xp` with `discount=`. | Makes `CostFunction` 100% `xp`-native and param-aware *before* `score_trajectory` is written. |
| **Phase 3: Unified Scoring & Rollout Kernels** | **4. Unified `score_trajectory` (`V1-1`)** | Dual-backend `score_trajectory(problem, traj, *, params, constraints="cut"\|"ignore"\|"soft", discount=True\|False\|float)` using static `alive = xp.cumprod(inside)` mask; rewire `evaluate_jax` and `transcription.py::trajectory_cost` to call it. | Written **once** on top of the Phase 2 nouns; immediately deletes 3 duplicate scoring rules. |
| | **5. Traced signal rollout (`V1-2`)** | `rollout_p` on `JaxDiagramEvaluator` emitting `(t, xs, signals)` and traced `trajectory_of` slicing plant `(x, u)` (`AC-7` on compiled loops). | Gives `problem.compile` the plant's internal command `u` for `score_trajectory`. |
| **Phase 4: Compiled Problem, Draws & Program Bridge** | **6. `problem.compile` deterministic (`V1-3`)** | `PlanningProblemEvaluator` with `trajectory`, `final_state`, `cost(..., constraints=)`, `feasibility`; open-loop `controller=None` support. | Delivers deterministic closed-loop and open-loop BPTT gradients (`dJ/dK`, `dJ/dQ`, `dJ/dm`). |
| | **7. Stochastic draws (`V1-4` + `RN-4`/`RN-5`)** | Traced `realize(key)` seeds (`RN-4`), `mean_cost(params, keys)`, `success_rate`, `feasibility`; `MonteCarloEvaluator` becomes a thin wrapper (`RN-5`). | Unifies stochastic evaluation and pathwise stochastic gradients in one step (no separate "Monte Carlo part after RN-5"). |
| | **8. Program bridge + First-order `Optimizer` (`V1-5/6` + `S74`)** | `evaluator.program(wrt=, constraints="hard"\|"soft"\|"ignore")` with `ravel_pytree`; `optimizers/first_order.py` (`adam`, `gd`, `scipy_lbfgsb`); rewrite the 4 compile demos (`V1-6`), then run `K4` demo consolidation. | Solves constrained (`scipy_slsqp`, `ipopt`), soft/box-bounded (`scipy_lbfgsb`, `adam`), and stochastic (`adam`) tuning through one `Optimizer(program).solve()` contract. |
| **Phase 5: Parallel / Subsequent v0.3 Work** | **9. Traceable Linear Analysis (`S73`)** | `poles`, `frequency_response`, `margins`, and $M_s, M_t$ on `xp` (independent of Phase 2–4; can run in parallel any time before `G1`). | Enables frequency-domain and pole-placement NLP constraints in `.program()` for GMC714 Robust Control. |
| | **10. Spatial Nouns (`A1`, `A3`) & Wave C (`G1`, `C1–C6`, `S72`)** | `core/fields.py` (`A1`), `core/geometry/` (`A3`), `S72` (`MPPI`, using `score_trajectory` with `constraints="soft" \| "cut"`), and GMC714 Wave C rows. | `A1`, `A3`, and `S72` plug directly into the landed `score_trajectory` and `params` spine. |

## 8. What needs the maintainer

- The names (the brainstorm of step 0), now including the verb family of 4.4.
- Which of 4.4–4.10 and the guardrails of §5 are accepted, folded into a plan, or dropped;
  the workboard rows S73 and S74 wait on that.
- Whether the one sentence of §1 goes into the constitution's vision text.
- Whether one tutorial keeps a hand-written gradient loop on purpose (4.8, 5.5).
