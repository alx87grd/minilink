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
    R15–R47, R71–R77) and [the 2026-10-25 rulings](../reviews/2026-10-25-open-questions-rulings.md)
    (R91–R97);
  - the design of the landed steps is in git history.
- **This doc holds what remains:**
  - the `StepRollout` fold;
  - step 7: `disturbances=` port signals and cross-backend draws in `problem.realize`;
  - step 7b: the sampled face;
  - step 8: sets as costs, a source policy's samples as leaves;
  - step 9: the material;
  - §9: a draft idea to unify the evaluation from the rollout on (2026-10-26, not ruled);
  - traceable linear analysis (S73, §4.6).
- **Two leftovers of landed steps** wait for the maintainer in [TODO.md](TODO.md) §2: the LQR
  warning on a declared `U` (OD-33 of the run's log), and the three demos that teach a hand
  gradient loop (OD-32).

**Where it sits** (the core, [core-architecture.md](core-architecture.md); rows in
[TODO.md](TODO.md)):
- the `StepRollout` fold: §1.2;
- step 8: §1.1;
- step 7, with RN-4 of [randomness.md](randomness.md): §1.3;
- step 7b, after S31's kernel: §1.5;
- §9's one score: §1.6, a pillar;
- step 9, with the merge: §1.10;
- S73: any time (R76), a v0.4 row (§4).

The open steps are **[ask — core]**.

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
| The criterion over the problem's draws (mean, CVaR, worst) | landed (loop noise blocks included, R92; `disturbances=` ports step 7) | step 8 | landed | cost landed; sets step 8 | landed |
| The feasibility and reachability margins (`Evaluation.feasibility_margin`, `.reachability_margin`) | landed | step 8 | landed | step 8 | landed |
| Closed-loop poles, damping ratios, natural frequencies | S73 | — | S73 | — | — (linearized at `x̄`) |
| Gain and phase margins, Bode values, the sensitivity peaks `M_s`, `M_t` | S73 | — | S73 | — | — |

Every cell has the same consumer: the program export (`pe.program(wrt, bounds)`, DESIGN §6),
which today's `Optimizer` solves. The worked examples still to come, each one demo once it
lands: an open-loop input sequence by single shooting (today's `cartpole_rollout_gradients`
hand loop and the showcase cell); robust tuning on the mean over a plant-parameter family;
parameter identification from logged data (today's `params_gradient` and the showcase cell,
C4); the plant-parameter sensitivity of poles and margins (S73).

## 3. The objects still to change

| Object | Today | After (step) |
| --- | --- | --- |
| `StepRollout` (`core/discrete/step_rollout.py`) | the step twin of `Trajectory`: the hybrid simulator's computer record and MPC's dual-rate view | a constructor of `Trajectory` with `t = k·dt` (step 1, last commit, with S31: OD-36) |
| `Set` | `margin` traces; `contains` is that test on `xp` (landed); the library sets hold no parameters | reads `params` with A5, `Set.as_cost(weight=, shaping=)` (step 8) |
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
  one `NeuralPolicyController` (step 9).
- **Crossover margins jump; sensitivity peaks do not.** A gain or phase margin is defined at a
  crossover that can appear, vanish or switch during a descent, and `eig` has no derivative at
  a repeated pole; `M_s`, `M_t` are the smooth robustness bounds for an NLP, the margins the
  numbers a course reports (§4.6).

## 8. The steps still open, bottom-up

One commit per step or sub-step; each records a seeded baseline of every branch it touches
before any edit, runs it twice and `cmp`s, and ends byte-identical unless the step names its
one intended change, which gets its own test and a ledger line. Every math line reads next to
`planning/dynamic_programming/dp.py` before it lands. **The gates are the TODO rows** (TODO §1:
the fold §1.2, step 7 §1.3, step 7b §1.5, step 8 §1.1, step 9 §1.10); this list is the
content. Steps 0–6 and 7a landed on 2026-10-24.

- **Step 1, its last commit: the `StepRollout` fold** (with S31, in the loop lane; OD-36). `StepRollout`
  becomes a constructor of `Trajectory` with `t = k·dt`, byte-identical; it waits for S31
  because it is the hybrid simulator's computer record and MPC's dual-rate view, and every
  hybrid piece went with S31 (R86 then said v0.4; the v0.3 core since 2026-10-26).
- **Step 7, `disturbances=` port signals and `problem.realize` seeds** (with RN-4): per-trial
  realization of random blocks inside the evaluated loop landed in v0.3 (`trial_seeds`, R92);
  step 7 moves those seeds into `problem.realize(key, n)` (alongside cross-backend start and
  parameter draws) and wires `disturbances=` signal blocks onto their ports in `evaluate`
  (today `evaluate` holds the `disturbances` ports at their nominal values and warns,
  `warn_if_disturbed` in `planning/evaluator.py`).
- **Step 7b, the sampled face** (after S31's kernel): `compile` dispatches a
  sampled controller (a digital PID, a held neural policy, MPPI, the RL policies, and later
  the fixed-iteration MPC of [native-optimizers.md](native-optimizers.md) §4.6) to the
  kernel's JAX face; the refusal goes (today `require_continuous` in `planning/evaluator.py`
  refuses a `StepSystem` controller); a continuous law's numbers unchanged.
- **Step 8, sets as costs, the lowerings, the metrics as constraints**. `shaping.py`
  (today `planning/spatial/shaping.py`) to `core/`, `Set.as_cost(weight=, shaping=)`; sets
  reading `params`; `TrajectorySource` with its table in `params` (its `h` already traces), so
  the start `"x_start"` and `"controller.values"` are decision leaves with `Xf` lowered; the
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

## 9. Draft idea: one discrete evaluator from the rollout on (the maintainer, 2026-10-26)

**Status: a draft idea, not ruled; it needs refinement (§9.4).** The ask is in QUESTIONS.md
(it became a core pillar on 2026-10-26).

> Once the cost function is set and the time discretization is decided (`dt`, the substeps,
> the Runge–Kutta scheme), a continuous system's rollout is the same thing as a system that
> is already discrete and stepping. So the evaluation pipeline should be unified from that
> point on. The objects of the planning problem are written in continuous time, so it needs a
> method that discretizes the problem: the time argument of the cost and of the sets becomes
> an index, and everything is defined the same way with time as an index.

### 9.1 What already holds in the code

- **The run is already a step map.** The evaluator's rollout is the step model
  (`core/compile/evaluators/stepping.py`, DESIGN §5 "The step model"): the same one-period
  body as `discretize(plant, dt, solver=, substeps=)`, the input held over each period.
- **The score is already discrete.** `score_run` (`planning/evaluation.py`) reads the samples
  `(t_k, x_k, u_k)`: the margins are minima over the samples, the cost a trapezoid sum over
  them (`CostFunction.integrate`, `core/costs.py`).
- **R15** makes the fixed-step run the one engine and judge; `solver="scipy"` swaps in the
  adaptive integrator and nothing else. **R105** dispatches a sampled controller to the
  sampled kernel's JAX face with "one compile, one evaluate, the same `J`". **The `StepRollout`
  fold** (step 1's last commit, §8) makes a step rollout a `Trajectory` with `t = k·dt`.
  [sampled-path.md](sampled-path.md) §8.4 (a tentative draft vision) makes `computer @ plant`
  a step diagram on a base clock.
- **What is not unified: the per-step cost.** The same continuous cost becomes a per-step cost
  three ways: value iteration takes `g(x_k, u_k, t_k) dt` (a left Riemann sum,
  `dynamic_programming/dp.py`), the RL environment the reward `−g(x_k, u_k, t_k) dt`
  (`learning/reinforcement_learning/environment.py`), the evaluator the trapezoid over the
  samples. One policy has three slightly different `J`s; R80 checks closeness, not equality.

### 9.2 The proposal, in two stages

1. **Discretize (once).** The continuous loop with `(dt, solver, substeps)` becomes a step map
   `x_{k+1} = Φ(x_k, u_k, k; p)` (what `discretize` already builds). The continuous problem with
   the same `dt` becomes a **discrete problem**: the per-step cost
   `c_k = ∫_{t_k}^{t_{k+1}} e^{−ρt} g dt` with `u_k` held, and the terminal `h(x_N)`; the sets
   at the index, `X.margin(x_k, t_k)`, `U.margin(u_k, x_k, t_k)`, `Xf` at `N = tf / dt`; the
   discount `γ = e^{−ρ dt}`; the start (or its distribution) unchanged.
2. **Evaluate (one kernel).** Every loop that steps (a continuous loop discretized, a plant
   already a `StepSystem`, a sampled computer on its plant) is scored by one discrete
   evaluator: a scan over `k` accumulating `c_k`, the least margin and the terminal term. Value
   iteration, the RL environment, the evaluator and the sampled loop read the same discrete
   problem, so a policy has one `J`.

The continuous problem stays the primary object (CONSTITUTION §5, "the continuous contract is
primary"): the discrete problem is a lowering a tool asks for, like `discretize` for a plant.
The tools of continuous time keep reading the continuous problem: direct collocation (its
defects are written on `f`), the continuous LQR, the linear analysis, the adaptive simulation
(`solver="scipy"`, R15's cross-check).

### 9.3 The agent's read

The direction is sound, and the code is closer to it than the docs say: the run and the score
are already discrete, so stage 2 is mostly a renaming of what the evaluator does, extended to
loops that are discrete from the start. What it buys: one `J` for every planner (the three
quadratures above become one), the sampled face of step 7b for free (a sampled loop is one
more step loop), and a discrete problem that native discrete models (a grid world) can state
directly.

One refinement makes it exact rather than approximate: **integrate the cost with the state.**
Append the running cost to the state, `ċ = e^{−ρt} g(x, u, t)`, and let the same Runge–Kutta
step with the same substeps integrate it; the step then returns `c_k` with the accuracy of the
state itself, no separate quadrature, and the substeps count for the cost as they do for `x`
(today the trapezoid sees only the samples).

### 9.4 What needs refinement before it is a plan

1. **The per-step cost's rule.** The augmented-state integral above, or a quadrature on the
   samples (left Riemann, as DP and RL; the trapezoid, as the evaluator, which is a transition
   cost `c(x_k, u_k, x_{k+1})`). Value iteration and RL need it per transition, which all three
   forms give. Whichever is chosen, the numbers of the planners that change move once (R80's
   closeness check).
2. **The margins between samples.** Today a set is checked at the samples only; a violation
   inside a period is invisible. The step could return the least margin over its substeps.
3. **The object and its name.** `problem.discretize(dt, solver=, substeps=)` returning a
   discrete problem (`StepPlanningProblem`?): a core object, maintainer-owned. What it holds
   (the step plant, the per-step cost as a function of `(x_k, u_k, k)`, the sets at the index,
   `N`, `γ`) and whether a native discrete problem is the same class.
4. **Time-varying objects.** A cost or a set that reads `t` binds `t_k = t₀ + k dt`; a
   time-varying set read between samples (point 2) needs `t` inside the step.
5. **The discount.** `e^{−ρ t_k}` and `γ^k` agree with `γ = e^{−ρ dt}`; the RL planner's own
   `DEFAULT_GAMMA` then follows from the problem's rate when one is set.
6. **The order of work.** After the `StepRollout` fold and S31's kernel, and
   together with step 7b, since the sampled face is one of the loops this kernel scores.
