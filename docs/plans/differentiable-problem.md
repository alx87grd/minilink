# The differentiable planning problem: one path from any parameter to any metric

**Status:** steps 0–6 and the Monte Carlo fold (step 7a) landed on 2026-10-24: the native
`Trajectory`, one scoring formula on the problem's fields, the problem evaluator
(`problem.compile`, `evaluate`, `compare`) and its program export, the cost parameters, and the
draws of the start and of the plant parameters. The contract is DESIGN §4–§6 (§6 "One scoring
formula", "Stochastic problem", "The problem evaluator"); the decisions the run took without the
maintainer are [the run's log](../reviews/2026-10-24-v03-run-decisions.md) §2, and the rulings
are [the rulings record](../reviews/2026-10-22-rulings-session-1.md) (R9–R11, R15–R47,
R71–R77). The design of the landed steps is in git history. This doc now holds what remains,
all v0.4: the `StepRollout` fold, step 7 (a noise block's per-trial seeds), step 7b (the sampled
face), step 8 (sets as costs, a source policy's samples as leaves) and step 9 (the material),
and traceable linear analysis (S73, §4.6), any time. Two leftovers of landed steps wait for the
maintainer in [TODO.md](TODO.md) §4: the LQR warning on a declared `U` (OD-33 of the run's log)
and the three demos that teach a hand gradient loop (OD-32).  
**Rung:** v0.4, the rows of [TODO.md](TODO.md) §5: the `StepRollout` fold with the sampled
loop (S31); step 7 with RN-4 of [randomness.md](randomness.md); step 7b after S31's kernel;
steps 8 and 9; S73 any time (R76).  
**Lane:** core; the open steps are **[ask — core]**.

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
| The criterion over the problem's draws (mean, CVaR, worst) | landed; a noise block's draws step 7 | step 8 | landed | cost landed; sets step 8 | landed |
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
`planning/dynamic_programming/dp.py` before it lands. **The gates are the TODO rows** (§5 for
these steps; the `StepRollout` fold is the row "V1.1, the rest" of §4); this list is the
content. Steps 0–6 and 7a landed on 2026-10-24.

- **Step 1, its last commit: the `StepRollout` fold** (v0.4, with S31; OD-36). `StepRollout`
  becomes a constructor of `Trajectory` with `t = k·dt`, byte-identical; it waits for S31
  because it is the hybrid simulator's computer record and MPC's dual-rate view, and every
  hybrid piece moved to v0.4 (R86).
- **Step 7, a disturbance block's draws** (v0.4, RN-4): one seed per noise block in `realize`,
  joining the same verb; a noise block inside `sys` and on a `disturbances` port realized per
  trial. Today `evaluate` holds the `disturbances` ports at their nominal values and warns
  (`warn_if_disturbed` in `planning/evaluator.py`).
- **Step 7b, the sampled face** (the hybrid phase, after S31's kernel): `compile` dispatches a
  sampled controller (a digital PID, a held neural policy, MPPI, the RL policies) to the
  kernel's JAX face; the refusal goes (today `require_continuous` in `planning/evaluator.py`
  refuses a `StepSystem` controller); a continuous law's numbers unchanged.
- **Step 8, sets as costs, the lowerings, the metrics as constraints** (v0.4). `shaping.py`
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
