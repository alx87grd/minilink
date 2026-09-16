# Core objects, phase 3: the state set is a hard constraint, in every tool

Status: applied 2026-09-15 (uncommitted) on the maintainer's rulings: `X` unconstrained unless
declared, `U` the input ports' box (an actuator limit is part of the plant), `on_exit` retired,
`exit_cost` renamed `infeasible_cost`, and the RL planner's `training_zone` (the plant's state
box by default) separated from the constraint. Verification: see the end of this file.
Finding F1: docs/reviews/2026-09-15-foundations-review.md.

## The design as landed

- **`X` is a hard constraint, unconstrained by default.** `PlanningProblem(plant, ...)` constrains
  nothing you did not declare; `X=plant.state.box` constrains the plant to its declared range,
  `X=plant.state.box & free` adds obstacles. `U` still defaults to the input ports' box.
- **An exit is a failure with infinite cost, everywhere.** Trajectory optimization and RRT keep
  `X` exact; dynamic programming, policy evaluation and the tabular learner charge
  `out_of_bound_cost` (1e6, or the problem's `infeasible_cost`) and treat leaving the *grid* as
  infeasible too (the table has no value off its domain); the RL environment, the Monte Carlo
  score and the Gymnasium problem view charge `infeasible_cost` — declared on the problem
  (scalar or `infeasible_cost(x, t)`), else a bound of any feasible cost that the environment
  derives from the running cost over the zone and the action range and announces once
  (`+inf` when nothing bounds it). `Evaluation.failure_rate` is the constraint's verdict;
  `score_trajectory(..., infeasible_cost=)` charges the evaluator's bound, `+inf` by default.
- **The training zone is a training choice.** `ReinforcementLearningPlanner(training_zone=)` and
  `RolloutEnvironment(training_zone=)`: leaving it truncates the episode (the critic's value
  stands in), because the model is not studied there. Default: the plant's state box, which is
  exactly what the unpriced demos did before, so their training is unchanged. The tabular
  learner's zone and constraint are the grid.
- **`on_exit` is gone.** Both values meant failure; "terminate without cost", the Gym default,
  rewards escaping under a cost.

## What changed where

Library: `planning/problems.py` (fields, `unconstrained(n)`, `infeasible_penalty`),
`reinforcement_learning/environment.py` (`training_zone`, `X.margin` exit test,
`price_of_infeasibility`, `feasible_cost_bound`), `reinforcement_learning/planner.py`
(`training_zone` through), `reinforcement_learning/tabular.py` (grid as zone and constraint),
`planning/evaluation.py`, `interfaces/gymnasium.py` (`ProblemEnv` follows `X`),
`policy_synthesis/{dp,policy_eval,discretizer}.py` (the grid owns its domain; extent falls back
to the plant's box), `search/rrt.py` (sample box finite, else the plant's), `core/sets.py`
(`is_finite_box`). Examples: 21 files whose problems meant the state box as a constraint now
say `X=plant.state.box` (trajopt and MPC demos and projects, README, tutorial 09, the showcase,
the cart-pole LQR notebook, the priced RL demos and the VI-to-PPO notebook); `exit_cost` →
`infeasible_cost` and `on_exit` dropped wherever they appeared; tutorial 11's exit-rule
paragraph rewritten. Docs: DESIGN §6, ROADMAP §2 deprecation note, `rl-planner-vision.md` §4.

## What `X` means today, tool by tool

| Tool | On leaving `X` | Meaning |
| --- | --- | --- |
| Trajectory optimization | `margin ≥ 0` as NLP inequalities; an exit is infeasible | hard |
| RRT | a state outside `X` is rejected | hard |
| Dynamic programming | the transition is charged `out_of_bound_cost` (1e6), the cell joins the infeasible set | hard, `∞` written as a big number |
| RL environment, priced (`exit_cost` set or `on_exit="terminate"`) | episode terminates, price charged, no bootstrap | hard, `∞` written as a finite price |
| RL environment, unpriced (**the default**: `on_exit="infeasible"`, no price) | episode *truncated*, the critic's value at the exit state bootstraps the return | not hard: leaving costs nothing, and with costs `g ≥ 0` it *stops* the cost — an escape the policy can learn (tutorial 11 says so) |
| Monte Carlo evaluation | trial cut at the first exit, `failed = True`; the price added when priced, else the cost so far | hard as a statistic (`failure_rate`), soft in `J` when unpriced |
| `Sys2Gym` (the Gymnasium bridge) | truncated on the state bounds, no price; optional `clipping_states` | the classic Gym convention |
| `ProblemEnv` (`Sys2Gym.from_problem`) | follows the problem: terminated + price when priced, truncated otherwise | as the RL environment |
| Region of attraction | a sample outside the domain blocks the level | hard (the certificate never reaches outside `X`) |
| Simulator | nothing: bounds are information | not a constraint |

The Gym default, "terminate without cost", is a reward-sign artifact: with positive per-step
rewards an early end is a punishment; with a cost formulation (`reward = -g dt`, `g ≥ 0`) an
early end without a price is a reward for escaping, and bootstrapping does not repair it (the
critic's value outside `X` is fiction). So the one setting that today applies by default to a
hard constraint is the one that does not enforce it. Every shipped RL demo sets
`on_exit="terminate"` and an explicit `exit_cost`, which is the evidence.

Three events are entangled in "the episode ends":

1. leaving `X` — a **failure**; by definition the cost is `+∞`;
2. reaching `Xf` — **success**, the task ends (RL and Monte Carlo do not use `Xf` yet; `TimeCost`'s
   absorbing `eps` ball is this event in disguise);
3. the clock — a finite horizon `tf` (terminated, `h` charged) or an infinite-horizon episode length
   (truncated with bootstrap: correct, it is not a set event). A non-finite state is event 1.

## Evidence (2026-09-15): what the hard definition would do to the RL demos

Each unpriced demo trained twice with the same seed, as today (an exit truncates and
bootstraps) and with the derived price `M` (an exit terminates and charges `M`); both policies
then scored on the same problem, 50 starts, 10 s episodes. The learned weights differed in every
demo, so exploration leaves the box in all of them.

| Demo | box | today: mean `J`, failures | priced: mean `J`, failures | verdict |
| --- | --- | --- | --- | --- |
| `pendulum_swing_up_rl` (PPO) | angle ±4π, rate ±20 | 3.43, 0 % | 5.89, 0 % | worse |
| `pendulum_ppo_vs_sac_rl` PPO / SAC | same | 3.43 / 3.33, 0 % | 5.89 / 3.80, 0 % | worse |
| `pendulum_reinforce_vs_actor_critic_vs_ppo_rl` | angle ±1e3, rate ±20 | 1.29, 0 % | 3.33, 0 % | worse |
| `drone_learn_to_fly_rl` | ±10 m, ±2π, ±10 | 93.4, 20 % | 87.7, 16 % | same |
| `cartpole_swing_up_rl` | track ±5 m, rates ±20 / ±30 | 2.06, **100 %** | 1.30, 0 % | fixed |

Two boxes, two roles. The pendulum's box is a *domain* (the angle window of the grid and the
normalisation): leaving it is not a failure, and pricing it teaches timidity. The cart-pole's
box is a *constraint* (the track): today's demo learns to leave it in every evaluation trial,
and its lower `J` is only the cost of a trial cut short. The drone's box is a constraint whose
exits are real failures either way. One field, `X`, plays both roles today, and each learner's
exit rule suits one of them.

## Proposal

**P1 — One meaning for every result.** `X` is a hard constraint: an exit is a failure (boolean
results say `False`, `Evaluation.failure_rate` counts it) with infinite cost (a number larger
than any feasible cost where a number is needed: `exit_cost`, else the derived bound of P2).
This governs every *report*: trajopt and RRT feasibility, the DP infeasible set, the Monte
Carlo score and `PlanningSolution.evaluation`. `on_exit` loses its second value (both mean
failure); keep the keyword this term, remove at v0.2 (**R1**).

**P1b — Training keeps its two rules, named, defaults unchanged.** Infinity is not learnable, and
the evidence shows the right approximation depends on what the box is: a *constraint* is
priced (terminate, charge, no bootstrap); a *domain* is truncated (bootstrap: the model is no
longer valid there, nothing is charged). Today's rule — price when a price is declared,
truncate otherwise — stays the default, so every current demo trains byte-identically; the
documentation names the truncation as the domain relaxation and the evaluation reports exits
as failures regardless. The one incoherent combination, `on_exit="terminate"` with no price
("terminate without cost", the Gym default that rewards escape under a cost), charges the
derived bound instead. Consequence for the demos (student-facing, **R5**): the cart-pole demo
should declare a price (its policy exits 100 %); the pendulum demos may state that the angle is
a domain (`X` unbounded in the angle, or the documented relaxation); the drone should price its
crash.

**P2 — Each tool represents `∞` as large as it tolerates.** Exact in the NLP and the search; a big
number on the grid (DP keeps 1e6 this term: nothing can exploit it, only the plots suffer — S42);
a finite price `M` in the sampling methods (RL, Monte Carlo, `ProblemEnv`), where a huge `M` makes
learning timid and a small one makes escape attractive. The penalty method says what `M` must be:
at least the bound of any feasible cost, so no feasible policy prefers leaving. **Default `M`,
derived and announced** (RULES 4.12): sample `g` over `X.bounding_box() × U.bounding_box()`
(a Halton set, as the Lyapunov window does) for `g_max`, then
`M = g_max · dt · (1 − γ^N) / (1 − γ)` over the effective horizon (`γ = exp(−ρ dt)`, `N` steps of
`tf` or of the episode length) — the discounted sum of the worst running cost. Warn once in domain
units: *"leaving X is priced at 812 (the bound of any feasible cost over 10 s); set exit_cost to
choose"*. An explicit `exit_cost` (scalar or shaped `exit_cost(x, t)`) overrides it. When `X` has
no finite bounding box the bound does not exist and the sampling methods require an explicit
price (a clear error) (**R2**).

**P3 — The evaluation reports the constraint as a constraint.** `Evaluation.failure_rate` is the
hard-constraint verdict; `J` includes the price so training and evaluation agree; `mean` is the
expected cost *with the price*, and the docstring says so. `score_trajectory` takes the price
from the same rule (an `exit_cost=` argument filled by the evaluator), never "the cost so far".

**P4 — The Gymnasium bridge keeps Gym's conventions** (`Sys2Gym` truncates on bounds, `clipping_*`
options): it is the view of the domain's standard, taught as such. `ProblemEnv` follows the
problem: always priced under P2.

**P5 — Later: `Xf` as the success event** for RL and Monte Carlo (reach-avoid tasks: `X` hard, `Xf`
terminal with `h`, no bootstrap). Not now; recorded (**R3**: schedule or drop).

**P6 — DP's price.** Keep 1e6 this term (the VI notebooks' tables and plots depend on it); unify
with the P2 bound at v0.2 alongside S42 (**R4**).

## Steps (after the rulings)

- [x] **3.1 The exit test uses the set.** `RolloutEnvironment.step`, `MonteCarloEvaluator.evaluate_jax`,
  `ProblemEnv.step`: `outside = ~xp.all(X.margin(x_next, t_next) >= 0) | ~finite`. Test: a double
  integrator with a forbidden disc in `X` — a step into the disc is a failure; `"jax"` and
  `"numpy"` backends report the same failure rate over 20 seeds; the box-problem baseline
  byte-identical.
- [x] **3.2 One meaning (P1, P1b).** `PlanningProblem` docstring and `describe()`: both `on_exit`
  values are failure; training prices a declared price and truncates otherwise (named as the
  domain relaxation); `on_exit="terminate"` without a price charges the derived bound.
- [x] **3.3 The derived price (P2).** `RolloutEnvironment.exit_price` (explicit, else the bound,
  announced once); `MonteCarloEvaluator` and `ProblemEnv` read it; `score_trajectory(problem,
  traj, params=None, exit_cost=None)`. Tests: the bound exceeds the cost of any feasible
  trajectory on the pendulum problem; explicit `exit_cost` unchanged; infinite box raises with a
  clear message; the pendulum swing-up demo (no explicit price) still learns.
- [x] **3.4 RRT samples the problem's box** (from phase 2): test that proposals lie in
  `X.bounding_box()` when `X` is narrower than the system bounds.
- [x] **3.5 Docs.** DESIGN §6 exit paragraph rewritten to P1–P4; `MonteCarloEvaluator` docstring;
  tutorial 11's exit-rule paragraph **[ask — student-facing]** (it currently teaches the
  bootstrap default); ROADMAP §6 terminal-cost item gains the note that the rocket's `h = 100`
  "crash" is an exit price and should become `exit_cost`.

## Byte-identity

3.1 is byte-identical on box problems. Training is byte-identical in every shipped demo (P1b keeps
the default rule). Only Monte Carlo reports change for problems with no declared price: a failed
trial is charged the derived bound instead of the cost so far.

## Verification (2026-09-15)

- Every RL and tabular demo's learned weights and tables captured before and after: byte-identical
  for eight of nine (`pendulum_swing_up_rl`, `pendulum_ppo_vs_sac_rl` PPO and SAC,
  `pendulum_reinforce_vs_actor_critic_vs_ppo_rl`, `drone_learn_to_fly_rl`,
  `pendulum_q_learning_vs_value_iteration_rl`, `rocket_landing_rl`, `car_circuit_rl`). The
  cart-pole differs: its exploration reaches non-finite states, which used to truncate with
  bootstrap and now fail (charged, terminated). Its policy scores mean `J` 7.2 over full 10 s
  episodes against 26 000 for the old one when it is not allowed to escape, and leaves the box in
  20 % of trials instead of 100 %; declaring the track as `X=plant.state.box` with a price trains a
  policy at 1.3 and 0 % (maintainer's call on the demo).
- Full `pytest` green (1180 passed, 2 skipped); ruff clean; demo sweep 70 passed, 0 failed, 3
  skipped (interactive); notebook smoke 26 / 27 (the Gymnasium notebook's `stable_baselines3`
  import abort, pre-existing); regression gates 5 / 5 suites after the benchmark scenarios
  declared `X=plant.state.box` like the demos (their baselines were recorded with the bounds).
