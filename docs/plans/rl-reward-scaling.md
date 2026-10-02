# Automatic reward scaling for the RL planner

**Status:** Proposal (2026-09-27). Evidence gathered; not scheduled.
**Lane:** teaching (a keyword on `ReinforcementLearningPlanner`) — **[ask — planner API]**.
**Found on:** the GRO860 Gymnasium lab (`examples/teaching/courses/udes_gro860/particle_gymnasium_sb3.ipynb`), where the native planner trained a worse policy than Stable-Baselines3 on the same task.

---

## 1. The problem

`ReinforcementLearningPlanner` trains on the raw reward `r = -g(x, u, t) dt`. When the running cost is large, PPO learns slowly and unreliably, although its advantages are normalized. The user gets no warning: training runs, the learning curve rises, and the law is just mediocre.

Nothing in the problem is wrong. Scaling the cost by a positive constant leaves the optimal law unchanged, and with that one change the native planner reaches LQR-level laws on every seed tested. The fix belongs in the planner, not in each notebook's cost weights.

## 2. Evidence

**Task.** A 2D particle in a viscous medium: `m = 10`, `b = 5`, `|F| <= 100` per axis, a 100 x 100 plane, target at the center; `g = (x - xbar)' Q (x - xbar) + u' R u` with `Q = diag(1, 1, 0.1, 0.1)` and `R = 0.01 I`. The time step is `dt = 0.1`, episodes last 20 s, and leaving the plane is a failure priced at `g_max T`. At the starts, `g dt` is of order 100 per step.

**Yardstick.** Every policy is scored on the same 100 fixed starts: Euler integration, `J = sum g dt`, plus the failure price. On it, LQR scores 3162, a hand-tuned PD 3172, and `u = 0` 31320.

| PPO run (3 seeds) | Reward | J per seed | Mean |
| --- | --- | --- | --- |
| SB3, 100k steps | raw | 4714 / 7978 / 5888 | 6190 |
| SB3, 100k steps | x 1e-3 | 3179 / 3262 / 3180 | 3207 |
| minilink, SB3's settings (1 env, `n_steps=2048`, `batch_size=64`), 100k | raw | 5546 / 6171 / 5741 | 5819 |
| minilink, SB3's settings, 100k | x 1e-3 | 3201 / 3233 / 3247 | 3227 |
| minilink defaults (16 envs), 500k | raw | 4756 / 4811 / 3458 | 4342 |
| minilink defaults, 500k | x 1e-3 | 3168 / 3192 / 3180 | 3180 |
| minilink defaults, 500k | x 1 / (g_max dt) = 1.88e-3 | 3171 / 3182 / 3190 | 3181 |

Reading the table:

- The two implementations agree once the settings and the reward scale match.
- The reward scale alone moves J from about 6000 to about 3200 in both libraries.
- A scale computed from the problem, `1 / (g_max dt)`, does as well as the hand-picked one.
- With matched settings, minilink ran 100k steps in about 3.5 s, against about 24 s for SB3 on the same CPU.

## 3. Mechanism

Both PPOs normalize the advantages, so the policy gradient does not see the reward scale. The value loss does: the critic regresses returns of order 10^3 to 10^4.

Both optimizers clip the gradient to a global norm of 0.5, taken over the policy and the critic together (`optim.clip_by_global_norm` on the whole parameter tree). The critic's gradient therefore sets the clip factor, and the policy's share is shrunk with it.

Measured on one minibatch of 64, as the median over 20 minibatches, with the default planner:

| Reward | Steps | Policy gradient norm | Critic gradient norm | Clip factor | Policy per-weight gradient after the clip |
| --- | --- | --- | --- | --- | --- |
| raw | 0 | 0.40 | 17800 | 2.8e-5 | 1.7e-7 |
| raw | 200k | 0.45 | 3610 | 1.4e-4 | 9.1e-7 |
| raw | 500k | 0.63 | 1580 | 3.2e-4 | 2.9e-6 |
| x 1e-3 | 0 | 0.40 | 17.9 | 0.028 | 1.7e-4 |
| x 1e-3 | 200k | 4.6 | 0.083 | 0.11 | 7.4e-3 |

With the raw reward, the policy's clipped gradient sits one to two orders of magnitude below Adam's `eps = 1e-5`. Adam's step `m / (sqrt(v) + eps)` then shrinks with it, so the policy moves at a few percent of its intended rate until the critic's error comes down. Scaled, the policy gradient stays well above `eps` and Adam restores full-size steps.

## 4. What does not fix it

**Removing the clip** (`max_grad_norm=None`, raw reward, defaults, 500k steps) gives J = 8071 / 3339 / 3443. Two seeds improve. The first gets worse, with 3 failures in 100 trials: an unscaled critic is also a noisy baseline. The clip is doing its job, and the scale is what needs fixing.

**Tuning the notebook's `Q` and `R`** works, but every author has to rediscover it. It also mixes two decisions: what the cost *means* and how the learner is *conditioned*.

## 5. Options

| | What | For | Against |
| --- | --- | --- | --- |
| A | **Fixed scale `1 / (g_max dt)`** from the problem, computed once | Deterministic and stateless; compiles as a constant. `feasible_cost_bound` already samples `g_max` on the training box for the failure price, so one pass serves both. The optimum is unchanged. | Needs a finite box (the same condition as the failure price). A pessimistic `g_max` makes rewards small, but the table shows that is harmless here. |
| B | **Running return normalization** (SB3's `VecNormalize(norm_reward=True)`): divide rewards by a running std of the discounted return | Works without a box | State in the train state, carried through `scan`. The critic's targets drift during training. The learning curve needs unscaling. |
| C | **Value-target normalization** (PopArt): normalize the critic's targets and rescale its output layer | Treats the actual cause (the critic's scale); leaves the reward's meaning intact | Most code; an algorithm-level change in every actor-critic family |
| D | **Clip the policy and the critic separately** | Removes the throttling directly | §4 shows an unscaled critic is unstable anyway; not enough alone |

**Related but different: potential-based shaping** (Ng, Harada and Russell, 1999). This adds `F = gamma Phi(x') - Phi(x)` to the reward: it changes the learning signal and preserves the optimal policies. A natural `Phi` is a known cost-to-go (LQR's `x' S x`, or a DP table), which would tie RL to the course's earlier chapters. It is a research-lane follow-up, not what this finding needs.

## 6. Proposal

Option A, as a planner keyword:

```python
ReinforcementLearningPlanner(problem, ..., reward_scale="auto")  # or a float, or 1.0 to opt out
```

- `"auto"` sets `c = 1 / (g_max dt)` from the same sampling as the failure price. The algorithm sees `c r` and `c` times the failure price.
- Everything reported to the user stays in cost units: the learning curve, "mean episode cost", the solver record and `MonteCarloEvaluator`. The solver record also carries `c`.
- With no finite box, `"auto"` falls back to `c = 1` and prints one line saying so. Option B can come later if a consumer needs it.
- Whether `"auto"` is the default is the maintainer's call. It moves every RL demo's numbers once, so it lands between cohorts, like RN-4 and RN-5 of [randomness.md](randomness.md).

## 7. Steps

- **RS-1** **[ask — planner API]** The `reward_scale` keyword and its default. Done when:
  - the particle task reproduces the last row of §2's table;
  - a test shows that scaling the cost leaves the greedy law of a converged tabular or LQR-like case unchanged;
  - the learning-curve units are unchanged.
- **RS-2** A small benchmark: particle, pendulum swing-up, planar drone hover; 3 seeds each, raw vs `"auto"`, scored with one evaluator per task. Record it in `docs/reviews/`. It shows whether the hyperparameters tuned in the existing demos were compensating for the scale (the pendulum notebook's `learning_rate=3e-3` and `n_envs=64`, for example).
- **RS-3** **[ask — student-facing]** Re-baseline the RL notebooks and `examples/demos/rl/` once the default lands, between cohorts.
- **RS-4** Option B, only when a task without a finite box needs it.

## 8. A related observation (not this feature)

In the same lab, `MonteCarloEvaluator` scored the LQR law at 4485, against 3324 on the same starts in the yardstick above.

The plant saturated the force inside `f` (`u.clip(...)`), while the evaluator charged `R u^2` on the controller's command, up to 499 N at the start. Charging the command in the yardstick gives 4715; the rest is trapezoid vs rectangle integration and RK4 vs Euler.

This is consistent with the rule that port bounds are information, not saturation. The consequence: a demo comparing an unsaturated law (LQR) with a clipped one (the neural policy) must saturate in the controller or with a saturation block, not in `f`, or the comparison is unfair.

## 9. Reproducing

The runs used `git archive origin/main` of 2026-09-27 in a clean venv (gymnasium, stable-baselines3, jax[cpu]).

- **Scaled cost.** `QuadraticCost.from_system(plant, xbar=..., Q=c * Q, R=c * R)` with `infeasible_cost=c * PENALTY`.
- **Gradient norms.** From `algorithm.loss` on minibatches built exactly as `PPO.update` builds them (`collect.advantages`, `collect.flatten`).
- **The scripts** lived in a session scratchpad and are not kept. They are about 150 lines and easy to rebuild from this description.
