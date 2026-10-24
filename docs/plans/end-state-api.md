# Open v0.4 decisions

**Status:** this ledger once held every decision that shaped a public name or a core contract
of the continuous phase. Those (D1–D12, D16–D24, D31) were ruled on 2026-10-22/23
([the rulings record](../reviews/2026-10-22-rulings-session-1.md), R1–R47, keeps how) and
landed on 2026-10-24 ([the run's log](../reviews/2026-10-24-v03-run-decisions.md); the contracts
are DESIGN §5 and §6; D23, the optimizer's method keys, landed with S74, OD-15 of the log). Their
entries are cut; git history keeps them, alternatives included. This doc now holds the
decisions still open that fix a v0.4 name or contract, each with the code as it stands and the
recommended answer: D13–D15, the sampled loop's details, parked to the hybrid phase and ruled
when the sampled loop (S31) starts; D25–D30, the parametric program's asks, MPPI's name,
contract and default, the seeded stochastic policy, and `RolloutEnvironment`'s home; and one
sentence proposed for CONSTITUTION §1. Deleted once these are ruled (one fact, one home).  
**Lane:** maintainer-owned (every item is an **[ask — core]** or **[ask — public names]**).  
**Rung:** v0.4: the sampled loop (S31) for D13–D15; the parametric program (S76), MPPI (S72)
and the seeded stochastic policy (C5) for D25–D30 (ROADMAP §6, "Before v0.4").

Every decision is written *as the code a user would write*, so each can be read and ruled on
its own. The ids are kept because ROADMAP §6, TODO, [sampled-path.md](sampled-path.md),
[mppi.md](mppi.md), [future-policies.md](future-policies.md) and
[parametric-program.md](parametric-program.md) cite them. The arguments and the evidence live
in those plan docs; a ruling taken here goes to the plan doc it shapes and to a dated record.

---

## 1. The sampled loop's details, parked to the hybrid phase (D13–D15)

The loop itself is ruled: a tool, not a `System`, the name `HybridDiagram` kept for now (R13),
on integer ticks (R14); [sampled-path.md](sampled-path.md) §8 designs it.

**Parked 2026-10-22 (the maintainer): D13, D14 and D15 below are ruled after the continuous
architecture and API are done and implemented; the hybrid tool's details wait for that phase.**

### D13 The run-state caches (S31; RULES 2.4)

**Today:** `StepSystem.rollout` and `HybridDiagram.last_result` cache a run beside the
allowed `self.traj`.

**Recommended:** they go. A tool returns its result; `loop.compute_trajectory()` returns the
`Trajectory` and stores it as `loop.traj` like every `System`, nothing else. The facades that
read `last_result` read `traj`.

### D14 Controller memory and the draws on the tick (S31; future-policies.md §5 points 2, 4)

**Today:** MPPI's shifted warm start and a diffusion policy's chunk have no channel; the RL
collectors thread their own keys.

**Recommended:** no new channel. A sampled controller with memory *is* a `StepSystem` with
state, and the kernel already carries the computer's `x` across ticks (`MPCStatefulController`
packs `z` there today). Its draws come from its own `params["seed"]` and the tick, the noise
block's counter-based pattern, so `vmap` over seeds and `realize(key)` work unchanged:

```python
class MPPIController(StepSystem):          # memory = the shifted input sequence, in x
    def step(self, x, u, k=0, params=None):
        eps = standard_normal(params["seed"], k, self.n_samples)   # the noise block's draw
        ...
```

### D15 The compiled problem dispatches on the policy's kind (S31; V1 step 7b)

**Recommended:** `problem.compile(policy)` reads the policy: a continuous law runs the loop as
a diagram; a `Computer` (anything built by `%`) runs the sampled kernel's JAX face. In v0.3 the
second branch refuses (`require_continuous` in `planning/evaluator.py` refuses a `StepSystem`
controller and names the hybrid loop); in v0.4 it lands. One `compile` and one `evaluate` for
both kinds.

---

## 2. The parametric program

### D25 S76's two remaining asks, for v0.4 (parametric-program.md §2)

**Recommended:** a hand-written program may declare its own `p` (`ParametricProgram(n_z, J, params=…)`
with `J(z, p)`), since it costs nothing and lets tutorial 08 solve one NLP for several
parameter values; the MPC block takes optional live-parameter ports, unwired ports at their
nominal:

```python
mpc = ModelPredictiveController(planner, dt_mpc=0.1, live=["plant.mu"])   # a port "mu" appears
loop = (estimator + mpc).autowire() @ plant                              # the estimate drives the plan
```

---

## 3. The v0.4 names the v0.3 code will already print

### D26 MPPI's name (mppi.md ask 1)

**Recommended:** `MPPIPlanner` and `MPPIRecord`, by the `LQRPlanner` precedent (an algorithm's
own acronym, as the course notes say it); `PathIntegralPlanner` stays the docstring's long
form.

### D27 The MPC block's planner contract (mppi.md ask 2; T6)

**Recommended:** the three `Planner` verbs (`decision_dimension()`, `prepare_online()`,
`warm_start_guess(z_prev, y, *, dt_mpc, k)`) and `z` on the tick's record; no `hasattr`
probing. `ModelPredictiveController(planner, dt_mpc=)` then wraps trajopt and MPPI alike.

### D28 The stochastic default of MPPI (mppi.md ask 3)

**Recommended:** `n_plant_samples=1` on a stochastic problem with the bias stated in the
docstring; the user raises it knowingly, as with `n_trials`.

### D29 The standard stochastic policy as a seeded block (future-policies.md §5 point 4)

**Recommended:** `GaussianPolicy(mean, log_std, *, dt, seed=None)` in `control/neural.py`, a
plain `System` whose `h` is `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)`; `seed=None` is the
deterministic face (`NeuralPolicyController` becomes that face); `log_prob(x, u)` and
`entropy(x)` as methods beside `h`, the squashed variant a subclass. The RL heads are rebuilt
on it in C5 (v0.4), the RL numbers changing once.

### D30 Where `RolloutEnvironment` lives (mppi.md ask 5)

**Recommended:** answered by D11 (the sampled loop a tool on one kernel, R13;
[sampled-path.md](sampled-path.md) §8): it becomes a caller of the sampled kernel and stays a
thin RL-facing adapter in `learning/reinforcement_learning/`; no move.

---

## 4. A sentence for CONSTITUTION §1 (maintainer-owned)

D1 ruled the shape on 2026-10-22 (R1, amended the same day): six tooling pillars on three
foundations, modelling, optimization and the problem statement. The sentence proposed then
for CONSTITUTION §1, still unplaced (the maintainer's to place or drop):

> One model and one differentiable path from it to a cost, solved by one optimizer; every
> approach that controls a vehicle or a robot is a tool on that path: classical and modern
> control, nonlinear control, trajectory optimization and MPC, reinforcement learning,
> identification and imitation, estimation.
