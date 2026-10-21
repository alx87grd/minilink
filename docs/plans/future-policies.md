# Projecting the API onto MPPI, behavior cloning and diffusion policy

**Status:** ideas to review, 2026-10-21 — an independent analysis; nothing decided.  
**Lane:** maintainer-owned: it shapes core decisions still open.  
**Serves:** the sampled loop (S31, [sampled-path.md](sampled-path.md) §8), the names of the
differentiable path ([differentiable-problem.md](differentiable-problem.md), step 0 and ask 5),
identification (C4), the MPPI planner ([mppi.md](mppi.md)), and the behavior-cloning row
after v1.0 (ROADMAP §5.6).

The question: do the proposals on the table (the step model, the sampled loop as a tool, the
compiled problem with `tune` and the program bridge, the optimizer's program classes, planners
returning a `PlanningSolution`) carry the features that come later, or will those features
force a redesign? Three are projected here: MPPI (planned), behavior cloning (planned after
v1.0) and diffusion policy (not planned).

## 1. The verdict

The proposals carry MPPI well, diffusion policy reasonably, and behavior cloning only half way.
The core nouns hold for all three: the system, the problem, the trajectory, the program and the
optimizer. The gaps sit in three places:

- no verb to fit parameters to **recorded data**;
- no **controller memory or random key** threaded through the sampled loop;
- no shared **stochastic-policy** contract outside the RL package.

## 2. MPPI

- **It fits the planner contract.** A planner on the existing planning problem: it rolls out
  many held-input control sequences on the step model, scores them with the one scoring
  kernel, and returns the usual solution record with an open-loop plan.
- **Its batch is already expressible.** The differentiable-problem proposal treats an open-loop
  plan as a policy whose table of values lives in the parameters, and the start state as a
  parameter. MPPI's inner loop is "the batched cost of many tables from one state", with no
  new machinery.
- **Two strains.** The receding-horizon wrapper still reads trajectory-optimization internals,
  so it needs the small planner interface [mppi.md](mppi.md) already asks for. And each tick
  carries memory (the shifted warm-start sequence) and a fresh random key; the sampled loop
  must thread both.
- **A payoff.** MPPI's update is a weighted average, so it is differentiable end to end: its
  temperature and noise level can be tuned by gradient with the same `tune` as a PID gain.
  That works only because tracing is decided by the controller, not blocked by the loop.

## 3. Behavior cloning

**Deployment fits.**
- The cloned policy is a neural controller, which is a `System`: it drops into diagrams, the
  sampled loop, `compare()` and animation.
- The recorder is the realtime game loop. Recorded at the step model's sample rate with the same
  input hold, demonstrations and deployment see the same model: a strong argument for the step
  model.

**Training does not fit.**
- The differentiable path is built around a rollout cost: simulate the closed loop, integrate
  the cost. Behavior cloning's loss is supervised: on recorded states, how far is the policy's
  action from the demonstrator's? No rollout is involved.
- The roadmap's "identification is `tune` with a data-mismatch cost" has the same blind spot:
  equation-error identification is also a one-step loss.
- So the framework needs **a second training verb** beside `tune`: fit these parameters to this
  dataset with this loss, on the same program bridge and optimizer, minibatches drawn through
  the random key the stochastic program class already plans for.
- It needs **a dataset noun**: a batch of trajectories with padding masks, which the native
  `Trajectory` and the scoring kernel's alive mask already almost provide.
- Real demonstrations observe outputs, not the full state, so **the recorder logs `y`**; the
  identification row already asks for it.

**A bonus.** With the sampled kernel recording internal signals, distilling MPC or MPPI into a
neural policy is behavior cloning with a library controller as the teacher: a better first
demo than human driving, and free of hardware.

## 4. Diffusion policy

- **Run time is the MPPI shape again.** A diffusion policy observes a short history, samples a
  chunk of future actions by iterative denoising, executes a few and replans: a sampled
  controller with memory (the observation history, the current chunk), a random key and a
  slower replanning rate. It cannot be a continuous `System`, which is fine under the "loop as
  a tool" recommendation; the MPPI requirements on the loop cover it.
- **Training reuses the fitting verb**, with a denoising loss instead of an action loss.
- **The real risk is scope.** Diffusion needs conditioned noise-prediction networks and noise
  schedules; minilink should not grow a deep-learning library. Its one MLP is right for
  teaching. The line to hold: a policy network is any pure function with a parameter tree,
  wrapped as a `System`; minilink owns the loop, the data path, the training bridge and the
  evaluation; an external JAX network library owns the architecture. JAX-only and research
  lane, like the RL planner.
- **A convergence.** An open-loop action table is the common output of trajectory
  optimization, MPC, MPPI and diffusion planning: one more reason to make the open-loop plan a
  first-class policy.

## 5. What the proposals should absorb

1. **Three training verbs on one bridge.** `solve` for planners, `tune` for parameters through
   a closed-loop rollout, `fit` for parameters against a dataset; all three produce a
   `MathematicalProgram` for the same `Optimizer`. The plans have the first two.
   ```python
   solution = MPPIPlanner(problem, n_samples=512).solve()          # a planner
   tuned    = tune(problem, controller, wrt=[...], dt=0.05)        # a rollout cost
   cloned   = fit(policy, demos, wrt="policy", loss="action")      # a dataset loss
   ```
2. **The sampled loop threads controller memory and a random key** on every tick, the key
   derived from the experiment's key and the tick count, as [randomness.md](randomness.md)
   already rules. MPPI, diffusion policy and any stochastic deployed policy depend on it.
3. **The evaluator dispatches on the policy kind.** A continuous law runs the continuous
   closed loop, as ruled (RD-D24); a sampled controller runs the sampled kernel's JAX face, so
   MPPI and diffusion policies get the same batched Monte Carlo evaluation and tuning. The plan
   today says a sampled loop falls back to NumPy "until the hybrid seam batches"; with the
   kernel, that fallback goes.
4. **One small stochastic-policy contract outside the RL package**: a policy whose output is a
   sample given a key and a state, with the mean as its deterministic face; a log-probability
   optional (RL and likelihood-based cloning need one, a diffusion policy has none). Sized like
   the constitution's two-method distributions, not a probability library. Today it lives in
   the RL package as an exploration head.
5. **A dataset noun**: batched, masked trajectories with outputs logged. The recorder,
   simulations and logs produce it; `fit`, identification and evaluation read it.
6. **Architectures stay out of the core.** The small MLP stays for teaching; larger networks
   come through a thin wrapper, which TODO's Later ideas already list.

## 6. Timing

- **Points 2 and 3 are cheap now and expensive later**: they shape the sampled loop and the
  evaluator, both designed this November.
- **Points 1 and 5 shape the names**: they belong in the names brainstorm of the
  differentiable path, so `tune` is not named without its sibling `fit`.
- **Point 4 can wait for the RL follow-ups**, as long as nothing new hard-codes the RL-only
  head meanwhile.
- **Behavior cloning stays after v1.0, and diffusion policy after it.** With points 1 to 5 in
  place, both are additive features rather than redesigns.

## 7. Questions for the maintainer (for discussion, not blocking)

1. Does `fit` join `solve` and `tune` as the third verb, and under that name?
2. Does the sampled loop's tick contract carry controller memory and a random key now?
3. Is diffusion policy in scope at all, and on the research lane only?
4. Is distilling MPC or MPPI into a neural policy the first imitation demo?
