# Projecting the API onto MPPI, behavior cloning and diffusion policy

**Status:** ideas to review, 2026-10-21 — an independent analysis. Aligned 2026-10-23 to the
rulings of 2026-10-22/23 ([the record](../reviews/2026-10-22-rulings-session-1.md)): the compiled problem
exports a `MathematicalProgram` and no `tune` facade exists (R23); the sampled loop is a tool on
ticks (R13, R14), its memory and dispatch parked to the hybrid phase (D13–D15); point 4 is D29. Points 2–4 of §5 now stand as workboard rows (S31, V1's step 7b, C5) or open
decisions (D14, D15, D29); §7's questions 1, 3, 4 and 5 are open, maintainer-owned.  
**Lane:** maintainer-owned: it shapes core decisions still open.  
**Where it sits:** points 2–4 of §5 are v0.3's core: the hybrid loop and the seeded policy
([core-architecture.md](core-architecture.md) §4, §5; [TODO.md](TODO.md) §1.4, §1.5). Behavior
cloning and diffusion policy stay after v1.0.  
**Serves:** the sampled loop (S31, [sampled-path.md](sampled-path.md) §8), the names of the
differentiable path (landed: DESIGN §6; the open steps in [differentiable-problem.md](differentiable-problem.md)),
identification (C4), the MPPI planner ([mppi.md](mppi.md)), and the behavior-cloning row
after v1.0 (ROADMAP §5.6).

The question: do the proposals on the table (the step model, the sampled loop as a tool, the
compiled problem and its program export, the optimizer's program classes (S74, landed in v0.3 on 2026-10-24, R74), planners
returning a `PlanningSolution`) carry the features that come later, or will those features
force a redesign? Three are projected here: MPPI (planned), behavior cloning (planned after
v1.0) and diffusion policy (not planned).

## 1. The verdict

The proposals carry MPPI well, diffusion policy reasonably, and behavior cloning only half way.
The core nouns hold for all three: the system, the problem, the trajectory, the program and the
optimizer. The gaps sit in three places:

- no **controller memory** threaded through the sampled loop (the random draws need no key
  threading: a seeded controller draws from its seed and the tick, as the noise block does);
- no shared **stochastic policy** outside the RL package (§5 point 4: a seeded block, the
  maintainer's direction);
- no verb to fit parameters to **recorded data** (not critical to the core tools, the
  maintainer judged on 2026-10-22; a note for behavior cloning).

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
  carries memory (the shifted warm-start sequence), which the sampled loop must thread; its
  fresh samples come from its own seed and the tick, as the noise block's do (§5 point 2).
- **A payoff.** MPPI's update is a weighted average, so it is differentiable end to end: its
  temperature and noise level can be optimized by gradient through the same program export
  (`pe.program(wrt=[...])` solved by `Optimizer`) as a PID gain.
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
- The roadmap's "identification is the program export with a data-mismatch cost" has the same
  blind spot:
  equation-error identification is also a one-step loss.
- So the framework needs **a second training verb** beside the program export: fit these parameters to this
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
  controller with memory (the observation history, the current chunk), seeded draws and a
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

1. **Three ways to train on one bridge.** `solve` for planners, the compiled problem's program
   export for parameters through a closed-loop rollout (ruled 2026-10-23: no `tune` facade in
   this phase), `fit` for parameters against a dataset (the open idea); all three produce a
   `MathematicalProgram` for the same `Optimizer`. The plans have the first two.
   ```python
   solution = MPPIPlanner(problem, n_samples=512).solve()                     # a planner
   pe = problem.compile(controller, dt=0.05)                                  # a rollout cost
   params = pe.unpack(Optimizer(pe.program(wrt=[...])).solve().z, wrt=[...])
   cloned   = fit(policy, demos, wrt="policy", loss="action")                 # a dataset loss (open)
   ```
2. **The sampled loop passes the tick and threads controller memory** (MPPI's warm-start
   sequence, a diffusion policy's observation history and action chunk). It threads no random
   key: a controller's draws come from its own seed and the tick, the noise block's
   counter-based pattern ([randomness.md](randomness.md)), as point 4 describes.
3. **The evaluator dispatches on the policy kind.** A continuous law runs the continuous
   closed loop, as ruled (RD-D24); a sampled controller runs the sampled kernel's JAX face, so
   MPPI and diffusion policies get the same batched `evaluate` and program export. Today the
   evaluator refuses a sampled controller (`require_continuous`, `planning/evaluator.py`)
   until the sampled face joins it (V1's step 7b); with the kernel, that refusal goes.
4. **One standard stochastic policy outside the RL package, built like the noise block**
   (the maintainer's direction, 2026-10-22). The white-noise block keeps its `seed` in
   `params`, draws `ε_k = F(seed, ⌊t/Δ⌋)` with a counter-based Threefry generator identical on
   NumPy and JAX, holds each sample over its period, returns the mean when `seed=None`, and
   gets a seed per experiment from `realize(key)`. A stochastic policy built the same way,
   `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)` with Δ the control period:
   - is a plain `System` with a pure `h`: diagrams, the sampled loop, `compare()`, the
     evaluator and `vmap` over seeds, on both backends;
   - has its deterministic face for free: `seed=None` gives the mean, the law one deploys or
     plots;
   - is reproducible by one key, with common random numbers across compared laws;
   - holds its draw over the control period, so the continuous evaluator's RK4 stages see one
     action per period, the rule the noise block already states;
   - gives pathwise gradients through the fixed `ε` (the reparameterization), so the program
     export and
     SAC-style updates reach `θ`.

   What the block form does not give, the contract adds as methods beside `h`, not outputs:
   the log-probability of an action and the entropy (PPO and likelihood-based cloning need
   them; a diffusion policy has none), the squashed Gaussian with its change-of-variables
   term. The RL Gaussian and squashed-Gaussian heads become such blocks; moving the RL
   collectors from their own keys to per-episode seeds changes the RL numbers once (a baseline
   and a re-pin, between cohorts). Sized like the constitution's two-method distributions, not
   a probability library.
5. **A dataset noun**: batched, masked trajectories with outputs logged. The recorder,
   simulations and logs produce it; `fit`, identification and evaluation read it.
6. **Architectures stay out of the core.** The small MLP stays for teaching; larger networks
   come through a thin wrapper, which TODO's Later ideas already list.

### 5.1 D29, the standard stochastic policy as a seeded block (moved from end-state-api.md, 2026-10-26)

**Preliminary choice, 2026-10-27 (R129, [record](../reviews/2026-10-27-design-questions-rulings.md)):**
one block, `sample_period=` in place of `dt`, its noise internal and deterministic given the seed
(the `WhiteNoise` signal at `⌊t/Δ⌋`), `log_prob` and `entropy` on the block. The text below is
the recommendation as it stood.

Moved verbatim from end-state-api.md. The block is a v0.3 core pillar (TODO §1.4); whether the RL
heads' rebuild lands with it or in C5 (v0.4) is a placement detail of its row.

**Recommended:** `GaussianPolicy(mean, log_std, *, dt, seed=None)` in `control/neural.py`, a
plain `System` whose `h` is `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)`; `seed=None` is the
deterministic face (`NeuralPolicyController` becomes that face); `log_prob(x, u)` and
`entropy(x)` as methods beside `h`, the squashed variant a subclass. The RL heads are rebuilt
on it (with the block or in C5: open), the RL numbers changing once.

## 6. Timing

- **Points 2 and 3 are cheap now and expensive later**: they shape the sampled loop and the
  evaluator. Ruled 2026-10-22/23: the loop a tool on ticks (R13, R14); the tick's memory and the
  dispatch (D14, D15) parked to the hybrid phase, after the continuous core; point 3 is V1's step
  7b.
- **Point 4 is planned now**: the block is the core's (TODO §1.4); the RL heads' rebuild is
  with it or with the RL follow-ups (C5, TODO §4), open. It was a C5 row (v0.4) before 2026-10-26;
  nothing new hard-codes the RL-only head meanwhile.
- **Points 1 and 5 are not critical to the core tools** (the maintainer, 2026-10-22): the
  supervised pipeline, a `fit` verb and a dataset noun, stays a note for behavior cloning, not
  part of the names brainstorm.
- **Behavior cloning stays after v1.0, and diffusion policy after it.** With points 1 to 5 in
  place, both are additive features rather than redesigns.

## 7. Questions for the maintainer (for discussion, not blocking)

**Preliminary choices, 2026-10-27 (R136, [record](../reviews/2026-10-27-design-questions-rulings.md)):**
how fitting enters the API (question 1) is decided at the behavior-cloning row, identification
(C4) flagging its route for it; diffusion policy (question 3) is research-lane only, JAX-only,
after behavior cloning; distilling MPC or MPPI (question 4) stays a pointer.

1. Does `fit` join `solve` and the program export as the third way to train, and under that
   name?
2. Does the sampled loop's tick contract carry controller memory (the draws coming from the
   controller's seed and the tick, no key threaded)? Parked to the hybrid phase (D14).
3. Is diffusion policy in scope at all, and on the research lane only?
4. Is distilling MPC or MPPI into a neural policy the first imitation demo?
5. The standard stochastic policy as a seeded block (point 4) is the maintainer's direction;
   still to rule: its contract beyond `h` (the methods' names, which heads ship first);
   recommended as `GaussianPolicy` in D29 (§5.1); the maintainer named the seeded policy a core
   pillar of v0.3 on 2026-10-26 ([core-architecture.md](core-architecture.md) §4).
